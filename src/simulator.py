import numpy as np
from tqdm import tqdm # Optional: for progress bar
import typing # Import typing for Type hint

# Use relative imports now, assuming src is a package
from .system import System
from .controller import Controller
from .plotter import Plotter

class Simulator:
    def __init__(self, system: System, controller: Controller, dt: float, num_steps: int):
        """
        Initializes the Simulation.

        Args:
            system: The system to simulate.
            controller: The controller to use.
            dt: Simulation time step.
            num_steps: Total number of simulation steps.
        """
        self.system = system
        self.controller = controller
        self.dt = dt
        self.num_steps = num_steps
        self.time_vector = np.linspace(0, dt * num_steps, num_steps + 1)

        # History storage
        initial_state = self.system.get_state()
        state_dim = initial_state.shape[0]
        self.state_history = np.zeros((num_steps + 1, state_dim))
        self.derivative_history = np.zeros((num_steps, state_dim)) # Added history for derivatives
        self.time_history = np.zeros(num_steps + 1) # Matches state history length

        # Determine control dimension dynamically
        try:
            # Assume controller.compute_control returns the full vector needed by system.step
            example_control = self.controller.compute_control(current_state=initial_state, t=0.0)
            control_dim = np.asarray(example_control).shape[0]
            if control_dim == 0: # Handle scalar case
                 control_dim = 1 
        except Exception as e:
            print(f"Warning: Could not determine control output dimension from controller. Error: {e}. Assuming 1.")
            control_dim = 1
        
        # control_history stores the full control vector passed to system.step at each step k
        self.control_history = np.zeros((num_steps, control_dim))

        # --- Added: History for controller's internal parameters (e.g., T_out_hat) --- 
        self.controller_param_history = None
        self.controller_param_names = [] # Names for the parameters
        # Check if the controller has a method to get trackable parameters
        if hasattr(self.controller, 'get_trackable_parameters') and callable(self.controller.get_trackable_parameters):
            try:
                param_dict = self.controller.get_trackable_parameters()
                if isinstance(param_dict, dict):
                    num_params = len(param_dict)
                    if num_params > 0:
                        self.controller_param_history = np.zeros((num_steps + 1, num_params))
                        self.controller_param_names = list(param_dict.keys())
                        print(f"Simulator: Tracking {num_params} controller parameters: {self.controller_param_names}")
                    else:
                        print("Simulator: Controller get_trackable_parameters returned empty dict, not tracking parameters.")
                else:
                    print("Warning: Controller get_trackable_parameters did not return a dictionary.")
            except Exception as e:
                print(f"Warning: Error calling or processing controller.get_trackable_parameters(): {e}")
        elif hasattr(self.controller, 'Q_heater_controller') and hasattr(self.controller.Q_heater_controller, 'get_estimated_T_out') and callable(self.controller.Q_heater_controller.get_estimated_T_out):
            # Fallback for GarageInputController wrapping AdaptiveController
            print("Simulator: Tracking T_out_hat from wrapped controller.")
            self.controller_param_history = np.zeros((num_steps + 1, 1))
            self.controller_param_names = ["T_out_hat_C"]
        else:
            print("Simulator: Controller does not have get_trackable_parameters or get_estimated_T_out method. No controller parameters will be tracked.")
        # ----------------------------------------------------------------------------

    def run(self):
        """Runs the simulation loop."""
        self.state_history[0, :] = self.system.get_state()
        self.time_history[0] = 0
        
        # Store initial controller parameter values
        if self.controller_param_history is not None:
            if hasattr(self.controller, 'get_trackable_parameters') and callable(self.controller.get_trackable_parameters):
                try:
                    initial_params = self.controller.get_trackable_parameters()
                    if len(initial_params) == self.controller_param_history.shape[1]:
                        self.controller_param_history[0, :] = list(initial_params.values())
                    else:
                         print("Warning: Mismatch between initial parameters and history shape. Initial params not stored.")
                except Exception as e:
                    print(f"Warning: Could not get initial controller parameters: {e}")
            elif hasattr(self.controller, 'Q_heater_controller') and hasattr(self.controller.Q_heater_controller, 'get_estimated_T_out'): # Fallback
                 try:
                    self.controller_param_history[0, 0] = self.controller.Q_heater_controller.get_estimated_T_out()
                 except Exception as e:
                    print(f"Warning: Could not get initial wrapped controller parameter: {e}")

        step_range = range(self.num_steps)
        if 'tqdm' in globals():
            step_range = tqdm(step_range, desc="Simulation Progress", leave=True) # Use leave=True for script

        for i in step_range:
            current_time = self.time_vector[i]
            current_state = self.system.get_state()
            
            # 1. Compute control input using the controller
            # Controller is expected to return the full input vector needed by system.step
            # The controller might update its internal state (like T_out_hat) here
            control_input = self.controller.compute_control(current_state=current_state, t=current_time)
            control_input = np.atleast_1d(control_input) # Ensure it's at least 1D array
            
            # --- Store controller parameters AFTER compute_control --- 
            if self.controller_param_history is not None:
                if hasattr(self.controller, 'get_trackable_parameters') and callable(self.controller.get_trackable_parameters):
                    try:
                        current_params = self.controller.get_trackable_parameters()
                        if len(current_params) == self.controller_param_history.shape[1]:
                            self.controller_param_history[i + 1, :] = list(current_params.values())
                        else:
                            print(f"Warning: Parameter mismatch at step {i}. Storing NaNs.")
                            self.controller_param_history[i + 1, :] = np.nan
                    except Exception as e:
                        print(f"Warning: Could not get controller parameters at step {i}: {e}")
                        self.controller_param_history[i + 1, :] = np.nan
                elif hasattr(self.controller, 'Q_heater_controller') and hasattr(self.controller.Q_heater_controller, 'get_estimated_T_out'): # Fallback
                    try:
                        self.controller_param_history[i + 1, 0] = self.controller.Q_heater_controller.get_estimated_T_out()
                    except Exception as e:
                         print(f"Warning: Could not get wrapped controller parameter at step {i}: {e}")
                         self.controller_param_history[i + 1, 0] = np.nan
            # ---------------------------------------------------------

            # --- Calculate and store derivative BEFORE stepping --- 
            try:
                derivative = self.system.get_state_derivative(control_input, state=current_state, t=current_time)
                self.derivative_history[i, :] = np.atleast_1d(derivative)
            except Exception as e:
                 # Handle cases where derivative calculation might fail or not be needed for step
                 # print(f"Warning: Could not compute/store state derivative at step {i}. Error: {e}")
                 # Store NaNs or zeros, depending on desired behavior
                 self.derivative_history[i, :] = np.nan # Use NaN for missing data
            # -------------------------------------------------------

            # Basic dimension check (optional)
            if control_input.shape[0] != self.control_history.shape[1]:
                 print(f"Warning: Control input dimension mismatch at step {i}. Expected {self.control_history.shape[1]}, got {control_input.shape[0]}. Storing NaN for this step.")
                 self.control_history[i, :] = np.nan # Store NaN if dimension mismatch
                 # Optionally, try resizing or raise error (resizing is complex)
            else:
                # Store the full control input vector used for this step
                self.control_history[i, :] = control_input

            # 2. Apply FULL control input and step the system
            # Ensure control_input passed to step is valid even if there was a mismatch above
            # For simplicity, we proceed, but a more robust system might handle the NaN
            self.system.step(self.dt, control_input if not np.isnan(self.control_history[i, 0]) else np.zeros_like(control_input)) 

            # 3. Store results
            self.state_history[i + 1, :] = self.system.get_state()
            self.time_history[i + 1] = current_time + self.dt

    def plot_results(self, save_path: str | None = None,
                       control_names: list[str] | None = None,
                       control_units: list[str] | None = None,
                       target_state: np.ndarray | None = None,
                       T_out_func: typing.Callable | None = None): # <-- Use typing.Callable
        """
        Plots the simulation results.

        Args:
            save_path: Optional path to save the plot.
            control_names: Optional list of names for control inputs.
            control_units: Optional list of units for control inputs.
            target_state: Optional array representing the target state for plotting.
            T_out_func: Optional callable function T_out(t) to plot actual T_out.
        """
        # Pass the histories and optional names/units to the Plotter.
        plotter = Plotter(
            time_vector=self.time_history,
            state_history=self.state_history,
            control_history=self.control_history,
            controller_param_history=self.controller_param_history, # <-- Pass controller params
            controller_param_names=self.controller_param_names, # <-- Pass param names
            system=self.system,
            control_names=control_names,
            control_units=control_units,
            target_state=target_state, # Pass target_state
            T_out_func=T_out_func # <-- Pass T_out_func
        )
        plotter.plot_results(save_path=save_path)

    def get_results(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray | None]:
        """Returns the simulation results (time, state, control_input, derivative, controller_params)."""
        return self.time_history, self.state_history, self.control_history, self.derivative_history, self.controller_param_history

    @classmethod
    def run_multiple(
        cls,
        initial_states_list: list[np.ndarray],
        system_class: typing.Type[System],
        system_args: dict,
        controller_class: typing.Type[Controller],
        controller_args: dict,
        dt: float,
        num_steps: int
    ) -> list[tuple[np.ndarray, np.ndarray]]:
        """
        Runs multiple simulations for a list of initial states.

        Args:
            cls: The class itself (automatically passed by @classmethod).
            initial_states_list: List of initial state NumPy arrays.
                                 For LightHouseKeeper, expects shape (2,) [pos, vel].
            system_class: The class of the system to simulate (e.g., Pendulum).
            system_args: Dictionary of arguments to pass to the system constructor
                         (excluding initial state related args like 'initial_state', 'initial_position', 'initial_velocity').
            controller_class: The class of the controller to use.
            controller_args: Dictionary of arguments to pass to the controller constructor.
            dt: Simulation time step.
            num_steps: Total number of simulation steps for each trajectory.

        Returns:
            A list of tuples, where each tuple contains (time_history, state_history)
            for one simulation run.
        """
        results_list = []
        print(f"Running {len(initial_states_list)} simulations...")

        # Use tqdm for the outer loop if available
        outer_loop_range = range(len(initial_states_list))
        if 'tqdm' in globals():
             outer_loop_range = tqdm(outer_loop_range, desc="Overall Progress")

        for i in outer_loop_range:
            initial_state = initial_states_list[i]
            if not isinstance(initial_state, np.ndarray) or initial_state.ndim != 1:
                raise ValueError(f"Element {i} in initial_states_list is not a 1D NumPy array.")

            # Prepare arguments for system constructor
            current_system_args = system_args.copy()

            # --- Handle system-specific initial state arguments --- 
            # Check if the system is LightHouseKeeper based on its name to avoid import issues
            if system_class.__name__ == 'LightHouseKeeper':
                if initial_state.shape != (2,):
                     raise ValueError(f"Initial state for LightHouseKeeper must have shape (2,) [pos, vel]. Got {initial_state.shape}")
                current_system_args['initial_position'] = initial_state[0]
                current_system_args['initial_velocity'] = initial_state[1]
            else: # Assume generic system accepts 'initial_state' argument
                current_system_args['initial_state'] = initial_state
            # --------------------------------------------------------

            # Create system instance with the specific initial state arguments
            system = system_class(**current_system_args)

            # Create controller instance
            controller = controller_class(**controller_args)

            # Create simulation instance using cls
            simulation = cls(
                system=system,
                controller=controller,
                dt=dt,
                num_steps=num_steps
            )

            # Run the individual simulation (no progress bar here)
            simulation.run() # Consider adding a quiet mode to sim.run()
            # Get results - ignore derivative and control history for multiple runs for now
            time_hist, state_hist, _, _, _ = simulation.get_results()

            results_list.append((time_hist, state_hist))

        print(f"{len(initial_states_list)} simulations finished.")
        return results_list 