import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.collections import LineCollection
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from typing import Any, Callable # Import Any and Callable
from .system import System # Corrected relative import
import os

class Plotter:
    def __init__(self, time_vector: np.ndarray | None = None, 
                 state_history: np.ndarray | None = None, 
                 control_history: np.ndarray | None = None, 
                 derivative_history: np.ndarray | None = None, 
                 controller_param_history: np.ndarray | None = None,
                 controller_param_names: list[str] | None = None,
                 system: System | Any | None = None,
                 control_names: list[str] | None = None,
                 control_units: list[str] | None = None,
                 target_state: np.ndarray | None = None,
                 T_out_func: Callable[[float], float] | None = None):
        """
        Initializes the Plotter.

        Args:
            time_vector: Array of time points. Can be None.
            state_history: Array of state vectors over time. Can be None.
            control_history: Array of control inputs over time. Can be None.
            derivative_history: Array of state derivative vectors over time. Can be None.
            controller_param_history: Array of controller parameters over time. Can be None.
            controller_param_names: List of names for controller parameters. Can be None.
            system: The system object (instance of System or similar). Can be None.
            control_names: Optional list of names for each control input variable.
            control_units: Optional list of units for each control input variable.
            target_state: Optional array representing the target state for plotting.
            T_out_func: Optional callable function T_out(t) to plot actual T_out.
        """
        self.t = time_vector
        self.states = state_history
        self.system = system
        self.controls = control_history
        self.derivatives = derivative_history
        self.controller_params = controller_param_history
        self.controller_param_names = controller_param_names if controller_param_names is not None else []
        self.control_names = control_names
        self.control_units = control_units
        self.target_state = target_state
        self.T_out_func = T_out_func
        self.E_kin_history = None
        self.E_pot_history = None
        self.E_tot_history = None
        self.E_des = None
        # Store original state names (or defaults)
        self.state_names = ["State 0", "State 1"] # Default names

        # Determine state names from system or generate defaults
        if self.states is not None:
            num_states = self.states.shape[1]
            default_names = [f"State {i}" for i in range(num_states)]
            self.state_names = default_names # Start with defaults
            if self.system is not None and isinstance(self.system, System) and hasattr(self.system, 'state_names') and self.system.state_names is not None:
                if len(self.system.state_names) == num_states:
                    self.state_names = self.system.state_names
                else:
                    print(f"Warning: Mismatch between system.state_names ({len(self.system.state_names)}) and state history dimension ({num_states}). Using defaults: {default_names}")
        elif self.system is not None and isinstance(self.system, System) and hasattr(self.system, 'state_names') and self.system.state_names is not None:
             # Use system names if states not available yet (e.g., for multiplot setup)
             self.state_names = self.system.state_names

        # Determine state units from system or default to empty strings
        self.state_units = None
        if self.states is not None:
            num_states = self.states.shape[1]
            self.state_units = ["" for _ in range(num_states)] # Default to empty strings
            if self.system is not None and isinstance(self.system, System) and hasattr(self.system, 'state_units') and self.system.state_units is not None:
                if len(self.system.state_units) == num_states:
                    self.state_units = self.system.state_units
                else:
                    print(f"Warning: Mismatch between system.state_units ({len(self.system.state_units)}) and state history dimension ({num_states}). Using default empty units.")
        elif self.system is not None and isinstance(self.system, System) and hasattr(self.system, 'state_units') and self.system.state_units is not None:
             # Use system units if states not available yet
             self.state_units = self.system.state_units

        # Validate control names/units if provided
        if self.controls is not None:
            num_controls = self.controls.shape[1]
            if self.control_names is not None and len(self.control_names) != num_controls:
                 print(f"Warning: Length of control_names ({len(self.control_names)}) does not match control_history dimension ({num_controls}). Ignoring control_names.")
                 self.control_names = None
            if self.control_units is not None and len(self.control_units) != num_controls:
                 print(f"Warning: Length of control_units ({len(self.control_units)}) does not match control_history dimension ({num_controls}). Ignoring control_units.")
                 self.control_units = None

        # Check if state dimension is suitable for phase portrait (needs exactly 2)
        self.can_plot_phase_portrait = self.states is not None and self.states.shape[1] == 2
        self.phase_plot_is_1d = False
        if self.states is not None:
             num_states = self.states.shape[1]
             if num_states == 1 and self.derivatives is not None and self.derivatives.shape[0] == self.states.shape[0]-1 and self.derivatives.shape[1] == 1:
                 self.can_plot_phase_portrait = True
                 self.phase_plot_is_1d = True
             elif num_states == 2:
                 self.can_plot_phase_portrait = True
                 self.phase_plot_is_1d = False

        # Perform checks and calculations only if data for a single run is provided
        if self.t is not None and self.states is not None and self.controls is not None:
            # Ensure control history has NaN padding if needed
            if self.controls.shape[0] == len(self.t) - 1:
                nan_row = np.full((1, self.controls.shape[1]), np.nan)
                self.controls = np.vstack([self.controls, nan_row])
            elif self.controls.shape[0] != len(self.t):
                 raise ValueError(f"Control history shape {self.controls.shape} mismatch with time vector length {len(self.t)}.")

            if self.states.shape[0] != len(self.t):
                raise ValueError("State history must have the same length as the time vector.")
            # Keep check for suitable state variables for phase portrait
            # Note: The main state plot will show all states, but phase portrait requires 2.
            if not self.can_plot_phase_portrait:
                 print("Info: State history dimension is not 2. Phase portrait will not be generated.")

            # Conditionally calculate energy history if system and methods exist
            if self.system is not None and \
               hasattr(self.system, 'get_kinetic_energy') and \
               hasattr(self.system, 'get_potential_energy'):
                try:
                    self.E_kin_history = np.array([self.system.get_kinetic_energy(s) for s in self.states])
                    self.E_pot_history = np.array([self.system.get_potential_energy(s) for s in self.states])
                    self.E_tot_history = self.E_kin_history + self.E_pot_history
                    # Conditionally get desired energy
                    if hasattr(self.system, 'get_desired_energy'):
                        self.E_des = self.system.get_desired_energy()
                except Exception as e:
                    print(f"Warning: Could not calculate energy history. Error: {e}")
                    self.E_kin_history = None # Ensure reset on error
                    self.E_pot_history = None
                    self.E_tot_history = None
                    self.E_des = None
            else:
                 print("Info: System object not provided or missing energy methods. Skipping energy calculation.")

    def plot_results(self, save_path: str | None = None):
        """Plots the state variables, control input, energies (if available), and phase portrait (if possible)."""
        if self.t is None or self.states is None or self.controls is None:
            raise ValueError("Cannot plot results: missing time, state or control data.")

        num_states = self.states.shape[1]
        # Extract state variables - keep all for the state plot
        state_vars = [self.states[:, i] for i in range(num_states)]

        # Determine if energy plot is possible
        plot_energy = self.E_tot_history is not None
        # Determine if phase portrait plot is possible
        plot_phase = self.can_plot_phase_portrait

        # Adjust layout based on available plots
        n_cols = 2
        # Determine number of rows and height ratios based on which plots are active
        if plot_phase and plot_energy:
            n_main_rows = 3
            height_ratios = [1, 2, 1] # Ratios for State/Control, Phase, Energy
        elif plot_phase and not plot_energy:
            n_main_rows = 2
            height_ratios = [1, 2] # Ratios for State/Control, Phase
        elif not plot_phase and plot_energy:
            n_main_rows = 2
            height_ratios = [1, 1] # Ratios for State/Control, Energy
        else: # Only State/Control
            n_main_rows = 1
            height_ratios = [1]
            
        fig = plt.figure(figsize=(14, 3 + 3 * n_main_rows), constrained_layout=True) # Adjust height based on rows
        gs = gridspec.GridSpec(n_main_rows, n_cols, figure=fig, height_ratios=height_ratios)

        # Add overall title using system name if available
        if self.system and hasattr(self.system, 'name'):
             fig.suptitle(f'Simulation Results for {self.system.name}', fontsize=16)

        # Create subplot axes
        ax_state = fig.add_subplot(gs[0, 0])
        ax_control = fig.add_subplot(gs[0, 1])
        
        current_row = 1
        ax_phase = None
        if plot_phase:
            ax_phase = fig.add_subplot(gs[current_row, :])
            current_row += 1
            
        ax_energy = None
        if plot_energy:
             # Use the next available row index
            ax_energy = fig.add_subplot(gs[current_row, :])
            # current_row += 1 # Increment if more plots were added below

        # --- Plotting Data --- #
        # 1. State Variables & Derivatives (Top Left)
        y_label_state = 'Value' 
        time_for_derivatives = self.t[:-1]
        
        lines_state = []
        labels_state = []
        lines_deriv = []
        labels_deriv = []
        
        ax_deriv = None 

        # --- Added: Grid setup for the states axis ---
        ax_state.yaxis.grid(True, linestyle='--', alpha=0.7) # Grid along the main Y-axis
        ax_state.xaxis.grid(False) # Disable vertical grid if not needed
        # ----------------------------------------------------

        for i in range(num_states):
            # Determine color based on state index
            # Assign distinct colors for state and its derivative - REVERTED
            if i == 0:
                 color = 'blue' # Single color for state and derivative
            elif i == 1:
                 color = 'red' # Single color for state and derivative
            else:
                 color = None # Use default color cycle

            # State Plot
            state_name = self.state_names[i] if i < len(self.state_names) else f"State {i}"
            unit = self.state_units[i] if self.state_units and i < len(self.state_units) and self.state_units[i] else ""
            label_state = f"{state_name}{f' ({unit})' if unit else ''}"
            # Use determined single color, solid line
            line, = ax_state.plot(self.t, state_vars[i], label=label_state, color=color, linestyle='-') 
            lines_state.append(line)
            labels_state.append(label_state)
            
            if i == 0 and unit:
                 y_label_state = f"{state_name} ({unit})"
            elif i == 0:
                 y_label_state = state_name
                 
            # Derivative Plot (if available) on secondary axis
            if self.derivatives is not None and i < self.derivatives.shape[1]:
                if not np.all(np.isnan(self.derivatives[:, i])):
                    derivative_values = self.derivatives[:, i]
                    if len(derivative_values) == len(time_for_derivatives):
                        label_deriv = f"d({state_name})/dt{f' ({unit}/s)' if unit else ''}" 
                        if ax_deriv is None:
                             ax_deriv = ax_state.twinx()
                             # Use color of the first state for axis
                             deriv_axis_color = 'blue' if num_states > 0 else 'grey' # Use blue if i=0
                             y_label_deriv = f"Derivative{f' ({unit}/s)' if unit else ''}" if i==0 else "Derivative Value"
                             ax_deriv.set_ylabel(y_label_deriv, color=deriv_axis_color)
                             ax_deriv.tick_params(axis='y', labelcolor=deriv_axis_color)
                             
                             # --- Added: Disable grid for secondary axis ---
                             ax_deriv.grid(False) # Ensure secondary axis does not have its own grid

                        # Use determined single color, dashed line
                        line_d, = ax_deriv.plot(time_for_derivatives, derivative_values, label=label_deriv, color=color, linestyle='--')
                        lines_deriv.append(line_d)
                        labels_deriv.append(label_deriv)
                        if num_states > 1 and i > 0:
                             ax_deriv.set_ylabel("Derivative Value", color='grey')
                             ax_deriv.tick_params(axis='y', labelcolor='grey')
                    else:
                         print(f"Warning: Skipping derivative plot for state {i} due to length mismatch (time {len(time_for_derivatives)}, derivative {len(derivative_values)}).")
                else:
                     print(f"Info: Derivative history for state {i} contains only NaNs. Skipping plot.")
                      
        ax_state.set_title('State Variables & Derivatives')
        ax_state.set_ylabel(y_label_state)
        ax_state.set_xlabel('Time (s)')
        ax_state.grid(True, axis='x') # Grid only on x-axis for primary
        if ax_deriv: 
             ax_deriv.grid(True, axis='y', linestyle='--', alpha=0.7) # Grid only on y-axis for secondary
        # Combine legends
        ax_state.legend(lines_state + lines_deriv, labels_state + labels_deriv, loc='best')

        # --- Add target state line --- 
        if self.target_state is not None:
            if isinstance(self.target_state, np.ndarray) and self.target_state.shape[0] == num_states:
                target_label = f'Target {self.state_names[0]}' if num_states > 0 else 'Target'
                target_line = ax_state.axhline(self.target_state[0], color='grey', linestyle='--', linewidth=1, label=target_label)
                # Ensure we don't add duplicate labels
                current_handles, current_labels = ax_state.get_legend_handles_labels()
                is_present = any(lbl == target_label for lbl in current_labels)
                if not is_present:
                     lines_state.append(target_line)
                     labels_state.append(target_label)
            else:
                print("Warning: target_state provided but has incorrect shape or type. Skipping target line.")
        # ---------------------------

        # 2. Control Inputs (Top Right)
        num_controls = self.controls.shape[1]
        lines_control = []
        labels_control = []
        y_labels_control = []
        
        # --- Prepare T_out and T_out_hat data --- 
        T_out_actual = None
        T_out_hat = None
        T_out_index = -1
        T_out_hat_index = -1
        
        # Try to find T_out in control inputs
        if self.control_names:
            try:
                T_out_index = self.control_names.index("T_out")
                # Check if T_out_func is provided to calculate the actual T_out
                if self.T_out_func is not None:
                     T_out_actual = np.array([self.T_out_func(t_val) for t_val in self.t])
                     print("Plotter: Calculated actual T_out using provided T_out_func.")
                else: # Use the one from control history (might be noisy or just the input)
                    # Need self.t for plotting, control history might miss last point
                     if self.controls.shape[0] == len(self.t):
                         T_out_actual = self.controls[:, T_out_index]
                     elif self.controls.shape[0] == len(self.t) - 1:
                         # Pad control history for plotting continuity
                         padded_controls = np.vstack([self.controls, np.full((1, num_controls), np.nan)])
                         T_out_actual = padded_controls[:, T_out_index]
                     print("Plotter: Using T_out from control history (T_out_func not provided).")
            except ValueError:
                print("Plotter: 'T_out' not found in control_names.")
                T_out_index = -1 # Ensure it's -1 if not found

        # Try to find T_out_hat in controller parameters
        if self.controller_params is not None and self.controller_param_names:
            try:
                T_out_hat_index = self.controller_param_names.index("T_out_hat_C")
                if self.controller_params.shape[0] == len(self.t):
                    T_out_hat = self.controller_params[:, T_out_hat_index]
                    print("Plotter: Found T_out_hat_C in controller_param_history.")
                else:
                     print(f"Warning: Mismatch between controller_param_history length ({self.controller_params.shape[0]}) and time vector length ({len(self.t)}). Skipping T_out_hat plot.")
                     T_out_hat = None # Reset if length mismatch
            except ValueError:
                print("Plotter: 'T_out_hat_C' not found in controller_param_names.")
                T_out_hat_index = -1 # Ensure it's -1
        # -------------------------------------------

        # --- Plot Control Signals --- 
        twin_axes = {}
        base_ax = ax_control
        base_color_idx = 0

        for i in range(num_controls):
            # Skip plotting T_out explicitly here if we plot actual/hat below
            if i == T_out_index and (T_out_actual is not None or T_out_hat is not None):
                 continue 

            control_name = self.control_names[i] if self.control_names and i < len(self.control_names) else f"Control {i}"
            unit = self.control_units[i] if self.control_units and i < len(self.control_units) and self.control_units[i] else ""
            label_control = f"{control_name}{f' ({unit})' if unit else ''}"

            # Use different axes for different units if provided and units differ
            current_ax = base_ax
            current_color = f'C{base_color_idx}'
            if unit and unit not in y_labels_control and len(y_labels_control) > 0:
                 if unit not in twin_axes:
                     twin_axes[unit] = base_ax.twinx()
                     # Position the new axis to avoid overlap
                     rspine = twin_axes[unit].spines['right']
                     rspine.set_position(('axes', 1.0 + 0.15 * (len(twin_axes) - 1))) # Adjust position based on number of twins
                 current_ax = twin_axes[unit]
                 # Use next colors for twin axes
                 current_color = f'C{len(y_labels_control)}'
            elif len(y_labels_control) == 0: # First plot
                 base_color_idx += 1
            else: # Same unit as base axis or no unit
                 current_color = f'C{base_color_idx}'
                 base_color_idx += 1 # Use next color on base axis

            # Plot control signal (use self.t[:-1] as control is applied between steps)
            # Need to handle potential NaN padding if control history was shorter
            if self.controls.shape[0] == len(self.t):
                control_values = self.controls[:, i]
                time_for_control = self.t
            elif self.controls.shape[0] == len(self.t) - 1:
                 control_values = self.controls[:, i]
                 time_for_control = self.t[:-1]
            else:
                 print(f"Warning: Unexpected control history length {self.controls.shape[0]} vs time {len(self.t)}. Skipping plot for control {i}.")
                 continue # Skip this control signal
            
            # Replace potential NaNs from simulator adjustment with previous value for plotting step-like
            nan_mask = np.isnan(control_values)
            if np.any(nan_mask):
                 print(f"Plotter Warning: NaNs found in control history for '{control_name}'. Filling forward for plot.")
                 # Simple forward fill for plotting
                 idx = np.where(~nan_mask, np.arange(len(control_values)), 0)
                 np.maximum.accumulate(idx, out=idx) 
                 control_values = control_values[idx]
            
            # Use step plot for control signals
            line, = current_ax.step(time_for_control, control_values, where='post', label=label_control, color=current_color)
            lines_control.append(line)
            labels_control.append(label_control)
            
            if unit and unit not in y_labels_control:
                 y_labels_control.append(unit)
                 current_ax.set_ylabel(f"{control_name} ({unit})", color=current_color)
                 current_ax.tick_params(axis='y', labelcolor=current_color)
                 current_ax.grid(False) # Turn off grid for twin axes initially
            elif not unit and not y_labels_control: # First plot, no unit
                  y_labels_control.append("") # Placeholder
                  base_ax.set_ylabel(control_name)
                  base_ax.tick_params(axis='y', labelcolor=current_color)

        # --- Plot T_out_actual and T_out_hat on the control plot axis --- 
        T_out_unit = "°C" # Assume Celsius
        T_out_axis = base_ax # Default to base axis
        T_out_color = f'C{base_color_idx}' # Color for T_out plots
        
        # Find or create axis for Temperature
        temp_axis_found = False
        if T_out_unit in y_labels_control:
             # Find which axis corresponds to T_out_unit
             if T_out_index != -1: #If T_out was plotted before, use its axis.
                 # Find the axis used for T_out originally
                 pass # Logic to find axis is complex, default to base or new twin
             # Simplified: If a temp axis exists, use it. Check base first.
             if base_ax.get_ylabel().endswith(f"({T_out_unit})"):
                  T_out_axis = base_ax
                  temp_axis_found = True
             else:
                  for unit, ax in twin_axes.items():
                       if unit == T_out_unit:
                            T_out_axis = ax
                            temp_axis_found = True
                            break
        
        # If no suitable axis exists, create a new twin axis if needed
        if not temp_axis_found and (T_out_actual is not None or T_out_hat is not None):
             if T_out_unit not in y_labels_control and len(y_labels_control) > 0:
                 if T_out_unit not in twin_axes:
                     twin_axes[T_out_unit] = base_ax.twinx()
                     rspine = twin_axes[T_out_unit].spines['right']
                     rspine.set_position(('axes', 1.0 + 0.15 * (len(twin_axes) - 1)))
                 T_out_axis = twin_axes[T_out_unit]
                 T_out_color = f'C{len(y_labels_control)}'
                 T_out_axis.set_ylabel(f"Temperature ({T_out_unit})", color=T_out_color)
                 T_out_axis.tick_params(axis='y', labelcolor=T_out_color)
                 T_out_axis.grid(False)
                 y_labels_control.append(T_out_unit)
             elif len(y_labels_control) == 0: # First plot is Temperature
                  T_out_axis = base_ax
                  T_out_color = f'C{base_color_idx}'
                  T_out_axis.set_ylabel(f"Temperature ({T_out_unit})", color=T_out_color)
                  T_out_axis.tick_params(axis='y', labelcolor=T_out_color)
                  y_labels_control.append(T_out_unit)
        
        # Plot Actual T_out if available
        if T_out_actual is not None:
            label = f"Actual T_out ({T_out_unit})"
            line, = T_out_axis.plot(self.t, T_out_actual, label=label, color='black', linestyle='--', linewidth=1.5) # Thicker dashed black
            lines_control.append(line)
            labels_control.append(label)
            
        # Plot Estimated T_out (T_out_hat) if available
        if T_out_hat is not None:
            label = f"Estimated T_out_hat ({T_out_unit})"
            # Plot T_out_hat using self.t (it has N+1 points like state)
            line, = T_out_axis.plot(self.t, T_out_hat, label=label, color='red', linestyle=':', linewidth=1.5) # Thicker dotted red
            lines_control.append(line)
            labels_control.append(label)
        # --------------------------------------------------------------
        
        ax_control.set_title('Control Inputs & Temperature Estimate')
        ax_control.set_xlabel('Time (s)')
        # Combine legends for all axes
        base_ax.legend(lines_control, labels_control, loc='best')
        # Apply grid to base axis
        base_ax.grid(True, axis='both', linestyle='--', alpha=0.6)
        # Align all y-axis zeros if axes are different
        # align_yaxis_zeros(base_ax, list(twin_axes.values())) # Helper function needed

        # 3. Phase Portrait (Middle Full Width, if possible)
        if plot_phase and ax_phase is not None:
            time_for_phase = self.t # Default full time
            x = None
            x_dot = None
            
            if self.phase_plot_is_1d:
                # Plot state vs derivative
                # Ensure derivatives are available and have correct dimensions
                if self.derivatives is None or self.derivatives.shape[0] != len(self.t) - 1 or self.derivatives.shape[1] != 1:
                    print("Warning: Cannot plot 1D phase portrait - derivatives missing or incorrect shape.")
                    plot_phase = False
                else:
                    x = self.states[:, 0]         # Length N+1
                    x_dot = self.derivatives[:, 0] # Length N
                    time_for_phase = self.t[:-1]   # Length N
                    x = x[:-1]                     # Adjust state x to match derivative length (N)
            else: # 2D case
                 # Plot state[0] vs state[1]
                 if self.states.shape[1] == 2:
                    x = self.states[:, 0]     # Length N+1
                    x_dot = self.states[:, 1] # Length N+1
                    time_for_phase = self.t   # Length N+1
                 else:
                    print(f"Warning: Cannot plot 2D phase portrait - system state dimension is not 2 (it is {self.states.shape[1]}).")
                    plot_phase = False

            # Proceed only if plot is still enabled and data prepared
            if plot_phase and x is not None and x_dot is not None:
                # --- Filter NaN/Inf values --- 
                valid_mask = ~np.isnan(x) & ~np.isnan(x_dot) & ~np.isinf(x) & ~np.isinf(x_dot)
                if len(time_for_phase) == len(x): # Ensure time matches the potentially adjusted x
                     time_plot = time_for_phase[valid_mask]
                elif len(self.t) == len(x): # Case for 2D
                     time_plot = self.t[valid_mask]
                else:
                     print("Warning: Time vector length mismatch in phase plot after masking. Using unmasked time.")
                     time_plot = time_for_phase # Fallback, might not align perfectly
                     
                x_plot = x[valid_mask]
                x_dot_plot = x_dot[valid_mask]
                # -----------------------------

                if len(x_plot) == 0:
                    print("Warning: No valid points to plot in phase portrait after removing NaNs/Infs.")
                else:
                    scatter = ax_phase.scatter(x_plot, x_dot_plot, c=time_plot, cmap=cm.rainbow, s=10, label='Phase Trajectory', alpha=0.6)
                    
                    # Plot start/end points using original indices but checking the mask
                    try:
                        start_idx = 0
                        end_idx = len(x) - 1 # Use length of original x before filtering
                        if valid_mask[start_idx]:
                             # Use original x, x_dot for plotting start/end if they were valid
                            ax_phase.scatter(x[start_idx], x_dot[start_idx], color='blue', s=100, marker='o', label='Start', zorder=5, alpha=0.8)
                        else:
                             print("Info: Start point contained NaN/Inf, not plotted.")
                             
                        # For 1D plot, x_dot has length N, state has N+1 initially.
                        # If phase_plot_is_1d, x was truncated. We need to compare end_idx with len(x_dot)
                        if end_idx < len(x_dot): # Ensure index is valid for x_dot
                            if valid_mask[end_idx]:
                                ax_phase.scatter(x[end_idx], x_dot[end_idx], color='red', s=100, marker='x', label='End', zorder=5, alpha=0.8)
                            else:
                                 print("Info: End point contained NaN/Inf, not plotted.")
                        else:
                             print("Info: End point index mismatch for 1D plot, not plotted.")
                             
                    except IndexError:
                        print("Warning: Could not plot start/end points (short simulation or NaN issues?).")
                    
                    # Use state names and units for labels
                    name_state_0 = self.state_names[0]
                    unit_state_0 = self.state_units[0] if self.state_units and self.state_units[0] else ""
                    
                    if self.phase_plot_is_1d:
                        # Labeling for State vs Derivative
                        name_state_1_label = f"d{name_state_0}/dt"
                        unit_state_1 = f"{unit_state_0}/s" if unit_state_0 else "" 
                        title_state_0_fmt = f"{name_state_0}{f' ({unit_state_0})' if unit_state_0 else ''}"
                        title_state_1_fmt = f"{name_state_1_label}{f' ({unit_state_1})' if unit_state_1 else ''}"
                        ax_phase.set_title(f'Phase Portrait ({title_state_1_fmt} vs {title_state_0_fmt})')
                        label_state_0 = f'${name_state_0}$' if name_state_0.replace('_', '').isalnum() else name_state_0
                        label_state_0 += f' ({unit_state_0})' if unit_state_0 else ''
                        label_state_1 = f'$\dot{{{name_state_0}}}$' if name_state_0.replace('_', '').isalnum() else name_state_1_label 
                        label_state_1 += f' ({unit_state_1})' if unit_state_1 else ''
                    else: # 2D Case
                        name_state_1 = self.state_names[1]
                        unit_state_1 = self.state_units[1] if self.state_units and len(self.state_units) > 1 and self.state_units[1] else ""
                        title_state_0_fmt = f"{name_state_0}{f' ({unit_state_0})' if unit_state_0 else ''}"
                        title_state_1_fmt = f"{name_state_1}{f' ({unit_state_1})' if unit_state_1 else ''}"
                        ax_phase.set_title(f'Phase Portrait ({title_state_1_fmt} vs {title_state_0_fmt})')
                        label_state_0 = f'${name_state_0}$' if name_state_0.replace('_', '').isalnum() else name_state_0
                        label_state_0 += f' ({unit_state_0})' if unit_state_0 else ''
                        label_state_1 = f'${name_state_1}$' if name_state_1.replace('_', '').isalnum() else name_state_1
                        label_state_1 += f' ({unit_state_1})' if unit_state_1 else ''
                        
                    ax_phase.set_xlabel(label_state_0, fontsize=12)
                    ax_phase.set_ylabel(label_state_1, fontsize=12)
                    ax_phase.grid(True)
                    
                    # Automatic axis limits based on plotted data
                    if len(x_plot) > 0: # Ensure there's data to compute limits
                         x_range = np.ptp(x_plot) if len(x_plot) > 1 else 0.1
                         x_dot_range = np.ptp(x_dot_plot) if len(x_dot_plot) > 1 else 0.1
                         x_padding = x_range * 0.1 if x_range > 1e-6 else 0.1
                         x_dot_padding = x_dot_range * 0.1 if x_dot_range > 1e-6 else 0.1
                         ax_phase.set_xlim(np.min(x_plot) - x_padding, np.max(x_plot) + x_padding)
                         ax_phase.set_ylim(np.min(x_dot_plot) - x_dot_padding, np.max(x_dot_plot) + x_dot_padding)

                    cbar = plt.colorbar(scatter, ax=ax_phase)
                    cbar.set_label('Time (s)')
                    # Add legend for Start/End markers (ensure no duplicates)
                    handles, labels = ax_phase.get_legend_handles_labels()
                    by_label = dict(zip(labels, handles))
                    ax_phase.legend(by_label.values(), by_label.keys(), loc='best')

        # 4. Energy Plot (Bottom Full Width, if possible)
        if plot_energy and ax_energy is not None:
            ax_energy.plot(self.t, self.E_kin_history, label='Kinetic Energy', color='blue', alpha=0.7)
            ax_energy.plot(self.t, self.E_pot_history, label='Potential Energy', color='green', alpha=0.7)
            ax_energy.plot(self.t, self.E_tot_history, label='Total Energy', color='red', linewidth=2)

            # Plot desired energy if available
            if self.E_des is not None:
                ax_energy.axhline(y=self.E_des, color='black', linestyle='--', label='Desired Energy')

            ax_energy.set_title('Energy Components')
            ax_energy.set_xlabel('Time (s)')
            ax_energy.set_ylabel('Energy')
            ax_energy.grid(True)
            ax_energy.legend(loc='upper right')

        # Save or show the plot
        if save_path:
            output_dir = os.path.dirname(save_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            try:
                plt.savefig(save_path, bbox_inches='tight')
                print(f"Plot saved to {save_path}")
            except Exception as e:
                print(f"Error saving plot to {save_path}: {e}")
            plt.close(fig) # Close the figure after saving
        else:
            plt.show()

    def plot_multiple_phase_portraits(self, simulation_results_list: list[tuple[np.ndarray, np.ndarray]],
                                      equilibrium_point: np.ndarray | None = None, # Made optional
                                      k_coeffs: tuple[float, float] | None = None,
                                      eigenvalues: tuple[complex, complex] | None = None,
                                      title: str | None = None, # Use system name if None
                                      plot_range: tuple[float, float, float, float] | None = None,
                                      save_path: str | None = None):
        """
        Plots multiple phase portraits on the same figure.

        Args:
            simulation_results_list: A list of tuples, where each tuple contains
                                     (time_vector, state_history). state_history
                                     should have shape (num_steps+1, 2).
            equilibrium_point: Optional equilibrium point [x, x_dot] to plot.
            k_coeffs: Optional tuple (k1, k2) for title annotation.
            eigenvalues: Optional tuple (lambda1, lambda2) for title annotation.
            title: Optional title for the plot. If None, uses system name or default.
            plot_range: Optional tuple (xmin, xmax, ymin, ymax) for axis limits.
            save_path: Optional path to save the plot. If None, shows the plot.
        """
        # --- Check for 2D compatibility --- 
        if not simulation_results_list: 
            print("Warning: No simulation results provided for multi-plot.")
            return None
        
        first_state_history = simulation_results_list[0][1]
        if first_state_history.shape[1] != 2:
             print(f"Warning: plot_multiple_phase_portraits currently only supports 2D systems (state vs state_dot). System has {first_state_history.shape[1]} dimensions. Skipping plot.")
             return None
        # ------------------------------------
        
        fig, ax = plt.subplots(figsize=(12, 10))

        # Setup colormap
        cmap = cm.cool
        try:
            t_norm_ref = simulation_results_list[0][0]
            norm = mcolors.Normalize(vmin=t_norm_ref.min(), vmax=t_norm_ref.max())
        except (IndexError, TypeError):
             print("Warning: Could not determine time range for color normalization.")
             norm = mcolors.Normalize(vmin=0, vmax=1) # Default norm

        print(f"Plotting {len(simulation_results_list)} trajectories...")

        all_x = []
        all_x_dot = []

        for i, (t, states) in enumerate(simulation_results_list):
            if states.shape[1] != 2:
                 print(f"Warning: Skipping trajectory {i} due to incorrect state dimension ({states.shape[1]} != 2).")
                 continue
            x = states[:, 0]
            x_dot = states[:, 1]
            all_x.extend(x)
            all_x_dot.extend(x_dot)

            if len(t) < 2 or len(x) < 2:
                 print(f"Warning: Skipping trajectory {i} due to insufficient points.")
                 continue

            points = np.array([x, x_dot]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)

            lc = LineCollection(segments, cmap=cmap, norm=norm)
            segment_times = (t[:-1] + t[1:]) / 2
            lc.set_array(segment_times)
            lc.set_linewidth(1.5)
            ax.add_collection(lc)

            # Plot start/end points if enough data
            if len(x) > 0:
                ax.plot(x[0], x_dot[0], 'o', color='blue', markersize=8, label='Start' if i == 0 else "")
            if len(x) > 1:
                 ax.plot(x[-1], x_dot[-1], 'x', color='red', markersize=8, label='End' if i == 0 else "")

        # Plot equilibrium point if provided
        if equilibrium_point is not None:
            ax.plot(equilibrium_point[0], equilibrium_point[1], 'o', color='firebrick', markersize=10, label='Equilibrium Point')
            ax.plot(equilibrium_point[0], equilibrium_point[1], 'o', mec='black', mfc='none', markersize=10) # Outline

        # Add axes lines crossing at the equilibrium point or origin
        cross_point = equilibrium_point if equilibrium_point is not None else np.array([0.0, 0.0])
        ax.axhline(cross_point[1], color='grey', linestyle='--', linewidth=0.8)
        ax.axvline(cross_point[0], color='grey', linestyle='--', linewidth=0.8)

        # Add colorbar
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        cbar = fig.colorbar(sm, ax=ax)
        cbar.set_label('Time evolution', fontsize=12)
        cbar.ax.tick_params(labelsize=10)

        # Setup title
        if title is None:
             if self.system and hasattr(self.system, 'name'):
                 base_title = f"Phase Portraits for {self.system.name}"
             else:
                 # Use assumed derivative name convention for default title if system name unavailable
                 name_state_0 = self.state_names[0] if len(self.state_names) > 0 else "State 0"
                 name_state_1_label = f"{name_state_0}_dot"
                 base_title = f'Phase Portraits ({name_state_0} vs {name_state_1_label})'
        else:
             base_title = title

        subtitle_parts = []
        if k_coeffs:
            subtitle_parts.append(f"$k_1 = {k_coeffs[0]:.2f}, k_2 = {k_coeffs[1]:.2f}$")
        if eigenvalues:
             lambda1_str = f"{eigenvalues[0].real:.2f}{eigenvalues[0].imag:+.2f}i"
             lambda2_str = f"{eigenvalues[1].real:.2f}{eigenvalues[1].imag:+.2f}i"
             subtitle_parts.append(rf"$\lambda_1 = {lambda1_str}, \lambda_2 = {lambda2_str}$")

        full_title = base_title
        if subtitle_parts:
            full_title += "\n" + "\n".join(subtitle_parts)

        ax.set_title(full_title, fontsize=14)

        # Setup axes labels using state_names and state_units
        name_state_0 = self.state_names[0] if len(self.state_names) > 0 else "State 0"
        unit_state_0 = self.state_units[0] if self.state_units and len(self.state_units) > 0 and self.state_units[0] else ""
        name_state_1 = self.state_names[1] if len(self.state_names) > 1 else "State 1"
        unit_state_1 = self.state_units[1] if self.state_units and len(self.state_units) > 1 and self.state_units[1] else ""

        # Format labels
        label_x_unit = f' ({unit_state_0})' if unit_state_0 else ''
        label_x_fmt = f'${name_state_0}$' if name_state_0.replace('_', '').isalnum() else name_state_0
        label_x_fmt += label_x_unit # Append unit
        
        label_y_unit = f' ({unit_state_1})' if unit_state_1 else ''
        label_y_fmt = f'${name_state_1}$' if name_state_1.replace('_', '').isalnum() else name_state_1
        label_y_fmt += label_y_unit # Append unit

        ax.set_xlabel(label_x_fmt, fontsize=14)
        ax.set_ylabel(label_y_fmt, fontsize=14)
        ax.grid(True, linestyle=':', alpha=0.7)

        ax.tick_params(axis='both', which='major', labelsize=12)

        # Set axis limits
        if plot_range:
            ax.set_xlim(plot_range[0], plot_range[1])
            ax.set_ylim(plot_range[2], plot_range[3])
        elif all_x and all_x_dot: # Set limits only if data exists
            pad_x = (max(all_x) - min(all_x)) * 0.1 if max(all_x) != min(all_x) else 0.1
            pad_y = (max(all_x_dot) - min(all_x_dot)) * 0.1 if max(all_x_dot) != min(all_x_dot) else 0.1
            pad_x = max(pad_x, 0.1) # Ensure minimum padding
            pad_y = max(pad_y, 0.1)

            min_x_val = min(all_x)
            max_x_val = max(all_x)
            min_y_val = min(all_x_dot)
            max_y_val = max(all_x_dot)

             # Include equilibrium point in limits if defined
            if equilibrium_point is not None:
                 min_x_val = min(min_x_val, equilibrium_point[0])
                 max_x_val = max(max_x_val, equilibrium_point[0])
                 min_y_val = min(min_y_val, equilibrium_point[1])
                 max_y_val = max(max_y_val, equilibrium_point[1])

            ax.set_xlim(min_x_val - pad_x, max_x_val + pad_x)
            ax.set_ylim(min_y_val - pad_y, max_y_val + pad_y)
        else:
             print("Warning: No data to determine automatic plot limits.")

        # Add legend for Start/End/Equilibrium points
        handles, labels = ax.get_legend_handles_labels()
        # Remove duplicate labels before creating legend
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='best')

        plt.tight_layout(rect=[0, 0, 1, 0.96] if full_title.count('\n') > 0 else None) # Adjust layout for long titles
        
        # Save or show the plot
        if save_path:
            output_dir = os.path.dirname(save_path)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            try:
                plt.savefig(save_path, bbox_inches='tight')
                print(f"Plot saved to {save_path}")
            except Exception as e:
                print(f"Error saving plot to {save_path}: {e}")
            plt.close(fig) # Close the figure after saving
        else:
             plt.show()
        return ax 

    # --- NEW METHOD FOR ADAPTIVE SIMULATION ---
    def plot_adaptive_results(
        self,
        time: np.ndarray,
        state_history: np.ndarray,
        state_names: list[str],
        state_units: list[str],
        control_history: np.ndarray,
        control_names: list[str],
        control_units: list[str],
        target_state_history: np.ndarray | None = None,
        target_state_names: list[str] | None = None,
        target_state_units: list[str] | None = None,
        T_out_func: Callable[[float], float] | None = None, # For plotting actual T_out
        adaptive_history: dict | None = None, # Dictionary from Adaptive controller
        true_params: dict | None = None,      # Optional dict like {'K1': val, 'K2': val}
        save_path: str | None = None,
        title_suffix: str = "Adaptive Control"
    ):
        """Plots results for a simulation, including adaptive parameters if available."""
        if time is None or state_history is None or control_history is None:
            raise ValueError("Missing essential data: time, state_history, or control_history.")

        num_states = state_history.shape[1]
        num_controls = control_history.shape[1]

        # Validate inputs
        if len(state_names) != num_states: raise ValueError("Length mismatch: state_names")
        if len(state_units) != num_states: raise ValueError("Length mismatch: state_units")
        if len(control_names) != num_controls: raise ValueError("Length mismatch: control_names")
        if len(control_units) != num_controls: raise ValueError("Length mismatch: control_units")
        if target_state_history is not None:
            if target_state_names is None or len(target_state_names) != target_state_history.shape[1]: raise ValueError("Length mismatch: target_state_names")
            if target_state_units is None or len(target_state_units) != target_state_history.shape[1]: raise ValueError("Length mismatch: target_state_units")

        # Check for adaptive data
        plot_adaptive_params = adaptive_history is not None and \
                               'K_total_hat' in adaptive_history and \
                               'C_air_hat' in adaptive_history and \
                               len(adaptive_history.get('t', [])) == len(time) # Safer check

        plot_adaptive_gains = adaptive_history is not None and \
                              'Kp' in adaptive_history and \
                              'Ki' in adaptive_history and \
                              len(adaptive_history.get('t', [])) == len(time) # Safer check

        # --- Layout Setup (2x2 Grid) ---
        fig = plt.figure(figsize=(15, 8), constrained_layout=True) # Adjusted size for 2x2
        gs = gridspec.GridSpec(2, 2, figure=fig, height_ratios=[1, 1], width_ratios=[1,1])

        fig.suptitle(f'Simulation Results - {title_suffix}', fontsize=16)

        # Create axes in 2x2 grid
        ax_state = fig.add_subplot(gs[0, 0])
        ax_control = fig.add_subplot(gs[0, 1])
        ax_params = fig.add_subplot(gs[1, 0], sharex=ax_state) if plot_adaptive_params else None
        ax_gains = fig.add_subplot(gs[1, 1], sharex=ax_control) if plot_adaptive_gains else None
        
        # Handle cases where adaptive plots are missing
        if not plot_adaptive_params:
             ax_params_placeholder = fig.add_subplot(gs[1, 0], sharex=ax_state)
             ax_params_placeholder.text(0.5, 0.5, 'Parameter Estimation Plot Unavailable',
                                     horizontalalignment='center', verticalalignment='center',
                                     transform=ax_params_placeholder.transAxes, color='grey')
             ax_params_placeholder.tick_params(axis='both', which='both', bottom=False, top=False, left=False, right=False, labelbottom=False, labelleft=False)
             ax_params_placeholder.set_xlabel('Time (s)') # Still need x-label if bottom left
             if not plot_adaptive_gains:
                  plt.setp(ax_control.get_xticklabels(), visible=True)

        if not plot_adaptive_gains:
             ax_gains_placeholder = fig.add_subplot(gs[1, 1], sharex=ax_control)
             ax_gains_placeholder.text(0.5, 0.5, 'Gain Calculation Plot Unavailable',
                                    horizontalalignment='center', verticalalignment='center',
                                    transform=ax_gains_placeholder.transAxes, color='grey')
             ax_gains_placeholder.tick_params(axis='both', which='both', bottom=False, top=False, left=False, right=False, labelbottom=False, labelleft=False)
             ax_gains_placeholder.set_xlabel('Time (s)') # Still need x-label if bottom right
             if not plot_adaptive_params:
                   plt.setp(ax_state.get_xticklabels(), visible=True)

        # --- Plotting --- #

        # 1. State and Target (Top Left)
        ax_state.set_title('System State and Target')
        ax_state.set_ylabel('Value')
        ax_state.grid(True, linestyle='--', alpha=0.7)

        # Plot states, targets, T_out_actual (same as before)
        # ... (plotting code for ax_state remains the same) ...
        # Plot states
        for i in range(num_states):
            unit_str = f' ({state_units[i]})' if state_units[i] else ''
            ax_state.plot(time, state_history[:, i], label=f'{state_names[i]}{unit_str}')
            
        # Plot targets
        if target_state_history is not None:
            for i in range(target_state_history.shape[1]):
                unit_str = f' ({target_state_units[i]})' if target_state_units[i] else ''
                ax_state.plot(time, target_state_history[:, i], label=f'{target_state_names[i]}{unit_str}', linestyle='--', alpha=0.8)
                
        # Plot actual T_out if function provided
        if T_out_func is not None:
            T_out_actual = np.array([T_out_func(t) for t in time])
            ax_state.plot(time, T_out_actual, label='T_out (Actual)', linestyle=':', color='gray', alpha=0.9)
            
        ax_state.legend(loc='best')
        plt.setp(ax_state.get_xticklabels(), visible=False) # Hide x-labels for top row

        # 2. Control Inputs (Top Right)
        ax_control.set_title('Control Inputs (T_out / Q_heater)')
        ax_control.grid(True, linestyle='--', alpha=0.7)
        ax_control.tick_params(axis='y', labelcolor='tab:blue')
        ax_control.set_ylabel(f'{control_names[0]}{f" ({control_units[0]})" if control_units[0] else ""}', color='tab:blue') # Assume first is T_out
        
        ax_control_q = ax_control.twinx() # Create twin axis for Q_heater
        ax_control_q.set_ylabel(f'{control_names[1]}{f" ({control_units[1]})" if control_units[1] else ""}', color='tab:red') # Assume second is Q_heater
        ax_control_q.tick_params(axis='y', labelcolor='tab:red')
        ax_control_q.grid(False) # No grid for twin axis

        lines = []
        labels = []

        # Plot T_out (index 0) on primary axis
        if num_controls > 0:
            control_data_0 = control_history[:, 0]
            valid_indices_0 = ~np.isnan(control_data_0)
            line0, = ax_control.plot(time[valid_indices_0], control_data_0[valid_indices_0], 
                                     label=f'{control_names[0]}{f" ({control_units[0]})" if control_units[0] else ""}', 
                                     color='tab:blue')
            lines.append(line0)
            labels.append(f'{control_names[0]}{f" ({control_units[0]})" if control_units[0] else ""}')
            
        # Plot Q_heater (index 1) on secondary axis
        if num_controls > 1:
            control_data_1 = control_history[:, 1]
            valid_indices_1 = ~np.isnan(control_data_1)
            line1, = ax_control_q.plot(time[valid_indices_1], control_data_1[valid_indices_1], 
                                       label=f'{control_names[1]}{f" ({control_units[1]})" if control_units[1] else ""}', 
                                       color='tab:red')
            lines.append(line1)
            labels.append(f'{control_names[1]}{f" ({control_units[1]})" if control_units[1] else ""}')

        # Combine legends
        ax_control.legend(lines, labels, loc='best')
        
        plt.setp(ax_control.get_xticklabels(), visible=False) # Hide x-labels for top row
        plt.setp(ax_control_q.get_xticklabels(), visible=False)

        # 3. Adaptive Parameter Estimates (Bottom Left, Optional)
        if plot_adaptive_params and ax_params is not None:
            ax_params.set_title('Adaptive Parameter Estimates ($K_{total}, C_{air}$)')
            ax_params.set_ylabel('$\hat{K}_{total}$ Estimate', color='tab:blue')
            ax_params.tick_params(axis='y', labelcolor='tab:blue')
            ax_params.grid(True, linestyle='--', alpha=0.7)

            ax_params_c_air = ax_params.twinx()
            ax_params_c_air.set_ylabel('$\hat{C}_{air}$ Estimate', color='tab:red')
            ax_params_c_air.tick_params(axis='y', labelcolor='tab:red')
            ax_params_c_air.grid(False)

            hist_t = adaptive_history['t']
            hist_k_total = adaptive_history.get('K_total_hat')
            hist_c_air = adaptive_history.get('C_air_hat')

            if hist_k_total is None or hist_c_air is None or \
               len(hist_t) != len(time) or \
               len(hist_k_total) != len(time) or \
               len(hist_c_air) != len(time):
                 print(f"Warning: Length mismatch or missing K_total/C_air data in adaptive_history. Skipping param plot.")
            else:
                 param_plot_handles = []

                 line_k_total, = ax_params.plot(hist_t, hist_k_total, color='tab:blue', label='$\hat{K}_{total}$')
                 param_plot_handles.append(line_k_total)

                 if true_params and 'K_total' in true_params:
                     k_total_true_val = true_params['K_total']
                     label_k_total_true = f'$K_{{total}}$ (True = {k_total_true_val:.2e})'
                     line_k_total_true = ax_params.axhline(k_total_true_val, color='tab:blue', linestyle='--', alpha=0.5, label=label_k_total_true)
                     param_plot_handles.append(line_k_total_true)

                 line_c_air, = ax_params_c_air.plot(hist_t, hist_c_air, color='tab:red', label='$\hat{C}_{air}$')
                 param_plot_handles.append(line_c_air)

                 if true_params and 'C_air' in true_params:
                     c_air_true_val = true_params['C_air']
                     label_c_air_true = f'$C_{{air}}$ (True = {c_air_true_val:.2e})'
                     line_c_air_true = ax_params_c_air.axhline(c_air_true_val, color='tab:red', linestyle='--', alpha=0.5, label=label_c_air_true)
                     param_plot_handles.append(line_c_air_true)

                 lines1, labels1 = ax_params.get_legend_handles_labels()
                 lines2, labels2 = ax_params_c_air.get_legend_handles_labels()
                 ax_params.legend(lines1 + lines2, labels1 + labels2, loc='best')

                 all_k_total_vals = list(hist_k_total)
                 if true_params and 'K_total' in true_params: all_k_total_vals.append(true_params['K_total'])
                 min_k_total, max_k_total = min(all_k_total_vals), max(all_k_total_vals)
                 pad_k_total = (max_k_total - min_k_total) * 0.1 if max_k_total != min_k_total else abs(max_k_total * 0.1) + 0.1
                 ax_params.set_ylim(min_k_total - pad_k_total, max_k_total + pad_k_total)
                 
                 all_c_air_vals = list(hist_c_air)
                 if true_params and 'C_air' in true_params: all_c_air_vals.append(true_params['C_air'])
                 min_c_air, max_c_air = min(all_c_air_vals), max(all_c_air_vals)
                 pad_c_air = (max_c_air - min_c_air) * 0.1 if max_c_air != min_c_air else abs(max_c_air * 0.1) + 0.1
                 ax_params_c_air.set_ylim(min_c_air - pad_c_air, max_c_air + pad_c_air)
                 
                 plt.setp(ax_params.get_xticklabels(), visible=True)

        # 4. Adaptive Gains (Bottom Right, Optional)
        if plot_adaptive_gains and ax_gains is not None:
            ax_gains.set_title('Calculated PID Gains (Kp, Ki)')
            ax_gains.set_ylabel('Gain Value')
            ax_gains.grid(True, linestyle='--', alpha=0.7)

            # Plot Kp, Ki (same as before)
            # ... (plotting code for ax_gains remains the same) ...
            hist_t = adaptive_history['t']
            hist_kp = adaptive_history['Kp']
            hist_ki = adaptive_history['Ki']

            if len(hist_t) != len(time) or len(hist_kp) != len(time) or len(hist_ki) != len(time):
                 print(f"Warning: Length mismatch in adaptive_history gains. Skipping gains plot.")
            else:
                 gain_plot_handles = []
                 line_kp, = ax_gains.plot(hist_t, hist_kp, label='$K_p$')
                 gain_plot_handles.append(line_kp)
                 
                 ax_gains_ki = ax_gains.twinx()
                 # Сделаем линию Ki пунктирной и красной
                 line_ki, = ax_gains_ki.plot(hist_t, hist_ki, label='$K_i$', color='red', linestyle='--')
                 gain_plot_handles.append(line_ki)
                 ax_gains_ki.set_ylabel('$K_i$ Value', color='red')
                 ax_gains_ki.tick_params(axis='y', labelcolor='red')
                 ax_gains_ki.grid(False)

                 # Combine legends
                 lines, labels = ax_gains.get_legend_handles_labels()
                 lines2, labels2 = ax_gains_ki.get_legend_handles_labels()
                 ax_gains_ki.legend(lines + lines2, labels + labels2, loc='best')
                 
                 # Set Y label for primary axis (Kp - blue)
                 ax_gains.set_ylabel('$K_p$ Value', color='tab:blue') # Используем цвет линии Kp
                 ax_gains.tick_params(axis='y', labelcolor='tab:blue')
                 
            ax_gains.set_xlabel('Time (s)') # Add x-label to bottom row
            if ax_gains_ki: ax_gains_ki.set_xlabel('Time (s)')

        # --- Final Touches ---
        # Remove redundant x-labels if both bottom plots exist
        if plot_adaptive_params and plot_adaptive_gains:
            plt.setp(ax_params.get_xticklabels(), visible=True) # Keep bottom left
            plt.setp(ax_gains.get_xticklabels(), visible=True) # Keep bottom right
            if ax_params_c_air: plt.setp(ax_params_c_air.get_xticklabels(), visible=True)
            if ax_gains_ki: plt.setp(ax_gains_ki.get_xticklabels(), visible=True)
        elif plot_adaptive_params: # Only params plot on bottom row
             plt.setp(ax_params.get_xticklabels(), visible=True)
             if ax_params_c_air: plt.setp(ax_params_c_air.get_xticklabels(), visible=True)
             plt.setp(ax_control.get_xticklabels(), visible=True) # Show x-labels for top-right
        elif plot_adaptive_gains: # Only gains plot on bottom row
             plt.setp(ax_gains.get_xticklabels(), visible=True)
             if ax_gains_ki: plt.setp(ax_gains_ki.get_xticklabels(), visible=True)
             plt.setp(ax_state.get_xticklabels(), visible=True) # Show x-labels for top-left
        else: # No plots on bottom row
             plt.setp(ax_state.get_xticklabels(), visible=True)
             plt.setp(ax_control.get_xticklabels(), visible=True)

        # Save or show
        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")
        else:
            plt.show()
        plt.close(fig) # Close the figure after saving/showing

# Ensure the class definition ends properly 