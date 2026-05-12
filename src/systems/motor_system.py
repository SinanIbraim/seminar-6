import numpy as np
from ..system import System # Using relative import for base class

class MotorSystem(System):
    """
    First-order motor system model.

    Dynamics: d(tau)/dt = (1/gamma) * (-tau + u)
    State: [tau] (torque)
    Control: [u] (torque command)
    """
    def __init__(self, initial_state: np.ndarray, gamma: float = 0.1):
        """
        Initializes the motor system.

        Args:
            initial_state (np.ndarray): Initial state [tau].
            gamma (float): Motor time constant.
        """
        if not isinstance(initial_state, np.ndarray) or initial_state.ndim != 1 or initial_state.shape[0] != 1:
            raise ValueError("Initial state must be a 1D NumPy array with one element (tau).")
        if not isinstance(gamma, (int, float)) or gamma <= 0:
            raise ValueError("gamma must be a positive number.")

        _name = "First Order Motor"
        _state_names = ["Torque"]
        _state_units = ["Nm"]

        super().__init__(name=_name, 
                         initial_state=initial_state,
                         state_names=_state_names,
                         state_units=_state_units)
        self.gamma = gamma
        self._input_dim = 1 # Input dimension for internal checks
        
        # self.name, self.state_names, self.state_units attributes are set by the base class.
        # control_input_names and control_input_units are specific to this system and used by the plotter.
        self.control_input_names = ["Torque Command"]
        self.control_input_units = ["Nm"]

    def get_state_derivative(self, u: np.ndarray, state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        """
        Calculates the state derivative.

        Args:
            u (np.ndarray): Control vector [u_tau].
            state (np.ndarray | None): Current state [tau]. If None, self.state is used.
            t (float | None): Current time (not used in this model).

        Returns:
            np.ndarray: State derivative [d_tau_dt].
        """
        current_tau = self._state[0] if state is None else state[0]
        # Check dimension of u before use
        if not isinstance(u, np.ndarray) or u.ndim != 1 or u.shape[0] != self._input_dim:
            raise ValueError(f"Control u must be a 1D NumPy array with {self._input_dim} element(s). Received: {u}")
        u_tau = u[0]

        d_tau_dt = (1 / self.gamma) * (-current_tau + u_tau)
        return np.array([d_tau_dt])

    def step(self, dt: float, u: np.ndarray) -> None:
        """
        Performs a simulation step using Euler's method.
        Overridden to use _input_dim for checking u if necessary,
        or to ensure the correct state is passed to get_state_derivative.

        Args:
            dt (float): Time step.
            u (np.ndarray): Control vector [u_tau].
        """
        if not isinstance(u, np.ndarray) or u.ndim != 1 or u.shape[0] != self._input_dim:
            raise ValueError(f"Control u for step must be a 1D NumPy array with {self._input_dim} element(s). Received: {u}")
            
        current_state_for_derivative = self.get_state() # Use get_state() to get a copy
        
        state_derivative = self.get_state_derivative(u, state=current_state_for_derivative)
        
        new_state = current_state_for_derivative + state_derivative * dt
        self.set_state(new_state) 