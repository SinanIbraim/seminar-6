import numpy as np
from ..controller import Controller # Using relative import for base class
from typing import List, Tuple, Optional, Callable

class PIController(Controller):
    """
    Proportional-Integral (PI) controller.
    Control: u = Kp * e + Ki * integral(e*dt)
    Can accept a static target or a function to generate the target over time.
    """
    def __init__(self, 
                 Kp: float, 
                 Ki: float,
                 dt: float, # Time step, necessary for numerical integration
                 static_target_state: np.ndarray | None = None, 
                 target_func: Callable[[float], np.ndarray] | None = None,
                 output_dim: int = 1,
                 integral_limit: float | None = None, # Optional limit for the integral sum
                 initial_integral_error: np.ndarray | None = None,
                 name: str = "PIController"): # Added name argument
        """
        Initializes the PI controller.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            dt (float): Simulation time step (for discrete integrator).
            static_target_state (np.ndarray | None): Static target state.
            target_func (Callable[[float], np.ndarray] | None): Target function of time.
            output_dim (int): Dimension of the control output.
            integral_limit (float | None): Limit for the absolute value of the integral sum (anti-windup).
            initial_integral_error (np.ndarray | None): Initial value of the integral error (defaults to 0).
            name (str): Name of the controller.
        """
        super().__init__(target_state=static_target_state)
        self.name = name # Set name after super()
        
        if not isinstance(Kp, (int, float)) or Kp < 0:
            raise ValueError("Kp must be a non-negative number.")
        if not isinstance(Ki, (int, float)) or Ki < 0:
            raise ValueError("Ki must be a non-negative number.")
        if not isinstance(dt, (int, float)) or dt <= 0:
            raise ValueError("dt (time step) must be a positive number.")
        if not isinstance(output_dim, int) or output_dim < 1:
            raise ValueError("output_dim must be a positive integer.")
        if static_target_state is not None and target_func is not None:
            print("Warning: PIController - both static_target_state and target_func are specified. target_func will be used.")
        if integral_limit is not None and integral_limit < 0:
            raise ValueError("integral_limit must be non-negative if specified.")
            
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.target_func = target_func
        self.output_dim = output_dim # Set attribute directly (was _output_dim)
        self._limits = [(None, None)] * self.output_dim # Limits on controller output
        self.integral_limit = integral_limit

        if initial_integral_error is None:
            self._integral_error = np.zeros(self.output_dim) 
        elif isinstance(initial_integral_error, np.ndarray) and initial_integral_error.shape == (self.output_dim,):
            self._integral_error = initial_integral_error.copy()
        else:
            raise ValueError(f"initial_integral_error must be a NumPy array of shape ({self.output_dim},) or None.")

    def _get_target_to_use(self, arg_target_state: np.ndarray | None, t: float | None) -> np.ndarray | None:
        """Helper method to determine which target_state to use."""
        if self.target_func is not None and t is not None:
            return self.target_func(t)
        if arg_target_state is not None:
            return arg_target_state
        return self.target_state # target_state is set in super().__init__

    def compute_control(self, current_state: np.ndarray, target_state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        actual_target = self._get_target_to_use(arg_target_state=target_state, t=t)
        
        if actual_target is None:
            print("Warning: PIController - failed to determine target. Returning zero control.")
            # Reset integral if target is not defined, so it doesn't accumulate uncontrollably
            self.reset_integral()
            return np.zeros(self.output_dim) # USING self.output_dim
            
        if current_state.shape != actual_target.shape:
            # Attempt to handle case where current_state might be (N,) and actual_target (1,) for broadcasting
            # This check assumes output_dim correctly reflects the target dimension for error calculation.
            if not (current_state.ndim == 1 and actual_target.ndim == 1 and 
                    actual_target.shape[0] == self.output_dim and 
                    (current_state.shape[0] == self.output_dim or self.output_dim == 1)):
                 # Added more specific condition: if output_dim is 1, current_state can be (1,) or (N,) if it's to be compared to a scalar target.
                 # This logic is getting complex; ideally, inputs should be pre-shaped to match output_dim.
                 raise ValueError(
                    f"Dimension of current state {current_state.shape} "
                    f"is incompatible with dimension of target state {actual_target.shape} or output_dim {self.output_dim}."
                 )
        
        error = actual_target - current_state
        if error.shape != (self.output_dim,):
             # This block tries to handle mismatched error shape vs output_dim
             if self.output_dim == 1 and error.ndim == 1 and error.shape[0] == 1:
                 # If error is (1,) and output_dim is 1, it's fine.
                 pass # error = error (already correct shape for scalar output_dim)
             elif self.output_dim == 1 and error.ndim == 1 and error.shape[0] > 1:
                 # If output_dim is 1, but error is multi-dim [e1, e2...], what to do?
                 # Defaulting to using the first component of error. This should be explicitly defined by use case.
                 print(f"Warning: PIController - error is multi-dimensional {error.shape} but output_dim is 1. Using first component of error: {error[0]}.")
                 error = np.array([error[0]]) 
             elif error.ndim > 1 or (error.ndim ==1 and error.shape[0] != self.output_dim) :
                 raise ValueError(f"Error {error.shape} must have shape ({self.output_dim},) or be adaptable for PI controller output_dim {self.output_dim}.")

        # Integral part (rectangular method)
        self._integral_error += error * self.dt

        # Apply anti-windup (limit integral sum)
        if self.integral_limit is not None:
            self._integral_error = np.clip(self._integral_error, -self.integral_limit, self.integral_limit)
            
        # Proportional part
        p_term = self.Kp * error
        # Integral part
        i_term = self.Ki * self._integral_error
        
        control_output = p_term + i_term
        
        # Ensure output is a 1D array of shape (output_dim,)
        if control_output.ndim == 0 and self.output_dim == 1:
            control_output = np.array([control_output]) # Ensure it's an array
        elif control_output.shape != (self.output_dim,):
            try:
                control_output = control_output.reshape((self.output_dim,))
            except ValueError:
                 print(f"Warning: PIController - output signal shape {control_output.shape} could not be reshaped to output_dim ({self.output_dim},). Returning as is.")

        # Apply general limits on controller output (not implemented in detail yet)
        # for i in range(self.output_dim):
        #     min_val, max_val = self._limits[i]
        #     if min_val is not None: control_output[i] = max(min_val, control_output[i])
        #     if max_val is not None: control_output[i] = min(max_val, control_output[i])
                
        return control_output

    def reset_integral(self) -> None:
        """Resets the accumulated integral error."""
        self._integral_error = np.zeros(self.output_dim)

    def get_integral_error(self) -> np.ndarray:
        """Returns the current accumulated integral error."""
        return self._integral_error.copy()

    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        if self.output_dim == 1:
             return self._limits[0]
        else:
             return self._limits

    # set_Kp, set_Ki can be added if needed
    def set_coefficients(self, Kp: float, Ki: float):
        if not isinstance(Kp, (int, float)) or Kp < 0: raise ValueError("Kp >= 0")
        if not isinstance(Ki, (int, float)) or Ki < 0: raise ValueError("Ki >= 0")
        self.Kp = Kp
        self.Ki = Ki 