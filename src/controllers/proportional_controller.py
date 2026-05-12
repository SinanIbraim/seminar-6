import numpy as np
from ..controller import Controller # Using relative import for base class
from typing import List, Tuple, Optional, Callable

class ProportionalController(Controller):
    """
    Proportional controller.
    Control: u = Kp * (target_state - current_state)
    Can accept a static target or a function to generate the target over time.
    """
    def __init__(self, 
                 Kp: float, 
                 static_target_state: np.ndarray | None = None, 
                 target_func: Callable[[float], np.ndarray] | None = None,
                 output_dim: int = 1):
        """
        Initializes the proportional controller.

        Args:
            Kp (float): Proportional gain coefficient.
            static_target_state (np.ndarray | None): Static target state. 
                                                      Used if target_func is not provided.
            target_func (Callable[[float], np.ndarray] | None): Function generating the target state at time t.
                                                                Takes float (time), returns np.ndarray (target).
            output_dim (int): Dimension of the control output signal (default 1).
        """
        super().__init__(target_state=static_target_state)
        
        if not isinstance(Kp, (int, float)):
            raise ValueError("Kp must be a number.")
        if not isinstance(output_dim, int) or output_dim < 1:
            raise ValueError("output_dim must be a positive integer.")
        if static_target_state is not None and target_func is not None:
            print("Warning: ProportionalController - both static_target_state and target_func are specified. target_func will be used.")
            
        self.Kp = Kp
        self.target_func = target_func
        self.output_dim = output_dim
        self._limits = [(None, None)] * output_dim

    def _get_target_to_use(self, arg_target_state: np.ndarray | None, t: float | None) -> np.ndarray | None:
        """Helper method to determine which target_state to use."""
        if self.target_func is not None and t is not None:
            return self.target_func(t)
        if arg_target_state is not None:
            return arg_target_state
        return self.target_state # This is static_target_state from the constructor

    def compute_control(self, current_state: np.ndarray, target_state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        """
        Computes the control action.

        Args:
            current_state (np.ndarray): Current state of the system.
            target_state (np.ndarray | None): Target state (argument). If None or if self.target_func is set,
                                              this argument might be ignored in favor of self.target_func(t) or self.target_state.
            t (float | None): Current time (important if target_func is used).

        Returns:
            np.ndarray: Calculated control action.
        """
        actual_target = self._get_target_to_use(arg_target_state=target_state, t=t)
        
        if actual_target is None:
            print("Warning: ProportionalController - failed to determine target. Returning zero control.")
            return np.zeros(self.output_dim)
            
        current_state_arr = np.asarray(current_state).reshape(-1)
        actual_target_arr = np.asarray(actual_target).reshape(-1)

        if current_state_arr.shape[0] != self.output_dim or actual_target_arr.shape[0] != self.output_dim:
            if self.output_dim == 1 and current_state_arr.size == 1 and actual_target_arr.size == 1:
                current_state_arr = current_state_arr.reshape((1,))
                actual_target_arr = actual_target_arr.reshape((1,))
            else:
                raise ValueError(
                    f"Dimensions of current state ({current_state_arr.shape}) and target state ({actual_target_arr.shape}) "
                    f"must match controller output_dim ({self.output_dim})."
                )

        error = actual_target_arr - current_state_arr
        control_output = self.Kp * error
        
        if control_output.shape != (self.output_dim,):
            try:
                control_output = control_output.reshape((self.output_dim,))
            except ValueError:
                 print(f"Warning: ProportionalController - output signal shape {control_output.shape} could not be reshaped to ({self.output_dim},). Returning as is.")
               
        return control_output

    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        """Returns the control limits."""
        if self.output_dim == 1:
             return self._limits[0]
        else:
             return self._limits

    def set_Kp(self, Kp: float):
        if not isinstance(Kp, (int, float)):
            raise ValueError("Kp must be a number.")
        self.Kp = Kp
