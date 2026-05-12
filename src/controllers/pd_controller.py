import numpy as np
from ..controller import Controller # Using relative import for base class
from typing import Tuple, List, Optional, Union # Added List, Optional, Union

class PDController(Controller):
    """
    Proportional-Derivative (PD) controller.
    """
    def __init__(self,
                 Kp: float,
                 Kd: float,
                 dt: float, # dt is needed if we want to numerically differentiate the error, but here Kd is already on the error derivative
                 output_dim: int = 1,
                 control_limits: Optional[Tuple[float, float]] = None,
                 target_is_angle: bool = False,
                 name: str = "PDController"):
        """
        Initializes the PD controller.

        Args:
            Kp (float): Proportional gain.
            Kd (float): Derivative gain.
            dt (float): Time step, can be used for internal state if required.
                        In this implementation, Kd is multiplied by the already provided error derivative.
            output_dim (int): Dimension of the output signal.
            control_limits (Optional[Tuple[float, float]]): Tuple (min_output, max_output) to limit the output.
                                                             If None, no limit is applied.
            target_is_angle (bool): If True, the error in value will be normalized to the range [-pi, pi].
            name (str): Name of the controller.
        """
        super().__init__(target_state=None) # Base class might expect target_state
        self.name = name
        self.Kp = Kp
        self.Kd = Kd
        self.dt = dt # Store dt, although it might not be used directly in compute_control if error derivative is provided explicitly
        self.output_dim = output_dim
        self.target_is_angle = target_is_angle

        if control_limits is not None:
            # This check is for a single tuple limit. If a list of tuples is passed for multi-dim, it won't be caught here.
            if not (isinstance(control_limits, tuple) and len(control_limits) == 2 and
                    all(isinstance(x, (int, float)) for x in control_limits) and control_limits[0] <= control_limits[1]) \
               and not (isinstance(control_limits, list) and all(isinstance(item, tuple) for item in control_limits)): # Allow list of tuples
                raise ValueError("control_limits must be a tuple (min, max) or a list of such tuples.")
        
        if output_dim > 0:
            if control_limits is not None and isinstance(control_limits, list) and all(isinstance(item, tuple) for item in control_limits):
                 if len(control_limits) != output_dim:
                     raise ValueError("Length of control_limits list must match output_dim if a list of tuples is provided.")
                 self._control_limits_internal = list(control_limits)
            elif control_limits is not None and isinstance(control_limits, tuple): # Single tuple for all dimensions or scalar
                self._control_limits_internal = [control_limits] * output_dim
            elif control_limits is None:
                 self._control_limits_internal = [None] * output_dim
            else:
                # This case might arise if control_limits is a single tuple but not caught by the specific isinstance(control_limits, tuple) above
                # due to the complexity of the initial check when list of tuples is also allowed.
                # For safety, assume it's a single limit if it's not a list of tuples and not None.
                if isinstance(control_limits, tuple) and len(control_limits) == 2 and control_limits[0] <= control_limits[1]:
                    self._control_limits_internal = [control_limits] * output_dim
                else:
                    raise ValueError("Invalid format for control_limits.")
        else:
            self._control_limits_internal = []


    def compute_control(self,
                        current_value: Union[float, np.ndarray],
                        current_derivative: Union[float, np.ndarray],
                        target_value: Union[float, np.ndarray],
                        target_derivative: Optional[Union[float, np.ndarray]] = None,
                        t: Optional[float] = None) -> np.ndarray:
        """
        Computes the control signal based on the current value, its derivative, and target values.

        Args:
            current_value (Union[float, np.ndarray]): Current value of the controlled variable.
            current_derivative (Union[float, np.ndarray]): Current derivative of the controlled variable.
            target_value (Union[float, np.ndarray]): Target value of the controlled variable.
            target_derivative (Optional[Union[float, np.ndarray]]): Target derivative. If None, assumed to be 0.
            t (Optional[float]): Current time (not used by this controller).

        Returns:
            np.ndarray: Control signal (shape (output_dim,)).
        """
        current_value_arr = np.asarray(current_value).flatten()
        current_derivative_arr = np.asarray(current_derivative).flatten()
        target_value_arr = np.asarray(target_value).flatten()

        if target_derivative is None:
            # If dimension is > 1, create a zero array of the corresponding dimension
            if len(current_value_arr) > 1:
                 target_derivative_arr = np.zeros_like(current_value_arr)
            else: # Scalar case
                 target_derivative_arr = np.array([0.0])

        else:
            target_derivative_arr = np.asarray(target_derivative).flatten()
        
        if not (len(current_value_arr) == len(current_derivative_arr) == len(target_value_arr) == len(target_derivative_arr)):
             raise ValueError("Dimensions of current_value, current_derivative, target_value, and target_derivative must match.")

        error = target_value_arr - current_value_arr
        
        # Normalize error if it's an angle
        if self.target_is_angle:
            # Apply normalization for each element of error if it's multi-dimensional
            for i in range(len(error)):
                 error[i] = (error[i] + np.pi) % (2 * np.pi) - np.pi

        error_derivative = target_derivative_arr - current_derivative_arr

        control_output_unbounded = self.Kp * error + self.Kd * error_derivative
        
        # Ensure output is np.ndarray of shape (output_dim,)
        if self.output_dim == 1 and control_output_unbounded.ndim == 0: # If scalar
            control_output_unbounded = np.array([control_output_unbounded])
        elif control_output_unbounded.shape != (self.output_dim,) and self.output_dim == 1 and len(control_output_unbounded) == 1:
             control_output_unbounded = control_output_unbounded.reshape((self.output_dim,))
        elif control_output_unbounded.shape != (self.output_dim,):
             # Attempt to reshape if possible (e.g., if (N,) where N=output_dim)
            try:
                control_output_unbounded = control_output_unbounded.reshape((self.output_dim,))
            except ValueError:
                 raise ValueError(f"Computed control_output_unbounded shape {control_output_unbounded.shape} does not match output_dim ({self.output_dim},)")


        # Apply limits
        control_output_saturated = np.zeros(self.output_dim)
        for i in range(self.output_dim):
            limit = self._control_limits_internal[i]
            if limit:
                control_output_saturated[i] = np.clip(control_output_unbounded[i], limit[0], limit[1])
            else:
                control_output_saturated[i] = control_output_unbounded[i]
                
        return control_output_saturated

    def reset(self):
        # PD controller usually doesn't have internal state requiring reset (like an integrator in PID).
        pass

    def get_control_limits(self) -> Union[List[Optional[Tuple[float, float]]], Optional[Tuple[float, float]]]:
        """Returns the limits on the controller's output signal."""
        if self.output_dim == 1:
            return self._control_limits_internal[0]
        return self._control_limits_internal

if __name__ == '__main__':
    # Example usage
    pd_ctrl = PDController(Kp=10.0, Kd=0.5, dt=0.01, control_limits=(-5.0, 5.0))
    
    current_pos = 0.0
    current_vel = 0.0
    target_pos = 1.0
    target_vel = 0.0 # Target velocity 0

    control_signal = pd_ctrl.compute_control(current_pos, current_vel, target_pos, target_vel)
    print(f"State: pos={current_pos}, vel={current_vel}")
    print(f"Target: pos={target_pos}, vel={target_vel}")
    print(f"Control signal: {control_signal}")

    current_pos = 0.8
    current_vel = 0.1
    control_signal = pd_ctrl.compute_control(current_pos, current_vel, target_pos, target_vel)
    print(f"State: pos={current_pos}, vel={current_vel}, Control signal: {control_signal}")
    
    # Example with larger error causing saturation
    current_pos = -1.0
    current_vel = 0.0
    control_signal = pd_ctrl.compute_control(current_pos, current_vel, target_pos, target_vel)
    print(f"State: pos={current_pos}, vel={current_vel}, Control signal (saturation?): {control_signal}")

    # Example for MIMO (although the controller currently handles it as a set of SISO)
    pd_ctrl_2d = PDController(Kp=10.0, Kd=0.5, dt=0.01, output_dim=2, control_limits=(-5.0, 5.0))
    current_vals = np.array([0.1, -0.1])
    current_derivs = np.array([0.0, 0.0])
    target_vals = np.array([1.0, 0.0])
    # target_derivs defaults to None -> [0,0]
    control_signal_2d = pd_ctrl_2d.compute_control(current_vals, current_derivs, target_vals)
    print(f"MIMO State: val={current_vals}, der={current_derivs}")
    print(f"MIMO Target: val={target_vals}")
    print(f"MIMO Control signal: {control_signal_2d}")

    pd_ctrl_2d_limits_list = PDController(
        Kp=10.0, Kd=0.5, dt=0.01, output_dim=2, 
        control_limits=[(-5.0, 5.0), (-2.0, 2.0)] # If we want different limits
    )
    # For this to work, the logic of _control_limits_internal needs a slight change.
    # The current implementation _control_limits_internal = [control_limits] * output_dim will take the whole list as one limit.
    # To support a list of limits, __init__ needs to be modified.
    # But for the current task, this is not needed, as output_dim=1 for torque.
    # Let's leave it as is for now. It can be improved if needed. 