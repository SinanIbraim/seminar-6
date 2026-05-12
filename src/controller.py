from abc import ABC, abstractmethod
import numpy as np
from typing import List, Tuple, Optional

class Controller(ABC):
    """Abstract base class for controllers."""

    def __init__(self, target_state: np.ndarray | None = None):
        """
        Initializes the Controller.

        Args:
            target_state (np.ndarray | None, optional): The desired target state for the system. 
                                                     Defaults to None. If provided, it might be used 
                                                     by compute_control if no target is passed there.
        """
        if target_state is not None and not isinstance(target_state, np.ndarray):
            raise TypeError("target_state must be a NumPy array or None.")
        self._target_state = target_state.copy() if target_state is not None else None

    def get_target_state(self) -> np.ndarray | None:
        """Returns the target state set during initialization (if any)."""
        return self._target_state.copy() if self._target_state is not None else None

    def set_target_state(self, target_state: np.ndarray):
        """
        Sets or updates the internal target state after initialization.

        Args:
            target_state (np.ndarray): The new target state vector.
        """
        if not isinstance(target_state, np.ndarray):
            raise TypeError("target_state must be a NumPy array.")
        # Optional: Add shape validation if needed, e.g.,
        # if self._target_state is not None and target_state.shape != self._target_state.shape:
        #     raise ValueError(f"New target state shape {target_state.shape} must match original shape {self._target_state.shape}.")
        self._target_state = target_state.copy()

    @abstractmethod
    def compute_control(self, current_state: np.ndarray, target_state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        """
        Computes the control input based on the current state, target state, and time.

        Implementations MUST handle the logic for determining the target state:
        1. Prioritize the `target_state` passed to this method.
        2. If `target_state` is None during the call, use the internal `self._target_state` (set via __init__ or set_target_state).
        3. If no target state is available from either source (both are None), the implementation MUST raise a ValueError.

        Args:
            current_state: The current state vector of the system.
            target_state: The desired target state vector for this specific computation. If None, 
                          the controller will use the internal target state.
            t: The current time (optional).

        Returns:
            The computed control input(s) as a NumPy array.

        Raises:
            ValueError: If `target_state` is None and no internal target state has been set.
        """
        pass

    @abstractmethod
    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        """
        Returns the minimum and maximum limits for the control output(s).

        Returns:
            - Tuple[Optional[float], Optional[float]]: (min_limit, max_limit) for a single output controller.
            - List[Tuple[Optional[float], Optional[float]]]: List of (min_limit, max_limit) tuples for multi-output controllers.
              None indicates no limit.
        """
        pass

class ZeroController(Controller):
    """A controller that always outputs zero control input."""

    def __init__(self, n_outputs: int = 1, **kwargs):
        """
        Initializes the ZeroController.

        Args:
            n_outputs: The dimension of the control output vector (default is 1).
            **kwargs: Catches additional arguments like 'target_state' but ignores them.
        """
        super().__init__(**kwargs) # Pass potential target_state up, though it's not used here
        if n_outputs < 1:
             raise ValueError("Number of outputs must be at least 1.")
        self._output_dim = n_outputs
        self._zero_output = np.zeros(n_outputs) # Pre-compute zero array
        # Define limits based on zero output
        self._limits = [(0.0, 0.0)] * n_outputs

    def compute_control(self, current_state: np.ndarray, target_state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        """
        Returns a zero control input vector of the specified dimension. Ignores state and target.

        Args:
            current_state: The current state vector (ignored).
            target_state: The target state vector (ignored).
            t: The current time (ignored).

        Returns:
            A NumPy array of zeros with shape (n_outputs,).
        """
        # Return the pre-computed zero array for efficiency
        return self._zero_output
        
    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        """Returns the control limits, which are always (0.0, 0.0) for each output."""
        if self._output_dim == 1:
             return self._limits[0]
        else:
             return self._limits

# Example usage:
# if __name__ == '__main__':
#     # Example for a system with 2 control inputs
#     zero_ctrl = ZeroController(n_outputs=2)
#     
#     # Example state (doesn't affect the output)
#     example_state = np.array([1.0, -0.5])
#     example_target = np.array([0.0, 0.0])
#     
#     control_action = zero_ctrl.compute_control(example_state, target_state=example_target)
#     
#     print(f"Controller type: {type(zero_ctrl).__name__}")
#     print(f"Output dimension: {zero_ctrl._output_dim}")
#     print(f"Computed control for state {example_state} and target {example_target}: {control_action}")
#
#     # Example with target state during init (still ignored by ZeroController)
#     zero_ctrl_with_target = ZeroController(n_outputs=1, target_state=np.array([10.0]))
#     control_action_2 = zero_ctrl_with_target.compute_control(example_state)
#     print(f"Computed control with target in init: {control_action_2}")
