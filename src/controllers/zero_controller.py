import numpy as np
from typing import List, Tuple, Optional
from ..controller import Controller # Import base Controller class

class ZeroController(Controller):
    """A controller that always outputs zero control input."""

    def __init__(self, n_outputs: int = 1, target_state: np.ndarray | None = None):
        """
        Initializes the ZeroController.

        Args:
            n_outputs: The dimension of the control output vector (default is 1).
            target_state: Optional target state, ignored by this controller but included for API consistency.
        """
        super().__init__(target_state=target_state) # Pass target_state up to the base class
        if not isinstance(n_outputs, int) or n_outputs < 1:
             raise ValueError("Number of outputs (n_outputs) must be a positive integer.")
        self._output_dim = n_outputs
        self._zero_output = np.zeros(n_outputs) # Pre-compute zero array
        # Define limits based on zero output - immutable zero
        self._limits = [(0.0, 0.0)] * n_outputs 

    def compute_control(self, current_state: np.ndarray, target_state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        """
        Returns a zero control input vector of the specified dimension. Ignores state and target.

        Args:
            current_state: The current state vector (ignored).
            target_state: The target state vector (ignored). If None, will use internal target if set, still ignored.
            t: The current time (ignored).

        Returns:
            A NumPy array of zeros with shape (n_outputs,).
        """
        # No need to check for target_state as it's ignored.
        return self._zero_output.copy() # Return a copy to prevent modification
        
    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        """Returns the control limits, which are always (0.0, 0.0) for each output."""
        if self._output_dim == 1:
             return self._limits[0] # Returns a tuple
        else:
             return self._limits # Returns a list of tuples 