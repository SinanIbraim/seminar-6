from abc import ABC, abstractmethod
import numpy as np

class System(ABC):
    def __init__(self, name: str, initial_state: np.ndarray, 
                 state_names: list[str] | None = None, 
                 state_units: list[str] | None = None):
        """
        Initializes the system.

        Args:
            name: The name of the system (e.g., "Inverted Pendulum", "Mass-Spring").
            initial_state: The initial state vector of the system.
            state_names: Optional list of names for each state variable. 
                         If None, defaults to ["state_0", "state_1", ...].
            state_units: Optional list of units (strings) for each state variable.
                         If provided, must match the length of initial_state.
        """
        if not name:
            raise ValueError("System name cannot be empty.")
        if not isinstance(initial_state, np.ndarray):
            raise TypeError("Initial state must be a NumPy array.")

        num_states = len(initial_state)

        if state_names is None:
            state_names = [f"state_{i}" for i in range(num_states)]
        else:
            if not isinstance(state_names, list) or not all(isinstance(s, str) for s in state_names):
                raise TypeError("state_names must be a list of strings.")
            if len(state_names) != num_states:
                raise ValueError(f"Length of state_names ({len(state_names)}) must match length of initial_state ({num_states}).")

        # Validate state_units if provided
        if state_units is not None:
            if not isinstance(state_units, list) or not all(isinstance(u, str) for u in state_units):
                raise TypeError("state_units must be a list of strings.")
            if len(state_units) != num_states:
                raise ValueError(f"Length of state_units ({len(state_units)}) must match length of initial_state ({num_states}).")

        self.name = name
        self._state = initial_state.copy() # Store state internally, use copy
        self.state_names = state_names # Guaranteed to be a list of strings
        self.state_units = state_units # List of strings or None

    def get_num_states(self) -> int:
        """Returns the number of state variables."""
        return len(self._state)

    def get_state(self) -> np.ndarray:
        """Returns the current internal state vector."""
        return self._state.copy() # Return a copy to prevent external modification

    def set_state(self, state: np.ndarray):
        """Sets the internal state vector.

        Args:
            state: The new state vector.
        """
        if not isinstance(state, np.ndarray):
            raise TypeError("State must be a NumPy array.")
        if state.shape != self._state.shape:
            raise ValueError(f"New state shape {state.shape} must match existing state shape {self._state.shape}.")
        self._state = state.copy() # Store a copy

    @abstractmethod
    def get_state_derivative(self, control_input: np.ndarray, state: np.ndarray | None = None, t: float | None = None) -> np.ndarray:
        """Computes the derivative of the state vector."""
        pass

    def step(self, dt: float, control_input: np.ndarray):
        """
        Performs one simulation step using the forward Euler method.
        Default implementation: state_new = state_old + get_state_derivative(control_input, state_old) * dt.
        Override this method for different integration schemes (e.g., RK4).

        Args:
            dt: Time step.
            control_input: Control input vector at the current step.
        """
        current_state = self.get_state()
        state_derivative = self.get_state_derivative(control_input, state=current_state) # Pass current state explicitly
        new_state = current_state + state_derivative * dt
        self.set_state(new_state)

    def get_kinetic_energy(self, state: np.ndarray | None = None) -> float:
        """
        Computes the kinetic energy of the system.
        Default implementation: 0.5 * state[1]**2.
        Assumes the second element of the state vector is velocity-like.
        Override this method for system-specific calculations.

        Args:
            state: State vector [x, x_dot]. If None, uses the current state.

        Returns:
            Kinetic energy.
        """
        if state is None:
            state = self.get_state()
        if len(state) < 2:
            raise ValueError("State vector must have at least 2 elements for default kinetic energy calculation.")
        return 0.5 * state[1]**2

    def get_potential_energy(self, state: np.ndarray | None = None) -> float:
        """
        Computes the potential energy of the system.
        Default implementation: 0.5 * state[0]**2.
        Assumes the first element of the state vector is position-like.
        Override this method for system-specific calculations.

        Args:
            state: State vector [x, x_dot]. If None, uses the current state.

        Returns:
            Potential energy.
        """
        if state is None:
            state = self.get_state()
        if len(state) < 1:
            raise ValueError("State vector must have at least 1 element for default potential energy calculation.")
        return 0.5 * state[0]**2

    def get_energy(self, state: np.ndarray | None = None) -> float:
        """
        Computes the total energy of the system.
        Default implementation: sum of default kinetic and potential energy.
        Override this method if total energy is not a simple sum or uses different definitions.

        Args:
            state: State vector. If None, uses the current state.

        Returns:
            Total energy.
        """
        if state is None:
            # Get state once to pass to both methods if state is initially None
            state = self.get_state()
        return self.get_kinetic_energy(state) + self.get_potential_energy(state) 