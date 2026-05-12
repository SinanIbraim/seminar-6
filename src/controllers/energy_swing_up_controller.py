import numpy as np
from ..controller import Controller # Using relative import for base class
from typing import Tuple, List, Optional # Added List, Optional

class EnergySwingUpController(Controller):
    """
    Controller for energy-based swing-up of a pendulum.
    Calculates the desired torque (tau_desired) that should be applied to the pendulum
    to reach the energy of the upright equilibrium position.
    """
    def __init__(self, 
                 mass: float, 
                 length: float, 
                 gravity: float, 
                 u_max: float, 
                 pendulum_state_indices: Tuple[int, int] = (0, 1),
                 s2_deadband_threshold: float = 1e-3, # Threshold for defining "almost zero" velocity
                 energy_target_factor: float = 1.0, # Multiplier for target energy (e.g., 1.0 for exact energy, >1 for margin)
                 name: str = "EnergySwingUpController"):
        """
        Initializes the energy swing-up controller.

        Args:
            mass (float): Mass of the pendulum (kg).
            length (float): Length of the pendulum to the center of mass (m).
            gravity (float): Acceleration due to gravity (m/s^2).
            u_max (float): Maximum desired torque that this controller can output (Nm).
            pendulum_state_indices (Tuple[int, int]): Indices to extract theta and theta_dot
                                                     from the full system state vector.
            s2_deadband_threshold (float): Threshold for angular velocity, below which it is considered "zero"
                                          for the "push-start" logic.
            energy_target_factor (float): Multiplier for the target energy E_upright.
                                          A value > 1.0 can help "overshoot" the threshold.
            name (str): Name of the controller.
        """
        super().__init__(target_state=None) # CHANGED
        self.name = name # Set name after super()
        
        if mass <= 0: raise ValueError("mass must be > 0.")
        if length <= 0: raise ValueError("length must be > 0.")
        if gravity <= 0: raise ValueError("gravity must be > 0.")
        if u_max <= 0: raise ValueError("u_max must be > 0.")
        if not (isinstance(pendulum_state_indices, tuple) and len(pendulum_state_indices) == 2 and 
                all(isinstance(i, int) for i in pendulum_state_indices)):
            raise ValueError("pendulum_state_indices must be a tuple of two integers.")

        self.m = mass
        self.l = length
        self.g = gravity
        self.J = mass * length**2  # Moment of inertia
        self.u_max = u_max
        self.pendulum_state_indices = pendulum_state_indices
        self.s2_deadband_threshold = s2_deadband_threshold

        # Energy at the bottom position (theta=0, theta_dot=0) -> V=0, E_down=0
        # Energy at the upright position (theta=pi, theta_dot=0) -> V=2mgl, E_upright=2mgl
        self.E_target_upright_nominal = 2 * self.m * self.g * self.l
        self.E_target_operational = self.E_target_upright_nominal * energy_target_factor
        
        self.output_dim = 1 # Set attribute directly
        # Store u_max for get_control_limits
        self._control_limits_internal = [(-abs(u_max), abs(u_max))] 

    def _calculate_energy(self, s1_theta: float, s2_theta_dot: float) -> float:
        """ Calculates the current mechanical energy of the pendulum. """
        # Potential energy V = mgl(1 - cos(theta)), V(0)=0, V(pi)=2mgl
        # Kinetic energy T = 0.5 * J * theta_dot^2
        potential_energy = self.m * self.g * self.l * (1 - np.cos(s1_theta))
        kinetic_energy = 0.5 * self.J * s2_theta_dot**2
        return potential_energy + kinetic_energy

    def compute_control(self, current_state_full: np.ndarray, t: float | None = None) -> np.ndarray:
        """
        Calculates the desired torque for swinging up the pendulum.

        Args:
            current_state_full (np.ndarray): Full system state vector from which
                                             theta and theta_dot will be extracted.
            t (float | None): Current time (not used by this controller).

        Returns:
            np.ndarray: Desired torque [tau_desired] (shape (1,)).
        """
        s1_theta = current_state_full[self.pendulum_state_indices[0]]
        s2_theta_dot = current_state_full[self.pendulum_state_indices[1]]

        current_energy = self._calculate_energy(s1_theta, s2_theta_dot)
        
        desired_torque = 0.0 # Default torque is 0 (for hysteresis zone)

        E_op = self.E_target_operational
        lower_bound = 0.97 * E_op
        upper_bound = 1.03 * E_op

        energy_error = E_op - current_energy # Positive if energy is low; negative if high

        if current_energy < lower_bound: # Energy is too low - NEED TO INCREASE
            if abs(s2_theta_dot) < self.s2_deadband_threshold:
                # Velocity is low, pendulum might have stopped (e.g., at the bottom or at the peak of an incomplete swing-up)
                # Give a "kick" with maximum effort to get it moving.
                # The direction of the kick u_max should be such as to increase energy.
                # If it's at the bottom (cos(theta) ~ 1), any u_max push will start motion.
                # If it's at a peak, u_max should also help.
                # Original logic here always gave self.u_max, this seems reasonable for a "kick".
                desired_torque = self.u_max 
            else:
                # Velocity is sufficient, pump energy: u = u_max * sign(s2 * error_E)
                # Since error_E is positive here, u = u_max * sign(s2)
                desired_torque = self.u_max * np.sign(s2_theta_dot)
        
        elif current_energy > upper_bound: # Energy is too high - NEED TO DECREASE
            if abs(s2_theta_dot) < self.s2_deadband_threshold:
                # Energy is high, but velocity is almost zero (e.g., pendulum stuck at the top).
                # Active braking here might be excessive or lead to jerks.
                # Safer not to apply torque.
                desired_torque = 0.0
            else:
                # Velocity is sufficient, actively brake: u = u_max * sign(s2 * error_E)
                # Since error_E is negative here, u = u_max * sign(s2 * (-1)) = -u_max * sign(s2)
                # This means torque opposite to the direction of motion s2.
                desired_torque = -self.u_max * np.sign(s2_theta_dot)
        
        # If current_energy is in the range [lower_bound, upper_bound], 
        # then desired_torque remains 0.0 (hysteresis applies).
            
        return np.array([desired_torque])

    def reset(self):
        # This controller has no internal state to reset (apart from parameters)
        pass

    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        """Returns the limits on the controller's output signal (desired torque)."""
        # Since output_dim = 1, return a tuple for a single output
        return self._control_limits_internal[0]

if __name__ == '__main__':
    # Example usage
    swing_up_ctrl = EnergySwingUpController(
        mass=0.5, 
        length=0.5, 
        gravity=9.81, 
        u_max=1.0, # Maximum desired torque for swing-up
        pendulum_state_indices=(0,1) # theta at 0th position, theta_dot at 1st
    )
    print(f"Control limits for EnergySwingUpController: {swing_up_ctrl.get_control_limits()}")

    # Example state: pendulum at the bottom, at rest
    # [theta, theta_dot, some_other_state]
    state_at_bottom = np.array([0.0, 0.0, 0.0]) 
    tau_des_bottom = swing_up_ctrl.compute_control(state_at_bottom)
    print(f"State: {state_at_bottom}, Desired torque: {tau_des_bottom}")
    # Expected: u_max, as s2 is close to zero and energy is low (0 < E_target)

    # Example state: pendulum moving upwards
    state_moving_up = np.array([np.pi/2, 1.0, 0.0])
    tau_des_moving_up = swing_up_ctrl.compute_control(state_moving_up)
    print(f"State: {state_moving_up}, Desired torque: {tau_des_moving_up}")
    # Expected: u_max (sign(s2_theta_dot)=1)

    # Example state: pendulum moving downwards
    state_moving_down = np.array([np.pi/2, -1.0, 0.0])
    tau_des_moving_down = swing_up_ctrl.compute_control(state_moving_down)
    print(f"State: {state_moving_down}, Desired torque: {tau_des_moving_down}")
    # Expected: -u_max (sign(s2_theta_dot)=-1)

    # Example state: energy reached (hypothetically)
    # Let E_target = 2*0.5*9.81*0.5 = 4.905
    # If theta=pi, theta_dot=0, E = 2*m*g*l. 
    state_energy_high = np.array([np.pi, 0.0001, 0.0]) # Almost at the top
    # Energy will be close to 2mgl
    print(f"Target energy (nominal): {swing_up_ctrl.E_target_upright_nominal:.3f}")
    print(f"Energy for state_energy_high: {swing_up_ctrl._calculate_energy(state_energy_high[0], state_energy_high[1]):.3f}")
    tau_des_energy_high = swing_up_ctrl.compute_control(state_energy_high)
    print(f"State: {state_energy_high}, Desired torque: {tau_des_energy_high}")
    # Expected: 0.0, as energy should be >= operational target

    # Check with energy_target_factor > 1
    swing_up_ctrl_factor = EnergySwingUpController(
        mass=0.5, length=0.5, gravity=9.81, u_max=1.0, energy_target_factor=1.1
    )
    print(f"Control limits for factor controller: {swing_up_ctrl_factor.get_control_limits()}")
    print(f"Target energy (operational factor=1.1): {swing_up_ctrl_factor.E_target_operational:.3f}")
    tau_des_factor = swing_up_ctrl_factor.compute_control(state_energy_high) # Use the same state as above
    print(f"State: {state_energy_high} (with factor 1.1), Desired torque: {tau_des_factor}")
    # Now, if nominal energy was reached, but operational (with factor) not yet,
    # then control might not be zero.
    # In this case, state_energy_high has energy ~4.905. E_op ~ 4.905*1.1 = 5.395. So, control will be u_max. 