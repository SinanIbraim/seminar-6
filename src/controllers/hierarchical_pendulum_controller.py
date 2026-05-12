import numpy as np
from ..controller import Controller
from .pi_controller import PIController
from .energy_swing_up_controller import EnergySwingUpController
from typing import Tuple, List, Optional

class HierarchicalPendulumController(Controller):
    """
    Hierarchical controller for a pendulum-motor system.
    Consists of:
    1. Upper level: EnergySwingUpController - determines the desired torque (tau_desired)
       for swinging the pendulum up to the upright position.
    2. Lower level: PIController - controls the motor torque (tau_m)
       to track tau_desired, by issuing a command 'a' to the motor.
    """
    def __init__(self, 
                 energy_controller: EnergySwingUpController,
                 pi_motor_controller: PIController,
                 pendulum_state_indices: Tuple[int, int] = (0, 1), # Indices of theta, theta_dot in the full state
                 motor_torque_state_index: int = 2, # Index of tau_m in the full state
                 name: str = "HierarchicalPendulumController"):
        """
        Initializes the hierarchical controller.

        Args:
            energy_controller (EnergySwingUpController): Instance of the energy swing-up controller.
            pi_motor_controller (PIController): Instance of the PI controller for motor torque control.
            pendulum_state_indices (Tuple[int, int]): Tuple of indices (theta, theta_dot)
                                                       in the full system state vector.
            motor_torque_state_index (int): Index of motor torque (tau_m)
                                             in the full system state vector.
            name (str): Name of the controller.
        """
        super().__init__(target_state=None)
        self.name = name
        self.energy_controller = energy_controller
        self.pi_motor_controller = pi_motor_controller
        
        if hasattr(pi_motor_controller, 'output_dim'):
            self.output_dim = pi_motor_controller.output_dim
        elif hasattr(pi_motor_controller, '_output_dim'): # Check for a private attribute if public not found
            self.output_dim = pi_motor_controller._output_dim
        else:
            print("Warning: HierarchicalPendulumController - failed to determine output_dim from pi_motor_controller. Set to 1.")
            self.output_dim = 1
            
        if not hasattr(energy_controller, 'pendulum_state_indices') or \
           energy_controller.pendulum_state_indices != pendulum_state_indices:
            print(f"Warning: HierarchicalPendulumController - pendulum_state_indices ({pendulum_state_indices}) "
                  f"differ from those in energy_controller ({getattr(energy_controller, 'pendulum_state_indices', 'N/A')}). "
                  f"Ensure energy_controller is configured to use these indices or receives an already sliced state.")
            # We will use the pendulum_state_indices of this class for slicing.
            # energy_controller should be initialized with the same indices or expect an already sliced [theta, theta_dot].
            # It's better if energy_controller.pendulum_state_indices matches what's passed here,
            # or if energy_controller.compute_control expects [s1,s2] already.
            # My energy_controller expects the full state and slices it itself using its pendulum_state_indices.
            # Therefore, it's important that they match. The EnergySwingUpController constructor should set them.

        self.pendulum_state_indices = pendulum_state_indices
        self.motor_torque_state_index = motor_torque_state_index

    def compute_control(self, current_state_full: np.ndarray, t: float | None = None) -> np.ndarray:
        """
        Computes the control signal 'a' for the motor.

        Args:
            current_state_full (np.ndarray): Full system state vector 
                                             [theta, theta_dot, motor_torque, ...].
            t (float | None): Current time.

        Returns:
            np.ndarray: Control signal [a] for the motor (shape (1,) or as defined in PIController).
        """
        # 1. Get desired torque from the energy swing-up controller
        # EnergySwingUpController.compute_control expects the full state vector
        # and will extract theta and theta_dot itself using its pendulum_state_indices.
        tau_desired_from_energy_ctrl = self.energy_controller.compute_control(current_state_full, t)
        # tau_desired_from_energy_ctrl should be of shape (1,)

        # 2. Get current motor torque from the full state
        current_motor_torque = np.array([current_state_full[self.motor_torque_state_index]])
        # current_motor_torque should be of shape (1,)

        # 3. Compute command 'a' using the PI controller
        # PIController.compute_control(current_state, target_state, t)
        motor_command_a = self.pi_motor_controller.compute_control(
            current_state=current_motor_torque, 
            target_state=tau_desired_from_energy_ctrl, # This should be np.array([value])
            t=t
        )
        
        return motor_command_a

    def reset(self):
        """ Resets the state of nested controllers. """
        if hasattr(self.energy_controller, 'reset'):
            self.energy_controller.reset()
        if hasattr(self.pi_motor_controller, 'reset_integral'): # PIController has reset_integral
            self.pi_motor_controller.reset_integral()
        elif hasattr(self.pi_motor_controller, 'reset'): # Fallback to generic reset if specific not found
            self.pi_motor_controller.reset()

    def get_control_limits(self) -> List[Tuple[Optional[float], Optional[float]]] | Tuple[Optional[float], Optional[float]]:
        """Returns the limits on the controller's output signal (command 'a').
           These limits are determined by the underlying motor PI controller.
        """
        # PIController should have a get_control_limits method
        if hasattr(self.pi_motor_controller, 'get_control_limits'):
            return self.pi_motor_controller.get_control_limits()
        else:
            # If PIController doesn't have such a method (which would be strange if it inherits from Controller)
            # Return undefined limits
            print("Warning: HierarchicalPendulumController - pi_motor_controller does not have get_control_limits method. Returning undefined limits.")
            if self.output_dim == 1:
                return (None, None)
            else:
                return [(None, None)] * self.output_dim

if __name__ == '__main__':
    # --- Parameters for example ---
    # Pendulum
    m_pend, l_pend, g_pend = 0.5, 0.5, 9.81
    u_max_swing_up = 1.5 # Max torque for swing-up
    # Motor
    gamma_motor = 0.05
    # PI controller (example gains)
    Kp_pi, Ki_pi, dt_pi = 10.0, 5.0, 0.01 

    # --- Create controller instances ---
    energy_ctrl = EnergySwingUpController(
        mass=m_pend, length=l_pend, gravity=g_pend, u_max=u_max_swing_up,
        pendulum_state_indices=(0, 1) # theta, theta_dot
    )

    # PI controller for the motor in the hierarchical scheme should be initialized 
    # without a static target, as the target will be dynamic.
    pi_ctrl = PIController(
        Kp=Kp_pi, Ki=Ki_pi, dt=dt_pi,
        static_target_state=None, # Explicitly None
        target_func=None          # And target_func=None
    )
    if not hasattr(pi_ctrl, 'output_dim'): # Defensive
        pi_ctrl.output_dim = 1 

    if hasattr(pi_ctrl, '_limits'): # For testing, try to set limits if attribute exists
        pi_ctrl._limits = [(-20.0, 20.0)] # Example limits for 'a'
        print(f"Test limits set for pi_ctrl: {pi_ctrl.get_control_limits()}")
    else:
        print("Warning: pi_ctrl does not have _limits attribute to set test limits.")


    hierarchical_ctrl = HierarchicalPendulumController(
        energy_controller=energy_ctrl,
        pi_motor_controller=pi_ctrl,
        pendulum_state_indices=(0, 1), # theta, theta_dot
        motor_torque_state_index=2    # motor_torque
    )
    print(f"HierarchicalController output_dim: {hierarchical_ctrl.output_dim}")
    print(f"Control limits for HierarchicalController: {hierarchical_ctrl.get_control_limits()}")

    # --- Example usage ---
    # Initial system state [theta, theta_dot, motor_torque]
    example_state_bottom = np.array([0.01, 0.0, 0.0]) # Pendulum at the bottom, at rest
    
    print(f"Testing HierarchicalPendulumController with state: {example_state_bottom}")
    
    # Call compute_control
    time_t = 0.0
    motor_command = hierarchical_ctrl.compute_control(example_state_bottom, time_t)
    
    print(f"Computed motor command 'a': {motor_command}")

    # Check tau_desired from energy_controller for this state
    tau_des_test = energy_ctrl.compute_control(example_state_bottom, time_t)
    print(f"Intermediate tau_desired from EnergyController: {tau_des_test}")
    # Expected u_max_swing_up = 1.5, as pendulum is at bottom and velocity is low.

    # PI controller should receive current_motor_torque = 0.0, target_state = tau_des_test.
    # Error e = tau_des_test[0] - 0.0 = 1.5
    # Integral error = 1.5 * dt_pi = 1.5 * 0.01 = 0.015
    # Control a = Kp * e + Ki * integral_e = 10.0 * 1.5 + 5.0 * 0.015
    #             = 15.0 + 0.075 = 15.075
    print(f"Expected 'a' value (manual calculation): Kp*e + Ki*int(e*dt) = {Kp_pi}*{tau_des_test[0]:.2f} + {Ki_pi}*({tau_des_test[0]:.2f}*{dt_pi:.2f}) = {Kp_pi*tau_des_test[0] + Ki_pi*tau_des_test[0]*dt_pi:.4f}")
    print(f"Integral error in PI after call: {pi_ctrl.get_integral_error()}") # Should be [0.015]

    hierarchical_ctrl.reset() # Reset integrator in PI
    print(f"Integral error in PI after reset: {pi_ctrl.get_integral_error()}") 