import numpy as np
from ..system import System # Using relative import for base class

class PendulumMotorSystem(System):
    """
    Model of a pendulum system driven by a first-order motor.
    State: [theta, theta_dot, motor_torque]
    theta (s1): pendulum angle from vertical downwards (rad)
    theta_dot (s2): pendulum angular velocity (rad/s)
    motor_torque (s3): torque developed by the motor (Nm)

    Control (a): command to the motor (can be voltage or an abstract quantity
                     affecting the rate of change of motor torque).
    """
    def __init__(self, 
                 initial_state: np.ndarray, 
                 mass: float = 1.0, 
                 length: float = 1.0, 
                 gravity: float = 9.81,
                 motor_gamma: float = 0.1, # Motor time constant
                 name: str = "PendulumMotorSystem"):
        """
        Initializes the pendulum system with a motor.

        Args:
            initial_state (np.ndarray): Initial state [theta, theta_dot, motor_torque].
            mass (float): Mass of the pendulum (kg).
            length (float): Length of the pendulum to the center of mass (m).
            gravity (float): Acceleration due to gravity (m/s^2).
            motor_gamma (float): Motor time constant (s). gamma in the equation tau_dot = 1/gamma * (-tau + a).
            name (str): Name of the system.
        """
        super().__init__(initial_state=initial_state, name=name)
        
        if not isinstance(initial_state, np.ndarray) or initial_state.shape != (3,):
            raise ValueError("initial_state must be a NumPy array of shape (3,).")
        if mass <= 0: raise ValueError("mass must be > 0.")
        if length <= 0: raise ValueError("length must be > 0.")
        if gravity <= 0: raise ValueError("gravity must be > 0.")
        if motor_gamma <= 0: raise ValueError("motor_gamma must be > 0.")

        self.m = mass
        self.l = length
        self.g = gravity
        self.J = mass * length**2  # Moment of inertia of the pendulum about the pivot point
        self.motor_gamma = motor_gamma

        # Defining attributes directly, not via _private and @property
        self.state_dim = 3
        self.input_dim = 1
        self.state_names = [r'Angle $\theta$', r'Ang. vel. $\dot{\theta}$', r'Motor Torque $\tau_m$']
        self.state_units = ['rad', 'rad/s', 'Nm']
        self.control_input_names = [r'Motor Command $a$']
        self.control_input_units = ['Nm (effective)']

    def get_state_derivative(self, t: float, state: np.ndarray, control_input: np.ndarray) -> np.ndarray:
        """
        Right-hand side of the system of differential equations.
        dot_state = f(t, state, control_input)

        Args:
            t (float): Current time.
            state (np.ndarray): Current state [theta, theta_dot, motor_torque].
            control_input (np.ndarray): Control signal [a].

        Returns:
            np.ndarray: State derivatives [d_theta_dt, d_theta_dot_dt, d_motor_torque_dt].
        """
        s1_theta, s2_theta_dot, s3_motor_torque = state
        
        a_val = 0.0 # default value for 'a'
        if control_input is None:
            a_val = 0.0 
        elif isinstance(control_input, (int, float)):
             a_val = float(control_input)
        elif isinstance(control_input, np.ndarray):
            if control_input.ndim == 0: # Scalar ndarray
                a_val = float(control_input)
            elif control_input.shape == (1,):
                a_val = control_input[0]
            else:
                raise ValueError(f"Incorrect control_input shape: {control_input.shape}, expected scalar or (1,).")
        else:
            raise TypeError(f"Incorrect control_input type: {type(control_input)}.")

        ds1_dt = s2_theta_dot
        ds2_dt = (-self.g / self.l * np.sin(s1_theta)) + (s3_motor_torque / self.J)
        ds3_dt = (1 / self.motor_gamma) * (-s3_motor_torque + a_val)
        
        return np.array([ds1_dt, ds2_dt, ds3_dt])

if __name__ == '__main__':
    # Example usage
    initial_pendulum_state = np.array([0.1, 0.0, 0.0]) # Small deviation, zero velocity, zero initial torque
    pendulum_sys = PendulumMotorSystem(
        initial_state=initial_pendulum_state,
        mass=0.5,
        length=0.5,
        motor_gamma=0.05
    )

    print(f"System: {pendulum_sys.name}")
    print(f"Initial state: {pendulum_sys.current_state}")
    print(f"State dimension: {pendulum_sys.state_dim}")
    print(f"Input dimension: {pendulum_sys.input_dim}")
    print(f"State names: {pendulum_sys.state_names}")
    print(f"State units: {pendulum_sys.state_units}")
    print(f"Control input names: {pendulum_sys.control_input_names}")
    print(f"Control input units: {pendulum_sys.control_input_units}")

    # Example of calculating the right-hand side
    time = 0.0
    current_s = pendulum_sys.current_state
    control_a = np.array([0.5]) # Example control input
    
    derivatives = pendulum_sys.get_state_derivative(time, current_s, control_a)
    print(f"State derivatives with control a={control_a[0]}: {derivatives}")

    derivatives_no_control = pendulum_sys.get_state_derivative(time, current_s, np.array([0.0]))
    print(f"State derivatives without control (a=0): {derivatives_no_control}") 