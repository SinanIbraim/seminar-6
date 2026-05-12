import numpy as np
import os
import sys

# Add the seminar directory to sys.path to allow imports from src
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SEMINAR_DIR = os.path.dirname(SCRIPT_DIR) # This is seminar_7
if SEMINAR_DIR not in sys.path:
    sys.path.append(SEMINAR_DIR)

try:
    from src.systems.pendulum import Pendulum
    from src.controllers.zero_controller import ZeroController # Corrected import path
    from src.simulator import Simulator
except ImportError as e:
    print(f"Error importing modules: {e}")
    print(f"Ensure that the seminar directory '{SEMINAR_DIR}' is correct and contains 'src'.")
    print(f"And that 'pendulum.py', 'controller.py', 'simulator.py' are in their respective 'src' locations.")
    sys.exit(1)

def main():
    """
    Main function to set up and run the pendulum simulation with zero control.
    """
    print(f"Running script from: {SCRIPT_DIR}")
    print(f"Attempting to import from seminar root: {SEMINAR_DIR}")

    # Pendulum parameters
    mass = 1.0  # kg
    length = 1.0  # m
    damping = 0.2 # Nms/rad (friction coefficient)
    gravity = 9.81 # m/s^2
    
    # Initial state: [theta (angle from vertical), theta_dot (angular velocity)]
    # Start at pi/4 radians (45 degrees) with zero initial velocity
    initial_state_pendulum = np.array([np.pi / 4, 5.0]) 

    # Simulation parameters
    dt = 0.01  # time step in seconds
    total_time = 10.0  # total simulation time in seconds
    num_steps = int(total_time / dt)

    # --- Instantiate system and controller ---
    # The Pendulum class in src.systems.pendulum already handles its own state_names/units
    # or the plotter will use defaults if they are not found on the instance.
    # For more descriptive plots, one could either:
    # 1. Modify Pendulum.__init__ to call super().__init__(...) with state_names/units
    # 2. Manually assign pendulum_system.state_names and pendulum_system.state_units after creation.
    pendulum_system = Pendulum(
        mass=mass,
        length=length,
        damping=damping,
        gravity=gravity,
        initial_state=initial_state_pendulum
    )
    # Manually setting state names and units for better plotting, as Pendulum doesn't use System's __init__
    pendulum_system.state_names = ["theta", "thetadot"]
    pendulum_system.state_units = ["rad", "rad/s"]
    pendulum_system.name = "Damped Pendulum (Zero Control)" # Give a name for the plot title


    # Zero controller (1 output for torque)
    zero_ctrl = ZeroController(n_outputs=1)

    # --- Instantiate Simulator ---
    simulator = Simulator(
        system=pendulum_system,
        controller=zero_ctrl,
        dt=dt,
        num_steps=num_steps
    )

    # --- Run Simulation ---
    print("Starting simulation...")
    simulator.run()
    print("Simulation finished.")

    # --- Plot Results ---
    # Define path to save the plot
    img_dir = os.path.join(SEMINAR_DIR, "img")
    if not os.path.exists(img_dir):
        os.makedirs(img_dir)
    plot_save_path = os.path.join(img_dir, "img_0_1_pendulum_zero_control.png")
    
    print(f"Plotting results. Saving to: {plot_save_path}")
    # The simulator's plot_results uses the Plotter class internally.
    # We can provide control_names and control_units for clarity in plots.
    simulator.plot_results(
        save_path=plot_save_path,
        control_names=["Torque"],
        control_units=["Nm"]
    )
    print("Script finished.")

if __name__ == "__main__":
    main() 