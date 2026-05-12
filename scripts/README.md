# Seminar 7 Scripts (`scripts/`)

This directory contains executable Python scripts demonstrating the use of the simulation framework (`../src`) for designing, simulating, and analyzing control systems, with a focus on the backstepping technique for Seminar 7.

## Available Scripts (by Seminar Parts):

*   `script_0_1_pendulum_zero_control.py`: Demonstration of pendulum behavior without control.
*   `script_0_2_motor_meander_target.py`: Demonstration of a meander-type target value generator for the motor system.
*   `script_0_3_motor_pi_control_meander.py`: PI controller for the motor, tracking a meander-type trajectory.
*   `script_1_1_pendulum_swing_up_manual_sim.py`: Manual control for pendulum swing-up and stabilization.
*   `script_2_1_pendulum_pd_stability_sim.py`: PD controller for stabilizing the pendulum in the upright position.
*   `script_3_1_pendulum_swing_up_pd_sim.py`: Combined control (energy-based for swing-up and PD for stabilization) of the pendulum.
*   `script_4_1_pendulum_backstepping_sim.py`: Pendulum control using the backstepping method (or preparation for its use).

*(This list reflects the current scripts. New scripts, especially those implementing backstepping, will be added as the seminar progresses.)*

## How to Run:

1.  Navigate to the root directory of the project (`classedu2025-advctrl`).
2.  Execute the desired script (ensure the path reflects your current seminar directory, e.g., `seminars/seminar_7_backstepping`):
    ```bash
    # Example for a script in this seminar:
    python seminars/seminar_7_backstepping/scripts/script_0_1_pendulum_zero_control.py
    ```
3.  Logs (`.csv`) will be saved in `../log` (i.e., `seminars/seminar_7_backstepping/log/`).
4.  Plots (`.png`) will be saved in `../img` (i.e., `seminars/seminar_7_backstepping/img/`).

## General Script Structure:

Scripts will typically follow this pattern:
1.  Modify `sys.path` to include the seminar directory.
2.  Import necessary components from `../src` (e.g., `Simulator`, `SystemBuilder`, specific systems and controllers).
3.  Define simulation parameters (time, step, initial conditions).
4.  Build the system and controller instances.
5.  Configure the simulator with the system, controller, and logger.
6.  Run the simulation loop.
7.  Generate plots using a dedicated plotter class. 