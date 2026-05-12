# System Implementations (`src/systems/`)

This directory will house implementations of various dynamic systems, inheriting from the base `System` class (`../../system.py`). For Seminar 7, these systems will be chosen or designed to be suitable for the application of backstepping control techniques.

## Implemented Systems:

*   `motor_system.py` (`MotorSystem`): First-order motor model.
    *   State: `[tau]` (torque).
    *   Control: `[u]` (torque command).

*   `pendulum.py` (`Pendulum`): Classic mathematical pendulum model.
    *   State: `[theta, theta_dot]` (angle and angular velocity).
    *   Control: `tau` (torque).

*   `pendulum_motor_system.py` (`PendulumMotorSystem`): Model of a pendulum system driven by a first-order motor.
    *   State: `[theta, theta_dot, motor_torque]` (pendulum angle, angular velocity, motor torque).
    *   Control: `a` (motor command).

## Planned System Implementations (for Backstepping Focus):

*   **`strict_feedback_system.py` (`StrictFeedbackSystem`)**: (Planned) A generic template or a specific example of a system in strict-feedback form.
*   **`higher_order_integrator.py` (`HigherOrderIntegratorChain`)**: (Planned) A chain of integrators.

*(Other systems may be added as needed for examples or assignments in Seminar 7.)*

## General Characteristics:

*   Each system class will define its state variables, parameters, and the method `get_state_derivative(t, x, u)` or `get_state_at_time(t, u)` as appropriate.
*   They will be designed to clearly show the structure for which backstepping is applicable (e.g., lower-triangular form, strict-feedback form).

## Usage:

Instances of these system classes will be created in the simulation scripts (`../../scripts/`) and used by the `Simulator` to model the plant dynamics. 