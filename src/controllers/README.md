# Controller Implementations (`src/controllers/`)

This directory will contain implementations of various control algorithms, inheriting from the base `Controller` class (`../../controller.py`). For Seminar 7, the focus will be on controllers designed using the backstepping methodology.

## Implemented Controllers (General Purpose):

*   `zero_controller.py` (`ZeroController`): Controller that always outputs zero control input.
*   `proportional_controller.py` (`ProportionalController`): Proportional controller (P-controller).
*   `pi_controller.py` (`PIController`): Proportional-Integral controller (PI-controller).
*   `pd_controller.py` (`PDController`): Proportional-Differential controller (PD-controller).
*   `energy_swing_up_controller.py` (`EnergySwingUpController`): Controller for energy-based swing-up of the pendulum.
*   `hierarchical_pendulum_controller.py` (`HierarchicalPendulumController`): Hierarchical controller for the pendulum, using `EnergySwingUpController` and `PIController`.

## Planned Controller Implementations (Focus on Backstepping for Seminar 7):

*   **`backstepping_controller_template.py` (`BacksteppingControllerTemplate`)**: (Planned) Base or template class for standard backstepping controllers.
*   **`scalar_backstepping_controller.py` (`ScalarBacksteppingController`)**: (Planned) Controller for a simple scalar nonlinear system.
*   **`strict_feedback_backstepping.py` (`StrictFeedbackBacksteppingController`)**: (Planned) Controller for systems in strict-feedback form.
*   **`pendulum_backstepping_controller.py` (`PendulumBacksteppingController`)**: (Planned) Backstepping controller for the nonlinear inverted pendulum (likely for `PendulumMotorSystem`).
*   **`adaptive_backstepping_controller.py` (`AdaptiveBacksteppingController`)**: (Planned) Adaptive backstepping for systems with unknown parameters.

*(Other controller variants, especially related to backstepping, will be added as Seminar 7 progresses.)*

## General Characteristics:

*   Each controller class will implement the `compute_control(t, x, target_state, system_state_for_control)` method to calculate the control input.
*   They may also implement `update_state(t, x_system, target_state)` if the controller has its own internal states (e.g., in adaptive control or with filters).
*   The implementation will clearly reflect the step-by-step design procedure of backstepping.

## Usage:

Controller instances will be created and configured in the simulation scripts (`../../scripts/`) and then passed to the `Simulator` to generate control actions for the simulated systems. 