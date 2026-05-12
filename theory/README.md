# Theoretical Materials (`theory/`)

This directory will contain Markdown (`.md`) files with theoretical explanations, formulas, and derivations for the corresponding parts and scripts of Seminar 7 (Backstepping), located in `../scripts/` (i.e., `seminars/seminar_7_backstepping/scripts/`).

## File Naming

The name of each theory file will correspond to the seminar part name (e.g., `1_1_simple_scalar_example`) with the prefix `theory_` and the extension `.md`.

**Example:** For the script `../scripts/script_4_1_pendulum_backstepping_sim.py`, the corresponding theory file might be `theory_4_1_pendulum_backstepping.md` or as available (e.g., `4_1_backstepping_pendulum_stabilization.md`).

## Content of Theory Files

Each theory file should typically include:

*   **Problem Statement:** Description of the control problem being solved in that part of the seminar.
*   **System Model:** Equations of motion for the system under consideration.
*   **Backstepping Design:** Step-by-step derivation of the control law using the backstepping procedure.
    *   Definition of virtual controls.
    *   Choice of Lyapunov function candidates for each step.
    *   Derivation of the actual control input.
*   **Stability Analysis:** Proof of stability (e.g., asymptotic stability) using the derived Lyapunov function for the overall closed-loop system.
*   **Connection to Code:** Pointers to how the theoretical concepts are implemented in the corresponding classes in `../src/` and used in the script in `../scripts/`.

## Formatting

*   Use Markdown syntax.
*   For mathematical formulas, use LaTeX syntax:
    *   `$ ... $` for inline formulas.
    *   `$$ ... $$` for display formulas.
    *   Use standard LaTeX commands (e.g., `\alpha`, `\gamma`, `\dot{x}`, `\hat{x}`, `\mathcal{L}`).

## Key Theory Files:

*   `backstepping.md`: General theoretical material on backstepping (from lecture).
*   `theory_0_1_pi_controller_stability_analysis.md`: Analysis of PI controller stability for a first-order motor model (corresponds to `script_0_3_motor_pi_control_meander.py`).
*   `theory_2_1_pd_controller_pendulum_stability.md`: Stability analysis for PD control of a pendulum (corresponds to `script_2_1_pendulum_pd_stability_sim.py`).
*   `theory_2_1_pd_controller_pendulum_stability_en.md`: English version of the PD controller stability analysis.
*   `4_1_backstepping_pendulum_stabilization.md`: Theory for backstepping-based stabilization of a pendulum (corresponds to `script_4_1_pendulum_backstepping_sim.py`). (Note: filename does not strictly follow `theory_X_Y` convention but content is relevant).

*   `(Planned) theory_1_1_simple_scalar_example.md`: Derivation of backstepping controller for a basic scalar nonlinear system.
*   `(Planned) theory_1_2_system_strict_feedback_form.md`: General theory for systems in strict-feedback form and Lyapunov stability proof.

*(This list will be updated as the seminar content is developed.)* 