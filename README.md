# Seminar 7: Backstepping

This seminar focuses on the study and application of the backstepping method for designing nonlinear control systems. We will cover the theoretical foundations, Lyapunov function construction, and control law development for various systems.

## Learning Objectives:

*   Understand the theoretical principles of the backstepping method.
*   Be able to apply backstepping to systems in strict-feedback form.
*   Develop Lyapunov functions for stability analysis of closed-loop systems.
*   Implement backstepping algorithms in code.
*   Simulate and analyze the behavior of control systems designed using backstepping.
*   Adapt and utilize the existing simulation framework (`System`, `Controller`, `Simulator`, `Plotter`) for backstepping tasks.

## Current Focus:

Developing understanding and base implementations for backstepping. The existing framework supports general simulation; seminar-specific backstepping controllers are to be implemented.

## Directory Structure for `seminars/seminar_7_backstepping`:

*   `/src`: Contains the core components of the simulation framework (`system.py`, `controller.py`, `simulator.py`, `plotter.py`) and subdirectories for specific implementations:
    *   `systems/`: Implementations of systems (e.g., `PendulumMotorSystem`, `MotorSystem`, `Pendulum`). See `src/systems/README.md`.
    *   `controllers/`: Implementations of controllers (e.g., `PDController`, `PIController`, `EnergySwingUpController`; backstepping controllers are planned). See `src/controllers/README.md`.
*   `/scripts`: Executable Python scripts for each part of the seminar (e.g., `script_4_1_pendulum_backstepping_sim.py`). See `scripts/README.md`.
*   `/log`: CSV files with simulation logs. See `log/README.md`.
*   `/img`: PNG files with plots of results. See `img/README.md`.
*   `/theory`: Theoretical materials, derivation of equations, and stability analysis. See `theory/README.md`.

## Import Structure between `/scripts` and `/src`

It's important to understand how imports are organized for running the scripts:

1.  **Location of `src`**: The `src/` directory is located inside `seminars/seminar_7_backstepping/`.
2.  **Imports in scripts**: Scripts in `seminars/seminar_7_backstepping/scripts/` import components from `src/` using the `src.` prefix, e.g., `from src.simulator import Simulator` or `from src.systems.pendulum_motor_system import PendulumMotorSystem`.
3.  **`sys.path` Configuration**: For these imports to work, each script in `/scripts` **must** modify `sys.path` at the beginning, adding the path to `seminars/seminar_7_backstepping/`. This allows Python to find the `src` package.
4.  **Internal imports in `src`**: Within the `src` package itself, **relative imports** (e.g., `from .system import System` or `from .systems.pendulum import Pendulum`) are used.

**Example Run:** When running a script like `python seminars/seminar_7_backstepping/scripts/script_0_1_pendulum_zero_control.py` from the project's root directory, the script first adds `seminars/seminar_7_backstepping` to `sys.path` and then successfully executes `import src.module`.

## Key Files (Examples):

*   `scripts/script_4_1_pendulum_backstepping_sim.py`: Target script for implementing and simulating backstepping for a pendulum.
*   `src/systems/pendulum_motor_system.py` (`PendulumMotorSystem`): A key system for backstepping examples.
*   `src/controllers/README.md`: Outlines planned backstepping controllers (e.g., `PendulumBacksteppingController`).
*   `src/plotter.py` (`Plotter`): Used for visualizing simulation results.
*   `theory/4_1_backstepping_pendulum_stabilization.md`: Theoretical basis for a pendulum backstepping controller.
*   `theory/backstepping.md`: General theory of the backstepping method.

## How to Run (General Instructions):

1.  Ensure you have the necessary dependencies installed (numpy, matplotlib).
2.  Navigate to the root directory of the project (`classedu2025-advctrl`).
3.  Run the desired script from the project root directory, e.g.:
    ```bash
    python seminars/seminar_7_backstepping/scripts/script_4_1_pendulum_backstepping_sim.py
    ```
4.  Simulation results will be saved to:
    *   Log: `seminars/seminar_7_backstepping/log/log_SCRIPT_NAME.csv`
    *   Plot: `seminars/seminar_7_backstepping/img/img_SCRIPT_NAME.png`
    (where SCRIPT_NAME corresponds to the executed script, e.g., `4_1_pendulum_backstepping_sim`)

## Expected Results:

*   Plots should demonstrate the achievement of control objectives (e.g., state stabilization, trajectory tracking).
*   Lyapunov function analysis should confirm system stability for backstepping controllers.

# Instructions for LLM on Preparing for Seminar 7

**Goal:** Understand the project structure and dependencies to assist with Seminar 7 materials, focusing on Backstepping.

**Steps:**

1.  **Familiarize yourself with the project structure:**
    *   Read this `README.md` and the `README.md` files in subdirectories (`src/`, `scripts/`, `theory/`, etc.) for `seminars/seminar_7_backstepping`.
    *   Review the `README.md` of the project root and other relevant seminars/lectures for overall context.

2.  **Analyze the seminar's developing scripts:**
    *   Focus on scripts in `seminars/seminar_7_backstepping/scripts/`, especially those intended for backstepping (e.g., `script_4_1_pendulum_backstepping_sim.py`).
    *   Pay attention to each script's purpose, the classes used (systems, controllers, plotter, simulator), and the main execution loop.

3.  **Analyze script dependencies:**
    *   Carefully examine the `import` statements in the scripts.
    *   Pay special attention to imports from `seminars/seminar_7_backstepping/src/`.
    *   Study the code of the corresponding classes in `src/` (especially base `Controller`, `System`, and any planned backstepping controller structures) to understand their interfaces and logic.

**Outcome:** After completing these steps, you should have a good understanding of the project structure for Seminar 7, the logic of its scripts, and the implementation details of the components, enabling you to assist with developing and explaining backstepping controllers.

# Seminar 7: Simulation Framework (Focus on Backstepping)

This project contains materials for Seminar 7 (`seminars/seminar_7_backstepping`), focusing on the development and application of backstepping-based controllers using the provided simulation framework.

## Directory Structure (Reminder)

*   `/src`: Core Python classes for the framework and backstepping-specific implementations. See `src/README.md`.
*   `/scripts`: Executable Python scripts for each part of the seminar. See `scripts/README.md`.
*   `/theory`: Theoretical descriptions, derivation of equations for backstepping. See `theory/README.md`.
*   `/log`: `.csv` files with simulation results (logs). See `log/README.md`.
*   `/img`: `.png` image files (plots). See `img/README.md`.

## File Naming and Seminar Parts

The seminar will be divided into parts and sub-parts. Part names are used for naming related files.

*   **Part Name Format:** `PART_SUBPART_DESCRIPTION` (e.g., `4_1_pendulum_backstepping`).
*   **File Name Format:** Files related to one seminar part are named using the pattern: `PREFIX_PART_NAME.EXTENSION`.
    *   `PREFIX`: `script_` (for `/scripts`), `theory_` (for `/theory`), `log_` (for `/log`), `img_` (for `/img`).
    *   `EXTENSION`: `.py`, `.md`, `.csv`, `.png` respectively.

**Example:** For part `4_1_pendulum_backstepping_sim`:
*   Script: `scripts/script_4_1_pendulum_backstepping_sim.py`
*   Theory: `theory/4_1_backstepping_pendulum_stabilization.md` (or `theory_4_1_pendulum_backstepping_sim.md`)
*   Log: `log/log_4_1_pendulum_backstepping_sim.csv`
*   Plot: `img/img_4_1_pendulum_backstepping_sim.png`
*   Used classes from `src/`: `Simulator`, `PendulumMotorSystem`, a (planned) backstepping controller, `Plotter`.

**Important:** Log and image file names are **fixed** for each seminar part. Rerunning a script will overwrite these files.

**Important:** Before starting work, familiarize yourself with the code of the developing scripts in `seminars/seminar_7_backstepping/scripts/` and all their imports, especially the classes from `seminars/seminar_7_backstepping/src/`.

## Debugging Notes / Common Errors (General)

1.  **Incorrect `src` File Placement:** New system, controller, or plotter classes should be placed in the appropriate subdirectories (`src/systems/`, `src/controllers/`) or, for general utilities, in the `src/` root, not left there by mistake.
2.  **Import Errors:** `ImportError` can occur due to incorrect file placement or improper `sys.path` configuration. Scripts in `seminars/seminar_7_backstepping/scripts/` should add `seminars/seminar_7_backstepping/` to `sys.path` and use imports like `from src...`. Inside `src/`, relative imports should be used (e.g. `from .system import System`).
3.  **Logging Data Access:** The `Simulator` likely handles logging. Access to logged data for plotting is managed by the `Plotter` class, which would take simulation results (time, states, controls) as input.
4.  **Plotter Usage:** Refer to `src/plotter.py` for how to use the `Plotter` class.
5.  **LaTeX Issues in `matplotlib`:** Rendering errors might occur if LaTeX is not installed. It can be temporarily disabled via `plt.rcParams['text.usetex'] = False` in the relevant plotter class or globally in a script. 