# Simulation Framework Components (`src/`)

This directory contains the core Python modules that make up the simulation framework used in Seminar 7, along with specific implementations for backstepping control.

## Core Components:

*   **`system.py`**: Abstract base class `System`.
    *   Defines the interface for dynamic systems.
*   **`systems/`**: Implementations of specific systems. For Seminar 7, this will include systems suitable for backstepping design. See `systems/README.md`.

*   **`controller.py`**: Abstract base class `Controller`.
    *   Defines the controller interface.
*   **`controllers/`**: Implementations of specific controllers. For Seminar 7, this will primarily house backstepping controllers and other standard controllers. See `controllers/README.md`.

*   **`simulator.py`**: Class `Simulator` that manages the simulation process.
    *   Coordinates interactions between systems and controllers.
    *   Executes simulation steps.
    *   Manages a logger (internally).

*   **`plotter.py`**: Contains the main `Plotter` class for visualizing simulation data, including states, controls, phase portraits, and energy plots.

## Import Structure and Directories

*   **Location of `src`**: This `src` directory is located *inside* the seminar folder (e.g., `seminars/seminar_7_backstepping/`).
*   **Internal Imports**: Within `src` modules, use **relative imports** to access other components of this package. E.g.:
    *   `from .system import System` (import from a file in the same directory).
    *   `from .systems import SpecificSystem` (import from a submodule like `systems`).
    *   `from ..controller import Controller` (example if a file in `systems` needed to import from `src` root - though direct parent imports `..` are less common for `main.py` like structures and more for sub-package organization).
*   **Standard Subdirectories**:
    *   Implementations of specific systems (`System`) are placed in `src/systems/`.
    *   Implementations of specific controllers (`Controller`) are placed in `src/controllers/`.
    *   Plotter implementations (`PlotterSubplots` derivatives) can reside directly in `src/` or in a separate subdirectory if needed.

These components are used by scripts in the `/scripts` directory to build and run simulations focusing on backstepping control.

## Debugging Notes / Common Errors (Components in `src`)

These notes are adapted from general framework usage and may apply here:

1.  **File Placement:** New classes (e.g., systems, controllers, plotters) should be placed in their respective subdirectories (`systems/`, `controllers/`) or directly in `src/` (for plotters), and not left in the `src/` root by mistake.
2.  **Imports:** Within `src` modules, use **relative imports** (`.`, `..`). Absolute imports like `from src...` will not work correctly here when scripts from outside try to import `src` as a package.
3.  **Logger Methods (`core.NamedVectorHistory`):** The method `get_last_record()` might be absent. To access the latest data, use the `variables` property (which returns a dictionary) and get the last element `[-1]` from the desired array (e.g., `logger.variables['my_var'][-1]`).
4.  **Plotter Methods (`plotter_subplots.py` and derivatives):**
    *   For data retrieval, use methods like `_get_variable_data(name)` (the exact name might vary based on the base class version).
    *   To finalize and save/show plots, there might not be a single `finalize_plot` method. Use `self._fig.tight_layout()`, and then `self.save_plot(filename)` or `plt.show()`.
    *   When inheriting from `PlotterSubplots`, ensure necessary method overrides (e.g., `setup_figure`) are correctly implemented if specific subplot arrangements are needed.
5.  **LaTeX Issues (`plotter_subplots.py`):** If rendering errors occur, check your LaTeX installation or temporarily disable LaTeX usage: `plt.rcParams['text.usetex'] = False`. 