# Path Optimization - Fly-by Smoothing

This project implements a G1-continuous path smoothing algorithm (Fly-by / Rounding) for a 2-DoF robot using Quadratic Bezier Curves. It includes an interactive dashboard to explore smoothing parameters like radius ($r$) and asymmetry ($k$).

## Setup and Installation

This project manages its dependencies and virtual environments using **[uv](https://github.com/astral-sh/uv)**, an extremely fast Python package installer and resolver.

### 1. Prerequisites

You must have `uv` installed on your system.

**macOS / Linux:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**Windows:**
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

For other installation methods (Homebrew, Pip, etc.), please refer to the [official uv documentation](https://github.com/astral-sh/uv).

### 2. Installation

Clone the repository and sync the dependencies. This will automatically create a virtual environment and install the exact versions defined in `uv.lock`.

```bash
# Clone the repository
git clone https://github.com/Nico-Sander/robotprogramming-project-2526.git
```

```bash
# Enter the repository
cd robotprogramming-project-2526
```

```shell
# Sync dependencies
uv sync
```

## Usage

This project is designed to be explored interactively using **Visual Studio Code**.

### 1. Open the Project
Open the cloned folder `robotprogramming-project-2526` in VS Code.

### 2. Install Recommended Extensions
To run the notebooks, ensure you have the following extensions installed:
* **Python** (`ms-python.python`)
* **Jupyter** (`ms-toolsai.jupyter`)

### 3. Open a Notebook
Navigate to the `notebooks/` folder in the file explorer. There are three notebooks available depending on your needs:

* **`Flyby-Optimization-Interactive.ipynb`** (Recommended)
    The main interactive dashboard. It allows you to select environments, perform parameter sweeps on Radius ($r$) and Asymmetry ($k$), and visualize the results instantly without clutter.
* **`Flyby-Optimization-Interactive-with-documentation.ipynb`**
    Contains the same interactive tools as above, but includes markdown documentation and explanations of the underlying algorithms inline.
* **`Flyby-Optimization.ipynb`**
    The fundamental implementation and step-by-step execution of the optimization logic without the UI widgets.

### 4. Select the Kernel
To use the dependencies installed by `uv`, you must select the correct virtual environment:
1.  Click on **Select Kernel** (usually at the top-right of the notebook editor).
2.  Choose **Python Environments**.
3.  Select the environment marked as **.venv** (or `.venv/bin/python`). This environment was automatically created by the `uv sync` command.

### 5. Run the Optimization
You can now execute the cells. The notebooks provide widgets to visualize the environment and adjust parameters in real-time.

## Project Structure

The project logic is modularized within the `src/` directory to separate algorithmic logic, visualization, and data generation.

### Core Modules (`src/`)

* **`optimize_path.py`**
    Contains the core **`OptimizeFlyby`** class. It implements:
    * The **Inverse Rounding** logic to calculate virtual control points ($P_{2n}$).
    * The iterative relaxation loop to propagate changes across neighbors.
    * The collision handling strategy (radius reduction) and the global/individual $k$ optimization algorithms.

* **`collision_checker.py`**
    Handles all geometric validation. It interfaces with `shapely` to check intersections for points, lines, and curves (via discretization). It also contains the **Visualization** logic (using `matplotlib`) to draw environments, paths, and optimization artifacts.

* **`interactive_widgets.py`**
    Manages the Jupyter Notebook UI. It implements a caching system to pre-calculate parameter sweeps (e.g., testing all radii values) to ensure the sliders respond instantly without re-running the heavy planner logic during interaction.

* **`benchmarks.py`**
    Defines the standard testing environments (1 through 4). It includes utilities like `fillet_corner_if_exists` to procedurally generate rounded obstacles for testing.

* **`evaluation.py`**
    Provides metrics and plotting tools. It captures performance data (execution time, collision check counts) and calculates the physical length of the optimized G1-continuous paths.

### Helper Modules (`src/planners/`)

* **`IPBasicPRM.py`**: A basic Probabilistic Roadmap planner used to generate the initial collision-free path.
* **`IPPerfMonitor.py`**: A decorator utility for profiling function calls and execution time.


## Task and Documentation

The underlying Task can be seen in `Documenation/Task.md` and the whole Documentation in `Documentation/Documentation.md`