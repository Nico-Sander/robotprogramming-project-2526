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

### 3. Open the Notebook
Navigate to `src/Flyby-Optimization-Interactive.ipynb` in the file explorer and open it.

### 4. Select the Kernel
To use the dependencies installed by `uv`, you must select the correct virtual environment:
1.  Click on **Select Kernel** (usually at the top-right of the notebook editor).
2.  Choose **Python Environments**.
3.  Select the environment marked as **.venv** (or `.venv/bin/python`). This environment was automatically created by the `uv sync` command.

### 5. Run the Optimization
You can now execute the cells. The notebook provides interactive widgets to visualize the environment and adjust parameters like radius ($r$) and asymmetry ($k$) in real-time.