import itertools
import ipywidgets as widgets
from IPython.display import display
import matplotlib.pyplot as plt
import numpy as np

from evaluation import (
    retrieve_path_positions,
    clear_graph_attributes,
    capture_performance_metrics,
    plot_performance_data,
    calculate_path_length
)
from planners.IPPerfMonitor import IPPerfMonitor

def create_interactive_viewer(param_config: dict, render_callback):
    """
    Constructs a generic, cached interactive dashboard for exploring parameter sweeps in Jupyter Notebooks.

    This utility solves the latency problem inherent in heavy rendering callbacks (like path optimization 
    and matplotlib plotting) by pre-calculating all possible states. It works in three phases:
    1. **Pre-computation**: It generates the Cartesian product of all provided parameter lists in `param_config` 
       and executes the `render_callback` for every combination.
    2. **Caching**: The visual output (plots, prints) of each execution is captured into a `widgets.Output` 
       object and stored in a dictionary keyed by the parameter tuple.
    3. **UI Generation**: It dynamically creates control widgets (Sliders or Dropdowns) based on the input data types 
       and links them to an observer that swaps the displayed output widget instantly upon user interaction.

    This approach front-loads the computation time, ensuring that the interactive experience is buttery smooth 
    without re-running the heavy planner logic during exploration.

    Parameters:
        param_config (dict): A dictionary where keys are parameter names (strings) and values are lists of 
                             possible values to sweep (e.g., `{'Radius': [0.1, 0.5], 'Env': ['1', '2']}`).
        render_callback (callable): A function that accepts keyword arguments matching the keys in `param_config`. 
                                    It should perform the plotting/printing.

    Returns:
        None: The function displays the widgets directly using IPython's `display()`.

    Side Effects:
        - Executes `render_callback` multiple times (once per combination), potentially triggering significant computation.
        - Displays progress bars and interactive widgets in the active notebook cell.
    """
    # 1. Prepare combinations
    # Extract keys and value lists to generate the Cartesian product of all parameters
    param_names = list(param_config.keys())
    param_values = list(param_config.values())
    combinations = list(itertools.product(*param_values))
    
    cache = {}
    
    # 2. Setup Progress Bar
    # Visualization of the pre-calculation phase so the user knows the kernel hasn't hung
    progress = widgets.IntProgress(min=0, max=len(combinations), description='Pre-calc:', style={'bar_color': 'maroon'})
    label = widgets.Label(value="Initializing...")
    display(widgets.VBox([label, progress]))
    
    # 3. Pre-calculation Loop
    # Iterate through every possible state to populate the view cache
    for combo in combinations:
        # Create a kwargs dictionary for the current state, e.g., {'Env': '1', 'Radius': 0.2}
        current_params = dict(zip(param_names, combo))
        
        # Update Label to show current activity
        param_str = ", ".join([f"{k}={v}" for k, v in current_params.items()])
        label.value = f"Processing: {param_str}"
        
        # Capture Output
        # The 'widgets.Output' context manager intercepts stdout and matplotlib plots
        out = widgets.Output()
        with out:
            render_callback(**current_params)
        
        # Store the captured output widget in the cache, using the parameter tuple as the lookup key
        cache[combo] = out
        progress.value += 1

    # 4. Hide Progress
    # Remove the loading bar once processing is complete to clean up the UI
    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 5. Generate Controls dynamically
    controls = []
    
    for name, values in param_config.items():
        # Heuristic: Choose the best widget type based on data characteristics.
        # Use a Slider for long lists of numbers; use a Dropdown for short lists or strings.
        if len(values) > 5 and isinstance(values[0], (int, float)):
             widget = widgets.SelectionSlider(options=values, description=f'{name}:', continuous_update=False)
        else:
             widget = widgets.Dropdown(options=values, description=f'{name}:')
        controls.append(widget)
    
    # 6. Interaction Logic
    # Set the initial view based on the default values of the created widgets
    initial_combo = tuple(c.value for c in controls)
    view_container = widgets.VBox([cache.get(initial_combo, widgets.Output())])
    
    def on_change(change):
        """Observer function triggered whenever a widget value changes."""
        # Get current values from all controls to form the lookup key
        new_combo = tuple(c.value for c in controls)
        # Swap the visible child of the container to the pre-cached output
        if new_combo in cache:
            view_container.children = [cache[new_combo]]
            
    # Link all controls to the observer
    for w in controls:
        w.observe(on_change, names='value')

    # 7. Display Layout
    # Organize controls horizontally and place the output container below them
    control_box = widgets.HBox(controls)
    display(control_box, view_container)

def render_initial_path(name, item):
    """
    Callback function to visualize the static, unoptimized environment state.

    This helper is designed to be passed into the interactive viewer or used standalone to render 
    the baseline scenario. It retrieves the planner instance associated with the specific benchmark 
    environment and uses the collision checker's visualization tools to plot both the obstacles 
    and the initial "jagged" (piecewise-linear) path derived from the node sequence.

    Parameters:
        name (str): The identifier of the environment (e.g., "1", "2").
        item (dict): A dictionary containing the environment context, specifically:
                     - 'planner': The configured planner object.
                     - 'solution_node_names': List of node IDs for the initial path.

    Returns:
        None: Displays the plot using matplotlib but returns no object.

    Side Effects:
        - Creates a new matplotlib figure via `planner._collisionChecker.draw_enviroments()`.
        - Modifies the global pyplot state by setting the title.
    """
    planner = item["planner"]
    
    # Draw Obstacles
    # Initialize the plot canvas and render the red polygon obstacles
    ax = planner._collisionChecker.draw_enviroments()
    
    # Draw Path
    # Convert the list of node names into a list of (x,y) coordinates using the local helper
    path_pos = retrieve_path_positions(planner.graph, item['solution_node_names'])
    
    # Overlay the straight-line path (Start -> Waypoints -> Goal) onto the environment map
    planner._collisionChecker.draw_path(path_pos, ax=ax)
    
    # Add a descriptive title identifying the specific benchmark being viewed
    plt.title(f"Environment {name} - Initial Path")

def interactive_environment_exploration(env_dict):
    """
    Launches a simple interactive dashboard to visualize the benchmark environments and their initial paths.

    This function serves as a preliminary inspection tool, allowing the user to browse through the 
    loaded benchmark scenarios (Obstacles + Start/Goal + Initial Path) before applying any optimization. 
    It leverages the generic `create_interactive_viewer` to generate a dropdown menu for environment selection, 
    simplifying the exploration process.

    Parameters:
        env_dict (dict): A dictionary containing the benchmark data, where keys are environment IDs 
                         and values contain the 'planner' instance and 'solution_node_names'.

    Returns:
        None: Displays the interactive widgets and plots directly.

    Side Effects:
        - Creates and displays IPython widgets (Dropdown).
        - Renders matplotlib figures corresponding to the selected environment.
    """
    # Extract the list of environment IDs (e.g., ['1', '2', '3']) to populate the dropdown options
    env_keys = list(env_dict.keys())

    # Define the render callback locally to capture 'env_dict' via closure
    # This wrapper allows the generic viewer to pass just the 'Env' name for the data lookup
    def render_dashboard(Env):
        item = env_dict[Env]
        # Reuse the existing helper function to handle the actual plotting logic (DRY principle)
        render_initial_path(Env, item)
        plt.show()

    # Define configuration for the generic viewer
    # The key 'Env' becomes the label for the dropdown widget
    config = {
        'Env': env_keys
    }
    
    # Launch the generic viewer factory to build the UI and link it to our callback
    create_interactive_viewer(config, render_dashboard)

def interactive_radius_exploration(env_dict, optimizer):
    """
    Performs a systematic parameter sweep of the smoothing radius ($r$) and launches an interactive dashboard.

    This function investigates how the size of the smoothing radius affects the resulting path length and 
    validity across all benchmark environments. It pre-calculates the optimization results for a range 
    of radii (from 0.05 to 0.5) to ensure instant feedback during interaction.

    Because the `planner` object is mutable and shared, a critical step involves caching the entire 
    "Graph State" (specifically the calculated control points $P_{2n}$ and attributes) for each simulation step. 
    The interactive viewer then restores this state before rendering, ensuring that the visualization correctly 
    reflects the geometry of the selected radius, not the last one computed.

    Parameters:
        env_dict (dict): Dictionary of benchmark environments containing the planner and node sequences.
        optimizer (object): The optimizer instance (e.g., `OptimizeFlyby`) used to generate the paths.

    Returns:
        None: Displays the interactive viewer directly.

    Side Effects:
        - Temporarily modifies the planner's graph during the pre-calculation loop.
        - Displays a progress bar during computation.
        - Renders a complex multi-panel figure (Map + Analysis Plot + Performance Bars) on user interaction.
    """
    # Define the sweep range for the radius parameter
    r_steps = np.linspace(0.05, 0.5, 10)
    # Clean up the values (rounding and removing duplicates) and include a very small radius (0.02) as a baseline
    r_values = sorted(list(set([0.02] + list(np.round(r_steps, 2)))))
    env_keys = list(env_dict.keys())

    # 1. Pre-calculation
    results_cache = {}
    # Structure to hold summary curves (Radius vs Length) for the analysis plot
    summary_data = {name: {'r': [], 'len': []} for name in env_keys}
    
    # Initialize UI progress feedback
    total_calcs = len(env_keys) * len(r_values)
    progress = widgets.IntProgress(min=0, max=total_calcs, description='Pre-calc:', style={'bar_color': 'maroon'})
    label = widgets.Label(value="Initializing...")
    display(widgets.VBox([label, progress]))

    # Iterate through every environment
    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        # Iterate through every radius value
        for r in r_values:
            # A. Run Optimization
            # Clean previous run data to ensure independence
            IPPerfMonitor.clearData()
            clear_graph_attributes(planner)
            
            # Execute the fly-by optimization with the current radius
            config = {'r_init': r}
            optimized_path = optimizer.optimizePath(node_names, planner, config)
            
            # Save the calculated P2n and r values because the planner 
            # object is mutable and will be overwritten in the next loop.
            # This 'snapshot' is essential for correct visualization later.
            graph_state = {}
            for node in node_names:
                node_data = planner.graph.nodes[node]
                graph_state[node] = {
                    'r': node_data.get('r'),
                    'P2n': node_data.get('P2n'),      # Important: Position of control point
                    'fixed_k': node_data.get('fixed_k')
                }
            # -----------------------------------------

            # B. Capture Metrics
            # Get performance stats and physical length
            stats, len_orig, len_opt = capture_performance_metrics(planner, node_names)
            
            # C. Store Data
            # Cache everything needed to reconstruct the scene later
            results_cache[(name, r)] = {
                'path': optimized_path,
                'graph_state': graph_state, # <--- Store the state
                'stats': stats,
                'len_orig': len_orig,
                'len_opt': len_opt
            }
            
            # Append data for the summary trend line
            summary_data[name]['r'].append(r)
            summary_data[name]['len'].append(len_opt)
            progress.value += 1

    # Cleanup UI
    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 2. Render Function
    # This callback updates the display when the user changes widgets
    def render_dashboard(Env, Radius):
        res = results_cache[(Env, Radius)]
        item = env_dict[Env]
        planner = item['planner']
        
        # --- CRITICAL FIX: RESTORE GRAPH STATE ---
        # Before drawing, force the planner graph to match the stored state for this Radius.
        # Without this, the plot would show the geometry of the LAST calculated radius, 
        # not the one currently selected by the user.
        for node, attrs in res['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
        # -----------------------------------------
        
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: Map (Left)
        # Visualizes the environment and the path geometry
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(res['path'], planner, ax=ax_map)
        ax_map.set_title(f"Environment {Env} | r={Radius}")

        # Plot 2: Summary (Right)
        # Shows the trend of Path Length vs Radius to help identify the "sweet spot"
        ax_sum = fig.add_subplot(gs[0, 1])
        r_data = summary_data[Env]['r']
        l_data = summary_data[Env]['len']
        
        ax_sum.plot(r_data, l_data, 'b-o', label='Path Length')
        # Highlight the currently selected data point
        ax_sum.plot(Radius, res['len_opt'], 'r*', markersize=15, label='Current')
        ax_sum.set_xlabel('Smoothing Radius (r)')
        ax_sum.set_ylabel('Length [m]')
        ax_sum.set_title('Radius Impact Analysis')
        ax_sum.grid(True, alpha=0.3)
        ax_sum.legend()
        
        plt.tight_layout()
        plt.show()
        
        # Render the performance bar chart (Counts and Timings) below the main figure
        plot_performance_data(res['stats'], res['len_orig'], res['len_opt'])

    # 3. Launch
    # Configure the generic viewer with our parameter lists
    config = {
        'Env': env_keys,
        'Radius': r_values
    }
    create_interactive_viewer(config, render_dashboard)


    
def interactive_k_exploration(env_dict, optimizer, r_fixed=0.5):
    """
    Performs a systematic parameter sweep of the asymmetry factor ($k$) and launches an interactive dashboard.

    This function investigates how the shape (skew) of the Bezier curves affects the path efficiency.
    It iterates through a range of $k$ values (from sharp/tight turns at low $k$ to wide/elongated turns at high $k$)
    while keeping the smoothing radius fixed. Like the radius exploration, it pre-calculates all states
    and caches them to provide instant visual feedback in the dashboard.

    The resulting analysis typically reveals a "U-shaped" optimization curve, where an intermediate $k$ value
    minimizes the path length better than symmetric rounding ($k=1.0$).

    Parameters:
        env_dict (dict): Dictionary of benchmark environments containing the planner and node sequences.
        optimizer (object): The optimizer instance used to generate the paths.
        r_fixed (float): The constant smoothing radius to use for all $k$ tests (default: 0.5).

    Returns:
        None: Displays the interactive viewer directly.

    Side Effects:
        - Temporarily modifies the planner's graph during the pre-calculation loop.
        - Displays a progress bar during computation.
        - Renders a complex multi-panel figure (Map + Optimization Curve + Performance Bars) on user interaction.
    """
    # 1. Define Parameter Ranges
    # We sweep k from 0.1 (very sharp start) to 3.0 (very sharp end) to cover the full spectrum of curve shapes.
    k_steps = np.linspace(0.1, 3.0, 15)
    # Clean up values: round to 2 decimals for readability and sort them.
    k_values = sorted(list(np.round(k_steps, 2)))
    
    # We define a neutral k=1.0 explicitly to ensure it's in the list for baseline comparison.
    if 1.0 not in k_values:
        k_values.append(1.0)
        k_values.sort()

    env_keys = list(env_dict.keys())

    # 2. Pre-calculation
    results_cache = {}
    # Structure to store the trend line data (K-Factor vs Path Length) for every environment.
    summary_data = {name: {'k': [], 'len': []} for name in env_keys}
    
    # Initialize UI progress feedback
    total_calcs = len(env_keys) * len(k_values)
    progress = widgets.IntProgress(min=0, max=total_calcs, description='Pre-calc:', style={'bar_color': 'purple'})
    label = widgets.Label(value=f"Initializing k-sweep (r={r_fixed})...")
    display(widgets.VBox([label, progress]))

    # Iterate through every environment
    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        # Iterate through every asymmetry factor k
        for k in k_values:
            label.value = f"Optimizing {name} with k={k}..."
            
            # A. Run Optimization
            # Reset planner state to ensure independent tests
            IPPerfMonitor.clearData()
            clear_graph_attributes(planner)
            
            # CONFIG: Fixed r, Dynamic k
            config = {'r_init': r_fixed, 'k': k}
            optimized_path = optimizer.optimizePath(node_names, planner, config)
            
            # Capture Graph State (Critical for visualization)
            # Store the exact geometric parameters (P2n, r, fixed_k) for this specific simulation step.
            graph_state = {}
            for node in node_names:
                node_data = planner.graph.nodes[node]
                graph_state[node] = {
                    'r': node_data.get('r'),
                    'P2n': node_data.get('P2n'),
                    'fixed_k': node_data.get('fixed_k')
                }

            # B. Capture Metrics
            # Get computational cost and path length stats
            stats, len_orig, len_opt = capture_performance_metrics(planner, node_names)
            
            # C. Store Data
            # Cache the full result set for the viewer
            results_cache[(name, k)] = {
                'path': optimized_path,
                'graph_state': graph_state,
                'stats': stats,
                'len_orig': len_orig,
                'len_opt': len_opt
            }
            
            # Append data for the "U-curve" analysis plot
            summary_data[name]['k'].append(k)
            summary_data[name]['len'].append(len_opt)
            progress.value += 1

    # Cleanup UI
    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 3. Define the Dashboard Renderer
    # This callback is triggered whenever the user changes the dropdown selections
    def render_dashboard(Env, K_Factor):
        res = results_cache[(Env, K_Factor)]
        item = env_dict[Env]
        planner = item['planner']
        
        # Restore Graph State
        # Force the planner to adopt the geometry of the selected K_Factor before plotting
        for node, attrs in res['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
        
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: Map (Left)
        # Visualizes the path geometry with the specific curve skew
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(res['path'], planner, ax=ax_map)
        ax_map.set_title(f"Env {Env} | r_init={r_fixed} | k={K_Factor}")

        # Plot 2: K vs Length (Right)
        # Visualizes the optimization landscape (The "U" curve) to find the minimum
        ax_sum = fig.add_subplot(gs[0, 1])
        k_data = summary_data[Env]['k']
        l_data = summary_data[Env]['len']
        
        ax_sum.plot(k_data, l_data, 'g-o', label='Path Length') # Green for K-plots
        # Highlight the current selection
        ax_sum.plot(K_Factor, res['len_opt'], 'r*', markersize=15, label='Current')
        
        ax_sum.set_xlabel('Asymmetry Factor (k)')
        ax_sum.set_ylabel('Total Path Length [m]')
        ax_sum.set_title(f'Impact of Asymmetry (k) at r={r_fixed}')
        ax_sum.grid(True, alpha=0.3)
        ax_sum.legend()
        
        plt.tight_layout()
        plt.show()
        
        # Plot 3: Performance (Bottom)
        # Shows execution time and operation counts
        plot_performance_data(res['stats'], res['len_orig'], res['len_opt'])

    # 4. Launch Viewer
    # Setup the generic viewer with Environment and K_Factor selectors
    config = {
        'Env': env_keys,
        'K_Factor': k_values
    }
    
    create_interactive_viewer(config, render_dashboard)

def interactive_global_k_optimization(env_dict, optimizer):
    """
    Automates the search for the globally optimal asymmetry factor ($k$) across all environments and presents 
    the results in an interactive dashboard.

    This function acts as a high-level manager that:
    1. Iterates through every benchmark environment in `env_dict`.
    2. Triggers the `optimize_global_k` method (from the `optimizer` class) to perform a full parameter sweep 
       for the best global $k$ value.
    3. Caches the optimal results, including the optimization curve (sweep history) and the final graph state.
    4. Generates a UI allowing the user to inspect the optimal result for each environment, visualizing both 
       the physical path and the mathematical "optimization landscape" (the convex curve showing why a specific 
       $k$ was chosen).

    Parameters:
        env_dict (dict): Dictionary of benchmark environments containing planner instances and node sequences.
        optimizer (object): The optimization engine (e.g., `OptimizeFlyby`) capable of running `optimize_global_k`.

    Returns:
        None: Displays the interactive dashboard directly.

    Side Effects:
        - Computationally intensive: Runs multiple optimization loops for every environment during the pre-calculation phase.
        - Displays progress bars and eventually a complex matplotlib layout with multiple subplots.
    """
    env_keys = list(env_dict.keys())
    cache = {}
    
    # 1. Progress Bar
    # Initialize visual feedback for the potentially long-running batch optimization process
    progress = widgets.IntProgress(min=0, max=len(env_keys), description='Optimizing:', style={'bar_color': 'green'})
    label = widgets.Label(value="Running Global Optimization...")
    display(widgets.VBox([label, progress]))
    
    # 2. Pre-calculation Loop
    # Process each environment sequentially to find its specific optimal 'k'
    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']

        # A. Clear Data before the optimization starts
        # Ensure fresh profiling stats and remove any old graph attributes (r, P2n)
        IPPerfMonitor.clearData()
        clear_graph_attributes(planner)
        
        # B. Run the Analyzer (Suppress internal plot)
        # Execute the parameter sweep. We set plot=False because we will build a custom dashboard later.
        # This returns the best k AND the sweep history (k_values vs lengths) needed for the curve plot.
        result = optimizer.optimize_global_k(planner, node_names, r_fixed=0.5, plot=False)

        # C. Capture the performance metrics
        # Get the collision check counts and execution times for the *optimal* run
        stats, len_orig, len_global = capture_performance_metrics(planner, node_names)
        
        # D. Store in Cache
        # Save all necessary data to reconstruct the visualization without re-running the solver
        cache[name] = {
            'result': result, 
            'path': result['optimized_path'], # Use the path returned by the optimizer
            'graph_state': {
                node: {
                    'r': planner.graph.nodes[node].get('r'),
                    'P2n': planner.graph.nodes[node].get('P2n'),
                    'fixed_k': planner.graph.nodes[node].get('fixed_k')
                } for node in node_names
            },
            'stats': stats,
            'len_orig': len_orig,
            'len_opt': len_global
        }
        
        progress.value += 1

    # Cleanup UI
    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 3. Define Dashboard Renderer
    # Callback to display the cached results for a specific environment
    def render_dashboard(Env):
        data = cache[Env]
        res = data['result']
        item = env_dict[Env]
        planner = item['planner']
        
        # Restore Graph State
        # Apply the optimal geometric attributes (P2n, k) to the planner before drawing
        for node, attrs in data['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
            
        # --- Top Row: Map | Curve ---
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: Map (Top Left)
        # Show the G1-smooth path within the obstacle field
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(data['path'], planner, ax=ax_map)
        ax_map.set_title(f"Environment {Env} | Optimal k={res['best_k']:.2f}")

        # Plot 2: K-Sweep Curve (Top Right)
        # Visualize the optimization landscape using the history data from the sweep
        ax_curve = fig.add_subplot(gs[0, 1])
        ax_curve.plot(res['k_values'], res['lengths'], 'b-o', label='Path Length')
        
        # Highlight the Global Minimum
        ax_curve.plot(res['best_k'], res['min_length'], 'r*', markersize=15, label=f'Optimum ({res["best_k"]:.2f})')
        
        ax_curve.set_xlabel('Asymmetry Factor (k)')
        ax_curve.set_ylabel('Total Path Length [m]')
        ax_curve.set_title(f'Optimization Landscape (r=0.5)')
        ax_curve.grid(True, alpha=0.3)
        ax_curve.legend()
        
        plt.tight_layout()
        plt.show()
        
        # --- Bottom Row: Performance ---
        # Display the stats for the optimal configuration
        plot_performance_data(data['stats'], data['len_orig'], data['len_opt'])

    # 4. Create Viewer
    # Since this viewer only has one parameter (Env), we build a simpler custom UI
    # instead of using the generic 'create_interactive_viewer'.
    dropdown = widgets.Dropdown(options=env_keys, value=env_keys[0], description='Select Env:')
    
    # Simple interaction container to hold the output
    container = widgets.VBox([widgets.Output()])
    
    def on_change(change):
        """Observer to update the view when the dropdown changes."""
        if change['type'] == 'change' and change['name'] == 'value':
            out = widgets.Output()
            with out:
                render_dashboard(change['new'])
            container.children = [out]
            
    # Initial Render
    with container.children[0]:
        render_dashboard(dropdown.value)
        
    dropdown.observe(on_change, names='value')
    display(dropdown, container)
    
def interactive_individual_k_optimization(env_dict, optimizer):
    """
    Executes a comparative benchmark of the 'Individual Corner Optimization' strategy against standard baselines 
    and presents the findings in an interactive dashboard.

    This function runs a three-way contest for every environment in `env_dict` to quantify the benefits of 
    fine-tuning the path geometry:
    1. **Baseline 1 (Symmetric)**: The standard algorithm using dynamic metric symmetry ($k=None$).
    2. **Baseline 2 (Global Best)**: The result of a parameter sweep finding the single best scalar $k$ for the entire path.
    3. **Target (Individual)**: The result of Coordinate Descent, where every corner is optimized independently.

    It captures the path length improvement and the computational cost for the Individual strategy. The resulting 
    dashboard displays the final optimized map alongside a bar chart comparing the four stages of improvement 
    (Original -> Symmetric -> Global -> Individual), highlighting the specific gains achieved by local fine-tuning.

    Parameters:
        env_dict (dict): Dictionary of benchmark environments containing planner instances and node sequences.
        optimizer (object): The optimization engine capable of running `optimize_individual_k` and `optimize_global_k`.

    Returns:
        None: Displays the interactive dashboard directly.

    Side Effects:
        - Computationally intensive: Runs multiple full optimization routines (Baseline 1, Baseline 2 Sweep, 
          Individual Coordinate Descent) for every environment.
        - Modifies the planner's graph repeatedly.
        - Displays progress widgets and matplotlib figures.
    """
    env_keys = list(env_dict.keys())
    cache = {}
    
    # 1. Progress Bar
    # Initialize a progress bar to track the multi-stage optimization across all environments
    progress = widgets.IntProgress(min=0, max=len(env_keys), description='Optimizing:', style={'bar_color': 'orange'})
    label = widgets.Label(value="Running Coordinate Descent (Individual k)...")
    display(widgets.VBox([label, progress]))

    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        # --- A. Baseline 1: Symmetric (k=None) ---
        clear_graph_attributes(planner)
        # Run standard optimization with dynamic symmetric rounding
        optimizer.optimizePath(node_names, planner, config={'r_init': 0.5, 'k': None})
        # Capture the length for the first baseline
        len_symmetric = calculate_path_length(planner, node_names, use_curves=True)

        # --- B. Baseline 2: Best Global K ---
        # Find the best single k for comparison via a parameter sweep.
        # Note: We suppress plotting and stats tracking here to avoid polluting the metrics 
        # for the final "Individual" run we want to profile.
        global_res = optimizer.optimize_global_k(planner, node_names, r_fixed=0.5, plot=False)
        len_global = global_res['min_length']
        best_global_k = global_res['best_k']
        
        # --- C. Target: Individual Optimization ---
        
        # 1. Clear Previous Data (Ensure we measure ONLY the individual optimization process)
        # Reset the performance monitor so the stats reflect purely the cost of the Coordinate Descent
        IPPerfMonitor.clearData() 
        clear_graph_attributes(planner)
        
        # 2. Run the Coordinate Descent
        # Execute the heavy optimization algorithm that tunes each node's k value iteratively
        optimizer.optimize_individual_k(node_names, planner, config={'r_init': 0.5})
        
        # 3. Capture Final State
        # Run optimizePath one last time to ensure graph attributes (P2n, etc.) are physically set 
        # on the graph nodes for accurate drawing in the dashboard.
        config = {'r_init': 0.5, 'k': None} 
        final_path_individual = optimizer.optimizePath(node_names, planner, config)
        
        # 4. Capture Metrics
        # Calculate the final length of the fully optimized path
        len_individual = calculate_path_length(planner, node_names, use_curves=True)
        
        # Capture performance stats (Time/Count) specifically for the Coordinate Descent process
        stats, len_orig, _ = capture_performance_metrics(planner, node_names)
        
        # 5. Capture Graph State for Visualization
        # Snapshot the geometric parameters so the dashboard can restore them later
        graph_state = {}
        for node in node_names:
            node_data = planner.graph.nodes[node]
            graph_state[node] = {
                'r': node_data.get('r'),
                'P2n': node_data.get('P2n'),
                'fixed_k': node_data.get('fixed_k')
            }

        # Store all baselines and results in the cache
        cache[name] = {
            'len_symmetric': len_symmetric,    # <--- Stored Symmetric Baseline
            'len_global': len_global,
            'best_global_k': best_global_k,
            'len_individual': len_individual,
            'path': final_path_individual,
            'graph_state': graph_state,
            'stats': stats,
            'len_orig': len_orig
        }
        progress.value += 1

    # Cleanup UI
    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 2. Define Dashboard Renderer
    # Callback to display the comparative results for a selected environment
    def render_dashboard(Env):
        data = cache[Env]
        item = env_dict[Env]
        planner = item['planner']
        
        # Restore Graph State
        # Apply the complex mix of individual 'k' values to the graph before plotting
        for node, attrs in data['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
            
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: The Final Map (Left)
        # Visualizes the path resulting from individual corner optimization
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(data['path'], planner, ax=ax_map)
        ax_map.set_title(f"Env {Env} | Individual Optimization")

        # Plot 2: Improvement Comparison (Right)
        ax_chart = fig.add_subplot(gs[0, 1])
        
        # Define labels for the four evolutionary stages of the path
        labels = [
            'Original\n(Linear)', 
            'Symmetric\n(k=Sym)',              # <--- Baseline 1
            'Best Global k\n(k={:.2f})'.format(data['best_global_k']), # <--- Baseline 2
            'Individual k\n(Mixed)'            # <--- Target
        ]
        
        # Collect the length values for the bars
        values = [
            round(data['len_orig'], 2), 
            round(data['len_symmetric'], 2),   
            round(data['len_global'], 2), 
            round(data['len_individual'], 2)
        ]
        
        # Color coding: Gray for baselines, Green for the final result
        colors = ['lightgray', 'lightblue', 'gray', 'green']
        
        bars = ax_chart.bar(labels, values, color=colors, alpha=0.9, width=0.6)
        ax_chart.set_ylabel('Total Path Length [m]')
        ax_chart.set_title(f"Optimization Results")
        
        # Dynamic Y-Limit to focus on the top of the bars (where the differences are)
        min_val = min(values)
        max_val = max(values)
        margin = (max_val - min_val) * 2 if max_val != min_val else 1.0
        ax_chart.set_ylim(min_val - margin*0.15, max_val + margin*0.1)
        
        # Add numeric labels on top of each bar
        for bar in bars:
            height = bar.get_height()
            ax_chart.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.2f} m',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')
            
        # Add Savings Annotation (Global vs Individual)
        # Calculates and displays the specific gain achieved by moving from Global to Individual optimization
        savings = data['len_global'] - data['len_individual']
        if savings > 0.0001:
            text_x_pos = 2.5 # Position text between the last two bars
            text_y_pos = (data['len_global'] + data['len_individual']) / 2
            
            ax_chart.text(text_x_pos, text_y_pos, 
                         f" Improvement:\n-{savings:.4f} m", 
                         ha='center', va='center', fontsize=11, color='green', 
                         bbox=dict(facecolor='white', alpha=0.9, edgecolor='green'))

        plt.tight_layout()
        plt.show()
        
        # Performance Stats (Comparison of the Individual Optimization Process cost)
        plot_performance_data(data['stats'], data['len_orig'], data['len_individual'])

    # 3. Create Viewer
    # Setup the environment selector dropdown
    dropdown = widgets.Dropdown(options=env_keys, value=env_keys[0], description='Select Env:')
    container = widgets.VBox([widgets.Output()])
    
    def on_change(change):
        """Observer to update the view when the dropdown changes."""
        if change['type'] == 'change' and change['name'] == 'value':
            out = widgets.Output()
            with out:
                render_dashboard(change['new'])
            container.children = [out]
            
    # Initial Render
    with container.children[0]:
        render_dashboard(dropdown.value)
        
    dropdown.observe(on_change, names='value')
    display(dropdown, container)