import itertools
import ipywidgets as widgets
from IPython.display import display
import matplotlib.pyplot as plt
import numpy as np

from analysis import (
    retrieve_path_positions,
    clear_graph_attributes,
    capture_performance_metrics,
    plot_performance_data,
    calculate_path_length
)
from planners.IPPerfMonitor import IPPerfMonitor

def create_interactive_viewer(param_config: dict, render_callback):
    """
    A generic interactive viewer that:
    1. Generates combinations of all parameters in param_config.
    2. Runs render_callback(**params) for each combination (pre-calculation).
    3. Creates widgets (Dropdowns/Sliders) automatically based on inputs.
    
    Args:
        param_config (dict): {'ParamName': [list_of_values], ...}
        render_callback (func): Function accepting kwargs matching param_config keys.
    """
    # 1. Prepare combinations
    param_names = list(param_config.keys())
    param_values = list(param_config.values())
    combinations = list(itertools.product(*param_values))
    
    cache = {}
    
    # 2. Setup Progress Bar
    progress = widgets.IntProgress(min=0, max=len(combinations), description='Pre-calc:', style={'bar_color': 'maroon'})
    label = widgets.Label(value="Initializing...")
    display(widgets.VBox([label, progress]))
    
    # 3. Pre-calculation Loop
    for combo in combinations:
        # Create a dictionary for the current state, e.g., {'Env': '1', 'Radius': 0.2}
        current_params = dict(zip(param_names, combo))
        
        # Update Label
        param_str = ", ".join([f"{k}={v}" for k, v in current_params.items()])
        label.value = f"Processing: {param_str}"
        
        # Capture Output
        out = widgets.Output()
        with out:
            render_callback(**current_params)
        
        # Store in cache using the tuple of values as key
        cache[combo] = out
        progress.value += 1

    # 4. Hide Progress
    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 5. Generate Controls dynamically
    controls = []
    
    for name, values in param_config.items():
        # Heuristic: Use Slider for numbers with many options, Dropdown otherwise
        if len(values) > 5 and isinstance(values[0], (int, float)):
             widget = widgets.SelectionSlider(options=values, description=f'{name}:', continuous_update=False)
        else:
             widget = widgets.Dropdown(options=values, description=f'{name}:')
        controls.append(widget)
    
    # 6. Interaction Logic
    # Initial view
    initial_combo = tuple(c.value for c in controls)
    view_container = widgets.VBox([cache.get(initial_combo, widgets.Output())])
    
    def on_change(change):
        # Get current values from all controls
        new_combo = tuple(c.value for c in controls)
        # Swap view
        if new_combo in cache:
            view_container.children = [cache[new_combo]]
            
    # Link all controls
    for w in controls:
        w.observe(on_change, names='value')

    # 7. Display Layout
    # Group controls horizontally, then stack plot below
    control_box = widgets.HBox(controls)
    display(control_box, view_container)

def render_initial_path(name, item):
    """
    Callback to render the initial jagged path.
    """
    planner = item["planner"]
    # Draw Obstacles
    ax = planner._collisionChecker.draw_enviroments()
    
    # Draw Path
    # We use the existing helper in this file to get coords
    path_pos = retrieve_path_positions(planner.graph, item['solution_node_names'])
    planner._collisionChecker.draw_path(path_pos, ax=ax)
    
    plt.title(f"Environment {name} - Initial Path")

def interactive_environment_exploration(env_dict):
    """
    Launches a simple dashboard to visualize the benchmark environments 
    and their initial (non-optimized) paths.
    Uses the existing render_initial_path helper to keep code DRY.
    """
    env_keys = list(env_dict.keys())

    # Define the render callback locally to capture env_dict
    def render_dashboard(Env):
        item = env_dict[Env]
        # Reuse the existing helper function
        render_initial_path(Env, item)
        plt.show()

    # Define configuration for the generic viewer
    config = {
        'Env': env_keys
    }
    
    # Launch the viewer
    create_interactive_viewer(config, render_dashboard)

def interactive_radius_exploration(env_dict, optimizer):
    """
    Performs a full sweep of radius values, caches the GRAPH STATE,
    and launches a dashboard viewer.
    """
    r_steps = np.linspace(0.05, 0.5, 10)
    r_values = sorted(list(set([0.02] + list(np.round(r_steps, 2)))))
    env_keys = list(env_dict.keys())

    # 1. Pre-calculation
    results_cache = {}
    summary_data = {name: {'r': [], 'len': []} for name in env_keys}
    
    total_calcs = len(env_keys) * len(r_values)
    progress = widgets.IntProgress(min=0, max=total_calcs, description='Pre-calc:', style={'bar_color': 'maroon'})
    label = widgets.Label(value="Initializing...")
    display(widgets.VBox([label, progress]))

    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        for r in r_values:
            # A. Run Optimization
            IPPerfMonitor.clearData()
            clear_graph_attributes(planner)
            
            config = {'r_init': r}
            optimized_path = optimizer.optimizePath(node_names, planner, config)
            
            # Save the calculated P2n and r values because the planner 
            # object is mutable and will be overwritten in the next loop.
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
            stats, len_orig, len_opt = capture_performance_metrics(planner, node_names)
            
            # C. Store Data
            results_cache[(name, r)] = {
                'path': optimized_path,
                'graph_state': graph_state, # <--- Store the state
                'stats': stats,
                'len_orig': len_orig,
                'len_opt': len_opt
            }
            
            summary_data[name]['r'].append(r)
            summary_data[name]['len'].append(len_opt)
            progress.value += 1

    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 2. Render Function
    def render_dashboard(Env, Radius):
        res = results_cache[(Env, Radius)]
        item = env_dict[Env]
        planner = item['planner']
        
        # --- CRITICAL FIX: RESTORE GRAPH STATE ---
        # Before drawing, force the planner graph to match the stored state for this Radius
        for node, attrs in res['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
        # -----------------------------------------
        
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: Map
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(res['path'], planner, ax=ax_map)
        ax_map.set_title(f"Environment {Env} | r={Radius}")

        # Plot 2: Summary
        ax_sum = fig.add_subplot(gs[0, 1])
        r_data = summary_data[Env]['r']
        l_data = summary_data[Env]['len']
        
        ax_sum.plot(r_data, l_data, 'b-o', label='Path Length')
        ax_sum.plot(Radius, res['len_opt'], 'r*', markersize=15, label='Current')
        ax_sum.set_xlabel('Smoothing Radius (r)')
        ax_sum.set_ylabel('Length [m]')
        ax_sum.set_title('Radius Impact Analysis')
        ax_sum.grid(True, alpha=0.3)
        ax_sum.legend()
        
        plt.tight_layout()
        plt.show()
        
        plot_performance_data(res['stats'], res['len_orig'], res['len_opt'])

    # 3. Launch
    config = {
        'Env': env_keys,
        'Radius': r_values
    }
    create_interactive_viewer(config, render_dashboard)

def interactive_k_exploration(env_dict, optimizer, r_fixed=0.5):
    """
    Performs a sweep of 'k' (asymmetry factor) values with a fixed radius 'r'.
    Generates a dashboard to analyze the impact of k on path length.
    """
    # 1. Define Parameter Ranges
    # We sweep k from 0.1 (sharp start) to 3.0 (sharp end)
    k_steps = np.linspace(0.1, 3.0, 15)
    k_values = sorted(list(np.round(k_steps, 2)))
    
    # We define a neutral k=1.0 explicitly to ensure it's in the list
    if 1.0 not in k_values:
        k_values.append(1.0)
        k_values.sort()

    env_keys = list(env_dict.keys())

    # 2. Pre-calculation
    results_cache = {}
    summary_data = {name: {'k': [], 'len': []} for name in env_keys}
    
    total_calcs = len(env_keys) * len(k_values)
    progress = widgets.IntProgress(min=0, max=total_calcs, description='Pre-calc:', style={'bar_color': 'purple'})
    label = widgets.Label(value=f"Initializing k-sweep (r={r_fixed})...")
    display(widgets.VBox([label, progress]))

    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        for k in k_values:
            label.value = f"Optimizing {name} with k={k}..."
            
            # A. Run Optimization
            IPPerfMonitor.clearData()
            clear_graph_attributes(planner)
            
            # CONFIG: Fixed r, Dynamic k
            config = {'r_init': r_fixed, 'k': k}
            optimized_path = optimizer.optimizePath(node_names, planner, config)
            
            # Capture Graph State (Critical for visualization)
            graph_state = {}
            for node in node_names:
                node_data = planner.graph.nodes[node]
                graph_state[node] = {
                    'r': node_data.get('r'),
                    'P2n': node_data.get('P2n'),
                    'fixed_k': node_data.get('fixed_k')
                }

            # B. Capture Metrics
            stats, len_orig, len_opt = capture_performance_metrics(planner, node_names)
            
            # C. Store Data
            results_cache[(name, k)] = {
                'path': optimized_path,
                'graph_state': graph_state,
                'stats': stats,
                'len_orig': len_orig,
                'len_opt': len_opt
            }
            
            summary_data[name]['k'].append(k)
            summary_data[name]['len'].append(len_opt)
            progress.value += 1

    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 3. Define the Dashboard Renderer
    def render_dashboard(Env, K_Factor):
        res = results_cache[(Env, K_Factor)]
        item = env_dict[Env]
        planner = item['planner']
        
        # Restore Graph State
        for node, attrs in res['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
        
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: Map
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(res['path'], planner, ax=ax_map)
        ax_map.set_title(f"Env {Env} | r_init={r_fixed} | k={K_Factor}")

        # Plot 2: K vs Length (The "U" curve)
        ax_sum = fig.add_subplot(gs[0, 1])
        k_data = summary_data[Env]['k']
        l_data = summary_data[Env]['len']
        
        ax_sum.plot(k_data, l_data, 'g-o', label='Path Length') # Green for K-plots
        ax_sum.plot(K_Factor, res['len_opt'], 'r*', markersize=15, label='Current')
        
        ax_sum.set_xlabel('Asymmetry Factor (k)')
        ax_sum.set_ylabel('Total Path Length [m]')
        ax_sum.set_title(f'Impact of Asymmetry (k) at r={r_fixed}')
        ax_sum.grid(True, alpha=0.3)
        ax_sum.legend()
        
        plt.tight_layout()
        plt.show()
        
        # Plot 3: Performance
        plot_performance_data(res['stats'], res['len_orig'], res['len_opt'])

    # 4. Launch Viewer
    config = {
        'Env': env_keys,
        'K_Factor': k_values
    }
    
    create_interactive_viewer(config, render_dashboard)

def interactive_global_k_optimization(env_dict, optimizer):
    """
    Runs the automatic k-optimization for all environments using the 
    optimizer's native analyze method, then displays a full dashboard.
    """
    env_keys = list(env_dict.keys())
    cache = {}
    
    # 1. Progress Bar
    progress = widgets.IntProgress(min=0, max=len(env_keys), description='Optimizing:', style={'bar_color': 'green'})
    label = widgets.Label(value="Running Global Optimization...")
    display(widgets.VBox([label, progress]))
    
    # 2. Pre-calculation Loop
    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        # A. Run the Analyzer (Suppress internal plot)
        # This returns the best k AND the sweep history we need for the curve
        result = optimizer.analyze_optimal_k(planner, node_names, r_fixed=0.5, plot=False)
        
        # B. Get Clean Performance Stats for the Final Run
        # The analyzer ran the final path, but the monitor has mixed data from the sweep.
        # We run it ONE last time purely to capture clean stats for the bar chart.
        IPPerfMonitor.clearData()
        clear_graph_attributes(planner)
        config = {'r_init': 0.5, 'k': result['best_k']}
        
        optimized_path = optimizer.optimizePath(node_names, planner, config)
        
        # Capture Graph State
        graph_state = {}
        for node in node_names:
            node_data = planner.graph.nodes[node]
            graph_state[node] = {
                'r': node_data.get('r'),
                'P2n': node_data.get('P2n'),
                'fixed_k': node_data.get('fixed_k')
            }
            
        # Capture Metrics
        stats, len_orig, len_opt = capture_performance_metrics(planner, node_names)
        
        # C. Store in Cache
        cache[name] = {
            'result': result, # Contains k_values, lengths, best_k
            'path': optimized_path,
            'graph_state': graph_state,
            'stats': stats,
            'len_orig': len_orig,
            'len_opt': len_opt
        }
        progress.value += 1

    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 3. Define Dashboard Renderer
    def render_dashboard(Env):
        data = cache[Env]
        res = data['result']
        item = env_dict[Env]
        planner = item['planner']
        
        # Restore Graph State
        for node, attrs in data['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
            
        # --- Top Row: Map | Curve ---
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: Map (Top Left)
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(data['path'], planner, ax=ax_map)
        ax_map.set_title(f"Environment {Env} | Optimal k={res['best_k']:.2f}")

        # Plot 2: K-Sweep Curve (Top Right)
        # We use the history returned by analyze_optimal_k
        ax_curve = fig.add_subplot(gs[0, 1])
        ax_curve.plot(res['k_values'], res['lengths'], 'b-o', label='Path Length')
        
        # Highlight Optimum
        ax_curve.plot(res['best_k'], res['min_length'], 'r*', markersize=15, label=f'Optimum ({res["best_k"]:.2f})')
        
        ax_curve.set_xlabel('Asymmetry Factor (k)')
        ax_curve.set_ylabel('Total Path Length [m]')
        ax_curve.set_title(f'Optimization Landscape (r=0.5)')
        ax_curve.grid(True, alpha=0.3)
        ax_curve.legend()
        
        plt.tight_layout()
        plt.show()
        
        # --- Bottom Row: Performance ---
        plot_performance_data(data['stats'], data['len_orig'], data['len_opt'])

    # 4. Create Viewer
    dropdown = widgets.Dropdown(options=env_keys, value=env_keys[0], description='Select Env:')
    
    # Simple interaction container
    container = widgets.VBox([widgets.Output()])
    
    def on_change(change):
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
    Runs 'Individual Corner Optimization' for all environments.
    Compares the result against the 'Best Global K' baseline.
    """
    env_keys = list(env_dict.keys())
    cache = {}
    
    # 1. Progress Bar
    progress = widgets.IntProgress(min=0, max=len(env_keys), description='Optimizing:', style={'bar_color': 'orange'})
    label = widgets.Label(value="Running Coordinate Descent (Individual k)...")
    display(widgets.VBox([label, progress]))

    for name in env_keys:
        item = env_dict[name]
        planner = item['planner']
        node_names = item['solution_node_names']
        
        # --- A. Baseline: Best Global K ---
        # Find the best single k for comparison
        global_res = optimizer.analyze_optimal_k(planner, node_names, r_fixed=0.5, plot=False)
        len_global = global_res['min_length']
        best_global_k = global_res['best_k']
        
        # --- B. Individual Optimization ---
        # 1. Clear previous state
        clear_graph_attributes(planner)
        
        # 2. Run the Coordinate Descent
        # FIX: Pass parameters inside a 'config' dictionary, not as kwargs
        optimizer.optimize_individual_corners(node_names, planner, config={'r_init': 0.5})
        
        # 3. Capture the Result
        # Run optimizePath one last time to ensure we get the final geometry 
        # based on the fixed_k values now stored in the graph.
        config = {'r_init': 0.5, 'k': None, 'r_decay': 0.8}
        final_path_individual = optimizer.optimizePath(node_names, planner, config)
        
        len_individual = calculate_path_length(planner, node_names, use_curves=True)
        
        # 4. Capture Graph State
        graph_state = {}
        for node in node_names:
            node_data = planner.graph.nodes[node]
            graph_state[node] = {
                'r': node_data.get('r'),
                'P2n': node_data.get('P2n'),
                'fixed_k': node_data.get('fixed_k')
            }
            
        # 5. Metrics
        stats, len_orig, _ = capture_performance_metrics(planner, node_names)

        cache[name] = {
            'len_global': len_global,
            'best_global_k': best_global_k,
            'len_individual': len_individual,
            'path': final_path_individual,
            'graph_state': graph_state,
            'stats': stats,
            'len_orig': len_orig
        }
        progress.value += 1

    progress.layout.display = 'none'
    label.layout.display = 'none'

    # 2. Define Dashboard Renderer
    def render_dashboard(Env):
        data = cache[Env]
        item = env_dict[Env]
        planner = item['planner']
        
        # Restore Graph State
        for node, attrs in data['graph_state'].items():
            planner.graph.nodes[node].update(attrs)
            
        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(1, 2)
        
        # Plot 1: The Final Map
        ax_map = fig.add_subplot(gs[0, 0])
        planner._collisionChecker.draw_enviroments(ax=ax_map)
        planner._collisionChecker.draw_optimized_path(data['path'], planner, ax=ax_map)
        ax_map.set_title(f"Env {Env} | Individual Optimization")

        # Plot 2: Improvement Comparison
        ax_chart = fig.add_subplot(gs[0, 1])
        
        labels = ['Best Global k\n(k={:.2f})'.format(data['best_global_k']), 'Individual k\n(Mixed)']
        values = [data['len_global'], data['len_individual']]
        colors = ['gray', 'green']
        
        bars = ax_chart.bar(labels, values, color=colors, alpha=0.7, width=0.5)
        ax_chart.set_ylabel('Total Path Length [m]')
        ax_chart.set_title(f"Optimization Results")
        
        # Dynamic Y-Limit
        min_val = min(values)
        max_val = max(values)
        margin = (max_val - min_val) * 2 if max_val != min_val else 1.0
        ax_chart.set_ylim(min_val - margin*0.1, max_val + margin*0.1)
        
        # Add labels
        for bar in bars:
            height = bar.get_height()
            ax_chart.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.4f} m',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')
            
        # Add Savings Annotation
        savings = data['len_global'] - data['len_individual']
        if savings > 0.0001:
            ax_chart.text(0.5, (min_val + max_val)/2, 
                         f" Improvement:\n-{savings:.4f} m", 
                         ha='center', va='center', fontsize=12, color='green', 
                         bbox=dict(facecolor='white', alpha=0.8, edgecolor='green'))

        plt.tight_layout()
        plt.show()
        
        # Performance Stats
        plot_performance_data(data['stats'], data['len_orig'], data['len_individual'])

    # 3. Create Viewer
    dropdown = widgets.Dropdown(options=env_keys, value=env_keys[0], description='Select Env:')
    container = widgets.VBox([widgets.Output()])
    
    def on_change(change):
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