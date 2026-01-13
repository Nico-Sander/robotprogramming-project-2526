import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display
import itertools

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

def find_optimal_k_interactive(env_dict, optimizer):
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

def capture_performance_metrics(planner, node_names):
    """
    Extracts collision check statistics from IPPerfMonitor and 
    calculates both original and optimized path lengths.
    
    Returns:
        tuple: (stats_dict, length_original, length_optimized)
    """
    # 1. Extract Performance Stats
    df = IPPerfMonitor.dataFrame()
    stats = {}
    target_funcs = ['lineInCollision', 'curveInCollision', 'pointInCollision']
    
    for f in target_funcs:
        f_data = df[df['name'] == f]
        if not f_data.empty:
            stats[f] = {
                'count': len(f_data), 
                'time': f_data['time'].sum()
            }
        else:
            stats[f] = {'count': 0, 'time': 0.0}

    # 2. Calculate Path Lengths
    len_orig = calculate_path_length(planner, node_names, use_curves=False)
    len_opt = calculate_path_length(planner, node_names, use_curves=True)

    return stats, len_orig, len_opt

def retrieve_path_positions(graph: nx.Graph, node_names: list) -> list:
    """ Retrieves coordinate tuples for a list of node names. """
    return [graph.nodes[name]["pos"] for name in node_names]

def calculate_path_length(planner, path: list, use_curves: bool = True) -> float:
    """
    Calculates the total length of the path.
    - use_curves=False: Returns standard Euclidean length of the original path.
    - use_curves=True: Returns length of the G1-optimized path (Lines + Bezier Arcs).
      Requires that the path has been optimized and 'P2n' is stored in the graph.
    """
    def get_pos(name):
        return np.array(planner.graph.nodes[name]['pos'])

    # Mode 1: Straight Line (Original)
    if not use_curves:
        length = 0.0
        for i in range(len(path) - 1):
            p1 = get_pos(path[i])
            p2 = get_pos(path[i+1])
            length += np.linalg.norm(p2 - p1)
        return length

    # Mode 2: Curved Path (G1 Optimized)
    total_length = 0.0
    
    # Pre-fetch P2n list
    p2n_list = []
    for name in path:
        p2n_list.append(planner.graph.nodes[name].get('P2n', planner.graph.nodes[name]['pos']))
        
    last_endpoint = p2n_list[0] # Start pos
    
    for i in range(1, len(path) - 1):
        node_name = path[i]
        r = planner.graph.nodes[node_name].get('r', 0.0)
        fixed_k = planner.graph.nodes[node_name].get('fixed_k')
        
        P_prev = p2n_list[i-1]
        P_curr = p2n_list[i]
        P_next = p2n_list[i+1]
        
        if r > 0:
            # --- Recalculate Geometry ---
            d_in = np.linalg.norm(P_curr - P_prev)
            d_out = np.linalg.norm(P_next - P_curr)
            
            if fixed_k is not None:
                li, lo = r, r * fixed_k
            else:
                if d_in <= d_out:
                    dist = r * d_in
                    li = r
                    lo = dist / d_out if d_out > 0 else 0
                else:
                    dist = r * d_out
                    lo = r
                    li = dist / d_in if d_in > 0 else 0
            
            S = P_curr + li * (P_prev - P_curr)
            E = P_curr + lo * (P_next - P_curr)
            
            # 1. Add Straight Segment Length (Last E -> Current S)
            total_length += np.linalg.norm(S - last_endpoint)
            
            # 2. Add Curve Length (Discrete Summation)
            steps = 20
            t_vals = np.linspace(0, 1, steps)
            term1 = np.outer((1 - t_vals)**2, S)
            term2 = np.outer(2 * (1 - t_vals) * t_vals, P_curr)
            term3 = np.outer(t_vals**2, E)
            curve_points = term1 + term2 + term3
            
            diffs = curve_points[1:] - curve_points[:-1]
            segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
            total_length += np.sum(segment_lengths)
            
            last_endpoint = E
        else:
            # No smoothing, add distance to the Control Point
            total_length += np.linalg.norm(P_curr - last_endpoint)
            last_endpoint = P_curr

    # Final Segment (Last E -> Goal)
    total_length += np.linalg.norm(p2n_list[-1] - last_endpoint)
    
    return total_length

def plot_performance_data(stats, len_original, len_optimized):
    """
    Plots performance metrics:
    1. Count of collision checks (Point, Line, Curve).
    2. Total time spent on checks.
    3. Path length comparison (Original vs Optimized).
    
    Uses a multi-axis layout (one left Y-axis, two right Y-axes) to display 
    different units (Counts, Seconds, Meters) on the same chart.
    """
    
    # Initialize the figure with sufficient height for the top legend
    fig, ax1 = plt.subplots(figsize=(12, 7))
    plt.title(f"Performance Metrics", pad=40)

    # Define categories for the X-axis
    categories = ['Point Checks', 'Line Checks', 'Curve Checks', 'Path Length']
    x_pos = np.arange(len(categories))
    width = 0.25  # Width of the bars

    # --- Axis 1 (Left): Operation Counts ---
    color1 = 'tab:blue'
    ax1.set_ylabel('Number of Calls', color=color1, fontweight='bold')
    
    # Retrieve data safely (default to 0 if missing)
    pt_count = stats.get('pointInCollision', {}).get('count', 0)
    ln_count = stats.get('lineInCollision', {}).get('count', 0)
    crv_count = stats.get('curveInCollision', {}).get('count', 0)
    
    # Plot counts for the first three categories
    counts = [pt_count, ln_count, crv_count, 0]
    
    # Plot Count Bars (Shifted left)
    bars1 = ax1.bar(x_pos - width, counts, width, label='Count', color=color1, alpha=0.7)
    ax1.tick_params(axis='y', labelcolor=color1)
    
    # Annotate bars with exact values
    for rect in bars1:
        height = rect.get_height()
        if height > 0:
            ax1.text(rect.get_x() + rect.get_width()/2., height,
                    f'{int(height)}', ha='center', va='bottom', color=color1)

    # --- Axis 2 (Right #1): Execution Time ---
    ax2 = ax1.twinx() 
    color2 = 'tab:orange'
    ax2.set_ylabel('Time (seconds)', color=color2, fontweight='bold')
    
    pt_time = stats.get('pointInCollision', {}).get('time', 0.0)
    ln_time = stats.get('lineInCollision', {}).get('time', 0.0)
    crv_time = stats.get('curveInCollision', {}).get('time', 0.0)
    
    times = [pt_time, ln_time, crv_time, 0]
    
    # Plot Time Bars (Centered on tick)
    bars2 = ax2.bar(x_pos, times, width, label='Time', color=color2, alpha=0.7)
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Annotate bars with time values
    for rect in bars2:
        height = rect.get_height()
        if height > 0:
            ax2.text(rect.get_x() + rect.get_width()/2., height,
                    f'{height:.4f}s', ha='center', va='bottom', color=color2, fontsize=9)

    # --- Axis 3 (Right #2): Path Length ---
    # Create a third axis and offset its spine outward to avoid overlap with Axis 2
    ax3 = ax1.twinx()
    ax3.spines["right"].set_position(("axes", 1.15)) 
    color3 = 'tab:purple'
    ax3.set_ylabel('Path Length', color=color3, fontweight='bold')
    
    # Plot comparison bars only at the 'Path Length' category index
    target_idx = 3 
    bars3_orig = ax3.bar(x_pos[target_idx] - width/2, len_original, width/2, color='gray', label='Original Len')
    bars3_opt  = ax3.bar(x_pos[target_idx] + width/2, len_optimized, width/2, color=color3, label='Optimized Len')
    
    ax3.tick_params(axis='y', labelcolor=color3)
    
    # Annotate length values
    ax3.text(x_pos[target_idx] - width/2, len_original, f'{len_original:.1f}', ha='center', va='bottom', color='black', fontsize=9)
    ax3.text(x_pos[target_idx] + width/2, len_optimized, f'{len_optimized:.1f}', ha='center', va='bottom', color=color3, fontsize=9, fontweight='bold')

    # Formatting and Grid
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(categories)
    ax1.grid(True, axis='y', alpha=0.3)
    
    # Calculate and display percentage change for path length
    if len_original > 0:
        diff_percent = ((len_optimized - len_original) / len_original) * 100
        # Position text slightly above the highest bar
        ax3.text(x_pos[target_idx], max(len_original, len_optimized) * 1.07, 
                 f"Change: {diff_percent:+.2f}%", ha='center', color='black', fontweight='bold')

    # --- Legend Configuration ---
    # Place a single unified legend above the plot area (horizontally centered)
    fig.legend([bars1, bars2, bars3_orig, bars3_opt], 
               ["Count", "Time (s)", "Orig. Length", "Opt. Length"], 
               loc="lower center", 
               bbox_to_anchor=(0.5, 0.88), # Anchored relative to the figure
               ncol=4, # Horizontal layout
               frameon=False)

    # Adjust layout to prevent clipping of the top legend
    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    plt.show()

def clear_graph_attributes(planner):
    """
    Cleans up graph node attributes 'r', 'fixed_k', and 'P2n'.
    Run this before optimization to ensure a fresh start.
    """
    graph = planner.graph
    for node_name in graph.nodes:
        node_attrs = graph.nodes[node_name]
        node_attrs.pop('r', None)
        node_attrs.pop('fixed_k', None)
        node_attrs.pop('P2n', None)
