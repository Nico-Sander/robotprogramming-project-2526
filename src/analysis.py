import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display

def create_interactive_viewer(data_dict, draw_callback):
    """
    Generates an interactive dropdown viewer for any set of environments.
    
    Args:
        data_dict (dict): Dictionary containing the environment data.
        draw_callback (func): Function with signature (name, item) that handles 
                              the actual matplotlib plotting.
    """
    plot_cache = {}
    
    # 1. Loading Indicator
    progress = widgets.Label(value="Pre-rendering plots... (0/{})".format(len(data_dict)))
    display(progress)
    
    # 2. Pre-render loop
    count = 0
    for name, item in data_dict.items():
        out = widgets.Output()
        with out:
            # Execute the user-defined drawing logic
            draw_callback(name, item)
            plt.show() # Ensure the plot is flushed to the widget
        plot_cache[name] = out
        
        count += 1
        progress.value = f"Pre-rendering plots... ({count}/{len(data_dict)})"

    # 3. Clean up loading bar
    progress.layout.display = 'none'

    # 4. Setup Widget Controls
    dropdown = widgets.Dropdown(
        options=list(data_dict.keys()),
        value=list(data_dict.keys())[0],
        description='Select Env:',
    )

    plot_container = widgets.VBox([plot_cache[dropdown.value]])

    def on_change(change):
        if change['type'] == 'change' and change['name'] == 'value':
            plot_container.children = [plot_cache[change['new']]]

    dropdown.observe(on_change)
    display(dropdown, plot_container)

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
