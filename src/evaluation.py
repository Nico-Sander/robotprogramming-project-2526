import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

from planners.IPPerfMonitor import IPPerfMonitor


def capture_performance_metrics(planner, node_names):
    """
    Aggregates execution statistics and geometric path metrics for the current planning state.

    This function serves as a summary utility that bridges profiling data with path analysis. It queries the 
    `IPPerfMonitor` class to extract runtime performance data—specifically focusing on the frequency and duration 
    of collision checks (points, lines, and curves). Simultaneously, it computes the physical length of the 
    generated solution, providing a direct comparison between the original piecewise-linear path and the 
    optimized G1-continuous trajectory.

    Parameters:
        planner (object): The planner instance containing the graph data structure and the active collision checker.
        node_names (list): A sequence of strings representing the node identifiers in the path to be evaluated.

    Returns:
        tuple: A triplet (stats, len_orig, len_opt) containing:
               - stats (dict): A dictionary mapping function names (e.g., 'lineInCollision') to a metric dict 
                 {'count': int, 'time': float}.
               - len_orig (float): The total Euclidean length of the unoptimized (straight-line) path.
               - len_opt (float): The total length of the optimized path, accounting for Bezier curves.

    Side Effects:
        - Accesses the static `IPPerfMonitor.dataFrame()` method, which processes accumulated profiling logs.
        - Triggers geometric calculations via `calculate_path_length` but does not modify the graph state.
    """
    # 1. Extract Performance Stats
    # Retrieve the full log of monitored function calls as a pandas DataFrame
    df = IPPerfMonitor.dataFrame()
    stats = {}
    
    # Define the specific collision check methods relevant to the optimization process
    target_funcs = ['lineInCollision', 'curveInCollision', 'pointInCollision']
    
    for f in target_funcs:
        # Filter the DataFrame for the current function name
        f_data = df[df['name'] == f]
        
        # Aggregate the data if records exist, otherwise default to zero
        if not f_data.empty:
            stats[f] = {
                'count': len(f_data),           # Total number of times the function was called
                'time': f_data['time'].sum()    # Total cumulative execution time in seconds
            }
        else:
            stats[f] = {'count': 0, 'time': 0.0}

    # 2. Calculate Path Lengths
    # Compute the baseline length: The sum of straight lines connecting the nodes (Euclidean distance)
    len_orig = calculate_path_length(planner, node_names, use_curves=False)
    
    # Compute the optimized length: Includes the arc lengths of Bezier curves and shortened straight segments
    len_opt = calculate_path_length(planner, node_names, use_curves=True)

    return stats, len_orig, len_opt

def retrieve_path_positions(graph: nx.Graph, node_names: list) -> list:
    """
    Extracts the geometric coordinates for a sequence of nodes from the graph data structure.

    Parameters:
        graph (nx.Graph): The NetworkX graph object where nodes and their attributes (specifically 'pos') are stored.
        node_names (list): A list of strings representing the ordered sequence of nodes (e.g., a path found by the planner).

    Returns:
        list: A list of coordinate tuples or arrays (e.g., `[(x, y), ...]`) corresponding to the input node sequence.

    Side Effects:
        None.
    """
    # Iterate through the list of node names and extract the 'pos' attribute from the graph for each one
    return [graph.nodes[name]["pos"] for name in node_names]

def calculate_path_length(planner, path: list, use_curves: bool = True) -> float:
    """
    Computes the precise length of the robot's trajectory, either as a simple piecewise-linear path 
    or as a G1-continuous smoothed path.

    This function operates in two distinct modes:
    1. **Linear Mode (`use_curves=False`)**: Calculates the sum of Euclidean distances between the 
       original waypoints. This serves as the baseline "jagged" length.
    2. **Curved Mode (`use_curves=True`)**: Reconstructs the exact geometry of the optimized path 
       stored in the planner's graph. It accounts for the straight connecting segments between curves 
       and approximates the arc length of the Quadratic Bezier curves using discrete summation.
    
    The curved calculation mirrors the logic used in the optimization routine (re-deriving tangent points 
    S and E based on stored `P2n` control points and `r` values) to ensure the measured length matches 
    the visualized path exactly.

    Parameters:
        planner (object): The planner instance containing the graph with node attributes ('pos', 'r', 'P2n', 'fixed_k').
        path (list): A list of node identifiers (strings) representing the sequence of the path.
        use_curves (bool): Flag to toggle between linear baseline (False) and optimized curved length (True).

    Returns:
        float: The total calculated length of the path in the environment's unit (usually meters).

    Side Effects:
        None.
    """
    def get_pos(name):
        # Helper to extract the numpy array position of a node
        return np.array(planner.graph.nodes[name]['pos'])

    # Mode 1: Straight Line (Original)
    # Simple summation of Euclidean distances between consecutive nodes in the path list
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
    # Retrieve the 'Virtual Control Points' (P2n) calculated during optimization.
    # Fallback to 'pos' (original position) if P2n is missing (e.g., for Start/Goal).
    p2n_list = []
    for name in path:
        p2n_list.append(planner.graph.nodes[name].get('P2n', planner.graph.nodes[name]['pos']))
        
    last_endpoint = p2n_list[0] # Start pos (Start node has no previous curve)
    
    # Iterate through intermediate nodes to measure curve and connection lengths
    for i in range(1, len(path) - 1):
        node_name = path[i]
        r = planner.graph.nodes[node_name].get('r', 0.0)
        fixed_k = planner.graph.nodes[node_name].get('fixed_k')
        
        P_prev = p2n_list[i-1]
        P_curr = p2n_list[i]
        P_next = p2n_list[i+1]
        
        # If the node has a valid smoothing radius, calculate the Bezier arc
        if r > 0:
            # --- Recalculate Geometry ---
            # Re-derive tangent points (S, E) using the same logic as the optimizer
            d_in = np.linalg.norm(P_curr - P_prev)
            d_out = np.linalg.norm(P_next - P_curr)
            
            # Determine asymmetry factors (li/lo) based on 'fixed_k' or dynamic symmetry
            if fixed_k is not None:
                li, lo = r, r * fixed_k
            else:
                # Dynamic Metric Symmetry logic
                if d_in <= d_out:
                    dist = r * d_in
                    li = r
                    lo = dist / d_out if d_out > 0 else 0
                else:
                    dist = r * d_out
                    lo = r
                    li = dist / d_in if d_in > 0 else 0
            
            # Calculate absolute tangent points
            S = P_curr + li * (P_prev - P_curr)
            E = P_curr + lo * (P_next - P_curr)
            
            # 1. Add Straight Segment Length (Last E -> Current S)
            # This is the straight line connecting the end of the previous curve to the start of this one.
            total_length += np.linalg.norm(S - last_endpoint)
            
            # 2. Add Curve Length (Discrete Summation)
            # Approximate the arc length of the Bezier curve by sampling 20 points along it.
            steps = 20
            t_vals = np.linspace(0, 1, steps)
            
            # Vectorized Bezier calculation: B(t) = (1-t)^2*S + 2(1-t)t*P + t^2*E
            term1 = np.outer((1 - t_vals)**2, S)
            term2 = np.outer(2 * (1 - t_vals) * t_vals, P_curr)
            term3 = np.outer(t_vals**2, E)
            curve_points = term1 + term2 + term3
            
            # Calculate sum of Euclidean distances between sampled points
            diffs = curve_points[1:] - curve_points[:-1]
            segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
            total_length += np.sum(segment_lengths)
            
            last_endpoint = E
        else:
            # No smoothing: Treat as a sharp corner (Line to node, then Line from node)
            total_length += np.linalg.norm(P_curr - last_endpoint)
            last_endpoint = P_curr

    # Final Segment (Last E -> Goal)
    # Add the remaining distance from the end of the last curve to the actual Goal node.
    total_length += np.linalg.norm(p2n_list[-1] - last_endpoint)
    
    return total_length

def plot_performance_data(stats, len_original, len_optimized):
    """
    Visualizes the computational cost and physical quality of the path planning solution using a multi-axis bar chart.

    This function creates a comprehensive matplotlib figure that aggregates three distinct types of data onto a 
    single timeline:
    1. **Operation Counts (Left Axis)**: Displays how many times collision checks (Point, Line, Curve) were invoked.
    2. **Execution Time (Right Axis #1)**: Shows the total time spent in those operations. Includes logic to 
       automatically switch units between seconds and milliseconds for readability.
    3. **Path Quality (Right Axis #2)**: Compares the Euclidean length of the original path versus the optimized 
       result, calculating the percentage improvement.

    The complex multi-axis setup (twinx) allows distinct magnitudes (e.g., thousands of checks vs. milliseconds of time) 
    to be compared side-by-side without scaling artifacts.

    Parameters:
        stats (dict): Performance dictionary captured from `IPPerfMonitor`. Keys (e.g., 'lineInCollision') map 
                      to dicts containing 'count' and 'time'.
        len_original (float): The length of the initial unoptimized path (meters).
        len_optimized (float): The length of the final smoothed path (meters).

    Returns:
        None: Displays the plot window via `plt.show()` but returns no value.

    Side Effects:
        - Creates and displays a matplotlib figure.
        - Adjusts global plot settings (subplots_adjust) to accommodate the custom legend position.
    """
    
    # Initialize the figure
    fig, ax1 = plt.subplots(figsize=(12, 7))
    plt.title(f"Performance Metrics", pad=40)

    # Define categories for the X-axis
    categories = ['Point Checks', 'Line Checks', 'Curve Checks', 'Path Length']
    x_pos = np.arange(len(categories))
    width = 0.25  # Width of the bars

    # --- Pre-calculate Data for Time Scaling ---
    # Extract raw timing values from the stats dictionary, defaulting to 0.0 if missing
    pt_time = stats.get('pointInCollision', {}).get('time', 0.0)
    ln_time = stats.get('lineInCollision', {}).get('time', 0.0)
    crv_time = stats.get('curveInCollision', {}).get('time', 0.0)
    raw_times = [pt_time, ln_time, crv_time, 0]

    # Determine Scale (Smart Logic)
    # Check if the longest operation took less than a second to decide on the time unit
    max_time = max(raw_times)
    
    if max_time < 1.0:
        # Scale to Milliseconds if operations are fast
        final_times = [t * 1000 for t in raw_times]
        time_label = "Time (ms)"
        # integer formatting (no decimals) for cleaner labels
        time_fmt_str = "{:.0f}" 
    else:
        # Keep as Seconds if operations are slow
        final_times = raw_times
        time_label = "Time (seconds)"
        # 2 decimal places + 's' suffix for precision
        time_fmt_str = "{:.2f}s" 

    # --- Axis 1 (Left): Operation Counts ---
    color1 = 'tab:blue'
    ax1.set_ylabel('Number of Calls', color=color1, fontweight='bold')
    
    # Extract counts from stats
    pt_count = stats.get('pointInCollision', {}).get('count', 0)
    ln_count = stats.get('lineInCollision', {}).get('count', 0)
    crv_count = stats.get('curveInCollision', {}).get('count', 0)
    counts = [pt_count, ln_count, crv_count, 0]
    
    # Plot bars offset to the left
    bars1 = ax1.bar(x_pos - width, counts, width, label='Count', color=color1, alpha=0.7)
    ax1.tick_params(axis='y', labelcolor=color1)
    
    # Annotate bars with the exact count
    for rect in bars1:
        height = rect.get_height()
        if height > 0:
            ax1.text(rect.get_x() + rect.get_width()/2., height,
                     f'{int(height)}', ha='center', va='bottom', color=color1)

    # --- Axis 2 (Right #1): Execution Time (Smart Scaled) ---
    # Create a twin axis sharing the same x-axis but with independent y-scale
    ax2 = ax1.twinx() 
    color2 = 'tab:orange'
    ax2.set_ylabel(time_label, color=color2, fontweight='bold')
    
    # Plot Time Bars using the scaled 'final_times' centrally aligned on ticks
    bars2 = ax2.bar(x_pos, final_times, width, label='Time', color=color2, alpha=0.7)
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Annotate bars using the dynamic format string (ms vs s)
    for rect in bars2:
        height = rect.get_height()
        if height > 0:
            label_text = time_fmt_str.format(height)
            ax2.text(rect.get_x() + rect.get_width()/2., height,
                     label_text, ha='center', va='bottom', color=color2, fontsize=9)

    # --- Axis 3 (Right #2): Path Length ---
    # Create a third axis by offsetting the spine to the right (outside the plot area)
    ax3 = ax1.twinx()
    ax3.spines["right"].set_position(("axes", 1.15)) 
    color3 = 'tab:purple'
    ax3.set_ylabel('Path Length', color=color3, fontweight='bold')
    
    # Plot comparison bars at the specific index for 'Path Length'
    target_idx = 3 
    bars3_orig = ax3.bar(x_pos[target_idx] - width/2, len_original, width/2, color='gray', label='Original Len')
    bars3_opt  = ax3.bar(x_pos[target_idx] + width/2, len_optimized, width/2, color=color3, label='Optimized Len')
    
    ax3.tick_params(axis='y', labelcolor=color3)
    
    # Annotate lengths and emphasize the optimized value
    ax3.text(x_pos[target_idx] - width/2, len_original, f'{len_original:.1f}', ha='center', va='bottom', color='black', fontsize=9)
    ax3.text(x_pos[target_idx] + width/2, len_optimized, f'{len_optimized:.1f}', ha='center', va='bottom', color=color3, fontsize=9, fontweight='bold')

    # Formatting and Grid
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(categories)
    ax1.grid(True, axis='y', alpha=0.3)
    
    # Calculate and display the percentage improvement above the bars
    if len_original > 0:
        diff_percent = ((len_optimized - len_original) / len_original) * 100
        ax3.text(x_pos[target_idx], max(len_original, len_optimized) * 1.07, 
                 f"Change: {diff_percent:+.2f}%", ha='center', color='black', fontweight='bold')

    # --- Legend Configuration ---
    # Manually constructing a single legend for multiple axes.
    # Using the dynamic 'time_label' variable ensures the legend matches the axis units.
    fig.legend([bars1, bars2, bars3_orig, bars3_opt], 
               ["Count", time_label, "Orig. Length", "Opt. Length"], 
               loc="lower center", 
               bbox_to_anchor=(0.5, 0.88),
               ncol=4, 
               frameon=False)

    plt.tight_layout()
    # Adjust top margin to make room for the title and floating legend
    plt.subplots_adjust(top=0.85)
    plt.show()

def clear_graph_attributes(planner):
    """
    Resets the graph state by removing optimization-specific attributes from all nodes.

    This utility function is essential for ensuring isolated tests during iterative optimization or parameter sweeps. 
    It iterates through every node in the planner's graph and strips away the persistent data generated by 
    previous "Fly-by" runs—specifically the smoothing radius ($r$), the asymmetry factor ($fixed_k$), and the 
    virtual control points ($P_{2n}$). Without this cleanup, subsequent optimization calls might read stale 
    geometric data, leading to incorrect calculations or visualizations.

    Parameters:
        planner (object): The planner instance containing the NetworkX graph object (`planner.graph`) to be cleaned.

    Returns:
        None

    Side Effects:
        - Modifies `planner.graph` in-place by deleting keys ('r', 'fixed_k', 'P2n') from the attribute 
          dictionary of every node.
    """
    graph = planner.graph
    
    # Iterate through every node in the graph to purge optimization artifacts
    for node_name in graph.nodes:
        node_attrs = graph.nodes[node_name]
        
        # Remove smoothing radius 'r' if it exists; defaults to None if missing
        node_attrs.pop('r', None)
        
        # Remove asymmetry factor 'fixed_k' used for individual corner optimization
        node_attrs.pop('fixed_k', None)
        
        # Remove the calculated virtual control point 'P2n'
        node_attrs.pop('P2n', None)
