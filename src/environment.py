from shapely.geometry import Polygon, Point
from planners.IPPerfMonitor import IPPerfMonitor
import numpy as np
import matplotlib.pyplot as plt
from shapely import plotting
import networkx as nx

X_LIMITS = [0.0, 20.0]
Y_LIMITS = [0.0, 20.0]

def fillet_corner_if_exists(poly: Polygon, corner_xy, r: float, n_arc: int = 24, tol: float = 1e-6) -> Polygon:
    """
    Utility function to round a specific corner of a Polygon.
    Used for creating benchmark environments with rounded obstacles.
    """
    coords = list(poly.exterior.coords)[:-1]
    cx, cy = corner_xy

    # 1. Find the index of the vertex matching corner_xy
    corner_idx = None
    for i, (x, y) in enumerate(coords):
        if abs(x - cx) <= tol and abs(y - cy) <= tol:
            corner_idx = i
            break

    if corner_idx is None:
        return poly

    # 2. Identify neighbors
    n = len(coords)
    Pm = np.array(coords[(corner_idx - 1) % n], float)
    P  = np.array(coords[corner_idx], float)
    Pp = np.array(coords[(corner_idx + 1) % n], float)

    # 3. Calculate tangent geometry
    u1 = (Pm - P); u1 = u1 / np.linalg.norm(u1)
    u2 = (Pp - P); u2 = u2 / np.linalg.norm(u2)

    ang = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
    d = r / np.tan(ang / 2.0)

    T1 = P + u1 * d
    T2 = P + u2 * d

    # 4. Generate Arc Points
    bis = (u1 + u2); bis = bis / np.linalg.norm(bis)
    h = r / np.sin(ang / 2.0)
    C = P + bis * h

    a1 = np.arctan2(T1[1] - C[1], T1[0] - C[0])
    a2 = np.arctan2(T2[1] - C[1], T2[0] - C[0])
    da = (a2 - a1 + np.pi) % (2*np.pi) - np.pi

    arc = []
    for k in range(n_arc + 1):
        a = a1 + da * (k / n_arc)
        arc.append((C[0] + r * np.cos(a), C[1] + r * np.sin(a)))

    # 5. Reconstruct Polygon
    new_coords = []
    for i in range(n):
        if i == corner_idx:
            new_coords.append((T1[0], T1[1]))
            new_coords.extend(arc[1:-1])
            new_coords.append((T2[0], T2[1]))
        else:
            new_coords.append(coords[i])

    return Polygon(new_coords)

def construct_benchmark_environments():
    """
    Generates a dictionary of benchmark environments.
    Each environment contains:
    - 'env': Dictionary of Shapely Polygons (Obstacles)
    - 'smooth_path': List of coordinate tuples representing the ideal path
    """
    env_dict = dict()

    ## Environment 1
    env_1 = dict()
    env_1["obs_1"] = Polygon([(5.0, 0.0), (8.0, 0.0), (8.0, 9.0), (5.0, 9.0)])
    env_1["obs_2"] = Polygon([(11.0, 0.0), (16.0, 0.0), (16.0, 6.0), (11.0, 6.0)])
    env_1["obs_3"] = fillet_corner_if_exists(
        Polygon([(10.0, 8.0), (15.0, 8.0), (15.0, 20.0), (10.0, 20.0)]), (10.0, 8.0), 3
    )
    start = (2.0, 2.0)
    goal = (18.0, 2.0)
    solution_positions = [start, (4.5, 9.5), (9.0, 10.0), (12.0, 7.0), (16.0, 6.5), goal]

    env_dict["1"] = {"env": env_1, "smooth_path": solution_positions}

    ## Environment 2
    env_2 = dict()
    env_2["obs_1"] = Polygon([(0.0, 2.0),(15.0, 2.0), (15.5, 1.5), (16.0, 2.0), (16.0, 3.0), (0.0, 3.0)])
    env_2["obs_2"] = Polygon([(10.0, 5.0), (20.0, 5.0), (20.0, 6.0), (10.0, 6.0)])
    env_2["obs_3"] = Polygon([(0.0, 8.0), (16.0, 8.0), (16.0, 10.0), (0.0, 10.0)])
    env_2["obs_4"] = Polygon([(2.0, 11.0), (20.0, 11.0), (20.0, 12.0), (2.0, 12.0)])
    env_2["obs_5"] = Polygon([(0.0, 14.0), (5.0, 14.0), (5.0, 15.0), (0.0, 15.0)])
    env_2["obs_6"] = Polygon([(4.0, 17.0), (20.0, 17.0), (20.0, 18.0), (4.0, 18.0)])
    env_2["obs_7"] = Polygon([(17.2, 0), (20.0, 0), (20.0, 5.0), (17.2, 5.0)])
    start = (3.0, 1.0)
    goal = (10.0, 19.0)
    solution_positions = [start, (17.0, 1.0), (17.0, 4.0), (9.5, 4.0), (9.5, 7.0), (17.0, 7.0), (17.0, 10.5), 
                        (1.0, 10.5), (1.0, 13.0), (6.0, 13.0), (6.0, 16.0), (3.0, 16.0), (3.0, 19.0), goal]

    env_dict["2"] = {"env": env_2, "smooth_path": solution_positions}

    ## Environment 3
    env_3 = dict()
    env_3["obs_1"] = Polygon([(6.0, 17.0), (7.0, 15.0), (20.0, 18.0), (20.0, 20.0)])
    env_3["obs_2"] = Polygon([(0.0, 10.2), (0.0, 13.0), (6.0, 13.0), (6.0, 10.2)])
    env_3["obs_3"] = fillet_corner_if_exists(
        Polygon([(3.0, 7.0), (3.0, 9.0), (8.0, 9.0), (8.0, 7.0)]), (3.0, 7.0), 2   
    )
    env_3["obs_4"] = Polygon([(8.0, 1.0), (7.0, 3.0), (12.0, 8.0), (13.0, 6.0)])
    env_3["obs_5"] = Polygon([(15.0, 8.0), (15.0, 15.0), (8.0, 15.0), (8.0, 8.0), (9.5, 8.0), (9.5, 10.0), 
                              (9.0, 10.0), (9.0, 13.0), (13.0, 13.0), (13.0, 8.0)])
    start = (7.0, 19.0)
    goal = (10.0, 12.0)
    solution_positions = [start, (4.0, 15.0), (7.0, 13.0), (7.0, 10.0), (2.0, 10.0), (2.0, 3.0), (12.0, 9.0), goal]

    env_dict["3"] = {"env": env_3, "smooth_path": solution_positions}

    ## Environment 4
    env_4 = dict()
    env_4["obs_1"] = fillet_corner_if_exists(
        Polygon([(0, 9.5), (6, 12.5), (5, 7), (13, 13), (17, 0), (0, 0)]), (13.0, 13.0), 4.5
    )
    env_4["obs_2"] = fillet_corner_if_exists(
        Polygon([(0, 12.0), (8, 16.0), (6.5, 11.5), (14, 11.0), (20.0, 0), (20, 20), (0, 20)]), (8.0, 16.0), 1
    )
    start = (1.0, 11.25)
    goal = (18.25, 1.0)
    solution_positions = [start, (6, 13), (6.5, 10), (13.5, 10.5), goal]

    env_dict["4"] = {"env": env_4, "smooth_path": solution_positions}

    return env_dict

class CollisionChecker(object):
    """
    Handles all collision detection against the environment (Shapely Polygons).
    Also includes visualization methods.
    """
    def __init__(self, scene, limits=[X_LIMITS, Y_LIMITS]):
        self.scene = scene
        self.limits = limits

    def getDim(self):
        return 2

    def getEnvironmentLimits(self):
        return list(self.limits)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """ Checks if a single 2D point is inside any obstacle. """
        assert (len(pos) == self.getDim())
        point = Point(pos[0], pos[1])
        for key, value in self.scene.items():
            if value.intersects(point):
                return True
        return False

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        """ 
        Checks if a straight line segment collides with obstacles.
        Uses discrete sampling (interpolation).
        """
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        p12 = p2-p1
        k = 40 # Number of test points
        
        for i in range(k):
            testPoint = p1 + (i+1)/k*p12
            if self.pointInCollision(testPoint):
                return True
        
        return False

    @IPPerfMonitor
    def curveInCollision(self, startPos, controlPos, endPos, steps=50):
        """ 
        Checks if a Quadratic Bezier curve is in collision.
        The curve is defined by Start (S), Control Point (P2n), and End (E).
        """
        assert (len(startPos) == self.getDim())
        assert (len(controlPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        S = np.array(startPos)
        P2n = np.array(controlPos)
        E = np.array(endPos)

        # Sample points along the curve
        t_values = np.linspace(0, 1, steps)

        for t in t_values:
            # Quadratic Bezier Formula: B(t) = (1-t)^2 * S + 2*(1-t)*t * P2n + t^2 * E
            p_curve = (1-t)**2 * S + 2*(1-t)*t * P2n + t**2 * E
            
            if self.pointInCollision(p_curve):
                return True
        
        return False
    
    # --- VISUALIZATION METHODS ---

    def create_axes(self, figsize: tuple = (10,10)) -> plt.Axes:
        _, ax = plt.subplots(figsize=figsize)
        ax.set_xlim(self.limits[0][0], self.limits[0][1])
        ax.set_ylim(self.limits[1][0], self.limits[1][1])
        ax.set_xticks(range(0, int(self.limits[0][1])))
        ax.set_yticks(range(0, int(self.limits[1][1])))
        ax.grid()
        return ax

    def draw_enviroments(self, ax: plt.Axes = None) -> plt.Axes:
        if not ax:
            ax = self.create_axes()
        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, color='red', ax=ax)
        return ax

    def draw_path(self, path_positions: list, ax: plt.Axes = None) -> plt.Axes:
        """ Draws a simple straight-line path. """
        if not ax:
            ax = self.create_axes()
        
        x_val, y_val = zip(*path_positions)
        ax.plot(x_val, y_val, color="k")
        ax.scatter(x_val[1:-1], y_val[1:-1], color="k", marker="o")

        # Start (Green)
        ax.scatter(x_val[0], y_val[0], marker="o", color="lightgreen", s=300)
        ax.text(x_val[0], y_val[0], "S", fontweight="heavy", size=16, ha="center", va="center")

        # Goal (Blue)
        ax.scatter(x_val[-1], y_val[-1], marker="o", color="lightblue", s=300)
        ax.text(x_val[-1], y_val[-1], "G", fontweight="heavy", size=16, ha="center", va="center")

        return ax

    def draw_optimized_path(self, optimized_results, planner, ax=None):
        """
        Visualizes the optimized G1-smooth path.
        
        Fetches the Control Points (P2n) directly from the planner graph
        and recalculates the curve start (S) and end (E) points on the fly
        to draw the Bezier curves and connection lines.
        """
        if not ax:
            ax = self.create_axes()

        # 1. Setup Data
        node_names = [x[0] for x in optimized_results]
        raw_positions = retrieve_path_positions(planner.graph, node_names)
        positions = [np.array(p) for p in raw_positions]

        # 2. Draw Original Path (Grey dashed)
        orig_x = [p[0] for p in positions]
        orig_y = [p[1] for p in positions]
        ax.plot(orig_x, orig_y, "gray", linestyle="--", label="Original")
        
        # 3. Retrieve Control Points (P2n) from Graph
        p2n_list = []
        for name in node_names:
            # Fallback to node position if P2n missing (e.g. for Start/End)
            p2n_list.append(planner.graph.nodes[name].get('P2n', planner.graph.nodes[name]['pos']))
        
        # Draw the "New Straight Path" (The Control Polygon / Hull)
        # This is the dashed black line connecting the P2n points
        p2n_x = [p[0] for p in p2n_list]
        p2n_y = [p[1] for p in p2n_list]
        ax.plot(p2n_x, p2n_y, "k--", alpha=0.8, label="New Straight Path")

        # 4. Draw Curves and Segments
        last_endpoint = positions[0] 
        
        # Draw Start Node
        ax.scatter(positions[0][0], positions[0][1], marker="o", color="lightgreen", s=300, zorder=5)
        ax.text(positions[0][0], positions[0][1], "S", fontweight="heavy", ha="center", va="center")

        # Iterate through intermediate nodes to draw curves
        for i in range(1, len(positions)-1):
            P_prev = p2n_list[i-1]
            P_curr = p2n_list[i]
            P_next = p2n_list[i+1]
            r = planner.graph.nodes[node_names[i]].get('r', 0)
            fixed_k = planner.graph.nodes[node_names[i]].get('fixed_k')

            # --- Recalculate S and E (Same logic as in optimizer) ---
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
            
            vec_in = P_prev - P_curr
            vec_out = P_next - P_curr
            S = P_curr + li * vec_in
            E = P_curr + lo * vec_out

            # --- Drawing ---
            
            # Control Point (P2n)
            ax.scatter(P_curr[0], P_curr[1], marker="*", color="red", s=100, zorder=4)
            # R-value text
            ax.text(positions[i][0], positions[i][1], f"r={r:.2f}", color="blue", fontsize=8, ha="center")
            
            # Straight Connection (From previous E to current S)
            ax.plot([last_endpoint[0], S[0]], [last_endpoint[1], S[1]], 'k-')
            
            if r > 0:
                # Draw Bezier Curve (S -> P2n -> E)
                t_steps = np.linspace(0, 1, 20)
                curve_pts = [(1-t)**2 * S + 2*(1-t)*t * P_curr + t**2 * E for t in t_steps]
                
                cx = [p[0] for p in curve_pts]
                cy = [p[1] for p in curve_pts]
                ax.plot(cx, cy, 'b-', linewidth=2)
                
                last_endpoint = E
            else:
                # No curve, just connect to the Control Point
                last_endpoint = P_curr

        # Final Segment to Goal
        ax.plot([last_endpoint[0], positions[-1][0]], [last_endpoint[1], positions[-1][1]], 'k-')
        
        # Draw Goal Node
        ax.scatter(positions[-1][0], positions[-1][1], marker="o", color="lightblue", s=300, zorder=5)
        ax.text(positions[-1][0], positions[-1][1], "G", fontweight="heavy", ha="center", va="center")

        return ax

# --- Helper Functions ---

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