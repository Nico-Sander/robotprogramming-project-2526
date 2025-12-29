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
    Ersetzt eine Ecke (Vertex) des Polygons durch einen Kreisbogen (Fillet).
    - corner_xy: gewünschte Ecke als Koordinate, z.B. (13, 13)
    - r: Radius des Kreisbogens
    - n_arc: Anzahl Stützpunkte des Kreisbogens (mehr = glatter)
    - tol: Toleranz beim Finden des Vertex (Float-Vergleich)

    Wenn corner_xy kein Vertex ist, wird das Polygon unverändert zurückgegeben.
    """

    # Polygon-Koordinaten (letzter Punkt ist identisch zum ersten -> entfernen)
    coords = list(poly.exterior.coords)[:-1]
    cx, cy = corner_xy

    # 1) Vertex-Index der gewünschten Ecke suchen
    corner_idx = None
    for i, (x, y) in enumerate(coords):
        if abs(x - cx) <= tol and abs(y - cy) <= tol:
            corner_idx = i
            break

    # Ecke existiert nicht -> nichts tun
    if corner_idx is None:
        return poly

    # 2) Nachbarpunkte der Ecke bestimmen (vorheriger und nächster Vertex)
    n = len(coords)
    Pm = np.array(coords[(corner_idx - 1) % n], float)
    P  = np.array(coords[corner_idx], float)
    Pp = np.array(coords[(corner_idx + 1) % n], float)

    # 3) Richtungsvektoren entlang der beiden Kanten (von der Ecke weg)
    u1 = (Pm - P); u1 = u1 / np.linalg.norm(u1)
    u2 = (Pp - P); u2 = u2 / np.linalg.norm(u2)

    # 4) Winkel zwischen den Kanten und Tangential-Abstand berechnen
    ang = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
    d = r / np.tan(ang / 2.0)

    # 5) Tangentialpunkte (wo der Kreisbogen an die Kanten anschließt)
    T1 = P + u1 * d
    T2 = P + u2 * d

    # 6) Kreismittelpunkt: liegt auf der Winkelhalbierenden
    bis = (u1 + u2)
    bis = bis / np.linalg.norm(bis)
    h = r / np.sin(ang / 2.0)
    C = P + bis * h

    # 7) Kreisbogenpunkte von T1 nach T2 generieren
    a1 = np.arctan2(T1[1] - C[1], T1[0] - C[0])
    a2 = np.arctan2(T2[1] - C[1], T2[0] - C[0])

    # „kürzere“ Drehrichtung für den Bogen wählen
    da = (a2 - a1 + np.pi) % (2*np.pi) - np.pi

    arc = []
    for k in range(n_arc + 1):
        a = a1 + da * (k / n_arc)
        arc.append((C[0] + r * np.cos(a), C[1] + r * np.sin(a)))

    # 8) Polygon neu zusammensetzen: Ecke P durch T1 -> Bogen -> T2 ersetzen
    new_coords = []
    for i in range(n):
        if i == corner_idx:
            new_coords.append((T1[0], T1[1]))
            new_coords.extend(arc[1:-1])     # Endpunkte weglassen (sonst doppelt)
            new_coords.append((T2[0], T2[1]))
        else:
            new_coords.append(coords[i])

    return Polygon(new_coords)

def construct_benchmark_environments():
    env_dict = dict()

    ## Environment 1
    env_1 = dict()
    env_1["obs_1"] = Polygon([(5.0, 0.0), (8.0, 0.0), (8.0, 9.0), (5.0, 9.0)])
    env_1["obs_2"] = Polygon([(11.0, 0.0), (16.0, 0.0), (16.0, 6.0), (11.0, 6.0)])
    env_1["obs_3"] = fillet_corner_if_exists(
        Polygon([(10.0, 8.0), (15.0, 8.0), (15.0, 20.0), (10.0, 20.0)]),
        (10.0, 8.0),
        3
    )

    start = (2.0, 2.0)
    goal = (18.0, 2.0)

    solution_positions = [start, (4.5, 9.5), (9.0, 10.0), (12.0, 7.0), (16.0, 6.5), goal]

    env_dict["1"] = dict()
    env_dict["1"]["env"] = env_1
    env_dict["1"]["smooth_path"] = solution_positions
    # pprint.pprint(env_dict)


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


    env_dict["2"] = dict()
    env_dict["2"]["env"] = env_2
    env_dict["2"]["smooth_path"] = solution_positions
    # pprint.pprint(env_dict)


    # Environment 3
    env_3 = dict()
    env_3["obs_1"] = Polygon([(6.0, 17.0), (7.0, 15.0), (20.0, 18.0), (20.0, 20.0)])
    env_3["obs_2"] = Polygon([(0.0, 10.2), (0.0, 13.0), (6.0, 13.0), (6.0, 10.2)])
    env_3["obs_3"] = fillet_corner_if_exists(
        Polygon([(3.0, 7.0), (3.0, 9.0), (8.0, 9.0), (8.0, 7.0)]),
        (3.0, 7.0),
        2   
    )
    env_3["obs_4"] = Polygon([(8.0, 1.0), (7.0, 3.0), (12.0, 8.0), (13.0, 6.0)])
    env_3["obs_5"] = Polygon([(15.0, 8.0), (15.0, 15.0), (8.0, 15.0), (8.0, 8.0), (9.5, 8.0), (9.5, 10.0), (9.0, 10.0), (9.0, 13.0), (13.0, 13.0), (13.0, 8.0)])


    start = (7.0, 19.0)
    goal = (10.0, 12.0)

    solution_positions = [start, (4.0, 15.0), (7.0, 13.0), (7.0, 10.0), (2.0, 10.0), (2.0, 3.0), (12.0, 9.0), goal]

    env_dict["3"] = dict()
    env_dict["3"]["env"] = env_3
    env_dict["3"]["smooth_path"] = solution_positions
    # pprint.pprint(env_dict)

    ## Environment 4
    env_4 = dict()
    env_4["obs_1"] = fillet_corner_if_exists(
        Polygon([(0, 9.5), (6, 12.5), (5, 7), (13, 13), (17, 0), (0, 0)]),
        (13.0, 13.0),
        4.5
    )
    env_4["obs_2"] = fillet_corner_if_exists(
        Polygon([(0, 12.0), (8, 16.0), (6.5, 11.5), (14, 11.0), (20.0, 0), (20, 20), (0, 20)]),
        (8.0, 16.0),
        1
    )

    start = (1.0, 11.25)
    goal = (18.25, 1.0)

    solution_positions = [start, (6, 13), (6.5, 10), (13.5, 10.5), goal]

    env_dict["4"] = dict()
    env_dict["4"]["env"] = env_4
    env_dict["4"]["smooth_path"] = solution_positions
    # pprint.pprint(env_dict)

    return env_dict

class CollisionChecker(object):

    def __init__(self, scene, limits=[X_LIMITS, Y_LIMITS]):
        self.scene = scene
        self.limits = limits

    def getDim(self):
        """ Return dimension of Environment"""
        return 2

    def getEnvironmentLimits(self):
        """ Return limits of Environment"""
        return list(self.limits)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """ Return whether a configuration is
        inCollision -> True
        Free -> False """
        assert (len(pos) == self.getDim())
        for key, value in self.scene.items():
            if value.intersects(Point(pos[0], pos[1])):
                return True
        return False

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        """ Check whether a line from startPos to endPos is colliding"""
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        p12 = p2-p1
        k = 40
        #print("testing")
        for i in range(k):
            testPoint = p1 + (i+1)/k*p12
            if self.pointInCollision(testPoint)==True:
                return True
        
        return False

    @IPPerfMonitor
    def curveInCollision(self, startPos, controlPos, endPos, steps=50):
        """ 
        Check whether a Quadratic Bezier curve is in collision.
        Defined by: Start (startPos), Control Point (controlPos), End (endPos)
        """
        assert (len(startPos) == self.getDim())
        assert (len(controlPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        S = np.array(startPos)
        P2n = np.array(controlPos)
        E = np.array(endPos)

        # Generate t values from 0 to 1
        t_values = np.linspace(0, 1, steps)

        for t in t_values:
            # Quadratic Bezier Formula
            # B(t) = (1-t)^2 * S + 2*(1-t)*t * P2n + t^2 * E
            p_curve = (1-t)**2 * S + 2*(1-t)*t * P2n + t**2 * E
            
            if self.pointInCollision(p_curve):
                return True
        
        return False
    
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
        for key, value in self.scene .items():
            plotting.plot_polygon(value, add_points=False, color='red', ax=ax)

        return ax

    def draw_path(self, path_positions: list, ax: plt.Axes = None) -> plt.Axes:
        if not ax:
            ax = self.create_axes()
        
        # Draw a line connecting the path nodes
        x_val, y_val = zip(*path_positions)
        ax.plot(x_val, y_val, color="k")
        ax.scatter(x_val[1:-1], y_val[1:-1], color="k", marker="o")

        # Draw Start and Goal in different colors
        # Start
        ax.scatter(x_val[0], y_val[0], marker="o", color="lightgreen", s=300)
        ax.text(x_val[0], y_val[0], "S", fontweight="heavy", size=16, ha="center", va="center")

        # Goal
        ax.scatter(x_val[-1], y_val[-1], marker="o", color="lightblue", s=300)
        ax.text(x_val[-1], y_val[-1], "G", fontweight="heavy", size=16, ha="center", va="center")

        return ax

    def draw_optimized_path(self, optimized_results, planner, ax=None):
        """
        Visualizes the path with Flyby (Inverse Rounding) curves.
        Handles both Dynamic K (Symmetric) and Fixed K (Asymmetric) modes.
        """
        if not ax:
            ax = self.create_axes()
        
        # 1. Extract Node Names and R-values
        node_names = [item[0] for item in optimized_results]
        r_values = [item[1] for item in optimized_results]
        
        # 2. Retrieve Positions
        # Assuming retrieve_path_positions is available in scope or imported
        raw_positions = retrieve_path_positions(planner.graph, node_names)
        positions = [np.array(p) for p in raw_positions]

        # --- Draw Original Path (Background) ---
        orig_x = [p[0] for p in positions]
        orig_y = [p[1] for p in positions]
        ax.plot(orig_x, orig_y, color="lightgray", linewidth=1.5, linestyle="--", label="Original Path")

        # 3. Draw Start Node
        last_endpoint = positions[0] 
        ax.scatter(last_endpoint[0], last_endpoint[1], marker="o", color="lightgreen", s=300, zorder=5)
        ax.text(last_endpoint[0], last_endpoint[1], "S", fontweight="heavy", ha="center", va="center")

        # 4. Iterate through intermediate nodes
        for i in range(1, len(positions) - 1):
            node_name = node_names[i]
            p_prev = positions[i-1]
            p_curr = positions[i]
            p_next = positions[i+1]
            r = r_values[i]
            
            # Draw the original waypoint lightly
            ax.scatter(p_curr[0], p_curr[1], marker="x", color="gray", alpha=0.5)
            ax.text(p_curr[0], p_curr[1] + 0.5, f"r={round(r, 2)}", color="blue", fontsize=8, ha="center")

            # --- CASE A: No Smoothing (r=0) ---
            if r == 0:
                ax.plot([last_endpoint[0], p_curr[0]], [last_endpoint[1], p_curr[1]], 'k-')
                last_endpoint = p_curr 
                continue

            # --- CASE B: Flyby Smoothing ---
            
            # Check for Fixed K in the graph attributes
            fixed_k = planner.graph.nodes[node_name].get('fixed_k')

            if fixed_k is not None:
                # --- FIXED K LOGIC ---
                # r is always on the incoming (prev) segment
                k = fixed_k
                S = p_curr + r * (p_prev - p_curr)
                E = p_curr + (r * k) * (p_next - p_curr)
            else:
                # --- DYNAMIC K LOGIC (Metric Symmetry) ---
                dist_prev = np.linalg.norm(p_curr - p_prev)
                dist_next = np.linalg.norm(p_next - p_curr)
                prev_is_shorter = dist_prev <= dist_next
                
                if prev_is_shorter:
                    k = dist_prev / dist_next if dist_next > 0 else 1.0
                    S = p_curr + r * (p_prev - p_curr)
                    E = p_curr + (r * k) * (p_next - p_curr)
                else:
                    k = dist_next / dist_prev if dist_prev > 0 else 1.0
                    S = p_curr + (r * k) * (p_prev - p_curr)
                    E = p_curr + r * (p_next - p_curr)

            # Draw Straight Line Connection
            ax.plot([last_endpoint[0], S[0]], [last_endpoint[1], S[1]], 'k-')
            
            # Calculate Control Point (Inverse Rounding)
            P2n = 2 * p_curr - 0.5 * S - 0.5 * E
            
            # Draw Control Point & Triangle
            ax.scatter(P2n[0], P2n[1], marker="*", color="red", s=100, zorder=4)
            ax.plot([S[0], P2n[0], E[0]], [S[1], P2n[1], E[1]], color="red", linestyle=":", linewidth=1, alpha=0.6)

            # Draw The Curve
            t_steps = np.linspace(0, 1, 20)
            curve_x = []
            curve_y = []
            for t in t_steps:
                p = (1-t)**2 * S + 2*(1-t)*t * P2n + t**2 * E
                curve_x.append(p[0])
                curve_y.append(p[1])
            
            ax.plot(curve_x, curve_y, 'b-', linewidth=2) 
            
            # Update last_endpoint
            last_endpoint = E

        # 5. Draw Final Segment to Goal
        goal = positions[-1]
        ax.plot([last_endpoint[0], goal[0]], [last_endpoint[1], goal[1]], 'k-')
        
        ax.scatter(goal[0], goal[1], marker="o", color="lightblue", s=300, zorder=5)
        ax.text(goal[0], goal[1], "G", fontweight="heavy", ha="center", va="center")

        return ax
    
    def draw_path_with_parabolas(self, path_positions: list, r: list, ax: plt.Axes = None) -> plt.Axes:
        """Draws a path with parabolas connecting the nodes according to r
        
        Args:
            path_positions (list): List of path positions
            r (list): List of r values at the path positions
            ax (plt.Axes, optional): Axes to draw on. Defaults to None.
        """
        if not ax:
            ax = self.create_axes()

        line_ending = path_positions[0]

        for i in range(1, len(path_positions) - 1):
            ax.scatter(path_positions[i][0], path_positions[i][1], color="k", marker="x")
            ax.text(path_positions[i][0], path_positions[i][1], round(r[i], 2), size=12, ha="center", va="bottom")
            if r[i] == 0:
                # Draw a line from position i-1 to position i
                ax.plot([path_positions[i-1][0], path_positions[i][0]], [path_positions[i-1][1], path_positions[i][1]], color="k")
                line_ending = path_positions[i]

            else:
                # Calculate S and E
                P2 = np.array(path_positions[i])
                P1 = np.array(path_positions[i - 1])
                P3 = np.array(path_positions[i + 1])

                # Calculate the vectors originating from P2
                vec_P2_P1 = P1 - P2
                vec_P2_P3 = P3 - P2

               
                S = P2 + r[i] * vec_P2_P1
                E = P2 + r[i] * vec_P2_P3

                # Draw a line from the previous segment ending to S
                ax.plot([line_ending[0], S[0]], [line_ending[1], S[1]], color="k")
                line_ending = S


                t = np.linspace(0, 1, 100)
                t_col = t[:, np.newaxis]

                curve = r[i] * vec_P2_P1 * (t_col - 1.0)**2 + r[i] * vec_P2_P3 * t_col**2 + P2

                ax.plot(curve[:, 0], curve[:, 1], color="b")
                line_ending = E

        # Draw a line from line_ending to the last position
        ax.plot([line_ending[0], path_positions[-1][0]], [line_ending[1], path_positions[-1][1]], color="k")

        return ax

               
def retrieve_path_positions(graph: nx.Graph, node_names: list) -> list:
    return [graph.nodes[name]["pos"] for name in node_names]

def calculate_path_length(planner, path: list, use_curves: bool = True) -> float:
    """
    Calculates path length. Supports both Dynamic and Fixed K modes.
    """
    def get_pos(name):
        return np.array(planner.graph.nodes[name]['pos'])

    if not use_curves:
        length = 0.0
        for i in range(len(path) - 1):
            p1 = get_pos(path[i])
            p2 = get_pos(path[i+1])
            length += np.linalg.norm(p2 - p1)
        return length

    total_length = 0.0
    last_endpoint = get_pos(path[0])
    
    for i in range(1, len(path) - 1):
        node_name = path[i]
        p_prev = get_pos(path[i-1])
        p_curr = get_pos(node_name)
        p_next = get_pos(path[i+1])
        
        r = planner.graph.nodes[node_name].get('r', 0.0)
        fixed_k = planner.graph.nodes[node_name].get('fixed_k')

        if r > 0:
            # --- GEOMETRY CALCULATION ---
            if fixed_k is not None:
                # Fixed K: r is always on prev
                k = fixed_k
                S = p_curr + r * (p_prev - p_curr)
                E = p_curr + (r * k) * (p_next - p_curr)
            else:
                # Dynamic K: r is on shorter side
                dist_prev = np.linalg.norm(p_curr - p_prev)
                dist_next = np.linalg.norm(p_next - p_curr)
                prev_is_shorter = dist_prev <= dist_next

                if prev_is_shorter:
                    k = dist_prev / dist_next if dist_next > 0 else 1.0
                    S = p_curr + r * (p_prev - p_curr)
                    E = p_curr + (r * k) * (p_next - p_curr)
                else:
                    k = dist_next / dist_prev if dist_prev > 0 else 1.0
                    S = p_curr + (r * k) * (p_prev - p_curr)
                    E = p_curr + r * (p_next - p_curr)
            
            # 1. Add Straight Segment
            total_length += np.linalg.norm(S - last_endpoint)
            
            # 2. Add Curve Length (Discrete Sum)
            P2n = 2 * p_curr - 0.5 * S - 0.5 * E
            steps = 20
            t_vals = np.linspace(0, 1, steps)
            curve_points = [(1-t)**2 * S + 2*(1-t)*t * P2n + t**2 * E for t in t_vals]
            
            for j in range(len(curve_points)-1):
                total_length += np.linalg.norm(curve_points[j+1] - curve_points[j])
            
            last_endpoint = E
        else:
            # No smoothing
            dist_to_node = np.linalg.norm(p_curr - last_endpoint)
            total_length += dist_to_node
            last_endpoint = p_curr

    # Final Segment
    goal = get_pos(path[-1])
    total_length += np.linalg.norm(goal - last_endpoint)
    
    return total_length

def plot_performance_data(stats, len_original, len_optimized):
    """
    Plots performance metrics (Counts, Time, Path Length) on a multi-axis chart.
    
    Args:
        stats (dict): Dictionary containing count and time for collision checks.
                      e.g. {'pointInCollision': {'count': 100, 'time': 0.5}, ...}
        len_original (float): Length of the original straight path.
        len_optimized (float): Length of the optimized flyby path.
    """
    
    # Create Figure
    fig, ax1 = plt.subplots(figsize=(10, 6))
    plt.title(f"Performance Metrics")

    # X-Axis Setup
    categories = ['Point Checks', 'Curve Checks', 'Path Length']
    x_pos = np.arange(len(categories))
    width = 0.25  # Bar width

    # --- Axis 1: Counts (Left Y-Axis) ---
    color1 = 'tab:blue'
    ax1.set_ylabel('Number of Calls', color=color1, fontweight='bold')
    
    # Extract counts (default to 0 if key missing)
    pt_count = stats.get('pointInCollision', {}).get('count', 0)
    crv_count = stats.get('curveInCollision', {}).get('count', 0)
    
    counts = [pt_count, crv_count, 0]
    bars1 = ax1.bar(x_pos - width, counts, width, label='Count', color=color1, alpha=0.7)
    ax1.tick_params(axis='y', labelcolor=color1)
    
    # Label Counts
    for rect in bars1:
        height = rect.get_height()
        if height > 0:
            ax1.text(rect.get_x() + rect.get_width()/2., height,
                    f'{int(height)}', ha='center', va='bottom', color=color1)

    # --- Axis 2: Time (Right Y-Axis 1) ---
    ax2 = ax1.twinx() 
    color2 = 'tab:orange'
    ax2.set_ylabel('Time (seconds)', color=color2, fontweight='bold')
    
    pt_time = stats.get('pointInCollision', {}).get('time', 0.0)
    crv_time = stats.get('curveInCollision', {}).get('time', 0.0)
    
    times = [pt_time, crv_time, 0]
    bars2 = ax2.bar(x_pos, times, width, label='Time', color=color2, alpha=0.7)
    ax2.tick_params(axis='y', labelcolor=color2)
    
    # Label Times
    for rect in bars2:
        height = rect.get_height()
        if height > 0:
            ax2.text(rect.get_x() + rect.get_width()/2., height,
                    f'{height:.4f}s', ha='center', va='bottom', color=color2, fontsize=9)

    # --- Axis 3: Length (Right Y-Axis 2 - Offset) ---
    ax3 = ax1.twinx()
    ax3.spines["right"].set_position(("axes", 1.15)) # Offset spine outward
    color3 = 'tab:purple'
    ax3.set_ylabel('Path Length (m)', color=color3, fontweight='bold')
    
    # Visual trick: Draw two bars at the 3rd x-position for comparison
    bars3_orig = ax3.bar(x_pos[2] - width/2, len_original, width/2, color='gray', label='Original Len')
    bars3_opt  = ax3.bar(x_pos[2] + width/2, len_optimized, width/2, color=color3, label='Optimized Len')
    
    ax3.tick_params(axis='y', labelcolor=color3)
    
    # Label Lengths
    ax3.text(x_pos[2] - width/2, len_original, f'{len_original:.1f}m', ha='center', va='bottom', color='black', fontsize=9)
    ax3.text(x_pos[2] + width/2, len_optimized, f'{len_optimized:.1f}m', ha='center', va='bottom', color=color3, fontsize=9, fontweight='bold')

    # Formatting
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(categories)
    ax1.grid(True, axis='y', alpha=0.3)
    
    # Calculate and display percentage change text
    if len_original > 0:
        diff_percent = ((len_optimized - len_original) / len_original) * 100
        ax3.text(x_pos[2], max(len_original, len_optimized) * 1.05, 
                 f"Change: {diff_percent:+.2f}%", ha='center', color='black', fontweight='bold')

    # Legend
    fig.legend([bars1, bars2, bars3_orig, bars3_opt], 
               ["Count", "Time (s)", "Orig. Length", "Opt. Length"], 
               loc="upper right", bbox_to_anchor=(0.9, 0.88))

    plt.tight_layout()
    plt.show()

def clear_graph_attributes(planner):
    """
    Removes 'r' and 'fixed_k' attributes from all nodes in the planner's graph.
    This ensures subsequent optimization runs start with a clean state.
    """
    graph = planner.graph
    
    # Iterate over all nodes in the graph
    for node_name in graph.nodes:
        # Access the mutable attribute dictionary for the node
        node_attrs = graph.nodes[node_name]
        
        # Safely remove attributes if they exist using pop(key, default)
        node_attrs.pop('r', None)
        node_attrs.pop('fixed_k', None)