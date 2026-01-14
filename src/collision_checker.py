from shapely import Point, plotting
import numpy as np
import matplotlib.pyplot as plt

from planners.IPPerfMonitor import IPPerfMonitor
from benchmarks import X_LIMITS, Y_LIMITS
from evaluation import retrieve_path_positions


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

    def _setup_axis(self, ax: plt.Axes):
        """ 
        Applies standard styling (limits, grid, ticks) to any given axis.
        This ensures subplots look the same as standalone figures.
        """
        ax.set_xlim(self.limits[0][0], self.limits[0][1])
        ax.set_ylim(self.limits[1][0], self.limits[1][1])
        ax.set_xticks(range(0, int(self.limits[0][1])+1))
        ax.set_yticks(range(0, int(self.limits[1][1])+1))
        ax.grid(True, which='both', linestyle='--', alpha=0.5, zorder=0)

    def create_axes(self, figsize: tuple = (10,10)) -> plt.Axes:
        _, ax = plt.subplots(figsize=figsize)
        self._setup_axis(ax)
        return ax

    def draw_enviroments(self, ax: plt.Axes = None) -> plt.Axes:
        if not ax:
            ax = self.create_axes()
        else:
            self._setup_axis(ax)

        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, color='red', ax=ax)
        return ax

    def draw_path(self, path_positions: list, ax: plt.Axes = None) -> plt.Axes:
        """ Draws a simple straight-line path. """
        if not ax:
            ax = self.create_axes()
        else:
            self._setup_axis(ax)
        
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
        """
        if not ax:
            ax = self.create_axes()
        else:
            # Apply standard styling to the passed subplot
            self._setup_axis(ax)

        # 1. Setup Data
        node_names = [x[0] for x in optimized_results]
        raw_positions = retrieve_path_positions(planner.graph, node_names)
        positions = [np.array(p) for p in raw_positions]

        # 2. Draw Original Path (Grey dashed)
        orig_x = [p[0] for p in positions]
        orig_y = [p[1] for p in positions]
        ax.plot(orig_x, orig_y, "lightgray", linestyle="--", label="Original", zorder=1)
        
        # 3. Retrieve Control Points (P2n) from Graph
        p2n_list = []
        for name in node_names:
            p2n_list.append(planner.graph.nodes[name].get('P2n', planner.graph.nodes[name]['pos']))
        
        # Draw the "New Straight Path" (Control Polygon)
        p2n_x = [p[0] for p in p2n_list]
        p2n_y = [p[1] for p in p2n_list]
        ax.plot(p2n_x, p2n_y, "k--", alpha=0.8, label="New Straight Path", zorder=2)

        # 4. Draw Curves and Segments
        last_endpoint = positions[0] 
        
        # Draw Start Node
        ax.scatter(positions[0][0], positions[0][1], marker="o", color="lightgreen", s=300, zorder=5)
        ax.text(positions[0][0], positions[0][1], "S", fontweight="heavy", ha="center", va="center", zorder=6)

        # Iterate through intermediate nodes
        for i in range(1, len(positions)-1):
            P_prev = p2n_list[i-1]
            P_curr = p2n_list[i]
            P_next = p2n_list[i+1]
            
            node_name = node_names[i]
            r = planner.graph.nodes[node_name].get('r', 0)
            fixed_k = planner.graph.nodes[node_name].get('fixed_k')

            # --- Recalculate S and E ---
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

            # --- DRAWING ---
            
            # Control Point (Star)
            ax.scatter(P_curr[0], P_curr[1], marker="*", color="red", s=100, zorder=4)
            
            # Straight Connection (From previous E to current S)
            ax.plot([last_endpoint[0], S[0]], [last_endpoint[1], S[1]], 'k-', zorder=3)
            
            if r > 0:
                # Draw Bezier Curve (S -> P2n -> E)
                t_steps = np.linspace(0, 1, 20)
                curve_pts = [(1-t)**2 * S + 2*(1-t)*t * P_curr + t**2 * E for t in t_steps]
                
                cx = [p[0] for p in curve_pts]
                cy = [p[1] for p in curve_pts]
                # ax.plot(cx, cy, 'b-', linewidth=2, zorder=3)
                ax.plot(cx, cy, color="#0084FF", linestyle='-', linewidth=2, zorder=3)
                last_endpoint = E
            else:
                last_endpoint = P_curr

            # --- LABELING (OUTSIDE CORNER) ---
            v1 = P_prev - P_curr
            v2 = P_next - P_curr
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 > 0: v1 /= n1
            if n2 > 0: v2 /= n2
            
            # Direction pointing OUTWARD
            bisector = -(v1 + v2) 
            bnorm = np.linalg.norm(bisector)
            
            # --- CHANGE: Increased distance from 0.7 to 1.5 ---
            offset_dist = 1.2
            
            if bnorm < 0.01:
                offset = np.array([0, offset_dist])
            else:
                offset = (bisector / bnorm) * offset_dist
                
            text_pos = P_curr + offset

            # Format & Draw
            k_label = f"k={fixed_k:.2f}" if fixed_k is not None else "k=Sym"
            label_text = f"r={r:.2f}\n{k_label}"
            
            ax.text(text_pos[0], text_pos[1], label_text, 
                    color="darkblue", fontsize=8, 
                    ha="center", va="center",
                    zorder=10, 
                    bbox=dict(facecolor='white', alpha=0.85, edgecolor='lightgray', boxstyle='round,pad=0.2'))

        # Final Segment to Goal
        ax.plot([last_endpoint[0], positions[-1][0]], [last_endpoint[1], positions[-1][1]], 'k-', zorder=3)
        
        # Draw Goal Node
        ax.scatter(positions[-1][0], positions[-1][1], marker="o", color="lightblue", s=300, zorder=5)
        ax.text(positions[-1][0], positions[-1][1], "G", fontweight="heavy", ha="center", va="center", zorder=6)

        return ax