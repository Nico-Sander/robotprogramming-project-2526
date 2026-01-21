from shapely import Point, plotting
import numpy as np
import matplotlib.pyplot as plt

from planners.IPPerfMonitor import IPPerfMonitor
from benchmarks import X_LIMITS, Y_LIMITS
from evaluation import retrieve_path_positions


class CollisionChecker(object):
    """
    Handles all collision detection against the environment (Shapely Polygons) and provides visualization utilities.
    
    This class acts as the central interface for validity checking within the planning framework. It wraps the 
    geometric definitions of the environment (obstacles) and exposes methods to test whether specific geometric 
    primitives (points, lines, curves) intersect with these obstacles.
    
    In addition to safety checks, this class manages the plotting of the environment and the resulting paths 
    using `matplotlib`. This coupling ensures that the visualization logic always matches the collision 
    logic (e.g., using the same environment limits and obstacle definitions).

    Attributes:
        scene (dict): A dictionary mapping obstacle names to Shapely Polygon objects.
        limits (list): A list containing [X_LIMITS, Y_LIMITS] defining the workspace boundaries.
    """
    def __init__(self, scene, limits=[X_LIMITS, Y_LIMITS]):
        """
        Initializes the collision checker with a specific environment scene.

        Parameters:
            scene (dict): Dictionary of Shapely Polygons representing obstacles.
            limits (list): Workspace boundaries [min, max] for X and Y axes.
        """
        self.scene = scene
        self.limits = limits

    def getDim(self):
        """
        Returns the dimensionality of the configuration space.

        Returns:
            int: The dimension count (fixed at 2 for this planar planner).
        
        Side Effects:
            None.
        """
        return 2

    def getEnvironmentLimits(self):
        """
        Retrieves the physical boundaries of the planning environment.

        Returns:
            list: A copy of the [X_LIMITS, Y_LIMITS] list.

        Side Effects:
            None.
        """
        return list(self.limits)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """
        Checks if a single 2D configuration (point) lies inside any environmental obstacle.

        Iterates through the dictionary of obstacles in the scene and performs a geometric intersection test 
        using the Shapely library. This is the fundamental atomic check upon which line and curve checks are built.

        Parameters:
            pos (tuple/list): A tuple (x, y) representing the position to check.

        Returns:
            bool: True if the point is inside an obstacle (collision), False otherwise (free).

        Side Effects:
            None (Calculations only).
        """
        assert (len(pos) == self.getDim())
        
        # specific geometric implementation using Shapely
        point = Point(pos[0], pos[1])
        
        # Iterate over all obstacles in the scene
        for key, value in self.scene.items():
            # Use Shapely's efficient geometric intersection
            if value.intersects(point):
                return True
        return False

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        """ 
        Checks if a straight line segment connecting two points collides with any obstacles.

        This method approximates the line segment by discretizing it into a series of test points (interpolation) 
        and verifying each point individually. This approach is generally faster than analytic polygon clipping 
        for complex scenes but relies on the sampling resolution ($k$) being sufficient to detect thin obstacles.

        Parameters:
            startPos (tuple/list): The (x, y) coordinates of the line start.
            endPos (tuple/list): The (x, y) coordinates of the line end.

        Returns:
            bool: True if any sampled point on the line collides, False otherwise.

        Side Effects:
            None.
        """
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        p12 = p2-p1 # Direction vector
        
        k = 40 # Number of test points (resolution of the check)
        
        # Iterate through discrete steps along the line segment
        for i in range(k):
            # Linear interpolation formula: P(t) = P1 + t*(P2-P1)
            testPoint = p1 + (i+1)/k*p12
            if self.pointInCollision(testPoint):
                return True
        
        return False

    @IPPerfMonitor
    def curveInCollision(self, startPos, controlPos, endPos, steps=50):
        """ 
        Checks if a Quadratic Bezier curve collides with the environment.

        The curve is mathematically defined by three points: Start ($S$), Control Point ($P_{2n}$), and End ($E$). 
        Similar to the line check, this method discretizes the curve formula $B(t)$ into `steps` intervals 
        and validates each point. This is crucial for verifying the "rounded corners" generated by the 
        Fly-By optimization.

        Parameters:
            startPos (tuple/list): The curve's starting tangent point ($S$).
            controlPos (tuple/list): The geometric control point ($P_{2n}$ or apex).
            endPos (tuple/list): The curve's ending tangent point ($E$).
            steps (int): The number of samples to check along the curve (default: 50).

        Returns:
            bool: True if any part of the curve intersects an obstacle, False otherwise.

        Side Effects:
            None.
        """
        assert (len(startPos) == self.getDim())
        assert (len(controlPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        S = np.array(startPos)
        P2n = np.array(controlPos)
        E = np.array(endPos)

        # Sample parameter t from 0 to 1
        t_values = np.linspace(0, 1, steps)

        for t in t_values:
            # Quadratic Bezier Formula: B(t) = (1-t)^2 * S + 2*(1-t)*t * P2n + t^2 * E
            p_curve = (1-t)**2 * S + 2*(1-t)*t * P2n + t**2 * E
            
            # Check the generated point for collision
            if self.pointInCollision(p_curve):
                return True
        
        return False
    
    # --- VISUALIZATION METHODS ---

    def _setup_axis(self, ax: plt.Axes):
        """ 
        Applies standard styling (limits, grid, ticks) to a matplotlib axis object.
        
        Ensures consistent visual formatting across different plots (main view, subplots, etc.) 
        by enforcing the environment limits and enabling a background grid.
        
        Parameters:
            ax (plt.Axes): The matplotlib axis to style.
            
        Returns:
            None
            
        Side Effects:
            Modifies the state of the passed `ax` object (ticks, limits, grid).
        """
        ax.set_xlim(self.limits[0][0], self.limits[0][1])
        ax.set_ylim(self.limits[1][0], self.limits[1][1])
        ax.set_xticks(range(0, int(self.limits[0][1])+1))
        ax.set_yticks(range(0, int(self.limits[1][1])+1))
        ax.grid(True, which='both', linestyle='--', alpha=0.5, zorder=0)

    def create_axes(self, figsize: tuple = (10,10)) -> plt.Axes:
        """
        Creates a new matplotlib figure and axis with the correct environment setup.
        
        Parameters:
            figsize (tuple): Dimensions of the figure (width, height).
            
        Returns:
            plt.Axes: The configured axis object ready for plotting.
            
        Side Effects:
            Creates a new matplotlib figure.
        """
        _, ax = plt.subplots(figsize=figsize)
        self._setup_axis(ax)
        return ax

    def draw_enviroments(self, ax: plt.Axes = None) -> plt.Axes:
        """
        Renders the obstacles (polygons) onto a plot.
        
        Parameters:
            ax (plt.Axes): Optional existing axis. If None, a new one is created.
            
        Returns:
            plt.Axes: The axis containing the plotted environment.
            
        Side Effects:
            Adds polygon patches to the matplotlib axis.
        """
        if not ax:
            ax = self.create_axes()
        else:
            self._setup_axis(ax)

        # Plot each obstacle polygon in red
        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, color='red', ax=ax)
        return ax

    def draw_path(self, path_positions: list, ax: plt.Axes = None) -> plt.Axes:
        """ 
        Visualizes a basic piecewise-linear path (raw waypoints).
        
        Parameters:
            path_positions (list): A list of (x,y) tuples representing the path nodes.
            ax (plt.Axes): Optional existing axis.
            
        Returns:
            plt.Axes: The axis with the path drawn.
            
        Side Effects:
            Plots lines, scatter points, and text labels (S/G) onto the axis.
        """
        if not ax:
            ax = self.create_axes()
        else:
            self._setup_axis(ax)
        
        # Unzip coordinates for plotting
        x_val, y_val = zip(*path_positions)
        ax.plot(x_val, y_val, color="k")
        ax.scatter(x_val[1:-1], y_val[1:-1], color="k", marker="o")

        # Draw Start Node (Green)
        ax.scatter(x_val[0], y_val[0], marker="o", color="lightgreen", s=300)
        ax.text(x_val[0], y_val[0], "S", fontweight="heavy", size=16, ha="center", va="center")

        # Draw Goal Node (Blue)
        ax.scatter(x_val[-1], y_val[-1], marker="o", color="lightblue", s=300)
        ax.text(x_val[-1], y_val[-1], "G", fontweight="heavy", size=16, ha="center", va="center")

        return ax

    def draw_optimized_path(self, optimized_results, planner, ax=None):
        """
        Visualizes the detailed G1-smooth path, including Bezier curves, control points, and geometric parameters.

        This method reconstructs the full geometry of the optimized path for verification and debugging. 
        It does not rely on a simple list of points; instead, it reads the geometric parameters (r, fixed_k, P_2n) 
        stored in the graph nodes and recalculates the exact tangent points (S, E) and curve trajectory.
        This ensures the visualization matches the collision checking logic exactly.

        Visual elements included:
        - **Grey Dashed Line**: The original unoptimized path.
        - **Black Dashed Line**: The "New Straight Path" (control polygon) connecting the optimized control points.
        - **Blue Curves**: The Quadratic Bezier segments.
        - **Red Stars**: The virtual control points (P_2n) governing the curves.
        - **Text Labels**: Annotations showing the specific radius (r) and asymmetry (k) values at each corner.

        Parameters:
            optimized_results (list): Output from the optimizer, a list of (node_name, radius) tuples.
            planner (object): The planner instance containing the graph with node attributes.
            ax (plt.Axes): Optional axis to plot on.

        Returns:
            plt.Axes: The updated plot axis.

        Side Effects:
            Performs significant geometric calculations to reconstruct the path for rendering. 
            Adds multiple complex plot elements (lines, scatter, text) to the axis.
        """
        if not ax:
            ax = self.create_axes()
        else:
            # Apply standard styling to the passed subplot
            self._setup_axis(ax)

        # 1. Setup Data
        # Extract node names and original raw positions for reference
        node_names = [x[0] for x in optimized_results]
        raw_positions = retrieve_path_positions(planner.graph, node_names)
        positions = [np.array(p) for p in raw_positions]

        # 2. Draw Original Path (Grey dashed)
        # Visual reference to see how much the path deviates from the original
        orig_x = [p[0] for p in positions]
        orig_y = [p[1] for p in positions]
        ax.plot(orig_x, orig_y, "lightgray", linestyle="--", label="Original", zorder=1)
        
        # 3. Retrieve Control Points (P2n) from Graph
        # These are the actual "turning points" of the new path logic
        p2n_list = []
        for name in node_names:
            p2n_list.append(planner.graph.nodes[name].get('P2n', planner.graph.nodes[name]['pos']))
        
        # Draw the "New Straight Path" (Control Polygon)
        # This shows the effective corridor the curves are inscribed within
        p2n_x = [p[0] for p in p2n_list]
        p2n_y = [p[1] for p in p2n_list]
        ax.plot(p2n_x, p2n_y, "k--", alpha=0.8, label="New Straight Path", zorder=2)

        # 4. Draw Curves and Segments
        last_endpoint = positions[0] 
        
        # Draw Start Node
        ax.scatter(positions[0][0], positions[0][1], marker="o", color="lightgreen", s=300, zorder=5)
        ax.text(positions[0][0], positions[0][1], "S", fontweight="heavy", ha="center", va="center", zorder=6)

        # Iterate through intermediate nodes to reconstruct corners
        for i in range(1, len(positions)-1):
            P_prev = p2n_list[i-1]
            P_curr = p2n_list[i]
            P_next = p2n_list[i+1]
            
            node_name = node_names[i]
            r = planner.graph.nodes[node_name].get('r', 0)
            fixed_k = planner.graph.nodes[node_name].get('fixed_k')

            # --- Recalculate S and E ---
            # Re-implement the tangent logic to determine exactly where the curve starts and ends
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
            
            # Control Point (Star) - The apex of the turn
            ax.scatter(P_curr[0], P_curr[1], marker="*", color="red", s=100, zorder=4)
            
            # Straight Connection (From previous E to current S)
            ax.plot([last_endpoint[0], S[0]], [last_endpoint[1], S[1]], 'k-', zorder=3)
            
            if r > 0:
                # Draw Bezier Curve (S -> P2n -> E)
                t_steps = np.linspace(0, 1, 20)
                curve_pts = [(1-t)**2 * S + 2*(1-t)*t * P_curr + t**2 * E for t in t_steps]
                
                cx = [p[0] for p in curve_pts]
                cy = [p[1] for p in curve_pts]
                ax.plot(cx, cy, color="#0084FF", linestyle='-', linewidth=2, zorder=3)
                last_endpoint = E
            else:
                # If radius is 0, the path goes directly to the corner
                last_endpoint = P_curr

            # --- LABELING (OUTSIDE CORNER) ---
            # Calculate the bisector vector to offset the label away from the inside of the turn
            v1 = P_prev - P_curr
            v2 = P_next - P_curr
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 > 0: v1 /= n1
            if n2 > 0: v2 /= n2
            
            # Direction pointing OUTWARD (opposite to the sum of vectors)
            bisector = -(v1 + v2) 
            bnorm = np.linalg.norm(bisector)
            
            # Distance to push the label away from the point
            offset_dist = 1.2
            
            if bnorm < 0.01:
                offset = np.array([0, offset_dist])
            else:
                offset = (bisector / bnorm) * offset_dist
                
            text_pos = P_curr + offset

            # Format & Draw Label Box
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