import numpy as np
import matplotlib.pyplot as plt

from evaluation import calculate_path_length, clear_graph_attributes

class OptimizeFlyby():
    def __init__(self):
        pass

    def optimizePath(self, path: list = [], planner = None, config: dict = {}):
        """
        Executes the "Fly-by" optimization routine to smooth a jagged path using Quadratic Bezier curves.

        This method implements an iterative approach to convert a list of waypoints into a G1-continuous 
        path consisting of straight line segments and Bezier curves (rounding) at the corners. The algorithm 
        alternates between calculating optimal control points ($P_{2n}$) and verifying collision constraints.

        The optimization process consists of two main phases within a loop:
        1. Relaxation Phase (Inverse Rounding): 
           To prevent the smoothed path from "cutting the corner" too deeply and colliding with obstacles, 
           the algorithm calculates a new virtual control point, $P_{2n}$, for every corner. This point is 
           moved outward such that the resulting curve's apex touches the original node position ($P_{org}$).
           
           **Why is this a loop?** The formula to calculate a node's $P_{2n}$ depends on the positions of its immediate neighbors 
           ($P_{prev}$ and $P_{next}$). Because those neighbors are also shifting their positions simultaneously 
           to correct their own curves, a change in one node affects its neighbors, which in turn affects 
           the original node again. The algorithm must therefore iterate through the entire path multiple 
           times to propagate these changes until the whole chain of control points stabilizes (converges).

        2. Collision Check Phase: 
           Generates the actual geometry (tangent points S and E, and the curve itself) using the calculated 
           $P_{2n}$. It validates these segments against the environment. If a collision is detected 
           (in the curve or connecting lines), the smoothing radius ($r$) for the affected node is reduced 
           to pull the geometry tighter to the corner, and the loop repeats.

        Parameters:
            path (list): A list of strings representing the node names in the solution path (e.g., ['start', '1', 'goal']).
            planner (object): The planner instance containing the graph data structure (planner.graph) and 
                              the collision checker (planner._collisionChecker).
            config (dict): Configuration dictionary for optimization parameters:
                           - 'r_init' (float): Initial radius for smoothing (default: 0.49).
                           - 'r_step' (float): Decrement step size for radius reduction upon collision (default: 0.01).
                           - 'k' (float/None): Global asymmetry factor. If None, dynamic symmetric rounding is used.

        Returns:
            list: A list of tuples, where each tuple contains (node_name, final_r_value).

        Side Effects:
            Modifies the `planner.graph` nodes in-place. Specifically, it updates or adds:
            - 'r': The final optimized radius for the node.
            - 'P2n': The coordinates of the calculated virtual control point.
            - 'fixed_k': The asymmetry factor used (if a global k was provided).
        """

        # --- Initialization ---
        # Retrieve configuration values with defaults
        r_init = min(config.get('r_init', 0.49), 0.49)
        r_step = config.get('r_step', 0.01)
        r_min = 0.02
        # Global 'k' serves as a fallback if no specific 'k' is defined on the node
        global_k = config.get('k', None) 

        cc = planner._collisionChecker
        
        # Initialize the radius map: tracks the current 'r' for each node
        r_map = {node: r_init for node in path}
        
        # Initialize the control point map: starts with the original node positions
        # These P2n values will drift during the relaxation phase
        p2n_map = {name: np.array(planner.graph.nodes[name]['pos']) for name in path}

        # --- Helper: Geometry Calculation ---
        def get_tangent_points(P_prev, P_curr, P_next, r_val, node_name):
            """
            Calculates the start (S) and end (E) tangent points for the Bezier curve at a specific corner.
            """
            d_in = np.linalg.norm(P_curr - P_prev)
            d_out = np.linalg.norm(P_next - P_curr)
            
            # Determine the asymmetry factor (k)
            # 1. Check if this specific node has a fixed 'k' attribute set in the graph
            node_k = planner.graph.nodes[node_name].get('fixed_k')
            
            # 2. Priority Logic: Node-specific 'k' > Global Config 'k' > None (Dynamic Metric Symmetry)
            target_k = node_k if node_k is not None else global_k

            # Calculate lambda (lam_in/lam_out), which represents the distance along the tangent vectors
            if target_k is not None:
                # Use fixed asymmetry factor if provided
                lam_in, lam_out = r_val, r_val * target_k
            else:
                # Use Dynamic Metric Symmetry: Adjusts smoothing to respect the lengths of incoming/outgoing edges.
                # If one edge is significantly shorter, the curve is constrained to fit within it.
                if d_in <= d_out:
                    dist = r_val * d_in
                    lam_in = r_val
                    lam_out = dist / d_out if d_out > 0 else 0
                else:
                    dist = r_val * d_out
                    lam_out = r_val
                    lam_in = dist / d_in if d_in > 0 else 0
            
            # Compute vector directions
            vec_in = P_prev - P_curr
            vec_out = P_next - P_curr
            
            # Calculate absolute positions of S (Start of curve) and E (End of curve)
            S = P_curr + lam_in * vec_in
            E = P_curr + lam_out * vec_out
            return S, E

        # --- Main Optimization Loop ---
        max_iterations = 50
        for _ in range(max_iterations):
            path_valid = True
            
            # 1. RELAXATION PHASE
            # Iteratively adjust P2n (virtual control points) based on neighbors.
            # This 'Inverse Rounding' ensures the resulting curve passes closer to the original path nodes.
            # 15 iterations are generally sufficient for the positions to converge.
            for _relax in range(15): 
                for i in range(1, len(path) - 1):
                    node = path[i]
                    prev_node = path[i-1]
                    next_node = path[i+1]
                    
                    # Retrieve coordinates
                    P_org = np.array(planner.graph.nodes[node]['pos']) # Original fixed position
                    P_prev_p2n = p2n_map[prev_node]                    # Current neighbor control point
                    P_curr_p2n = p2n_map[node]                         # Current node control point
                    P_next_p2n = p2n_map[next_node]                    # Current neighbor control point
                    r = r_map[node]
                    
                    d_in = np.linalg.norm(P_curr_p2n - P_prev_p2n)
                    d_out = np.linalg.norm(P_next_p2n - P_curr_p2n)
                    
                    # Recalculate lambda weights (li/lo) for the relaxation formula
                    node_k = planner.graph.nodes[node].get('fixed_k')
                    target_k = node_k if node_k is not None else global_k

                    if target_k is not None:
                        li, lo = r, r * target_k
                    else:
                        if d_in == 0 or d_out == 0: 
                            li, lo = r, r
                        elif d_in <= d_out:
                            li = r
                            lo = (r * d_in) / d_out
                        else:
                            lo = r
                            li = (r * d_out) / d_in

                    # Apply Inverse Rounding Formula: P2n = (4*P_org - li*P_prev - lo*P_next) / (4 - li - lo)
                    # This shifts P2n such that the apex of the Bezier curve aligns with P_org.
                    numerator = 4 * P_org - li * P_prev_p2n - lo * P_next_p2n
                    denominator = 4 - li - lo
                    if abs(denominator) < 0.001: denominator = 0.001
                    p2n_map[node] = numerator / denominator

            # 2. COLLISION CHECK PHASE
            # Reconstruct the geometry for all nodes using the new P2n positions
            p2n_list = [p2n_map[name] for name in path]
            current_geom = [] 
            for i in range(1, len(path) - 1):
                S, E = get_tangent_points(p2n_list[i-1], p2n_list[i], p2n_list[i+1], r_map[path[i]], path[i])
                current_geom.append({'S': S, 'E': E})

            # Verify all path components for collisions:
            # - Straight segments between curves
            # - The curves themselves
            # - Dot products (ensure geometry is not folding back on itself)
            
            # A. Check Start Segment (From Start Node -> First Curve Start S)
            start_node = p2n_list[0]
            first_S = current_geom[0]['S']
            # If the vector reverses direction or the line collides, shrink the first corner's radius
            if np.dot(p2n_list[1]-start_node, first_S-start_node) < 0 or cc.lineInCollision(start_node, first_S):
                 path_valid = False
                 r_map[path[1]] = max(r_min, r_map[path[1]] - r_step)

            # B. Check Connectors (Lines connecting Curve A End -> Curve B Start)
            for i in range(len(current_geom) - 1):
                end_pt_prev = current_geom[i]['E']
                start_pt_next = current_geom[i+1]['S']
                
                # Vector between control points vs Vector between tangent points
                vec_p2n = p2n_list[i+2] - p2n_list[i+1]
                vec_connect = start_pt_next - end_pt_prev
                
                # Check for geometry validity (no reversal) and collision
                if np.dot(vec_p2n, vec_connect) < 0 or cc.lineInCollision(end_pt_prev, start_pt_next):
                    path_valid = False
                    # Reduce radius of both adjacent corners to create more space for the connector
                    r_map[path[i+1]] = max(r_min, r_map[path[i+1]] - r_step)
                    r_map[path[i+2]] = max(r_min, r_map[path[i+2]] - r_step)

            # C. Check End Segment (From Last Curve End E -> Goal Node)
            last_E = current_geom[-1]['E']
            end_node = p2n_list[-1]
            if np.dot(end_node-p2n_list[-2], end_node-last_E) < 0 or cc.lineInCollision(last_E, end_node):
                 path_valid = False
                 r_map[path[-2]] = max(r_min, r_map[path[-2]] - r_step)

            # D. Check Curves (Discretized Bezier check)
            for i in range(len(current_geom)):
                geom = current_geom[i]
                node_name = path[i+1]
                # P2n (p2n_list[i+1]) is the control point for the Bezier curve
                if cc.curveInCollision(geom['S'], p2n_list[i+1], geom['E'], steps=20):
                    path_valid = False
                    r_map[node_name] = max(r_min, r_map[node_name] - r_step)
            
            # If no collisions occurred and geometry is valid, the optimization is successful
            if path_valid:
                break
        
        # --- Final Save ---
        # Persist the calculated attributes back to the planner's graph for visualization and analysis
        results = []
        for i, node_name in enumerate(path):
            val = r_map[node_name]
            
            # Clamp values: Start/Goal must be 0, others must be >= r_min or 0
            if val <= r_min: val = 0.0
            if i == 0 or i == len(path)-1: val = 0.0
            
            planner.graph.nodes[node_name]['r'] = val
            
            # Store the calculated P2n (Control Point)
            # For start/goal/straight nodes, P2n is simply the node position
            if val > 0 and i > 0 and i < len(path)-1:
                planner.graph.nodes[node_name]['P2n'] = p2n_map[node_name]
            else:
                planner.graph.nodes[node_name]['P2n'] = np.array(planner.graph.nodes[node_name]['pos'])
            
            # Store global_k if it was used, ensuring subsequent calculations (like path length)
            # use the correct geometry settings.
            if global_k is not None:
                planner.graph.nodes[node_name]['fixed_k'] = global_k

            results.append((node_name, val))
            
        return results

    def optimize_global_k(self, planner, node_names, r_fixed=0.5, plot=True):
        """
        Performs a parameter sweep to identify the optimal global asymmetry factor ($k$) that minimizes the G1-continuous path length.

        This method systematically tests a range of scalar values for the asymmetry factor $k$, which controls the 
        "skew" of the Bezier curves at path corners. For each candidate value, it executes the full fly-by 
        optimization routine and measures the resulting total path length. It identifies the global minimum from 
        these samples, effectively performing a grid search over the hyperparameter space.

        If plotting is enabled, it generates a visualization of the optimization landscape (Length vs. $k$), 
        which typically exhibits a convex shape where the optimal $k$ balances cutting corners tightly versus 
        maintaining efficient straight-line connections. Finally, the method reapplies the optimal configuration 
        to the planner graph to ensure the object's state reflects the best found solution.

        Parameters:
            planner (object): The planner instance containing the graph data structure and collision checker.
            node_names (list): A list of node identifiers (strings) representing the sequence of the path.
            r_fixed (float): The initial smoothing radius to use consistently across the sweep (default: 0.5).
            plot (bool): If True, prints the results to stdout and renders a matplotlib graph of the sweep (default: True).

        Returns:
            dict: A dictionary containing the optimization results:
                  - 'best_k' (float): The asymmetry factor that yielded the shortest path.
                  - 'min_length' (float): The length of the path at the optimal k.
                  - 'k_values' (array): The array of k candidates tested.
                  - 'lengths' (list): The corresponding path lengths for each candidate.
                  - 'optimized_path' (list): The optimization result (radius map) for the best k.

        Side Effects:
            - Modifies `planner.graph` attributes ('r', 'P2n', 'fixed_k') repeatedly during the sweep.
            - Leaves the `planner.graph` in the state corresponding to the optimal `best_k`.
            - Generates a matplotlib figure and prints text to stdout if `plot` is True.
        """
        # 1. Define range of k to test
        # Create a linear space of candidate values for the asymmetry factor from 0.1 to 3.0
        k_values = np.linspace(0.1, 3.0, 30)
        lengths = []
        
        if plot:
            print(f"--- Starting K-Sweep for r_init={r_fixed} ---")
        
        # 2. Sweep Loop
        # Iterate through every candidate k value to evaluate its performance
        for k in k_values:
            # Clear existing graph attributes to ensure a fair comparison for this specific k
            # without artifacts from previous iterations
            clear_graph_attributes(planner)
            
            # Run Optimizer (Sweep)
            # Execute the fly-by optimization with the current candidate k
            config = {'r_init': r_fixed, 'k': k}
            self.optimizePath(node_names, planner, config)
            
            # Measure Length
            # Calculate the resulting G1-continuous path length
            length = calculate_path_length(planner, node_names, use_curves=True)
            lengths.append(length)

        # 3. Find the Minimum
        # Identify the global minimum length and the corresponding k value
        min_length = min(lengths)
        min_idx = lengths.index(min_length)
        best_k = k_values[min_idx]
        
        if plot:
            print(f"--- Result ---")
            print(f"Optimal Global k: {best_k:.2f}")
            print(f"Minimum Length:   {min_length:.4f}m")
            
            # Visualization (Internal)
            # Plot the optimization landscape: Length vs Asymmetry Factor
            plt.figure(figsize=(10, 6))
            plt.plot(k_values, lengths, 'b-o', label='Path Length')
            plt.plot(best_k, min_length, 'r*', markersize=15, label=f'Optimum (k={best_k:.2f})')
            plt.title(f'Optimization of k (for r_init={r_fixed})')
            plt.xlabel('Asymmetry Factor k')
            plt.ylabel('Total Path Length [m]')
            plt.grid(True)
            plt.legend()
            plt.show()
        
        # 4. Final Application (Side Effect)
        # Apply the best k found to the graph so it persists for the caller.
        # This ensures the planner object is left in the optimized state.
        if plot:
            print(f"Applying optimal k={best_k:.2f} to graph...")
            
        clear_graph_attributes(planner)
        config = {'r_init': r_fixed, 'k': best_k}
        final_path = self.optimizePath(node_names, planner, config)
        
        # 5. Return Rich Data
        # Return a dictionary containing both the result and the sweep data for external analysis
        return {
            'best_k': best_k,
            'min_length': min_length,
            'k_values': k_values,
            'lengths': lengths,
            'optimized_path': final_path
        }

    def optimize_individual_k(self, path, planner, config={}):
        """
        Optimizes the asymmetry factor ($k$) for each intermediate node individually using a Coordinate Descent algorithm.

        This method fine-tunes the path geometry by iterating through every corner and testing a discrete set 
        of asymmetry factors ($k$). Unlike global optimization, which applies a single $k$ to the entire path, 
        this approach allows each corner to adopt a unique skew that best fits its local geometry (e.g., adapting 
        differently to sharp turns versus obtuse angles).

        The algorithm employs a greedy Coordinate Descent strategy:
        1. It iterates sequentially through the list of nodes.
        2. For each node, it measures the current total path length as a baseline.
        3. It then tentatively applies every candidate $k$ value (including `None` for dynamic symmetry) to 
           that specific node while keeping others fixed.
        4. The fly-by optimization is executed for each candidate, and the $k$ yielding the shortest total 
           path is "locked in" before moving to the next node.
        
        This process repeats for up to two full passes over the path or until the solution converges (no further 
        improvements are detected).

        Parameters:
            path (list): A list of node identifiers (strings) representing the sequence of the path.
            planner (object): The planner instance containing the graph data structure and collision checker.
            config (dict): Configuration dictionary. Primarily uses 'r_init' (default: 0.5) as the base radius 
                           for the optimization runs.

        Returns:
            list: The result of the final `optimizePath` run, containing a list of tuples (node_name, final_r_value).

        Side Effects:
            - Modifies `planner.graph` nodes in-place, specifically setting or updating the 'fixed_k' attribute 
              for every intermediate node in the path.
            - Overwrites 'r' and 'P2n' attributes during the internal optimization calls.
        """

        # 1. Config
        r_base = config.get('r_init', 0.5)
        
        # We test a range of skews. 'None' represents the default dynamic/symmetric mode.
        # The range 0.4 to 2.6 covers acute to obtuse rounding behaviors.
        k_candidates = [None] + list(np.linspace(0.4, 2.6, 12))
        
        # Clear existing fixed_k attributes to start fresh
        # This ensures no previous optimization artifacts interfere with the baseline
        for node in path:
            planner.graph.nodes[node].pop('fixed_k', None)

        # 2. Coordinate Descent Loop (2 Passes)
        # We perform up to 2 full passes over all nodes to allow changes to propagate.
        # If node A changes, it might enable node B to improve further in the next pass.
        for pass_idx in range(2):
            changes_made = False
            
            # Iterate over intermediate nodes only (excluding Start and Goal)
            for i in range(1, len(path) - 1):
                node_name = path[i]
                current_best_k = planner.graph.nodes[node_name].get('fixed_k')
                
                # A. Measure Baseline
                # Run the optimizer with the current configuration of the graph
                self.optimizePath(path, planner, config={'r_init': r_base})
                # Calculate the score (Length) to beat
                best_len = calculate_path_length(planner, path, use_curves=True)
                
                # B. Test Candidates
                # Try every possible k value for THIS specific node
                for k_test in k_candidates:
                    if k_test == current_best_k: continue
                    
                    # Apply Candidate K to this node temporarily
                    planner.graph.nodes[node_name]['fixed_k'] = k_test
                    
                    # Run Engine
                    # Re-optimize the path to see how the new k affects the geometry and collisions
                    self.optimizePath(path, planner, config={'r_init': r_base})
                    curr_len = calculate_path_length(planner, path, use_curves=True)
                    
                    # Check for improvement
                    # We use a small epsilon (1mm) to avoid oscillation on float precision noise
                    if curr_len < best_len - 0.001: 
                        best_len = curr_len
                        current_best_k = k_test
                        changes_made = True
                
                # C. Lock in the Winner
                # The best k found becomes the fixed setting for this node before moving to the next node
                planner.graph.nodes[node_name]['fixed_k'] = current_best_k

            # Convergence Check: If a full pass resulted in no changes, we have found a local minimum.
            if not changes_made:
                break
                
        
        # Final Run
        # Execute one last optimization with the fully tuned parameters to ensure 
        # the graph state (P2n, r) exactly matches the best found configuration.
        return self.optimizePath(path, planner, config={'r_init': r_base})