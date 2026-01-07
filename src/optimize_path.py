import numpy as np

class OptimizeFlyby():
    """
    Optimizer class that implements 'Flyby' smoothing (Inverse Rounding).
    
    This technique replaces sharp corners in a path with smooth quadratic Bezier curves.
    It uses a G1-continuous approach where:
    1. A new 'Control Polygon' is formed by moving the original corner nodes to new positions (P2n).
    2. Bezier curves are defined within this new polygon.
    3. The curves are constrained to pass exactly through the original path nodes.
    """
    def __init__(self):
        pass

    def optimizePath(self, path: list = [], planner = None, config: dict = {}):
        """
        Main optimization routine. 
        
        Iteratively adjusts the path geometry to maximize smoothness (radius 'r') 
        while avoiding collisions with the environment.

        Side Effects:
            Modifies the 'planner.graph' by adding the following attributes to nodes:
            - 'r': The optimized radius (float).
            - 'P2n': The calculated Control Point position (np.array).
            - 'fixed_k': The asymmetry factor (float), if configured.

        Args:
            path (list): List of node IDs representing the path to optimize.
            planner (Planner): The planner object containing the graph and collision checker.
            config (dict): Configuration options:
                - 'r_init': Initial dimensionless radius (0.0 to 0.5). Default 0.49.
                - 'r_step': Step size for reducing r upon collision. Default 0.01.
                - 'k': Optional fixed ratio for curve asymmetry. If None, k is dynamic.

        Returns:
            optimized_results (list): A list of tuples (node_name, final_r_value).
        """
        # --- Configuration Setup ---
        r_init = config.get('r_init', 0.49)
        r_step = config.get('r_step', 0.01)
        r_min = 0.02
        fixed_k = config.get('k', None)

        cc = planner._collisionChecker
        
        # Initialize the radius 'r' for all nodes to the maximum desired value.
        # This value is shrunk locally if collisions are detected.
        r_map = {node: r_init for node in path}
        
        # 'p2n_map' stores the position of the Control Points (P2n).
        # Initially, they are placed exactly at the original node positions.
        p2n_map = {name: np.array(planner.graph.nodes[name]['pos']) for name in path}

        # --- Helper Function: Geometry Calculation ---
        def get_tangent_points(P_prev, P_curr, P_next, r_val):
            """
            Calculates the Start (S) and End (E) points of the curve on the segments
            connecting the Control Points (P2n).
            
            The positions are determined by the radius 'r' and asymmetry 'k'.
            - S is located on the segment P_prev -> P_curr
            - E is located on the segment P_curr -> P_next
            """
            # Calculate lengths of the current control polygon segments
            d_in = np.linalg.norm(P_curr - P_prev)
            d_out = np.linalg.norm(P_next - P_curr)
            
            # Determine the ratio (lambda) for S and E positions.
            # If k is fixed, ratios are constant.
            # If k is dynamic, ratios are adjusted to ensure the curve starts and ends
            # at equidistant points from the corner (Metric Symmetry).
            if fixed_k is not None:
                lam_in, lam_out = r_val, r_val * fixed_k
            else:
                # Metric Symmetry Logic: Ensure dist(S, P_curr) == dist(E, P_curr)
                # The curve is limited based on the shorter segment.
                if d_in <= d_out:
                    dist = r_val * d_in
                    lam_in = r_val
                    lam_out = dist / d_out if d_out > 0 else 0
                else:
                    dist = r_val * d_out
                    lam_out = r_val
                    lam_in = dist / d_in if d_in > 0 else 0
            
            # Vector calculation of the points
            vec_in = P_prev - P_curr
            vec_out = P_next - P_curr
            S = P_curr + lam_in * vec_in
            E = P_curr + lam_out * vec_out
            return S, E

        # --- Main Optimization Loop ---
        # Alternates between "relaxing" the geometry (finding valid Control Points)
        # and checking for collisions. If a collision occurs, 'r' is reduced.
        max_iterations = 50
        for _ in range(max_iterations):
            path_valid = True
            
            # 1. RELAXATION PHASE (Positioning the Control Points)
            # In this phase, the Control Points (P2n) are moved. The goal is to find a position
            # for P2n such that the resulting Bezier curve passes exactly through the
            # original path node P_original.
            # This is an iterative process because moving one P2n affects its neighbors.
            for _relax in range(15): 
                for i in range(1, len(path) - 1):
                    node = path[i]
                    prev_node = path[i-1]
                    next_node = path[i+1]
                    
                    # Retrieve current positions
                    P_org = np.array(planner.graph.nodes[node]['pos'])
                    P_prev_p2n = p2n_map[prev_node]
                    P_curr_p2n = p2n_map[node]
                    P_next_p2n = p2n_map[next_node]
                    r = r_map[node]
                    
                    # Calculate segment lengths
                    d_in = np.linalg.norm(P_curr_p2n - P_prev_p2n)
                    d_out = np.linalg.norm(P_next_p2n - P_curr_p2n)
                    
                    # Determine weighting factors (lambdas) for the relaxation equation.
                    # These match the logic in 'get_tangent_points'.
                    if fixed_k is not None:
                        li, lo = r, r * fixed_k
                    else:
                        if d_in == 0 or d_out == 0: li, lo = r, r
                        elif d_in <= d_out:
                            li = r
                            lo = (r * d_in) / d_out
                        else:
                            lo = r
                            li = (r * d_out) / d_in

                    # Solve for the new P2n Position (P_curr_p2n).
                    # Derived from the quadratic Bezier equation at t=0.5.
                    numerator = 4 * P_org - li * P_prev_p2n - lo * P_next_p2n
                    denominator = 4 - li - lo
                    if abs(denominator) < 0.001: denominator = 0.001
                    p2n_map[node] = numerator / denominator

            # 2. COLLISION CHECK PHASE
            # Ensure the consistent geometry is safe.
            p2n_list = [p2n_map[name] for name in path]
            
            # Pre-calculate S and E for all nodes in this pass
            current_geom = [] 
            for i in range(1, len(path) - 1):
                S, E = get_tangent_points(p2n_list[i-1], p2n_list[i], p2n_list[i+1], r_map[path[i]])
                current_geom.append({'S': S, 'E': E})

            # Four parts of the path are checked for collisions:
            
            # A. The Start Segment: From Start Node -> S of the first curve
            start_node = p2n_list[0]
            first_S = current_geom[0]['S']
            
            # Geometric Sanity Check: Ensure the segment points in the right direction (no overlap)
            if np.dot(p2n_list[1]-start_node, first_S-start_node) < 0:
                 path_valid = False
                 r_map[path[1]] = max(r_min, r_map[path[1]] - r_step)
            # Collision Check
            elif cc.lineInCollision(start_node, first_S):
                 path_valid = False
                 r_map[path[1]] = max(r_min, r_map[path[1]] - r_step)

            # B. The Intermediate Straight Connectors: From E of curve i -> S of curve i+1
            for i in range(len(current_geom) - 1):
                end_pt_prev = current_geom[i]['E']
                start_pt_next = current_geom[i+1]['S']
                
                # Geometric Sanity Check: Ensure connector aligns with the control polygon
                vec_p2n = p2n_list[i+2] - p2n_list[i+1]
                vec_connect = start_pt_next - end_pt_prev
                
                if np.dot(vec_p2n, vec_connect) < 0:
                    path_valid = False
                    # Overlap detected: shrink r for both involved nodes
                    r_map[path[i+1]] = max(r_min, r_map[path[i+1]] - r_step)
                    r_map[path[i+2]] = max(r_min, r_map[path[i+2]] - r_step)
                    continue

                if cc.lineInCollision(end_pt_prev, start_pt_next):
                    path_valid = False
                    r_map[path[i+1]] = max(r_min, r_map[path[i+1]] - r_step)
                    r_map[path[i+2]] = max(r_min, r_map[path[i+2]] - r_step)

            # C. The End Segment: From E of the last curve -> Goal Node
            last_E = current_geom[-1]['E']
            end_node = p2n_list[-1]
            
            if np.dot(end_node-p2n_list[-2], end_node-last_E) < 0:
                 path_valid = False
                 r_map[path[-2]] = max(r_min, r_map[path[-2]] - r_step)
            elif cc.lineInCollision(last_E, end_node):
                 path_valid = False
                 r_map[path[-2]] = max(r_min, r_map[path[-2]] - r_step)

            # D. The Curves Themselves: From S -> E (guided by P2n)
            for i in range(len(current_geom)):
                geom = current_geom[i]
                node_name = path[i+1] # Note offset: geom[0] corresponds to path[1]
                
                # Check the actual curve trajectory
                if cc.curveInCollision(geom['S'], p2n_list[i+1], geom['E'], steps=20):
                    path_valid = False
                    r_map[node_name] = max(r_min, r_map[node_name] - r_step)
            
            # If all checks pass without shrinking any r, optimization is complete
            if path_valid:
                break
        
        # --- Finalization: Save Results to Graph ---
        optimized_results = []
        for i, node_name in enumerate(path):
            val = r_map[node_name]
            
            # Clean up near-zero values for cleaner output
            if val <= r_min: val = 0.0
            if i == 0 or i == len(path)-1: val = 0.0
            
            # 1. Save optimized radius
            planner.graph.nodes[node_name]['r'] = val
            
            # 2. Save Control Point (P2n) Position
            # Only strictly necessary for intermediate nodes with r > 0
            if val > 0 and i > 0 and i < len(path)-1:
                planner.graph.nodes[node_name]['P2n'] = p2n_map[node_name]
            else:
                # Fallback: P2n is at the node position
                planner.graph.nodes[node_name]['P2n'] = np.array(planner.graph.nodes[node_name]['pos'])

            # 3. Save Configuration attributes (needed for visualization)
            if fixed_k is not None:
                planner.graph.nodes[node_name]['fixed_k'] = fixed_k
            
            optimized_results.append((node_name, val))
            
        return optimized_results