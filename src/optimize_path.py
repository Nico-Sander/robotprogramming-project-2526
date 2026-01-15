import numpy as np
import matplotlib.pyplot as plt

from analysis import calculate_path_length, clear_graph_attributes, retrieve_path_positions

class OptimizeFlyby():
    def __init__(self):
        pass
    def optimizePath(self, path: list = [], planner = None, config: dict = {}):
        
        
        optimized_path = []

    def optimizePath(self, path: list = [], planner = None, config: dict = {}):
        """
        Main optimization routine. 
        """
        r_init = config.get('r_init', 0.49)
        r_step = config.get('r_step', 0.01)
        r_min = 0.02
        global_k = config.get('k', None) # Global fallback

        cc = planner._collisionChecker
        
        r_map = {node: r_init for node in path}
        p2n_map = {name: np.array(planner.graph.nodes[name]['pos']) for name in path}

        # --- Helper: Geometry Calculation ---
        def get_tangent_points(P_prev, P_curr, P_next, r_val, node_name):
            d_in = np.linalg.norm(P_curr - P_prev)
            d_out = np.linalg.norm(P_next - P_curr)
            
            # 1. Check if this specific node has a fixed 'k' set
            node_k = planner.graph.nodes[node_name].get('fixed_k')
            
            # 2. Priority: Node Attribute > Global Config > None (Dynamic)
            target_k = node_k if node_k is not None else global_k

            if target_k is not None:
                lam_in, lam_out = r_val, r_val * target_k
            else:
                # Dynamic Metric Symmetry
                if d_in <= d_out:
                    dist = r_val * d_in
                    lam_in = r_val
                    lam_out = dist / d_out if d_out > 0 else 0
                else:
                    dist = r_val * d_out
                    lam_out = r_val
                    lam_in = dist / d_in if d_in > 0 else 0
            
            vec_in = P_prev - P_curr
            vec_out = P_next - P_curr
            S = P_curr + lam_in * vec_in
            E = P_curr + lam_out * vec_out
            return S, E

        # --- Main Optimization Loop ---
        max_iterations = 50
        for _ in range(max_iterations):
            path_valid = True
            
            # 1. RELAXATION PHASE
            for _relax in range(15): 
                for i in range(1, len(path) - 1):
                    node = path[i]
                    prev_node = path[i-1]
                    next_node = path[i+1]
                    
                    P_org = np.array(planner.graph.nodes[node]['pos'])
                    P_prev_p2n = p2n_map[prev_node]
                    P_curr_p2n = p2n_map[node]
                    P_next_p2n = p2n_map[next_node]
                    r = r_map[node]
                    
                    d_in = np.linalg.norm(P_curr_p2n - P_prev_p2n)
                    d_out = np.linalg.norm(P_next_p2n - P_curr_p2n)
                    
                    # Logic to pick K for relaxation
                    node_k = planner.graph.nodes[node].get('fixed_k')
                    target_k = node_k if node_k is not None else global_k

                    if target_k is not None:
                        li, lo = r, r * target_k
                    else:
                        if d_in == 0 or d_out == 0: li, lo = r, r
                        elif d_in <= d_out:
                            li = r
                            lo = (r * d_in) / d_out
                        else:
                            lo = r
                            li = (r * d_out) / d_in

                    numerator = 4 * P_org - li * P_prev_p2n - lo * P_next_p2n
                    denominator = 4 - li - lo
                    if abs(denominator) < 0.001: denominator = 0.001
                    p2n_map[node] = numerator / denominator

            # 2. COLLISION CHECK PHASE
            p2n_list = [p2n_map[name] for name in path]
            current_geom = [] 
            for i in range(1, len(path) - 1):
                # Pass node_name to helper
                S, E = get_tangent_points(p2n_list[i-1], p2n_list[i], p2n_list[i+1], r_map[path[i]], path[i])
                current_geom.append({'S': S, 'E': E})

            # Check Segments and Curves
            
            # Start Segment
            start_node = p2n_list[0]
            first_S = current_geom[0]['S']
            if np.dot(p2n_list[1]-start_node, first_S-start_node) < 0 or cc.lineInCollision(start_node, first_S):
                 path_valid = False
                 r_map[path[1]] = max(r_min, r_map[path[1]] - r_step)

            # Connectors
            for i in range(len(current_geom) - 1):
                end_pt_prev = current_geom[i]['E']
                start_pt_next = current_geom[i+1]['S']
                vec_p2n = p2n_list[i+2] - p2n_list[i+1]
                vec_connect = start_pt_next - end_pt_prev
                
                if np.dot(vec_p2n, vec_connect) < 0 or cc.lineInCollision(end_pt_prev, start_pt_next):
                    path_valid = False
                    r_map[path[i+1]] = max(r_min, r_map[path[i+1]] - r_step)
                    r_map[path[i+2]] = max(r_min, r_map[path[i+2]] - r_step)

            # End Segment
            last_E = current_geom[-1]['E']
            end_node = p2n_list[-1]
            if np.dot(end_node-p2n_list[-2], end_node-last_E) < 0 or cc.lineInCollision(last_E, end_node):
                 path_valid = False
                 r_map[path[-2]] = max(r_min, r_map[path[-2]] - r_step)

            # Curves
            for i in range(len(current_geom)):
                geom = current_geom[i]
                node_name = path[i+1]
                if cc.curveInCollision(geom['S'], p2n_list[i+1], geom['E'], steps=20):
                    path_valid = False
                    r_map[node_name] = max(r_min, r_map[node_name] - r_step)
            
            if path_valid:
                break
        
        # --- Final Save ---
        results = []
        for i, node_name in enumerate(path):
            val = r_map[node_name]
            if val <= r_min: val = 0.0
            if i == 0 or i == len(path)-1: val = 0.0
            
            planner.graph.nodes[node_name]['r'] = val
            
            if val > 0 and i > 0 and i < len(path)-1:
                planner.graph.nodes[node_name]['P2n'] = p2n_map[node_name]
            else:
                planner.graph.nodes[node_name]['P2n'] = np.array(planner.graph.nodes[node_name]['pos'])
            
            # --- FIX: Save global_k to the graph if it was used ---
            # This ensures calculate_path_length uses the correct geometry
            if global_k is not None:
                planner.graph.nodes[node_name]['fixed_k'] = global_k

            results.append((node_name, val))
            
        return results

    def analyze_optimal_k(self, planner, node_names, r_fixed=0.5, plot=True):
        """
        Performs a parameter sweep to find the optimal global 'k'.
        Returns detailed results to allow custom visualization.
        """
        # 1. Define range of k to test
        k_values = np.linspace(0.1, 3.0, 30)
        lengths = []
        
        if plot:
            print(f"--- Starting K-Sweep for r_init={r_fixed} ---")
        
        # 2. Sweep Loop
        for k in k_values:
            # Clear for fair comparison
            clear_graph_attributes(planner)
            
            # Run Optimizer (Sweep)
            config = {'r_init': r_fixed, 'k': k}
            self.optimizePath(node_names, planner, config)
            
            # Measure Length
            length = calculate_path_length(planner, node_names, use_curves=True)
            lengths.append(length)

        # 3. Find the Minimum
        min_length = min(lengths)
        min_idx = lengths.index(min_length)
        best_k = k_values[min_idx]
        
        if plot:
            print(f"--- Result ---")
            print(f"Optimal Global k: {best_k:.2f}")
            print(f"Minimum Length:   {min_length:.4f}m")
            
            # Visualization (Internal)
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
        # Apply the best k found to the graph so it persists for the caller
        if plot:
            print(f"Applying optimal k={best_k:.2f} to graph...")
            
        clear_graph_attributes(planner)
        config = {'r_init': r_fixed, 'k': best_k}
        final_path = self.optimizePath(node_names, planner, config)
        
        # 5. Return Rich Data
        return {
            'best_k': best_k,
            'min_length': min_length,
            'k_values': k_values,
            'lengths': lengths,
            'optimized_path': final_path
        }

    def optimize_individual_corners(self, path, planner, config={}):
        """
        Optimizes 'k' individually for every node using Coordinate Descent.
        """

        # 1. Config
        r_base = config.get('r_init', 0.5)
        # We test a range of skews. 'None' represents the default dynamic/symmetric mode.
        k_candidates = [None] + list(np.linspace(0.4, 2.6, 12))
        
        print(f"Starting Individual Corner Optimization...")
        
        # Clear existing fixed_k attributes to start fresh
        for node in path:
            planner.graph.nodes[node].pop('fixed_k', None)

        # 2. Coordinate Descent Loop (2 Passes)
        for pass_idx in range(2):
            changes_made = False
            print(f"--- Pass {pass_idx + 1} ---")
            
            for i in range(1, len(path) - 1):
                node_name = path[i]
                current_best_k = planner.graph.nodes[node_name].get('fixed_k')
                
                # A. Measure Baseline
                self.optimizePath(path, planner, config={'r_init': r_base})
                best_len = calculate_path_length(planner, path, use_curves=True)
                
                # B. Test Candidates
                for k_test in k_candidates:
                    if k_test == current_best_k: continue
                    
                    # Apply Candidate K to this node
                    planner.graph.nodes[node_name]['fixed_k'] = k_test
                    
                    # Run Engine
                    self.optimizePath(path, planner, config={'r_init': r_base})
                    curr_len = calculate_path_length(planner, path, use_curves=True)
                    
                    # Check for improvement
                    if curr_len < best_len - 0.001: 
                        best_len = curr_len
                        current_best_k = k_test
                        changes_made = True
                
                # C. Lock in the Winner
                planner.graph.nodes[node_name]['fixed_k'] = current_best_k

            if not changes_made:
                print("Converged early.")
                break
                
        print(f"Optimization Complete.")
        
        # Final Run
        return self.optimizePath(path, planner, config={'r_init': r_base})
