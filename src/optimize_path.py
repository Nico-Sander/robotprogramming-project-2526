import numpy as np

class OptimizeFlyby():
    def __init__(self):
        pass

    def optimizePath(self, path: list = [], planner = None, config: dict = {}):
        """Implements the the first model of inverse rounding (slide 101).
        
        Args:
            path (list): Path to optimize.
            planner (Planner): Planner to use.
            config (dict): Configuration.
                           - 'r_init': Initial max radius (0.49)
                           - 'k': Optional. If set, forces a fixed ratio. 
                             (k=1.0 is relative symmetry, k!=1.0 is skewed)

        Returns:
            optimized path (list): tuples with node_name and respective r-value
        """

        # Configuration defaults
        r_init = config.get('r_init', 0.49)
        r_step = config.get('r_step', 0.01)
        r_min = 0.02
        
        # Check if k is provided in config (Fixed Mode)
        fixed_k = config.get('k', None)

        cc = planner._collisionChecker
        optimized_results = []

        for i, node_name in enumerate(path):
            # 1. Handle start and end
            if i == 0 or i == len(path)-1:
                planner.graph.nodes[node_name]['r'] = 0.0
                optimized_results.append((node_name, 0.0))
                continue

            # 2. Get Coordinates
            p_prev = np.array(planner.graph.nodes[path[i-1]]['pos'])
            p_curr = np.array(planner.graph.nodes[node_name]['pos'])
            p_next = np.array(planner.graph.nodes[path[i+1]]['pos'])

            # 3. Determine k and max_r
            dist_prev = np.linalg.norm(p_curr - p_prev)
            dist_next = np.linalg.norm(p_next - p_curr)
            
            # --- CASE A: FIXED K (User Configured) ---
            if fixed_k is not None:
                k = fixed_k
                
                # In Fixed Mode, we define r on the INCOMING segment.
                # r applies to prev, (r*k) applies to next.
                # We must ensure both r and (r*k) are <= 0.49.
                # So r must be <= 0.49 AND r <= 0.49/k.
                max_safe_r = min(0.49, 0.49 / k)
                
                # Initialize r
                r = min(r_init, max_safe_r)
                
            # --- CASE B: DYNAMIC K (Metric Symmetry) ---
            else:
                prev_is_shorter = dist_prev <= dist_next
                if prev_is_shorter:
                    shortest = dist_prev
                    longest = dist_next
                else:
                    shortest = dist_next
                    longest = dist_prev
                
                # k scales the longer side to match the shorter side
                k = shortest / longest if longest > 0 else 1.0
                
                # In Dynamic Mode, r is always relative to the shortest side, 
                # so the max is simply 0.49.
                r = min(r_init, 0.49)

            # 4. Greedy Search Optimization
            best_r = 0.0
            
            while r >= r_min:
                
                # 5. Calculate S and E
                if fixed_k is not None:
                    # Fixed k: r is on incoming (prev), r*k is on outgoing (next)
                    S = p_curr + r * (p_prev - p_curr)
                    E = p_curr + (r * k) * (p_next - p_curr)
                else:
                    # Dynamic k: r is on the shorter side
                    if prev_is_shorter:
                        S = p_curr + r * (p_prev - p_curr)
                        E = p_curr + (r * k) * (p_next - p_curr)
                    else:
                        S = p_curr + (r * k) * (p_prev - p_curr)
                        E = p_curr + r * (p_next - p_curr)

                # 6. Inverse Rounding Control Point
                p_2n = 2 * p_curr - 0.5 * S - 0.5 * E

                # 7. Check the resulting parabola for collisions
                if cc.curveInCollision(S, p_2n, E, steps=40):
                    r -= r_step
                    continue

                best_r = r
                break
            
            # Save r (and k logic if needed) to graph
            planner.graph.nodes[node_name]['r'] = best_r
            # Hint: You might want to save 'k' or the mode here too if your visualization 
            # depends on recalculating geometry later.
            if fixed_k is not None:
                 planner.graph.nodes[node_name]['fixed_k'] = fixed_k
                 
            optimized_results.append((node_name, best_r))

        return optimized_results
