import numpy as np

class OptimizeFlyby():
    def __init__(self):
        pass

    def optimizePath(self,path: list = [], planner = None, config: dict = {}):
        """Returns name of node and the Überschleifwert (r) belonging to it.
        
        Args:
            path (list): Path to optimize.
            planner (Planner): Planner to use.
            config (dict): Configuration.
        """

        # Configuration defaults
        r_init = config.get('r_init', 0.49)
        r_step = config.get('r_step', 0.01)
        r_min = 0.02

        cc = planner._collisionChecker
        optimized_results = []

        for i, node_name in enumerate(path):
            # 1. Handle start and end
            if i == 0 or i == len(path)-1:
                optimized_results.append((node_name, 0.0))
                continue

            # 2. Get Coordinates of the previous, current and next node
            p_prev = np.array(planner.graph.nodes[path[i-1]]['pos'])
            p_curr = np.array(planner.graph.nodes[node_name]['pos'])
            p_next = np.array(planner.graph.nodes[path[i+1]]['pos'])

            # 3. Calculate Ratio k
            # This ensures S and E are the same distance from the corner
            dist_prev = np.linalg.norm(p_curr - p_prev)
            dist_next = np.linalg.norm(p_next - p_curr)
            
            # Determine which segment is the limiting factor (shortest)
            prev_is_shorter = dist_prev <= dist_next

            if prev_is_shorter:
                shortest_dist = dist_prev
                longest_dist = dist_next
            else:
                shortest_dist = dist_next
                longest_dist = dist_prev
            
            k = shortest_dist / longest_dist if longest_dist > 0 else 1.0

            # 4. Greedy Search Optimization
            best_r = 0.0
            r = r_init

            while r >= r_min:
                # r is relative to the shortest segment -> r*k must therefor be smaller than r
                if r > 0.49:
                    r = 0.49
                    
                # 5. Calculate S and E based on which side is shorter
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
            
            planner.graph.nodes[node_name]['r'] = best_r
            optimized_results.append((node_name, best_r))

        return optimized_results
