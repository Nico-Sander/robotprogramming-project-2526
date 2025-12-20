import numpy as np
from environment import CollisionChecker

class OptimizeFlyby():
    def __init__(self):
        pass
    def optimizePath(self, path: list = [], planner = None, config: dict = {}):
        
        
        optimized_path = []

        r_default = config.get("r_init", 0.45)


        for i, node_name in enumerate(path):
            if i == 1 or i == len(path)-1:
                optimized_path.append((node_name, 0.0))
            else:
                optimized_path.append((node_name, r_default))




import matplotlib.pyplot as plt

class OptimizeFlyby:
    def __init__(self):
        pass

    def optimizePath_firstmodel(self, path, planner, config):


        r_start = config.get('r_init', 0.49) 
        optimized_results = []
        
        for i in range(len(path)):
            node_name = path[i]
            
            # Start- und Endpunkt: r ist immer 0
            if i == 0 or i == len(path) - 1:
                optimized_results.append((node_name, 0.0))
                continue
            
            # Koordinaten aus dem Planer-Graphen abrufen
            p_prev = np.array(planner.graph.nodes[path[i-1]]['pos'])
            p_curr = np.array(planner.graph.nodes[node_name]['pos'])
            p_next = np.array(planner.graph.nodes[path[i+1]]['pos'])


            
            # Initialer r-Wert
            r = r_start

            # Berechnung des neuen idealen Punktes
            collision = True

            while collision and r >= 0.02:

                factor = 1 / (2*(r-2))
                p_2n = factor*(r*p_prev - 4*p_curr + r*p_next)

                point_coll = planner._collisionChecker.pointInCollision(p_2n)

                line1_coll = planner._collisionChecker.lineInCollision(p_prev, p_2n)
                line2_coll = planner._collisionChecker.lineInCollision(p_2n, p_next)


                if not point_coll and not line1_coll and not line2_coll:
                    collision = False
                else:
                    r -= 0.01
                    r = round(r, 2)

            if r < 0.02: r = 0

            optimized_results.append((node_name, r))

        return optimized_results
    


    def optimizePath_combinedmodel(self, path, planner, config):


        r_init = config.get('r_init', 0.49)
        r_min = config.get('r_min', 0.02)
        r_step = config.get('r_step', 0.01)
        
        cc = planner._collisionChecker
        optimized_results = []
        
        for i, node in enumerate(path):
            
            # Start- und Endpunkt: r ist immer 0
            if i == 0 or i == len(path) - 1:
                optimized_results.append((node, 0.0, "none"))
                continue
            
            # Koordinaten aus dem Planer-Graphen abrufen
            p_prev = np.array(planner.graph.nodes[path[i-1]]['pos'])
            p_curr = np.array(planner.graph.nodes[node]['pos'])
            p_next = np.array(planner.graph.nodes[path[i+1]]['pos'])

            #Verhältnis der Segmentlängen k berechnen
            dist12 = np.linalg.norm(p_curr - p_prev)
            dist23 = np.linalg.norm(p_next - p_curr)
            k = dist12 / dist23 if dist23 !=0 else 1.0
            
            best_r = 0.0
            best_mode = "none"

            #---------- beide Modelle testen -----------

            for mode in ("flyby", "cutting"):

                r = r_init

                while r >= r_min:

                    if mode == "flyby":
                        nenner = 1.0 - 0.25 * r * (1.0 + k)
                        zaehler = p_curr - 0.25 * r * (p_prev + k * p_next)
                        p_2n = zaehler / nenner

                        S = p_2n + r * (p_prev - p_2n)
                        E = p_2n + (r * k) * (p_next - p_2n)

                    else:
                        # cutting
                        p_2n = p_curr

                        S = p_curr + r * (p_prev - p_curr)
                        E = p_curr + (r * k) * (p_next - p_curr)

                    segment_safe = (
                        not cc.pointInCollision(p_2n) and 
                        not cc.lineInCollision(S, p_2n) and 
                        not cc.lineInCollision(p_2n, E))
                    
                    parabola_safe = True
                    if segment_safe:
                        # Diskretisierung der Kurve in 10 Schritte
                        for t in np.linspace(0, 1, 10):
                            # Quadratische Bézier-Formel
                            p_curve = (1-t)**2 * S + 2*(1-t)*t * p_2n + t**2 * E
                            if cc.pointInCollision(p_curve):
                                parabola_safe = False
                                break

                    if segment_safe and parabola_safe:
                        if r > best_r:
                            best_r = r
                            best_mode = mode
                        break

                    r -= r_step
                    r = round(r, 3)

            optimized_results.append((node, best_r, best_mode))

        return optimized_results

                    

