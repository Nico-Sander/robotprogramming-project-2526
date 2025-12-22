import numpy as np
from environment import CollisionChecker
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MatplotPolygon

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

                    # Sicherheitscheck: r und r*k drüfen nicht >0.5 sein

                    r_S = r
                    r_E = r * k

                    # Wenn r_S oder r_E die Segmentmitte überschreiten, r verkleinern
                    if r_S > 0.5 or r_E > 0.5:
                        r-= r_step
                        continue

                    S = p_curr + r * (p_prev - p_curr)
                    E = p_curr + (r * k) * (p_next - p_curr)

                    if mode == "flyby":
                        #nenner = 1.0 - 0.25 * r * (1.0 + k)
                        #zaehler = p_curr - 0.25 * r * (p_prev + k * p_next)
                        #p_2n = zaehler / nenner

                        p_2n = 2 * p_curr - 0.5 * S - 0.5 * E

                    else:
                        # cutting
                        p_2n = p_curr

                        #S = p_curr + r * (p_prev - p_curr)
                        #E = p_curr + (r * k) * (p_next - p_curr)

                    segment_safe = (
                        not cc.pointInCollision(p_2n) and 
                        not cc.lineInCollision(S, p_2n) and 
                        not cc.lineInCollision(p_2n, E))
                    
                    parabola_safe = True
                    if segment_safe:
                        # Diskretisierung der Kurve in 20 Schritte
                        for t in np.linspace(0, 1, 20):
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

                    

    def visualize_results(self, env_id, results, env_dict, figsize: tuple=(8, 8))-> plt.Axes:
        data = env_dict[str(env_id)]
        planner = data['planner']
        obs_dict = data['env']
        original_path = np.array(data['smooth_path'])
        
        fig, ax = plt.subplots(figsize=figsize)
        
        # 1. Achsen & Limits (Dein Stil)
        try:
            limits = planner.environment.limits 
            ax.set_xlim(limits[0][0], limits[0][1])
            ax.set_ylim(limits[1][0], limits[1][1])
            ax.set_xticks(range(0, int(limits[0][1]) + 1))
            ax.set_yticks(range(0, int(limits[1][1]) + 1))
        except:
            pass
        ax.grid(True, linestyle='-', alpha=0.5)

        # 2. Hindernisse (Rot, wie gewünscht)
        for obs_name, poly in obs_dict.items():
            if hasattr(poly, 'exterior'):
                x, y = poly.exterior.xy
                ax.add_patch(MatplotPolygon(np.column_stack((x, y)), color='red', alpha=0.25, zorder=1))

        # 3. Originaler Pfad (Referenz)
        ax.plot(original_path[:, 0], original_path[:, 1], 'k--', alpha=0.2, label='Referenzpfad', zorder=2)

        # 4. Kurven, Kontrollpunkte und Segmente
        for i in range(1, len(original_path) - 1):
            p1, p2, p3 = original_path[i-1], original_path[i], original_path[i+1]
            node_name, r, mode = results[i]
            
            if r > 0:
                dist12 = np.linalg.norm(p1 - p2)
                dist23 = np.linalg.norm(p3 - p2)
                k = dist12 / dist23 if dist23 != 0 else 1.0
                
                S = p2 + r * (p1 - p2)
                E = p2 + (r * k) * (p3 - p2)
                
                # Berechnung P2n (deine Logik)
                nenner = 1.0 - 0.25 * r * (1.0 + k)
                zaehler = p2 - 0.25 * r * (p1 + k * p3)
                p2n_flyby = zaehler / nenner
                
                if mode == "flyby":
                    p2n = p2n_flyby
                    color = 'blue'
                else:
                    p2n = p2 - (p2n_flyby - p2)
                    color = 'green'
                
                # --- NEU: Hilfslinien zum virtuellen Punkt P2n ---
                ax.plot([S[0], p2n[0], E[0]], [S[1], p2n[1], E[1]], 
                        color=color, linestyle=':', linewidth=1, alpha=0.6, zorder=3)
                # Punkt P2n markieren
                ax.scatter(p2n[0], p2n[1], color=color, marker='x', s=30, zorder=5)
                
                # Parabel zeichnen
                t_vals = np.linspace(0, 1, 20)
                parabola = np.array([(1-t)**2 * S + 2*(1-t)*t * p2n + t**2 * E for t in t_vals])
                ax.plot(parabola[:, 0], parabola[:, 1], color=color, linewidth=2.5, zorder=4)
                
                # Verbindungslinien (schwarzer Pfad)
                if i == 1:
                    ax.plot([original_path[0][0], S[0]], [original_path[0][1], S[1]], 'k-', linewidth=1.5, zorder=3)
                
                next_r = results[i+1][1] if i+1 < len(original_path)-1 else 0
                if next_r == 0:
                     ax.plot([E[0], p3[0]], [E[1], p3[1]], 'k-', linewidth=1.5, zorder=3)
                else:
                    S_next = p3 + next_r * (p2 - p3)
                    ax.plot([E[0], S_next[0]], [E[1], S_next[1]], 'k-', linewidth=1.5, zorder=3)

        # 5. Start (S) und Ziel (G) Markierungen
        ax.scatter(original_path[0,0], original_path[0,1], color="lightgreen", s=400, edgecolors='k', zorder=6)
        ax.text(original_path[0,0], original_path[0,1], "S", fontweight="bold", ha="center", va="center", zorder=7)
        
        ax.scatter(original_path[-1,0], original_path[-1,1], color="lightblue", s=400, edgecolors='k', zorder=6)
        ax.text(original_path[-1,0], original_path[-1,1], "G", fontweight="bold", ha="center", va="center", zorder=7)

        ax.set_aspect('equal')
        ax.set_title(f"Optimierter Pfad (Env {env_id}) - Blau: Fly-by / Green: Cutting")
        plt.tight_layout()
        plt.show()