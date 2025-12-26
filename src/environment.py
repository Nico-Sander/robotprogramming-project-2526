from shapely.geometry import Polygon, Point
from planners.IPPerfMonitor import IPPerfMonitor
import numpy as np
import matplotlib.pyplot as plt
from shapely import plotting

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
    env_3["obs_2"] = Polygon([(0.0, 11.0), (0.0, 13.0), (6.0, 13.0), (6.0, 11.0)])
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
        Polygon([(0, 12.0), (8, 16.0), (7.5, 12), (14, 11.0), (20.0, 0), (20, 20), (0, 20)]),
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

    def parabolaInCollision(self, startPos, anchorPos, endPos, r):
        """ Check whether a parabola from startPos to endPos is colliding"""
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())

        P1 = np.array(startPos)
        P2 = np.array(anchorPos)
        P3 = np.array(endPos)
        
        for t in np.linspace(0, 1, 40):
            testPoint = r * (P2 - P1) * (t - 1.0)**2 + r * (P2 - P3) * t**2 + P2
            if self.pointInCollision(testPoint)==True:
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

               