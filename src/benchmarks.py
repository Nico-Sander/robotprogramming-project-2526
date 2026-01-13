from shapely import Polygon, plotting
import numpy as np

X_LIMITS = [0.0, 20.0]
Y_LIMITS = [0.0, 20.0]

def fillet_corner_if_exists(poly: Polygon, corner_xy, r: float, n_arc: int = 24, tol: float = 1e-6) -> Polygon:
    """
    Utility function to round a specific corner of a Polygon.
    Used for creating benchmark environments with rounded obstacles.
    """
    coords = list(poly.exterior.coords)[:-1]
    cx, cy = corner_xy

    # 1. Find the index of the vertex matching corner_xy
    corner_idx = None
    for i, (x, y) in enumerate(coords):
        if abs(x - cx) <= tol and abs(y - cy) <= tol:
            corner_idx = i
            break

    if corner_idx is None:
        return poly

    # 2. Identify neighbors
    n = len(coords)
    Pm = np.array(coords[(corner_idx - 1) % n], float)
    P  = np.array(coords[corner_idx], float)
    Pp = np.array(coords[(corner_idx + 1) % n], float)

    # 3. Calculate tangent geometry
    u1 = (Pm - P); u1 = u1 / np.linalg.norm(u1)
    u2 = (Pp - P); u2 = u2 / np.linalg.norm(u2)

    ang = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
    d = r / np.tan(ang / 2.0)

    T1 = P + u1 * d
    T2 = P + u2 * d

    # 4. Generate Arc Points
    bis = (u1 + u2); bis = bis / np.linalg.norm(bis)
    h = r / np.sin(ang / 2.0)
    C = P + bis * h

    a1 = np.arctan2(T1[1] - C[1], T1[0] - C[0])
    a2 = np.arctan2(T2[1] - C[1], T2[0] - C[0])
    da = (a2 - a1 + np.pi) % (2*np.pi) - np.pi

    arc = []
    for k in range(n_arc + 1):
        a = a1 + da * (k / n_arc)
        arc.append((C[0] + r * np.cos(a), C[1] + r * np.sin(a)))

    # 5. Reconstruct Polygon
    new_coords = []
    for i in range(n):
        if i == corner_idx:
            new_coords.append((T1[0], T1[1]))
            new_coords.extend(arc[1:-1])
            new_coords.append((T2[0], T2[1]))
        else:
            new_coords.append(coords[i])

    return Polygon(new_coords)

def construct_benchmark_environments():
    """
    Generates a dictionary of benchmark environments.
    Each environment contains:
    - 'env': Dictionary of Shapely Polygons (Obstacles)
    - 'smooth_path': List of coordinate tuples representing the ideal path
    """
    env_dict = dict()

    ## Environment 1
    env_1 = dict()
    env_1["obs_1"] = Polygon([(5.0, 0.0), (8.0, 0.0), (8.0, 9.0), (5.0, 9.0)])
    env_1["obs_2"] = Polygon([(11.0, 0.0), (16.0, 0.0), (16.0, 6.0), (11.0, 6.0)])
    env_1["obs_3"] = fillet_corner_if_exists(
        Polygon([(10.0, 8.0), (15.0, 8.0), (15.0, 20.0), (10.0, 20.0)]), (10.0, 8.0), 3
    )
    start = (2.0, 2.0)
    goal = (18.0, 2.0)
    solution_positions = [start, (4.5, 9.5), (9.0, 10.0), (12.0, 7.0), (16.0, 6.5), goal]

    env_dict["1"] = {"env": env_1, "smooth_path": solution_positions}

    ## Environment 2
    env_2 = dict()
    env_2["obs_1"] = Polygon([(0.0, 2.0),(15.0, 2.0), (15.5, 1.5), (16.0, 2.0), (16.0, 3.0), (0.0, 3.0)])
    env_2["obs_2"] = Polygon([(10.0, 5.0), (20.0, 5.0), (20.0, 6.0), (10.0, 6.0)])
    env_2["obs_3"] = Polygon([(0.0, 8.0), (16.0, 8.0), (16.0, 10.0), (0.0, 10.0)])
    env_2["obs_4"] = Polygon([(2.0, 11.0), (20.0, 11.0), (20.0, 12.0), (2.0, 12.0)])
    env_2["obs_5"] = Polygon([(0.0, 14.0), (5.0, 14.0), (5.0, 15.0), (0.0, 15.0)])
    env_2["obs_6"] = Polygon([(4.0, 17.0), (20.0, 17.0), (20.0, 18.0), (4.0, 18.0)])
    env_2["obs_7"] = Polygon([(17.2, 0), (20.0, 0), (20.0, 5.0), (17.2, 5.0)])
    start = (3.0, 1.0)
    goal = (10.0, 19.0)
    solution_positions = [start, (17.0, 1.0), (17.0, 4.0), (9.5, 4.0), (9.5, 7.0), (17.0, 7.0), (17.0, 10.5), 
                        (1.0, 10.5), (1.0, 13.0), (6.0, 13.0), (6.0, 16.0), (3.0, 16.0), (3.0, 19.0), goal]

    env_dict["2"] = {"env": env_2, "smooth_path": solution_positions}

    ## Environment 3
    env_3 = dict()
    env_3["obs_1"] = Polygon([(6.0, 17.0), (7.0, 15.0), (20.0, 18.0), (20.0, 20.0)])
    env_3["obs_2"] = Polygon([(0.0, 10.2), (0.0, 13.0), (6.0, 13.0), (6.0, 10.2)])
    env_3["obs_3"] = fillet_corner_if_exists(
        Polygon([(3.0, 7.0), (3.0, 9.0), (8.0, 9.0), (8.0, 7.0)]), (3.0, 7.0), 2   
    )
    env_3["obs_4"] = Polygon([(8.0, 1.0), (7.0, 3.0), (12.0, 8.0), (13.0, 6.0)])
    env_3["obs_5"] = Polygon([(15.0, 8.0), (15.0, 15.0), (8.0, 15.0), (8.0, 8.0), (9.5, 8.0), (9.5, 10.0), 
                              (9.0, 10.0), (9.0, 13.0), (13.0, 13.0), (13.0, 8.0)])
    start = (7.0, 19.0)
    goal = (10.0, 12.0)
    solution_positions = [start, (4.0, 15.0), (7.0, 13.0), (7.0, 10.0), (2.0, 10.0), (2.0, 3.0), (12.0, 9.0), goal]

    env_dict["3"] = {"env": env_3, "smooth_path": solution_positions}

    ## Environment 4
    env_4 = dict()
    env_4["obs_1"] = fillet_corner_if_exists(
        Polygon([(0, 9.5), (6, 12.5), (5, 7), (13, 13), (17, 0), (0, 0)]), (13.0, 13.0), 4.5
    )
    env_4["obs_2"] = fillet_corner_if_exists(
        Polygon([(0, 12.0), (8, 16.0), (6.5, 11.5), (14, 11.0), (20.0, 0), (20, 20), (0, 20)]), (8.0, 16.0), 1
    )
    start = (1.0, 11.25)
    goal = (18.25, 1.0)
    solution_positions = [start, (6, 13), (6.5, 10), (13.5, 10.5), goal]

    env_dict["4"] = {"env": env_4, "smooth_path": solution_positions}

    return env_dict