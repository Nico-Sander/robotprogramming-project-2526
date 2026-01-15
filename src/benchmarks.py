from shapely import Polygon
import numpy as np

X_LIMITS = [0.0, 20.0]
Y_LIMITS = [0.0, 20.0]

def fillet_corner_if_exists(poly: Polygon, corner_xy, r: float, n_arc: int = 24, tol: float = 1e-6) -> Polygon:
    """
    Smooths a specific sharp corner of a Shapely Polygon by replacing the vertex with a circular arc (fillet).

    This utility function is primarily used to generate benchmark environments containing obstacles with rounded 
    corners. It locates a target vertex within the polygon's exterior coordinates and, if found, 
    mechanically truncates the corner. It calculates the geometric tangent points where the rounding radius 
    meets the adjacent edges and generates a sequence of discretized points forming a circular arc between 
    these tangents.

    The geometric calculation relies on finding the angle bisector of the corner to position the arc's center, 
    ensuring the resulting curve is tangent to both original edges (G1 continuity).

    Parameters:
        poly (Polygon): The original Shapely Polygon object containing the sharp corner.
        corner_xy (tuple): A tuple (x, y) specifying the coordinates of the vertex to round.
        r (float): The radius of the fillet arc.
        n_arc (int): The number of discrete points used to approximate the circular arc (default: 24).
        tol (float): The numerical tolerance used when matching the `corner_xy` to the polygon's vertices (default: 1e-6).

    Returns:
        Polygon: A new Shapely Polygon object with the specified corner replaced by an approximated arc. 
                 Returns the original polygon if the corner is not found.

    Side Effects:
        None. This function is pure and returns a new object without modifying the input polygon.
    """
    # Extract the exterior coordinates (excluding the closing duplicate point)
    coords = list(poly.exterior.coords)[:-1]
    cx, cy = corner_xy

    # 1. Find the index of the vertex matching corner_xy
    # Iterate through vertices to find the index of the corner the user wants to round
    corner_idx = None
    for i, (x, y) in enumerate(coords):
        if abs(x - cx) <= tol and abs(y - cy) <= tol:
            corner_idx = i
            break

    # If the specified corner is not part of the polygon, return the original unchanged
    if corner_idx is None:
        return poly

    # 2. Identify neighbors
    # Retrieve the coordinates of the Previous (Pm), Current (P), and Next (Pp) vertices
    n = len(coords)
    Pm = np.array(coords[(corner_idx - 1) % n], float) # Previous neighbor (wrap-around using modulo)
    P  = np.array(coords[corner_idx], float)           # The corner to be rounded
    Pp = np.array(coords[(corner_idx + 1) % n], float) # Next neighbor (wrap-around using modulo)

    # 3. Calculate tangent geometry
    # Compute unit vectors pointing FROM the corner P TOWARDS the neighbors
    u1 = (Pm - P); u1 = u1 / np.linalg.norm(u1)
    u2 = (Pp - P); u2 = u2 / np.linalg.norm(u2)

    # Calculate the internal angle at the corner and the distance 'd' from P to the tangent points
    ang = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
    d = r / np.tan(ang / 2.0)

    # Determine the absolute positions of the tangent points (Start and End of the arc)
    T1 = P + u1 * d
    T2 = P + u2 * d

    # 4. Generate Arc Points
    # Calculate the center of the arc (C) lying on the angle bisector
    bis = (u1 + u2); bis = bis / np.linalg.norm(bis)
    h = r / np.sin(ang / 2.0) # Distance from corner P to center C
    C = P + bis * h

    # Determine the start and end angles of the arc relative to the center C
    a1 = np.arctan2(T1[1] - C[1], T1[0] - C[0])
    a2 = np.arctan2(T2[1] - C[1], T2[0] - C[0])
    
    # Calculate angular difference, ensuring the arc takes the shorter path (interior)
    da = (a2 - a1 + np.pi) % (2*np.pi) - np.pi

    # Generate the discretized points along the circular arc
    arc = []
    for k in range(n_arc + 1):
        a = a1 + da * (k / n_arc)
        arc.append((C[0] + r * np.cos(a), C[1] + r * np.sin(a)))

    # 5. Reconstruct Polygon
    # Build the new coordinate list by inserting the arc sequence in place of the original corner vertex
    new_coords = []
    for i in range(n):
        if i == corner_idx:
            new_coords.append((T1[0], T1[1]))   # Start tangent
            new_coords.extend(arc[1:-1])        # Arc points (excluding duplicate ends)
            new_coords.append((T2[0], T2[1]))   # End tangent
        else:
            new_coords.append(coords[i])

    return Polygon(new_coords)

def construct_benchmark_environments():
    """
    Generates a dictionary of standardized benchmark environments for testing path planning and optimization algorithms.

    This function constructs four distinct 2D scenarios ("1" through "4"), each presenting different geometric 
    challenges such as narrow corridors, sharp corners, and rounded obstacles. Each environment entry packages 
    the obstacle definitions (as Shapely Polygons) alongside a pre-calculated "smooth_path" (a list of coordinate 
    tuples) that serves as a rough reference solution or initialization for optimization routines.

    Some obstacles are procedurally modified using `fillet_corner_if_exists` to introduce non-linear boundaries, 
    testing the planner's ability to handle curved geometry.

    Parameters:
        None

    Returns:
        dict: A dictionary where keys are environment IDs ("1", "2", etc.) and values are dictionaries containing:
              - 'env' (dict): A map of obstacle names to Shapely Polygon objects.
              - 'smooth_path' (list): A list of (x, y) tuples representing the sequence of waypoints for a valid path.

    Side Effects:
        None.
    """
    env_dict = dict()

    ## Environment 1
    # A simple scenario with three block-like obstacles. 
    # One obstacle (obs_3) has a rounded corner to test basic curve handling.
    env_1 = dict()
    env_1["obs_1"] = Polygon([(5.0, 0.0), (8.0, 0.0), (8.0, 9.0), (5.0, 9.0)])
    env_1["obs_2"] = Polygon([(11.0, 0.0), (16.0, 0.0), (16.0, 6.0), (11.0, 6.0)])
    env_1["obs_3"] = fillet_corner_if_exists(
        Polygon([(10.0, 8.0), (15.0, 8.0), (15.0, 20.0), (10.0, 20.0)]), (10.0, 8.0), 3
    )
    start = (2.0, 2.0)
    goal = (18.0, 2.0)
    # Define a reference path that weaves between the blocks
    solution_positions = [start, (4.5, 9.5), (9.0, 10.0), (12.0, 7.0), (16.0, 6.5), goal]

    env_dict["1"] = {"env": env_1, "smooth_path": solution_positions}

    ## Environment 2
    # A "maze-like" environment characterized by long, horizontal barrier walls.
    # This tests the planner's ability to navigate tight, winding corridors.
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
    # A complex multi-step path requiring significant backtracking and turning
    solution_positions = [start, (17.0, 1.0), (17.0, 4.0), (9.5, 4.0), (9.5, 7.0), (17.0, 7.0), (17.0, 10.5), 
                        (1.0, 10.5), (1.0, 13.0), (6.0, 13.0), (6.0, 16.0), (3.0, 16.0), (3.0, 19.0), goal]

    env_dict["2"] = {"env": env_2, "smooth_path": solution_positions}

    ## Environment 3
    # A cluttered environment with diverse shapes (triangles, blocks, L-shapes).
    # Contains a mix of sharp and rounded obstacles (obs_3 is filleted).
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
    # A scenario dominated by two large, heavily rounded obstacles.
    # The path must pass through a diagonal gap ("squeeze") between them.
    env_4 = dict()
    # Large radius (4.5) rounding on obs_1 creates a wide arc
    env_4["obs_1"] = fillet_corner_if_exists(
        Polygon([(0, 9.5), (6, 12.5), (5, 7), (13, 13), (17, 0), (0, 0)]), (13.0, 13.0), 4.5
    )
    # Rounded corner on obs_2 creates the opposing side of the gap
    env_4["obs_2"] = fillet_corner_if_exists(
        Polygon([(0, 12.0), (8, 16.0), (6.5, 11.5), (14, 11.0), (20.0, 0), (20, 20), (0, 20)]), (8.0, 16.0), 1
    )
    start = (1.0, 11.25)
    goal = (18.25, 1.0)
    solution_positions = [start, (6, 13), (6.5, 10), (13.5, 10.5), goal]

    env_dict["4"] = {"env": env_4, "smooth_path": solution_positions}

    return env_dict