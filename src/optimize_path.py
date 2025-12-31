import numpy as np


class OptimizeFlyby:
    """
    Slides-like rounding with INWARD-first and OUTWARD fallback.

    Key points:
    - Virtual points A (before corner) and B (after corner) depend on r and the shortest segment.
    - A->B is replaced by a parabolic rounding curve (quadratic Bezier).
      * INWARD: control point P = corner p1  (this matches your "good old" look)
      * OUTWARD: mirrored control point P_out = A + B - p1 (same curve model, other side)
    - If collision: reduce r stepwise until collision-free.
    - IMPORTANT FIX: For consecutive corners we use the *actual* last point of the already
      optimized path as incoming point p0, not the original prev waypoint. This prevents spikes.
    """

    @staticmethod
    def _as_np(p):
        return np.array(p, dtype=float)

    @staticmethod
    def _unique_name(G, base):
        if base not in G:
            return base
        k = 1
        while f"{base}_{k}" in G:
            k += 1
        return f"{base}_{k}"

    @staticmethod
    def _bezier_quad(A, P, B, t: float):
        """Quadratic Bezier curve point."""
        u = 1.0 - t
        return (u * u) * A + 2.0 * u * t * P + (t * t) * B

    @staticmethod
    def _polyline_collision_free(cc, pts):
        """Collision test for a discretized curve (polyline)."""
        for i in range(len(pts) - 1):
            if cc.lineInCollision(pts[i], pts[i + 1]):
                return False
        return True

    @staticmethod
    def optimizePath(path=None, planner=None, config=None):
        """
        Args:
            path: list[str] node names that exist in planner.graph (e.g. ["start","1",...,"goal"])
            planner: has .graph (networkx) and ._collisionChecker with lineInCollision(p,q)
            config:
                r: initial rounding factor, e.g. 0.30 (range in task: ~0.02..0.49)
                r_step: decrement for r on collision, e.g. 0.02
                r_min: minimum r, e.g. 0.02
                n_samples: samples for curve discretization, e.g. 50..150

        Returns:
            optimized_path: list[str] including created virtual curve nodes
            r_used: dict[original_corner_node -> used r] (0 if no rounding)
            side_used: dict[original_corner_node -> "in"|"out"|"none"]
        """
        if path is None:
            path = []
        if config is None:
            config = {}
        if planner is None or not hasattr(planner, "graph"):
            raise ValueError("planner missing or has no graph")

        # Always return 3 values
        if len(path) < 3:
            return path, {n: 0.0 for n in path}, {n: "none" for n in path}

        G = planner.graph
        cc = planner._collisionChecker

        r_init = float(config.get("r", 0.30))
        r_step = float(config.get("r_step", 0.02))
        r_min = float(config.get("r_min", 0.02))
        n_samples = int(config.get("n_samples", 80))

        def node_pos(name: str):
            return OptimizeFlyby._as_np(G.nodes[name]["pos"])

        optimized = [path[0]]
        r_used = {path[0]: 0.0}
        side_used = {path[0]: "none"}

        # iterate over original corners (exclude start/goal)
        for i in range(1, len(path) - 1):
            curr_n = path[i]
            next_n = path[i + 1]

            # IMPORTANT: incoming point is the *current end* of optimized path
            p0 = node_pos(optimized[-1])   # real incoming point
            p1 = node_pos(curr_n)          # corner (original)
            p2 = node_pos(next_n)          # next original waypoint

            v_in = p1 - p0
            v_out = p2 - p1
            d_in = float(np.linalg.norm(v_in))
            d_out = float(np.linalg.norm(v_out))

            # If degenerate, keep the corner
            if d_in < 1e-9 or d_out < 1e-9:
                if optimized[-1] != curr_n:
                    G.add_edge(optimized[-1], curr_n)
                    optimized.append(curr_n)
                r_used[curr_n] = 0.0
                side_used[curr_n] = "none"
                continue

            u_in = v_in / d_in
            u_out = v_out / d_out

            best = None  # (r_ok, side_ok, curve_pts)
            r = r_init

            while r >= r_min:
                # shortest-segment rule (+ keep A/B inside segments)
                offset = r * min(d_in, d_out)
                offset = min(offset, 0.49 * d_in, 0.49 * d_out)
                if offset < 1e-6:
                    break

                # Virtual points around the corner
                A = p1 - u_in * offset
                B = p1 + u_out * offset

                # INWARD curve exactly like your "good old" model:
                P_in = p1
                # OUTWARD curve: mirrored control point (same curve family, other side)
                P_out = A + B - p1

                # Try inward first, outward only if inward fails
                for side, P in (("in", P_in), ("out", P_out)):
                    curve_pts = [
                        OptimizeFlyby._bezier_quad(A, P, B, k / (n_samples - 1))
                        for k in range(n_samples)
                    ]

                    # Collision checks: incoming line, curve, outgoing line
                    if (not cc.lineInCollision(p0, A)
                            and OptimizeFlyby._polyline_collision_free(cc, curve_pts)
                            and not cc.lineInCollision(B, p2)):
                        best = (float(r), side, curve_pts)
                        break

                if best is not None:
                    break

                r -= r_step

            if best is None:
                # No rounding possible -> keep the corner as is
                if optimized[-1] != curr_n:
                    G.add_edge(optimized[-1], curr_n)
                    optimized.append(curr_n)
                r_used[curr_n] = 0.0
                side_used[curr_n] = "none"
                continue

            r_ok, side_ok, curve_pts_ok = best
            r_used[curr_n] = r_ok
            side_used[curr_n] = side_ok

            # Add curve sample points as virtual nodes and connect them
            last = optimized[-1]
            prev_added = None

            for idx, pt in enumerate(curve_pts_ok):
                new_name = OptimizeFlyby._unique_name(G, f"{curr_n}_{side_ok}_rb_{idx}")
                G.add_node(new_name, pos=pt, color="orange")

                if idx == 0:
                    G.add_edge(last, new_name)
                else:
                    G.add_edge(prev_added, new_name)

                prev_added = new_name
                optimized.append(new_name)

            # NOTE: We do not append curr_n itself (the corner stop is replaced by rounding)

        # Connect to goal (original last node)
        goal = path[-1]
        if optimized[-1] != goal:
            G.add_edge(optimized[-1], goal)
            optimized.append(goal)

        r_used[goal] = 0.0
        side_used[goal] = "none"

        return optimized, r_used, side_used
