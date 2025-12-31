import numpy as np


class OptimizeFlyby:
    """Optimize path corners using *inverse rounding* (Slides model).

    - For every corner p0 -> p1 -> p2 we create two virtual points A and B around the corner.
      Distance to p1 uses the **shortest-segment rule**:
         offset = r * min(|p1-p0|, |p2-p1|)

    - We replace the corner section by a **quadratic curve that passes through the original corner p1**:
         C(0)   = A
         C(0.5) = p1   (IMPORTANT: must pass the original corner)
         C(1)   = B

      This is NOT a quadratic Bezier (which would typically *not* pass through p1).

    - "Inverse rounding": find the **largest r** that is collision-free.
      We do this via **bisection** between r_min and r_max.

    Expected planner interface:
    - planner.graph: networkx graph with node attribute "pos" (2D np.array-like)
    - planner._collisionChecker.lineInCollision(p, q): bool

    Config keys:
    - r_max: upper bound for rounding factor (default 0.49)
    - r_min: lower bound (default 0.02)
    - max_iter: bisection iterations (default 15)
    - n_samples: number of curve samples for collision test / drawing (default 81)
    """

    # ------------------------- small helpers -------------------------
    @staticmethod
    def _as_np(p):
        return np.asarray(p, dtype=float)

    @staticmethod
    def _unique_name(G, base: str) -> str:
        if base not in G:
            return base
        k = 1
        while f"{base}_{k}" in G:
            k += 1
        return f"{base}_{k}"

    @staticmethod
    def _polyline_collision_free(cc, pts) -> bool:
        for i in range(len(pts) - 1):
            if cc.lineInCollision(pts[i], pts[i + 1]):
                return False
        return True

    # ------------------------- slide-like parabola -------------------------
    @staticmethod
    def _parabola_through_A_p1_B(A, p1, B, n_samples: int):
        """Quadratic curve through 3 points: C(0)=A, C(0.5)=p1, C(1)=B.

        Lagrange interpolation with parameter t in [0,1] and fixed middle knot t=0.5.
        Guarantees the curve passes through the original corner point p1.
        """
        A = OptimizeFlyby._as_np(A)
        p1 = OptimizeFlyby._as_np(p1)
        B = OptimizeFlyby._as_np(B)

        ts = np.linspace(0.0, 1.0, max(3, int(n_samples)))
        pts = np.empty((len(ts), A.shape[0]), dtype=float)
        for i, t in enumerate(ts):
            # Lagrange basis for t0=0, t1=0.5, t2=1
            L0 = 2.0 * (t - 0.5) * (t - 1.0)
            L1 = -4.0 * t * (t - 1.0)
            L2 = 2.0 * t * (t - 0.5)
            pts[i] = L0 * A + L1 * p1 + L2 * B
        return pts

    # ------------------------- main API -------------------------
    @staticmethod
    def optimizePath(path=None, planner=None, config=None):
        """Optimize a waypoint path.

        Args:
            path: list[str] of node names existing in planner.graph (e.g. ["start","1",...,"goal"])
            planner: planner with .graph and ._collisionChecker.lineInCollision(p,q)
            config: dict (see class docstring)

        Returns:
            optimized_path: list[str] including created virtual nodes
            r_used: dict[node_name -> used r] (0 if not rounded)
            side_used: dict[node_name -> "inv"|"none"] (kept for compatibility with main.ipynb)
        """
        if path is None:
            path = []
        if config is None:
            config = {}
        if planner is None or not hasattr(planner, "graph"):
            raise ValueError("planner missing or has no graph")

        if len(path) < 3:
            return path, {n: 0.0 for n in path}, {n: "none" for n in path}

        G = planner.graph
        cc = planner._collisionChecker

        r_max = float(config.get("r_max", 0.49))
        r_min = float(config.get("r_min", 0.02))
        max_iter = int(config.get("max_iter", 15))
        n_samples = int(config.get("n_samples", 81))

        # guardrails
        r_max = min(max(r_max, 0.0), 0.49)
        r_min = min(max(r_min, 0.0), r_max)
        n_samples = max(11, n_samples)

        def node_pos(name: str):
            return OptimizeFlyby._as_np(G.nodes[name]["pos"])

        optimized = [path[0]]
        r_used = {path[0]: 0.0}
        side_used = {path[0]: "none"}

        # Iterate over original corners (exclude start/goal)
        for i in range(1, len(path) - 1):
            curr_n = path[i]
            next_n = path[i + 1]

            # incoming point is the *actual end* of already optimized path
            p0 = node_pos(optimized[-1])
            p1 = node_pos(curr_n)
            p2 = node_pos(next_n)

            v_in = p1 - p0
            v_out = p2 - p1
            d_in = float(np.linalg.norm(v_in))
            d_out = float(np.linalg.norm(v_out))

            # Degenerate segments -> keep corner
            if d_in < 1e-9 or d_out < 1e-9:
                if optimized[-1] != curr_n:
                    G.add_edge(optimized[-1], curr_n)
                    optimized.append(curr_n)
                r_used[curr_n] = 0.0
                side_used[curr_n] = "none"
                continue

            u_in = v_in / d_in
            u_out = v_out / d_out

            # -------------------- inverse rounding: maximize r --------------------
            def is_r_ok(r: float):
                # shortest-segment rule (+ keep A/B inside segments)
                offset = r * min(d_in, d_out)
                offset = min(offset, 0.49 * d_in, 0.49 * d_out)
                if offset < 1e-9:
                    return False, None, None, None

                A = p1 - u_in * offset
                B = p1 + u_out * offset

                curve_pts = OptimizeFlyby._parabola_through_A_p1_B(A, p1, B, n_samples=n_samples)

                ok = (not cc.lineInCollision(p0, A)
                      and OptimizeFlyby._polyline_collision_free(cc, curve_pts)
                      and not cc.lineInCollision(B, p2))
                return ok, A, B, curve_pts

            # First check if anything is possible at all
            ok_min, _, _, _ = is_r_ok(r_min)
            if not ok_min:
                # No rounding possible -> keep corner as is
                if optimized[-1] != curr_n:
                    G.add_edge(optimized[-1], curr_n)
                    optimized.append(curr_n)
                r_used[curr_n] = 0.0
                side_used[curr_n] = "none"
                continue

            lo, hi = r_min, r_max
            ok_lo, A_lo, B_lo, curve_lo = is_r_ok(lo)
            best = (lo, A_lo, B_lo, curve_lo)

            ok_hi, A_hi, B_hi, curve_hi = is_r_ok(hi)
            if ok_hi:
                best = (hi, A_hi, B_hi, curve_hi)
            else:
                for _ in range(max_iter):
                    mid = 0.5 * (lo + hi)
                    ok_mid, A_mid, B_mid, curve_mid = is_r_ok(mid)
                    if ok_mid:
                        best = (mid, A_mid, B_mid, curve_mid)
                        lo = mid
                    else:
                        hi = mid

            r_ok, A_ok, B_ok, curve_pts_ok = best
            r_used[curr_n] = float(r_ok)
            side_used[curr_n] = "inv"

            # -------------------- add nodes for the curve --------------------
            last = optimized[-1]
            prev_added = None

            # index of sample closest to t=0.5 -> this should equal p1
            ts = np.linspace(0.0, 1.0, len(curve_pts_ok))
            mid_idx = int(np.argmin(np.abs(ts - 0.5)))

            for idx, pt in enumerate(curve_pts_ok):
                if idx == mid_idx:
                    node_name = curr_n  # re-use original corner node
                    G.nodes[curr_n]["pos"] = OptimizeFlyby._as_np(pt)  # numerical safety
                else:
                    node_name = OptimizeFlyby._unique_name(G, f"{curr_n}_inv_{idx}")
                    G.add_node(node_name, pos=OptimizeFlyby._as_np(pt), color="orange")

                if idx == 0:
                    G.add_edge(last, node_name)
                else:
                    G.add_edge(prev_added, node_name)

                prev_added = node_name
                optimized.append(node_name)

        # Connect to goal
        goal = path[-1]
        if optimized[-1] != goal:
            G.add_edge(optimized[-1], goal)
            optimized.append(goal)

        r_used[goal] = 0.0
        side_used[goal] = "none"

        return optimized, r_used, side_used
