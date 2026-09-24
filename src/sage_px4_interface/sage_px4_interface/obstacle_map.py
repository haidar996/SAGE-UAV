"""Map-based obstacle avoidance helpers (pure functions, NED x/y).

Obstacles are axis-aligned rectangles (x_min, x_max, y_min, y_max) taken
from a known static map. They are inflated by a safety margin; paths are
planned on the visibility graph of the inflated rectangles' corners.
This avoids KNOWN static obstacles only (no perception of unknown ones).
"""

import heapq
import math

EPS = 1e-6


def parse_rects(flat):
    """[x0,x1,y0,y1, ...] -> list of (x0,x1,y0,y1)."""
    flat = [float(v) for v in flat]
    return [
        (min(flat[i], flat[i + 1]), max(flat[i], flat[i + 1]),
         min(flat[i + 2], flat[i + 3]), max(flat[i + 2], flat[i + 3]))
        for i in range(0, len(flat) - 3, 4)
    ]


def inflate(rects, margin):
    return [(a - margin, b + margin, c - margin, d + margin)
            for a, b, c, d in rects]


def inside(p, rects):
    return any(a < p[0] < b and c < p[1] < d for a, b, c, d in rects)


def segment_hits(p, q, rect):
    """True if segment p-q passes through the OPEN interior of rect."""
    a, b, c, d = rect
    t0, t1 = 0.0, 1.0
    dx, dy = q[0] - p[0], q[1] - p[1]

    for pp, qq in ((-dx, p[0] - a), (dx, b - p[0]),
                   (-dy, p[1] - c), (dy, d - p[1])):
        if abs(pp) < EPS:
            if qq <= EPS:
                return False
        else:
            r = qq / pp
            if pp < 0:
                if r > t1:
                    return False
                t0 = max(t0, r)
            else:
                if r < t0:
                    return False
                t1 = min(t1, r)

    return t1 - t0 > EPS


def blocked(p, q, rects):
    return any(segment_hits(p, q, r) for r in rects)


def project_free(p, rects, step=0.05):
    """Push a point that lies inside a rectangle out through the
    nearest edge."""
    p = (float(p[0]), float(p[1]))

    for _ in range(len(rects) + 2):
        hit = next(
            (r for r in rects if r[0] < p[0] < r[1] and r[2] < p[1] < r[3]),
            None
        )

        if hit is None:
            return p

        a, b, c, d = hit
        options = [
            (p[0] - a, (a - step, p[1])),
            (b - p[0], (b + step, p[1])),
            (p[1] - c, (p[0], c - step)),
            (d - p[1], (p[0], d + step)),
        ]
        p = min(options, key=lambda o: o[0])[1]

    return p


def plan_path(start, goal, rects):
    """Waypoints (excluding start, including goal) from start to goal
    around the inflated rectangles; [goal] if the straight line is
    free; None if no path exists."""
    start = project_free(start, rects)
    goal = project_free(goal, rects)

    if not blocked(start, goal, rects):
        return [goal]

    nodes = [start, goal]

    for a, b, c, d in rects:
        for corner in ((a - 0.1, c - 0.1), (a - 0.1, d + 0.1),
                       (b + 0.1, c - 0.1), (b + 0.1, d + 0.1)):
            if not inside(corner, rects):
                nodes.append(corner)

    dist = {0: 0.0}
    prev = {}
    heap = [(0.0, 0)]
    done = set()

    while heap:
        du, u = heapq.heappop(heap)

        if u in done:
            continue

        done.add(u)

        if u == 1:
            break

        for v in range(len(nodes)):
            if v == u or v in done or blocked(nodes[u], nodes[v], rects):
                continue

            nd = du + math.hypot(
                nodes[u][0] - nodes[v][0], nodes[u][1] - nodes[v][1]
            )

            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))

    if 1 not in prev:
        return None

    path = []
    n = 1

    while n != 0:
        path.append(nodes[n])
        n = prev[n]

    return list(reversed(path))
