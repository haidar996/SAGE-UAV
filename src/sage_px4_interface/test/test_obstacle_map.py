import math

from sage_px4_interface.obstacle_map import (
    blocked, inflate, inside, parse_rects, plan_path, project_free)

WALL = [(-1.0, 1.0, -5.0, 5.0)]   # wall across x in [-1,1], y in [-5,5]


def length(pts):
    return sum(math.hypot(a[0] - b[0], a[1] - b[1])
               for a, b in zip(pts, pts[1:]))


def test_parse_and_inflate():
    r = parse_rects([0, 2, 0, 3])
    assert r == [(0.0, 2.0, 0.0, 3.0)]
    assert inflate(r, 1.0) == [(-1.0, 3.0, -1.0, 4.0)]


def test_free_line_is_direct():
    assert plan_path((5, 0), (10, 0), WALL) == [(10.0, 0.0)]


def test_detour_around_wall_is_collision_free():
    start, goal = (-4.0, 0.0), (4.0, 0.0)
    path = plan_path(start, goal, WALL)
    assert path is not None and len(path) >= 2
    pts = [start] + path
    assert not any(blocked(a, b, WALL) for a, b in zip(pts, pts[1:]))
    assert length(pts) < 8.0 + 2 * 5.5   # not absurdly long


def test_touching_edge_is_not_blocked():
    assert not blocked((-3, 1.0), (3, 1.0), [(-1.0, 1.0, -5.0, 1.0)])


def test_project_free_pushes_out():
    p = project_free((0.9, 0.0), WALL)
    assert not inside(p, WALL)
    assert abs(p[0] - 1.0) < 0.2


def test_goal_inside_obstacle_is_projected():
    path = plan_path((-4, 0), (0, 0), WALL)
    assert path and not inside(path[-1], WALL)
