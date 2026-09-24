"""
Drawing primitives from Objectives.md: point, straight line, curve,
Y-branch. Geometry (branch_points, _normalize, _rotate) is pure and
hardware-independent; the draw_* functions compose it with Robot.
"""

from __future__ import annotations

import math
import time

from . import config
from .robot import Robot

Point = tuple[float, float]


# ---------- pure geometry ----------

def _normalize(v: tuple[float, float]) -> tuple[float, float]:
    mag = math.hypot(*v)
    if mag == 0:
        raise ValueError("direction must be nonzero")
    return v[0] / mag, v[1] / mag


def _rotate(v: tuple[float, float], deg: float) -> tuple[float, float]:
    r = math.radians(deg)
    x, y = v
    return (x * math.cos(r) - y * math.sin(r),
            x * math.sin(r) + y * math.cos(r))


def branch_points(origin: Point, direction: tuple[float, float], length: float,
                   split_frac: float = 0.6, arm_length: float | None = None,
                   arm_angle_deg: float = 30.0) -> tuple[Point, Point, Point, Point]:
    """The 4 key points of a Y-branch: stem_start, split_point, arm1_end,
    arm2_end -- exactly the 3 line segments draw_branch() needs. `origin`
    and the returned points are ORIGIN-relative mm offsets, same frame as
    Robot.move_to()."""
    dx, dy = _normalize(direction)
    ox, oy = origin
    split = (ox + dx * length * split_frac, oy + dy * length * split_frac)
    if arm_length is None:
        arm_length = length * (1 - split_frac) * 1.2
    left_dir = _rotate((dx, dy), arm_angle_deg)
    right_dir = _rotate((dx, dy), -arm_angle_deg)
    arm1 = (split[0] + left_dir[0] * arm_length, split[1] + left_dir[1] * arm_length)
    arm2 = (split[0] + right_dir[0] * arm_length, split[1] + right_dir[1] * arm_length)
    return origin, split, arm1, arm2


def circle_points(center: Point, radius: float, n_points: int = 36) -> list[Point]:
    """n_points+1 points sampled evenly around the circle, starting and
    ending at the same point so draw_curve()'s straight-line hops close
    the loop. `center` and the returned points are ORIGIN-relative mm
    offsets, same frame as Robot.move_to()."""
    cx, cy = center
    ring = [
        (cx + radius * math.cos(2 * math.pi * i / n_points),
         cy + radius * math.sin(2 * math.pi * i / n_points))
        for i in range(n_points)
    ]
    return ring + [ring[0]]


# ---------- hardware-calling drawing functions ----------

def _trace(robot: Robot, points: list[Point], speed: int = config.DEFAULT_SPEED) -> None:
    """Pen up above points[0], pen down, straight-line hop through each
    remaining point at draw height, pen up at the end."""
    x0, y0 = points[0]
    robot.move_to(x0, y0, config.SAFE_Z, speed)
    robot.move_to(x0, y0, config.DRAWING_PLANE, speed)
    for x, y in points[1:]:
        robot.move_to(x, y, config.DRAWING_PLANE, speed)
    x_last, y_last = points[-1]
    robot.move_to(x_last, y_last, config.SAFE_Z, speed)


def draw_point(robot: Robot, at: Point, speed: int = config.DEFAULT_SPEED,
                dwell: float = config.POINT_DWELL_S) -> None:
    """A dwell dot: touch down, pause so the pen marks the paper, lift."""
    x, y = at
    robot.move_to(x, y, config.SAFE_Z, speed)
    robot.move_to(x, y, config.DRAWING_PLANE, speed)
    time.sleep(dwell)
    robot.move_to(x, y, config.SAFE_Z, speed)


def draw_line(robot: Robot, start: Point, end: Point,
              speed: int = config.DEFAULT_SPEED) -> None:
    _trace(robot, [start, end], speed)


def draw_curve(robot: Robot, points: list[Point],
               speed: int = config.DEFAULT_SPEED) -> None:
    """points: a caller-supplied list of >=2 (x, y) ORIGIN-relative mm
    offsets (e.g. sampled from a sine wave or arc). No smoothing/blending
    in M1 -- each point is a straight-line hop from the last."""
    if len(points) < 2:
        raise ValueError("draw_curve needs at least 2 points")
    _trace(robot, points, speed)


def draw_branch(robot: Robot, origin: Point, direction: tuple[float, float],
                 length: float, speed: int = config.DEFAULT_SPEED,
                 split_frac: float = 0.6, arm_angle_deg: float = 30.0) -> None:
    """Y-branch = stem + two diverging arms, drawn as 3 separate strokes
    (pen lifts between each) sharing the same split point."""
    stem_start, split, arm1, arm2 = branch_points(
        origin, direction, length, split_frac, arm_angle_deg=arm_angle_deg)
    draw_line(robot, stem_start, split, speed)
    draw_line(robot, split, arm1, speed)
    draw_line(robot, split, arm2, speed)


if __name__ == "__main__":
    # Pure-geometry self-check -- no hardware needed.
    origin, split, arm1, arm2 = branch_points((0.0, 0.0), (0.0, 1.0), 100.0,
                                               split_frac=0.6, arm_angle_deg=30.0)
    assert origin == (0.0, 0.0)
    assert math.isclose(split[0], 0.0, abs_tol=1e-9) and math.isclose(split[1], 60.0)

    # split lies on the stem direction from origin
    stem_dir = _normalize((split[0] - origin[0], split[1] - origin[1]))
    assert math.isclose(stem_dir[0], 0.0, abs_tol=1e-9) and math.isclose(stem_dir[1], 1.0)

    # arms are symmetric about the stem axis: equal distance from split,
    # mirrored x, equal y
    arm_len_1 = math.hypot(arm1[0] - split[0], arm1[1] - split[1])
    arm_len_2 = math.hypot(arm2[0] - split[0], arm2[1] - split[1])
    assert math.isclose(arm_len_1, arm_len_2)
    assert math.isclose(arm1[0] - split[0], -(arm2[0] - split[0]), abs_tol=1e-9)
    assert math.isclose(arm1[1] - split[1], arm2[1] - split[1], abs_tol=1e-9)

    print("branch_points geometry OK:", origin, split, arm1, arm2)

    # circle_points self-check
    pts = circle_points((10.0, -5.0), 25.0, n_points=36)
    assert len(pts) == 37
    assert pts[0] == pts[-1]
    for px, py in pts:
        assert math.isclose(math.hypot(px - 10.0, py - (-5.0)), 25.0, abs_tol=1e-9)

    print("circle_points geometry OK:", len(pts), "points, closed loop")
