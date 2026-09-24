#!/usr/bin/env python3
"""
Draws a circle of a fixed center and radius.

    python3 scripts/draw_circle.py --mock
    python3 scripts/draw_circle.py --port /dev/ttyTHS1 --n-points 36
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse

from drawing_robot import Arm, config

HOME = [0.0, 0.0, -90.0, 0.0, 0.0, 0.0]

CENTER = (16.9, -6.5)   # cm, robot-frame -- close to HOME's natural resting xy
RADIUS = 3.0             # cm

SPEED = config.DEFAULT_SPEED_CM_S


def circle_points(center: tuple[float, float], radius: float,
                   n_points: int = 36) -> list[tuple[float, float]]:
    """n_points evenly spaced around the circle. The caller closes the
    loop by revisiting points[0]."""
    cx, cy = center
    return [
        (cx + radius * math.cos(2 * math.pi * i / n_points),
         cy + radius * math.sin(2 * math.pi * i / n_points))
        for i in range(n_points)
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=config.DEFAULT_PORT)
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--n-points", type=int, default=36,
                     help="number of straight-line segments approximating the circle")
    args = ap.parse_args()

    points = circle_points(CENTER, RADIUS, args.n_points)

    with Arm(port=args.port, mock=args.mock) as arm:
        if not arm.conn.is_power_on():
            arm.conn.power_on()

        arm.move_joints(HOME, duration=3.0)

        x0, y0 = points[0]
        if arm.send_coords(x=x0, y=y0, z=config.SAFE_Z, speed=SPEED) != 1:
            print("approach failed:", arm.last_error)
            return
        if arm.send_coords(z=config.DRAWING_PLANE_Z, speed=SPEED) != 1:
            print("pen-down failed:", arm.last_error)
            return

        rest = points[1:] + [points[0]]   # close the loop back to the start
        xs = [p[0] for p in rest]
        ys = [p[1] for p in rest]
        if arm.send_path(x=xs, y=ys, z=config.DRAWING_PLANE_Z, speed=SPEED) == 1:
            print("circle traced")
        else:
            print("trace failed:", arm.last_error)

        if arm.send_coords(z=config.SAFE_Z, speed=SPEED) != 1:
            print("pen-up failed:", arm.last_error)

        arm.move_joints(HOME, duration=3.0)


if __name__ == "__main__":
    main()
