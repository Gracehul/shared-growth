"""
draw_circle.py -- draws a circle of a fixed center and radius.

Assumes the pen is already gripped (run pen_setup.py first). Edit CENTER/
RADIUS below to change the shape.

Usage:
    python3 scripts/draw_circle.py --port /dev/ttyTHS1 --n-points 36
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from drawing_robot.robot import Robot
from drawing_robot.shapes import Point, circle_points, draw_curve

CENTER: Point = (0.0, 0.0)   # mm, ORIGIN-relative
RADIUS = 40.0                 # mm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    parser.add_argument("--n-points", type=int, default=36,
                         help="number of straight-line segments approximating the circle")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    points = circle_points(CENTER, RADIUS, args.n_points)
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            draw_curve(robot, points, speed=args.speed)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
