"""
draw_square.py -- draws a square (or any closed quadrilateral) from a
fixed set of corner points.

Assumes the pen is already gripped (run pen_setup.py first). Edit CORNERS
below to change the shape.

Usage:
    python3 scripts/draw_square.py --port /dev/ttyTHS1
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from drawing_robot.robot import Robot
from drawing_robot.shapes import Point, draw_curve

CORNERS: list[Point] = [
    (-20.0, -20.0),
    (20.0, -20.0),
    (20.0, 20.0),
    (-20.0, 20.0),
]  # mm, ORIGIN-relative -- edit these to change the square


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            draw_curve(robot, CORNERS + [CORNERS[0]], speed=args.speed)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
