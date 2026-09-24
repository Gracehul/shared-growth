"""
draw_branch.py -- draws a simple Y-branch: a stem from the origin pointing
+Y, splitting into two diverging arms.

Assumes the pen is already gripped (run pen_setup.py first).

Usage:
    python3 scripts/draw_branch.py --port /dev/ttyTHS1 --length 80 --angle 30
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from drawing_robot.robot import Robot
from drawing_robot.shapes import draw_branch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--length", type=float, default=80.0, help="stem length in mm")
    parser.add_argument("--angle", type=float, default=30.0,
                         help="arm split angle in degrees from the stem direction")
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            draw_branch(robot, (0.0, 0.0), (0.0, 1.0), args.length,
                        speed=args.speed, arm_angle_deg=args.angle)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
