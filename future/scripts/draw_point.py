"""
draw_point.py -- draws a single dwell dot on the paper.

Assumes the pen is already gripped (run pen_setup.py first).

Usage:
    python3 scripts/draw_point.py --port /dev/ttyTHS1 --x 0 --y 0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bao_arm import config
from bao_arm.robot import Robot
from bao_arm.shapes import draw_point


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--x", type=float, default=0.0, help="mm offset from ORIGIN_X")
    parser.add_argument("--y", type=float, default=0.0, help="mm offset from ORIGIN_Y")
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            print(robot.get_pose())
            draw_point(robot, (args.x, args.y), speed=args.speed)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
