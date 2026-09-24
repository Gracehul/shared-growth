"""
draw_line.py -- draws a straight horizontal line centered on the origin.

Assumes the pen is already gripped (run pen_setup.py first).

Usage:
    python3 scripts/draw_line.py --port /dev/ttyTHS1 --length 80
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bao_arm import config
from bao_arm.robot import Robot
from bao_arm.shapes import draw_line


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--length", type=float, default=80.0,
                         help="line length in mm (default: %(default)s, "
                              "within the 5-10cm range from Objectives.md)")
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    half = args.length / 2.0
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            draw_line(robot, (-half, 0.0), (half, 0.0), speed=args.speed)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
