"""
repeatability_check.py -- draws the same line N times, logging the pose
before and after each repeat so wobble/positioning error can be measured
(Objectives.md acceptance criterion: "Same 5-10 cm line can be repeated
consistently"). Wobble/error analysis is done on the printed output, not
computed in-script.

Assumes the pen is already gripped (run pen_setup.py first).

Usage:
    python3 scripts/repeatability_check.py --port /dev/ttyTHS1 --n 10 --length 80
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
    parser.add_argument("--n", type=int, default=10, help="number of repeats")
    parser.add_argument("--length", type=float, default=80.0, help="line length in mm")
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    half = args.length / 2.0
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            for i in range(args.n):
                pose_before = robot.get_pose()
                draw_line(robot, (-half, 0.0), (half, 0.0), speed=args.speed)
                pose_after = robot.get_pose()
                print(f"rep {i}: before={pose_before} after={pose_after}")
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
