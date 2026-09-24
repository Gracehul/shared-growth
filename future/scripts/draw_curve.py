"""
draw_curve.py -- draws a sine-wave curve centered on the origin.

Assumes the pen is already gripped (run pen_setup.py first).

Usage:
    python3 scripts/draw_curve.py --port /dev/ttyTHS1 --length 80 --amplitude 15
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bao_arm import config
from bao_arm.robot import Robot
from bao_arm.shapes import draw_curve


def sine_points(length: float, amplitude: float, n_points: int) -> list[tuple[float, float]]:
    half = length / 2.0
    return [
        (-half + length * i / (n_points - 1),
         amplitude * math.sin(2 * math.pi * i / (n_points - 1)))
        for i in range(n_points)
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    parser.add_argument("--length", type=float, default=80.0, help="curve span in mm")
    parser.add_argument("--amplitude", type=float, default=15.0, help="peak offset in mm")
    parser.add_argument("--n-points", type=int, default=20, help="number of sampled points")
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    points = sine_points(args.length, args.amplitude, args.n_points)
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            draw_curve(robot, points, speed=args.speed)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
