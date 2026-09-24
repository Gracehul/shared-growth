"""
square_drawing.py -- traces a square on paper lying flat in front of the
myCobot280, using the drawing_robot library.

The paper is horizontal, so the drawing plane is a fixed Z height; the pen
moves in X/Y at that height, and lifts to a safe Z between strokes.

Assumes the pen is already gripped (run pen_setup.py first).

Usage:
    python3 scripts/square_drawing.py --port /dev/ttyTHS1
    python3 scripts/square_drawing.py --port /tmp/ttyMyCobot
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from drawing_robot.robot import Robot
from drawing_robot.shapes import draw_curve


def square(t: float, size: float = 60.0) -> tuple[float, float]:
    """A square of side `size`, centered on the origin.
    t in [0, 1] sweeps once around the perimeter, starting at the
    top-right corner and closing back to it.
    """
    h = size / 2
    corners = [(h, h), (-h, h), (-h, -h), (h, -h), (h, h)]
    seg = t * 4
    i = min(int(seg), 3)
    frac = seg - i
    x0, y0 = corners[i]
    x1, y1 = corners[i + 1]
    return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT,
                         help="serial port the arm is connected to (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD,
                         help="serial baud rate (default: %(default)s)")
    parser.add_argument("--size", type=float, default=60.0, help="square side length in mm")
    parser.add_argument("--speed", type=int, default=config.DEFAULT_SPEED)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    corners = [square(t, size=args.size) for t in (0.0, 0.25, 0.5, 0.75, 1.0)]
    with Robot(args.port, args.baud) as robot:
        try:
            robot.home()
            draw_curve(robot, corners, speed=args.speed)
            robot.home()
        except (KeyboardInterrupt, Exception):
            robot.stop()
            raise
