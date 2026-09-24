"""
pen_setup.py -- one-time-per-session pen gripping/releasing.

Run this once after hand-placing (or removing) the pen; the drawing
scripts (draw_point.py, draw_line.py, ...) assume the pen is already
gripped and don't touch the gripper themselves, so it isn't re-gripped
before every primitive test.

Usage:
    python3 scripts/pen_setup.py --port /dev/ttyTHS1            # grab
    python3 scripts/pen_setup.py --port /tmp/ttyMyCobot --release # release
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bao_arm import config
from bao_arm.robot import Robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT,
                         help="serial port the arm is connected to (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD,
                         help="serial baud rate (default: %(default)s)")
    parser.add_argument("--release", action="store_true",
                         help="open the gripper and release the pen instead of grabbing it")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    with Robot(args.port, args.baud) as robot:
        robot.home()
        if args.release:
            robot.release_pen()
            print("Pen released.")
        else:
            robot.init_gripper()
            input("Place pen in gripper, then press Enter...")
            robot.grab_pen()
            print("Pen grabbed. Current pose:", robot.get_pose())
