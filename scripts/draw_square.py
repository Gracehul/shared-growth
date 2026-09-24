#!/usr/bin/env python3
"""
Draws a square from a fixed list of corner points.

    python3 scripts/draw_square.py --mock
    python3 scripts/draw_square.py --port /dev/ttyTHS1
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse

from bao_arm import Arm, config

HOME = [0.0, 0.0, -90.0, 0.0, 0.0, 0.0]

CORNERS = [
    (14.0, -4.0),
    (22.0, -4.0),
    (22.0, 4.0),
    (14.0, 4.0),
]  # cm, robot-frame -- placeholder, edit these to change the square

SPEED = config.DEFAULT_SPEED_CM_S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=config.DEFAULT_PORT)
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()

    with Arm(port=args.port, mock=args.mock) as arm:
        if not arm.conn.is_power_on():
            arm.conn.power_on()

        arm.move_joints(HOME, duration=3.0)

        x0, y0 = CORNERS[0]
        if arm.send_coords(x=x0, y=y0, z=config.SAFE_Z, speed=SPEED) != 1:
            print("approach failed:", arm.last_error)
            return
        if arm.send_coords(z=config.DRAWING_PLANE_Z, speed=SPEED) != 1:
            print("pen-down failed:", arm.last_error)
            return

        rest = CORNERS[1:] + [CORNERS[0]]   # close the loop back to the start
        xs = [p[0] for p in rest]
        ys = [p[1] for p in rest]
        if arm.send_path(x=xs, y=ys, z=config.DRAWING_PLANE_Z, speed=SPEED) == 1:
            print("square traced")
        else:
            print("trace failed:", arm.last_error)

        if arm.send_coords(z=config.SAFE_Z, speed=SPEED) != 1:
            print("pen-up failed:", arm.last_error)

        arm.move_joints(HOME, duration=3.0)


if __name__ == "__main__":
    main()
