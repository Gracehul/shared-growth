"""
calibrate.py -- interactive helper for measuring the placeholder constants
in drawing_robot/config.py (ORIGIN_X/Y, DRAWING_PLANE, PEN_RX/RY/RZ, HOME_ANGLES).

Connects, relaxes the servos (teaching mode) so the arm can be moved by
hand, then repeatedly prints the current pose and joint angles on demand
so you can hand-position the pen tip and read off real numbers to paste
into config.py.

Usage:
    python3 scripts/calibrate.py --port /dev/ttyTHS1
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from drawing_robot.robot import Robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUD)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    with Robot(args.port, args.baud) as robot:
        robot.mc.release_all_servos()
        print("Servos relaxed -- you can now move the arm by hand.")
        print("Position the pen tip where you want to measure it, then "
              "press Enter to read it. Ctrl-C to quit.\n")
        try:
            while True:
                input("Ready> ")
                x, y, z, rx, ry, rz = robot.get_pose()
                angles = robot.get_angles()
                print(f"  pose:   x={x:.2f} y={y:.2f} z={z:.2f}  "
                      f"rx={rx:.2f} ry={ry:.2f} rz={rz:.2f}")
                print("          (x,y -> ORIGIN_X/ORIGIN_Y; z -> DRAWING_PLANE "
                      "or SAFE_Z; rx,ry,rz -> PEN_RX/PEN_RY/PEN_RZ)")
                print(f"  angles: J1={angles[0]:.2f} J2={angles[1]:.2f} "
                      f"J3={angles[2]:.2f} J4={angles[3]:.2f} "
                      f"J5={angles[4]:.2f} J6={angles[5]:.2f}")
                print("          (-> HOME_ANGLES, if this is the desired safe fold)\n")
        except KeyboardInterrupt:
            print("\nDone. The arm stays relaxed until the next connect "
                  "(which powers it back on).")
