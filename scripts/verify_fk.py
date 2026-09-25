#!/usr/bin/env python3
"""Compare firmware and Python forward kinematics without robot motion."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import ArmConnection, config
from drawing_robot.inspection import compare_fk, inspect_robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--position-tolerance-mm", type=float, default=2.0)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with ArmConnection(
        port=args.port,
        baudrate=args.baud,
        mock=args.mock,
        read_only=True,
    ) as connection:
        snapshot = inspect_robot(connection)
    comparison = compare_fk(
        snapshot,
        position_tolerance_mm=args.position_tolerance_mm,
        orientation_tolerance_deg=args.orientation_tolerance_deg,
    )
    print(json.dumps({
        "joint_angles_deg": snapshot.joint_angles_deg,
        "firmware_flange_pose_mm_deg": snapshot.firmware_flange_pose_mm_deg,
        "python_flange_pose_mm_deg": snapshot.python_flange_pose_mm_deg,
        "comparison": comparison.to_dict(),
    }, indent=2))
    if comparison.passed:
        print("FK verification passed at the current static pose.")
        return 0
    print("FK verification failed; do not proceed to motion testing.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
