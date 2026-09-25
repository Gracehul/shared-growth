#!/usr/bin/env python3
"""Read myCobot state without configuring or moving the robot."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import ArmConnection, config
from drawing_robot.inspection import inspect_robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--model", default="myCobot 280 JN")
    parser.add_argument("--mock", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with ArmConnection(
        port=args.port,
        baudrate=args.baud,
        mock=args.mock,
        read_only=True,
    ) as connection:
        snapshot = inspect_robot(connection, expected_model=args.model)
    print(json.dumps(snapshot.to_dict(), indent=2))
    print("Read-only inspection complete; no control command was issued.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
