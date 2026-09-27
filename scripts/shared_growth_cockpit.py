#!/usr/bin/env python3
"""Open the read-only Shared Growth desktop cockpit."""

from __future__ import annotations

import argparse
import json

from drawing_robot import config
from drawing_robot.cockpit import (
    CameraSource,
    CockpitModel,
    ExecutionLogSource,
    RobotStateSource,
    SOURCE_MAPPING,
    TrajectorySource,
    Workspace,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("off", "mock", "read-only"), default="off")
    parser.add_argument("--robot-port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--camera", default="off", help="off, mock, auto or a camera device")
    parser.add_argument("--trajectory", help="optional Stage-2B JSON bundle")
    parser.add_argument("--execution-log", help="optional Stage-3/4A ExecutionLog JSON")
    parser.add_argument("--refresh-ms", type=int, default=200)
    parser.add_argument(
        "--workspace", choices=tuple(item.value.lower() for item in Workspace),
        default="run",
    )
    parser.add_argument("--screenshot", help="capture the shell after startup and exit")
    parser.add_argument("--check", action="store_true", help="print source mapping without opening a GUI")
    return parser.parse_args()


def build_model(args) -> CockpitModel:
    camera = CameraSource(
        device="off" if args.camera == "mock" else args.camera,
        mock=args.camera == "mock",
    )
    if args.robot == "mock":
        robot = RobotStateSource.mock()
    elif args.robot == "read-only":
        robot = RobotStateSource.read_only_hardware(args.robot_port, args.baud)
    else:
        robot = RobotStateSource()
    model = CockpitModel(
        camera,
        robot,
        TrajectorySource(args.trajectory),
        ExecutionLogSource(args.execution_log),
    )
    camera.start()
    return model


def main() -> int:
    args = parse_args()
    if args.check:
        print(json.dumps([
            {"ui_element": ui, "source_module": module, "source_data": data}
            for ui, module, data in SOURCE_MAPPING
        ], indent=2))
        return 0
    model = build_model(args)
    try:
        from drawing_robot.cockpit.app import CockpitApp
        CockpitApp(
            model,
            args.refresh_ms,
            initial_workspace=Workspace(args.workspace.upper()),
            screenshot_path=args.screenshot,
        ).run()
    finally:
        model.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
