#!/usr/bin/env python3
"""Sample firmware and Python FK at five nearby supervised hardware poses."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from characterize_joint import (
    GateFailure,
    read_angles,
    read_error,
    require_safe_temperatures,
    run_trial,
)
from drawing_robot import config
from drawing_robot.kinematics import flange_pose_coords, wrap180
from drawing_robot.robot import RobotService


def read_coords(robot, retries: int = 4) -> tuple[list[float], int]:
    last_value: object = None
    for attempt in range(1, retries + 1):
        last_value = robot.get_coords()
        if (
            isinstance(last_value, (list, tuple))
            and len(last_value) == 6
            and all(isinstance(value, (int, float)) for value in last_value)
        ):
            return [float(value) for value in last_value], attempt
        time.sleep(0.05)
    raise GateFailure(f"invalid firmware pose after {retries} attempts: {last_value!r}")


def sample_pose(robot, name: str) -> dict[str, object]:
    time.sleep(0.2)
    angles, angle_attempts = read_angles(robot)
    error, error_attempts = read_error(robot)
    firmware, coords_attempts = read_coords(robot)
    if error != 0:
        raise GateFailure(f"controller error at {name}: {error}")
    python_pose = [float(value) for value in flange_pose_coords(angles)]
    position_error = [python_pose[i] - firmware[i] for i in range(3)]
    orientation_error = [
        float(value)
        for value in wrap180(
            [python_pose[i] - firmware[i] for i in range(3, 6)]
        )
    ]
    return {
        "name": name,
        "joint_angles_deg": angles,
        "firmware_flange_pose_mm_deg": firmware,
        "python_flange_pose_mm_deg": python_pose,
        "position_error_xyz_mm": position_error,
        "position_error_norm_mm": math.sqrt(sum(value * value for value in position_error)),
        "orientation_error_rpy_deg": orientation_error,
        "orientation_error_max_deg": max(abs(value) for value in orientation_error),
        "read_attempts": {
            "angles": angle_attempts,
            "error": error_attempts,
            "coords": coords_attempts,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--speed", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-servo-temp-c", type=float, default=config.TEMPERATURE_ABORT_C)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.speed < 1 or args.speed > 5:
        raise SystemExit("--speed must be 1..5")
    if not args.execute:
        print(json.dumps({
            "dry_run": True,
            "poses": ["baseline", "j6_plus_2", "j6_minus_2", "j4_plus_2", "j3_plus_2"],
            "speed": args.speed,
        }, indent=2))
        print("No robot connection opened. Pass --execute only during a supervised test.")
        return 0

    report: dict[str, object] = {
        "schema": "shared-growth/fk-pose-samples/v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "speed": args.speed,
        "poses": [],
        "moves": [],
        "result": "running",
    }
    robot = RobotService(args.port, args.baud, telemetry=False)
    exit_code = 1
    completed_plan = False
    baseline: list[float] | None = None
    try:
        baseline, _ = read_angles(robot)
        error, _ = read_error(robot)
        if error != 0:
            raise GateFailure(f"initial controller error must be 0, got {error}")
        if robot.is_all_servo_enable() != 1:
            raise GateFailure("all servos must be enabled")
        report["initial_servo_temps_c"] = require_safe_temperatures(
            robot, args.max_servo_temp_c
        )
        robot.arm_motion(locally_confirmed=True)
        for index, (low, high) in enumerate(config.JOINT_LIMITS_DEG):
            if not low + config.JOINT_LIMIT_MARGIN_DEG <= baseline[index] <= high - config.JOINT_LIMIT_MARGIN_DEG:
                raise GateFailure(f"J{index + 1} lacks configured limit margin")

        report["baseline_angles_deg"] = baseline
        report["poses"].append(sample_pose(robot, "baseline"))

        def move(joint: int, target: float) -> None:
            require_safe_temperatures(robot, args.max_servo_temp_c)
            low, high = config.JOINT_LIMITS_DEG[joint - 1]
            margin = config.JOINT_LIMIT_MARGIN_DEG
            if not low + margin <= target <= high - margin:
                raise GateFailure(f"J{joint} target lacks limit margin")
            trial = run_trial(robot, joint, target, args.speed)
            trial["joint"] = joint
            report["moves"].append(trial)

        move(6, baseline[5] + 2.0)
        report["poses"].append(sample_pose(robot, "j6_plus_2"))

        move(6, baseline[5] - 2.0)
        report["poses"].append(sample_pose(robot, "j6_minus_2"))

        move(6, baseline[5])
        move(4, baseline[3] + 2.0)
        report["poses"].append(sample_pose(robot, "j4_plus_2"))

        move(4, baseline[3])
        move(3, baseline[2] + 2.0)
        report["poses"].append(sample_pose(robot, "j3_plus_2"))

        move(3, baseline[2])
        completed_plan = True
        report["result"] = "complete"
        exit_code = 0
    except Exception as exc:
        report["result"] = "aborted"
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            robot.stop()
            report["stop_sent"] = True
        except Exception as exc:
            report["stop_sent"] = False
            report["stop_error"] = f"{type(exc).__name__}: {exc}"
        try:
            final, _ = read_angles(robot)
            report["final_angles_deg"] = final
            report["final_error"], _ = read_error(robot)
            if completed_plan and baseline is not None:
                report["return_error_deg"] = [
                    final[index] - baseline[index] for index in range(config.DOF)
                ]
        except Exception as exc:
            report["final_read_error"] = f"{type(exc).__name__}: {exc}"
        robot.close()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    norms = [pose["position_error_norm_mm"] for pose in report["poses"]]
    z_errors = [pose["position_error_xyz_mm"][2] for pose in report["poses"]]
    summary = {
        "result": report["result"],
        "poses_sampled": len(report["poses"]),
        "position_error_norm_range_mm": [min(norms), max(norms)] if norms else None,
        "z_error_range_mm": [min(z_errors), max(z_errors)] if z_errors else None,
        "final_angles_deg": report.get("final_angles_deg"),
        "final_error": report.get("final_error"),
        "output": str(args.output),
        "error": report.get("error"),
    }
    print(json.dumps(summary, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
