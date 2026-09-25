#!/usr/bin/env python3
"""Run one small supervised travelling joint wave and return to REST."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from drawing_robot.telemetry import TelemetryFailure, TelemetryRecorder, maximum_joint_error
from drawing_robot.robot import RobotService
from scripts.two_pose_cycle import UPRIGHT_DEG, validate_pose


START_TEMPERATURE_C = config.TEMPERATURE_WARNING_C
ABORT_TEMPERATURE_C = config.TEMPERATURE_ABORT_C


def wave_waypoints(frames: int = 13) -> list[list[float]]:
    if frames < 5:
        raise ValueError("wave needs at least five frames")
    amplitudes = [0.0, 6.0, 6.0, 2.0, 5.0, 0.0]
    phases = [0.0, 0.0, -math.pi / 2, -math.pi, -3 * math.pi / 2, 0.0]
    points: list[list[float]] = []
    for index in range(frames):
        progress = index / (frames - 1)
        envelope = math.sin(math.pi * progress)
        phase = 2 * math.pi * progress
        point = [
            round(amplitude * envelope * math.sin(phase + offset), 3)
            for amplitude, offset in zip(amplitudes, phases, strict=True)
        ]
        validate_pose(point)
        points.append(point)
    points[0] = list(UPRIGHT_DEG)
    points[-1] = list(UPRIGHT_DEG)
    return points


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--frame-s", type=float, default=0.8)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.speed <= 10:
        raise SystemExit("--speed must be 1..10")
    if not 0.5 <= args.frame_s <= 2.0:
        raise SystemExit("--frame-s must be 0.5..2.0")
    waypoints = wave_waypoints()
    plan = {
        "sequence": ["measured REST", "UPRIGHT", "joint wave", "UPRIGHT", "REST"],
        "frames": len(waypoints),
        "frame_s": args.frame_s,
        "speed": args.speed,
        "amplitude_deg": {"J2": 6.0, "J3": 6.0, "J4": 2.0, "J5": 5.0},
        "start_temperature_c": START_TEMPERATURE_C,
        "abort_temperature_c": ABORT_TEMPERATURE_C,
        "output": str(args.output),
    }
    if not args.execute:
        print(json.dumps({"dry_run": True, "plan": plan, "waypoints": waypoints}, indent=2))
        return 0

    robot = RobotService(args.port, args.baud, telemetry=False)
    previous_fresh_mode = robot.get_fresh_mode()
    if previous_fresh_mode not in (0, 1):
        raise SystemExit(f"invalid fresh-mode response: {previous_fresh_mode!r}")
    robot.set_fresh_mode(1)
    recorder = TelemetryRecorder(robot, maximum_temperature_c=ABORT_TEMPERATURE_C)
    report: dict[str, object] = {
        "schema": "shared-growth/joint-wave/v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "plan": plan,
        "result": "running",
        "waypoints_deg": waypoints,
    }
    result_code = 1
    try:
        recorder.start()
        initial = recorder.latest()
        rest = list(initial["angles_deg"])
        validate_pose(rest)
        if int(initial["controller_error"]) != 0:
            raise TelemetryFailure(f"initial controller error: {initial['controller_error']}")
        if max(float(value) for value in initial["temperatures_c"]) > START_TEMPERATURE_C:
            raise TelemetryFailure(
                f"start temperature must be at or below {START_TEMPERATURE_C:.0f} C"
            )
        if robot.is_power_on() != 1 or robot.is_all_servo_enable() != 1:
            raise TelemetryFailure("robot power and all servos must be enabled")
        robot.arm_motion(locally_confirmed=True)
        report["rest_angles_deg"] = rest

        outward_initial = list(recorder.latest()["angles_deg"])
        outward = recorder.command("REST_to_UPRIGHT", UPRIGHT_DEG, args.speed)
        first_motion, settled = recorder.wait_for_target(
            UPRIGHT_DEG, initial_angles=outward_initial
        )
        report["outward_metrics"] = {
            "command_api_duration_s": outward.api_returned_s - outward.issued_s,
            "first_motion_after_command_s": first_motion - outward.issued_s,
            "settled_after_command_s": settled - outward.issued_s,
        }
        recorder.hold(1.0)

        for index, target in enumerate(waypoints[1:], start=1):
            recorder.command(f"wave_{index:02d}", target, args.speed)
            recorder.hold(args.frame_s)
        recorder.wait_for_target(UPRIGHT_DEG, timeout_s=15.0)

        inward_initial = list(recorder.latest()["angles_deg"])
        inward = recorder.command("UPRIGHT_to_REST", rest, args.speed)
        first_motion, settled = recorder.wait_for_target(rest, initial_angles=inward_initial)
        report["inward_metrics"] = {
            "command_api_duration_s": inward.api_returned_s - inward.issued_s,
            "first_motion_after_command_s": first_motion - inward.issued_s,
            "settled_after_command_s": settled - inward.issued_s,
            "final_error_max_deg": maximum_joint_error(
                list(recorder.latest()["angles_deg"]), rest
            ),
        }
        robot.stop()
        report["result"] = "completed"
        result_code = 0
    except Exception as exc:
        report["result"] = "aborted"
        report["failure"] = f"{type(exc).__name__}: {exc}"
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        recorder.close()
        report["commands"] = [asdict(event) for event in recorder.commands]
        report["samples"] = recorder.samples
        report["telemetry_failure"] = recorder.failure
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        try:
            robot.set_fresh_mode(previous_fresh_mode)
        except Exception:
            pass
        robot.close()

    temperatures = [
        float(value)
        for sample in recorder.samples
        for value in sample.get("temperatures_c", [])
    ]
    print(json.dumps({
        "result": report["result"],
        "failure": report.get("failure"),
        "samples": len(recorder.samples),
        "commands": len(recorder.commands),
        "maximum_temperature_c": max(temperatures) if temperatures else None,
        "outward_metrics": report.get("outward_metrics"),
        "inward_metrics": report.get("inward_metrics"),
        "output": str(args.output),
    }, indent=2))
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
