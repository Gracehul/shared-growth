#!/usr/bin/env python3
"""Measure application-level stop latency during one small supervised move."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from characterize_joint import (
    FIRST_MOTION_THRESHOLD_DEG,
    GateFailure,
    read_angles,
    read_error,
    require_safe_temperatures,
)
from drawing_robot import config


SAMPLE_INTERVAL_S = 0.03
STABLE_STEP_DEG = 0.10
STABLE_SAMPLES = 4
TIMEOUT_S = 5.0
MAX_OTHER_JOINT_DRIFT_DEG = 2.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--joint", type=int, default=6)
    parser.add_argument("--amplitude", type=float, default=4.0)
    parser.add_argument("--speed", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-servo-temp-c", type=float, default=60.0)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.joint <= config.DOF:
        raise SystemExit("--joint must be 1..6")
    if args.amplitude <= 0 or args.amplitude > 5:
        raise SystemExit("--amplitude must be >0 and <=5 degrees")
    if args.speed < 1 or args.speed > 5:
        raise SystemExit("--speed must be 1..5")
    if not args.execute:
        print(json.dumps({"dry_run": True, "joint": args.joint, "amplitude": args.amplitude, "speed": args.speed}, indent=2))
        print("No robot connection opened. Pass --execute only during a supervised test.")
        return 0

    from pymycobot import MyCobot280

    report: dict[str, object] = {
        "schema": "shared-growth/stop-latency/v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "joint": args.joint,
        "amplitude_deg": args.amplitude,
        "speed": args.speed,
        "samples_before_stop": [],
        "samples_after_stop": [],
        "command_sent": False,
        "stop_sent": False,
    }
    robot = MyCobot280(args.port, args.baud)
    exit_code = 1
    try:
        initial, initial_attempts = read_angles(robot)
        error, _ = read_error(robot)
        servos = robot.is_all_servo_enable()
        report["initial_angles_deg"] = initial
        report["initial_angle_read_attempts"] = initial_attempts
        report["initial_error"] = error
        report["servos_enabled"] = servos
        report["initial_servo_temps_c"] = require_safe_temperatures(
            robot, args.max_servo_temp_c
        )
        if error != 0:
            raise GateFailure(f"initial controller error must be 0, got {error}")
        if servos != 1:
            raise GateFailure("all servos must be enabled")

        for index, (low, high) in enumerate(config.JOINT_LIMITS_DEG):
            if not low + config.JOINT_LIMIT_MARGIN_DEG <= initial[index] <= high - config.JOINT_LIMIT_MARGIN_DEG:
                raise GateFailure(f"J{index + 1} lacks configured limit margin")

        start_joint = initial[args.joint - 1]
        target = start_joint + args.amplitude
        low, high = config.JOINT_LIMITS_DEG[args.joint - 1]
        if not low + config.JOINT_LIMIT_MARGIN_DEG <= target <= high - config.JOINT_LIMIT_MARGIN_DEG:
            raise GateFailure("stop-test target lacks configured limit margin")
        report["target_deg"] = target

        robot.stop()
        time.sleep(0.25)
        started = time.monotonic()
        robot.send_angle(args.joint, round(target, 2), args.speed)
        report["command_sent"] = True
        report["command_call_s"] = time.monotonic() - started

        detected_angles: list[float] | None = None
        while time.monotonic() - started < TIMEOUT_S:
            angles, angle_attempts = read_angles(robot)
            controller_error, error_attempts = read_error(robot)
            elapsed = time.monotonic() - started
            other_drift = max(
                abs(angles[index] - initial[index])
                for index in range(config.DOF)
                if index != args.joint - 1
            )
            sample = {
                "elapsed_s": round(elapsed, 6),
                "angles_deg": angles,
                "controller_error": controller_error,
                "angle_read_attempts": angle_attempts,
                "error_read_attempts": error_attempts,
                "max_other_joint_drift_deg": other_drift,
            }
            report["samples_before_stop"].append(sample)
            if controller_error != 0:
                raise GateFailure(f"controller error before stop: {controller_error}")
            if other_drift > MAX_OTHER_JOINT_DRIFT_DEG:
                raise GateFailure(f"other-joint drift reached {other_drift:.2f} deg")
            if abs(angles[args.joint - 1] - start_joint) >= FIRST_MOTION_THRESHOLD_DEG:
                detected_angles = angles
                report["first_motion_detected_s"] = elapsed
                break
            time.sleep(SAMPLE_INTERVAL_S)
        if detected_angles is None:
            raise GateFailure("no movement detected before timeout")

        stop_started = time.monotonic()
        robot.stop()
        stop_returned = time.monotonic()
        report["stop_sent"] = True
        report["stop_call_s"] = stop_returned - stop_started
        report["joint_at_stop_detection_deg"] = detected_angles[args.joint - 1]

        previous = detected_angles[args.joint - 1]
        stable_count = 0
        while time.monotonic() - stop_started < TIMEOUT_S:
            angles, angle_attempts = read_angles(robot)
            controller_error, error_attempts = read_error(robot)
            elapsed = time.monotonic() - stop_started
            joint_deg = angles[args.joint - 1]
            step = abs(joint_deg - previous)
            sample = {
                "elapsed_since_stop_s": round(elapsed, 6),
                "angles_deg": angles,
                "joint_step_deg": step,
                "controller_error": controller_error,
                "angle_read_attempts": angle_attempts,
                "error_read_attempts": error_attempts,
            }
            report["samples_after_stop"].append(sample)
            if controller_error != 0:
                raise GateFailure(f"controller error after stop: {controller_error}")
            stable_count = stable_count + 1 if step <= STABLE_STEP_DEG else 0
            if stable_count >= STABLE_SAMPLES:
                report["result"] = "stable_after_stop"
                report["stop_to_stable_s"] = elapsed
                report["final_joint_deg"] = joint_deg
                report["movement_before_stop_deg"] = detected_angles[args.joint - 1] - start_joint
                report["movement_after_stop_detection_deg"] = joint_deg - detected_angles[args.joint - 1]
                report["final_servo_temps_c"] = require_safe_temperatures(
                    robot, args.max_servo_temp_c
                )
                exit_code = 0
                break
            previous = joint_deg
            time.sleep(SAMPLE_INTERVAL_S)
        else:
            raise GateFailure("joint did not become stable after stop")

    except Exception as exc:
        report["result"] = "aborted"
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            robot.stop()
            report["final_stop_sent"] = True
        except Exception as exc:
            report["final_stop_sent"] = False
            report["final_stop_error"] = f"{type(exc).__name__}: {exc}"
        try:
            report["final_angles_deg"], _ = read_angles(robot)
            report["final_error"], _ = read_error(robot)
        except Exception as exc:
            report["final_read_error"] = f"{type(exc).__name__}: {exc}"
        serial_port = getattr(robot, "_serial_port", None)
        if serial_port is not None:
            serial_port.close()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(json.dumps({
        "result": report.get("result"),
        "error": report.get("error"),
        "first_motion_detected_s": report.get("first_motion_detected_s"),
        "stop_call_s": report.get("stop_call_s"),
        "stop_to_stable_s": report.get("stop_to_stable_s"),
        "movement_before_stop_deg": report.get("movement_before_stop_deg"),
        "movement_after_stop_detection_deg": report.get("movement_after_stop_detection_deg"),
        "final_angles_deg": report.get("final_angles_deg"),
        "final_error": report.get("final_error"),
        "output": str(args.output),
    }, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
