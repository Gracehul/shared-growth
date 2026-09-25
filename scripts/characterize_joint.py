#!/usr/bin/env python3
"""Measure local joint-command latency and repeatability on a myCobot 280 JN.

This is a supervised hardware characterization tool, not a safety-rated test.
It connects once, uses a monotonic clock, stores every telemetry sample, and
stops after each target or immediately on a controller/safety-gate failure.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config


FIRST_MOTION_THRESHOLD_DEG = 0.15
TARGET_TOLERANCE_DEG = 1.20
SETTLE_STEP_DEG = 0.15
SETTLE_SAMPLES = 3
MAX_OTHER_JOINT_DRIFT_DEG = 2.0
TRIAL_TIMEOUT_S = 5.0
SAMPLE_INTERVAL_S = 0.05
READ_RETRIES = 4
READ_RETRY_DELAY_S = 0.05
DEFAULT_MAX_DEVELOPMENT_TEMP_C = 60.0


class GateFailure(RuntimeError):
    """Raised when a hardware state violates a test gate."""


def valid_angles(value: object) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == config.DOF
        and all(isinstance(item, (int, float)) for item in value)
    )


def read_angles(robot, retries: int = READ_RETRIES) -> tuple[list[float], int]:
    """Read six joint angles, retrying the firmware's sporadic ``-1`` reply."""
    last_value: object = None
    for attempt in range(1, retries + 1):
        last_value = robot.get_angles()
        if valid_angles(last_value):
            return [float(value) for value in last_value], attempt
        time.sleep(READ_RETRY_DELAY_S)
    raise GateFailure(f"invalid telemetry after {retries} attempts: {last_value!r}")


def read_error(robot, retries: int = READ_RETRIES) -> tuple[int, int]:
    """Read a non-negative controller status with bounded retries."""
    last_value: object = None
    for attempt in range(1, retries + 1):
        last_value = robot.get_error_information()
        if isinstance(last_value, int) and last_value >= 0:
            return last_value, attempt
        time.sleep(READ_RETRY_DELAY_S)
    raise GateFailure(f"invalid error status after {retries} attempts: {last_value!r}")


def require_safe_temperatures(
    robot,
    maximum_c: float = DEFAULT_MAX_DEVELOPMENT_TEMP_C,
) -> list[float]:
    """Apply a conservative project gate, not a manufacturer safety rating."""
    values = robot.get_servo_temps()
    if not (
        isinstance(values, (list, tuple))
        and len(values) == config.DOF
        and all(isinstance(value, (int, float)) for value in values)
    ):
        raise GateFailure(f"invalid servo temperatures: {values!r}")
    temperatures = [float(value) for value in values]
    hot = [index + 1 for index, value in enumerate(temperatures) if value >= maximum_c]
    if hot:
        raise GateFailure(
            f"project temperature gate {maximum_c:.1f} C reached by "
            + ", ".join(f"J{index}={temperatures[index - 1]:.1f} C" for index in hot)
        )
    return temperatures


def joint_targets(center_deg: float, amplitude_deg: float, repetitions: int) -> list[float]:
    targets: list[float] = []
    for _ in range(repetitions):
        targets.extend((center_deg + amplitude_deg, center_deg - amplitude_deg))
    targets.append(center_deg)
    return targets


def trial_metrics(
    samples: list[dict[str, object]],
    start_deg: float,
    target_deg: float,
    command_elapsed_s: float,
) -> dict[str, float | None]:
    first_motion = next(
        (
            float(sample["elapsed_s"])
            for sample in samples
            if abs(float(sample["joint_deg"]) - start_deg) >= FIRST_MOTION_THRESHOLD_DEG
        ),
        None,
    )
    target_entry = next(
        (
            float(sample["elapsed_s"])
            for sample in samples
            if abs(float(sample["joint_deg"]) - target_deg) <= TARGET_TOLERANCE_DEG
        ),
        None,
    )
    final_deg = float(samples[-1]["joint_deg"])
    direction = 1.0 if target_deg >= start_deg else -1.0
    directed_overshoot = max(
        0.0,
        max(direction * (float(sample["joint_deg"]) - target_deg) for sample in samples),
    )
    return {
        "command_call_s": command_elapsed_s,
        "first_motion_s": first_motion,
        "target_entry_s": target_entry,
        "settled_s": float(samples[-1]["elapsed_s"]),
        "final_error_deg": final_deg - target_deg,
        "overshoot_deg": directed_overshoot,
    }


def run_trial(robot, joint_id: int, target_deg: float, speed: int) -> dict[str, object]:
    before, before_read_attempts = read_angles(robot)
    start_deg = before[joint_id - 1]

    started = time.monotonic()
    robot.send_angle(joint_id, round(target_deg, 2), speed)
    command_elapsed_s = time.monotonic() - started
    samples: list[dict[str, object]] = []
    stable_count = 0
    previous_joint = start_deg

    while time.monotonic() - started < TRIAL_TIMEOUT_S:
        angles, angle_read_attempts = read_angles(robot)
        error, error_read_attempts = read_error(robot)
        elapsed = time.monotonic() - started
        joint_deg = angles[joint_id - 1]
        other_drift = max(
            abs(angles[index] - before[index])
            for index in range(config.DOF)
            if index != joint_id - 1
        )
        sample = {
            "elapsed_s": round(elapsed, 6),
            "angles_deg": angles,
            "joint_deg": joint_deg,
            "controller_error": error,
            "max_other_joint_drift_deg": other_drift,
            "angle_read_attempts": angle_read_attempts,
            "error_read_attempts": error_read_attempts,
        }
        samples.append(sample)

        if error != 0:
            raise GateFailure(f"controller error during motion: {error!r}")
        if other_drift > MAX_OTHER_JOINT_DRIFT_DEG:
            raise GateFailure(f"other-joint drift reached {other_drift:.2f} deg")

        in_target = abs(joint_deg - target_deg) <= TARGET_TOLERANCE_DEG
        low_step = abs(joint_deg - previous_joint) <= SETTLE_STEP_DEG
        stable_count = stable_count + 1 if in_target and low_step else 0
        if stable_count >= SETTLE_SAMPLES:
            robot.stop()
            return {
                "start_deg": start_deg,
                "before_read_attempts": before_read_attempts,
                "target_deg": target_deg,
                "speed": speed,
                "result": "settled",
                "samples": samples,
                "metrics": trial_metrics(samples, start_deg, target_deg, command_elapsed_s),
            }
        previous_joint = joint_deg
        time.sleep(SAMPLE_INTERVAL_S)

    raise GateFailure(
        f"trial timeout at {samples[-1]['joint_deg'] if samples else start_deg:.2f} deg"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--joint", type=int, default=6)
    parser.add_argument("--amplitude", type=float, default=2.0)
    parser.add_argument("--speeds", type=int, nargs="+", default=[2, 5, 10])
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--max-servo-temp-c",
        type=float,
        default=DEFAULT_MAX_DEVELOPMENT_TEMP_C,
        help="conservative project gate; not a manufacturer-rated threshold",
    )
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.joint <= config.DOF:
        raise SystemExit("--joint must be 1..6")
    if args.amplitude <= 0 or args.amplitude > 3:
        raise SystemExit("--amplitude must be >0 and <=3 degrees")
    if args.repetitions < 1 or args.repetitions > 10:
        raise SystemExit("--repetitions must be 1..10")
    if any(speed < 1 or speed > 10 for speed in args.speeds):
        raise SystemExit("characterization speeds must be 1..10")

    plan = {
        "joint": args.joint,
        "amplitude_deg": args.amplitude,
        "speeds": args.speeds,
        "repetitions": args.repetitions,
        "output": str(args.output),
        "max_servo_temp_c": args.max_servo_temp_c,
    }
    if not args.execute:
        print(json.dumps({"dry_run": True, "plan": plan}, indent=2))
        print("No robot connection opened. Pass --execute only during a supervised test.")
        return 0

    from pymycobot import MyCobot280

    report: dict[str, object] = {
        "schema": "shared-growth/joint-characterization/v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "plan": plan,
        "trials": [],
        "result": "running",
    }
    robot = MyCobot280(args.port, args.baud)
    exit_code = 1
    try:
        initial, initial_read_attempts = read_angles(robot)
        error, initial_error_read_attempts = read_error(robot)
        servos = robot.is_all_servo_enable()
        report["initial_angles_deg"] = initial
        report["initial_angle_read_attempts"] = initial_read_attempts
        report["initial_error"] = error
        report["initial_error_read_attempts"] = initial_error_read_attempts
        report["servos_enabled"] = servos
        report["initial_servo_temps_c"] = require_safe_temperatures(
            robot, args.max_servo_temp_c
        )
        if error != 0:
            raise GateFailure(f"initial controller error must be 0, got {error!r}")
        if servos != 1:
            raise GateFailure("all servos must be enabled")

        for index, (low, high) in enumerate(config.JOINT_LIMITS_DEG):
            angle = initial[index]
            margin = config.JOINT_LIMIT_MARGIN_DEG
            if not low + margin <= angle <= high - margin:
                raise GateFailure(f"J{index + 1} lacks {margin:.1f} deg limit margin")

        center = initial[args.joint - 1]
        low, high = config.JOINT_LIMITS_DEG[args.joint - 1]
        targets = joint_targets(center, args.amplitude, args.repetitions)
        if any(not low + config.JOINT_LIMIT_MARGIN_DEG <= target <= high - config.JOINT_LIMIT_MARGIN_DEG for target in targets):
            raise GateFailure("planned target lacks configured joint-limit margin")

        for speed in args.speeds:
            for target in targets:
                trial_number = len(report["trials"]) + 1
                try:
                    trial = run_trial(robot, args.joint, target, speed)
                except Exception as exc:
                    robot.stop()
                    report["trials"].append(
                        {
                            "trial": trial_number,
                            "speed": speed,
                            "target_deg": target,
                            "result": "aborted",
                            "error": f"{type(exc).__name__}: {exc}",
                        }
                    )
                    raise
                trial["trial"] = trial_number
                trial["servo_temps_c_after"] = require_safe_temperatures(
                    robot, args.max_servo_temp_c
                )
                report["trials"].append(trial)
                time.sleep(0.3)

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
            report["final_angles_deg"], report["final_angle_read_attempts"] = read_angles(robot)
            report["final_error"], report["final_error_read_attempts"] = read_error(robot)
        except Exception as exc:
            report["final_read_error"] = f"{type(exc).__name__}: {exc}"
        serial_port = getattr(robot, "_serial_port", None)
        if serial_port is not None:
            serial_port.close()

        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    summary = {
        "result": report["result"],
        "trials_completed": sum(
            trial.get("result") == "settled" for trial in report["trials"]
        ),
        "output": str(args.output),
        "error": report.get("error"),
        "final_angles_deg": report.get("final_angles_deg"),
        "final_error": report.get("final_error"),
    }
    print(json.dumps(summary, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
