#!/usr/bin/env python3
"""Run one supervised REST -> UPRIGHT -> REST cycle with telemetry logging.

UPRIGHT is the manufacturer's calibrated zero pose [0, 0, 0, 0, 0, 0].
The exact measured start pose becomes REST for this run. This is a development
test, not a safety-rated motion controller.
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
from drawing_robot.kinematics import flange_pose_coords
from scripts.characterize_joint import GateFailure, read_angles, read_error


UPRIGHT_DEG = [0.0] * 6
TARGET_TOLERANCE_DEG = 2.0
SETTLE_STEP_DEG = 0.20
SETTLE_SAMPLES = 4
SAMPLE_INTERVAL_S = 0.10
DEFAULT_TIMEOUT_S = 45.0
DEFAULT_MAX_TEMP_C = 55.0
READ_RETRIES = 4
READ_RETRY_DELAY_S = 0.05


def validate_pose(pose: list[float], margin_deg: float = 3.0) -> None:
    if len(pose) != config.DOF:
        raise GateFailure(f"expected {config.DOF} joint values, got {len(pose)}")
    for index, (angle, limits) in enumerate(zip(pose, config.JOINT_LIMITS_DEG, strict=True)):
        low, high = limits
        if not low + margin_deg <= angle <= high - margin_deg:
            raise GateFailure(
                f"J{index + 1}={angle:.2f} deg violates {margin_deg:.1f} deg limit margin"
            )


def maximum_error(angles: list[float], target: list[float]) -> float:
    return max(abs(actual - expected) for actual, expected in zip(angles, target, strict=True))


def read_vector(robot_call, name: str) -> tuple[list[float], int]:
    last_value: object = None
    for attempt in range(1, READ_RETRIES + 1):
        last_value = robot_call()
        if (
            isinstance(last_value, (list, tuple))
            and len(last_value) == config.DOF
            and all(isinstance(value, (int, float)) for value in last_value)
        ):
            return [float(value) for value in last_value], attempt
        time.sleep(READ_RETRY_DELAY_S)
    raise GateFailure(f"invalid {name} after {READ_RETRIES} attempts: {last_value!r}")


def read_safe_temperatures(
    robot, maximum_c: float = DEFAULT_MAX_TEMP_C
) -> tuple[list[float], int]:
    temperatures, attempts = read_vector(robot.get_servo_temps, "servo temperatures")
    hot = [index + 1 for index, value in enumerate(temperatures) if value >= maximum_c]
    if hot:
        raise GateFailure(
            f"project temperature gate {maximum_c:.1f} C reached by "
            + ", ".join(f"J{index}={temperatures[index - 1]:.1f} C" for index in hot)
        )
    return temperatures, attempts


def read_sample(robot, started: float, maximum_temperature_c: float) -> dict[str, object]:
    angles, angle_attempts = read_angles(robot)
    error, error_attempts = read_error(robot)
    temperatures, temperature_attempts = read_safe_temperatures(robot, maximum_temperature_c)
    coords, coordinate_attempts = read_vector(robot.get_coords, "firmware pose")
    speeds, speed_attempts = read_vector(robot.get_servo_speeds, "servo speeds")
    voltages, voltage_attempts = read_vector(robot.get_servo_voltages, "servo voltages")
    return {
        "elapsed_s": round(time.monotonic() - started, 6),
        "angles_deg": angles,
        "firmware_flange_pose_mm_deg": [float(value) for value in coords],
        "python_flange_pose_mm_deg": [float(value) for value in flange_pose_coords(angles)],
        "temperatures_c": temperatures,
        "servo_speeds": [float(value) for value in speeds],
        "servo_voltages_v": [float(value) for value in voltages],
        "controller_error": error,
        "angle_read_attempts": angle_attempts,
        "error_read_attempts": error_attempts,
        "temperature_read_attempts": temperature_attempts,
        "coordinate_read_attempts": coordinate_attempts,
        "speed_read_attempts": speed_attempts,
        "voltage_read_attempts": voltage_attempts,
    }


def move_and_measure(
    robot,
    *,
    name: str,
    target: list[float],
    speed: int,
    timeout_s: float,
    maximum_temperature_c: float = DEFAULT_MAX_TEMP_C,
) -> dict[str, object]:
    validate_pose(target)
    before, _ = read_angles(robot)
    started = time.monotonic()
    robot.send_angles([round(value, 2) for value in target], speed)
    samples: list[dict[str, object]] = []
    stable = 0
    previous = before
    first_motion_s = None

    while time.monotonic() - started < timeout_s:
        sample = read_sample(robot, started, maximum_temperature_c)
        angles = sample["angles_deg"]
        assert isinstance(angles, list)
        samples.append(sample)
        if int(sample["controller_error"]) != 0:
            raise GateFailure(f"controller error during {name}: {sample['controller_error']}")
        if first_motion_s is None and maximum_error(angles, before) >= 0.20:
            first_motion_s = float(sample["elapsed_s"])
        in_target = maximum_error(angles, target) <= TARGET_TOLERANCE_DEG
        low_step = maximum_error(angles, previous) <= SETTLE_STEP_DEG
        stable = stable + 1 if in_target and low_step else 0
        if stable >= SETTLE_SAMPLES:
            robot.stop()
            return {
                "name": name,
                "target_angles_deg": target,
                "speed": speed,
                "first_motion_s": first_motion_s,
                "settled_s": float(sample["elapsed_s"]),
                "final_error_max_deg": maximum_error(angles, target),
                "samples": samples,
            }
        previous = angles
        time.sleep(SAMPLE_INTERVAL_S)
    raise GateFailure(f"{name} did not settle within {timeout_s:.1f} s")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--hold-s", type=float, default=3.0)
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.speed <= 10:
        raise SystemExit("--speed must be 1..10 for this supervised test")
    if not 0 <= args.hold_s <= 10:
        raise SystemExit("--hold-s must be 0..10")
    if args.timeout_s < 10 or args.timeout_s > 90:
        raise SystemExit("--timeout-s must be 10..90")

    plan = {
        "sequence": ["measured REST", "UPRIGHT zero pose", "measured REST"],
        "upright_angles_deg": UPRIGHT_DEG,
        "speed": args.speed,
        "hold_s": args.hold_s,
        "maximum_temperature_c": DEFAULT_MAX_TEMP_C,
        "output": str(args.output),
    }
    if not args.execute:
        print(json.dumps({"dry_run": True, "plan": plan}, indent=2))
        print("No robot connection opened. Pass --execute only with an operator present.")
        return 0

    from pymycobot import MyCobot280

    robot = MyCobot280(args.port, args.baud)
    report: dict[str, object] = {
        "schema": "shared-growth/two-pose-cycle/v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "plan": plan,
        "legs": [],
        "result": "running",
    }
    result_code = 1
    try:
        rest, rest_attempts = read_angles(robot)
        validate_pose(rest)
        error, error_attempts = read_error(robot)
        temperatures, temperature_attempts = read_safe_temperatures(robot)
        if error != 0:
            raise GateFailure(f"initial controller error: {error}")
        if robot.is_power_on() != 1 or robot.is_all_servo_enable() != 1:
            raise GateFailure("robot power and all servos must be enabled")
        report.update({
            "rest_angles_deg": rest,
            "rest_angle_read_attempts": rest_attempts,
            "initial_error_read_attempts": error_attempts,
            "initial_temperatures_c": temperatures,
            "initial_temperature_read_attempts": temperature_attempts,
            "rest_firmware_pose_mm_deg": robot.get_coords(),
            "upright_python_flange_pose_mm_deg": [float(value) for value in flange_pose_coords(UPRIGHT_DEG)],
        })

        outward = move_and_measure(
            robot,
            name="REST_to_UPRIGHT",
            target=UPRIGHT_DEG,
            speed=args.speed,
            timeout_s=args.timeout_s,
        )
        report["legs"].append(outward)
        time.sleep(args.hold_s)
        inward = move_and_measure(
            robot,
            name="UPRIGHT_to_REST",
            target=rest,
            speed=args.speed,
            timeout_s=args.timeout_s,
        )
        report["legs"].append(inward)
        report["final_angles_deg"], _ = read_angles(robot)
        report["final_temperatures_c"], report["final_temperature_read_attempts"] = read_safe_temperatures(robot)
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
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        serial_port = getattr(robot, "_serial_port", None)
        if serial_port is not None:
            serial_port.close()

    print(json.dumps({
        "result": report["result"],
        "output": str(args.output),
        "rest_angles_deg": report.get("rest_angles_deg"),
        "legs": [
            {
                "name": leg["name"],
                "first_motion_s": leg["first_motion_s"],
                "settled_s": leg["settled_s"],
                "final_error_max_deg": leg["final_error_max_deg"],
                "samples": len(leg["samples"]),
            }
            for leg in report["legs"]
        ],
        "failure": report.get("failure"),
    }, indent=2))
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
