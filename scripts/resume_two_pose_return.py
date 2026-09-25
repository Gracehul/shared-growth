#!/usr/bin/env python3
"""Resume only the return leg of an interrupted two-pose cycle."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config
from scripts.characterize_joint import GateFailure, read_error
from scripts.two_pose_cycle import move_and_measure, read_safe_temperatures, validate_pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--max-servo-temp-c", type=float, default=60.0)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.report.exists():
        raise SystemExit(f"report does not exist: {args.report}")
    report = json.loads(args.report.read_text(encoding="utf-8"))
    rest = [float(value) for value in report.get("rest_angles_deg", [])]
    validate_pose(rest)
    plan = {
        "action": "current pose to recorded REST only",
        "rest_angles_deg": rest,
        "speed": args.speed,
        "maximum_temperature_c": args.max_servo_temp_c,
    }
    if not args.execute:
        print(json.dumps({"dry_run": True, "plan": plan}, indent=2))
        return 0

    from pymycobot import MyCobot280

    robot = MyCobot280(args.port, args.baud)
    result_code = 1
    try:
        error, _ = read_error(robot)
        temperatures, attempts = read_safe_temperatures(robot, args.max_servo_temp_c)
        if error != 0:
            raise GateFailure(f"initial controller error: {error}")
        if robot.is_power_on() != 1 or robot.is_all_servo_enable() != 1:
            raise GateFailure("robot power and all servos must be enabled")
        recovery = move_and_measure(
            robot,
            name="thermal_recovery_to_REST",
            target=rest,
            speed=args.speed,
            timeout_s=args.timeout_s,
            maximum_temperature_c=args.max_servo_temp_c,
        )
        report.setdefault("recovery_legs", []).append(recovery)
        report["recovery_initial_temperatures_c"] = temperatures
        report["recovery_initial_temperature_read_attempts"] = attempts
        report["result"] = "completed_after_thermal_pause"
        result_code = 0
    except Exception as exc:
        report["recovery_failure"] = f"{type(exc).__name__}: {exc}"
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        report["recovery_finished_utc"] = datetime.now(timezone.utc).isoformat()
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        serial_port = getattr(robot, "_serial_port", None)
        if serial_port is not None:
            serial_port.close()

    summary = {
        "result": report.get("result"),
        "recovery_failure": report.get("recovery_failure"),
        "recovery": [
            {
                "first_motion_s": leg["first_motion_s"],
                "settled_s": leg["settled_s"],
                "final_error_max_deg": leg["final_error_max_deg"],
                "samples": len(leg["samples"]),
            }
            for leg in report.get("recovery_legs", [])
        ],
    }
    print(json.dumps(summary, indent=2))
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
