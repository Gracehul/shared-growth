#!/usr/bin/env python3
"""Run one model-based, pen-up XZ S-curve with threaded telemetry."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import Arm, config
from drawing_robot.kinematics import frame_chain, manipulability, pose_coords
from drawing_robot.robot import RobotService
from drawing_robot.telemetry import TelemetryFailure, TelemetryRecorder, maximum_joint_error
from scripts.two_pose_cycle import validate_pose


START_TEMPERATURE_C = config.TEMPERATURE_WARNING_C
ABORT_TEMPERATURE_C = config.TEMPERATURE_ABORT_C
CONTROL_RATE_HZ = config.COMMAND_RATE_HZ
FIRMWARE_SPEED = 10
HALF_WIDTH_MM = 16.0
HALF_HEIGHT_MM = 7.0
CURVE_POINTS = 17
PATH_SPEED_CM_S = 0.8


def build_s_curve_plan(rest_angles_deg: list[float]) -> dict[str, object]:
    """Plan the full curve offline from a non-singular intermediate posture."""
    validate_pose(rest_angles_deg)
    center_q = np.asarray(rest_angles_deg, dtype=float) * 0.5
    validate_pose([float(value) for value in center_q])
    center_pose = pose_coords(center_q)
    progress = np.linspace(-1.0, 1.0, CURVE_POINTS)
    x_mm = center_pose[0] + HALF_WIDTH_MM * progress
    y_mm = np.full(CURVE_POINTS, center_pose[1])
    z_mm = center_pose[2] + HALF_HEIGHT_MM * np.sin(np.pi * progress)

    arm = Arm(mock=True, control_rate_hz=CONTROL_RATE_HZ)
    arm.conn.set_mock_angles(center_q)
    plan = arm.plan_path(
        x=(x_mm / 10.0).tolist(),
        y=(y_mm / 10.0).tolist(),
        z=(z_mm / 10.0).tolist(),
        speed=PATH_SPEED_CM_S,
    )
    if not plan.ok or plan.q_waypoints is None or plan.timestamps is None:
        raise TelemetryFailure(f"offline S-curve plan failed: {plan.error}")

    q_waypoints = np.asarray(plan.q_waypoints, dtype=float)
    for q in q_waypoints:
        validate_pose([float(value) for value in q])
    worst_manipulability = min(manipulability(q) for q in q_waypoints)
    if worst_manipulability < 1_000:
        raise TelemetryFailure(
            f"offline S-curve approaches a singularity: {worst_manipulability:.1f}"
        )
    minimum_link_z = min(
        float(transform[2, 3])
        for q in q_waypoints
        for transform in frame_chain(q)[1:]
    )
    if minimum_link_z < 100.0:
        raise TelemetryFailure(f"offline S-curve link height too low: {minimum_link_z:.1f} mm")

    return {
        "center_angles_deg": [float(value) for value in center_q],
        "center_tcp_pose_mm_deg": [float(value) for value in center_pose],
        "cartesian_targets_mm": [
            [float(x), float(y), float(z)] for x, y, z in zip(x_mm, y_mm, z_mm, strict=True)
        ],
        "q_waypoints_deg": q_waypoints.tolist(),
        "timestamps_s": np.asarray(plan.timestamps, dtype=float).tolist(),
        "duration_s": float(plan.duration_s),
        "peak_joint_dps": float(plan.peak_joint_dps),
        "worst_manipulability": float(worst_manipulability),
        "minimum_link_z_mm": float(minimum_link_z),
    }


def stream_waypoints(
    recorder: TelemetryRecorder,
    *,
    label: str,
    q_waypoints: list[list[float]],
    timestamps_s: list[float],
) -> None:
    started = recorder.elapsed()
    for index in range(1, len(q_waypoints)):
        deadline = started + float(timestamps_s[index])
        while recorder.elapsed() < deadline:
            recorder.raise_if_failed()
            time.sleep(min(0.02, deadline - recorder.elapsed()))
        recorder.command(f"{label}_{index:03d}", q_waypoints[index], FIRMWARE_SPEED)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    static_plan = {
        "plane": "XZ",
        "full_width_mm": 2 * HALF_WIDTH_MM,
        "full_height_mm": 2 * HALF_HEIGHT_MM,
        "nominal_speed_cm_s": PATH_SPEED_CM_S,
        "control_rate_hz": CONTROL_RATE_HZ,
        "firmware_speed": FIRMWARE_SPEED,
        "sequence": ["REST", "CENTER", "S forward", "S reverse", "CENTER", "REST"],
        "start_temperature_c": START_TEMPERATURE_C,
        "abort_temperature_c": ABORT_TEMPERATURE_C,
    }
    if not args.execute:
        example_rest = [0.7, 110.21, -145.63, -41.48, -19.77, 14.67]
        plan = build_s_curve_plan(example_rest)
        print(json.dumps({"dry_run": True, "plan": static_plan, "offline": {
            "duration_each_direction_s": plan["duration_s"],
            "streamed_waypoints": len(plan["q_waypoints_deg"]),
            "peak_joint_dps": plan["peak_joint_dps"],
            "worst_manipulability": plan["worst_manipulability"],
            "minimum_link_z_mm": plan["minimum_link_z_mm"],
            "center_tcp_pose_mm_deg": plan["center_tcp_pose_mm_deg"],
        }}, indent=2))
        return 0

    if not config.CARTESIAN_HARDWARE_ENABLED:
        raise SystemExit(
            "Cartesian hardware execution is disabled until READY/PARK thermal validation"
        )

    robot = RobotService(args.port, args.baud, telemetry=False)
    previous_fresh_mode = robot.get_fresh_mode()
    if previous_fresh_mode not in (0, 1):
        raise SystemExit(f"invalid fresh-mode response: {previous_fresh_mode!r}")
    robot.set_fresh_mode(1)
    recorder = TelemetryRecorder(robot, maximum_temperature_c=ABORT_TEMPERATURE_C)
    report: dict[str, object] = {
        "schema": "shared-growth/cartesian-s-curve/v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "plan": static_plan,
        "result": "running",
    }
    result_code = 1
    try:
        recorder.start()
        initial = recorder.latest()
        rest = [float(value) for value in initial["angles_deg"]]
        if int(initial["controller_error"]) != 0:
            raise TelemetryFailure(f"initial controller error: {initial['controller_error']}")
        if max(float(value) for value in initial["temperatures_c"]) > START_TEMPERATURE_C:
            raise TelemetryFailure(
                f"start temperature must be at or below {START_TEMPERATURE_C:.0f} C"
            )
        if robot.is_power_on() != 1 or robot.is_all_servo_enable() != 1:
            raise TelemetryFailure("robot power and all servos must be enabled")
        robot.arm_motion(locally_confirmed=True)
        offline = build_s_curve_plan(rest)
        report["rest_angles_deg"] = rest
        report["offline"] = offline

        center = list(offline["center_angles_deg"])
        initial_angles = list(recorder.latest()["angles_deg"])
        outward = recorder.command("REST_to_CENTER", center, 5)
        first_motion, settled = recorder.wait_for_target(center, initial_angles=initial_angles)
        report["center_outward_metrics"] = {
            "command_api_duration_s": outward.api_returned_s - outward.issued_s,
            "first_motion_after_command_s": first_motion - outward.issued_s,
            "settled_after_command_s": settled - outward.issued_s,
        }
        recorder.hold(0.8)

        q_waypoints = list(offline["q_waypoints_deg"])
        timestamps = list(offline["timestamps_s"])
        stream_waypoints(
            recorder,
            label="s_forward",
            q_waypoints=q_waypoints,
            timestamps_s=timestamps,
        )
        recorder.wait_for_target(q_waypoints[-1], timeout_s=10.0)
        recorder.hold(0.5)

        reverse_q = list(reversed(q_waypoints))
        stream_waypoints(
            recorder,
            label="s_reverse",
            q_waypoints=reverse_q,
            timestamps_s=timestamps,
        )
        recorder.wait_for_target(center, timeout_s=10.0)

        initial_angles = list(recorder.latest()["angles_deg"])
        inward = recorder.command("CENTER_to_REST", rest, 5)
        first_motion, settled = recorder.wait_for_target(rest, initial_angles=initial_angles)
        report["rest_return_metrics"] = {
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
    command_durations = [event.api_returned_s - event.issued_s for event in recorder.commands]
    print(json.dumps({
        "result": report["result"],
        "failure": report.get("failure"),
        "samples": len(recorder.samples),
        "commands": len(recorder.commands),
        "maximum_temperature_c": max(temperatures) if temperatures else None,
        "maximum_command_api_duration_s": max(command_durations) if command_durations else None,
        "center_outward_metrics": report.get("center_outward_metrics"),
        "rest_return_metrics": report.get("rest_return_metrics"),
        "output": str(args.output),
    }, indent=2))
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
