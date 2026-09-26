#!/usr/bin/env python3
"""Run the strict Stage-4A observational gate without state-changing calls."""

import argparse

from drawing_robot.hardware import Stage4AConfig
from drawing_robot.hardware.read_only_validation import validate_read_only_session
from drawing_robot.robot import RobotService


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--period", type=float, default=0.1)
    parser.add_argument("--physical-preflight-pass", action="store_true")
    args = parser.parse_args()
    if not args.physical_preflight_pass:
        raise SystemExit("NO_GO: require --physical-preflight-pass")
    with RobotService(
        args.port, args.baud, read_only=True, telemetry=False
    ) as service:
        result = validate_read_only_session(
            service,
            Stage4AConfig(),
            physical_preflight_pass=True,
            min_consecutive_samples=args.samples,
            sample_period_s=args.period,
        )
        result.log.write(args.output)
    print(f"Stage-4A read-only decision: {result.decision}")
    if result.current_tcp_pose_mm_deg is not None:
        print("Current TCP pose [mm, deg]:", list(result.current_tcp_pose_mm_deg))
    if result.abort_reason:
        print("Abort reason:", result.abort_reason)
    print("Log:", args.output)
    return 0 if result.decision == "GO" else 2


if __name__ == "__main__":
    raise SystemExit(main())
