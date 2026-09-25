#!/usr/bin/env python3
"""Execute a validated Stage-2 bundle against the deterministic SimRobot."""

from __future__ import annotations

import argparse

from drawing_robot.execution import (
    CommandLatencyConfig,
    ExecutionConfig,
    ExecutionLog,
    FaultSettings,
    LatencyMode,
    MotionExecutor,
    RobotState,
    RobotStatus,
    SimRobot,
    SimulationClock,
)
from drawing_robot.stage2.visualization import load_visualization_bundle


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", help="Stage-2B JSON bundle")
    parser.add_argument("--output", required=True, help="execution log JSON")
    parser.add_argument("--trajectory-id", default="trajectory")
    parser.add_argument(
        "--latency-mode", choices=[mode.value for mode in LatencyMode], default="ideal"
    )
    parser.add_argument("--base-delay-ms", type=float, default=0.0)
    parser.add_argument("--jitter-min-ms", type=float, default=0.0)
    parser.add_argument("--jitter-max-ms", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--stop-at", type=float)
    parser.add_argument("--drop-command", action="append", default=[])
    parser.add_argument("--stale-state-at", type=float)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cartesian, trajectory, validation = load_visualization_bundle(args.trajectory)
    del cartesian
    config = ExecutionConfig(
        command_latency=CommandLatencyConfig(
            LatencyMode(args.latency_mode),
            args.base_delay_ms / 1000.0,
            args.jitter_min_ms / 1000.0,
            args.jitter_max_ms / 1000.0,
        ),
        faults=FaultSettings(
            dropped_command_ids=tuple(args.drop_command),
            stale_state_at_s=args.stale_state_at,
        ),
        random_seed=args.seed,
    )
    clock = SimulationClock()
    initial_angles = trajectory.samples[0].positions_deg
    initial_state = RobotState(
        clock.now(), initial_angles, initial_angles, RobotStatus.IDLE
    )
    log = ExecutionLog(
        args.trajectory_id,
        config.to_dict(),
        config.random_seed,
        initial_state,
    )
    robot = SimRobot(clock, config, initial_angles, log)
    result = MotionExecutor(robot, clock, config, log).execute(
        trajectory,
        validation,
        trajectory_id=args.trajectory_id,
        stop_at_s=args.stop_at,
    )
    log.write(args.output)
    print(
        f"Stage-3 result: {result.status.value}; "
        f"commands={result.commands_issued}; log={args.output}"
    )
    return 0 if result.status.value in {"COMPLETED", "STOPPED"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
