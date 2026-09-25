#!/usr/bin/env python3
"""Replay recorded Stage-3 states and events without re-simulation."""

from __future__ import annotations

import argparse

from drawing_robot.execution import ExecutionLog, replay_log


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("execution_log")
    args = parser.parse_args()
    log = ExecutionLog.read(args.execution_log)
    print(
        f"run={log.run_id} trajectory={log.trajectory_id} "
        f"result={log.result.get('status')}"
    )
    for frame in replay_log(log):
        kinds = ",".join(event["kind"] for event in frame.events) or "-"
        print(
            f"t={frame.timestamp_s:8.3f}s "
            f"state={frame.state['status']:8s} events={kinds}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
