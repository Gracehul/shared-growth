#!/usr/bin/env python3
"""Compare matching Stage-3 simulation and Stage-4A hardware logs."""

import argparse
import json

from drawing_robot.execution import ExecutionLog
from drawing_robot.hardware.comparison import compare_sim_to_real


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulation_log")
    parser.add_argument("hardware_log")
    parser.add_argument("--output")
    args = parser.parse_args()
    result = compare_sim_to_real(
        ExecutionLog.read(args.simulation_log), ExecutionLog.read(args.hardware_log)
    )
    text = json.dumps(result, indent=2)
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
