#!/usr/bin/env python3
"""Print evidence metrics from an existing run log; never accesses hardware."""

import argparse
import json

from drawing_robot.hardware import analyze_execution_log


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    args = parser.parse_args()
    print(json.dumps(analyze_execution_log(args.log).to_dict(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
