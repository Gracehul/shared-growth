#!/usr/bin/env python3
"""Write offline timing-profile samples; never opens a robot connection."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot.motion_profiles import PROFILES, sample_profile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--sample-rate", type=float, default=25.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows: list[dict[str, float | str]] = []
    summary: dict[str, object] = {
        "duration_s": args.duration,
        "sample_rate_hz": args.sample_rate,
        "profiles": {},
        "robot_connection_opened": False,
    }
    for name in PROFILES:
        samples = sample_profile(name, args.duration, args.sample_rate)
        summary["profiles"][name] = samples
        rows.extend({"profile": name, **sample} for sample in samples)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.suffix.lower() == ".csv":
        with args.output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["profile", "time_s", "progress"])
            writer.writeheader()
            writer.writerows(rows)
    else:
        args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(json.dumps({
        "profiles": list(PROFILES),
        "samples_per_profile": len(next(iter(summary["profiles"].values()))),
        "output": str(args.output),
        "robot_connection_opened": False,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
