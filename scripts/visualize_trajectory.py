#!/usr/bin/env python3
"""Inspect or export a Stage-2 trajectory without hardware access."""

from __future__ import annotations

import argparse
import sys


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", help="Stage-2B JSON bundle")
    parser.add_argument("--save", help="write a PNG/PDF/SVG instead of opening a window")
    parser.add_argument("--sample", type=int, default=0, help="initial zero-based sample index")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.save:
        import matplotlib

        matplotlib.use("Agg")

    from drawing_robot.stage2.visualization import (
        MotionVisualizer,
        VisualizationInputError,
        load_visualization_bundle,
    )

    try:
        cartesian, joint, validation = load_visualization_bundle(args.trajectory)
        visualizer = MotionVisualizer(cartesian, joint, validation)
        visualizer.select_sample(args.sample)
    except VisualizationInputError as exc:
        print(f"Visualization input error: {exc}", file=sys.stderr)
        return 2
    if args.save:
        output = visualizer.save(args.save)
        print(f"Saved Stage-2B visualization to {output}")
    else:
        visualizer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
