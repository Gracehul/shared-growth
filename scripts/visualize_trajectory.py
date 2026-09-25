#!/usr/bin/env python3
"""Inspect or export a Stage-2 trajectory without hardware access."""

from __future__ import annotations

import argparse
import sys


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", help="Stage-2B JSON bundle")
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--save", help="write a PNG/PDF/SVG instead of opening a window")
    output.add_argument("--save-animation", help="write planned sample replay as MP4")
    parser.add_argument("--sample", type=int, default=0, help="initial zero-based sample index")
    parser.add_argument("--fps", type=float, default=30.0, help="MP4 frame rate")
    parser.add_argument(
        "--playback-speed", type=float, default=1.0, help="MP4 time scale (1.0 = timestamps)"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.save or args.save_animation:
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
    if args.save_animation:
        output = visualizer.save_animation(
            args.save_animation,
            fps=args.fps,
            playback_speed=args.playback_speed,
        )
        print(f"Saved Stage-2B planned-sample replay to {output}")
    elif args.save:
        output = visualizer.save(args.save)
        print(f"Saved Stage-2B visualization to {output}")
    else:
        visualizer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
