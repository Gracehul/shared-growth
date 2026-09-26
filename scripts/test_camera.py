#!/usr/bin/env python3
"""Run a camera-only baseline capture. This script never accesses the robot."""

import argparse
import json

from camera import CameraConfig, capture_baseline, detect_camera_devices


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()

    devices = detect_camera_devices()
    if args.device == "auto":
        if not devices:
            raise SystemExit("No /dev/video* camera device detected")
        device = devices[0]
    else:
        device = args.device
    config = CameraConfig(
        device=device,
        width=args.width,
        height=args.height,
        requested_fps=args.fps,
        duration_s=args.duration,
        preview=args.preview,
    )
    report = capture_baseline(config, args.output_dir)
    summary = dict(report["metrics"])
    summary.update({
        "camera_device": report["camera_device"],
        "camera_backend": report["camera_backend"],
        "requested": report["requested"],
        "selected": report["selected"],
        "robot_commands_sent": report["robot_commands_sent"],
    })
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
