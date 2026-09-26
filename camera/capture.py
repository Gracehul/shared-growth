"""Timestamped OpenCV/V4L2 capture with no robot-control dependencies."""

import glob
import json
import math
import platform
import time
from datetime import datetime, timezone
from pathlib import Path

from .camera_config import CameraConfig


def detect_camera_devices():
    """Return Linux V4L2 capture device paths in stable lexical order."""
    return tuple(sorted(glob.glob("/dev/video*")))


def timing_metrics(timestamps_s, requested_fps, failed_frames=0):
    """Summarize monotonic frame timestamps without requiring OpenCV."""
    values = [float(value) for value in timestamps_s]
    intervals = [b - a for a, b in zip(values, values[1:])]
    duration = values[-1] - values[0] if len(values) > 1 else 0.0
    effective_fps = (len(values) - 1) / duration if duration > 0 else 0.0
    expected_interval = 1.0 / float(requested_fps)
    estimated_dropped = sum(
        max(0, int(round(interval / expected_interval)) - 1)
        for interval in intervals
        if math.isfinite(interval) and interval > 0
    )
    return {
        "frame_count": len(values),
        "capture_duration_s": duration,
        "effective_fps": effective_fps,
        "mean_frame_interval_s": sum(intervals) / len(intervals) if intervals else None,
        "max_frame_interval_s": max(intervals) if intervals else None,
        "failed_frames": int(failed_frames),
        "estimated_dropped_frames": estimated_dropped,
    }


def _load_cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for camera capture (import cv2 failed)"
        ) from exc
    return cv2


def capture_baseline(config, output_dir, cv2_module=None):
    """Capture one timestamped baseline run and return its JSON-compatible report."""
    config.validate()
    cv2 = cv2_module or _load_cv2()
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    sample_path = output / "sample_frame.png"
    video_path = output / "camera_test.mp4"
    json_path = output / "camera_test.json"

    backend_flag = getattr(cv2, "CAP_V4L2", 0)
    capture = cv2.VideoCapture(config.device, backend_flag)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError("camera could not be opened: " + config.device)

    writer = None
    timestamps = []
    frame_records = []
    failed_frames = 0
    video_frames_written = 0
    start_monotonic = time.monotonic()
    start_utc = datetime.now(timezone.utc).isoformat()
    report = None
    try:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.pixel_format))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
        capture.set(cv2.CAP_PROP_FPS, config.requested_fps)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        selected_width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        selected_height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        selected_fps = float(capture.get(cv2.CAP_PROP_FPS))
        selected_fourcc_value = int(capture.get(cv2.CAP_PROP_FOURCC))
        selected_fourcc = "".join(
            chr((selected_fourcc_value >> (8 * index)) & 0xFF) for index in range(4)
        ).rstrip("\x00")
        try:
            backend_name = capture.getBackendName()
        except Exception:
            backend_name = "unknown"

        writer = cv2.VideoWriter(
            str(video_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            config.requested_fps,
            (selected_width, selected_height),
        )
        if not writer.isOpened():
            raise RuntimeError("MP4 video writer could not be opened")

        deadline = start_monotonic + config.duration_s
        while time.monotonic() < deadline:
            ok, frame = capture.read()
            captured_at = time.monotonic()
            if not ok or frame is None:
                failed_frames += 1
                continue
            if frame.shape[1] != selected_width or frame.shape[0] != selected_height:
                raise RuntimeError(
                    "camera frame dimensions changed during capture: "
                    + repr(tuple(frame.shape[:2]))
                )
            timestamps.append(captured_at)
            frame_records.append({
                "frame_index": len(timestamps) - 1,
                "monotonic_s": captured_at,
                "offset_s": captured_at - start_monotonic,
            })
            if len(timestamps) == 1 and not cv2.imwrite(str(sample_path), frame):
                raise RuntimeError("sample frame could not be saved")
            desired_video_frames = (
                int(round((captured_at - timestamps[0]) * config.requested_fps)) + 1
            )
            while video_frames_written < desired_video_frames:
                writer.write(frame)
                video_frames_written += 1
            if config.preview:
                cv2.imshow("Shared Growth camera baseline", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

        metrics = timing_metrics(timestamps, config.requested_fps, failed_frames)
        if metrics["frame_count"] == 0:
            raise RuntimeError("camera opened but produced no valid frames")
        report = {
            "schema_version": "shared-growth/camera-baseline/v1",
            "start_time_utc": start_utc,
            "start_monotonic_s": start_monotonic,
            "camera_device": config.device,
            "camera_backend": backend_name,
            "opencv_version": getattr(cv2, "__version__", "unknown"),
            "python_version": platform.python_version(),
            "requested": {
                "resolution": [config.width, config.height],
                "fps": config.requested_fps,
                "pixel_format": config.pixel_format,
            },
            "selected": {
                "resolution": [selected_width, selected_height],
                "fps": selected_fps,
                "pixel_format": selected_fourcc,
            },
            "frames": frame_records,
            "metrics": metrics,
            "video": {
                "encoded_fps": config.requested_fps,
                "frames_written": video_frames_written,
                "expected_duration_s": (
                    video_frames_written / config.requested_fps
                    if video_frames_written
                    else 0.0
                ),
            },
            "artifacts": {
                "sample_frame": sample_path.name,
                "video": video_path.name,
            },
            "robot_commands_sent": 0,
        }
        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        if config.preview:
            cv2.destroyAllWindows()
