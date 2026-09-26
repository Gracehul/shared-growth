import math

import pytest

from camera import CameraConfig, timing_metrics


def test_camera_config_rejects_invalid_capture_values():
    with pytest.raises(ValueError, match="dimensions"):
        CameraConfig(width=0).validate()
    with pytest.raises(ValueError, match="requested_fps"):
        CameraConfig(requested_fps=0).validate()
    with pytest.raises(ValueError, match="duration"):
        CameraConfig(duration_s=0).validate()


def test_timing_metrics_use_monotonic_frame_spacing():
    result = timing_metrics([10.0, 10.1, 10.2, 10.4], requested_fps=10.0)
    assert result["frame_count"] == 4
    assert result["capture_duration_s"] == pytest.approx(0.4)
    assert result["effective_fps"] == pytest.approx(7.5)
    assert result["mean_frame_interval_s"] == pytest.approx(0.4 / 3)
    assert result["max_frame_interval_s"] == pytest.approx(0.2)
    assert result["estimated_dropped_frames"] == 1


def test_empty_timing_metrics_are_explicit():
    result = timing_metrics([], requested_fps=30.0, failed_frames=2)
    assert result["effective_fps"] == 0.0
    assert result["mean_frame_interval_s"] is None
    assert result["failed_frames"] == 2
    assert math.isfinite(result["capture_duration_s"])
