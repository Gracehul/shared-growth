import pytest

from drawing_robot.telemetry import TelemetryFailure, maximum_joint_error, read_vector
from scripts.joint_wave import wave_waypoints


def test_wave_starts_and_ends_upright() -> None:
    points = wave_waypoints()
    assert points[0] == [0.0] * 6
    assert points[-1] == [0.0] * 6


def test_wave_respects_joint_amplitudes() -> None:
    points = wave_waypoints()
    assert max(abs(point[1]) for point in points) <= 6.0
    assert max(abs(point[2]) for point in points) <= 6.0
    assert max(abs(point[3]) for point in points) <= 2.0
    assert max(abs(point[4]) for point in points) <= 5.0
    assert all(point[0] == 0.0 and point[5] == 0.0 for point in points)


def test_vector_reader_rejects_repeated_invalid_values() -> None:
    with pytest.raises(TelemetryFailure):
        read_vector(lambda: -1, "test")


def test_maximum_joint_error() -> None:
    assert maximum_joint_error([0, 1, 2, 3, 4, 5], [0, 1, 2, 3, 4, 7]) == 2
