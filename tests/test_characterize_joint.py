import pytest

from scripts.characterize_joint import (
    GateFailure,
    joint_targets,
    read_angles,
    require_safe_temperatures,
    trial_metrics,
    valid_angles,
)


def test_joint_targets_alternate_and_return_to_center() -> None:
    assert joint_targets(12.0, 2.0, 2) == [14.0, 10.0, 14.0, 10.0, 12.0]


def test_valid_angles_requires_six_numeric_values() -> None:
    assert valid_angles([0, 1.0, 2, 3, 4, 5])
    assert not valid_angles([0, 1, 2])
    assert not valid_angles([0, 1, 2, 3, 4, None])


def test_trial_metrics_extracts_latency_error_and_overshoot() -> None:
    samples = [
        {"elapsed_s": 0.01, "joint_deg": 12.0},
        {"elapsed_s": 0.10, "joint_deg": 12.2},
        {"elapsed_s": 0.20, "joint_deg": 14.2},
        {"elapsed_s": 0.30, "joint_deg": 14.0},
    ]
    metrics = trial_metrics(samples, 12.0, 14.0, 0.005)
    assert metrics["command_call_s"] == 0.005
    assert metrics["first_motion_s"] == 0.10
    assert metrics["target_entry_s"] == 0.20
    assert metrics["settled_s"] == 0.30
    assert metrics["final_error_deg"] == 0.0
    assert abs(metrics["overshoot_deg"] - 0.2) < 1e-9


def test_read_angles_retries_sporadic_minus_one() -> None:
    class Robot:
        replies = iter([-1, [0, 1, 2, 3, 4, 5]])

        def get_angles(self):
            return next(self.replies)

    angles, attempts = read_angles(Robot())
    assert angles == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert attempts == 2


def test_temperature_gate_rejects_hot_joint() -> None:
    class Robot:
        def get_servo_temps(self):
            return [40, 41, 42, 60, 43, 44]

    with pytest.raises(GateFailure, match="J4=60.0 C"):
        require_safe_temperatures(Robot(), maximum_c=60.0)
