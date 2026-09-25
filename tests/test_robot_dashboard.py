from scripts.robot_dashboard import TelemetrySampler
from drawing_robot.robot import RobotService

import time


def test_mock_sample_is_read_only_and_complete() -> None:
    sampler = TelemetrySampler("/dev/null", 1_000_000, 1.0, mock=True)
    sample = sampler._mock_sample()
    assert sample["connected"] is True
    assert sample["read_only"] is True
    assert sample["motion_enabled"] is False
    assert sample["controller_error"] == 0
    assert len(sample["joints"]) == 6


def test_temperature_gate_creates_motion_pause_alert() -> None:
    sampler = TelemetrySampler("/dev/null", 1_000_000, 1.0, mock=True)
    sample = sampler._build_sample(
        angles=[0, 0, 0, 0, 0, 0],
        temperatures=[40, 41, 42, 72, 44, 45],
        voltages=[12, 12, 12, 7, 7, 7],
        statuses=[0, 0, 0, 0, 0, 0],
        speeds=[0, 0, 0, 0, 0, 0],
        firmware_pose=[0, 0, 0, 0, 0, 0],
        error=0,
        powered=True,
        servos_enabled=True,
        latency_ms=10,
    )
    assert sample["severity"] == "danger"
    assert sample["state_label"] == "Motion paused"
    assert any(alert["title"] == "J4 temperature gate" for alert in sample["alerts"])


def test_joint_limit_margin_creates_alert() -> None:
    sampler = TelemetrySampler("/dev/null", 1_000_000, 1.0, mock=True)
    sample = sampler._build_sample(
        angles=[167, 0, 0, 0, 0, 0],
        temperatures=[40] * 6,
        voltages=[12] * 6,
        statuses=[0] * 6,
        speeds=[0] * 6,
        firmware_pose=[0] * 6,
        error=0,
        powered=True,
        servos_enabled=True,
        latency_ms=10,
    )
    assert any(alert["title"] == "J1 limit margin" for alert in sample["alerts"])


def test_dashboard_consumes_published_service_state() -> None:
    sampler = TelemetrySampler("/dev/null", 1_000_000, 1.0, mock=True)
    with RobotService(mock=True, read_only=True) as service:
        unsubscribe = service.subscribe(sampler._consume_state)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not sampler.payload()["current"]["connected"]:
            time.sleep(0.01)
        unsubscribe()
    payload = sampler.payload()
    assert payload["current"]["connected"] is True
    assert payload["current"]["motion_enabled"] is False
