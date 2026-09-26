"""Explicitly opted-in real-device smoke check; it never commands motion."""

import os
import time

import pytest

from drawing_robot.robot import RobotService


pytestmark = pytest.mark.hardware


@pytest.mark.skipif(
    os.environ.get("RUN_REAL_ROBOT_TESTS") != "1",
    reason="requires RUN_REAL_ROBOT_TESTS=1 on the supervised Nano",
)
def test_real_robot_publishes_complete_read_only_state() -> None:
    with RobotService("/dev/ttyTHS1", 1_000_000, read_only=True) as service:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            state = service.latest_state()
            if len(state.angles_deg) == len(state.temperatures_c) == 6:
                break
            time.sleep(0.05)
        else:
            raise AssertionError("complete telemetry not available")
        assert state.connected
        assert state.monotonic_s > 0
