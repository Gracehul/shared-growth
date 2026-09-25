import math

import pytest

from drawing_robot import (
    Arm,
    DrawingController,
    DrawingError,
    DrawingWorkspace,
    branch_points,
    branch_segments,
)


@pytest.fixture
def workspace() -> DrawingWorkspace:
    # Small mock-only area around the default mock arm pose. These are not
    # physical calibration values.
    return DrawingWorkspace(
        x_min_cm=16.5,
        x_max_cm=17.5,
        y_min_cm=-7.0,
        y_max_cm=-6.0,
        drawing_z_cm=10.1,
        safe_z_cm=10.3,
    )


def test_branch_geometry_is_symmetric() -> None:
    origin, split, left, right = branch_points(
        (0.0, 0.0), (0.0, 1.0), 10.0, split_fraction=0.6, arm_angle_deg=30.0
    )

    assert origin == (0.0, 0.0)
    assert split == pytest.approx((0.0, 6.0))
    assert left[0] == pytest.approx(-right[0])
    assert left[1] == pytest.approx(right[1])
    assert math.dist(split, left) == pytest.approx(math.dist(split, right))


def test_branch_is_three_strokes() -> None:
    segments = branch_segments((1.0, 2.0), (1.0, 0.0), 4.0)
    assert len(segments) == 3
    assert segments[0][0] == (1.0, 2.0)
    assert segments[1][0] == segments[0][1]
    assert segments[2][0] == segments[0][1]


@pytest.mark.parametrize(
    "direction,length",
    [((0.0, 0.0), 1.0), ((1.0, 0.0), 0.0), ((1.0, 0.0), -1.0)],
)
def test_invalid_branch_geometry_is_rejected(direction, length) -> None:
    with pytest.raises(ValueError):
        branch_points((0.0, 0.0), direction, length)


def test_workspace_rejects_points_before_motion(workspace: DrawingWorkspace) -> None:
    with Arm(mock=True) as arm:
        controller = DrawingController(arm, workspace)
        with pytest.raises(DrawingError, match="outside"):
            controller.draw_line((16.9, -6.4), (30.0, -6.4))
        assert arm.conn.raw.commands == []


def test_mock_line_ends_at_safe_height(workspace: DrawingWorkspace) -> None:
    with Arm(mock=True, control_rate_hz=100.0) as arm:
        controller = DrawingController(arm, workspace, speed_cm_s=5.0)
        controller.draw_line((16.9, -6.4), (17.1, -6.4))

        assert arm.conn.raw.commands
        assert arm.get_coords()[2] == pytest.approx(workspace.safe_z_cm, abs=0.05)


def test_failed_command_stops_without_guessing_recovery(
    workspace: DrawingWorkspace, monkeypatch: pytest.MonkeyPatch
) -> None:
    with Arm(mock=True) as arm:
        controller = DrawingController(arm, workspace)
        stopped: list[bool] = []
        monkeypatch.setattr(arm, "send_coords", lambda **_kwargs: 0)
        monkeypatch.setattr(arm, "stop", lambda: stopped.append(True))
        arm.last_error = "simulated command failure"

        with pytest.raises(DrawingError, match="simulated command failure"):
            controller.draw_line((16.9, -6.4), (17.1, -6.4))

        assert stopped == [True]


def test_branch_is_fully_validated_before_first_motion(workspace: DrawingWorkspace) -> None:
    with Arm(mock=True) as arm:
        controller = DrawingController(arm, workspace)
        with pytest.raises(DrawingError, match="outside"):
            controller.draw_branch((17.4, -6.4), (1.0, 0.0), 1.0)
        assert arm.conn.raw.commands == []
