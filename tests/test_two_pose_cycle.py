import pytest

from scripts.characterize_joint import GateFailure
from scripts.two_pose_cycle import UPRIGHT_DEG, maximum_error, validate_pose


def test_upright_pose_is_inside_project_margins() -> None:
    validate_pose(UPRIGHT_DEG)


def test_pose_near_hard_limit_is_rejected() -> None:
    with pytest.raises(GateFailure, match="J3"):
        validate_pose([0, 0, -149, 0, 0, 0])


def test_maximum_error_uses_all_joints() -> None:
    assert maximum_error([0, 1, 2, 3, 4, 5], [0, 1, 2, 3, 4, 7]) == 2
