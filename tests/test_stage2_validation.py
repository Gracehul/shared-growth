from dataclasses import replace

import numpy as np
import pytest

from drawing_robot.kinematics import pose_coords
from drawing_robot.stage2 import (
    CartesianTrajectory,
    JointTrajectory,
    LimitValue,
    SequentialIKError,
    SequentialIKSolver,
    Stage2Config,
    TrajectoryValidator,
    WorkspaceBounds,
)


def configured(**changes):
    base = Stage2Config()
    defaults = {
        "workspace_bounds": LimitValue(
            WorkspaceBounds((-500, 500), (-500, 500), (-200, 600)),
            "mm",
            "test_fixture",
            True,
        ),
        "max_joint_velocity": LimitValue((100,) * 6, "deg/s", "test_fixture", True),
        "max_joint_acceleration": LimitValue(
            (200,) * 6, "deg/s^2", "test_fixture", True
        ),
        "max_joint_step_deg": LimitValue(10.0, "deg", "test_fixture", True),
        "minimum_dt": LimitValue(0.01, "s", "test_fixture", True),
        "max_cartesian_step_mm": LimitValue(20.0, "mm", "test_fixture", True),
        "fk_position_tolerance_mm": LimitValue(0.01, "mm", "test_fixture", True),
        "fk_orientation_tolerance_deg": LimitValue(
            0.01, "deg", "test_fixture", True
        ),
        "singularity_warning_threshold": LimitValue(
            0.0, "mixed_jacobian_units", "test_fixture", True
        ),
        "tool_transform_verified": True,
    }
    defaults.update(changes)
    return replace(base, **defaults)


def trajectories(qs, times=None, poses=None):
    times = list(range(len(qs))) if times is None else times
    poses = [pose_coords(q) for q in qs] if poses is None else poses
    return (
        CartesianTrajectory.from_arrays(times, poses),
        JointTrajectory.from_arrays(times, qs),
    )


def codes(result):
    return {issue.code for issue in (*result.errors, *result.warnings)}


def test_valid_small_trajectory() -> None:
    qs = [[0, 60, -100, 40, 0, 0], [0.5, 60, -100, 40, 0, 0]]
    cart, joint = trajectories(qs)
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert result.valid
    assert result.errors == ()
    assert result.metrics["max_joint_step_deg"] == pytest.approx(0.5)
    assert result.metrics["minimum_joint_margin_deg"] > 0


def test_joint_limit_violation_reports_location() -> None:
    qs = [[0, 0, 0, 0, 0, 0], [166, 0, 0, 0, 0, 0]]
    cart, joint = trajectories(qs)
    result = TrajectoryValidator(configured()).validate(cart, joint)
    issue = next(i for i in result.errors if i.code == "JOINT_LIMIT_EXCEEDED")
    assert not result.valid
    assert (issue.sample_index, issue.joint_index) == (1, 0)
    assert result.metrics["minimum_joint_margin_deg"] == pytest.approx(-1.0)


def test_joint_discontinuity_and_branch_jump() -> None:
    qs = [[0, 30, -60, 30, 0, 0], [0, 30, -60, 30, 120, 0]]
    cart, joint = trajectories(qs)
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert "JOINT_DISCONTINUITY" in codes(result)


def test_excessive_velocity() -> None:
    qs = [[0, 30, -60, 30, 0, 0], [2, 30, -60, 30, 0, 0]]
    cart, joint = trajectories(qs, times=[0.0, 0.1])
    cfg = configured(
        max_joint_velocity=LimitValue((10,) * 6, "deg/s", "test_fixture", True)
    )
    result = TrajectoryValidator(cfg).validate(cart, joint)
    assert "VELOCITY_EXCEEDED" in codes(result)
    assert result.metrics["max_joint_velocity_deg_s"] == pytest.approx(20.0)


def test_excessive_acceleration() -> None:
    qs = [[0, 30, -60, 30, 0, 0], [1, 30, -60, 30, 0, 0], [4, 30, -60, 30, 0, 0]]
    cart, joint = trajectories(qs, times=[0.0, 1.0, 2.0])
    cfg = configured(
        max_joint_acceleration=LimitValue((1,) * 6, "deg/s^2", "test_fixture", True)
    )
    result = TrajectoryValidator(cfg).validate(cart, joint)
    assert "ACCELERATION_EXCEEDED" in codes(result)
    assert result.metrics["max_joint_acceleration_deg_s2"] == pytest.approx(2.0)


@pytest.mark.parametrize(
    ("times", "expected"),
    [
        ([0.0, 0.0], "TIMESTAMP_NOT_INCREASING"),
        ([1.0, 0.0], "TIMESTAMP_NOT_INCREASING"),
        ([0.0, float("nan")], "TIMESTAMP_NONFINITE"),
        ([0.0, float("inf")], "TIMESTAMP_NONFINITE"),
    ],
)
def test_invalid_timestamps(times, expected) -> None:
    qs = [[0, 30, -60, 30, 0, 0]] * 2
    cart, joint = trajectories(qs, times=times)
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert expected in codes(result)
    assert not result.valid


def test_nonuniform_timestamp_spacing_uses_interval_midpoints() -> None:
    # v1=1 deg/s at midpoint 0.5; v2=3 deg/s at midpoint 2.0.
    # a=(3-1)/(2.0-0.5)=1.333... deg/s^2.
    qs = [[0, 30, -60, 30, 0, 0], [1, 30, -60, 30, 0, 0], [4, 30, -60, 30, 0, 0]]
    cart, joint = trajectories(qs, times=[0.0, 1.0, 2.0])
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert result.metrics["max_joint_acceleration_deg_s2"] == pytest.approx(2.0)

    cart, joint = trajectories(qs, times=[0.0, 1.0, 3.0])
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert result.metrics["max_joint_acceleration_deg_s2"] == pytest.approx(1 / 3)


def test_nonfinite_pose_and_joint_are_rejected() -> None:
    q = [0, 30, -60, 30, 0, 0]
    poses = [pose_coords(q), [np.nan, 0, 0, 0, 0, 0]]
    cart, joint = trajectories([q, [0, 30, np.inf, 30, 0, 0]], poses=poses)
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert {"CARTESIAN_SAMPLE_INVALID", "JOINT_SAMPLE_INVALID"} <= codes(result)


def test_fk_tolerance_violation() -> None:
    q = [0, 30, -60, 30, 0, 0]
    target = pose_coords(q)
    target[0] += 1.0
    cart, joint = trajectories([q], poses=[target])
    result = TrajectoryValidator(configured()).validate(cart, joint)
    assert "FK_POSITION_ERROR" in codes(result)


def test_workspace_and_cartesian_sample_spacing() -> None:
    qs = [[0, 30, -60, 30, 0, 0], [0, 30, -60, 30, 0, 0]]
    poses = [np.zeros(6), np.array([25, 0, 0, 0, 0, 0])]
    cart, joint = trajectories(qs, poses=poses)
    cfg = configured(
        workspace_bounds=LimitValue(
            WorkspaceBounds((-10, 10), (-10, 10), (-10, 10)),
            "mm",
            "test_fixture",
            True,
        ),
        max_cartesian_step_mm=LimitValue(5.0, "mm", "test_fixture", True),
        fk_position_tolerance_mm=LimitValue(1000, "mm", "test_fixture", True),
        fk_orientation_tolerance_deg=LimitValue(180, "deg", "test_fixture", True),
    )
    result = TrajectoryValidator(cfg).validate(cart, joint)
    assert {"WORKSPACE_EXCEEDED", "CARTESIAN_STEP_EXCEEDED"} <= codes(result)


def test_provisional_workspace_and_tool_are_warnings_only() -> None:
    q = [0, 30, -60, 30, 0, 0]
    cart, joint = trajectories([q])
    cfg = replace(
        configured(),
        workspace_bounds=LimitValue(
            WorkspaceBounds((-500, 500), (-500, 500), (-200, 600)),
            "mm",
            "project_assumption",
            False,
        ),
        tool_transform_verified=False,
    )
    result = TrajectoryValidator(cfg).validate(cart, joint)
    assert result.valid
    assert {"WORKSPACE_PROVISIONAL", "TOOL_TRANSFORM_PROVISIONAL"} <= codes(result)


def test_missing_workspace_rejects_instead_of_inventing_bounds() -> None:
    q = [0, 30, -60, 30, 0, 0]
    cart, joint = trajectories([q])
    cfg = replace(configured(), workspace_bounds=Stage2Config().workspace_bounds)
    result = TrajectoryValidator(cfg).validate(cart, joint)
    assert not result.valid
    assert "WORKSPACE_UNDEFINED" in codes(result)


def test_numerical_jacobian_failure_is_error() -> None:
    q = [0, 30, -60, 30, 0, 0]
    cart, joint = trajectories([q])
    bad_jacobian = lambda _q: np.full((6, 6), np.nan)
    result = TrajectoryValidator(configured(), jacobian=bad_jacobian).validate(cart, joint)
    assert "SINGULARITY_NUMERICAL_FAILURE" in codes(result)


def test_low_singularity_is_warning_only() -> None:
    q = [0, 30, -60, 30, 0, 0]
    cart, joint = trajectories([q])
    zero_jacobian = lambda _q: np.zeros((6, 6))
    cfg = replace(
        configured(),
        singularity_warning_threshold=LimitValue(
            0.1, "mixed_jacobian_units", "test_fixture", True
        ),
    )
    result = TrajectoryValidator(cfg, jacobian=zero_jacobian).validate(cart, joint)
    assert result.valid
    assert "LOW_SINGULARITY_MARGIN" in codes(result)


def test_sequential_ik_uses_previous_solution_as_seed() -> None:
    targets = [[0, 0, 0, 0, 0, 0]] * 3
    cart = CartesianTrajectory.from_arrays([0, 1, 2], targets)
    received_seeds = []

    def solver(_target, seed):
        received_seeds.append(seed.copy())
        return seed + np.array([1, 0, 0, 0, 0, 0])

    result = SequentialIKSolver(configured(), solver).solve(cart, seed=[0] * 6)
    assert received_seeds[0][0] == 0
    assert received_seeds[1][0] == 1
    assert received_seeds[2][0] == 2
    assert result.samples[-1].positions_deg[0] == 3


@pytest.mark.parametrize(
    ("solver", "expected"),
    [
        (lambda _target, _seed: None, "IK_FAILED"),
        (lambda _target, _seed: [0] * 5, "IK_JOINT_COUNT_INVALID"),
        (lambda _target, _seed: [0, 0, 0, 0, 0, np.nan], "IK_SOLUTION_NONFINITE"),
        (lambda _target, _seed: [166, 0, 0, 0, 0, 0], "JOINT_LIMIT_EXCEEDED"),
    ],
)
def test_sequential_ik_rejects_invalid_solutions(solver, expected) -> None:
    cart = CartesianTrajectory.from_arrays([0], [[0] * 6])
    with pytest.raises(SequentialIKError) as caught:
        SequentialIKSolver(configured(), solver).solve(cart, seed=[0] * 6)
    assert caught.value.issue.code == expected


def test_cartesian_s_curve_runs_through_sequential_ik_and_validation() -> None:
    q0 = np.array([0.0, 60.0, -100.0, 40.0, 0.0, 0.0])
    center = pose_coords(q0)
    times = np.linspace(0.0, 2.0, 9)
    poses = []
    for u in np.linspace(0.0, 1.0, 9):
        smooth = 3 * u**2 - 2 * u**3
        pose = center.copy()
        pose[0] += 4.0 * smooth
        pose[2] += 1.5 * np.sin(np.pi * smooth)
        poses.append(pose)
    cart = CartesianTrajectory.from_arrays(times, poses)
    cfg = configured(
        max_cartesian_step_mm=LimitValue(3.0, "mm", "test_fixture", True),
        max_joint_step_deg=LimitValue(10.0, "deg", "test_fixture", True),
        fk_position_tolerance_mm=LimitValue(1.0, "mm", "test_fixture", True),
        fk_orientation_tolerance_deg=LimitValue(0.5, "deg", "test_fixture", True),
    )
    joint = SequentialIKSolver(cfg).solve(cart, seed=q0)
    result = TrajectoryValidator(cfg).validate(cart, joint)
    assert result.valid, [issue.code for issue in result.errors]
