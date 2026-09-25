from __future__ import annotations

import copy
import json
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pytest

from drawing_robot.kinematics import pose_coords
from drawing_robot.stage2 import (
    CartesianTrajectory,
    JointTrajectory,
    LimitValue,
    Stage2Config,
    TrajectoryValidator,
    ValidationIssue,
    ValidationResult,
    WorkspaceBounds,
)
from drawing_robot.stage2.types import Severity
from drawing_robot.stage2.visualization import (
    MotionVisualizer,
    VisualizationInputError,
    load_visualization_bundle,
)
import drawing_robot.stage2.visualization as visualization_module


def visual_config() -> Stage2Config:
    return replace(
        Stage2Config(),
        workspace_bounds=LimitValue(
            WorkspaceBounds((-500, 500), (-500, 500), (-200, 600)),
            "mm",
            "test_fixture",
            True,
        ),
        max_joint_velocity=LimitValue((100,) * 6, "deg/s", "test_fixture", True),
        max_joint_acceleration=LimitValue(
            (200,) * 6, "deg/s^2", "test_fixture", True
        ),
        max_joint_step_deg=LimitValue(10.0, "deg", "test_fixture", True),
        minimum_dt=LimitValue(0.01, "s", "test_fixture", True),
        max_cartesian_step_mm=LimitValue(20.0, "mm", "test_fixture", True),
        fk_position_tolerance_mm=LimitValue(0.01, "mm", "test_fixture", True),
        fk_orientation_tolerance_deg=LimitValue(
            0.01, "deg", "test_fixture", True
        ),
        singularity_warning_threshold=LimitValue(
            0.0, "mixed_jacobian_units", "test_fixture", True
        ),
        tool_transform_verified=True,
    )


def stage2_artifacts():
    qs = np.asarray(
        [
            [0.0, 60.0, -100.0, 40.0, 0.0, 0.0],
            [0.5, 60.0, -100.0, 40.0, 0.0, 0.0],
            [1.0, 60.0, -100.0, 40.0, 0.0, 0.0],
        ]
    )
    times = [0.0, 0.5, 1.0]
    cartesian = CartesianTrajectory.from_arrays(times, [pose_coords(q) for q in qs])
    joint = JointTrajectory.from_arrays(times, qs)
    config = visual_config()
    result = TrajectoryValidator(config).validate(cartesian, joint)
    assert result.valid
    return cartesian, joint, result, config


@pytest.fixture
def visualizer():
    cartesian, joint, result, config = stage2_artifacts()
    view = MotionVisualizer(cartesian, joint, result, config)
    yield view
    plt.close(view.figure)


def test_valid_synchronized_trajectories_load(visualizer) -> None:
    assert visualizer.selected_index == 0
    assert visualizer.status_label == "VALID WITH WARNINGS"
    assert len(visualizer._robot_geometry) == 3


def test_mismatched_sample_counts_fail_cleanly() -> None:
    cartesian, joint, result, config = stage2_artifacts()
    shorter = JointTrajectory(joint.samples[:-1])
    with pytest.raises(VisualizationInputError, match="sample counts differ"):
        MotionVisualizer(cartesian, shorter, result, config)


def test_empty_trajectories_fail_cleanly() -> None:
    with pytest.raises(VisualizationInputError, match="must not be empty"):
        MotionVisualizer(
            CartesianTrajectory(()),
            JointTrajectory(()),
            ValidationResult(False),
            visual_config(),
        )


def test_mismatched_timestamps_fail_cleanly() -> None:
    cartesian, joint, result, config = stage2_artifacts()
    shifted = JointTrajectory.from_arrays(
        [0.0, 0.5, 1.01], [sample.positions_deg for sample in joint.samples]
    )
    with pytest.raises(VisualizationInputError, match="timestamps do not match"):
        MotionVisualizer(cartesian, shifted, result, config)


def test_invalid_dimensions_and_nonfinite_values_fail() -> None:
    cartesian, joint, result, config = stage2_artifacts()
    bad_dimension = JointTrajectory.from_arrays([0.0], [[0, 0, 0, 0, 0]])
    one_cart = CartesianTrajectory((cartesian.samples[0],))
    with pytest.raises(VisualizationInputError, match="six values"):
        MotionVisualizer(one_cart, bad_dimension, result, config)

    bad_finite = JointTrajectory.from_arrays([0.0], [[0, 0, 0, 0, 0, np.nan]])
    with pytest.raises(VisualizationInputError, match="must be finite"):
        MotionVisualizer(one_cart, bad_finite, result, config)


def test_fk_display_geometry_is_finite_and_uses_complete_chain(visualizer) -> None:
    for points in visualizer._robot_geometry:
        assert points.shape == (8, 3)  # base, six joint frames, tool
        assert np.isfinite(points).all()
    assert visualizer.ax_robot.get_xlabel() == "X [mm]"
    assert visualizer.ax_robot.get_ylabel() == "Y [mm]"
    assert visualizer.ax_robot.get_zlabel() == "Z [mm]"


def test_nonfinite_fk_display_geometry_fails_cleanly(monkeypatch) -> None:
    cartesian, joint, result, config = stage2_artifacts()
    monkeypatch.setattr(
        visualization_module,
        "forward_kinematics",
        lambda _q: np.full((4, 4), np.nan),
    )
    with pytest.raises(VisualizationInputError, match="FK display geometry is invalid"):
        MotionVisualizer(cartesian, joint, result, config)


def test_first_and_last_samples_can_be_selected(visualizer) -> None:
    original_bounds = visualizer.spatial_bounds
    visualizer.select_sample(2)
    assert visualizer.selected_index == 2
    assert visualizer.slider.valtext.get_text() == "3 / 3"
    assert np.asarray(visualizer.current_time_line.get_xdata()).tolist() == [1.0, 1.0]
    assert visualizer.spatial_bounds == original_bounds
    visualizer.select_sample(0)
    assert visualizer.selected_index == 0


def test_all_six_joint_series_and_current_markers_are_available(visualizer) -> None:
    assert len(visualizer.joint_series) == 6
    assert len(visualizer.joint_limit_lines) == 12
    assert all(series.shape == (3,) for series in visualizer.joint_series)
    visualizer.select_sample(1)
    for joint_index, marker in enumerate(visualizer.joint_markers):
        assert marker.get_xdata()[0] == pytest.approx(0.5)
        assert marker.get_ydata()[0] == pytest.approx(
            visualizer.joint_trajectory.samples[1].positions_deg[joint_index]
        )
    assert "Sample:            2 / 3" in visualizer.status_text.get_text()


def test_invalid_validation_result_remains_explicitly_invalid() -> None:
    cartesian, joint, _, config = stage2_artifacts()
    issue = ValidationIssue(
        "JOINT_LIMIT_EXCEEDED",
        Severity.ERROR,
        "fixture failure",
        sample_index=1,
        joint_index=1,
    )
    invalid = ValidationResult(False, errors=(issue,), metrics={})
    view = MotionVisualizer(cartesian, joint, invalid, config)
    try:
        assert view.status_label == "INVALID"
        text = view.status_text.get_text()
        assert "INVALID" in text
        assert "JOINT_LIMIT_EXCEEDED" in text
        assert "Joint J2" in text
    finally:
        plt.close(view.figure)


def test_visualization_does_not_mutate_stage2_inputs() -> None:
    cartesian, joint, result, config = stage2_artifacts()
    before = copy.deepcopy((cartesian, joint, result))
    view = MotionVisualizer(cartesian, joint, result, config)
    try:
        view.select_sample(2)
        assert (cartesian, joint, result) == before
    finally:
        plt.close(view.figure)


def test_headless_figure_creation_and_export(visualizer, tmp_path) -> None:
    destination = visualizer.save(tmp_path / "stage2b.png")
    assert destination.exists()
    assert destination.stat().st_size > 0
    assert matplotlib.get_backend().lower() == "agg"


def test_json_bundle_loader_preserves_validation_decision(tmp_path) -> None:
    cartesian, joint, result, _ = stage2_artifacts()
    path = tmp_path / "trajectory.json"
    path.write_text(
        json.dumps(
            {
                "cartesian_trajectory": {
                    "timestamps_s": [s.time_s for s in cartesian.samples],
                    "poses_mm_deg": [s.pose_mm_deg for s in cartesian.samples],
                },
                "joint_trajectory": {
                    "timestamps_s": [s.time_s for s in joint.samples],
                    "positions_deg": [s.positions_deg for s in joint.samples],
                },
                "validation_result": {
                    "valid": False,
                    "errors": [
                        {
                            "code": "TEST_REJECTION",
                            "severity": "ERROR",
                            "message": "fixture",
                            "sample_index": 0,
                        }
                    ],
                    "warnings": [],
                    "metrics": result.metrics,
                },
            }
        ),
        encoding="utf-8",
    )
    loaded_cart, loaded_joint, loaded_result = load_visualization_bundle(path)
    assert loaded_cart == cartesian
    assert loaded_joint == joint
    assert loaded_result.valid is False
    assert loaded_result.errors[0].code == "TEST_REJECTION"


def test_cli_headless_export(tmp_path) -> None:
    cartesian, joint, result, _ = stage2_artifacts()
    bundle = tmp_path / "trajectory.json"
    output = tmp_path / "stage2b.png"
    bundle.write_text(
        json.dumps(
            {
                "cartesian_trajectory": {
                    "timestamps_s": [s.time_s for s in cartesian.samples],
                    "poses_mm_deg": [s.pose_mm_deg for s in cartesian.samples],
                },
                "joint_trajectory": {
                    "timestamps_s": [s.time_s for s in joint.samples],
                    "positions_deg": [s.positions_deg for s in joint.samples],
                },
                "validation_result": {
                    "valid": result.valid,
                    "errors": [],
                    "warnings": [],
                    "metrics": result.metrics,
                },
            }
        ),
        encoding="utf-8",
    )
    repository = Path(__file__).resolve().parent.parent
    completed = subprocess.run(
        [
            sys.executable,
            str(repository / "scripts" / "visualize_trajectory.py"),
            str(bundle),
            "--save",
            str(output),
            "--sample",
            "2",
        ],
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert completed.returncode == 0, completed.stderr
    assert output.exists()
    assert output.stat().st_size > 0
