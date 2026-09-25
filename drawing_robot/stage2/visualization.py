"""Offline visual inspection of Stage-2 trajectories and validation output."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

from ..kinematics import forward_kinematics, frame_chain
from .config import Stage2Config
from .types import (
    CartesianTrajectory,
    JointTrajectory,
    Severity,
    ValidationIssue,
    ValidationResult,
)


class VisualizationInputError(ValueError):
    """The supplied Stage-2 artifacts cannot be displayed without repair."""


def _issue_from_dict(raw: dict[str, Any]) -> ValidationIssue:
    try:
        return ValidationIssue(
            code=str(raw["code"]),
            severity=Severity(str(raw["severity"])),
            message=str(raw.get("message", raw["code"])),
            sample_index=raw.get("sample_index"),
            joint_index=raw.get("joint_index"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise VisualizationInputError(f"invalid validation issue: {exc}") from exc


def load_visualization_bundle(
    path: str | Path,
) -> tuple[CartesianTrajectory, JointTrajectory, ValidationResult]:
    """Load the documented Stage-2B JSON interchange format."""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        cart_raw = raw["cartesian_trajectory"]
        joint_raw = raw["joint_trajectory"]
        validation_raw = raw["validation_result"]
        cartesian = CartesianTrajectory.from_arrays(
            cart_raw["timestamps_s"], cart_raw["poses_mm_deg"]
        )
        joint = JointTrajectory.from_arrays(
            joint_raw["timestamps_s"], joint_raw["positions_deg"]
        )
        errors = tuple(_issue_from_dict(item) for item in validation_raw.get("errors", []))
        warnings = tuple(
            _issue_from_dict(item) for item in validation_raw.get("warnings", [])
        )
        if not isinstance(validation_raw.get("valid"), bool):
            raise VisualizationInputError("validation_result.valid must be a JSON boolean")
        validation = ValidationResult(
            valid=validation_raw["valid"],
            errors=errors,
            warnings=warnings,
            metrics=dict(validation_raw.get("metrics", {})),
        )
    except VisualizationInputError:
        raise
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        raise VisualizationInputError(f"cannot load Stage-2B input: {exc}") from exc
    return cartesian, joint, validation


class MotionVisualizer:
    """Matplotlib view over immutable Stage-2 artifacts.

    The class performs only display-input checks. It neither validates motion
    nor changes the supplied trajectories or ValidationResult.
    """

    def __init__(
        self,
        cartesian_trajectory: CartesianTrajectory,
        joint_trajectory: JointTrajectory,
        validation_result: ValidationResult,
        config: Stage2Config | None = None,
    ) -> None:
        self.cartesian_trajectory = cartesian_trajectory
        self.joint_trajectory = joint_trajectory
        self.validation_result = validation_result
        self.config = config or Stage2Config()
        self._times, self._poses, self._joints = self._check_inputs()
        self._robot_geometry = self._build_robot_geometry()
        self.selected_index = 0
        self._build_figure()
        self.select_sample(0, redraw=False)

    @property
    def joint_series(self) -> tuple[np.ndarray, ...]:
        """Six read-only display series, one per joint."""
        return tuple(self._joints[:, index].copy() for index in range(6))

    def _check_inputs(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        carts = self.cartesian_trajectory.samples
        joints = self.joint_trajectory.samples
        if not carts or not joints:
            raise VisualizationInputError("trajectories must not be empty")
        if len(carts) != len(joints):
            raise VisualizationInputError(
                "CartesianTrajectory and JointTrajectory sample counts differ"
            )

        cart_times = np.asarray([sample.time_s for sample in carts], dtype=float)
        joint_times = np.asarray([sample.time_s for sample in joints], dtype=float)
        if not np.isfinite(cart_times).all() or not np.isfinite(joint_times).all():
            raise VisualizationInputError("trajectory timestamps must be finite")
        if np.any(np.diff(cart_times) <= 0) or np.any(np.diff(joint_times) <= 0):
            raise VisualizationInputError("trajectory timestamps must be strictly increasing")
        if not np.array_equal(cart_times, joint_times):
            raise VisualizationInputError("trajectory timestamps do not match exactly")

        try:
            poses = np.asarray([sample.pose_mm_deg for sample in carts], dtype=float)
            joint_values = np.asarray(
                [sample.positions_deg for sample in joints], dtype=float
            )
        except (TypeError, ValueError) as exc:
            raise VisualizationInputError(f"trajectory values are malformed: {exc}") from exc
        if poses.shape != (len(carts), 6):
            raise VisualizationInputError("every Cartesian sample must contain six values")
        if joint_values.shape != (len(joints), 6):
            raise VisualizationInputError("every joint sample must contain six values")
        if not np.isfinite(poses).all() or not np.isfinite(joint_values).all():
            raise VisualizationInputError("trajectory values must be finite")
        return cart_times, poses, joint_values

    def _build_robot_geometry(self) -> tuple[np.ndarray, ...]:
        geometry: list[np.ndarray] = []
        for index, q in enumerate(self._joints):
            try:
                frames = frame_chain(q)
                points = np.asarray([frame[:3, 3] for frame in frames], dtype=float)
                tool = np.asarray(forward_kinematics(q)[:3, 3], dtype=float)
                points = np.vstack((points, tool))
            except Exception as exc:
                raise VisualizationInputError(
                    f"FK display geometry failed at sample {index}: {exc}"
                ) from exc
            if points.shape != (8, 3) or not np.isfinite(points).all():
                raise VisualizationInputError(
                    f"FK display geometry is invalid at sample {index}"
                )
            geometry.append(points)
        return tuple(geometry)

    def _build_figure(self) -> None:
        self.figure = plt.figure(figsize=(14.0, 9.5))
        grid = self.figure.add_gridspec(
            2, 2, width_ratios=(1.25, 1.0), height_ratios=(1.35, 1.0)
        )
        self.ax_robot = self.figure.add_subplot(grid[:, 0], projection="3d")
        self.ax_joints = self.figure.add_subplot(grid[0, 1])
        self.ax_status = self.figure.add_subplot(grid[1, 1])
        self.figure.subplots_adjust(left=0.05, right=0.98, top=0.94, bottom=0.11, wspace=0.24)
        self._draw_spatial_view()
        self._draw_joint_view()
        self.ax_status.axis("off")
        self.status_text = self.ax_status.text(
            0.0,
            1.0,
            "",
            transform=self.ax_status.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=9.5,
        )

        slider_axis = self.figure.add_axes((0.18, 0.025, 0.64, 0.03))
        if len(self._times) > 1:
            self.slider = Slider(
                slider_axis,
                "Sample",
                valmin=0,
                valmax=len(self._times) - 1,
                valinit=0,
                valstep=1,
            )
            self.slider.on_changed(lambda value: self.select_sample(int(value)))
        else:
            self.slider = None
            slider_axis.axis("off")
            slider_axis.text(0.5, 0.5, "Sample 1 / 1", ha="center", va="center")
        self.figure.suptitle("Stage 2B — planned motion inspection")

    def _draw_spatial_view(self) -> None:
        path = self._poses[:, :3]
        self.ax_robot.plot(
            path[:, 0], path[:, 1], path[:, 2], color="tab:blue", linewidth=2, label="planned TCP path"
        )
        self.ax_robot.scatter(*path[0], marker="^", s=60, color="tab:green", label="path start")
        self.ax_robot.scatter(*path[-1], marker="s", s=50, color="tab:red", label="path end")
        points = self._robot_geometry[0]
        (self.robot_line,) = self.ax_robot.plot(
            points[:, 0], points[:, 1], points[:, 2], "-o", color="tab:orange", label="robot links"
        )
        self.current_tcp = self.ax_robot.scatter(
            *path[0], marker="o", s=80, color="black", label="selected TCP"
        )
        self.ax_robot.scatter(0, 0, 0, marker="x", s=70, color="black", label="base origin")

        all_points = np.vstack((*self._robot_geometry, path, np.zeros((1, 3))))
        low = np.min(all_points, axis=0)
        high = np.max(all_points, axis=0)
        center = (low + high) / 2.0
        span = max(float(np.max(high - low)), 20.0) * 1.12
        half = span / 2.0
        self.spatial_bounds = tuple((float(c - half), float(c + half)) for c in center)
        self.ax_robot.set_xlim(*self.spatial_bounds[0])
        self.ax_robot.set_ylim(*self.spatial_bounds[1])
        self.ax_robot.set_zlim(*self.spatial_bounds[2])
        self.ax_robot.set_box_aspect((1, 1, 1))

        axis_length = span * 0.12
        origin = np.zeros(3)
        for direction, label, color in (
            ((1, 0, 0), "X", "tab:red"),
            ((0, 1, 0), "Y", "tab:green"),
            ((0, 0, 1), "Z", "tab:blue"),
        ):
            vector = np.asarray(direction, dtype=float) * axis_length
            self.ax_robot.quiver(*origin, *vector, color=color, arrow_length_ratio=0.12)
            self.ax_robot.text(*(vector * 1.08), label, color=color)
        self.ax_robot.set_xlabel("X [mm]")
        self.ax_robot.set_ylabel("Y [mm]")
        self.ax_robot.set_zlabel("Z [mm]")
        self.ax_robot.set_title("Robot pose and Cartesian path")
        self.ax_robot.legend(loc="upper left", fontsize="small")

    def _draw_joint_view(self) -> None:
        colors = [f"C{index}" for index in range(6)]
        self.joint_lines = []
        self.joint_markers = []
        self.joint_limit_lines = []
        limits = self.config.joint_limits.value
        for index in range(6):
            (line,) = self.ax_joints.plot(
                self._times,
                self._joints[:, index],
                color=colors[index],
                label=f"J{index + 1}",
            )
            self.joint_lines.append(line)
            self.joint_limit_lines.append(
                self.ax_joints.axhline(
                    limits[index][0], color=colors[index], alpha=0.2, linewidth=0.8
                )
            )
            self.joint_limit_lines.append(
                self.ax_joints.axhline(
                    limits[index][1], color=colors[index], alpha=0.2, linewidth=0.8
                )
            )
            (marker,) = self.ax_joints.plot(
                [self._times[0]], [self._joints[0, index]], marker="o", color=colors[index]
            )
            self.joint_markers.append(marker)
        self.current_time_line = self.ax_joints.axvline(
            self._times[0], color="black", linewidth=1.2
        )
        self.ax_joints.set_xlabel("Time [s]")
        self.ax_joints.set_ylabel("Joint angle [deg]")
        self.ax_joints.set_title("Joint trajectory and Stage-2 project limits")
        self.ax_joints.grid(True, alpha=0.2)
        self.ax_joints.legend(ncol=3, fontsize="small")

    @property
    def status_label(self) -> str:
        if not self.validation_result.valid or self.validation_result.errors:
            return "INVALID"
        if self.validation_result.warnings:
            return "VALID WITH WARNINGS"
        return "VALID"

    @staticmethod
    def _metric(metrics: dict[str, Any], name: str, unit: str) -> str:
        value = metrics.get(name)
        if value is None:
            return "n/a"
        try:
            return f"{float(value):.3f} {unit}".rstrip()
        except (TypeError, ValueError):
            return str(value)

    def _sample_metric(self, name: str, index: int, unit: str) -> str:
        values = self.validation_result.metrics.get(name)
        if not isinstance(values, (list, tuple)) or index >= len(values):
            return "n/a"
        value = values[index]
        if value is None:
            return "n/a"
        return f"{float(value):.3f} {unit}".rstrip()

    def _status_block(self, index: int) -> str:
        result = self.validation_result
        metrics = result.metrics
        primary = result.errors[0] if result.errors else None
        lines = [self.status_label]
        if primary is not None:
            lines.append(primary.code)
            if primary.sample_index is not None:
                lines.append(f"Sample {primary.sample_index + 1}")
            if primary.joint_index is not None:
                lines.append(f"Joint J{primary.joint_index + 1}")
        lines.extend(
            [
                "",
                "Global",
                "------",
                "Min joint margin: "
                + self._metric(metrics, "minimum_joint_margin_deg", "deg"),
                "Max velocity:     "
                + self._metric(metrics, "max_joint_velocity_deg_s", "deg/s"),
                "Max acceleration: "
                + self._metric(metrics, "max_joint_acceleration_deg_s2", "deg/s^2"),
                "Max FK error:     "
                + self._metric(metrics, "max_fk_position_error_mm", "mm"),
                "Min sigma(J):     "
                + self._metric(metrics, "minimum_jacobian_singular_value", ""),
                "",
                "Current sample",
                "--------------",
                f"Sample:            {index + 1} / {len(self._times)}",
                f"Time:              {self._times[index]:.3f} s",
                "J1...J6 [deg]:     "
                + "  ".join(f"{value:.2f}" for value in self._joints[index]),
                "FK position error: "
                + self._sample_metric("fk_position_error_by_sample_mm", index, "mm"),
                "FK rotation error: "
                + self._sample_metric("fk_orientation_error_by_sample_deg", index, "deg"),
            ]
        )
        sample_issues = [
            issue.code
            for issue in (*result.errors, *result.warnings)
            if issue.sample_index == index
        ]
        sigma = self._sample_metric("jacobian_min_singular_value_by_sample", index, "")
        singularity = (
            "WARNING"
            if "LOW_SINGULARITY_MARGIN" in sample_issues
            else ("n/a" if sigma == "n/a" else f"OK ({sigma})")
        )
        lines.append(f"Singularity:       {singularity}")
        lines.append("Issues:            " + (", ".join(sample_issues) or "none"))
        return "\n".join(lines)

    def select_sample(self, index: int, redraw: bool = True) -> None:
        if isinstance(index, bool) or int(index) != index:
            raise VisualizationInputError("sample index must be an integer")
        index = int(index)
        if not 0 <= index < len(self._times):
            raise VisualizationInputError(
                f"sample index {index} is outside [0, {len(self._times) - 1}]"
            )
        self.selected_index = index
        if self.slider is not None and int(self.slider.val) != index:
            self.slider.eventson = False
            self.slider.set_val(index)
            self.slider.eventson = True
        if self.slider is not None:
            self.slider.valtext.set_text(f"{index + 1} / {len(self._times)}")
        points = self._robot_geometry[index]
        self.robot_line.set_data_3d(points[:, 0], points[:, 1], points[:, 2])
        tcp = self._poses[index, :3]
        self.current_tcp._offsets3d = ([tcp[0]], [tcp[1]], [tcp[2]])
        self.current_time_line.set_xdata([self._times[index], self._times[index]])
        for joint_index, marker in enumerate(self.joint_markers):
            marker.set_data(
                [self._times[index]], [self._joints[index, joint_index]]
            )
        self.status_text.set_text(self._status_block(index))
        if redraw:
            self.figure.canvas.draw_idle()

    def save(self, path: str | Path, dpi: int = 160) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        self.figure.savefig(destination, dpi=dpi, bbox_inches="tight")
        return destination

    def show(self) -> None:
        """Open the Matplotlib window; planned samples only, not simulation."""
        plt.show()
