"""Offline validation gate for sampled Cartesian and joint trajectories."""

from __future__ import annotations

from collections.abc import Callable, Sequence

import numpy as np

from ..kinematics import coord_error, pose_coords, task_jacobian
from .config import Stage2Config
from .types import (
    CartesianTrajectory,
    JointTrajectory,
    Severity,
    ValidationIssue,
    ValidationResult,
)


FKFunction = Callable[[Sequence[float]], np.ndarray]
JacobianFunction = Callable[[np.ndarray], np.ndarray]


class TrajectoryValidator:
    """Validate samples only; it makes no claim about continuous-time safety."""

    def __init__(
        self,
        config: Stage2Config | None = None,
        fk: FKFunction = pose_coords,
        jacobian: JacobianFunction = task_jacobian,
    ) -> None:
        self.config = config or Stage2Config()
        self._fk = fk
        self._jacobian = jacobian

    def validate(
        self,
        cartesian_trajectory: CartesianTrajectory,
        joint_trajectory: JointTrajectory,
    ) -> ValidationResult:
        issues: list[ValidationIssue] = []
        metrics: dict[str, object] = {
            "sample_count_cartesian": len(cartesian_trajectory.samples),
            "sample_count_joint": len(joint_trajectory.samples),
            "units": {
                "position": "mm",
                "joint_angle": "deg",
                "velocity": "deg/s",
                "acceleration": "deg/s^2",
                "time": "s",
            },
            "unverified_assumptions": list(self.config.assumptions()),
        }

        def add(
            code: str,
            severity: Severity,
            message: str,
            sample_index: int | None = None,
            joint_index: int | None = None,
        ) -> None:
            issues.append(
                ValidationIssue(code, severity, message, sample_index, joint_index)
            )

        if self.config.assumptions():
            add(
                "UNVERIFIED_PROJECT_LIMITS",
                Severity.WARNING,
                "Unverified project assumptions: " + ", ".join(self.config.assumptions()),
            )
        if not self.config.tool_transform_verified:
            add(
                "TOOL_TRANSFORM_PROVISIONAL",
                Severity.WARNING,
                "The tool transform is provisional and the known approximately 11 mm "
                "Python-vs-firmware FK discrepancy is not corrected; FK consistency "
                "is model-internal only",
            )

        carts = cartesian_trajectory.samples
        joints = joint_trajectory.samples
        if not carts or not joints:
            add("TRAJECTORY_EMPTY", Severity.ERROR, "Both trajectories require at least one sample")
        if len(carts) != len(joints):
            add(
                "SAMPLE_COUNT_MISMATCH",
                Severity.ERROR,
                "Cartesian and joint trajectories have different sample counts",
            )

        cart_values: list[np.ndarray | None] = []
        joint_values: list[np.ndarray | None] = []
        for index, sample in enumerate(carts):
            value = np.asarray(sample.pose_mm_deg, dtype=float)
            if value.shape != (6,) or not np.isfinite(value).all():
                add(
                    "CARTESIAN_SAMPLE_INVALID",
                    Severity.ERROR,
                    "Cartesian sample must contain six finite pose values",
                    index,
                )
                cart_values.append(None)
            else:
                cart_values.append(value)
        for index, sample in enumerate(joints):
            value = np.asarray(sample.positions_deg, dtype=float)
            if value.shape != (6,) or not np.isfinite(value).all():
                add(
                    "JOINT_SAMPLE_INVALID",
                    Severity.ERROR,
                    "Joint sample must contain six finite angles",
                    index,
                )
                joint_values.append(None)
            else:
                joint_values.append(value)

        cart_times = np.asarray([s.time_s for s in carts], dtype=float)
        joint_times = np.asarray([s.time_s for s in joints], dtype=float)
        cart_timing_ok = self._check_timestamps(cart_times, "Cartesian", add)
        joint_timing_ok = self._check_timestamps(joint_times, "Joint", add)

        if len(carts) == len(joints):
            for index, (tc, tj) in enumerate(zip(cart_times, joint_times)):
                if np.isfinite(tc) and np.isfinite(tj) and not np.isclose(tc, tj, atol=1e-12):
                    add(
                        "TIMESTAMP_MISMATCH",
                        Severity.ERROR,
                        f"Cartesian time {tc:g} s differs from joint time {tj:g} s",
                        index,
                    )

        self._check_workspace_and_cartesian_spacing(cart_values, add, metrics)
        self._check_joint_limits(joint_values, add, metrics)
        self._check_joint_steps(joint_values, add, metrics)

        if joint_timing_ok:
            self._check_dynamics(joint_values, joint_times, add, metrics)

        paired = min(len(cart_values), len(joint_values))
        self._check_fk_and_singularity(
            cart_values[:paired], joint_values[:paired], add, metrics
        )

        # This metric helps distinguish invalid time data from a zero-duration motion.
        if cart_timing_ok and len(cart_times) > 1:
            metrics["duration_s"] = float(cart_times[-1] - cart_times[0])
        return ValidationResult.from_issues(issues, metrics)

    def _check_timestamps(self, times, label, add) -> bool:
        ok = True
        for index, timestamp in enumerate(times):
            if not np.isfinite(timestamp):
                add(
                    "TIMESTAMP_NONFINITE",
                    Severity.ERROR,
                    f"{label} timestamp must be finite",
                    index,
                )
                ok = False
        if not np.isfinite(times).all():
            return False
        for index, dt in enumerate(np.diff(times), start=1):
            if dt <= 0.0:
                add(
                    "TIMESTAMP_NOT_INCREASING",
                    Severity.ERROR,
                    f"{label} timestamps must be strictly increasing (dt={dt:g} s)",
                    index,
                )
                ok = False
            elif dt < self.config.minimum_dt.value:
                add(
                    "TIMESTEP_TOO_SMALL",
                    Severity.ERROR,
                    f"{label} dt={dt:g} s is below {self.config.minimum_dt.value:g} s",
                    index,
                )
                ok = False
        return ok

    def _check_workspace_and_cartesian_spacing(self, values, add, metrics) -> None:
        workspace = self.config.workspace_bounds
        if workspace.value is None:
            add(
                "WORKSPACE_UNDEFINED",
                Severity.ERROR,
                "No workspace bounds are configured, so workspace validity cannot be established",
            )
        elif not workspace.verified:
            add(
                "WORKSPACE_PROVISIONAL",
                Severity.WARNING,
                "Workspace bounds are provisional",
            )

        max_step = 0.0
        worst_step_sample = None
        previous = None
        for index, value in enumerate(values):
            if value is None:
                previous = None
                continue
            if workspace.value is not None and not workspace.value.contains(*value[:3]):
                add(
                    "WORKSPACE_EXCEEDED",
                    Severity.ERROR,
                    f"Cartesian position {value[:3].tolist()} mm is outside workspace bounds",
                    index,
                )
            if previous is not None:
                step = float(np.linalg.norm(value[:3] - previous[:3]))
                if step > max_step:
                    max_step, worst_step_sample = step, index
                if step > self.config.max_cartesian_step_mm.value:
                    add(
                        "CARTESIAN_STEP_EXCEEDED",
                        Severity.ERROR,
                        f"Cartesian step {step:.3f} mm exceeds configured maximum",
                        index,
                    )
            previous = value
        metrics["max_cartesian_step_mm"] = max_step
        metrics["max_cartesian_step_sample"] = worst_step_sample

    def _check_joint_limits(self, values, add, metrics) -> None:
        limits = np.asarray(self.config.joint_limits.value, dtype=float)
        minimum_margin = np.inf
        worst_joint = None
        worst_sample = None
        for sample_index, value in enumerate(values):
            if value is None:
                continue
            margins = np.minimum(value - limits[:, 0], limits[:, 1] - value)
            local_joint = int(np.argmin(margins))
            if margins[local_joint] < minimum_margin:
                minimum_margin = float(margins[local_joint])
                worst_joint = local_joint
                worst_sample = sample_index
            for joint_index in np.flatnonzero(margins < 0.0):
                add(
                    "JOINT_LIMIT_EXCEEDED",
                    Severity.ERROR,
                    f"J{joint_index + 1}={value[joint_index]:.3f} deg is outside project limits",
                    sample_index,
                    int(joint_index),
                )
        metrics["minimum_joint_margin_deg"] = (
            None if not np.isfinite(minimum_margin) else minimum_margin
        )
        metrics["minimum_joint_margin_joint_index"] = worst_joint
        metrics["minimum_joint_margin_sample_index"] = worst_sample

    def _check_joint_steps(self, values, add, metrics) -> None:
        maximum = 0.0
        worst_joint = None
        worst_sample = None
        for index in range(1, len(values)):
            if values[index - 1] is None or values[index] is None:
                continue
            steps = np.abs(values[index] - values[index - 1])
            joint = int(np.argmax(steps))
            if steps[joint] > maximum:
                maximum, worst_joint, worst_sample = float(steps[joint]), joint, index
            for joint_index in np.flatnonzero(
                steps > self.config.max_joint_step_deg.value
            ):
                add(
                    "JOINT_DISCONTINUITY",
                    Severity.ERROR,
                    f"J{joint_index + 1} step {steps[joint_index]:.3f} deg exceeds configured maximum",
                    index,
                    int(joint_index),
                )
        metrics["max_joint_step_deg"] = maximum
        metrics["max_joint_step_joint_index"] = worst_joint
        metrics["max_joint_step_sample_index"] = worst_sample

    def _check_dynamics(self, values, times, add, metrics) -> None:
        velocities: list[np.ndarray | None] = []
        velocity_limits = np.asarray(self.config.max_joint_velocity.value, dtype=float)
        acceleration_limits = np.asarray(
            self.config.max_joint_acceleration.value, dtype=float
        )
        max_velocity = 0.0
        max_velocity_joint = None
        max_velocity_sample = None
        for index in range(1, len(values)):
            if values[index - 1] is None or values[index] is None:
                velocities.append(None)
                continue
            velocity = (values[index] - values[index - 1]) / (times[index] - times[index - 1])
            velocities.append(velocity)
            magnitude = np.abs(velocity)
            joint = int(np.argmax(magnitude))
            if magnitude[joint] > max_velocity:
                max_velocity = float(magnitude[joint])
                max_velocity_joint = joint
                max_velocity_sample = index
            for joint_index in np.flatnonzero(magnitude > velocity_limits):
                add(
                    "VELOCITY_EXCEEDED",
                    Severity.ERROR,
                    f"J{joint_index + 1} velocity {magnitude[joint_index]:.3f} deg/s exceeds project limit",
                    index,
                    int(joint_index),
                )
        metrics["max_joint_velocity_deg_s"] = max_velocity
        metrics["max_joint_velocity_joint_index"] = max_velocity_joint
        metrics["max_joint_velocity_sample_index"] = max_velocity_sample

        max_acceleration = 0.0
        max_acceleration_joint = None
        max_acceleration_sample = None
        for interval in range(1, len(velocities)):
            before, after = velocities[interval - 1], velocities[interval]
            if before is None or after is None:
                continue
            # Velocities live at interval midpoints. This denominator therefore
            # remains correct for non-uniform sample spacing.
            midpoint_dt = 0.5 * (
                (times[interval] - times[interval - 1])
                + (times[interval + 1] - times[interval])
            )
            acceleration = (after - before) / midpoint_dt
            magnitude = np.abs(acceleration)
            joint = int(np.argmax(magnitude))
            sample_index = interval + 1
            if magnitude[joint] > max_acceleration:
                max_acceleration = float(magnitude[joint])
                max_acceleration_joint = joint
                max_acceleration_sample = sample_index
            for joint_index in np.flatnonzero(magnitude > acceleration_limits):
                add(
                    "ACCELERATION_EXCEEDED",
                    Severity.ERROR,
                    f"J{joint_index + 1} acceleration {magnitude[joint_index]:.3f} deg/s^2 exceeds project limit",
                    sample_index,
                    int(joint_index),
                )
        metrics["max_joint_acceleration_deg_s2"] = max_acceleration
        metrics["max_joint_acceleration_joint_index"] = max_acceleration_joint
        metrics["max_joint_acceleration_sample_index"] = max_acceleration_sample

    def _check_fk_and_singularity(self, carts, joints, add, metrics) -> None:
        max_position_error = 0.0
        max_position_sample = None
        max_orientation_error = 0.0
        max_orientation_sample = None
        minimum_sigma = np.inf
        minimum_sigma_sample = None
        for index, (target, q) in enumerate(zip(carts, joints)):
            if target is None or q is None:
                continue
            try:
                reconstructed = np.asarray(self._fk(q), dtype=float)
                if reconstructed.shape != (6,) or not np.isfinite(reconstructed).all():
                    raise ValueError("FK did not return six finite pose values")
                error = coord_error(reconstructed, target)
                position_error = float(np.linalg.norm(error[:3]))
                orientation_error = float(np.max(np.abs(error[3:])))
            except Exception as exc:
                add("FK_NUMERICAL_FAILURE", Severity.ERROR, f"FK failed: {exc}", index)
            else:
                if position_error > max_position_error:
                    max_position_error, max_position_sample = position_error, index
                if orientation_error > max_orientation_error:
                    max_orientation_error, max_orientation_sample = orientation_error, index
                if position_error > self.config.fk_position_tolerance_mm.value:
                    add(
                        "FK_POSITION_ERROR",
                        Severity.ERROR,
                        f"FK position error {position_error:.3f} mm exceeds tolerance",
                        index,
                    )
                if orientation_error > self.config.fk_orientation_tolerance_deg.value:
                    add(
                        "FK_ORIENTATION_ERROR",
                        Severity.ERROR,
                        f"FK orientation error {orientation_error:.3f} deg exceeds tolerance",
                        index,
                    )

            try:
                jacobian = np.asarray(self._jacobian(q), dtype=float)
                if jacobian.shape != (6, 6) or not np.isfinite(jacobian).all():
                    raise ValueError("Jacobian must be a finite 6x6 matrix")
                sigma = float(np.min(np.linalg.svd(jacobian, compute_uv=False)))
            except Exception as exc:
                add(
                    "SINGULARITY_NUMERICAL_FAILURE",
                    Severity.ERROR,
                    f"Jacobian diagnostic failed: {exc}",
                    index,
                )
            else:
                if sigma < minimum_sigma:
                    minimum_sigma, minimum_sigma_sample = sigma, index
                if sigma < self.config.singularity_warning_threshold.value:
                    add(
                        "LOW_SINGULARITY_MARGIN",
                        Severity.WARNING,
                        f"Jacobian minimum singular value {sigma:.6g} is below warning threshold",
                        index,
                    )
        metrics["max_fk_position_error_mm"] = max_position_error
        metrics["max_fk_position_error_sample_index"] = max_position_sample
        metrics["max_fk_orientation_error_deg"] = max_orientation_error
        metrics["max_fk_orientation_error_sample_index"] = max_orientation_sample
        metrics["minimum_jacobian_singular_value"] = (
            None if not np.isfinite(minimum_sigma) else minimum_sigma
        )
        metrics["minimum_jacobian_singular_value_sample_index"] = minimum_sigma_sample
