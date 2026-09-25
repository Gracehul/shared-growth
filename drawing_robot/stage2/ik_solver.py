"""Sequential Cartesian-to-joint conversion with explicit branch seeding."""

from __future__ import annotations

from collections.abc import Callable, Sequence

import numpy as np

from .. import ik
from .config import Stage2Config
from .types import (
    CartesianTrajectory,
    JointSample,
    JointTrajectory,
    SequentialIKError,
    Severity,
    ValidationIssue,
)


IKFunction = Callable[[np.ndarray, np.ndarray], Sequence[float] | None]


class SequentialIKSolver:
    def __init__(
        self,
        config: Stage2Config | None = None,
        solve_sample: IKFunction | None = None,
    ) -> None:
        self.config = config or Stage2Config()
        self._solve_sample = solve_sample or self._default_solve

    @staticmethod
    def _default_solve(target: np.ndarray, seed: np.ndarray) -> np.ndarray:
        return ik.solve(target, np.ones(6, dtype=bool), seed)

    def _fail(self, code: str, message: str, sample_index: int) -> None:
        raise SequentialIKError(
            ValidationIssue(code, Severity.ERROR, message, sample_index=sample_index)
        )

    def solve(
        self,
        cartesian_trajectory: CartesianTrajectory,
        seed: Sequence[float],
    ) -> JointTrajectory:
        previous = np.asarray(seed, dtype=float)
        if previous.shape != (6,) or not np.isfinite(previous).all():
            self._fail("IK_SEED_INVALID", "IK seed must contain six finite joint angles", 0)

        output: list[JointSample] = []
        limits = np.asarray(self.config.joint_limits.value, dtype=float)
        for index, sample in enumerate(cartesian_trajectory.samples):
            target = np.asarray(sample.pose_mm_deg, dtype=float)
            if target.shape != (6,) or not np.isfinite(target).all():
                self._fail(
                    "CARTESIAN_SAMPLE_INVALID",
                    "Cartesian sample must contain six finite pose values",
                    index,
                )
            try:
                solution = self._solve_sample(target.copy(), previous.copy())
            except Exception as exc:
                self._fail("IK_FAILED", f"IK failed: {exc}", index)
            if solution is None:
                self._fail("IK_FAILED", "IK returned no solution", index)

            q = np.asarray(solution, dtype=float)
            if q.shape != (6,):
                self._fail("IK_JOINT_COUNT_INVALID", "IK solution must contain six joints", index)
            if not np.isfinite(q).all():
                self._fail("IK_SOLUTION_NONFINITE", "IK solution contains NaN or Inf", index)
            outside = np.flatnonzero((q < limits[:, 0]) | (q > limits[:, 1]))
            if outside.size:
                joint = int(outside[0])
                raise SequentialIKError(
                    ValidationIssue(
                        "JOINT_LIMIT_EXCEEDED",
                        Severity.ERROR,
                        f"IK solution J{joint + 1}={q[joint]:.3f} deg is outside project limits",
                        sample_index=index,
                        joint_index=joint,
                    )
                )
            output.append(JointSample(sample.time_s, q))
            previous = q
        return JointTrajectory(output)
