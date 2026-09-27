#!/usr/bin/env python3
"""Plan one 25-second physical demo around a measured stable start pose."""

from __future__ import annotations

import argparse
from dataclasses import replace

import numpy as np

from drawing_robot.hardware import Stage4AConfig
from drawing_robot.kinematics import pose_coords
from drawing_robot.stage2 import (
    CartesianTrajectory,
    JointTrajectory,
    LimitValue,
    Stage2Config,
    TrajectoryValidator,
    WorkspaceBounds,
)


PHRASE_KNOTS = (
    (0.0, (0, 0, 0, 0, 0, 0)),
    (5.0, (0, 0, 3, -2, 2, 2)),       # open
    (13.0, (5, 0, 1, 2, 1, 4)),       # sweep
    (20.0, (2, 0, 2, 2, 1, -1)),      # small arc / turn
    (25.0, (1, 0, 2, -1, 0, 1)),      # settle
)


def build_demo(start_angles_deg, sample_period_s=0.2):
    start = np.asarray(start_angles_deg, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError("start_angles_deg must contain six finite values")
    times = np.arange(0.0, PHRASE_KNOTS[-1][0] + sample_period_s / 2, sample_period_s)
    knots = [(time_s, start + np.asarray(offset)) for time_s, offset in PHRASE_KNOTS]
    joints = []
    for time_s in times:
        for (start_s, start_q), (end_s, end_q) in zip(knots, knots[1:]):
            if start_s <= time_s <= end_s + 1e-12:
                u = (time_s - start_s) / (end_s - start_s)
                blend = 10 * u**3 - 15 * u**4 + 6 * u**5
                joints.append(start_q + (end_q - start_q) * blend)
                break
    joint_trajectory = JointTrajectory.from_arrays(times, joints)
    cartesian_trajectory = CartesianTrajectory.from_arrays(
        times, [pose_coords(angles) for angles in joints]
    )
    config = Stage4AConfig()
    stage2_config = replace(
        Stage2Config(),
        workspace_bounds=LimitValue(
            WorkspaceBounds(*config.allowed_workspace_mm),
            "mm", config.workspace_source, config.workspace_verified,
        ),
        max_joint_velocity=LimitValue((15.0,) * 6, "deg/s", "stage4a", False),
        max_joint_acceleration=LimitValue((60.0,) * 6, "deg/s^2", "stage4a", False),
        max_joint_step_deg=LimitValue(2.0, "deg", "stage4a", False),
    )
    validation = TrajectoryValidator(stage2_config).validate(
        cartesian_trajectory, joint_trajectory
    )
    return cartesian_trajectory, joint_trajectory, validation


def main():
    import matplotlib

    matplotlib.use("Agg")
    from drawing_robot.stage2.visualization import (
        MotionVisualizer,
        save_visualization_bundle,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-angles", type=float, nargs=6, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--figure", required=True)
    args = parser.parse_args()
    cartesian, joints, validation = build_demo(args.start_angles)
    save_visualization_bundle(args.output, cartesian, joints, validation)
    MotionVisualizer(cartesian, joints, validation).save(args.figure)
    print("VALID" if validation.valid else "INVALID")
    print(validation.metrics)
    return 0 if validation.valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
