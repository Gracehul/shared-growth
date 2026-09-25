import numpy as np

from scripts.cartesian_s_curve import (
    HALF_HEIGHT_MM,
    HALF_WIDTH_MM,
    build_s_curve_plan,
)


REST = [0.7, 110.21, -145.63, -41.48, -19.77, 14.67]


def test_s_curve_plan_is_safe_and_executable() -> None:
    plan = build_s_curve_plan(REST)
    assert plan["duration_s"] > 0
    assert len(plan["q_waypoints_deg"]) > 20
    assert plan["worst_manipulability"] > 1_000
    assert plan["minimum_link_z_mm"] >= 100


def test_s_curve_has_requested_xz_extent_and_constant_y() -> None:
    plan = build_s_curve_plan(REST)
    targets = np.asarray(plan["cartesian_targets_mm"])
    assert np.ptp(targets[:, 0]) == 2 * HALF_WIDTH_MM
    assert np.ptp(targets[:, 2]) == 2 * HALF_HEIGHT_MM
    assert np.ptp(targets[:, 1]) == 0
