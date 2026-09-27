import numpy as np

from scripts.plan_physical_demo import PHRASE_KNOTS, build_demo


START = (1.23, 51.24, -70.75, -63.63, -3.86, 14.67)


def test_demo_is_25_seconds_stage2_valid_and_uses_five_joints():
    cartesian, joints, result = build_demo(START)
    assert result.valid
    assert joints.samples[0].time_s == 0.0
    assert joints.samples[-1].time_s == 25.0
    moved = np.ptp(np.asarray([sample.positions_deg for sample in joints.samples]), axis=0)
    assert np.count_nonzero(moved > 0.1) == 5
    assert len(cartesian.samples) == len(joints.samples) == 126


def test_demo_phrase_knots_have_open_sweep_turn_settle_structure():
    assert [item[0] for item in PHRASE_KNOTS] == [0.0, 5.0, 13.0, 20.0, 25.0]
