import numpy as np
import pytest

from drawing_robot.motion_profiles import PROFILES, deliberate, sample_profile


@pytest.mark.parametrize("name", PROFILES)
def test_profiles_are_bounded_monotonic_and_complete(name: str) -> None:
    samples = sample_profile(name, duration_s=2.0, sample_rate_hz=25.0)
    progress = np.asarray([sample["progress"] for sample in samples])
    assert progress[0] == pytest.approx(0.0)
    assert progress[-1] == pytest.approx(1.0)
    assert np.all(progress >= 0.0)
    assert np.all(progress <= 1.0)
    assert np.all(np.diff(progress) >= -1e-12)


def test_all_profiles_share_duration_and_sample_count() -> None:
    sampled = {
        name: sample_profile(name, duration_s=3.0, sample_rate_hz=20.0)
        for name in PROFILES
    }
    assert {len(samples) for samples in sampled.values()} == {61}
    assert {samples[-1]["time_s"] for samples in sampled.values()} == {3.0}


def test_deliberate_profile_holds_before_moving() -> None:
    assert deliberate([0.0, 0.10, 0.15, 0.20], hold_fraction=0.15).tolist()[:3] == [0.0, 0.0, 0.0]
    assert deliberate([0.20], hold_fraction=0.15)[0] > 0.0


def test_invalid_profile_input_is_rejected() -> None:
    with pytest.raises(ValueError, match="within"):
        PROFILES["smooth"]([-0.1, 1.0])
