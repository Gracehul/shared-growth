"""Normalized temporal profiles for offline motion-design experiments.

The functions map normalized time ``u`` in ``[0, 1]`` to normalized path
progress in ``[0, 1]``. Geometry and total duration stay fixed, allowing timing
style to be compared without changing the spatial path.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np


Profile = Callable[[np.ndarray], np.ndarray]


def _unit_interval(u) -> np.ndarray:
    values = np.asarray(u, dtype=float)
    if np.any(~np.isfinite(values)) or np.any(values < 0.0) or np.any(values > 1.0):
        raise ValueError("normalized time must be finite and within [0, 1]")
    return values


def functional(u) -> np.ndarray:
    """Linear baseline: constant normalized path speed."""
    return _unit_interval(u)


def smooth(u) -> np.ndarray:
    """Quintic ease-in/ease-out with zero endpoint velocity/acceleration."""
    values = _unit_interval(u)
    return values**3 * (10.0 + values * (-15.0 + 6.0 * values))


def deliberate(u, hold_fraction: float = 0.15) -> np.ndarray:
    """Brief pre-movement hold followed by a smooth complete trajectory."""
    values = _unit_interval(u)
    if not 0.0 <= hold_fraction < 1.0:
        raise ValueError("hold_fraction must be within [0, 1)")
    active = np.clip((values - hold_fraction) / (1.0 - hold_fraction), 0.0, 1.0)
    return smooth(active)


def early_commitment(u) -> np.ndarray:
    """Ease-out profile that exposes the selected direction early."""
    values = _unit_interval(u)
    return 1.0 - (1.0 - values) ** 3


PROFILES: dict[str, Profile] = {
    "functional": functional,
    "smooth": smooth,
    "deliberate": deliberate,
    "early_commitment": early_commitment,
}


def sample_profile(name: str, duration_s: float, sample_rate_hz: float) -> list[dict[str, float]]:
    """Sample one profile including exact start and end points."""
    if name not in PROFILES:
        raise ValueError(f"unknown profile {name!r}; choose from {sorted(PROFILES)}")
    if duration_s <= 0 or sample_rate_hz <= 0:
        raise ValueError("duration and sample rate must be positive")
    count = max(2, int(round(duration_s * sample_rate_hz)) + 1)
    times = np.linspace(0.0, duration_s, count)
    progress = PROFILES[name](times / duration_s)
    return [
        {"time_s": float(timestamp), "progress": float(position)}
        for timestamp, position in zip(times, progress, strict=True)
    ]
