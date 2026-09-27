"""Compute threshold evidence from existing versioned execution logs."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..execution import ExecutionLog


@dataclass(frozen=True)
class UartTimingSummary:
    count: int
    mean_duration_ms: float | None
    max_duration_ms: float | None
    mean_queue_delay_ms: float | None
    max_queue_delay_ms: float | None


@dataclass(frozen=True)
class LogEvidence:
    run_id: str
    trajectory_id: str
    schema_version: str
    hardware_run: bool
    safety_profile: str | None
    telemetry_profile: dict[str, Any]
    provenance: dict[str, Any]
    ready_endpoint_error_deg: float | None
    probe_endpoint_error_deg: float | None
    mean_tracking_error_deg: float | None
    max_tracking_error_deg: float | None
    settling_duration_s: float | None
    mean_state_age_s: float | None
    max_state_age_s: float | None
    uart_by_operation: dict[str, UartTimingSummary]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _as_log(value: ExecutionLog | str | Path) -> ExecutionLog:
    return value if isinstance(value, ExecutionLog) else ExecutionLog.read(value)


def _phase_endpoint_error(log: ExecutionLog, keyword: str) -> float | None:
    for phase in reversed(log.phases):
        if keyword not in str(phase.get("phase", "")).lower():
            continue
        for key in ("max_final_error_deg", "max_target_error_deg", "final_error_deg"):
            value = phase.get(key)
            if value is not None:
                return float(value)
        per_joint = phase.get("final_error_deg_per_joint")
        if per_joint:
            return float(np.max(np.abs(np.asarray(per_joint, dtype=float))))
        samples = phase.get("settling_samples") or ()
        if samples:
            value = samples[-1].get("max_target_error_deg")
            if value is not None:
                return float(value)
    return None


def _tracking_errors(log: ExecutionLog) -> np.ndarray:
    values = []
    for state in log.states:
        target = np.asarray(state.get("commanded_angles_deg", ()), dtype=float)
        actual = np.asarray(state.get("actual_angles_deg", ()), dtype=float)
        if target.shape == actual.shape == (6,) and np.isfinite(target).all() and np.isfinite(actual).all():
            values.extend(np.abs(target - actual).tolist())
    return np.asarray(values, dtype=float)


def _settling_duration(log: ExecutionLog) -> float | None:
    settling_phases = [
        phase for phase in log.phases
        if "settling" in str(phase.get("phase", "")).lower()
    ]
    if settling_phases:
        for key in ("duration_s", "settling_duration_s"):
            if settling_phases[-1].get(key) is not None:
                return float(settling_phases[-1][key])
    if str(log.result.get("status", "")).lower() not in {"completed", "passed"}:
        return None
    if not log.commands or "timestamp_s" not in log.result:
        return None
    last_send = log.commands[-1].get("send_end_s", log.commands[-1].get("send_time_s"))
    if last_send is None:
        return None
    return max(0.0, float(log.result["timestamp_s"]) - float(last_send))


def _uart_metrics(log: ExecutionLog) -> dict[str, UartTimingSummary]:
    grouped: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for item in log.uart_transactions:
        start = item.get("io_start_s")
        end = item.get("io_end_s")
        queued = item.get("queued_at_s")
        if start is None or end is None:
            continue
        grouped[str(item.get("method", "unknown"))].append(
            (1000.0 * (float(end) - float(start)),
             1000.0 * (float(start) - float(queued)) if queued is not None else float("nan"))
        )
    result = {}
    for operation, values in grouped.items():
        durations = np.asarray([value[0] for value in values], dtype=float)
        delays = np.asarray([value[1] for value in values], dtype=float)
        finite_delays = delays[np.isfinite(delays)]
        result[operation] = UartTimingSummary(
            len(values),
            float(np.mean(durations)),
            float(np.max(durations)),
            float(np.mean(finite_delays)) if finite_delays.size else None,
            float(np.max(finite_delays)) if finite_delays.size else None,
        )
    return result


def analyze_execution_log(value: ExecutionLog | str | Path) -> LogEvidence:
    log = _as_log(value)
    tracking = _tracking_errors(log)
    ages = np.asarray(
        [float(state["state_age_s"]) for state in log.states if state.get("state_age_s") is not None],
        dtype=float,
    )
    return LogEvidence(
        log.run_id,
        log.trajectory_id,
        log.schema_version,
        log.hardware_run,
        log.safety_profile,
        {
            key: log.simulation_config.get(key)
            for key in (
                "profile",
                "critical_telemetry_hz",
                "temperature_telemetry_hz",
                "telemetry_freshness_timeout_s",
            )
            if key in log.simulation_config
        },
        dict(log.provenance),
        _phase_endpoint_error(log, "ready"),
        _phase_endpoint_error(log, "probe"),
        float(np.mean(tracking)) if tracking.size else None,
        float(np.max(tracking)) if tracking.size else None,
        _settling_duration(log),
        float(np.mean(ages)) if ages.size else None,
        float(np.max(ages)) if ages.size else None,
        _uart_metrics(log),
    )
