"""Read-only aggregate contract for cockpit and debugging consumers."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..execution import ExecutionLog
from ..robot.state import RobotMode, RobotState as ServiceState
from ..stage2 import ValidationResult
from .config import Stage4AConfig
from .policy import (
    ExecutionPhase,
    GateResult,
    GateStatus,
    hard_result,
    performance_result,
    tolerance_result,
)
from .safety_gate import runtime_gate, validation_gate_result


@dataclass(frozen=True)
class SystemSnapshot:
    """Aggregate view over existing state; deliberately not another RobotState."""

    timestamp_s: float
    execution_phase: ExecutionPhase
    q_target_deg: tuple[float, ...]
    q_actual_deg: tuple[float, ...]
    joint_error_deg: tuple[float, ...]
    temperatures_c: tuple[float, ...]
    telemetry_age_s: float | None
    controller_fault: int | str | None
    validation_status: GateStatus
    gate_results: tuple[GateResult, ...]
    run_id: str | None = None
    latest_execution_event: dict[str, object] | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "timestamp_s": self.timestamp_s,
            "execution_phase": self.execution_phase.value,
            "q_target_deg": list(self.q_target_deg),
            "q_actual_deg": list(self.q_actual_deg),
            "joint_error_deg": list(self.joint_error_deg),
            "temperatures_c": list(self.temperatures_c),
            "telemetry_age_s": self.telemetry_age_s,
            "controller_fault": self.controller_fault,
            "validation_status": self.validation_status.value,
            "gate_results": [result.to_dict() for result in self.gate_results],
            "run_id": self.run_id,
            "latest_execution_event": self.latest_execution_event,
        }


def _provider_value(provider, method_names: tuple[str, ...] = ()):
    if provider is None:
        return None
    if callable(provider):
        return provider()
    for name in method_names:
        method = getattr(provider, name, None)
        if callable(method):
            return method()
    return provider


def _phase_from_log(log: ExecutionLog | None) -> ExecutionPhase | None:
    if log is None:
        return None
    if log.phases:
        name = str(log.phases[-1].get("phase", "")).upper()
        aliases = {
            "PROVISIONAL_READY": ExecutionPhase.READY_CHECK,
            "READY": ExecutionPhase.READY_CHECK,
            "READY_CHECK": ExecutionPhase.READY_CHECK,
            "SETTLING": ExecutionPhase.SETTLING,
        }
        if name in aliases:
            return aliases[name]
        if name in ExecutionPhase.__members__:
            return ExecutionPhase[name]
    status = str(log.result.get("status", "running")).upper()
    return {
        "COMPLETED": ExecutionPhase.COMPLETED,
        "PASSED": ExecutionPhase.COMPLETED,
        "FAILED": ExecutionPhase.FAILED,
        "STOPPED": ExecutionPhase.STOPPED,
        "RUNNING": ExecutionPhase.RUNNING,
    }.get(status)


def _phase_from_state(state: ServiceState | None) -> ExecutionPhase:
    if state is None:
        return ExecutionPhase.IDLE
    return {
        RobotMode.DISCONNECTED: ExecutionPhase.IDLE,
        RobotMode.INITIALIZING: ExecutionPhase.PRECHECK,
        RobotMode.READY: ExecutionPhase.IDLE,
        RobotMode.EXECUTING: ExecutionPhase.RUNNING,
        RobotMode.STOPPING: ExecutionPhase.STOPPING,
        RobotMode.FAULT: ExecutionPhase.FAILED,
        RobotMode.PARKING: ExecutionPhase.RUNNING,
        RobotMode.PARKED: ExecutionPhase.STOPPED,
    }[state.mode]


class SystemSnapshotAdapter:
    """Expose core policy without UART, pymycobot or command methods."""

    def __init__(
        self,
        state_provider=None,
        *,
        config: Stage4AConfig | None = None,
        validation_provider=None,
        target_provider=None,
        phase_provider=None,
        log_provider=None,
        now_provider: Callable[[], float] = time.monotonic,
    ) -> None:
        self._state_provider = state_provider
        self._config = config or Stage4AConfig()
        self._validation_provider = validation_provider
        self._target_provider = target_provider
        self._phase_provider = phase_provider
        self._log_provider = log_provider
        self._now = now_provider

    def _state(self) -> ServiceState | None:
        value = _provider_value(self._state_provider, ("latest_state", "latest"))
        return value if isinstance(value, ServiceState) else None

    def _log(self) -> ExecutionLog | None:
        value = _provider_value(self._log_provider, ("latest",))
        return value if isinstance(value, ExecutionLog) else None

    def _validation(self) -> ValidationResult | None:
        value = _provider_value(self._validation_provider, ("latest",))
        return value if isinstance(value, ValidationResult) else None

    def _target(self, log: ExecutionLog | None) -> tuple[float, ...]:
        value = _provider_value(self._target_provider, ("latest",))
        if value is not None:
            return tuple(float(item) for item in value)
        if log is not None and log.commands:
            return tuple(float(item) for item in log.commands[-1]["target_angles_deg"])
        return ()

    def _actual(self, state: ServiceState | None, log: ExecutionLog | None) -> tuple[float, ...]:
        if state is not None:
            return tuple(state.angles_deg)
        if log is not None and log.states:
            return tuple(float(item) for item in log.states[-1]["actual_angles_deg"])
        return ()

    def _phase(self, state: ServiceState | None, log: ExecutionLog | None) -> ExecutionPhase:
        value = _provider_value(self._phase_provider, ("latest",))
        if value is not None:
            return ExecutionPhase(value)
        return _phase_from_log(log) or _phase_from_state(state)

    def get_latest_execution_event(self) -> dict[str, object] | None:
        log = self._log()
        if log is None or not log.events:
            return None
        event = log.events[-1]
        return {
            "sequence": event.sequence,
            "timestamp_s": event.timestamp_s,
            "kind": event.kind,
            "data": dict(event.data),
        }

    def get_gate_results(self) -> tuple[GateResult, ...]:
        state = self._state()
        log = self._log()
        validation = self._validation()
        now = self._now()
        results: list[GateResult] = [validation_gate_result(validation)]
        if state is None:
            results.append(
                hard_result(
                    "robot_state",
                    GateStatus.BLOCK,
                    reason="CRITICAL_STATE_UNKNOWN: RobotState unavailable",
                )
            )
        else:
            results.extend(runtime_gate(state, self._config, now_s=now).results)
        target = self._target(log)
        actual = self._actual(state, log)
        if len(target) == len(actual) == 6:
            error = float(np.max(np.abs(np.asarray(target) - np.asarray(actual))))
            results.append(
                tolerance_result(
                    "completion_error",
                    error,
                    self._config.position_tolerance_deg,
                    unit="deg",
                    reason="COMPLETION_TARGET_MISSED",
                )
            )
        phase = self._phase(state, log)
        if phase is ExecutionPhase.READY_CHECK and self._config.ready_angles_deg is not None and len(actual) == 6:
            ready_error = float(
                np.max(np.abs(np.asarray(actual) - self._config.ready_angles_deg))
            )
            results.append(
                tolerance_result(
                    "ready_error",
                    ready_error,
                    self._config.ready_tolerance_deg,
                    unit="deg",
                    reason="READY_TARGET_MISSED",
                )
            )
        if phase is ExecutionPhase.SETTLING:
            delta = None
            duration = None
            if log is not None and len(log.states) >= 2:
                previous = np.asarray(log.states[-2].get("actual_angles_deg", ()), dtype=float)
                latest = np.asarray(log.states[-1].get("actual_angles_deg", ()), dtype=float)
                if previous.shape == latest.shape == (6,):
                    delta = float(np.max(np.abs(latest - previous)))
                if log.commands:
                    sent = log.commands[-1].get(
                        "send_end_s", log.commands[-1].get("send_time_s")
                    )
                    if sent is not None:
                        duration = max(0.0, float(log.states[-1]["timestamp_s"]) - float(sent))
            settling_ok = (
                delta is not None
                and duration is not None
                and delta <= self._config.cessation_angle_delta_deg
                and duration
                >= self._config.ready_settling_samples
                * self._config.verification_sample_period_s
            )
            results.append(
                performance_result(
                    "settling",
                    GateStatus.NOMINAL
                    if settling_ok
                    else GateStatus.DEGRADED,
                    value={"max_delta_deg": delta, "duration_s": duration},
                    limit={
                        "max_delta_deg": self._config.cessation_angle_delta_deg,
                        "minimum_duration_s": self._config.ready_settling_samples
                        * self._config.verification_sample_period_s,
                    },
                    reason=None if settling_ok else "SETTLING_TARGET_MISSED",
                )
            )
        if phase in {ExecutionPhase.STOPPING, ExecutionPhase.STOPPED}:
            stop_confirmed = None if log is None else log.result.get("stop_confirmed")
            status = (
                GateStatus.PASS
                if stop_confirmed is True
                else GateStatus.BLOCK
            )
            results.append(
                hard_result(
                    "stop_confirmation",
                    status,
                    value=stop_confirmed,
                    reason=None
                    if status is GateStatus.PASS
                    else "STOP_STATE_UNKNOWN"
                    if stop_confirmed is None
                    else "STOP_FAILED",
                )
            )
        return tuple(results)

    def get_system_snapshot(self) -> SystemSnapshot:
        state = self._state()
        log = self._log()
        validation = self._validation()
        now = self._now()
        target = self._target(log)
        actual = self._actual(state, log)
        joint_error = (
            tuple(float(value) for value in np.asarray(target) - np.asarray(actual))
            if len(target) == len(actual) == 6
            else ()
        )
        temperatures = tuple(state.temperatures_c) if state is not None else ()
        if not temperatures and log is not None and log.telemetry:
            temperatures = tuple(log.telemetry[-1].get("temperatures_c", ()))
        critical = 0.0 if state is None else state.critical_monotonic_s or state.monotonic_s
        age = max(0.0, now - critical) if critical > 0 else None
        controller_fault = None if state is None else state.controller_error
        if controller_fault in (None, 0) and state is not None and state.fault:
            controller_fault = state.fault
        validation_status = validation_gate_result(validation).status
        return SystemSnapshot(
            now,
            self._phase(state, log),
            target,
            actual,
            joint_error,
            temperatures,
            age,
            controller_fault,
            validation_status,
            self.get_gate_results(),
            None if log is None else log.run_id,
            self.get_latest_execution_event(),
        )

    def get_thresholds(self) -> dict[str, dict[str, object]]:
        return {
            name: threshold.to_dict()
            for name, threshold in self._config.threshold_metadata().items()
        }
