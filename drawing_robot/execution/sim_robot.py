"""Deterministic rate-limited virtual robot backend; no physics model."""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

import numpy as np

from .clock import SimulationClock, SimulationPhase
from .commands import CommandAcknowledgement, JointCommand
from .config import ExecutionConfig, LatencyMode
from .log import ExecutionLog
from .state import RobotState, RobotStatus


@dataclass(order=True)
class _PendingCommand:
    activation_time_s: float
    sequence: int
    command: JointCommand = field(compare=False)


class SimRobot:
    """A command-delay and rate-limit model, deliberately not robot physics."""

    def __init__(
        self,
        clock: SimulationClock,
        config: ExecutionConfig,
        initial_angles_deg,
        execution_log: ExecutionLog | None = None,
    ) -> None:
        self.clock = clock
        self.config = config
        self.log = execution_log
        self._actual = np.asarray(initial_angles_deg, dtype=float)
        if self._actual.shape != (6,) or not np.isfinite(self._actual).all():
            raise ValueError("initial angles must contain six finite values")
        self._target = self._actual.copy()
        self._status = RobotStatus.IDLE
        self._last_command_id: str | None = None
        self._fault: str | None = None
        self._pending: list[_PendingCommand] = []
        self._pending_sequence = 0
        self._rng = np.random.default_rng(config.random_seed)
        self._stop_apply_time: float | None = None
        self._stop_start_actual: np.ndarray | None = None
        self._fault_applied = False
        self._publication_stale = False
        self._publish_period_s = 1.0 / config.state_update_rate_hz
        self._next_publish_s = clock.now() + self._publish_period_s
        self._published = self._snapshot(clock.now())

        clock.register(SimulationPhase.FAULTS, self._apply_scheduled_faults)
        clock.register(SimulationPhase.ACTIVATE_COMMANDS, self._activate_due_commands)
        clock.register(SimulationPhase.UPDATE_STATE, self._update_state)
        clock.register(SimulationPhase.PUBLISH_STATE, self._publish_if_due)

    def _snapshot(self, timestamp_s: float) -> RobotState:
        return RobotState(
            timestamp_s,
            self._target,
            self._actual,
            self._status,
            self._last_command_id,
            self._fault,
        )

    def _latency_s(self) -> float:
        latency = self.config.command_latency
        if latency.mode is LatencyMode.IDEAL:
            return 0.0
        if latency.mode is LatencyMode.FIXED:
            return latency.base_delay_s
        return latency.base_delay_s + float(
            self._rng.uniform(latency.jitter_min_s, latency.jitter_max_s)
        )

    def send_joint_command(self, command: JointCommand) -> CommandAcknowledgement:
        now = self.clock.now()
        target = np.asarray(command.target_angles_deg, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all():
            return CommandAcknowledgement(False, now, command.command_id, "invalid target")
        if self._status in {RobotStatus.STOPPING, RobotStatus.STOPPED, RobotStatus.FAULT}:
            return CommandAcknowledgement(
                False, now, command.command_id, f"backend status is {self._status.value}"
            )
        activation = now + self._latency_s()
        pending = _PendingCommand(activation, self._pending_sequence, command)
        self._pending_sequence += 1
        if activation <= now + 1e-12:
            self._activate_or_drop(pending, now)
        else:
            heapq.heappush(self._pending, pending)
        return CommandAcknowledgement(True, now, command.command_id)

    def _activate_or_drop(self, pending: _PendingCommand, timestamp_s: float) -> None:
        command = pending.command
        if command.command_id in self.config.faults.dropped_command_ids:
            if self.log is not None:
                self.log.command_dropped(command.command_id, timestamp_s)
            return
        self._target = np.asarray(command.target_angles_deg, dtype=float)
        self._last_command_id = command.command_id
        if np.all(
            np.abs(self._actual - self._target) <= self.config.position_tolerance_deg
        ):
            self._actual = self._target.copy()
            self._status = RobotStatus.IDLE
        else:
            self._status = RobotStatus.MOVING
        if self.log is not None:
            self.log.command_activated(command.command_id, timestamp_s)

    def _apply_scheduled_faults(self, _dt_s: float, now: float) -> None:
        faults = self.config.faults
        if faults.stale_state_at_s is not None and now >= faults.stale_state_at_s:
            self._publication_stale = True
        if (
            not self._fault_applied
            and faults.fault_at_s is not None
            and now >= faults.fault_at_s
        ):
            self._fault_applied = True
            self._fault = faults.fault_message
            self._status = RobotStatus.FAULT
            self._target = self._actual.copy()
            self._pending.clear()
            if self.log is not None:
                self.log.add_event("FAULT", now, fault=self._fault)

    def _activate_due_commands(self, _dt_s: float, now: float) -> None:
        if self._status in {RobotStatus.STOPPING, RobotStatus.STOPPED, RobotStatus.FAULT}:
            return
        while self._pending and self._pending[0].activation_time_s <= now + 1e-12:
            pending = heapq.heappop(self._pending)
            self._activate_or_drop(pending, now)

    def _update_state(self, dt_s: float, now: float) -> None:
        if self._status not in {RobotStatus.MOVING, RobotStatus.STOPPING}:
            return
        delta = self._target - self._actual
        maximum = np.asarray(self.config.joint_velocity_limits_deg_s) * dt_s
        step = np.sign(delta) * np.minimum(np.abs(delta), maximum)
        self._actual = self._actual + step
        reached = np.abs(self._target - self._actual) <= self.config.position_tolerance_deg
        self._actual[reached] = self._target[reached]
        if self._status is RobotStatus.MOVING and bool(np.all(reached)):
            self._status = RobotStatus.IDLE
        if (
            self._status is RobotStatus.STOPPING
            and self._stop_apply_time is not None
            and now >= self._stop_apply_time - 1e-12
        ):
            self._apply_stop(now)

    def _apply_stop(self, now: float) -> None:
        start = self._stop_start_actual
        movement = self._actual - start if start is not None else np.zeros(6)
        self._target = self._actual.copy()
        self._status = RobotStatus.STOPPED
        self._stop_apply_time = None
        if self.log is not None:
            self.log.add_event(
                "STOP_APPLIED",
                now,
                additional_movement_deg=movement.tolist(),
                additional_movement_norm_deg=float(np.linalg.norm(movement)),
            )

    def _publish_if_due(self, _dt_s: float, now: float) -> None:
        if self._publication_stale:
            return
        if now + 1e-12 >= self._next_publish_s:
            self._published = self._snapshot(now)
            while self._next_publish_s <= now + 1e-12:
                self._next_publish_s += self._publish_period_s

    def get_state(self) -> RobotState:
        return self._published

    def stop_motion(self) -> None:
        if self._status in {RobotStatus.STOPPED, RobotStatus.FAULT}:
            return
        now = self.clock.now()
        for pending in self._pending:
            if self.log is not None:
                self.log.command_dropped(pending.command.command_id, now)
        self._pending.clear()
        self._stop_start_actual = self._actual.copy()
        self._status = RobotStatus.STOPPING
        self._stop_apply_time = now + self.config.stop_delay_s
        if self.config.stop_delay_s == 0:
            self._apply_stop(now)

    @property
    def actual_angles_deg(self) -> tuple[float, ...]:
        return tuple(float(value) for value in self._actual)
