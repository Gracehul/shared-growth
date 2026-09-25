"""Open-loop command scheduling with closed-loop state monitoring."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from ..stage2.types import JointTrajectory, ValidationResult
from .clock import SimulationClock
from .commands import JointCommand
from .config import ExecutionConfig
from .interface import RobotInterface
from .log import ExecutionLog
from .state import RobotState, RobotStatus


class ExecutorStatus(str, Enum):
    READY = "READY"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ExecutionResult:
    status: ExecutorStatus
    final_state: RobotState
    commands_issued: int
    message: str | None = None


class MotionExecutor:
    """Backend-independent scheduler; only RobotInterface methods are used."""

    def __init__(
        self,
        robot: RobotInterface,
        clock: SimulationClock,
        config: ExecutionConfig,
        execution_log: ExecutionLog | None = None,
    ) -> None:
        if not isinstance(robot, RobotInterface):
            raise TypeError("robot must implement RobotInterface")
        self.robot = robot
        self.clock = clock
        self.config = config
        self.log = execution_log
        self.status = ExecutorStatus.READY

    def execute(
        self,
        trajectory: JointTrajectory,
        validation_result: ValidationResult,
        *,
        trajectory_id: str = "trajectory",
        stop_at_s: float | None = None,
    ) -> ExecutionResult:
        if not validation_result.valid or validation_result.errors:
            raise ValueError("executor requires a valid Stage-2 ValidationResult")
        samples = trajectory.samples
        if not samples:
            raise ValueError("joint trajectory must not be empty")
        times = np.asarray([sample.time_s for sample in samples], dtype=float)
        joints = np.asarray([sample.positions_deg for sample in samples], dtype=float)
        if joints.shape != (len(samples), 6) or not np.isfinite(joints).all():
            raise ValueError("joint trajectory must contain finite six-joint samples")
        if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
            raise ValueError("joint trajectory timestamps must be finite and increasing")
        if stop_at_s is not None and stop_at_s < 0:
            raise ValueError("stop_at_s must be non-negative")

        start_clock = self.clock.now()
        plan_origin = float(times[0])
        relative_times = times - plan_origin
        initial_state = self.robot.get_state()
        if self.log is None:
            self.log = ExecutionLog(
                trajectory_id,
                self.config.to_dict(),
                self.config.random_seed,
                initial_state,
            )
        self.log.record_state(initial_state)
        self.status = ExecutorStatus.RUNNING
        next_command = 0
        stop_requested = False
        last_recorded_state_timestamp = initial_state.timestamp_s
        last_command_id = f"{trajectory_id}:{len(samples) - 1:06d}"
        completion_deadline = (
            start_clock
            + float(relative_times[-1])
            + self.config.trajectory_completion_timeout_s
        )

        while True:
            now = self.clock.now()
            elapsed = now - start_clock

            if stop_at_s is not None and not stop_requested and elapsed >= stop_at_s - 1e-12:
                stop_requested = True
                self.status = ExecutorStatus.STOPPING
                self.log.add_event("STOP_REQUESTED", now)
                self.robot.stop_motion()

            while (
                not stop_requested
                and next_command < len(samples)
                and relative_times[next_command] <= elapsed + 1e-12
            ):
                command = JointCommand(
                    f"{trajectory_id}:{next_command:06d}",
                    samples[next_command].positions_deg,
                    samples[next_command].time_s,
                )
                self.log.command_sent(command, now)
                acknowledgement = self.robot.send_joint_command(command)
                if not acknowledgement.accepted:
                    self.log.add_event(
                        "COMMAND_REJECTED",
                        acknowledgement.backend_timestamp_s,
                        command_id=command.command_id,
                        error=acknowledgement.error,
                    )
                    return self._fail(
                        self.robot.get_state(),
                        next_command + 1,
                        f"command rejected: {acknowledgement.error}",
                    )
                next_command += 1

            state = self.robot.get_state()
            if state.timestamp_s != last_recorded_state_timestamp:
                self.log.record_state(state)
                last_recorded_state_timestamp = state.timestamp_s

            if state.status is RobotStatus.FAULT or state.fault is not None:
                return self._fail(state, next_command, state.fault or "backend fault")

            state_age = now - state.timestamp_s
            if state_age > self.config.state_freshness_timeout_s + 1e-12:
                self.log.add_event("STATE_STALE", now, state_age_s=state_age)
                return self._fail(state, next_command, "state freshness timeout")

            if stop_requested and state.status is RobotStatus.STOPPED:
                self.status = ExecutorStatus.STOPPED
                self.log.finish("stopped", now, commands_issued=next_command)
                return ExecutionResult(self.status, state, next_command)

            all_sent = next_command == len(samples)
            final_active = state.last_command_id == last_command_id
            final_error = np.max(np.abs(np.asarray(state.actual_angles_deg) - joints[-1]))
            final_reached = final_error <= self.config.position_tolerance_deg
            if all_sent and final_active and final_reached and not stop_requested:
                self.status = ExecutorStatus.COMPLETED
                self.log.add_event(
                    "RUN_COMPLETED", now, final_error_deg=float(final_error)
                )
                self.log.finish(
                    "completed",
                    now,
                    commands_issued=next_command,
                    final_error_deg=float(final_error),
                )
                return ExecutionResult(self.status, state, next_command)

            if not stop_requested and now > completion_deadline + 1e-12:
                self.log.add_event("TIMEOUT", now, timeout_type="completion")
                return self._fail(state, next_command, "trajectory completion timeout")

            self.clock.advance(self.config.simulation_timestep_s)

    def _fail(
        self, state: RobotState, commands_issued: int, message: str
    ) -> ExecutionResult:
        self.status = ExecutorStatus.FAILED
        now = self.clock.now()
        self.log.add_event("RUN_FAILED", now, reason=message)
        self.log.finish("failed", now, reason=message, commands_issued=commands_issued)
        return ExecutionResult(self.status, state, commands_issued, message)
