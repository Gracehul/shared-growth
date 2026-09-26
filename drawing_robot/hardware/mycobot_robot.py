"""Real myCobot adapter for the backend-independent Stage-3 executor."""

from __future__ import annotations

import platform
import sys
import time
from importlib.metadata import PackageNotFoundError, version

import numpy as np

from ..kinematics import pose_coords
from ..execution import (
    CommandAcknowledgement,
    ExecutionLog,
    JointCommand,
    RobotState,
    RobotStatus,
)
from ..robot import RobotService
from ..stage2 import JointTrajectory, ValidationResult
from .config import Stage4AConfig
from .errors import HardwareIntegrationError, PreflightRejected
from .safety_gate import motion_envelope_gate, preflight_gate, runtime_gate
from .telemetry import normalize_state


class MyCobotRobot:
    """Map one RobotService owner onto the Stage-3 RobotInterface.

    Construction never arms or moves the robot. ``preflight`` is mandatory
    before commands are accepted.
    """

    def __init__(
        self,
        service: RobotService,
        config: Stage4AConfig | None = None,
        execution_log: ExecutionLog | None = None,
    ) -> None:
        self.service = service
        self.config = config or Stage4AConfig()
        self.log = execution_log
        self._armed = False
        self._commanded: tuple[float, ...] = ()
        self._last_command_id: str | None = None
        self._stop_requested = False
        self._stop_confirmed = False
        self._stationary_samples = 0
        self._last_stationary_timestamp: float | None = None
        self._last_stop_angles: np.ndarray | None = None
        self._transaction_cursor = 0

    @staticmethod
    def hardware_metadata(service: RobotService) -> dict[str, object]:
        try:
            pymycobot_version = version("pymycobot")
        except PackageNotFoundError:
            pymycobot_version = "not-installed"
        return {
            "hardware_run": True,
            "safety_profile": "stage4a_conservative",
            "robot_model": "myCobot 280 JN",
            "backend_class": service.backend_class_name,
            "port": service.port,
            "baudrate": service.baudrate,
            "pymycobot_version": pymycobot_version,
            "python_version": platform.python_version(),
            "os": platform.platform(),
            "executable": sys.executable,
        }

    def preflight(
        self,
        trajectory: JointTrajectory,
        validation: ValidationResult,
        *,
        operator_supervising: bool,
        physical_stop_accessible: bool,
        workspace_clear: bool,
    ) -> None:
        envelope = motion_envelope_gate(trajectory, self.config)
        refresh = getattr(self.service, "refresh_state", None)
        state = refresh() if refresh is not None else self.service.latest_state()
        now = time.monotonic()
        gate = preflight_gate(
            state,
            self.config,
            validation,
            operator_supervising=operator_supervising,
            physical_stop_accessible=physical_stop_accessible,
            workspace_clear=workspace_clear,
            now_s=now,
        )
        reasons = list(envelope.reasons) + list(gate.reasons)
        if self.config.allowed_workspace_mm is None:
            reasons.append("STAGE4A_WORKSPACE_UNCONFIGURED")
        elif len(state.angles_deg) == 6:
            current_tcp = pose_coords(state.angles_deg)[:3]
            if any(
                value < low or value > high
                for value, (low, high) in zip(current_tcp, self.config.allowed_workspace_mm)
            ):
                reasons.append(
                    "CURRENT_POSE_OUTSIDE_PROVISIONAL_WORKSPACE: "
                    + repr([float(value) for value in current_tcp])
                )
        if trajectory.samples and len(state.angles_deg) == 6:
            start = np.asarray(trajectory.samples[0].positions_deg)
            if np.max(np.abs(np.asarray(state.angles_deg) - start)) > self.config.start_tolerance_deg:
                reasons.append("START_STATE_MISMATCH")
        if reasons:
            raise PreflightRejected("; ".join(reasons))
        self.service.arm_motion(locally_confirmed=True)
        self._commanded = tuple(state.angles_deg)
        self._armed = True

    def send_joint_command(self, command: JointCommand) -> CommandAcknowledgement:
        start = time.monotonic()
        if not self._armed or self._stop_requested:
            return CommandAcknowledgement(False, start, command.command_id, "hardware backend is not armed")
        state = self.service.latest_state()
        gate = runtime_gate(state, self.config, now_s=start)
        if not gate.allowed:
            self._armed = False
            try:
                self.service.stop_motion()
            except Exception:
                pass
            return CommandAcknowledgement(False, time.monotonic(), command.command_id, "; ".join(gate.reasons))
        target = np.asarray(command.target_angles_deg, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all():
            return CommandAcknowledgement(False, time.monotonic(), command.command_id, "invalid six-joint target")
        reference = np.asarray(self._commanded or state.angles_deg)
        if np.max(np.abs(target - reference)) > self.config.max_joint_step.value:
            return CommandAcknowledgement(False, time.monotonic(), command.command_id, "STAGE4A_JOINT_STEP_EXCEEDED")
        error: str | None = None
        accepted = False
        try:
            result = self.service.send_angles(target.tolist(), self.config.firmware_speed)
            accepted = result not in (False, -1)
            if not accepted:
                error = f"send_angles returned {result!r}"
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        end = time.monotonic()
        if accepted:
            self._commanded = tuple(float(v) for v in target)
            self._last_command_id = command.command_id
        self._annotate_command(command.command_id, start, end, accepted, error)
        self._drain_io_log()
        return CommandAcknowledgement(accepted, end, command.command_id, error)

    def get_state(self) -> RobotState:
        now = time.monotonic()
        raw = self.service.latest_state()
        actual = np.asarray(raw.angles_deg, dtype=float)
        target = np.asarray(self._commanded, dtype=float)
        moving = bool(actual.shape == (6,) and target.shape == (6,) and np.max(np.abs(actual - target)) > self.config.position_tolerance_deg)
        if (
            self._stop_requested
            and raw.critical_monotonic_s > 0
            and raw.critical_monotonic_s != self._last_stationary_timestamp
        ):
            self._last_stationary_timestamp = raw.critical_monotonic_s
            stopped = bool(
                actual.shape == (6,)
                and self._last_stop_angles is not None
                and np.max(np.abs(actual - self._last_stop_angles))
                <= self.config.cessation_angle_delta_deg
            )
            self._stationary_samples = self._stationary_samples + 1 if stopped else 0
            self._last_stop_angles = actual.copy() if actual.shape == (6,) else None
            if self._stationary_samples >= self.config.stopped_samples_required:
                self._stop_confirmed = True
        state = normalize_state(
            raw,
            now_s=now,
            freshness_timeout_s=self.config.telemetry_freshness_timeout_s,
            commanded_angles_deg=self._commanded or tuple(raw.angles_deg),
            last_command_id=self._last_command_id,
            moving=moving,
            stop_requested=self._stop_requested,
            stop_confirmed=self._stop_confirmed,
        )
        if self.log is not None:
            self.log.record_hardware_telemetry(raw.to_dict())
        self._drain_io_log()
        return state

    def stop_motion(self) -> None:
        self._armed = False
        self._stop_requested = True
        self._stop_confirmed = False
        self._stationary_samples = 0
        self._last_stationary_timestamp = None
        self._last_stop_angles = None
        self.service.disarm_motion()
        try:
            self.service.stop_motion()
        except Exception as exc:
            raise HardwareIntegrationError(f"STOP_FAILED: {exc}") from exc
        finally:
            self._drain_io_log()

    def disarm(self) -> None:
        """Reject future commands without issuing any hardware transaction."""
        self._armed = False
        self.service.disarm_motion()

    def _annotate_command(self, command_id: str, start: float, end: float, success: bool, error: str | None) -> None:
        if self.log is None:
            return
        record = self.log._commands_by_id.get(command_id)
        if record is not None:
            record.update({
                "send_start_s": start,
                "send_end_s": end,
                "api_duration_s": end - start,
                "success": success,
                "error": error,
                "observed_motion_onset_s": None,
            })

    def _drain_io_log(self) -> None:
        transactions = self.service.io.transactions(since=self._transaction_cursor)
        self._transaction_cursor += len(transactions)
        if self.log is not None:
            for transaction in transactions:
                self.log.record_uart_transaction(transaction.to_dict())
