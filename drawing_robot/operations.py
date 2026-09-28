"""High-level cockpit operation boundary over the existing execution stack."""

from __future__ import annotations

import uuid
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, Mapping, Sequence

from .execution import (
    ExecutionConfig,
    ExecutionLog,
    MotionExecutor,
    RobotInterface,
    RobotStatus,
)
from .hardware import GateResult, GateStatus, validation_gate_result
from .hardware.policy import hard_result
from .stage2 import JointTrajectory, ValidationResult


class InteractionMode(str, Enum):
    OBSERVE = "OBSERVE"
    MANUAL = "MANUAL"
    SHARED_GROWTH = "SHARED_GROWTH"


class Operation(str, Enum):
    GO_READY = "GO_READY"
    EXECUTE_TRAJECTORY = "EXECUTE_TRAJECTORY"
    START_SHARED_GROWTH = "START_SHARED_GROWTH"
    STOP = "STOP"


class EvidenceStatus(str, Enum):
    PRESENT = "PRESENT"
    ABSENT = "ABSENT"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class OperationRequest:
    request_id: str
    mode: InteractionMode
    operation: Operation
    trajectory_id: str | None = None
    session_id: str | None = None
    perception_id: str | None = None
    intent_id: str | None = None

    def __post_init__(self) -> None:
        if not self.request_id:
            raise ValueError("request_id must not be empty")
        object.__setattr__(self, "mode", InteractionMode(self.mode))
        object.__setattr__(self, "operation", Operation(self.operation))


@dataclass(frozen=True)
class PreparedTrajectory:
    """Resolved Stage-2 evidence; never constructed from cockpit raw motion values."""

    trajectory_id: str
    trajectory: JointTrajectory
    validation: ValidationResult
    perception_id: str | None = None
    intent_id: str | None = None


@dataclass(frozen=True)
class OperationEvidence:
    perception_id: str | None
    perception: EvidenceStatus
    intent_id: str | None
    intended: EvidenceStatus
    gate_decision: EvidenceStatus
    command_ids: tuple[str, ...]
    commanded: EvidenceStatus
    robot_state_timestamp_s: float | None
    actual: EvidenceStatus


@dataclass(frozen=True)
class OperationReceipt:
    request_id: str
    accepted: bool
    run_id: str | None
    state: str
    reason: str | None
    gate_results: tuple[GateResult, ...]
    session_id: str | None = None
    evidence: OperationEvidence | None = None
    latest_execution_event: dict[str, object] | None = None


GateProvider = Callable[[], Sequence[GateResult]]
OperationGateProvider = Callable[[PreparedTrajectory], Sequence[GateResult]]
ReceiptSink = Callable[[OperationReceipt], None]
ExecutionLogSink = Callable[[ExecutionLog], None]


_PERMISSIONS = {
    InteractionMode.OBSERVE: {Operation.STOP},
    InteractionMode.MANUAL: {
        Operation.GO_READY,
        Operation.EXECUTE_TRAJECTORY,
        Operation.STOP,
    },
    InteractionMode.SHARED_GROWTH: {
        Operation.GO_READY,
        Operation.START_SHARED_GROWTH,
        Operation.STOP,
    },
}


class OperationController:
    """Resolve high-level intent, apply existing gates, then call MotionExecutor.

    The controller contains no planning, telemetry polling or robot-specific
    command logic. A real composition supplies current core GateResults through
    ``gate_provider`` and an already prepared RobotInterface backend.
    """

    def __init__(
        self,
        robot: RobotInterface,
        clock,
        execution_config: ExecutionConfig,
        trajectories: Mapping[str, PreparedTrajectory],
        *,
        gate_provider: GateProvider | None,
        ready_trajectory_id: str | None = None,
        mode: InteractionMode = InteractionMode.OBSERVE,
        receipt_sink: ReceiptSink | None = None,
        operation_gate_provider: OperationGateProvider | None = None,
        execution_log_sink: ExecutionLogSink | None = None,
    ) -> None:
        if not isinstance(robot, RobotInterface):
            raise TypeError("robot must implement RobotInterface")
        self._robot = robot
        self._clock = clock
        self._execution_config = execution_config
        self._trajectories = dict(trajectories)
        self._gate_provider = gate_provider
        self._ready_trajectory_id = ready_trajectory_id
        self._mode = InteractionMode(mode)
        self._receipt_sink = receipt_sink
        self._operation_gate_provider = operation_gate_provider
        self._execution_log_sink = execution_log_sink
        self._shared_session_id: str | None = None
        self._active_run_id: str | None = None
        self._active_log: ExecutionLog | None = None
        self._latest_receipt: OperationReceipt | None = None

    @property
    def mode(self) -> InteractionMode:
        return self._mode

    def set_mode(self, mode: InteractionMode) -> None:
        self._mode = InteractionMode(mode)

    def latest_receipt(self) -> OperationReceipt | None:
        return self._latest_receipt

    def request(self, req: OperationRequest) -> OperationReceipt:
        if req.operation is Operation.STOP:
            return self.stop(req.request_id, req.mode, req.session_id)
        permission = self._permission_gate(req)
        if permission.hard_blocking:
            return self._reject(req, (permission,))
        if req.operation is Operation.START_SHARED_GROWTH:
            gates = (permission,) + self._runtime_gates()
            blocker = self._first_blocker(gates)
            if blocker is not None:
                return self._reject(req, gates, blocker)
            session_id = req.session_id or str(uuid.uuid4())
            self._shared_session_id = session_id
            return self._publish(
                OperationReceipt(
                    req.request_id,
                    True,
                    None,
                    "SHARED_GROWTH_ACTIVE",
                    None,
                    gates,
                    session_id,
                    self._evidence(req, None, (), None),
                    None,
                )
            )
        prepared, resolution_gate = self._resolve(req)
        if prepared is None:
            return self._reject(req, (permission, resolution_gate), resolution_gate)
        gates = (
            permission,
            resolution_gate,
            validation_gate_result(prepared.validation),
            *self._runtime_gates(),
        )
        blocker = self._first_blocker(gates)
        if blocker is not None:
            return self._reject(req, gates, blocker, prepared)
        operation_gates = self._operation_gates(prepared)
        gates = (*gates, *operation_gates)
        blocker = self._first_blocker(gates)
        if blocker is not None:
            return self._reject(req, gates, blocker, prepared)
        return self._execute(req, prepared, gates)

    def _operation_gates(self, prepared: PreparedTrajectory) -> tuple[GateResult, ...]:
        """Run an optional backend preflight without duplicating gate policy."""

        if self._operation_gate_provider is None:
            return ()
        try:
            return tuple(self._operation_gate_provider(prepared))
        except Exception as exc:
            return (
                hard_result(
                    "operation_preflight",
                    GateStatus.UNKNOWN,
                    reason=(
                        "OPERATION_PREFLIGHT_UNKNOWN: " f"{type(exc).__name__}: {exc}"
                    ),
                ),
            )

    def stop(
        self,
        request_id: str | None = None,
        mode: InteractionMode | None = None,
        session_id: str | None = None,
    ) -> OperationReceipt:
        request_id = request_id or f"stop-{uuid.uuid4()}"
        request_mode = self._mode if mode is None else InteractionMode(mode)
        req = OperationRequest(
            request_id,
            request_mode,
            Operation.STOP,
            session_id=session_id or self._shared_session_id,
        )
        requested = hard_result("stop_request", GateStatus.PASS, value=True)
        timestamp = self._clock.now()
        if self._active_log is not None:
            self._active_log.add_event(
                "STOP_REQUESTED", timestamp, request_id=request_id
            )
        try:
            self._robot.stop_motion()
            state = self._robot.get_state()
            confirmed = state.status is RobotStatus.STOPPED
            stop_gate = hard_result(
                "stop_confirmation",
                GateStatus.PASS if confirmed else GateStatus.UNKNOWN,
                value=confirmed,
                reason=(
                    None
                    if confirmed
                    else "STOP_PENDING: measured cessation not confirmed"
                ),
            )
            return self._publish(
                OperationReceipt(
                    request_id,
                    True,
                    self._active_run_id,
                    "STOPPED" if confirmed else "STOPPING",
                    None if confirmed else "STOP_PENDING",
                    (requested, stop_gate),
                    req.session_id,
                    self._evidence(req, None, (), state.timestamp_s),
                    self._latest_event(),
                )
            )
        except Exception as exc:
            if self._active_log is not None:
                self._active_log.add_event(
                    "STOP_FAILED",
                    self._clock.now(),
                    error=f"{type(exc).__name__}: {exc}",
                )
            failed = hard_result(
                "stop_confirmation",
                GateStatus.BLOCK,
                value=False,
                reason=f"STOP_FAILED: {type(exc).__name__}: {exc}",
            )
            return self._publish(
                OperationReceipt(
                    request_id,
                    False,
                    self._active_run_id,
                    "FAILED",
                    "STOP_FAILED",
                    (requested, failed),
                    req.session_id,
                    self._evidence(req, None, (), None),
                    self._latest_event(),
                )
            )

    def _permission_gate(self, req: OperationRequest) -> GateResult:
        if req.mode is not self._mode:
            return hard_result(
                "mode_permission",
                GateStatus.BLOCK,
                value=req.mode.value,
                limit=self._mode.value,
                reason="MODE_MISMATCH",
            )
        allowed = req.operation in _PERMISSIONS[self._mode]
        return hard_result(
            "mode_permission",
            GateStatus.PASS if allowed else GateStatus.BLOCK,
            value=f"{self._mode.value}:{req.operation.value}",
            reason=None if allowed else "MODE_PERMISSION_DENIED",
        )

    def _runtime_gates(self) -> tuple[GateResult, ...]:
        if self._gate_provider is None:
            return (
                hard_result(
                    "runtime_policy",
                    GateStatus.UNKNOWN,
                    reason="RUNTIME_GATES_UNKNOWN",
                ),
            )
        try:
            results = tuple(self._gate_provider())
        except Exception as exc:
            return (
                hard_result(
                    "runtime_policy",
                    GateStatus.UNKNOWN,
                    reason=f"RUNTIME_GATES_UNKNOWN: {type(exc).__name__}: {exc}",
                ),
            )
        # The prepared trajectory's own ValidationResult is authoritative here.
        return tuple(
            result for result in results if result.name != "trajectory_validity"
        )

    def _resolve(
        self, req: OperationRequest
    ) -> tuple[PreparedTrajectory | None, GateResult]:
        trajectory_id = (
            self._ready_trajectory_id
            if req.operation is Operation.GO_READY
            else req.trajectory_id
        )
        if not trajectory_id:
            reason = (
                "READY_TRAJECTORY_UNAVAILABLE"
                if req.operation is Operation.GO_READY
                else "TRAJECTORY_ID_REQUIRED"
            )
            return None, hard_result(
                "operation_resolution", GateStatus.BLOCK, reason=reason
            )
        prepared = self._trajectories.get(trajectory_id)
        return prepared, hard_result(
            "operation_resolution",
            GateStatus.PASS if prepared is not None else GateStatus.BLOCK,
            value=trajectory_id,
            reason=None if prepared is not None else "TRAJECTORY_NOT_FOUND",
        )

    @staticmethod
    def _first_blocker(results: Sequence[GateResult]) -> GateResult | None:
        return next(
            (
                result
                for result in results
                if result.hard_blocking or result.qualification_miss
            ),
            None,
        )

    def _execute(
        self,
        req: OperationRequest,
        prepared: PreparedTrajectory,
        gates: tuple[GateResult, ...],
    ) -> OperationReceipt:
        initial = self._robot.get_state()
        log = ExecutionLog(
            prepared.trajectory_id,
            self._execution_config.to_dict(),
            self._execution_config.random_seed,
            initial,
        )
        self._active_log = log
        self._active_run_id = log.run_id
        if self._execution_log_sink is not None:
            self._execution_log_sink(log)
        executor = MotionExecutor(self._robot, self._clock, self._execution_config, log)
        try:
            result = executor.execute(
                prepared.trajectory,
                prepared.validation,
                trajectory_id=prepared.trajectory_id,
            )
            command_ids = tuple(item["command_id"] for item in log.commands)
            success = result.status.value in {"COMPLETED", "STOPPED"}
            receipt = OperationReceipt(
                req.request_id,
                True,
                log.run_id,
                result.status.value,
                result.message,
                gates,
                req.session_id or self._shared_session_id,
                self._evidence(
                    req, prepared, command_ids, result.final_state.timestamp_s
                ),
                self._latest_event(),
            )
            if not success and receipt.reason is None:
                receipt = replace(receipt, reason="EXECUTION_FAILED")
            return self._publish(receipt)
        except Exception as exc:
            return self._publish(
                OperationReceipt(
                    req.request_id,
                    True,
                    log.run_id,
                    "FAILED",
                    f"{type(exc).__name__}: {exc}",
                    gates,
                    req.session_id or self._shared_session_id,
                    self._evidence(req, prepared, (), initial.timestamp_s),
                    self._latest_event(),
                )
            )

    def _reject(
        self,
        req: OperationRequest,
        gates: Sequence[GateResult],
        blocker: GateResult | None = None,
        prepared: PreparedTrajectory | None = None,
    ) -> OperationReceipt:
        blocker = blocker or self._first_blocker(gates)
        reason = self._reason_code(blocker) if blocker is not None else "REJECTED"
        return self._publish(
            OperationReceipt(
                req.request_id,
                False,
                None,
                "REJECTED",
                reason,
                tuple(gates),
                req.session_id,
                self._evidence(req, prepared, (), None),
                None,
            )
        )

    @staticmethod
    def _reason_code(result: GateResult) -> str:
        if result.category.value == "PERFORMANCE":
            return result.name.upper()
        if result.reason:
            return result.reason.split(":", 1)[0]
        return result.name.upper()

    @staticmethod
    def _evidence(
        req: OperationRequest,
        prepared: PreparedTrajectory | None,
        command_ids: tuple[str, ...],
        actual_timestamp_s: float | None,
    ) -> OperationEvidence:
        perception_id = req.perception_id or (
            None if prepared is None else prepared.perception_id
        )
        intent_id = req.intent_id or (None if prepared is None else prepared.intent_id)
        return OperationEvidence(
            perception_id,
            EvidenceStatus.PRESENT if perception_id else EvidenceStatus.ABSENT,
            intent_id,
            EvidenceStatus.PRESENT if intent_id else EvidenceStatus.ABSENT,
            EvidenceStatus.PRESENT,
            command_ids,
            EvidenceStatus.PRESENT if command_ids else EvidenceStatus.ABSENT,
            actual_timestamp_s,
            (
                EvidenceStatus.PRESENT
                if actual_timestamp_s is not None
                else EvidenceStatus.UNKNOWN
            ),
        )

    def _latest_event(self) -> dict[str, object] | None:
        if self._active_log is None or not self._active_log.events:
            return None
        event = self._active_log.events[-1]
        return {
            "sequence": event.sequence,
            "timestamp_s": event.timestamp_s,
            "kind": event.kind,
            "data": dict(event.data),
        }

    def _publish(self, receipt: OperationReceipt) -> OperationReceipt:
        self._latest_receipt = receipt
        if self._receipt_sink is not None:
            self._receipt_sink(receipt)
        return receipt
