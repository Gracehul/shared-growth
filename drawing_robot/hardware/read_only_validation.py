"""Strict observational Stage-4A hardware gate."""

from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import subprocess
import time
from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
from typing import Any

from ..execution import ExecutionLog, RobotState, RobotStatus
from ..kinematics import pose_coords
from ..robot.service import READ_METHODS, RobotService
from .config import Stage4AConfig


@dataclass(frozen=True)
class ReadOnlyGateResult:
    decision: str
    abort_reason: str | None
    log: ExecutionLog
    current_tcp_pose_mm_deg: tuple[float, ...] | None


def _git_commit() -> str:
    declared = os.environ.get("SHARED_GROWTH_GIT_COMMIT")
    if declared:
        return declared
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _package_version(name: str) -> str:
    try:
        return version(name)
    except PackageNotFoundError:
        return "not-installed"


def _config_hash(config: Stage4AConfig) -> str:
    payload = json.dumps(config.to_dict(), sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def _inside_workspace(pose: tuple[float, ...], config: Stage4AConfig) -> bool:
    bounds = config.allowed_workspace_mm
    return bounds is not None and all(
        low <= value <= high for value, (low, high) in zip(pose[:3], bounds)
    )


def validate_read_only_session(
    service: RobotService,
    config: Stage4AConfig,
    *,
    physical_preflight_pass: bool,
    min_consecutive_samples: int = 10,
    sample_period_s: float = 0.1,
) -> ReadOnlyGateResult:
    if not physical_preflight_pass:
        raise ValueError("physical preflight must be explicitly PASS")
    if not service.read_only:
        raise ValueError("strict validation requires RobotService(read_only=True)")
    if config.allowed_workspace_mm is None:
        raise ValueError("Stage-4A workspace is not configured")
    if min_consecutive_samples < 2 or sample_period_s <= 0:
        raise ValueError("invalid consecutive-sample policy")

    system_version = service.get_system_version()
    basic_version = service.get_basic_version()
    raw_samples = []
    normalized_states: list[RobotState] = []
    previous_timestamp = -math.inf
    previous_service_sequence = -1
    abort_reasons: list[str] = []

    for local_sequence in range(1, min_consecutive_samples + 1):
        sample_started = time.monotonic()
        try:
            raw = service.refresh_state()
        except Exception as exc:
            abort_reasons.append(f"ERROR_QUERY_OR_TELEMETRY_FAILED: {type(exc).__name__}: {exc}")
            break
        collected = time.monotonic()
        required_vectors = {
            "angles_deg": raw.angles_deg,
            "temperatures_c": raw.temperatures_c,
            "servo_status": raw.servo_status,
            "firmware_pose_mm_deg": raw.firmware_pose_mm_deg,
        }
        valid = True
        for name, values in required_vectors.items():
            if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
                abort_reasons.append(f"INVALID_TELEMETRY: {name}")
                valid = False
        if raw.controller_error is None:
            abort_reasons.append("ERROR_STATE_UNKNOWN")
            valid = False
        elif not isinstance(raw.controller_error, int) or raw.controller_error < 0:
            abort_reasons.append(f"ERROR_QUERY_FAILED_OR_INVALID: {raw.controller_error!r}")
            valid = False
        elif raw.controller_error != 0:
            abort_reasons.append(f"BLOCKING_CONTROLLER_FAULT_RAW_{raw.controller_error}")
            valid = False
        if raw.sequence <= previous_service_sequence:
            abort_reasons.append("LOCAL_SERVICE_SEQUENCE_NOT_INCREASING")
            valid = False
        if raw.monotonic_s <= previous_timestamp:
            abort_reasons.append("TELEMETRY_TIMESTAMP_NOT_INCREASING")
            valid = False
        state_age = collected - raw.monotonic_s
        if state_age < 0 or state_age > config.telemetry_freshness_timeout_s:
            abort_reasons.append(f"STALE_TELEMETRY: {state_age:.6f}s")
            valid = False
        tcp = tuple(float(v) for v in pose_coords(raw.angles_deg)) if len(raw.angles_deg) == 6 else ()
        sample = raw.to_dict()
        sample.update({
            "local_sample_sequence": local_sequence,
            "sample_started_monotonic_s": sample_started,
            "collected_monotonic_s": collected,
            "state_age_s": state_age,
            "raw_error_state": raw.controller_error,
            "error_query_status": "success" if raw.controller_error is not None else "unknown",
            "python_tcp_pose_mm_deg": list(tcp),
            "workspace_pose_basis": "python_tcp_from_joint_angles_using_provisional_tool_transform",
            "sample_valid": valid,
        })
        raw_samples.append(sample)
        normalized_states.append(RobotState(
            raw.monotonic_s,
            raw.angles_deg,
            raw.angles_deg,
            RobotStatus.FAULT if raw.controller_error not in (None, 0) else RobotStatus.IDLE,
            fault=(f"raw controller error {raw.controller_error}" if raw.controller_error not in (None, 0) else None),
            sample_sequence=local_sequence,
            state_age_s=state_age,
            is_fresh=state_age <= config.telemetry_freshness_timeout_s,
        ))
        previous_timestamp = raw.monotonic_s
        previous_service_sequence = raw.sequence
        if local_sequence < min_consecutive_samples:
            elapsed = time.monotonic() - sample_started
            time.sleep(max(0.0, sample_period_s - elapsed))

    transactions = service.io.transactions()
    unauthorized = [item.method for item in transactions if item.method not in READ_METHODS]
    if unauthorized:
        abort_reasons.append("READ_ONLY_ALLOWLIST_VIOLATION: " + ",".join(unauthorized))
    complete = len(raw_samples) == min_consecutive_samples and all(
        sample["sample_valid"] for sample in raw_samples
    )
    current_tcp = tuple(raw_samples[-1]["python_tcp_pose_mm_deg"]) if raw_samples else None
    if current_tcp is not None and not _inside_workspace(current_tcp, config):
        abort_reasons.append("CURRENT_POSE_OUTSIDE_PROVISIONAL_WORKSPACE")
    decision = "GO" if complete and not abort_reasons else "NO_GO"
    abort_reason = "; ".join(dict.fromkeys(abort_reasons)) or None

    initial = normalized_states[0] if normalized_states else RobotState(
        time.monotonic(), (), (), RobotStatus.FAULT, fault=abort_reason or "no telemetry"
    )
    log = ExecutionLog(
        "stage4a-read-only",
        config.to_dict(),
        0,
        initial,
        backend={
            "robot_model": "myCobot 280 JN",
            "backend_class": service.backend_class_name,
            "firmware_system_version_raw": system_version,
            "firmware_basic_version_raw": basic_version,
            "pymycobot_version": _package_version("pymycobot"),
            "python_version": platform.python_version(),
            "os": platform.platform(),
            "port": service.port,
            "baudrate": service.baudrate,
        },
        hardware_run=True,
        safety_profile="stage4a_conservative",
        provenance={
            "git_commit": _git_commit(),
            "configuration_hash_sha256": _config_hash(config),
            "trajectory_hash_sha256": None,
            "physical_preflight": "PASS",
        },
    )
    log.record_phase("physical_preflight", "PASS", "GO")
    log.record_phase(
        "configuration_preflight", "PASS", "GO",
        workspace_bounds_mm=config.allowed_workspace_mm,
        workspace_source=config.workspace_source,
        workspace_verified=config.workspace_verified,
        ready="UNVERIFIED",
        park="UNVERIFIED",
    )
    for raw, normalized in zip(raw_samples, normalized_states):
        log.record_hardware_telemetry(raw)
        log.record_state(normalized)
    for transaction in transactions:
        log.record_uart_transaction(transaction.to_dict())
    state_changing = len(unauthorized)
    log.record_phase(
        "strict_read_only_session",
        "PASS" if decision == "GO" else "BLOCKED",
        decision,
        abort_reason=abort_reason,
        operator_action_required=(None if decision == "GO" else "Do not move; inspect reported telemetry and pose."),
        required_samples=min_consecutive_samples,
        valid_samples=sum(bool(sample["sample_valid"]) for sample in raw_samples),
        motion_commands_sent=0,
        servo_release_calls=0,
        state_changing_calls=state_changing,
        current_tcp_pose_mm_deg=current_tcp,
        motion_state_known=True,
        stop_confirmed=None,
    )
    log.finish(
        "read_only_passed" if decision == "GO" else "blocked",
        time.monotonic(),
        decision=decision,
        abort_reason=abort_reason,
        operator_action_required=(None if decision == "GO" else "Do not move; inspect reported telemetry and pose."),
        motion_commands_sent=0,
        servo_release_calls=0,
        state_changing_calls=state_changing,
        motion_state_known=True,
        stop_confirmed=None,
    )
    return ReadOnlyGateResult(decision, abort_reason, log, current_tcp)
