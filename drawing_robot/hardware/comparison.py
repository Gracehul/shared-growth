"""Comparable summaries for Stage-3 simulation and Stage-4A hardware logs."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..execution import ExecutionLog


def summarize_execution(log: ExecutionLog) -> dict[str, Any]:
    commands = log.commands
    states = log.states
    result = log.result
    final_error = None
    if commands and states:
        target = np.asarray(commands[-1]["target_angles_deg"], dtype=float)
        actual = np.asarray(states[-1]["actual_angles_deg"], dtype=float)
        if target.shape == actual.shape == (6,):
            final_error = float(np.max(np.abs(target - actual)))
    send_times = [float(item["send_time_s"]) for item in commands]
    planned_times = [float(item["planned_time_s"]) for item in commands]
    send_lag = []
    if send_times:
        send_lag = [
            (sent - send_times[0]) - (planned - planned_times[0])
            for sent, planned in zip(send_times, planned_times)
        ]
    return {
        "hardware_run": log.hardware_run,
        "status": result.get("status"),
        "duration_s": result.get("timestamp_s"),
        "commands": len(commands),
        "states": len(states),
        "max_final_joint_error_deg": final_error,
        "max_planned_to_sent_lag_s": max(send_lag) if send_lag else None,
    }


def compare_sim_to_real(sim: ExecutionLog, real: ExecutionLog) -> dict[str, Any]:
    if sim.trajectory_id != real.trajectory_id:
        raise ValueError("logs must reference the same trajectory_id")
    if sim.hardware_run or not real.hardware_run:
        raise ValueError("expected one simulation log and one hardware log")
    return {
        "trajectory_id": sim.trajectory_id,
        "simulation": summarize_execution(sim),
        "real": summarize_execution(real),
        "interpretation": "model comparison only; numerical equality is not expected",
    }
