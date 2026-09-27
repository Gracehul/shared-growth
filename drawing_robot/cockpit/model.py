"""Compose existing sources into concise cockpit display values."""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from ..robot.state import RobotMode, RobotState
from .sources import CameraSource, ExecutionLogSource, RobotStateSource, TrajectorySource


SOURCE_MAPPING = (
    ("Camera preview + timing", "camera.capture / cockpit.CameraSource", "BGR frame, resolution, FPS, frame age, failures"),
    ("Robot health", "drawing_robot.robot.RobotService", "authoritative robot.state.RobotState"),
    ("Joint actual", "RobotState.angles_deg", "live degrees"),
    ("Joint target", "ExecutionLog.commands", "latest recorded target; unavailable outside a run log"),
    ("Temperature / fault / freshness", "RobotState", "temperatures_c, fault, controller_error, critical_monotonic_s"),
    ("Trajectory view", "Stage-2B bundle", "CartesianTrajectory + JointTrajectory"),
    ("Validation", "Stage-2B bundle", "ValidationResult"),
    ("Run result", "ExecutionLog", "recorded result; not presented as live telemetry"),
    ("Simulation", "SimRobot ExecutionLog", "same recorded state/event contract"),
)


@dataclass(frozen=True)
class PipelineNode:
    name: str
    status: str
    detail: str = ""
    active: bool = False
    kind: str = "normal"


class Workspace(str, Enum):
    AUTHOR = "AUTHOR"
    PREVIEW = "PREVIEW"
    RUN = "RUN"
    REVIEW = "REVIEW"


class CockpitModel:
    def __init__(
        self,
        camera: CameraSource,
        robot: RobotStateSource,
        trajectory: TrajectorySource,
        execution: ExecutionLogSource,
    ) -> None:
        self.camera = camera
        self.robot = robot
        self.trajectory = trajectory
        self.execution = execution

    @staticmethod
    def display_mode(state: RobotState | None) -> str:
        if state is None or not state.connected:
            return "OFFLINE"
        return {
            RobotMode.INITIALIZING: "IDLE",
            RobotMode.READY: "READY",
            RobotMode.EXECUTING: "RUNNING",
            RobotMode.STOPPING: "STOPPING",
            RobotMode.PARKING: "RUNNING",
            RobotMode.PARKED: "STOPPED",
            RobotMode.FAULT: "FAILED",
            RobotMode.DISCONNECTED: "OFFLINE",
        }[state.mode]

    @staticmethod
    def state_age_s(state: RobotState | None, now_s: float | None = None) -> float | None:
        if state is None or state.critical_monotonic_s <= 0:
            return None
        return max(0.0, (time.monotonic() if now_s is None else now_s) - state.critical_monotonic_s)

    def pipeline_nodes(self) -> tuple[PipelineNode, ...]:
        camera = self.camera.latest()
        state = self.robot.latest()
        execution = self.execution.latest()
        validation = self.trajectory.validation
        if validation is None:
            validation_status = "NOT LOADED" if self.trajectory.error is None else "INPUT ERROR"
        else:
            validation_status = "VALID" if validation.valid else "INVALID"
        robot_status = self.display_mode(state)
        return (
            PipelineNode(
                "Camera", "CONNECTED" if camera.connected else "OFFLINE",
                "—" if camera.effective_fps is None else f"{camera.effective_fps:.1f} FPS",
                camera.connected, "isolated",
            ),
            PipelineNode("Sensing / mapping", "NOT INTEGRATED", "future boundary", False, "placeholder"),
            PipelineNode(
                "Trajectory", "LOADED" if self.trajectory.sample_count else "NO DATA",
                f"{self.trajectory.sample_count} samples" if self.trajectory.sample_count else "",
                self.trajectory.sample_count > 0,
            ),
            PipelineNode("Validator", validation_status, "Stage 2", validation_status == "VALID"),
            PipelineNode(
                "MotionExecutor", execution.status,
                execution.trajectory_id or "no run log",
                execution.status in {"MOVING", "RUNNING"},
            ),
            PipelineNode("Robot", robot_status, "RobotService", robot_status not in {"OFFLINE", "FAILED"}),
            PipelineNode(
                "RobotState / Log", "LIVE" if state and state.connected else execution.result.upper(),
                "shared state + evidence", bool(state and state.connected),
            ),
        )

    def close(self) -> None:
        self.camera.close()
        self.robot.close()
