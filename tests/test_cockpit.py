import json
import time
from types import SimpleNamespace

from drawing_robot.cockpit import (
    CameraSource,
    CockpitModel,
    ExecutionLogSource,
    RobotStateSource,
    SOURCE_MAPPING,
    TrajectorySource,
    Workspace,
)
from drawing_robot.execution import ExecutionConfig, ExecutionLog, RobotState as ExecutionRobotState, RobotStatus
from drawing_robot.robot.state import RobotMode, RobotState


def test_source_mapping_uses_existing_authoritative_contracts() -> None:
    text = "\n".join(" | ".join(item) for item in SOURCE_MAPPING)
    assert "RobotService" in text
    assert "RobotState" in text
    assert "Stage-2B bundle" in text
    assert "ExecutionLog" in text


def test_camera_snapshot_reports_age_without_robot_dependency() -> None:
    source = CameraSource(mock=True, width=32, height=24, requested_fps=10)
    source.start()
    try:
        deadline = time.monotonic() + 1.0
        while not source.latest().connected and time.monotonic() < deadline:
            time.sleep(0.01)
        sample = source.latest()
        assert sample.connected
        assert sample.resolution == (32, 24)
        assert sample.frame_bgr.shape == (24, 32, 3)
        assert sample.frame_age_s() is not None
    finally:
        source.close()


def test_model_keeps_core_robot_state_as_authoritative_type() -> None:
    state = RobotState(
        monotonic_s=10.0,
        critical_monotonic_s=10.0,
        mode=RobotMode.READY,
        connected=True,
        angles_deg=(1, 2, 3, 4, 5, 6),
        temperatures_c=(30, 31, 32, 33, 34, 35),
    )

    class Source:
        def latest(self): return state
        def close(self): pass

    model = CockpitModel(CameraSource(), Source(), TrajectorySource(), ExecutionLogSource())
    assert model.robot.latest() is state
    assert model.display_mode(model.robot.latest()) == "READY"


def test_execution_log_source_supplies_recorded_target_and_result(tmp_path) -> None:
    initial = ExecutionRobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog("demo", ExecutionConfig().to_dict(), 0, initial)
    from drawing_robot.execution import JointCommand
    command = JointCommand("demo:0", [1, 2, 3, 4, 5, 6], 0)
    log.command_sent(command, 0)
    log.finish("completed", 1)
    path = log.write(tmp_path / "run.json")
    sample = ExecutionLogSource(path).latest()
    assert sample.target_angles_deg == (1, 2, 3, 4, 5, 6)
    assert sample.result == "completed"
    assert sample.events[-1][1] == "COMMAND_SENT"


def test_application_shell_exposes_four_lightweight_workspaces() -> None:
    assert [item.value for item in Workspace] == ["AUTHOR", "PREVIEW", "RUN", "REVIEW"]


def test_cockpit_app_name_remains_compatible() -> None:
    from drawing_robot.cockpit.app import ApplicationShell, CockpitApp
    assert CockpitApp is ApplicationShell


def test_pipeline_marks_camera_mapping_as_not_integrated() -> None:
    model = CockpitModel(CameraSource(), RobotStateSource(), TrajectorySource(), ExecutionLogSource())
    nodes = model.pipeline_nodes()
    mapping = next(node for node in nodes if node.name == "Sensing / mapping")
    assert mapping.status == "NOT INTEGRATED"
    assert mapping.kind == "placeholder"


def test_cockpit_modules_have_no_direct_hardware_or_executor_stack() -> None:
    from pathlib import Path
    root = Path(__file__).parents[1] / "drawing_robot" / "cockpit"
    code = "\n".join(path.read_text(encoding="utf-8") for path in root.glob("*.py"))
    assert "import pymycobot" not in code
    assert "MyCobot280(" not in code
    assert "RobotIO(" not in code
    assert "MotionExecutor(" not in code
