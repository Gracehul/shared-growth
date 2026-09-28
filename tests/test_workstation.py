import ast
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from urllib.request import Request, urlopen

import pytest

from drawing_robot.execution import ExecutionConfig, RobotStatus, SimRobot, SimulationClock
from drawing_robot.hardware import ExecutionPhase, SystemSnapshotAdapter
from drawing_robot.operations import (
    InteractionMode,
    Operation,
    OperationController,
    PreparedTrajectory,
)
from drawing_robot.robot.state import RobotMode, RobotState as ServiceRobotState
from drawing_robot.stage2 import JointTrajectory, ValidationResult
from drawing_robot.workstation import (
    API_SCHEMA_VERSION,
    WorkstationApplication,
    WorkstationOperation,
    create_workstation_server,
)


def workstation():
    clock = SimulationClock(1.0)
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=50,
        joint_velocity_limits_deg_s=(20.0,) * 6,
        position_tolerance_deg=0.05,
        trajectory_completion_timeout_s=1.0,
    )
    robot = SimRobot(clock, config, [0.0] * 6)
    validation = ValidationResult(True)
    ready = PreparedTrajectory(
        "ready",
        JointTrajectory.from_arrays(
            [0.0, 0.1], [[0.0] * 6, [0.0] * 6]
        ),
        validation,
    )
    demo = PreparedTrajectory(
        "demo",
        JointTrajectory.from_arrays(
            [0.0, 0.1], [[0.0] * 6, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        ),
        validation,
    )

    def state():
        measured = robot.get_state()
        mode = {
            RobotStatus.IDLE: RobotMode.READY,
            RobotStatus.MOVING: RobotMode.EXECUTING,
            RobotStatus.STOPPING: RobotMode.STOPPING,
            RobotStatus.STOPPED: RobotMode.PARKED,
            RobotStatus.FAULT: RobotMode.FAULT,
        }[measured.status]
        return ServiceRobotState(
            timestamp_utc=datetime.now(timezone.utc).isoformat(),
            monotonic_s=clock.now(),
            critical_monotonic_s=clock.now(),
            sequence=1,
            mode=mode,
            connected=True,
            powered=True,
            servos_enabled=True,
            angles_deg=tuple(measured.actual_angles_deg),
            temperatures_c=(35.0,) * 6,
            servo_status=(0,) * 6,
            controller_error=0,
        )

    def phase():
        return {
            RobotStatus.IDLE: ExecutionPhase.IDLE,
            RobotStatus.MOVING: ExecutionPhase.RUNNING,
            RobotStatus.STOPPING: ExecutionPhase.STOPPING,
            RobotStatus.STOPPED: ExecutionPhase.STOPPED,
            RobotStatus.FAULT: ExecutionPhase.FAILED,
        }[robot.get_state().status]

    adapter = SystemSnapshotAdapter(
        state,
        validation_provider=lambda: validation,
        target_provider=lambda: tuple(robot.get_state().commanded_angles_deg),
        phase_provider=phase,
        now_provider=clock.now,
    )
    controller = OperationController(
        robot,
        clock,
        config,
        {"ready": ready, "demo": demo},
        gate_provider=adapter.get_gate_results,
        ready_trajectory_id="ready",
        mode=InteractionMode.OBSERVE,
    )
    return WorkstationApplication(
        adapter,
        controller,
        (
            WorkstationOperation("ready", "GO TO READY", Operation.GO_READY),
            WorkstationOperation(
                "demo", "TEST MOTION", Operation.EXECUTE_TRAJECTORY, "demo"
            ),
            WorkstationOperation(
                "shared", "START SHARED GROWTH", Operation.START_SHARED_GROWTH
            ),
        ),
        backend_label="test core",
    )


def test_status_uses_existing_read_side_contract() -> None:
    app = workstation()
    status = app.status()
    assert status["schema_version"] == API_SCHEMA_VERSION
    assert status["snapshot"]["execution_phase"] == "IDLE"
    assert status["snapshot"]["q_actual_deg"] == [0.0] * 6
    assert status["gate_results"] == status["snapshot"]["gate_results"]
    assert "ready_tolerance_deg" in status["thresholds"]
    assert status["latest_receipt"] is None
    assert "calibrate-workspace" not in {
        item["operation_id"] for item in status["operations"]
    }


def test_observe_rejection_and_manual_execution_are_authoritative() -> None:
    app = workstation()
    rejected = app.request_operation(
        {"request_id": "observe-ready", "operation_id": "ready"}
    )
    assert rejected["accepted"] is False
    assert rejected["reason"] == "MODE_PERMISSION_DENIED"

    assert app.set_mode("MANUAL") == {"mode": "MANUAL"}
    receipt = app.request_operation(
        {"request_id": "manual-demo", "operation_id": "demo"}
    )
    assert receipt["accepted"] is True
    assert receipt["state"] == "COMPLETED"
    assert receipt["run_id"]
    assert receipt["evidence"]["command_ids"] == ["demo:000000", "demo:000001"]
    assert app.status()["latest_receipt"]["request_id"] == "manual-demo"


def test_shared_growth_keeps_absent_evidence_explicit() -> None:
    app = workstation()
    app.set_mode("SHARED_GROWTH")
    receipt = app.request_operation(
        {"request_id": "shared-1", "operation_id": "shared"}
    )
    assert receipt["state"] == "SHARED_GROWTH_ACTIVE"
    assert receipt["evidence"]["perception"] == "ABSENT"
    assert receipt["evidence"]["intended"] == "ABSENT"
    assert receipt["evidence"]["actual"] == "UNKNOWN"


def test_command_api_rejects_raw_motion_fields() -> None:
    app = workstation()
    app.set_mode("MANUAL")
    with pytest.raises(ValueError, match="target_angles_deg"):
        app.request_operation(
            {
                "request_id": "raw-motion",
                "operation_id": "demo",
                "target_angles_deg": [1, 2, 3, 4, 5, 6],
            }
        )


def test_stop_is_global_and_preserves_confirmation_receipt() -> None:
    app = workstation()
    receipt = app.stop({"request_id": "stop-observe"})
    assert receipt["accepted"] is True
    # The existing SimRobot stop delay has not advanced yet.  The adapter must
    # preserve that uncertainty instead of upgrading an API return to STOPPED.
    assert receipt["state"] == "STOPPING"
    confirmation = next(
        item for item in receipt["gate_results"] if item["name"] == "stop_confirmation"
    )
    assert confirmation["status"] == "UNKNOWN"


def test_http_adapter_serves_frontend_and_contract(tmp_path: Path) -> None:
    (tmp_path / "index.html").write_text("<h1>workstation</h1>", encoding="utf-8")
    server = create_workstation_server(workstation(), tmp_path, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(f"{base}/api/workstation/status") as response:
            status = json.load(response)
        assert status["backend"] == "test core"
        request = Request(
            f"{base}/api/workstation/mode",
            data=json.dumps({"mode": "MANUAL"}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request) as response:
            assert json.load(response) == {"mode": "MANUAL"}
        with urlopen(f"{base}/") as response:
            assert b"workstation" in response.read()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_http_adapter_and_frontend_have_no_hardware_command_dependency() -> None:
    root = Path(__file__).parents[1]
    backend = root / "drawing_robot" / "workstation.py"
    tree = ast.parse(backend.read_text(encoding="utf-8"))
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append(node.module or "")
    assert not any(name.startswith(("pymycobot", "serial")) for name in imports)
    backend_text = backend.read_text(encoding="utf-8")
    frontend_text = (
        root / "frontend" / "robot-interaction-workstation" / "app.js"
    ).read_text(encoding="utf-8")
    frontend_html = (
        root / "frontend" / "robot-interaction-workstation" / "index.html"
    ).read_text(encoding="utf-8")
    assert "CALIBRATE WORKSPACE" in frontend_text
    assert 'id="robot-pose-source"' in frontend_html
    assert 'id="current-robot-pose"' in frontend_html
    for forbidden in ("RobotIO", "send_angles", "release_all_servos", "pymycobot"):
        assert forbidden not in backend_text
        assert forbidden not in frontend_text


def test_frontend_separates_current_gates_from_last_action_receipt() -> None:
    root = Path(__file__).parents[1] / "frontend" / "robot-interaction-workstation"
    script = (root / "app.js").read_text(encoding="utf-8")
    html = (root / "index.html").read_text(encoding="utf-8")
    assert "const gate = primaryGate();" in script
    assert "receipt?.gate_results?.find" not in script
    assert "Last action result" in html
    assert "CURRENT GATE" in html
    assert "LAST ACTION" in html
