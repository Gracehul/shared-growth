import json

import pytest

from drawing_robot.runlog import RunLog


def test_runlog_writes_common_envelope(tmp_path) -> None:
    log = RunLog(
        "shared-growth/test/v1",
        hardware={"model": "mock"},
        config={"command_rate_hz": 4},
        run_id="run-001",
    )
    log.add("COMMAND_SENT", target=[0] * 6)
    log.add("STATE_SAMPLE", actual=[0] * 6)
    log.finish("completed")
    path = tmp_path / "run.json"
    log.write(path)
    value = json.loads(path.read_text(encoding="utf-8"))
    assert set(value) == {
        "schema", "run_id", "started_utc", "hardware", "software",
        "config", "timeline", "result",
    }
    assert value["timeline"][0]["kind"] == "COMMAND_SENT"
    assert value["result"]["status"] == "completed"


def test_runlog_rejects_unknown_event() -> None:
    log = RunLog("shared-growth/test/v1")
    with pytest.raises(ValueError, match="unknown event"):
        log.add("MYSTERY")
