import pytest

from drawing_robot import DrawingError, DrawingWorkspace
from drawing_robot.experiment import (
    GrowthRule,
    HumanStroke,
    ProtocolEvent,
    TimingCondition,
    TimingPolicy,
    TrialMachine,
    TrialState,
)


def test_timing_conditions_keep_geometry_independent() -> None:
    assert TimingPolicy(TimingCondition.BASELINE).sample().intended_delay_s == 0
    assert TimingPolicy(
        TimingCondition.FIXED_DELAY, fixed_delay_s=1.25
    ).sample().intended_delay_s == 1.25


def test_jitter_is_bounded_and_reproducible() -> None:
    first = TimingPolicy(
        TimingCondition.JITTER, jitter_min_s=0.4, jitter_max_s=1.2, seed=7
    )
    second = TimingPolicy(
        TimingCondition.JITTER, jitter_min_s=0.4, jitter_max_s=1.2, seed=7
    )
    a = [first.sample().intended_delay_s for _ in range(5)]
    b = [second.sample().intended_delay_s for _ in range(5)]
    assert a == b
    assert all(0.4 <= value <= 1.2 for value in a)


def test_timing_parameters_must_be_explicit() -> None:
    with pytest.raises(ValueError, match="fixed_delay_s"):
        TimingPolicy(TimingCondition.FIXED_DELAY)
    with pytest.raises(ValueError, match="jitter requires"):
        TimingPolicy(TimingCondition.JITTER)


def test_human_stroke_extracts_geometry_and_timing() -> None:
    stroke = HumanStroke((1.0, 2.0), (4.0, 6.0), 10.0, 12.0)
    assert stroke.length_cm == pytest.approx(5.0)
    assert stroke.direction == pytest.approx((0.6, 0.8))
    assert stroke.duration_s == pytest.approx(2.0)
    assert stroke.mean_speed_cm_s == pytest.approx(2.5)


def test_growth_rule_starts_at_human_endpoint() -> None:
    stroke = HumanStroke((1.0, 1.0), (3.0, 1.0), 0.0, 1.0)
    response = GrowthRule(length_ratio=0.5).generate(stroke)
    assert response.segments_cm[0][0] == stroke.end_cm
    assert response.response_length_cm == pytest.approx(1.0)
    assert len(response.segments_cm) == 3


def test_growth_rule_validates_full_response_workspace() -> None:
    workspace = DrawingWorkspace(0, 4, 0, 4, drawing_z_cm=1, safe_z_cm=2)
    stroke = HumanStroke((2.0, 2.0), (3.9, 2.0), 0.0, 1.0)
    with pytest.raises(DrawingError, match="outside"):
        GrowthRule().generate(stroke, workspace=workspace)


def test_trial_state_machine_follows_case_a_turn() -> None:
    machine = TrialMachine()
    events = [
        ProtocolEvent.INITIALIZE,
        ProtocolEvent.PROMPT_HUMAN,
        ProtocolEvent.STROKE_COMPLETE,
        ProtocolEvent.RESPONSE_READY,
        ProtocolEvent.DELAY_ELAPSED,
        ProtocolEvent.ROBOT_COMPLETE,
    ]
    for event in events:
        machine.apply(event)
    assert machine.state is TrialState.COMPLETE
    assert machine.apply(ProtocolEvent.NEXT_TURN) is TrialState.READY


def test_fault_can_only_transition_to_parked() -> None:
    machine = TrialMachine(TrialState.ROBOT_DRAWING)
    assert machine.apply(ProtocolEvent.FAULT) is TrialState.FAULT
    with pytest.raises(ValueError, match="invalid protocol transition"):
        machine.apply(ProtocolEvent.NEXT_TURN)
    assert machine.apply(ProtocolEvent.PARK) is TrialState.PARKED
