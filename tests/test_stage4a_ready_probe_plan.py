from drawing_robot.hardware import Stage4AConfig, motion_envelope_gate
from scripts.validate_ready_and_probe import sampled_joint_path, stage2_validator


CURRENT = (1.05, 50.71, -70.75, -63.01, -2.98, 9.58)


def test_ready_and_j6_probe_are_offline_valid_inside_temporary_workspace() -> None:
    config = Stage4AConfig()
    ready = config.ready_angles_deg
    assert ready is not None
    probe = list(ready)
    probe[config.probe_joint - 1] += config.probe_displacement_deg
    ready_cart, ready_joint = sampled_joint_path(CURRENT, ready, 1.0, samples=2)
    probe_cart, probe_joint = sampled_joint_path(
        ready, probe, config.probe_duration_s
    )
    validator = stage2_validator(config)
    ready_result = validator.validate(ready_cart, ready_joint)
    probe_result = validator.validate(probe_cart, probe_joint)
    assert ready_result.valid
    assert probe_result.valid
    assert motion_envelope_gate(ready_joint, config, ready_cart).allowed
    assert motion_envelope_gate(probe_joint, config, probe_cart).allowed
    assert config.ready_verified is False
    assert config.workspace_verified is False
