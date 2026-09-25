# Contributing

Thank you for contributing to Shared Growth. This repository combines physical
robot control and human-participant research, so changes must be reviewable,
safe, and reproducible.

## Roles and decisions

Gracehul leads the project, HRI design, research protocol, and final scope
decisions. Contributors own the implementation work they submit and should
document assumptions, limitations, and hardware observations.

## Workflow

1. Start from the latest `main` branch.
2. Create a focused branch such as `feat/camera-registration`,
   `fix/safe-z-recovery`, or `docs/experiment-protocol`.
3. Keep each change small enough to review and describe how it was verified.
4. Open a pull request into `main`; do not push experimental work directly to
   `main`.
5. Record unresolved calibration values as `TBD` rather than estimating them.

## Verification

For software-only changes, run the relevant offline checks:

```bash
python3 -m py_compile drawing_robot/*.py scripts/*.py
python3 -c "from drawing_robot import Arm, config, ik, kinematics"
python3 scripts/example.py --mock
```

Hardware changes require, in order:

1. offline or mock verification;
2. a pen-up dry run at reduced speed;
3. workspace and emergency-stop checks;
4. a supervised drawing test;
5. a short note describing the actual hardware result.

## Data and privacy

Do not commit participant identifiers, consent records, video/audio recordings,
biometric signals, raw sensor logs, or generated participant datasets. Store
only schemas, synthetic examples, and analysis code in this repository unless
an approved data-management plan explicitly permits otherwise.

## Commit and pull-request content

A contribution should state:

- what changed and why;
- whether robot hardware was used;
- which safety and verification steps were completed;
- known limitations or remaining `TBD` values.
