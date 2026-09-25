# Experiment protocol — draft

This document is a preregistration-ready scaffold, not an approved study
protocol. Sample size, recruitment, consent, compensation, ethics approval, and
exclusion rules remain `TBD`.

## Task and setting

Shared Growth is a tangible human-robot co-drawing task. The participant draws
a trunk or stem segment on a fixed shared surface. The robot responds in a
different color by drawing a generated branch. Human and robot repeatedly build
on the evolving drawing.

## Primary research question

Does robot response timing influence human drawing behavior and the experienced
responsiveness, predictability, control, and collaboration of the interaction?

## Case A conditions

| Condition | Intended behavior |
| --- | --- |
| Baseline | Respond as soon as processing and safety checks complete |
| Fixed delay | Add one predefined delay (`TBD` duration) |
| Jitter | Add a sampled delay from a predefined distribution (`TBD`) |

The condition changes response timing only. Geometry and robot motion parameters
should remain controlled unless explicitly included as factors.

## Trial sequence

1. Initialize the session, calibration, condition, and synchronized logger.
2. Confirm the safety checklist and move the robot to its ready pose.
3. Signal that the participant may draw one segment.
4. Detect stroke completion and extract its geometry and timing.
5. Generate and validate a bounded robot branch trajectory.
6. Apply the assigned timing condition.
7. Execute the trajectory and record actual robot timing and pose telemetry.
8. Return control to the participant and repeat for the configured turns.
9. Complete experience measures and a short debrief.

## Measures

Behavioral candidates:

- participant stroke duration, length, speed, and direction;
- pause before the participant's next stroke;
- turn-to-turn timing variability;
- spatial continuation or avoidance relative to the robot branch;
- adaptation across turns and conditions.

Experience candidates:

- perceived responsiveness;
- predictability;
- sense of control;
- perceived collaboration or coordination;
- frustration, comfort, and preference.

Final items, scales, hypotheses, and analysis models are `TBD` and should be
chosen before confirmatory data collection.

## Pilot stages

1. **Technical pilot:** researchers only; verify safety, timing, and logs.
2. **Interaction pilot:** a small convenience sample; inspect task clarity and
   experience measures without confirmatory claims.
3. **Study:** only after protocol, ethics, exclusion rules, and analysis plan are
   fixed.

## Failure and exclusion logging

Log failures rather than silently retrying. Examples include missed strokes,
incorrect registration, unreachable trajectories, safety stops, communication
loss, and condition timing outside tolerance. Participant-level exclusions must
be defined before confirmatory analysis and must never be inferred solely from
whether a result supports the hypothesis.

## Privacy and data management

Use pseudonymous session and participant identifiers. Store consent and the
re-identification key separately from research data. Raw recordings and
physiological signals must not be committed to this repository. Case C requires
an explicit ethics and data-management update before collection.
