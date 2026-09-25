# Roadmap

## Delivery strategy

Shared Growth is developed as progressive, testable levels. The levels are not
three parallel promises: each one depends on the previous level being stable.

## Foundation — Robot Drawing Base

**Goal:** make the myCobot 280 JN draw repeatable marks safely.

Exit criteria:

- the pen is mechanically secure;
- home, working, pen-up, and pen-down poses are documented;
- a 5–10 cm line can be repeated consistently;
- a simple Y-branch can be drawn;
- table, paper, gripper, and arm collisions are avoided;
- start, stop, and recovery procedures have been tested;
- planned and actual robot motion can be logged.

Hardware checkpoint (2026-09-25): the read-only connection, controller limits,
error-state inspection, authenticated remote development link, and first
bounded J3 recovery movement have been verified. The arm is now inside its
reported J3 limit with controller error `0`. Joint-margin, latency,
repeatability, stop, multi-pose FK, and pen-up Cartesian tests remain open; see
`HARDWARE_TEST_LOG.md` and `MOTION_TEST_PLAN.md`.

## Case A — Minimal / Timing Probe

**Research focus:** does temporal responsiveness affect participant behavior and
experience?

Required capabilities:

- detect a human stroke and extract start, end, direction, length, and time;
- generate a bounded branch trajectory;
- implement baseline, fixed-delay, and jitter conditions;
- record intended delay, actual end-to-end latency, geometry, and trial state;
- complete a small experience pilot.

Offline checkpoint (2026-09-25): the stroke data model, deterministic bounded
growth rule, explicit trial state machine, and reproducible baseline/fixed/
jitter timing policies are implemented and unit-tested. Camera extraction,
calibrated workspace mapping, robot integration and pilot measures remain open.

Case A is the committed three-week target after the Robot Drawing Base is safe.

## Case B — Medium / Coordination-aware Adaptation

**Research focus:** can the system adapt to the participant's interaction state?

Entry gate:

- Case A runs reliably across repeated trials;
- timestamps and latency measurements are trustworthy;
- one interpretable adaptation rule can be isolated from the timing condition.

Candidate inputs include drawing tempo, pause duration, movement variability,
and turn-to-turn coordination. Only one robot property should be adapted in the
first implementation to preserve interpretability.

## Case C — Ambitious / Physiology-informed Adaptation

**Research focus:** can movement, coordination, and physiology jointly inform an
adaptive robot response?

Entry gate:

- Case B is stable;
- sensors, synchronization, consent, privacy, and secure storage are ready;
- the signal has a defensible interpretation and does not create unsupported
  claims about participant state.

Possible signals include EDA, heart rate, or respiration. Case C is optional and
must not delay a valid Case A result.

## Three-week priority rule

When time is constrained, prioritize in this order:

1. physical safety and repeatable robot drawing;
2. a complete Case A loop with trustworthy timestamps;
3. a small pilot and documented limitations;
4. one Case B adaptation;
5. Case C only if every earlier gate is satisfied.
