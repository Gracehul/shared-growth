# Motion test plan

## Design position

Shared Growth should combine two traditions rather than choose between them:

1. **Robot safety and control engineering** supplies bounded workspaces,
   conservative speeds, stop behavior, fault handling, repeatability, and
   measured rather than assumed latency.
2. **Animation and HRI motion design** supplies readable intent, anticipation,
   timing, spacing, pauses, easing, and controlled stylistic variation.

The myCobot setup is an educational research platform, not an industrial robot
cell. ISO 10218 and ISO/TS 15066 are used as design references; this project
must not claim compliance or safety certification without a formal risk
assessment, validated safety functions, and qualified review.

## Test ladder

Each level is a gate. A failed test returns to the same level after diagnosis;
it does not advance to a more complex trajectory.

### L0 — Static inspection (complete)

- Read model, firmware, power, servo, joint, pose, limit, and error state.
- Compare controller FK with client-side FK.
- Record the physical configuration and attached tool.

### L1 — Joint-limit recovery (complete for current setup)

- Move one joint at a time, away from a known limit.
- Maximum planned displacement: 10 degrees per test.
- Minimum practical speed, continuous telemetry, timeout, and final stop.
- Pass: error `0`, stable final state, expected direction, no contact or sound.

Completed 2026-09-25: J3 reached `-145.98` degrees, remained stable, and the
controller error remained `0`. Do not return toward the lower limit during
development tests.

### L2 — Local latency and repeatability

Run a persistent process on the Nano so network and chat delay are excluded.
For a small, safe joint such as J6, execute repeated `+2 / -2` degree motions.

Record with a monotonic clock:

- command timestamp;
- first measured change above a noise threshold;
- target-entry timestamp;
- settled timestamp;
- overshoot and final error;
- controller error codes and all joint telemetry.

Test conservative firmware speeds such as `2`, `5`, and `10`, one variable at
a time. At least five repetitions per condition are needed before selecting a
development speed.

Status 2026-09-25: completed for small J6 motions, including bounded retries
for sporadic invalid serial replies. Endpoint error and local timing are now
measured; results are in `HARDWARE_TEST_LOG.md`.

### L3 — Stop and recovery behavior

- Normal stop while holding position.
- Stop during a small, slow, pen-up motion.
- Recovery from a deliberately injected software timeout, without creating a
  collision or disconnecting power.
- Pass: motion ceases within a measured threshold and no queued command resumes.

This validates application-level behavior only; it is not a safety-rated stop
test.

Status 2026-09-25: one small J6 application-stop test completed. Additional
movement after first detection was measured at `1.94 deg`; this must inform
development clearance. Repetition is deferred until the J4 thermal anomaly is
resolved.

### L4 — Pen-up Cartesian primitives

- Resolve or bound the FK discrepancy across at least five safe static poses.
- Calibrate tool offset, safe Z, drawing Z, and XY bounds.
- Execute a short line above the paper, followed by a small L and Y shape.
- Compare planned and measured joint paths and camera-observed TCP paths.

Status 2026-09-25: blocked. Three nearby FK samples showed a nearly constant
11.34 mm position error, but J4 did not follow a small command and reached
66 C while holding. No Cartesian motion is allowed until that condition is
understood and the project temperature gate passes.

### L5 — Motion-design comparison

Keep the geometric path and total duration fixed while varying only temporal
spacing. Candidate profiles:

- **Functional:** current controller baseline.
- **Ease:** smooth acceleration and deceleration (`smoothstep` or quintic).
- **Deliberate:** short pre-movement hold plus clear early commitment toward
  the selected branch.
- **Responsive:** timing scaled to the preceding human stroke, within fixed
  safety bounds.

Avoid animation-style overshoot or opposite-direction anticipation during early
hardware tests. First evaluate those variants in simulation or video because
they add path length and may reduce clearance.

Measures include peak joint step, settling time, path error, perceived
smoothness, predictability, legibility, confidence, and control. Predictability
and legibility are separate properties: a shortest expected path can be
predictable while an exaggerated early directional cue can make the goal more
legible.

Offline profiles are implemented as `functional`, `smooth`, `deliberate`, and
`early_commitment`. They preserve geometry and total duration. Physical
execution remains gated by L4.

### L6 — Pen contact and drawing

- Begin with dots and 10–20 mm lines at minimal contact force.
- Measure spring travel, line continuity, paper movement, and path error.
- Progress to a Y-branch only after repeated pen-up and contact-line passes.

### L7 — Shared Growth timing study

- Run the complete loop locally on the Nano.
- Separate **response-delay manipulation** from **within-motion profile**.
- Log intended delay, actual response onset, movement duration, and settling.
- Compare baseline, fixed delay, and jitter without changing motion style.
- Treat expressive/adaptive motion as a later factor so it does not confound
  the initial timing probe.

## References

- [ISO 10218-1:2025 — Safety requirements for industrial robots](https://www.iso.org/standard/73933.html)
- [ISO/TS 15066:2016 — Collaborative robot systems](https://www.iso.org/standard/62996.html)
- [ISO overview of collaborative methods](https://www.iso.org/news/2016/03/Ref2057.html)
- [Dragan, Lee, and Srinivasa — Legibility and Predictability of Robot Motion](https://publications.ri.cmu.edu/legibility-and-predictability-of-robot-motion)
- [Takayama et al. — Improving robot readability with animation principles](https://ieeexplore.ieee.org/document/6281390/)
- [Expressive Robot Motion Timing](https://ieeexplore.ieee.org/document/8534737/)
- [Disney Research — Vibration-Minimizing Motion Retargeting](https://la.disneyresearch.com/publication/publication-process-vibration-minimizing-motion-retargeting-for-robotic-characters/)
