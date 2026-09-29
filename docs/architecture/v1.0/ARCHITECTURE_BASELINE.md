# Shared Growth — Architecture Baseline v1.0

**Status:** FROZEN  
**Project:** Shared Growth  
**Baseline scope:** Robot Interaction Workstation + Stage-4A execution stack  
**Branch:** `feature/robot-interaction-workstation-integration`  
**Semantics migration:** `2d37fee` — `refactor: clarify gate and operation semantics`  
**Legacy cleanup:** `758d139` — `refactor: remove legacy execution and run logging`  
**Validation at freeze:** `201 passed, 1 intentionally skipped`

---

## 1. Purpose

This document records the frozen architecture baseline after the gate/operation semantics migration and legacy cleanup.

The baseline is intended to prevent architecture drift while Shared Growth moves from infrastructure work into:

- hardware evidence and motion qualification,
- perception integration,
- drawing / growth mapping,
- Research Case A.

A change to the architecture after this freeze should be driven by observed evidence, a concrete research requirement, or a safety requirement not represented by the current system.

---

## 2. Frozen architectural rule

> One command boundary, one backend contract, one hardware owner, one execution evidence model.

The authoritative command path is:

```text
Workstation
→ OperationController
→ Gate / Policy
→ MotionExecutor
→ RobotInterface
→ SimRobot | MyCobotRobot
→ RobotService
→ RobotIO
→ UART
→ myCobot
```

`RobotIO` remains the exclusive UART owner.

No browser component, debug script, calibration tool, interaction module, or future perception component may bypass this path.

---

## 3. Authoritative truth ownership

The system preserves four distinct authoritative roles:

```text
RobotState
= observed robot truth

OperationReceipt
= authoritative request decision

ExecutionLog
= authoritative execution evidence

SystemSnapshot
= derived read model for UI / monitoring
```

### Invariant

`SystemSnapshot` is a projection and must not become a second source of truth.

The UI may display derived state, but gate logic, execution logic, and hardware safety must not infer authoritative state from the UI/read model.

---

## 4. Interaction semantics

The frozen interaction model distinguishes five records:

### 4.1 Current Gate

```text
CURRENT GATE
= Can an operation run under current system conditions?
```

Live and continuously changing.

It is informative and does not authorize a future request.

### 4.2 Intended Operation

```text
INTENDED OPERATION
= What has the operator selected?
```

Selection alone is not a request and creates no operation decision.

Example:

```text
AVAILABLE
→ selected GO_TO_READY
→ NOT REQUESTED
→ NOT SENT
→ current measured state
```

### 4.3 Operation Decision

```text
OPERATION DECISION
= What did the authoritative request boundary decide?
```

Request-scoped and immutable once made.

Allowed values:

```text
ACCEPTED
ACCEPTED_WITH_WARNING
REJECTED
```

### 4.4 Commanded

```text
COMMANDED
= Did an accepted request actually enter execution?
```

Important invariant:

```text
ACCEPTED ≠ COMMAND_SENT
```

A request may be accepted without a command ever reaching execution.

### 4.5 Actual / Last Action

```text
ACTUAL / LAST ACTION
= What physically happened?
```

This is independent of Current Gate and Operation Decision.

Rejected requests that never entered execution must not replace `LAST ACTION`.

---

## 5. Gate semantics

The frozen gate vocabulary is:

```text
HARD
PASS | BLOCK

PERFORMANCE
NOMINAL | WARN | DEGRADED
```

### Admission behavior

```text
HARD/BLOCK
→ execution prohibited

PERFORMANCE/NOMINAL
→ execution permitted

PERFORMANCE/WARN
→ execution permitted with warning

PERFORMANCE/DEGRADED
→ execution permitted, non-nominal
```

If a condition must prohibit execution, its operational semantics are `HARD/BLOCK`, regardless of whether its technical origin is performance-related.

Invalid category/status combinations are rejected by the implementation.

---

## 6. Admission is not runtime validity

Request-time admission and runtime validity are separate questions.

```text
Gate / Policy
= May this request enter execution now?

MotionExecutor + runtime checks
= May execution continue?

ExecutionLog + RobotState
= What actually happened?
```

The existing runtime path continues to observe conditions such as:

- telemetry freshness,
- controller fault,
- temperature abort,
- timeout,
- stop request,
- communication failure.

No separate watchdog service is introduced as part of this baseline.

A separate watchdog may be considered only if later evidence shows that the current execution path cannot represent a required runtime safety condition.

---

## 7. Sim / Hardware equivalence

Both backends share the same execution contract:

```text
RobotInterface
├── SimRobot
└── MyCobotRobot
```

Divergence occurs only below `RobotInterface`.

### SimRobot

- deterministic offline backend,
- used for UI, policy, operation-contract and interaction development,
- must preserve realistic state / timing / stop semantics rather than behaving as an unconditional success stub.

### MyCobotRobot

- hardware adapter,
- uses the existing Stage-4A path,
- delegates serialized command / telemetry ownership to `RobotService → RobotIO`.

---

## 8. Evidence chain

The frozen evidence chain is:

```text
Perceived
→ Intended
→ Gate Decision
→ Commanded
→ Actual
```

Minimum correlation support currently includes:

```text
request_id
run_id
command_id
state timestamp / sequence
```

Optional identifiers such as `perception_id` and `intent_id` are used only when those stages actually exist.

Missing stages remain explicitly:

```text
ABSENT
UNKNOWN
```

They must never be synthesized by the UI.

---

## 9. Preserved components

The following components are part of the frozen baseline:

- `Stage-2 ValidationResult`
- `OperationController`
- Gate / Policy layer
- authoritative `MotionExecutor`
- `RobotInterface`
- `SimRobot`
- `MyCobotRobot`
- `RobotService`
- `RobotIO`
- `RobotState`
- `ExecutionLog`
- `SystemSnapshotAdapter`
- `OperationReceipt`
- Workstation HTTP boundary

These components have distinct responsibilities and should not be merged or duplicated without evidence that the current separation creates a concrete problem.

---

## 10. Removed legacy paths

The following legacy components have been removed:

- legacy `drawing_robot.motion.MotionExecutor`
- legacy `RunLog`

Useful test intent was migrated before removal.

Historical log files remain immutable and are not rewritten into the current schema.

`ExecutionLog` is the active execution evidence model.

---

## 11. Deferred — not part of the frozen baseline

The following are intentionally not part of Architecture Baseline v1.0:

- ROS 2 / DDS
- ZeroMQ / Redis message bus
- microservices
- separate watchdog service
- teleop / joystick control
- dynamic human-proximity safety
- workspace calibration workflow
- full perception / `WorldState`
- multi-robot orchestration
- time-series database
- React / Node rewrite

Deferred does not mean rejected permanently.

A deferred capability may be introduced when an observed constraint or research requirement justifies it.

---

## 12. Deployment baseline

### Hardware composition

```text
Laptop
└── Browser / Robot Interaction Workstation UI
        │
        │ HTTP
        ▼
Jetson Nano
├── Workstation backend
├── SystemSnapshotAdapter
├── OperationController
├── MotionExecutor
├── MyCobotRobot
├── RobotService
└── RobotIO
        │
        │ UART /dev/ttyTHS1
        ▼
myCobot 280 JN
```

### Offline composition

```text
Laptop / development host
├── Browser / Workstation
├── Workstation backend
├── OperationController
├── MotionExecutor
└── SimRobot
```

The browser never receives a raw UART, servo, joint-jog, or `pymycobot` command surface.

---

## 13. Architecture-change rule

After this freeze, a new architectural component requires evidence of at least one of the following:

1. a real observed failure or bottleneck;
2. a research requirement that cannot be met cleanly by the current architecture;
3. a safety requirement not represented by the current system.

Review question:

> Does this reduce a real uncertainty, or only describe another hypothetical failure?

If it only describes a hypothetical future problem, classify it as `LATER` or `DROP`.

---

## 14. Frozen status

```text
Backend semantics       FROZEN
Command path            FROZEN
State authority         FROZEN
Evidence model          FROZEN
UI / backend boundary   FROZEN
```

Architecture Baseline v1.0 should now change only when hardware or research evidence justifies a revision.

---

## 15. Next work after freeze

The next system work is driven by:

```text
HARDWARE EVIDENCE
→ MOTION QUALITY
→ PERCEPTION INTEGRATION
→ DRAWING / GROWTH MAPPING
→ RESEARCH CASE A
```

Architecture work is no longer the default activity.
