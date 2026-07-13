# Plan: Java-fidelity rewrite of the coordination core

**Goal:** Fix every divergence from the original Java `coordination_oru` found in the analysis (CS-identity/priority keying bug, missing CS obsoletion, missing forward model, weak deadlock handling, missing parking/stopping-point/artificial-dependency machinery, ghost-envelope bug) while renaming every ported class, method, and field to **exactly** match the Java original.

**Reference:** `github.com/FedericoPecora/coordination_oru` @ commit `7332cdeee220129c44aa3318e4812966fc85f261`. Clone it locally before starting and consult it for every signature — when this plan and the Java source disagree, **the Java source wins**. (A clone may exist at the session scratchpad under `scratchpad/coordination_oru`; re-clone if gone.)

## Approach

Rewrite the coordinator layer as a faithful port of the Java class structure (`AbstractTrajectoryEnvelopeCoordinator` → `TrajectoryEnvelopeCoordinator` → `TrajectoryEnvelopeCoordinatorSimulation`), keeping the existing **asyncio** runtime: `synchronized(x)` blocks → one `asyncio.Lock` per Java monitor object, tracker threads → asyncio tasks, `spawnWaitingThread` → `asyncio.create_task`. Nomenclature rule: anything with a Java counterpart keeps the Java name verbatim (camelCase — `CSToDepsOrder`, `updateDependencies`, `getCriticalPoint`, `te1Start`); async plumbing with no Java counterpart may stay pythonic. Deadlock re-planning is ported as a **pluggable `AbstractMotionPlanner` interface only** (no bundled planner; `breakDeadlocksByReplanning` is a no-op unless a planner is injected). Local reordering and global avoidance are fully implemented. This is a breaking API change — all tests/examples are updated, version bumps to 0.3.0. Java's `util/` visualization classes (BrowserVisualization etc.) are **out of scope**; the existing pyglet viewer is adapted to the new API, not renamed.

Scope of the naming mandate: classes in the Java `coordination_oru` package are mirrored exactly. Classes owned by the metaCSP library (`TrajectoryEnvelope`, `SpatialEnvelope`, `Pose`, `PoseSteering`, `Trajectory`) keep their current Python implementation but **gain the Java-named accessors the coordinator code calls** (`getRobotID()`, `getID()`, `getPathLength()`, `getSpatialEnvelope()`, `getFootprint()`, `makeFootprint(...)`, `getTrajectory().getPose()`, `getTrajectory().getPoseSteering()`), so ported coordinator code reads like the Java line-for-line.

## Changes

Module layout mirrors the Java package (one class per module, flat under `coordination_oru/`):

- `coordination_oru/critical_section.py` — `CriticalSection`: fields `te1, te2, te1Start, te1End, te2Start, te2End, te1Break, te2Break`; getters/setters as in Java; `__eq__`/`__hash__` porting Java `equals`/`hashCode` **including index ranges and the symmetric te1↔te2 swap**. This kills the frozenset-of-envelope-ids keying bug.
- `coordination_oru/dependency.py` — new `Dependency`: ctor `(teWaiting, teDriving, waitingPoint, thresholdPoint)`; `getWaitingRobotID/getDrivingRobotID/getWaitingPoint/getReleasingPoint/getWaitingPose/getReleasingPose/getWaitingTrajectoryEnvelope/getDrivingTrajectoryEnvelope`; `compareTo` → `__lt__` (+ `__eq__`/`__hash__` per Java).
- `coordination_oru/robot_report.py` — `RobotReport` re-shaped to Java fields (`robotID, pose, velocity, pathIndex, distanceTraveled, criticalPoint`) with Java getters. `criticalPoint` in the report is required by the `communicatedCPs` handshake.
- `coordination_oru/robot_at_critical_section.py` — `RobotAtCriticalSection` (report + CS pair used by heuristics).
- `coordination_oru/forward_model.py` — `ForwardModel` (ABC): `canStop(te, currentState, targetPathIndex, useVelocity)`, `getEarliestStoppingPathIndex(te, currentState)`; plus `ConstantAccelerationForwardModel` ported numerically from Java.
- `coordination_oru/abstract_trajectory_envelope_coordinator.py` — `AbstractTrajectoryEnvelopeCoordinator`: fields `trackers, solver, allCriticalSections, CSToDepsOrder, escapingCSToWaitingRobotIDandCP, communicatedCPs, currentDependencies, artificialDependencies, stoppingPoints, stoppingTimes, stoppingPointTimers, muted, yieldIfParking, checkEscapePoses, criticalSectionCounter, envelopesToTrack, currentParkingEnvelopes`; methods `computeCriticalSections()`, static `getCriticalSections(...)`, `getCriticalPoint(yieldingRobotID, cs, leadingRobotCurrentPathIndex)`, `getForwardModel(robotID)`, `placeRobot(...)`, `addMissions(...)`, `spawnWaitingThread(...)` (as asyncio task), `cleanUpRobotCS(...)`, `isAhead(...)`, `atStoppingPoint(...)`, abstract `updateDependencies()` — mirror the full mission/tracker lifecycle from the Java source.
- `coordination_oru/trajectory_envelope_coordinator.py` — `TrajectoryEnvelopeCoordinator`: full `updateDependencies()` pipeline plus `getOrder`, `computeClosestDependencies`, `depsToGraph`, `findSimpleNonliveCycles`, `findAndRepairNonliveCycles`, `callLocalReordering`, `callOnePathReplan`, `isDeadlocked`, `globalCheckAndRevise`, `canExitCriticalSection`, `setBreakDeadlocks(global, reorder, replan)`, fields `breakDeadlocksByReordering, breakDeadlocksByReplanning, avoidDeadlockGlobally, depsToCS, currentReversibleDependencies, replanningStoppingPoints, nonliveCyclesOld, holdingCS`.
- `coordination_oru/abstract_trajectory_envelope_tracker.py` — `AbstractTrajectoryEnvelopeTracker`: `setCriticalPoint(int)`, `getLastRobotReport()`, `getRobotReport()`, `getTrajectoryEnvelope()` (replaces `permit_index_until`).
- `coordination_oru/trajectory_envelope_tracker_dummy.py` — `TrajectoryEnvelopeTrackerDummy` (parked robot).
- `coordination_oru/motionplanning/abstract_motion_planner.py` — `AbstractMotionPlanner` ABC (`setStart`, `setGoals`, `plan`, `getPath`, `setFootprint`, per Java).
- `coordination_oru/simulation2D/trajectory_envelope_coordinator_simulation.py` — rename of `SimulationCoordinator` → `TrajectoryEnvelopeCoordinatorSimulation`; keeps asyncio sim loop.
- `coordination_oru/simulation2D/trajectory_envelope_tracker_rk4.py` — rename of `RK4SimulationTracker` → `TrajectoryEnvelopeTrackerRK4`; honours `setCriticalPoint` semantics (decelerate to stop **at** the CP using the forward model, not instant halt).
- `coordination_oru/metacsp/spatial/*` — add Java-named accessors (compat layer, see Approach).
- Delete `coordination_oru/coordinator/` package (contents superseded above); update `coordination_oru/viz/pyglet_viewer.py`, all of `tests/`, `examples/`, and `pyproject.toml` (version 0.3.0).

## Steps

### Milestone 1 — core coordination pipeline at Java fidelity

- [x] Clone the Java reference at the pinned commit; keep it open beside every ported file.
- [x] Port leaf classes: `CriticalSection` (identity semantics!), `Dependency`, `RobotReport`, `RobotAtCriticalSection`, `ForwardModel` + `ConstantAccelerationForwardModel`.
- [x] Add the Java-named accessor layer to `TrajectoryEnvelope`/`SpatialEnvelope`/`Pose`/`PoseSteering`.
- [x] Port `AbstractTrajectoryEnvelopeCoordinator`: mission lifecycle (`addMissions` → `envelopesToTrack` → tracker start), parking envelopes + `placeRobot`, `computeCriticalSections()` computed **incrementally on mission start** (driving×new, new×new, driving×parking, new×parking — not every tick), static `getCriticalSections(...)` with intersection-piece splitting, **merge of pieces closer than `maxDimensionOfSmallestRobot`**, and `checkEscapePoses`; `getCriticalPoint(...)` (existing logic survives, renamed); stopping points (`stoppingPoints/stoppingTimes/spawnWaitingThread/atStoppingPoint`); `cleanUpRobotCS` on envelope end (fixes ghost-envelope bug together with the parking lifecycle).
- [x] Port `TrajectoryEnvelopeCoordinator.updateDependencies()` in full: obsoletion rule (`pathIndex > teEnd` for either robot → remove CS, increment `criticalSectionCounter`, purge `escapingCSToWaitingRobotIDandCP`); parking-dependency branch (`TrajectoryEnvelopeTrackerDummy`); `canStopRobot1/2` via `communicatedCPs` induction + `getEarliestStoppingPathIndex`; the four ordering branches (both-can-stop → `getOrder`; one-can't; both-can't → re-impose from `CSToDepsOrder`, lost-order recovery via `communicatedCPs`/`isAhead`, raise on unrecoverable); `canExitCriticalSection` + `artificialDependencies`; wake-up-in-CS revision via `escapingCSToWaitingRobotIDandCP`; `CSToDepsOrder`/`depsToCS` bookkeeping; `computeClosestDependencies`; send CPs through `setCriticalPoint` recording `communicatedCPs` (never retract a CP behind what was communicated).
- [x] Port `getOrder` with `yieldIfParking`, the `comparators` chain (`addComparator`), and `muted` — unclamped distance heuristic as default.
- [x] Rework trackers: `AbstractTrajectoryEnvelopeTracker.setCriticalPoint`, `TrajectoryEnvelopeTrackerDummy`, rename sim classes into `simulation2D/`, make `TrajectoryEnvelopeTrackerRK4` decelerate to the CP via its forward model.
- [x] Update viewer, tests, and examples to the new API; all existing scenarios (two/three robots, convoy, dynamic missions) must pass.
- [x] **Verify** by running the example scripts end-to-end; **one commit** for the milestone.

### Milestone 2 — deadlock machinery + regression tests

- [x] Port `depsToGraph`, `findSimpleNonliveCycles` (networkx `simple_cycles` + the waiting-point ≤ releasing-point nonliveness condition), `isDeadlocked()` with `deadlockedCallback`.
- [x] Port `findAndRepairNonliveCycles` + `callLocalReordering`: reverse only **reversible** deps (both robots can stop, non-artificial), accept a reversal only if it strictly reduces nonlive-cycle count, atomically update `CSToDepsOrder`/`depsToCS`/`currentDependencies`; `nonliveCyclesOld` bookkeeping.
- [x] Port `avoidDeadlockGlobally` / `globalCheckAndRevise` with `holdingCS` ordering.
- [x] Add `AbstractMotionPlanner` ABC and the `callOnePathReplan` / `replacePath` / `replanningStoppingPoints` flow; `breakDeadlocksByReplanning` is inert without an injected planner; `setBreakDeadlocks(...)` wires all three flags.
- [x] New tests: (a) paths crossing **twice** get two independent CS orders; (b) CS obsoletion — passed CS disappears and stops feeding the dependency graph; (c) head-on deadlock resolved by `callLocalReordering`; (d) non-reversible situation produces an artificial dependency instead of a flip; (e) robot parked inside another's path (`yieldIfParking` + parking dep); (f) re-tasking a robot leaves no ghost envelope; (g) `ConstantAccelerationForwardModel.getEarliestStoppingPathIndex` numeric check.
- [x] Bump version to 0.3.0; **one commit** for the milestone (two commits total for the plan).

## Edge cases & risks

- **camelCase violates PEP8** — intentional per user mandate; silence lint (`N8xx`) for the package rather than renaming.
- `updateDependencies` has Java `throw new Error("FIXME! Lost dependency...")` paths — port as raised `RuntimeError` with the same messages (bug-for-bug fidelity beats silent recovery).
- `communicatedCPs` semantics assume the tracker echoes the CP back in its `RobotReport.criticalPoint`; the sim trackers must be updated to do this or the canStop induction breaks.
- The RK4 tracker currently stops instantly; with a real `ConstantAccelerationForwardModel` the coordinator will assume braking distance — sim tracker dynamics and forward-model parameters must agree or robots will overshoot CPs.
- Java relies on `CriticalSection` hash stability while `te1Break/te2Break` mutate — they are excluded from `hashCode`; keep them out of `__hash__` too.
- Global avoidance (`globalCheckAndRevise`) enumerates all simple cycles — exponential worst case; keep it behind the `avoidDeadlockGlobally` flag defaulting off, as in Java.
