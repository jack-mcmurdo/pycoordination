# The coordination loop

The paper's Algorithm 1 is a high-level control loop that regulates access to
critical sections at frequency $1/T$. The port implements it as an asyncio
task; the mapping is line-for-line.

## Algorithm 1 → `_inference_loop`

`AbstractTrajectoryEnvelopeCoordinator._inference_loop()`
(`abstract_trajectory_envelope_coordinator.py`), started by
`startInference()`, runs every `CONTROL_PERIOD` milliseconds ($T$):

| Algorithm 1 (paper) | Implementation |
| --- | --- |
| sample state $s_i$ of every robot | trackers' `getRobotReport()` → `RobotReport` (pose, path index, velocity, current critical point) |
| goal $g_i$ posted for idle robot $i$; compute path $p_i$ | `addMissions(Mission(robotID, path))` — paths are supplied with the mission (planned by any means); accepted only if the robot `isFree()`. One pooled mission is dispatched per cycle |
| $\mathcal{C} \gets$ `getIntersections`$(\mathcal{E}(p_i), \mathcal{E}(p_j))$ | `computeCriticalSections()` → `getCriticalSections()` (shapely polygon intersection, see [previous page](envelopes.md)) |
| $\mathcal{T} \gets$ `reviseConstraints`$(\mathcal{P}, \mathcal{C}, \mathcal{T}, t, s_1..s_n)$ | `updateDependencies()` → `TrajectoryEnvelopeCoordinator.localCheckAndRevise()` |
| `updateTrajectory`$(p_i, q_i^{\text{closest}})$ / `updateTrajectory`$(p_i, p_i(1))$ | `sendCriticalPoint()` → `setCriticalPoint(robotID, index, …)` → `tracker.setCriticalPointWithCounter()`; "no constraint" is the sentinel index `-1` |
| sleep until next period | `asyncio.sleep(CONTROL_PERIOD - elapsed)` |

Only the *closest* dependency per robot is communicated
(`computeClosestDependencies()`), matching Algorithm 1's
$q_i^{\text{closest}} = \arg\min_{q_i \in T_i} p_i^{-1}(q_i)$.

The loop holds one `asyncio.Lock` where Java held nested `synchronized`
monitors — asyncio is cooperative and the ported logic never awaits
mid-section, so one outer lock gives the same atomicity.

### Algorithm 1, line by line

The full listing from the paper, with the implementing function next to
each line ([`abstract_trajectory_envelope_coordinator.py`](../reference/coordinator.md)
unless noted):

**Input:** a set $G$ containing goals posted for robots $\{1, \ldots, n\}$
— the coordinator's `missionsPool`, fed by `addMissions()`.

| # | Algorithm 1: `coordination` | Implementation |
| --- | --- | --- |
| 1 | $\mathcal{P} \gets \emptyset,\; \mathcal{C} \gets \emptyset,\; \mathcal{T} \gets \emptyset$ | fresh coordinator state: `envelopesToTrack`, `allCriticalSections` and `currentDependencies` start empty in `__init__` |
| 2 | **while** true **do** | `_inference_loop()`'s `while not self._stopInference` — the task `startInference()` creates and `stopInference()` cancels |
| 3 | &emsp;$t \gets$ `getCurrentTime()` | `getCurrentTimeInMillis()` (kept as `thread_last_update`) |
| 4–5 | &emsp;**for** $i \in [1..n]$ **do** $s_i \gets$ `sampleState()` | every pass reads each tracker's `getRobotReport()` (pose, path index, velocity, last critical point), gathered as `currentReports` at the top of `localCheckAndRevise()` |
| 6 | &emsp;**for** $i : g_i \in G \land$ `isIdle`$(s_i)$ **do** | `addMissions()` only accepts a mission for a robot that `isFree()`; accepted missions queue in `missionsPool` |
| 7–8 | &emsp;&emsp;$G \gets G \setminus \{g_i\}$; drop robot $i$'s old elements from $\mathcal{P}, \mathcal{C}$ | `_pollMissionsPool()` pops one mission per cycle (`MAX_ADDED_MISSIONS = 1`, the paper's $\lvert G \rvert \le 1$ liveness assumption). The finished mission's envelope was already retired by the tracker-finished callback (`beforeTrackingFinished` swaps back to `TrajectoryEnvelopeTrackerDummy`); its passed critical sections fall out in Algorithm 2's obsoletion sweep |
| 9–10 | &emsp;&emsp;$q_i \gets$ `getConfiguration`$(s_i)$; $p_i \gets$ `computePath`$(q_i, g_i)$ | paths arrive precomputed inside `Mission(robotID, path)`, planned by any `AbstractMotionPlanner` — planning stays outside the loop |
| 11 | &emsp;&emsp;$\mathcal{P} \gets \mathcal{P} \cup \{p_i\}$ | `envelopesToTrack.append(te)`, then `startTrackingAddedMissions()` starts the tracker |
| 12–13 | &emsp;**for** $(p_i, p_{j \ne i}) \in \mathcal{P}^2$ **do** $\mathcal{C} \gets \mathcal{C}\, \cup$ `getIntersections`$(\mathcal{E}(p_i), \mathcal{E}(p_j))$ | `computeCriticalSections()` → static `getCriticalSections()` (shapely intersection of the spatial envelopes) |
| 14 | &emsp;$\mathcal{T} \gets$ `reviseConstraints`$(\mathcal{P}, \mathcal{C}, \mathcal{T}, t, s_1 \ldots s_n)$ | `updateDependencies()` → `localCheckAndRevise()` — Algorithm 2, below |
| 15–16 | &emsp;**for** $p_i \in \mathcal{P}$ **do** $T_i = \{q_i \mid \exists j : \langle p_i, p_j, q_i, q_j \rangle \in \mathcal{T}\}$ | the per-robot dependency sets (`currentDeps`) Algorithm 2 accumulates per critical section |
| 17–18 | &emsp;&emsp;**if** $T_i \ne \emptyset$ **then** $q_i^{\text{closest}} \gets \arg\min_{q_i \in T_i} p_i^{-1}(q_i)$ | `computeClosestDependencies()` keeps each robot's nearest constraint in `currentDependencies` |
| 19 | &emsp;&emsp;&emsp;`updateTrajectory`$(p_i, q_i^{\text{closest}})$ | `sendCriticalPoint()` → `setCriticalPoint()` → `tracker.setCriticalPointWithCounter()`, with bounded retransmission |
| 20–21 | &emsp;&emsp;**else** `updateTrajectory`$(p_i, p_i(1))$ | the sentinel critical point `-1`: drive to the end of the path |
| 22–23 | &emsp;**while** `getCurrentTime()` $- \,t < T$ **do** `sleep`$(\Delta t)$ | `await asyncio.sleep(max(0, CONTROL_PERIOD - elapsed) / 1000)` |

## Algorithm 2 → `localCheckAndRevise`

`reviseConstraints` (Algorithm 2) decides, per critical section, who waits
for whom. `TrajectoryEnvelopeCoordinator.localCheckAndRevise()`
(`trajectory_envelope_coordinator.py`) iterates `allCriticalSections`:

1. **Passed sections are dropped** — if either robot's path index is beyond
   its end of the section, the `CriticalSection` is removed (Alg. 2 line 3
   negated).
2. **Parked robots** (tracked by `TrajectoryEnvelopeTrackerDummy`) — a robot
   whose envelope conflicts with a parked robot gets a dependency on the
   parking envelope. This is the port's handling of "robot parked inside
   someone's critical section".
3. **Neither robot has committed** (both can still stop before the section,
   Alg. 2 line 4): the ordering is decided by `getOrder()` — the heuristic
   hook, next page.
4. **One robot can no longer stop** (Alg. 2 lines 6–9): the other one waits.
   This is what makes an ordering *stick* once a robot has entered a section:
   a robot inside can never be asked to yield, so the decided precedence is
   never flipped mid-section — the invariant behind the paper's Theorem 1.
5. **Neither can stop:** the order is recovered from the last decision
   (`CSToDepsOrder`) or, in the same-direction "following" case where the
   trailing robot cannot exit behind the leader
   (`canExitCriticalSection()`), an **artificial dependency** briefly holds
   the *leader* back so the follower is never left stranded inside.
6. The waiting point is computed with `getCriticalPoint()` (Eqs. 3–5) and
   recorded as a `Dependency`; `CSToDepsOrder` remembers the decision per
   section.

Then `computeClosestDependencies()` picks each robot's nearest constraint,
`findAndRepairNonliveCycles()` checks the result for deadlocks
([next page](ordering-and-deadlocks.md)), and `sendCriticalPoint()`
communicates critical points — with bounded retransmission using
`MAX_TX_DELAY`, `CONTROL_PERIOD` and the tracking period, exactly the
$2(\text{max tx delay} + \text{periods})$ bound the Java original uses.

### Algorithm 2, line by line

The full listing, with the implementing code next to each line (all in
`TrajectoryEnvelopeCoordinator.localCheckAndRevise()`,
[`trajectory_envelope_coordinator.py`](../reference/coordinator.md),
unless noted):

**Input:** paths $\mathcal{P}$, critical sections $\mathcal{C}$,
constraints $\mathcal{T}$, states $s_i$, time $t$ — implicit coordinator
state: `allCriticalSections`, `CSToDepsOrder` (last cycle's decisions) and
the trackers' `currentReports`.
**Output:** revised precedence constraints $\mathcal{T}_{\text{rev}}$.

| # | Algorithm 2: `reviseConstraints` | Implementation |
| --- | --- | --- |
| 1 | $\mathcal{T}_{\text{rev}} \gets \emptyset$ | fresh `currentDeps` / `artificialDependencies` dicts each pass |
| 2 | **for** $C_{ij} \in \mathcal{C}$ **do** | `for cs in self.allCriticalSections:` |
| 3 | &emsp;**if** $\sup_{p_i} C_{ij} \notin p_i^{[0,t]} \land \sup_{p_j} C_{ij} \notin p_j^{[0,t]}$ **then** | negated as the obsoletion sweep: a section either robot has fully passed (`pathIndex > cs.getTe1End()` / `getTe2End()`) joins `toRemove` and feeds no constraint. A *parked* robot (dummy tracker) short-circuits here into a parking dependency instead (`createAParkingDep`) |
| 4 | &emsp;&emsp;**if** $\inf_{p_i} C_{ij} \notin p_i^{[0,t]} \land \inf_{p_j} C_{ij} \notin p_j^{[0,t]}$ **then** | `_canStop()` for both robots — "not yet committed" is judged as *can still stop before the section* using the forward model's `getEarliestStoppingPathIndex()`, folding the paper's realizability check (eq. 10) into this line |
| 5 | &emsp;&emsp;&emsp;$(k, m) \gets$ `computeOrdering`$(C_{ij}, \mathcal{T}, s_i, s_j)$ | `getOrder()` — the heuristic hook ([next page](ordering-and-deadlocks.md)): user comparators, `yieldIfParking`, FCFS fallback; eq. 6's "an idle robot never has priority" surfaces as the `muted` checks |
| 6–7 | &emsp;&emsp;**else if** $\inf_{p_i} C_{ij} \in p_i^{[0,t]} \land \inf_{p_j} C_{ij} \notin p_j^{[0,t]}$ **then** $(k, m) \gets (i, j)$ | `elif canStopRobot1 and not canStopRobot2:` — the committed robot drives, the other waits. This is what makes an ordering *stick* mid-section (Theorem 1's invariant) |
| 8–9 | &emsp;&emsp;**else if** $\inf_{p_i} C_{ij} \notin p_i^{[0,t]} \land \inf_{p_j} C_{ij} \in p_j^{[0,t]}$ **then** $(k, m) \gets (j, i)$ | the symmetric branch |
| — | &emsp;&emsp;*(no paper line: both committed)* | impossible in the paper's idealized model, real in practice: the order is recovered from `CSToDepsOrder` (or re-estimated via `isAhead()`), and if the trailing robot cannot exit behind the leader (`canExitCriticalSection()`) an **artificial dependency** briefly holds the leader back |
| 10 | &emsp;&emsp;$q_m \gets$ `computeCriticalPoint`$(p_m, p_k, t)$ | `getCriticalPoint()` — eqs. 3–5, minus `TRAILING_PATH_POINTS` of slack |
| 11 | &emsp;&emsp;$\mathcal{T}_{\text{rev}} \gets \mathcal{T}_{\text{rev}} \cup \{\langle p_m, p_k, q_m, \sup_{p_k} C_{ij} \rangle\}$ | `Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd)` added to `currentDeps`; the decision is remembered in `CSToDepsOrder[cs]`, and line-4 decisions are additionally marked reversible for the deadlock breaker |
| 12 | **return** $\mathcal{T}_{\text{rev}}$ | `computeClosestDependencies()` installs `currentDependencies`; then — beyond the paper — `findAndRepairNonliveCycles()` repairs nonlive cycles, `sendCriticalPoint()` transmits per robot, and `isDeadlocked()` refreshes the deadlock flag (surfaced in the [web viewer](../guides/visualization.md) as **Deadlocked!**) |

## Why this is collision-free (Lemma 1, Theorem 1)

Lemma 1: if $\mathcal{T}$ contains a complete ordering through every critical
section and all temporal profiles respect it, robots never collide.
Theorem 1 adds that Algorithms 1–2 maintain such an ordering, *provided
yielding robots can actually stop at their critical points*.

That proviso is split between two pieces of the implementation:

- the coordinator only asks a robot to stop where the
  [forward model](ordering-and-deadlocks.md) says stopping is possible
  (`_canStop()` in `localCheckAndRevise`);
- the tracker treats a received critical point as a hard constraint —
  the simulator's `TrajectoryEnvelopeTrackerRK4` decelerates to reach zero
  velocity *at* the critical point, never beyond it.

## What the framework assumes of your robots

Directly from the paper (and visible in the code's interfaces): a robot
controller must (1) report its state — `getRobotReport()`; (2) accept
set-point updates — `setCriticalPoint()`; (3) not leave its spatial envelope.
Nothing else — which is why paths can come from any planner and trackers can
wrap any controller (subclass `AbstractTrajectoryEnvelopeTracker`).
