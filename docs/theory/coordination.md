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
