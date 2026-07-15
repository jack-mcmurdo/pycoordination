# Ordering, forward models & deadlocks

## Forward models (Eqs. 7–10)

To revise orderings while robots move, the coordinator must know whether a
robot *can still stop* before a critical section. The paper uses a
conservative forward model of the dynamics (Eqs. 7–9, a constant-acceleration
extrapolation): keep accelerating for one communication round-trip, then
brake at maximum deceleration; robot $i$ can yield at $C_{ij}$ iff the
resulting stopping configuration $\hat{q}_i$ satisfies

$$p_i^{-1}(\hat{q}_i) < p_i^{-1}(\inf\nolimits_{p_i} C_{ij}) \tag{10}$$

**In the code:** `ForwardModel` / `ConstantAccelerationForwardModel`
(`coordination_oru/forward_model.py`).

- `canStop(te, report, targetIndex, …)` is Eq. 10: accelerate (capped at
  `maxVel`) for the lookahead window
  $\text{CONTROL\_PERIOD} + 2(\text{max tx delay} + \text{tracking period})$,
  then add the braking distance $v^2 / 2a_{max}$, and compare against the
  distance to the target index.
- `getEarliestStoppingPathIndex()` is the same computation returned as a path
  index, with the Java original's margins (×1.1 optimistic acceleration,
  ×0.9 pessimistic braking).
- Where the Java version forward-simulates with RK4 at 0.1 ms steps, the port
  uses the exact closed form of the same piecewise-constant-acceleration
  dynamics — same result, without starving the asyncio loop.

Robots without a configured model get `_DefaultForwardModel`, which claims
stopping is always possible (the paper's "no knowledge of dynamics" case —
safe, maximally conservative orderings).

## Ordering heuristics (Eqs. 6, 11)

When *both* robots can still stop, the ordering is a free choice:

$$(k, m) = \arg\min_{(k,m) \in F_{ij}} h(s_k, s_m, k, m) \tag{11}$$

for any heuristic $h$ over the states of the two robots — collision safety
does not depend on the choice (Theorem 2), only fleet performance does. The
paper's $h_{\text{dist}}$ prefers the robot closest to the section:

$$h_{\text{dist}} \equiv p_k^{-1}(\inf\nolimits_{p_k} C_{km}) - p_k^{-1}(q(s_k))$$

**In the code:** `TrajectoryEnvelopeCoordinator.getOrder()`. With no
configuration it applies $h_{\text{dist}}$ in index space: the robot with the
smaller `csStart - pathIndex` gap drives first. Custom heuristics are
comparators over `RobotAtCriticalSection` pairs, added with
`addComparator()`; earlier comparators win, later ones break ties. Two policy
switches mirror the paper's discussion: `yieldIfParking` (a robot that *ends
its path inside* the section yields — it would otherwise trap the other
robot) and `muted` robots never get priority.

## Deadlocks (Def. 9, Remark 3)

Orderings are pairwise, so cycles are possible: the **dependency graph**
$D_\mathcal{T} = (V, E)$ has an edge $(i, j)$ when $i$ waits for $j$. A cycle
$\langle i_1, \dots, i_m, i_1 \rangle$ is **unsafe** (a deadlock in the
making) iff for some consecutive pair the constraints are mutually
unsatisfiable — robot $j$'s waiting point lies at or before the point robot
$i$ must reach to release *its* waiter:

$$p_{i_j}^{-1}(q_{i_j}) > p_{i_j}^{-1}(q'_{i_j}) \quad \text{for all } j \in [2..m]$$

**In the code** (`trajectory_envelope_coordinator.py`):

- `depsToGraph()` builds $D_\mathcal{T}$ as a `networkx.DiGraph`.
- `findSimpleNonliveCycles()` enumerates simple cycles; a cycle is *nonlive*
  if some consecutive dependency pair fails `nonlivePair(dep1, dep2)`, i.e.
  `dep2.waitingPoint <= dep1.releasingPoint` — the discrete form of the
  condition above.
- `isDeadlocked()` reports a *manifest* deadlock: every robot in a nonlive
  cycle is stopped exactly at its communicated critical point.

### Breaking cycles

The paper sketches several strategies; the port implements all three of the
Java original's, selected by `setBreakDeadlocks(global, reorder, replan)`:

1. **Local reordering** (`callLocalReordering`, default on) — for a nonlive
   cycle, try to *reverse* one reversible dependency (one where both robots
   can still stop). The reversal is committed only if it strictly reduces the
   number of nonlive cycles, and only if the newly-yielding robot can still
   honour the new waiting point.
2. **Replanning** (`callOnePathReplan` → `rePlanPath`, default on) — pick a
   robot in the cycle, re-run its motion planner from its waiting pose to its
   goal, treating the other involved robots — held at their critical points —
   as obstacles (`getObstaclesInCriticalPoints()`). This is the paper's
   "consider possible placements of other robots during path computation".
   Requires a planner injected via `setMotionPlanner()`; without one, this
   strategy silently defers to reordering.
3. **Global avoidance** (`globalCheckAndRevise`, `avoidDeadlockGlobally`,
   default **off**) — maintain a global orders graph and refuse any ordering
   whose cycle could become unsafe. Exponential worst case; the paper's
   Remark 3 explains why the cheap strategies are usually preferred: unsafe
   cycles are rare if robots don't start/end inside critical sections.

`replaceEnvelope`-style operations (`replacePath`, `truncateEnvelope`,
`reverseEnvelope`) carry the decided orderings over to the new envelope's
critical sections, so a replan never loses precedence decisions that other
robots are already relying on.
