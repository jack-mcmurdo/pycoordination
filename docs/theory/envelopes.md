# Envelopes & critical sections

This page and the next two walk through the ICAPS 2018 paper
([PDF](../assets/Paper.pdf)) definition by definition, pointing at the code
that implements each one. Notation: robot $i$ follows a path $p_i$; $R_i(q)$
is the placement of robot $i$'s footprint at configuration $q$.

## Paths and trajectories

A **path** is a map $p : [0,1] \to \mathcal{Q}$ from arc-length parameter to
configurations, and a **trajectory** is the path plus a temporal profile
$\sigma(t)$, i.e. $p(\sigma(t))$ (Defs. 1–2). Computing $\sigma$ is the robot
controller's job — the coordinator never touches it.

**In the code** the arc-length parameter is discretized to waypoint indices:
a path is a `tuple[PoseSteering, ...]`
([`metacsp/spatial/pose.py`](../reference/metacsp.md)), and everything the
paper states in terms of $\sigma \in [0,1]$ becomes an integer **path index**.
The temporal profile lives in the tracker (the simulator's RK4 tracker, or a
real controller), never in the coordinator.

## Spatial envelopes (Def. 3)

$$\mathcal{E}(p) \;=\; \bigcup_{\sigma \in [0,1]} R(p(\sigma))$$

The set of all footprint placements swept along the path.

**In the code:** `compute_spatial_envelope()` in
`coordination_oru/metacsp/spatial/trajectory_envelope.py` places the footprint
polygon at every waypoint and unions them with shapely. The result — the union
polygon *plus* the per-waypoint footprints (needed to localise interference to
indices) — is a `SpatialEnvelope`. A `TrajectoryEnvelope` bundles the path,
its spatial envelope, the owning robot ID, and two STP time-point variables;
it is the unit the coordinator reasons over, created by
`TrajectoryEnvelopeSolver.createEnvelopeNoParking()` (driving) and
`createParkingEnvelope()` (a one-pose envelope for a parked robot).

## Interference and critical sections (Defs. 4–5)

Paths $p_i, p_j$ **interfere** iff
$\mathcal{E}(p_i) \cap \mathcal{E}(p_j) \neq \emptyset$. The **critical
sections** $C_{ij} \in \mathcal{C}_{ij}$ are the largest contiguous subsets of

$$\mathcal{S} = \{\, q \mid R_i(q) \cap \mathcal{E}(p_j) \neq \emptyset \;\vee\; R_j(q) \cap \mathcal{E}(p_i) \neq \emptyset \,\}.$$

**In the code:** a `CriticalSection`
(`coordination_oru/critical_section.py`) is the quadruple

```
(te1, te2, [te1Start, te1End], [te2Start, te2End])
```

— robot 1 is "inside" between path indices `te1Start`–`te1End` while robot 2
is inside between `te2Start`–`te2End`; these index intervals are the discrete
$[\inf_{p} C_{ij},\, \sup_{p} C_{ij}]$. They are computed by the static
`AbstractTrajectoryEnvelopeCoordinator.getCriticalSections()`
(`abstract_trajectory_envelope_coordinator.py`): intersect the two envelope
polygons, then walk each path's per-waypoint footprints against each
intersection piece to find the enter/exit indices. Nearby intersection pieces
closer than the smaller robot's footprint dimension are merged (convex hull),
mirroring the Java original. `computeCriticalSections()` runs this for every
pair of driving/pending/parking envelopes and accumulates the results in
`allCriticalSections`.

The paper assumes no robot starts or ends its path inside a critical section;
where that fails (e.g. a robot parked in another's way), the port handles it
via parking envelopes and the dummy-tracker branch of the dependency update
(see [deadlocks](ordering-and-deadlocks.md)).

## Precedence constraints (Defs. 6–7)

Collisions are prevented purely by **ordering** robots through critical
sections. A precedence constraint $\langle p_i, p_j, q_i, q_j \rangle$ says:

$$q_j \notin p_j^{[0,t]} \;\Rightarrow\; q_i \notin p_i^{[0,t]} \tag{1}$$

— robot $i$ must not pass configuration $q_i$ until robot $j$ has passed
$q_j$. The **coordination problem** (Def. 7) is to synthesize, for each pair
of interfering paths, constraints on the temporal profiles so footprints
never intersect.

**In the code:** a `Dependency` (`coordination_oru/dependency.py`) is exactly
this tuple:

```
Dependency(teWaiting, teDriving, waitingPoint, thresholdPoint)
```

`waitingPoint` is the index of $q_i$ (the *critical point* sent to the
waiting robot's tracker) and `thresholdPoint` the index of $q_j$ (the end of
the critical section for the driving robot). Enforcement is one message:
`setCriticalPoint()` tells the tracker "do not drive past this index", and
the tracker guarantees it can stop there (that guarantee is the
[forward model](ordering-and-deadlocks.md)'s job).

## The critical point (Remarks 1–2, Eqs. 2–5)

The conservative constraint (Eq. 2) — wait *before* the critical section
until the other robot has fully exited — is safe but forbids two robots
moving through a long section in the same direction. The paper refines it
with the time-dependent critical point (Eqs. 4–5): the yielding robot may
advance up to

$$\mathrm{reach}(p_i, p_j, t) = \sup\nolimits_{p_i} \{\, q \in p_i^{[0,t]} \mid R_i(q) \cap R_j(p_j(\sigma_j(t))) = \emptyset \,\} \tag{3}$$

— the last configuration that does not collide with where robot $j$
*currently* is. This produces the "following" behaviour: the critical point
rolls forward as the leader progresses.

**In the code:**
`AbstractTrajectoryEnvelopeCoordinator.getCriticalPoint(yieldingRobotID, cs,
leadingRobotCurrentPathIndex)` implements Eqs. 3–5 geometrically:

- Leader not yet in the section → yield at `csStart - TRAILING_PATH_POINTS`
  (the discrete $\inf_{p_i} C_{ij}$, with a 3-waypoint safety margin).
- Leader inside → union the leader's footprints from its current index to the
  section end, then return the last yielding-robot index whose footprint does
  not touch that union (minus the same margin). Because the leader's *future*
  placements inside the section are included, the result is valid until the
  next control period regardless of how far the leader gets.

Re-evaluating this every control period is what turns the static constraint
of Eq. 2 into the dynamic one of Eqs. 4–5.

Next: [the coordination loop](coordination.md) that computes and revises
these constraints online.
