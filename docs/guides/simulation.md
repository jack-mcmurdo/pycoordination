# Simulation

`TrajectoryEnvelopeCoordinatorSimulation`
(`coordination_oru/simulation2D/trajectory_envelope_coordinator_simulation.py`)
is the coordinator wired to simulated robots: each dispatched mission gets a
`TrajectoryEnvelopeTrackerRK4`, and time is the wall clock.

## Lifecycle

```python
tec = TrajectoryEnvelopeCoordinatorSimulation(
    CONTROL_PERIOD=20,          # ms between coordination cycles (paper's T)
    MAX_VELOCITY=10.0,          # defaults for robots that don't set their own
    MAX_ACCELERATION=1.0,
)
tec.setupSolver()

tec.setFootprint(1, *coords)            # else a default AGV-ish footprint
tec.setRobotMaxVelocity(1, 0.5)         # optional per-robot kinodynamics
tec.placeRobot(1, Pose(x, y, theta))    # robot exists, parked

await tec.startInference()              # coordination loop starts
tec.addMissions(Mission(1, path))       # dispatched next cycle (if robot idle)
...
await tec.stopInference()
```

`addMissions` returns `False` if any target robot is not idle — dispatching
is explicitly the caller's job (see `examples/dynamic_missions.py` for a
dynamic dispatcher). Consecutive missions for one robot in a single call are
concatenated into one envelope with a stopping point between them.

## The RK4 tracker

`TrajectoryEnvelopeTrackerRK4` integrates a trapezoidal velocity profile
along the path (RK4, faithful numeric port of the Java tracker) and honours
critical points exactly: it decelerates so velocity reaches zero *at* the
critical point index, and resumes when the coordinator lifts or advances it.
This is what makes simulated coordination kinodynamically honest — a robot
that cannot brake in time is never asked to (see
[forward models](../theory/ordering-and-deadlocks.md#forward-models-eqs-710)).

Sharp turns: `addMissions` inserts brief internal stopping points where path
heading jumps by more than 90° (`useInternalCPs`), matching the Java
simulator's behaviour.

## asyncio, not threads

The whole stack runs on one event loop: the coordination loop, every tracker,
stopping-point timers, and the web viewer are `asyncio.Task`s. Consequences:

- Call coordinator methods from the same loop (the examples' `on_goal`
  callback shows the pattern; CPU-heavy planning goes to
  `asyncio.to_thread`).
- No data races by construction — shared state is guarded by one
  `asyncio.Lock` at the coordination-cycle boundary.

## Everything is observable

The examples' progress printer, the viewers, and the tests all read the same
public state: `tec.trackers`, `getRobotReport()`, `tec.allCriticalSections`,
`getCurrentDependencies()`, `isDeadlocked()`. Nothing needs hooks into the
core.
