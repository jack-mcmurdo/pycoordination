# Getting started

## Install

Python ≥ 3.12. The viewers (pyglet and browser) ship with the package:

```bash
pip install coordination-oru
```

From a source checkout:

```bash
git clone https://github.com/jack-mcmurdo/pycoordination
cd pycoordination
pip install -e .[dev]
```

The web viewer's frontend ships prebuilt in PyPI wheels. In a source checkout,
build it once:

```bash
npm --prefix frontend install && npm --prefix frontend run build
```

## Run the dynamic-missions demo

```bash
coordination-oru-demo
```

(From a source checkout: `python examples/dynamic_missions.py --web-viewer`.)

This opens a browser view of three car-like robots on a 20×20 m
occupancy-grid map. **Click a robot, then press-drag-release on the map** to
post a goal pose (RViz "2D Nav Goal" style — a plain click aims the heading
from the robot toward the click, Esc deselects). Each goal is planned by
the built-in Hybrid A* planner and dispatched as a mission; the coordinator
sequences the robots wherever their paths conflict.

![Dynamic missions demo](assets/CoordinatorPy.gif)

What you're seeing:

- The **shaded corridor** behind each robot is its *trajectory envelope* —
  the footprint swept along its path.
- **Highlighted segments** are *critical sections*: places where two
  envelopes overlap and the coordinator must impose an order.
- When two robots approach the same critical section, one of them slows and
  stops at its *critical point* just short of the conflict, then continues
  once the other robot has cleared it. No robot ever replans or deviates from
  its path merely to avoid another robot — coordination is purely temporal.

## Other run modes

Every example accepts the same flags:

```bash
python examples/dynamic_missions.py              # scripted scenario; pyglet if installed, else headless
python examples/dynamic_missions.py --pyglet     # force the pyglet window
python examples/dynamic_missions.py --headless   # text-only, prints per-robot progress
python examples/dynamic_missions.py --web-viewer --port 9000 --no-browser
```

More examples in [`examples/`](https://github.com/jack-mcmurdo/pycoordination/tree/master/examples):
two/three robots crossing, convoy following, five-robot sine-wave lattice,
and the original Java repo's recorded paths.

## A minimal program

```python
import asyncio
from coordination_oru import Mission, Pose
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)

async def main():
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20)
    tec.setupSolver()
    tec.setFootprint(1, (-0.5, -0.3), (-0.5, 0.3), (0.5, 0.3), (0.5, -0.3))
    tec.placeRobot(1, start_pose)          # robot 1 parked at start_pose
    await tec.startInference()             # the coordination loop (Algorithm 1)
    tec.addMissions(Mission(1, path))      # path: sequence of PoseSteering
    ...                                    # wait until parked again
    await tec.stopInference()

asyncio.run(main())
```

Paths can come from the built-in
[Hybrid A* planner](guides/motion-planning.md), from recorded files, or from
any external planner — the coordinator only cares about the sequence of
poses. Next: [how the theory maps to this API](theory/envelopes.md).
