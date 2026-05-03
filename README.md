# coordination-oru (Python)

A Python port of [coordination_oru](https://github.com/FedericoPecora/coordination_oru),
the trajectory-envelope multi-robot coordinator from
*Pecora et al., A loosely-coupled approach for multi-robot coordination, motion
planning and control, ICAPS 2018*.

This port targets production use. The ROS layer, OMPL motion planner, and
visualization are explicitly out of scope — the framework is loosely coupled,
so paths are supplied externally.

## Why Python?

| Java original              | Python replacement              | Notes                                            |
| -------------------------- | ------------------------------- | ------------------------------------------------ |
| JTS (vividsolutions)       | `shapely 2.x` (GEOS)            | GEOS is a direct C++ port of JTS — same semantics. |
| JGraphT                    | `networkx`                      | Near 1:1 graph API.                              |
| meta-csp APSPSolver        | `numpy` Floyd-Warshall          | ~20 lines.                                       |
| meta-csp Allen / Bounds    | thin custom layer               | ~50 lines, 13 relation types.                    |
| meta-csp TrajectoryEnvelope| custom port                     | dataclasses + shapely.                           |
| Java threads               | `asyncio` Tasks + Queues        | Cleaner cancellation and fan-in.                 |
| Swing/AWT                  | deferred (`matplotlib`/`pygame`) | Visualization is a future stage.                 |

We port only the meta-csp surface that `coordination_oru` actually exercises at
runtime: the STP solver, Allen relations, and the trajectory-envelope plumbing.
The general meta-CSP search engine, Boolean CSP solver, and timeline planner
are dropped.

## Install

```bash
pip install -e .[dev]
```

## Run the tests

```bash
pytest
```

## Layout

```
coordination_oru/
├── metacsp/
│   ├── temporal/  # Bounds, Allen relations, STP (Floyd-Warshall)
│   └── spatial/   # Pose, TrajectoryEnvelope, TrajectoryEnvelopeSolver
├── coordinator/   # Critical-section detection, ordering, deadlock check
├── simulation/    # In-process simulator with hardcoded paths
└── util/          # Footprint helpers, structlog setup
```
