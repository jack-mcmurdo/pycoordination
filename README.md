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

## Examples

Each example is a standalone script — run it directly with Python. With the
`viz` extra installed (`pip install -e .[viz]`) it opens an animated pyglet
viewer; without it, it runs headless and prints per-robot progress.

```bash
python examples/two_robots.py
```

Every example also accepts a viewer flag — no per-script setup needed:

```bash
python examples/two_robots.py --web-viewer   # browser-based viewer
python examples/two_robots.py --pyglet       # force the pyglet window
python examples/two_robots.py --headless     # force the text-only run
```

`--web-viewer` serves a Vite + React frontend on http://127.0.0.1:8723/
(`--port` to change, `--no-browser` to not auto-open) with live paths, swept
envelopes, critical-section highlights, footprints, zoom/pan, and dark mode.
PyPI wheels ship the frontend prebuilt; in a source checkout build it once
with `npm --prefix frontend install && npm --prefix frontend run build`.

| Script                            | Scenario                                                          |
| --------------------------------- | ----------------------------------------------------------------- |
| `examples/two_robots.py`          | Two RK4 robots cross at the origin; one yields at the intersection. |
| `examples/three_robots.py`        | Three RK4 robots through one intersection, deadlock-free.          |
| `examples/three_robots_oldpath.py`| The original Java repo's `debug1/2/3.path` recorded paths.         |
| `examples/convoy.py`              | Convoy following: a yielder trails the leader inside a shared corridor. |
| `examples/dynamic_missions.py`    | Robots get new missions after finishing their first ones.          |
| `examples/five_robots_sine.py`    | Five robots on interleaved sine/cosine waves whose crossings form a lattice of critical sections. |

## Run the tests

```bash
pytest
```

## License

GNU General Public License v3.0 or later — see [`LICENSE`](LICENSE). The
original Java `coordination_oru` is GPL-3.0, so this port keeps the same
licence.

## Layout

```
coordination_oru/
├── metacsp/
│   ├── temporal/  # Bounds, Allen relations, STP (Floyd-Warshall)
│   └── spatial/   # Pose, TrajectoryEnvelope, TrajectoryEnvelopeSolver
├── coordinator/   # Critical-section detection, ordering, deadlock check
├── simulation/    # In-process simulator with hardcoded paths
├── data/          # Bundled demo .path files (ship with the wheel)
└── util/          # Footprint helpers, path loaders/generators, structlog setup
examples/          # Standalone runnable demos (python examples/<name>.py)
```
