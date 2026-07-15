# coordination_oru (Python)

A Python port of [coordination_oru](https://github.com/FedericoPecora/coordination_oru),
the online multi-robot coordination framework of

> F. Pecora, H. Andreasson, M. Mansouri, V. Petkov,
> *A Loosely-Coupled Approach for Multi-Robot Coordination, Motion Planning and Control*,
> ICAPS 2018. ([PDF](assets/Paper.pdf))

The framework decides, online, **who yields to whom and where** when the paths
of multiple robots overlap — without assuming anything about how those paths
were planned or how the robots are controlled. Robot controllers only need to
(1) report their current state and (2) accept a *critical point*: the path
index beyond which they must not drive for now.

## What's in the box

- **The coordinator** — critical-section detection over swept-footprint
  envelopes, online precedence revision, heuristic ordering, and deadlock
  detection/repair by local reordering or replanning. A faithful port of the
  Java original (same algorithms, same class names).
- **A 2D simulator** — RK4-integrated trackers with trapezoidal velocity
  profiles, so coordination behaviour is kinodynamically honest.
- **A built-in motion planner** — Hybrid A* with Reeds-Shepp expansions over
  ROS-style occupancy-grid maps (optional; paths can come from anywhere).
- **Viewers** — a browser-based live viewer (websockets + React), a pyglet
  window, or plain headless logs.

Java's threads become `asyncio` tasks, JTS becomes `shapely` (GEOS *is* the
C++ port of JTS), JGraphT becomes `networkx`, and the meta-CSP temporal layer
is a compact STP solver on a numpy Floyd–Warshall matrix.

## Where to go

- **[Getting started](getting-started.md)** — install and run the
  point-and-click dynamic-missions demo in five minutes.
- **[Theory → implementation](theory/envelopes.md)** — the paper's
  definitions, algorithms and equations, each mapped to the class or function
  that implements it.
- **[Guides](guides/motion-planning.md)** — motion planning, simulation, and
  visualization in practice.
- **[API reference](reference/coordinator.md)** — generated from the
  docstrings.

## License

GPL-3.0-or-later, same as the original Java `coordination_oru`.
