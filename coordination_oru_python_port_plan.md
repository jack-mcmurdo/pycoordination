# coordination_oru Python Port — Multi-Stage Plan

## Background

`coordination_oru` (FedericoPecora/coordination_oru) is a robot-agnostic online coordination framework for multiple robots. It implements a loosely-coupled approach to multi-robot coordination using trajectory envelopes and temporal constraint reasoning. The core paper is: Pecora et al., *A loosely-coupled approach for multi-robot coordination, motion planning and control*, ICAPS 2018.

**Scope of this port:**
- Port only the `coordination_oru` functionality needed for production use
- Port only the minimal surface of `meta-csp-framework` required by `coordination_oru`
- No ROS layer (separate project — rclpy bridge is trivial to add later)
- No OMPL (the framework is loosely coupled; paths are provided externally)
- PRODUCTION quality from the start, with async-first design
- Testing via hardcoded paths; visualization deferred

---

## Dependency Verdict: Python 3.12+

Python is the clear winner. Every Java dependency has a direct equivalent or a better Python version.

| Java original | Role | Python replacement | Notes |
|---|---|---|---|
| JTS (vividsolutions) | Geometry: footprints, intersections, swept areas | `shapely 2.x` | GEOS is a direct port of JTS — identical semantics |
| JGraphT | Directed dependency graph, cycle detection | `networkx` | Near 1:1 API |
| meta-csp APSP (STP solver) | All-Pairs Shortest Path on temporal distance graph | `numpy` (Floyd-Warshall) | ~20 lines of numpy |
| meta-csp Allen constraints | Temporal relation types (Before, Meets, Overlaps…) | Custom thin layer | 13 relation types, ~50 lines total |
| meta-csp TrajectoryEnvelope | Core data: path + time bounds + footprint | Custom port | ~150 lines, dataclasses + shapely |
| meta-csp TrajectoryEnvelopeSolver | Envelope registry + STP network manager | Custom port | ~200 lines |
| Java threads | Per-robot tracker threads | `asyncio` Tasks + Queues | Cleaner in Python |
| Swing/AWT | Visualization | `matplotlib` / `pygame` | Deferred |
| AIMA `Pair` | Pair/tuple utility | `tuple` / `NamedTuple` | One line |
| Apache Commons | `ComparatorChain`, utilities | `functools`, `itertools` | stdlib |
| Reflections lib | Runtime class scanning | `importlib` | stdlib |

**What we explicitly drop from meta-csp:** The full meta-CSP search engine, the Boolean CSP solver, the state variable scheduler, and the timeline planner. These are not used by `coordination_oru`'s runtime. Total meta-csp surface ported: ~550 lines from a ~20,000 line framework.

**CSP library decision — numpy now, OR-Tools later.** The STP solver in meta-csp is Floyd-Warshall on a distance matrix — not a CP solver. `numpy` handles this in ~20 lines and is faster than any general-purpose CP solver for this specific problem. OR-Tools CP-SAT becomes relevant in future stages when global ordering search is needed.

**Geometry note:** Shapely 2.x uses GEOS — a C++ port of JTS — so `AffineTransformation` → `shapely.affinity`, `GeometryFactory` → direct constructors, `STRtree` → `shapely.STRtree`. No semantic drift.

---

## Stage 1: Dependency Resolution & Project Setup

### Deliverable
A `pyproject.toml` with pinned dependencies, a full `README` with dependency rationale, the folder skeleton with empty modules and complete docstrings, and the first implemented layer: pure data types with no external deps except numpy, fully tested.

### `pyproject.toml` dependencies
```toml
[project]
name = "coordination-oru"
requires-python = ">=3.12"

dependencies = [
    "shapely>=2.0",       # Geometry (replaces JTS via GEOS)
    "networkx>=3.0",      # Graph (replaces JGraphT)
    "numpy>=1.26",        # STP solver, path math
    "structlog>=24.0",    # Structured async-safe logging
    "attrs>=23.0",        # Fast, clean data classes
]

[project.optional-dependencies]
future = [
    "ortools>=9.8",       # Large-fleet ordering search (Stage 4+)
    "qualreas>=0.6",      # Full Allen interval algebra (Stage 4+)
    "matplotlib>=3.8",    # Visualization (deferred)
]
dev = [
    "pytest>=8.0",
    "pytest-asyncio>=0.23",
    "mypy>=1.8",
]
```

---

## Stage 2: Package Structure

```
coordination_oru/
│
├── metacsp/                          # Minimal meta-csp port (~550 lines total)
│   ├── spatial/
│   │   ├── pose.py                   # Pose(x,y,theta) · PoseSteering(pose, steering)
│   │   ├── trajectory_envelope.py    # TrajectoryEnvelope · SpatialEnvelope
│   │   └── trajectory_envelope_solver.py  # TECoordSolver + STP network
│   └── temporal/
│       ├── bounds.py                 # Bounds(lb, ub) as NamedTuple
│       ├── allen.py                  # AllenIntervalConstraint enum (13 types)
│       └── stp.py                   # Floyd-Warshall APSP on numpy distance matrix
│
├── coordinator/
│   ├── abstract_coordinator.py       # AbstractTrajectoryEnvelopeCoordinator
│   ├── abstract_tracker.py           # AbstractTrajectoryEnvelopeTracker (async Task)
│   ├── critical_section.py           # CriticalSection dataclass
│   ├── mission.py                    # Mission dataclass
│   └── robot_report.py              # RobotReport dataclass
│
├── simulation/
│   ├── sim_coordinator.py            # SimulationCoordinator (hardcoded paths)
│   └── sim_tracker.py               # SimulationTracker (fake pose updates)
│
├── util/
│   ├── geometry.py                   # Footprint helpers: pose → shapely polygon
│   └── logging.py                    # Structured async-safe logging setup
│
└── tests/
    ├── paths/                        # Hardcoded test paths (ported from Java paths/)
    ├── test_two_robots.py
    ├── test_three_robots.py          # Port of the canonical Java demo
    └── test_dynamic_missions.py
```

---

## Stage 3: Minimal meta-csp Port

This is the intellectual core. Three modules do the heavy lifting.

### `temporal/stp.py` — the STP solver

A Simple Temporal Problem is a set of difference constraints `xⱼ - xᵢ ≤ wᵢⱼ`. Consistency = no negative-weight cycles. The solver maintains a numpy `(2N × 2N)` distance matrix (each envelope gets two nodes: start and end), runs Floyd-Warshall on each update, and exposes `get_earliest(node)` / `get_latest(node)` queries.

This is a direct port of the Java `APSPSolver`. Fast enough for tens of robots in real-time on modern hardware.

```python
import numpy as np

INF = float('inf')

class STPSolver:
    def __init__(self, max_nodes: int = 128):
        self._d = np.full((max_nodes, max_nodes), INF)
        np.fill_diagonal(self._d, 0.0)
        self._n = 0

    def new_variable(self) -> int:
        idx = self._n
        self._n += 1
        return idx

    def add_constraint(self, src: int, dst: int, weight: float) -> bool:
        """Add edge dst - src <= weight. Returns False if inconsistent."""
        if weight < self._d[src, dst]:
            self._d[src, dst] = weight
            self._propagate()
        return self._d[i, i] >= 0 for i in range(self._n)

    def get_earliest(self, node: int) -> float:
        return -self._d[node, 0]

    def get_latest(self, node: int) -> float:
        return self._d[0, node]

    def _propagate(self):
        n = self._n
        d = self._d
        for k in range(n):
            d[:n, :n] = np.minimum(d[:n, :n], d[:n, k:k+1] + d[k:k+1, :n])
```

### `spatial/pose.py` — core data types

```python
from dataclasses import dataclass
import math

@dataclass(frozen=True, slots=True)
class Pose:
    x: float
    y: float
    theta: float  # radians
    z: float = math.nan
    roll: float = math.nan
    pitch: float = math.nan

    def is_3d(self) -> bool:
        return not math.isnan(self.z)

@dataclass(frozen=True, slots=True)
class PoseSteering:
    pose: Pose
    steering: float = 0.0
```

### `temporal/bounds.py` and `temporal/allen.py`

```python
from typing import NamedTuple
from enum import Enum, auto

class Bounds(NamedTuple):
    lb: float  # lower bound (seconds)
    ub: float  # upper bound (seconds, use math.inf for unbounded)

class AllenType(Enum):
    BEFORE    = auto()
    MEETS     = auto()
    OVERLAPS  = auto()
    STARTS    = auto()
    DURING    = auto()
    FINISHES  = auto()
    EQUALS    = auto()
    AFTER     = auto()
    MET_BY    = auto()
    OVERLAPPED_BY = auto()
    STARTED_BY    = auto()
    CONTAINS      = auto()
    FINISHED_BY   = auto()
```

### `spatial/trajectory_envelope.py` — the core data container

A `TrajectoryEnvelope` holds:
- `path: list[PoseSteering]` — the full path
- `robot_id: int` — which robot owns it
- `start_node, end_node: int` — STP variable indices
- `spatial_envelope: SpatialEnvelope` — the swept footprint polygon

The `SpatialEnvelope` is computed once at creation via `shapely.unary_union` over the robot's footprint polygon rotated and translated to each pose along the path.

```python
import shapely
import shapely.affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union

@dataclass
class SpatialEnvelope:
    geometry: Polygon           # full swept area (union of all footprints)
    footprints: list[Polygon]  # per-waypoint footprints (for index lookups)

def compute_spatial_envelope(
    path: list[PoseSteering],
    footprint: Polygon,        # robot footprint centered at origin
) -> SpatialEnvelope:
    polys = []
    for ps in path:
        rotated = shapely.affinity.rotate(footprint, ps.pose.theta,
                                          use_radians=True, origin=(0, 0))
        translated = shapely.affinity.translate(rotated, ps.pose.x, ps.pose.y)
        polys.append(translated)
    return SpatialEnvelope(geometry=unary_union(polys), footprints=polys)
```

### `spatial/trajectory_envelope_solver.py` — the network manager

Creates and tracks all envelopes, manages the STP distance matrix, and exposes `add_ordering_constraint(te_a, te_b)` for ordering two envelopes.

An ordering constraint `te_a BEFORE te_b` translates to STP edges:
- `start(b) - end(a) ≥ 0` → `end(a) - start(b) ≤ 0`

The solver checks consistency after each addition.

---

## Stage 4: Coordination Logic

### `abstract_coordinator.py`

The main coordination class. Key responsibilities:

**Critical section detection**

Iterate over all pairs of active trajectory envelopes and test `spatial_envelope_a.intersects(spatial_envelope_b)`. For intersecting pairs, find the index ranges of overlap by testing individual per-waypoint footprints.

Complexity: `O(N² × L)` where L is path length. Shapely's vectorized numpy ops make this fast in practice. Use `shapely.STRtree` (replacing Java's `STRtree`) to reduce to near `O(N log N)` for large fleets.

```python
import shapely
from shapely.strtree import STRtree

def find_critical_sections(
    envelopes: list[TrajectoryEnvelope],
) -> list[CriticalSection]:
    sections = []
    geoms = [te.spatial_envelope.geometry for te in envelopes]
    tree = STRtree(geoms)
    for i, te_a in enumerate(envelopes):
        candidates = tree.query(geoms[i])
        for j in candidates:
            if j <= i:
                continue
            te_b = envelopes[j]
            cs = compute_critical_section(te_a, te_b)
            if cs is not None:
                sections.append(cs)
    return sections
```

**Ordering decisions**

The minimal version uses a greedy heuristic: whoever is closest to the critical section entry gets priority (same logic as the Java original). No CSP search required at this stage.

**Constraint propagation**

The ordering decision becomes an STP constraint. If robot A goes first through a shared section:
```
start_CS(B) - end_CS(A) ≥ safety_buffer
```
Floyd-Warshall propagates this and may tighten time bounds on other envelopes.

**Deadlock detection**

Maintain a `networkx.DiGraph` where edge A→B means "A is waiting for B to clear". A cycle is a deadlock. `networkx.find_cycle()` runs in `O(V+E)` and is called after each ordering update.

```python
import networkx as nx

def check_deadlock(waiting_graph: nx.DiGraph) -> bool:
    try:
        nx.find_cycle(waiting_graph)
        return True
    except nx.NetworkXNoCycle:
        return False
```

### `abstract_tracker.py` — async per-robot task

Each robot tracker is an `asyncio.Task`. It holds a reference to its envelope and receives pose updates via an `asyncio.Queue`.

```python
import asyncio
from dataclasses import dataclass

@dataclass
class RobotReport:
    robot_id: int
    current_pose: Pose
    path_index: int         # current waypoint index along envelope
    timestamp: float        # monotonic time

class AbstractTrajectoryEnvelopeTracker:
    def __init__(self, robot_id: int, envelope: TrajectoryEnvelope, coordinator):
        self.robot_id = robot_id
        self.envelope = envelope
        self.coordinator = coordinator
        self._queue: asyncio.Queue[RobotReport] = asyncio.Queue()
        self._task: asyncio.Task | None = None

    def start(self):
        self._task = asyncio.create_task(self._run())

    async def update_pose(self, report: RobotReport):
        await self._queue.put(report)

    async def _run(self):
        while True:
            report = await self._queue.get()
            await self._on_report(report)

    async def _on_report(self, report: RobotReport):
        raise NotImplementedError
```

### Coordinator main loop

```python
COORDINATION_PERIOD = 0.1  # 10 Hz

async def coordination_loop(self):
    while self._running:
        await self._recheck_critical_sections()
        await self._enforce_orderings()
        await asyncio.sleep(COORDINATION_PERIOD)
```

---

## Stage 5: Testing Harness

Three test scenarios ported from the Java demos, using hardcoded paths loaded from the existing `paths/` directory text files (no Java parsing needed — they are plain text with pose data).

### `tests/test_two_robots.py`
Two robots, one intersection. Verify:
- Exactly one ordering constraint created
- Both robots complete their missions
- No collision (footprint intersection at any time step while both robots are in CS)

### `tests/test_three_robots.py`
The canonical three-robot demo. Verify:
- Deadlock-free completion
- Correct priority ordering at each intersection
- STP consistency maintained throughout

### `tests/test_dynamic_missions.py`
Robots continuously assigned new missions. Verify:
- Coordinator correctly handles envelope lifecycle (creation, activation, completion, removal)
- No resource leaks in the STP distance matrix
- Correct handling of robots re-entering a critical section on a new mission

All tests use `pytest-asyncio` and assertions only. No visualization required.

---

## Future Stages (outline)

| Stage | Description |
|---|---|
| 6 | OR-Tools CP-SAT ordering search for globally optimal priority assignment |
| 7 | Full `qualreas` integration for richer Allen interval reasoning |
| 8 | rclpy bridge — thin ROS 2 adapter layer publishing `nav_msgs/Path` and subscribing to robot odometry |
| 9 | Visualization layer (`matplotlib` real-time animation or `pygame` 2D sim) |
| 10 | Full `meta-csp-framework` port as a standalone Python library |

---

## Summary

Total lines to write for a working research version:

| Module | Est. lines |
|---|---|
| Minimal meta-csp port | ~550 |
| Coordination core | ~730 |
| Simulation + test harness | ~400 |
| Utilities + logging | ~150 |
| **Total** | **~1,830** |

Compare to the original Java codebase (~8,000 lines for `coordination_oru` alone, ~20,000 for `meta-csp-framework`). The reduction comes from Python's expressiveness, dropping unused meta-csp machinery, and using `shapely` and `numpy` instead of hand-rolled geometry and APSP implementations.
