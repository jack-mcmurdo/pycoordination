# Motion planning

The coordinator is planner-agnostic: a `Mission` carries a finished path
(`tuple[PoseSteering, ...]`), and where it came from is your business. The
built-in planner exists so the examples are self-contained and so
[deadlock replanning](../theory/ordering-and-deadlocks.md#breaking-cycles)
has something to call.

## Occupancy maps

`OccupancyMap` (`coordination_oru/motionplanning/occupancy_map.py`) loads
ROS `map_server`-style maps — a YAML descriptor plus a PGM image:

```python
from coordination_oru.motionplanning import OccupancyMap, load_bundled_map

omap = OccupancyMap.from_yaml("my_map.yaml")   # your own map
omap = load_bundled_map()                       # the 20×20 m demo map
```

The grid is stored y-up (row index grows with world $+y$), converts between
world and grid coordinates, inflates obstacles by a robot radius (cached per
radius), and can export a PNG for the web viewer.

## Hybrid A*

`HybridAStarPlanner` (`coordination_oru/motionplanning/hybrid_astar_planner.py`)
plans car-like (Reeds-Shepp) paths over an occupancy map — forward and
reverse, honouring start **and goal heading** via a collision-checked
analytic Reeds-Shepp expansion onto the goal pose:

```python
from coordination_oru.motionplanning import HybridAStarPlanner

planner = HybridAStarPlanner(omap, turning_radius=1.0)
planner.setFootprint(*footprint_coords)
planner.setStart(start_pose)
planner.setGoals(goal_pose)
if planner.plan():
    path = planner.getPath()   # tuple[PoseSteering, ...]
```

Worth knowing:

- Collision checking uses the footprint's circumcircle against the inflated
  grid, so state validity is heading-independent (conservative for elongated
  robots).
- `reverse_cost` (≥ 1) and `gear_switch_cost` shape gear usage;
  `heuristic_inflation > 1` trades optimality for speed.
- Planning is deterministic: identical calls return identical paths.
- Pure Python — fine for maps like the demo's, slow for millions of cells.

## Plugging planners into the coordinator

```python
tec.setMotionPlanner(robotID, planner)
```

This is only *required* for deadlock-breaking replanning
(`breakDeadlocksByReplanning`); day-to-day mission paths are planned by
whoever creates the `Mission`. To bring your own planner, subclass
`AbstractMotionPlanner` and implement `doPlanning()` — the coordinator calls
`setStart`/`setGoals`/`addObstacles`/`plan`/`getPath` and nothing else.
