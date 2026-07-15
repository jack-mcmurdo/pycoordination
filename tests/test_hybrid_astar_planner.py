"""Hybrid A* planner on the bundled demo map."""

from __future__ import annotations

import math

import pytest
import shapely.geometry

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.motionplanning import HybridAStarPlanner, OccupancyMap, load_bundled_map
from coordination_oru.util.geometry import footprint_coords

FP = footprint_coords(1.0, 0.6)
CIRCUMRADIUS = max(math.hypot(x, y) for x, y in FP)


def _wrap(theta: float) -> float:
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


@pytest.fixture(scope="module")
def omap() -> OccupancyMap:
    return load_bundled_map()


def _planner(omap: OccupancyMap, **kwargs: float) -> HybridAStarPlanner:
    planner = HybridAStarPlanner(omap, **kwargs)  # type: ignore[arg-type]
    planner.setFootprint(*FP)
    return planner


def _xy_length(path: tuple[PoseSteering, ...]) -> float:
    return sum(
        math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y) for a, b in zip(path, path[1:])
    )


def _reverse_distance(path: tuple[PoseSteering, ...]) -> float:
    total = 0.0
    for a, b in zip(path, path[1:]):
        dx, dy = b.pose.x - a.pose.x, b.pose.y - a.pose.y
        if dx * math.cos(a.pose.theta) + dy * math.sin(a.pose.theta) < 0.0:
            total += math.hypot(dx, dy)
    return total


def test_basic(omap: OccupancyMap) -> None:
    planner = _planner(omap)
    start, goal = Pose(-8.0, -8.0, 0.0), Pose(8.0, 8.0, math.pi / 2)
    planner.setStart(start)
    planner.setGoals(goal)
    assert planner.plan()
    path = planner.getPath()
    assert path is not None

    first, last = path[0].pose, path[-1].pose
    assert math.hypot(first.x - start.x, first.y - start.y) < 1e-6
    assert abs(_wrap(first.theta - start.theta)) < 1e-6
    assert math.hypot(last.x - goal.x, last.y - goal.y) < 1e-6
    assert abs(_wrap(last.theta - goal.theta)) < 1e-6

    grid = omap.inflated(CIRCUMRADIUS)
    for a, b in zip(path, path[1:]):
        spacing = math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y)
        assert spacing <= planner.path_step * 1.5
        assert abs(_wrap(b.pose.theta - a.pose.theta)) <= spacing / planner.turning_radius + 0.05
    for ps in path:
        row, col = omap.world_to_grid(ps.pose.x, ps.pose.y)
        assert omap.in_bounds(row, col)
        assert not grid[row, col]


def _as_tuples(path: tuple[PoseSteering, ...]) -> list[tuple[float, float, float, float]]:
    # Pose == Pose is always False (NaN z/roll/pitch fields), so compare
    # the planned coordinates directly
    return [(ps.pose.x, ps.pose.y, ps.pose.theta, ps.steering) for ps in path]


def test_determinism(omap: OccupancyMap) -> None:
    planner = _planner(omap)
    planner.setStart(Pose(-8.0, -8.0, 0.0))
    planner.setGoals(Pose(8.0, 8.0, math.pi / 2))
    assert planner.plan()
    first = planner.getPath()
    assert first is not None
    assert planner.plan()
    second = planner.getPath()
    assert second is not None
    assert _as_tuples(second) == _as_tuples(first)


def test_reversing_works(omap: OccupancyMap) -> None:
    planner = _planner(omap)
    planner.setStart(Pose(-7.0, 8.0, 0.0))
    planner.setGoals(Pose(-8.5, 8.0, 0.0))  # 1.5 m directly behind, same heading
    assert planner.plan()
    path = planner.getPath()
    assert path is not None
    # it backs up rather than driving a loop
    assert _xy_length(path) < 2.0 * math.pi * 1.0
    assert _reverse_distance(path) > 0.0


def test_reverse_penalty(omap: OccupancyMap) -> None:
    reverse_distances = {}
    for reverse_cost in (1.0, 1.5):
        planner = _planner(omap, reverse_cost=reverse_cost)
        planner.setStart(Pose(-7.0, 8.0, 0.0))
        planner.setGoals(Pose(-8.5, 8.0, 0.0))
        assert planner.plan()
        path = planner.getPath()
        assert path is not None
        reverse_distances[reverse_cost] = _reverse_distance(path)
    # the penalty never increases reversing
    assert reverse_distances[1.0] >= reverse_distances[1.5] - 1e-6


def test_failure_modes(omap: OccupancyMap) -> None:
    planner = _planner(omap)
    planner.setStart(Pose(-8.0, -8.0, 0.0))

    planner.setGoals(Pose(-4.0, 0.0, 0.0))  # inside a block
    assert not planner.plan()

    planner.setGoals(Pose(15.0, 0.0, 0.0))  # outside map bounds
    assert not planner.plan()

    planner.setGoals(Pose(8.0, 8.0, math.pi / 2))
    planner.addObstacles([shapely.geometry.box(0.0, 0.0, 10.0, 10.0)])  # seal the NE quadrant
    assert not planner.plan()
    planner.clearObstacles()
    assert planner.plan()


def test_multi_goal(omap: OccupancyMap) -> None:
    planner = _planner(omap)
    planner.setStart(Pose(-8.0, -8.0, 0.0))
    planner.setGoals(Pose(0.0, -8.0, 0.0), Pose(8.0, 8.0, math.pi / 2))
    assert planner.plan()
    path = planner.getPath()
    assert path is not None
    assert min(math.hypot(ps.pose.x - 0.0, ps.pose.y + 8.0) for ps in path) < 0.5


def test_dynamic_obstacle_detour(omap: OccupancyMap) -> None:
    planner = _planner(omap)
    # the short route runs through the slot between the west block and the
    # L-wall at (0, ~3); the box seals it, forcing a detour
    start, goal = Pose(-8.0, 0.0, 0.0), Pose(4.0, 1.0, 0.0)
    planner.setStart(start)
    planner.setGoals(goal)
    assert planner.plan()
    unobstructed = planner.getPath()
    assert unobstructed is not None
    box = shapely.geometry.box(-1.0, 2.0, 1.0, 8.0)
    assert any(
        box.contains(shapely.geometry.Point(ps.pose.x, ps.pose.y)) for ps in unobstructed
    ), "test premise: the unobstructed path crosses the box"

    planner.addObstacles([box])
    assert planner.plan()
    path = planner.getPath()
    assert path is not None
    for ps in path:
        point = shapely.geometry.Point(ps.pose.x, ps.pose.y)
        assert box.distance(point) >= CIRCUMRADIUS - 2.0 * omap.resolution
