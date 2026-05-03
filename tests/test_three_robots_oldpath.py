"""Three-robot scenario using the original Java repo's debug1/2/3.path files.

The three debug paths share a corridor around ``y ≈ 8.7``: each robot enters
from the north at a different x offset, drops down, and runs east through
the same horizontal stretch. So this is not a perpendicular crossing — it
exercises the coordinator on overlapping convoy lanes.
"""

from __future__ import annotations

import asyncio

import pytest
from shapely.geometry import Polygon

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from tests.conftest import assert_no_collisions
from tests.paths import load_path_file


pytestmark = pytest.mark.asyncio


async def test_three_robots_oldpath(
    coordinator: SimulationCoordinator, footprint: Polygon
) -> None:
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robot_id, path in enumerate(paths, start=1):
        coordinator.add_robot(
            Mission.make(robot_id=robot_id, path=path, footprint=footprint)
        )

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Allow the coordinator at least one cycle to detect intersections.
        await asyncio.sleep(0.1)
        css = coordinator.critical_sections
        assert css, "expected at least one critical section across the shared corridor"

        await coordinator.run_until_idle(timeout=20.0)
    finally:
        stop.set()
        await monitor

    envelopes = coordinator.solver.all_envelopes()
    assert len(envelopes) == 3
    assert all(e.completed for e in envelopes)
    assert coordinator.solver.is_consistent()