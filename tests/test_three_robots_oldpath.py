"""Three-robot scenario using the original Java repo's debug1/2/3.path files.

The three debug paths share a corridor around ``y ≈ 8.7``: each robot enters
from the north at a different x offset, drops down, and runs east through
the same horizontal stretch. So this is not a perpendicular crossing — it
exercises the coordinator on overlapping convoy lanes.
"""

from __future__ import annotations

import asyncio

import pytest

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from tests.conftest import assert_no_collisions, wait_until_idle
from tests.paths import load_path_file


pytestmark = pytest.mark.asyncio


async def test_three_robots_oldpath(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robotID, path in enumerate(paths, start=1):
        coordinator.setFootprint(robotID, *footprint)
        coordinator.placeRobot(robotID, path[0].getPose())
    coordinator.addMissions(*[Mission(robotID, path) for robotID, path in enumerate(paths, start=1)])

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Allow the coordinator at least one cycle to detect intersections.
        await asyncio.sleep(0.1)
        css = coordinator.allCriticalSections
        assert css, "expected at least one critical section across the shared corridor"

        await wait_until_idle(coordinator, timeout=20.0)
    finally:
        stop.set()
        await monitor

    assert coordinator.solver.is_consistent()
