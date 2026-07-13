"""Three-robot canonical demo.

Three robots all pass through the origin from different directions. The
coordinator must serialise them through up to three pairwise critical
sections without deadlock.
"""

from __future__ import annotations

import asyncio

import pytest

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from tests.conftest import assert_no_collisions, wait_until_idle
from tests.paths import three_robot_intersection


pytestmark = pytest.mark.asyncio


async def test_three_robot_deadlock_free(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    p1, p2, p3 = three_robot_intersection()
    for robotID, (path, v_max) in enumerate(((p1, 1.5), (p2, 1.2), (p3, 1.0)), start=1):
        coordinator.setFootprint(robotID, *footprint)
        coordinator.setRobotMaxVelocity(robotID, v_max)
        coordinator.setRobotMaxAcceleration(robotID, 1.0)
        coordinator.placeRobot(robotID, path[0].getPose())
    coordinator.addMissions(Mission(1, p1), Mission(2, p2), Mission(3, p3))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await asyncio.sleep(0.1)
        # We expect 3 pairwise CSes (each pair crosses at the origin).
        css = coordinator.allCriticalSections
        assert len(css) == 3, f"expected 3 pairwise CSes, got {len(css)}"

        await wait_until_idle(coordinator, timeout=30.0)
    finally:
        stop.set()
        await monitor

    assert coordinator.solver.is_consistent()
