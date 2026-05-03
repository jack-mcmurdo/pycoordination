"""Three-robot canonical demo.

Three robots all pass through the origin from different directions. The
coordinator must serialise them through up to three pairwise critical
sections without deadlock.
"""

from __future__ import annotations

import asyncio

import pytest
from shapely.geometry import Polygon

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from tests.conftest import assert_no_collisions
from tests.paths import three_robot_intersection


pytestmark = pytest.mark.asyncio


async def test_three_robot_deadlock_free(
    coordinator: SimulationCoordinator, footprint: Polygon
) -> None:
    p1, p2, p3 = three_robot_intersection()
    coordinator.add_robot(Mission.make(robot_id=1, path=p1, footprint=footprint))
    coordinator.add_robot(Mission.make(robot_id=2, path=p2, footprint=footprint))
    coordinator.add_robot(Mission.make(robot_id=3, path=p3, footprint=footprint))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await asyncio.sleep(0.1)
        # We expect 3 pairwise CSes (each pair crosses at the origin).
        css = coordinator.critical_sections
        assert len(css) == 3, f"expected 3 pairwise CSes, got {len(css)}"

        await coordinator.run_until_idle(timeout=15.0)
    finally:
        stop.set()
        await monitor

    assert all(e.completed for e in coordinator.solver.all_envelopes())
    assert coordinator.solver.is_consistent()
