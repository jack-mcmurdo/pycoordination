"""Dynamic missions: robots are given new work after their previous envelope
completes. Verifies the coordinator handles envelope lifecycle correctly:
no STP inconsistency, no leaked critical sections, no leaked priority entries.
"""

from __future__ import annotations

import asyncio

import pytest
from shapely.geometry import Polygon

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from tests.conftest import assert_no_collisions
from tests.paths import shuttle_path


pytestmark = pytest.mark.asyncio


async def test_sequential_missions_per_robot(
    coordinator: SimulationCoordinator, footprint: Polygon
) -> None:
    """Robot 1 does mission A then mission B; robot 2 does the orthogonal pair."""
    # Mission set 1: perpendicular cross at origin.
    coordinator.add_robot(
        Mission.make(robot_id=1, path=shuttle_path((-4.0, 0.0), (4.0, 0.0)), footprint=footprint)
    )
    coordinator.add_robot(
        Mission.make(robot_id=2, path=shuttle_path((0.0, -4.0), (0.0, 4.0)), footprint=footprint)
    )

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await coordinator.run_until_idle(timeout=10.0)

        # No more active envelopes — priority and CS state should be cleaned up.
        assert coordinator.critical_sections == []
        assert coordinator.priorities == {}

        # Submit mission set 2: another crossing pair.
        coordinator.add_robot(
            Mission.make(
                robot_id=1, path=shuttle_path((4.0, 4.0), (-4.0, -4.0)), footprint=footprint
            )
        )
        coordinator.add_robot(
            Mission.make(
                robot_id=2, path=shuttle_path((-4.0, 4.0), (4.0, -4.0)), footprint=footprint
            )
        )

        await asyncio.sleep(0.1)
        css = coordinator.critical_sections
        assert len(css) == 1, f"expected one CS for the diagonal pair, got {len(css)}"

        await coordinator.run_until_idle(timeout=10.0)

        # All four envelopes (2 per robot) must be done; STP must still be sound.
        all_envs = coordinator.solver.all_envelopes()
        assert len(all_envs) == 4
        assert all(e.completed for e in all_envs)
        assert coordinator.solver.is_consistent()
    finally:
        stop.set()
        await monitor
