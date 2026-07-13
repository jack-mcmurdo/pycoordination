"""Dynamic missions: robots are given new work after their previous envelope
completes. Verifies the coordinator handles envelope lifecycle correctly:
no leaked critical sections, no leaked precedence orders.
"""

from __future__ import annotations

import asyncio

import pytest

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from tests.conftest import assert_no_collisions, wait_until_idle
from tests.paths import shuttle_path


pytestmark = pytest.mark.asyncio


async def test_sequential_missions_per_robot(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """Robot 1 does mission A then mission B; robot 2 does the orthogonal pair."""
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)

    path1a = shuttle_path((-4.0, 0.0), (4.0, 0.0))
    path2a = shuttle_path((0.0, -4.0), (0.0, 4.0))
    coordinator.placeRobot(1, path1a[0].getPose())
    coordinator.placeRobot(2, path2a[0].getPose())
    coordinator.addMissions(Mission(1, path1a), Mission(2, path2a))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await wait_until_idle(coordinator, timeout=10.0)

        # No more active envelopes — CS and precedence-order state should be
        # cleaned up (this is the "ghost envelope"/CS-obsoletion fix).
        assert coordinator.allCriticalSections == set()
        assert coordinator.CSToDepsOrder == {}

        # Submit mission set 2: another crossing pair, on the diagonals.
        path1b = shuttle_path((4.0, 4.0), (-4.0, -4.0))
        path2b = shuttle_path((-4.0, 4.0), (4.0, -4.0))
        coordinator.addMissions(Mission(1, path1b), Mission(2, path2b))

        await asyncio.sleep(0.1)
        css = coordinator.allCriticalSections
        assert len(css) == 1, f"expected one CS for the diagonal pair, got {len(css)}"

        await wait_until_idle(coordinator, timeout=10.0)

        # All four envelopes (2 per robot) must have been created.
        all_envs = coordinator.solver.all_envelopes()
        assert len(all_envs) >= 4
        assert coordinator.solver.is_consistent()
    finally:
        stop.set()
        await monitor
