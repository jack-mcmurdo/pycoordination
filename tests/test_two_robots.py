"""Two-robot intersection scenario.

Two robots with perpendicular paths that cross at the origin. The
coordinator must detect exactly one critical section, decide a priority,
hold the loser short of the CS until the winner clears, and let both
missions finish without their footprints overlapping.
"""

from __future__ import annotations

import asyncio

import pytest

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from tests.conftest import assert_no_collisions, wait_until_idle
from tests.paths import two_robot_cross


pytestmark = pytest.mark.asyncio


def _cs_range(cs, robotID: int) -> tuple[int, int]:
    if cs.getTe1().getRobotID() == robotID:
        return cs.getTe1Start(), cs.getTe1End()
    return cs.getTe2Start(), cs.getTe2End()


async def test_two_robot_intersection(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Let the coordination loop run at least once before robots advance.
        await asyncio.sleep(0.05)
        css = coordinator.allCriticalSections
        assert len(css) == 1, f"expected exactly one CS, got {len(css)}"

        await wait_until_idle(coordinator, timeout=10.0)
    finally:
        stop.set()
        await monitor

    # Both robots parked again, no leftover critical sections/orders.
    assert coordinator.allCriticalSections == set()
    assert coordinator.CSToDepsOrder == {}


async def test_priority_winner_actually_goes_first(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))

    # let the first CS get picked up and an order decided
    await asyncio.sleep(0.1)
    [cs] = coordinator.allCriticalSections
    waitingRobotID, _ = coordinator.CSToDepsOrder[cs]
    drivingRobotID = cs.getTe1().getRobotID() if cs.getTe2().getRobotID() == waitingRobotID else cs.getTe2().getRobotID()

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Watch progression: when the winner hasn't cleared the CS yet, the
        # loser must be held at or before its CS entry.
        observed_hold = False
        winner_start, winner_end = _cs_range(cs, drivingRobotID)
        loser_start, _ = _cs_range(cs, waitingRobotID)
        for _ in range(600):
            await asyncio.sleep(0.005)
            # A robot no longer driving has necessarily cleared every CS it
            # was part of (idx would otherwise misleadingly read -1 = parked).
            winner_cleared = not coordinator.isDrivingRobot(drivingRobotID)
            winner_idx = coordinator.trackers[drivingRobotID].getRobotReport().getPathIndex()
            loser_idx = coordinator.trackers[waitingRobotID].getRobotReport().getPathIndex()
            if not winner_cleared and winner_idx <= winner_end and loser_idx >= loser_start:
                pytest.fail(
                    "loser entered CS while winner had not yet cleared: "
                    f"winner_idx={winner_idx}, winner_end={winner_end}, "
                    f"loser_idx={loser_idx}, loser_start={loser_start}"
                )
            if not winner_cleared and winner_idx <= winner_end and loser_idx <= max(0, loser_start - 1):
                observed_hold = True
            if not coordinator.isDrivingRobot(1) and not coordinator.isDrivingRobot(2):
                break
        await wait_until_idle(coordinator, timeout=5.0)
        assert observed_hold, "loser was never observed holding short of the CS"
    finally:
        stop.set()
        await monitor
