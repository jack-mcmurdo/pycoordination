"""Convoy following: yielder advances behind the leader inside a shared CS.

Mirrors the Java ``getCriticalPoint`` convoy semantics: when two robots
share a corridor in the same direction, the yielder must not be parked at
the entrance until the leader fully exits — it should follow the leader
through the CS at a fixed trailing buffer (3 waypoints, matching Java's
``TRAILING_PATH_POINTS``).
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


def _cs_range(cs, robotID: int) -> tuple[int, int]:
    if cs.getTe1().getRobotID() == robotID:
        return cs.getTe1Start(), cs.getTe1End()
    return cs.getTe2Start(), cs.getTe2End()


async def test_yielder_follows_leader_through_shared_corridor(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """The debug paths share a long east-bound corridor at y≈8.7.

    With dynamic critical points, the yielder should be observed *inside*
    the CS waypoints while the leader is also still inside (i.e. genuine
    convoy following, not strict serialisation).
    """
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robotID, path in enumerate(paths, start=1):
        coordinator.setFootprint(robotID, *footprint)
        coordinator.placeRobot(robotID, path[0].getPose())
    coordinator.addMissions(*[Mission(robotID, path) for robotID, path in enumerate(paths, start=1)])

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))

    # We track, for each CS, whether we ever saw both the leader AND the
    # yielder simultaneously inside their respective CS index ranges.
    saw_simultaneous_in_cs: set[object] = set()

    try:
        await asyncio.sleep(0.05)  # let the coordinator pick CSes
        css = coordinator.allCriticalSections
        assert css, "expected critical sections on the shared corridor"

        for _ in range(3000):
            await asyncio.sleep(0.005)
            for cs in coordinator.allCriticalSections:
                order = coordinator.CSToDepsOrder.get(cs)
                if order is None:
                    continue
                waitingRobotID = order[0]
                drivingRobotID = (
                    cs.getTe1().getRobotID() if cs.getTe2().getRobotID() == waitingRobotID else cs.getTe2().getRobotID()
                )
                waiting_tracker = coordinator.trackers.get(waitingRobotID)
                driving_tracker = coordinator.trackers.get(drivingRobotID)
                if waiting_tracker is None or driving_tracker is None:
                    continue
                li = waiting_tracker.getRobotReport().getPathIndex()
                wi = driving_tracker.getRobotReport().getPathIndex()
                ls, le = _cs_range(cs, waitingRobotID)
                ws, we = _cs_range(cs, drivingRobotID)
                if ls <= li <= le and ws <= wi <= we:
                    saw_simultaneous_in_cs.add(cs)
            if not any(coordinator.isDrivingRobot(rid) for rid in (1, 2, 3)):
                break

        await wait_until_idle(coordinator, timeout=15.0)
    finally:
        stop.set()
        await monitor

    # On the debug paths, at least one CS should have exhibited convoy
    # behaviour (both robots inside their CS ranges at the same moment).
    assert saw_simultaneous_in_cs, (
        "no CS ever showed both robots inside it simultaneously — "
        "convoy following is not happening"
    )
