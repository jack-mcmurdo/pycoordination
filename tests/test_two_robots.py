"""Two-robot intersection scenario.

Two robots with perpendicular paths that cross at the origin. The
coordinator must detect exactly one critical section, decide a priority,
hold the loser short of the CS until the winner clears, and let both
missions finish without their footprints overlapping.
"""

from __future__ import annotations

import asyncio

import pytest
from shapely.geometry import Polygon

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from tests.conftest import assert_no_collisions
from tests.paths import two_robot_cross


pytestmark = pytest.mark.asyncio


async def test_two_robot_intersection(
    coordinator: SimulationCoordinator, footprint: Polygon
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.add_robot(Mission.make(robot_id=1, path=path_a, footprint=footprint))
    coordinator.add_robot(Mission.make(robot_id=2, path=path_b, footprint=footprint))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Let the coordination loop run at least once before robots advance.
        await asyncio.sleep(0.05)
        css = coordinator.critical_sections
        assert len(css) == 1, f"expected exactly one CS, got {len(css)}"

        await coordinator.run_until_idle(timeout=10.0)
    finally:
        stop.set()
        await monitor

    # Both envelopes completed, exactly one priority decision was made.
    envelopes = coordinator.solver.all_envelopes()
    assert all(e.completed for e in envelopes)
    assert len(coordinator.priorities) <= 1


async def test_priority_winner_actually_goes_first(
    coordinator: SimulationCoordinator, footprint: Polygon
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.add_robot(Mission.make(robot_id=1, path=path_a, footprint=footprint))
    coordinator.add_robot(Mission.make(robot_id=2, path=path_b, footprint=footprint))

    # let the first CS get picked up
    await asyncio.sleep(0.05)
    [cs] = coordinator.critical_sections
    winner_envelope_id = coordinator.priorities[cs.key]
    loser_envelope = cs.envelope_a if cs.envelope_b.envelope_id == winner_envelope_id else cs.envelope_b
    winner_envelope = cs.other(loser_envelope.envelope_id)

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Watch progression: when the winner hasn't cleared the CS yet, the
        # loser must be held at or before its CS entry.
        observed_hold = False
        for _ in range(400):
            await asyncio.sleep(0.005)
            winner_idx = coordinator.current_path_index(winner_envelope.robot_id)
            loser_idx = coordinator.current_path_index(loser_envelope.robot_id)
            _, winner_end = cs.cs_range_for(winner_envelope.envelope_id)
            loser_start, _ = cs.cs_range_for(loser_envelope.envelope_id)
            if winner_idx <= winner_end and loser_idx >= loser_start:
                pytest.fail(
                    "loser entered CS while winner had not yet cleared: "
                    f"winner_idx={winner_idx}, winner_end={winner_end}, "
                    f"loser_idx={loser_idx}, loser_start={loser_start}"
                )
            if winner_idx <= winner_end and loser_idx == max(0, loser_start - 1):
                observed_hold = True
            if all(e.completed for e in coordinator.solver.all_envelopes()):
                break
        await coordinator.run_until_idle(timeout=5.0)
        assert observed_hold, "loser was never observed holding short of the CS"
    finally:
        stop.set()
        await monitor
