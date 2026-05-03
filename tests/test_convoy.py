"""Convoy following: yielder advances behind the leader inside a shared CS.

Mirrors the Java ``getCriticalPoint`` convoy semantics: when two robots
share a corridor in the same direction, the yielder must not be parked at
the entrance until the leader fully exits — it should follow the leader
through the CS at a fixed trailing buffer (``trailing_path_points``, 3 by
default).
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


async def test_yielder_follows_leader_through_shared_corridor(
    coordinator: SimulationCoordinator, footprint: Polygon
) -> None:
    """The debug paths share a long east-bound corridor at y≈8.7.

    With dynamic critical points, the yielder should be observed *inside*
    the CS waypoints while the leader is also still inside (i.e. genuine
    convoy following, not strict serialisation).
    """
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robot_id, path in enumerate(paths, start=1):
        coordinator.add_robot(
            Mission.make(robot_id=robot_id, path=path, footprint=footprint)
        )

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))

    # We track, for each CS, whether we ever saw both the leader AND the
    # yielder simultaneously inside their respective CS index ranges.
    saw_simultaneous_in_cs: dict[frozenset[int], bool] = {}

    try:
        await asyncio.sleep(0.05)  # let the coordinator pick CSes
        css = coordinator.critical_sections
        assert css, "expected critical sections on the shared corridor"

        for _ in range(2000):
            await asyncio.sleep(0.005)
            for cs in coordinator.critical_sections:
                winner_id = coordinator.priorities[cs.key]
                loser = (
                    cs.envelope_a if cs.envelope_b.envelope_id == winner_id else cs.envelope_b
                )
                winner = cs.other(loser.envelope_id)
                try:
                    li = coordinator.current_path_index(loser.robot_id)
                    wi = coordinator.current_path_index(winner.robot_id)
                except KeyError:
                    continue
                ls, le = cs.cs_range_for(loser.envelope_id)
                ws, we = cs.cs_range_for(winner.envelope_id)
                if ls <= li <= le and ws <= wi <= we:
                    saw_simultaneous_in_cs[cs.key] = True
            if all(e.completed for e in coordinator.solver.all_envelopes()):
                break

        await coordinator.run_until_idle(timeout=15.0)
    finally:
        stop.set()
        await monitor

    # On the debug paths, at least one CS should have exhibited convoy
    # behaviour (both robots inside their CS ranges at the same moment).
    assert saw_simultaneous_in_cs, (
        "no CS ever showed both robots inside it simultaneously — "
        "convoy following is not happening"
    )
