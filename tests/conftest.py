"""Shared pytest fixtures and helpers."""

from __future__ import annotations

import asyncio
from typing import AsyncIterator

import pytest

from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.geometry import rectangular_footprint
from coordination_oru.util.logging import configure_logging
from shapely.geometry import Polygon


@pytest.fixture(autouse=True, scope="session")
def _logging() -> None:
    configure_logging()


@pytest.fixture
def footprint() -> Polygon:
    """A 1m × 0.6m rectangular footprint — small enough that a 0.5m path
    spacing exposes per-waypoint intersection at crossing points."""
    return rectangular_footprint(1.0, 0.6)


@pytest.fixture
async def coordinator() -> AsyncIterator[SimulationCoordinator]:
    sim = SimulationCoordinator(period=0.01, sim_step_period=0.005)
    await sim.start()
    try:
        yield sim
    finally:
        await sim.stop()


async def assert_no_collisions(coord: SimulationCoordinator, stop_event: asyncio.Event) -> None:
    """Background monitor: verify the per-waypoint footprints of all active
    robots remain pairwise disjoint at every sim tick.
    """
    while not stop_event.is_set():
        envelopes = [e for e in coord._envelopes.values() if not e.completed]
        for i, e_a in enumerate(envelopes):
            try:
                idx_a = coord.current_path_index(e_a.robot_id)
            except KeyError:
                continue
            fa = e_a.spatial_envelope.footprints[idx_a]
            for j in range(i + 1, len(envelopes)):
                e_b = envelopes[j]
                try:
                    idx_b = coord.current_path_index(e_b.robot_id)
                except KeyError:
                    continue
                fb = e_b.spatial_envelope.footprints[idx_b]
                assert not fa.intersects(fb), (
                    f"collision: robot {e_a.robot_id} (envelope {e_a.envelope_id}, "
                    f"idx {idx_a}) overlaps robot {e_b.robot_id} (envelope "
                    f"{e_b.envelope_id}, idx {idx_b})"
                )
        await asyncio.sleep(0.002)
