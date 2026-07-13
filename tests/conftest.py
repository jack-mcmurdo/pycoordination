"""Shared pytest fixtures and helpers."""

from __future__ import annotations

import asyncio
from typing import AsyncIterator

import pytest

from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.logging import configure_logging


@pytest.fixture(autouse=True, scope="session")
def _logging() -> None:
    configure_logging()


@pytest.fixture
def footprint() -> tuple[tuple[float, float], ...]:
    """A 1m × 0.6m rectangular footprint — small enough that a 0.5m path
    spacing exposes per-waypoint intersection at crossing points."""
    return footprint_coords(1.0, 0.6)


@pytest.fixture
async def coordinator() -> AsyncIterator[TrajectoryEnvelopeCoordinatorSimulation]:
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=10, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    await tec.startInference()
    try:
        yield tec
    finally:
        await tec.stopInference()


async def wait_until_idle(tec: TrajectoryEnvelopeCoordinatorSimulation, timeout: float = 30.0) -> None:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        active = [robotID for robotID in tec.trackers if tec.isDrivingRobot(robotID)]
        if not active and tec.trackers:
            return
        if loop.time() > deadline:
            raise TimeoutError(f"simulation did not complete in {timeout}s; still active: {active}")
        await asyncio.sleep(0.005)


async def assert_no_collisions(coord: TrajectoryEnvelopeCoordinatorSimulation, stop_event: asyncio.Event) -> None:
    """Background monitor: verify the per-waypoint footprints of all driving
    robots remain pairwise disjoint at every sim tick.
    """
    while not stop_event.is_set():
        driving = [
            (robotID, tracker)
            for robotID, tracker in coord.trackers.items()
            if not isinstance(tracker, TrajectoryEnvelopeTrackerDummy)
        ]
        for i, (rid_a, tracker_a) in enumerate(driving):
            te_a = tracker_a.getTrajectoryEnvelope()
            idx_a = tracker_a.getRobotReport().getPathIndex()
            if idx_a < 0:
                continue
            fa = te_a.getSpatialEnvelope().footprints[idx_a]
            for rid_b, tracker_b in driving[i + 1 :]:
                te_b = tracker_b.getTrajectoryEnvelope()
                idx_b = tracker_b.getRobotReport().getPathIndex()
                if idx_b < 0:
                    continue
                fb = te_b.getSpatialEnvelope().footprints[idx_b]
                assert not fa.intersects(fb), (
                    f"collision: robot {rid_a} (envelope {te_a.getID()}, idx {idx_a}) "
                    f"overlaps robot {rid_b} (envelope {te_b.getID()}, idx {idx_b})"
                )
        await asyncio.sleep(0.002)
