"""RK4 tracker tests: dynamics + a two-robot crossing under RK4 motion."""

from __future__ import annotations

import asyncio
import math

import pytest
from shapely.geometry import Polygon

from coordination_oru.coordinator.mission import Mission
from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.simulation.rk4_tracker import RK4SimulationTracker
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.metacsp.spatial.trajectory_envelope import (
    TrajectoryEnvelope,
    compute_spatial_envelope,
)
from coordination_oru.util.geometry import rectangular_footprint
from tests.conftest import assert_no_collisions
from tests.paths import line_path, two_robot_cross


pytestmark = pytest.mark.asyncio


def _toy_envelope(path_pts: tuple[PoseSteering, ...]) -> TrajectoryEnvelope:
    """Build an envelope without going through the solver — used for unit tests
    that only exercise the integrator. ``start_node`` and ``end_node`` are
    bogus but unused here."""
    fp = rectangular_footprint(0.6, 0.4)
    spatial = compute_spatial_envelope(path_pts, fp)
    return TrajectoryEnvelope(
        envelope_id=1,
        robot_id=1,
        path=path_pts,
        start_node=1,
        end_node=2,
        spatial_envelope=spatial,
        footprint=fp,
        nominal_duration=1.0,
    )


async def test_rk4_accelerates_to_v_max() -> None:
    path = line_path(0.0, 0.0, 20.0, 0.0, step=0.5)
    envelope = _toy_envelope(path)
    tracker = RK4SimulationTracker(
        robot_id=1, envelope=envelope, coordinator=object(), v_max=1.0, a_max=0.5
    )
    # let it run unrestricted for a while
    dt = 0.01
    for step in range(2000):
        tracker.advance(dt, step * dt)
        if math.isclose(tracker.v, 1.0, abs_tol=1e-3):
            # Reached cruise; should keep cruising.
            for further in range(50):
                tracker.advance(dt, (2000 + further) * dt)
            assert tracker.v == pytest.approx(1.0, abs=1e-3)
            return
    pytest.fail("never reached v_max")


async def test_rk4_brakes_to_permit_boundary() -> None:
    path = line_path(0.0, 0.0, 20.0, 0.0, step=0.5)
    envelope = _toy_envelope(path)
    tracker = RK4SimulationTracker(
        robot_id=1, envelope=envelope, coordinator=object(), v_max=1.5, a_max=1.0
    )
    # Cap permit at index 10 (arclength 5.0). Tracker must come to rest there.
    tracker.permit_index_until = 10
    dt = 0.01
    for step in range(3000):
        tracker.advance(dt, step * dt)
        if tracker.v == 0.0 and tracker.s > 0.0:
            break
    assert tracker.s == pytest.approx(5.0, abs=0.05)
    assert tracker.v == pytest.approx(0.0, abs=1e-6)
    assert tracker.path_index == 10


async def test_rk4_two_robot_crossing_collision_free() -> None:
    sim = SimulationCoordinator(period=0.01, sim_step_period=0.01)
    await sim.start()
    try:
        path_a, path_b = two_robot_cross()
        fp = rectangular_footprint(1.0, 0.6)
        sim.add_rk4_robot(
            Mission.make(robot_id=1, path=path_a, footprint=fp),
            v_max=1.5,
            a_max=1.0,
        )
        sim.add_rk4_robot(
            Mission.make(robot_id=2, path=path_b, footprint=fp),
            v_max=1.5,
            a_max=1.0,
        )

        stop = asyncio.Event()
        monitor = asyncio.create_task(_assert_no_kinematic_collisions(sim, stop))
        try:
            await asyncio.sleep(0.05)
            assert len(sim.critical_sections) == 1
            await sim.run_until_idle(timeout=20.0)
        finally:
            stop.set()
            await monitor

        assert all(e.completed for e in sim.solver.all_envelopes())
    finally:
        await sim.stop()


async def _assert_no_kinematic_collisions(
    sim: SimulationCoordinator, stop: asyncio.Event
) -> None:
    """Stricter than the discrete monitor: check the *interpolated* footprint
    at the tracker's current arclength, not just the nearest waypoint footprint."""
    from coordination_oru.util.geometry import place_footprint

    while not stop.is_set():
        envelopes = [e for e in sim.envelopes_by_robot.values() if not e.completed]
        live: list[tuple[int, Polygon]] = []
        for e in envelopes:
            tracker = sim.trackers.get(e.robot_id)
            if tracker is None:
                continue
            pose = getattr(tracker, "current_pose", None)
            if pose is None:
                continue
            fp = place_footprint(e.footprint, pose)
            live.append((e.robot_id, fp))
        for i in range(len(live)):
            for j in range(i + 1, len(live)):
                rid_a, fa = live[i]
                rid_b, fb = live[j]
                assert not fa.intersects(fb), (
                    f"interpolated collision: r{rid_a} vs r{rid_b}"
                )
        await asyncio.sleep(0.005)


_ = Pose  # keep import for downstream typing
