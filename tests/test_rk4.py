"""RK4 tracker tests: integrator dynamics + real-coordinator kinematic checks.

The Java tracker (and this port) integrates in real wall-clock time via its
own task, not a caller-driven ``advance(dt)`` step function, so the pure
numerical core (``integrateRK4``/``computeDistance``) is unit-tested
directly, and the "decelerate to a critical point"/"collision-free
crossing" behaviours are exercised through the real coordinator.
"""

from __future__ import annotations

import asyncio

import pytest

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.state import State
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.simulation2D.trajectory_envelope_tracker_rk4 import integrateRK4
from coordination_oru.util.geometry import place_footprint
from tests.conftest import wait_until_idle
from tests.paths import line_path, two_robot_cross


def test_integrate_rk4_accelerates_to_v_max() -> None:
    state = State(0.0, 0.0)
    v_max, a_max = 1.0, 0.5
    dt = 0.01
    for step in range(2000):
        integrateRK4(state, step * dt, dt, False, v_max, 1.0, a_max)
        if state.getVelocity() >= v_max - 1e-3:
            for _ in range(50):
                integrateRK4(state, step * dt, dt, False, v_max, 1.0, a_max)
            # computeAcceleration switches off once velocity crosses v_max
            # rather than clamping, so a step can slightly overshoot before
            # settling — matches Java's (unclamped) bang-bang model.
            assert state.getVelocity() == pytest.approx(v_max, rel=0.02)
            return
    pytest.fail("never reached v_max")


def test_integrate_rk4_decelerates_to_stop() -> None:
    state = State(0.0, 1.0)
    a_max = 0.5
    dt = 0.01
    t = 0.0
    for _ in range(2000):
        integrateRK4(state, t, dt, True, 1.0, 1.0, a_max)
        t += dt
        if state.getVelocity() <= 0.0:
            break
    assert state.getVelocity() == pytest.approx(0.0, abs=1e-6)


@pytest.mark.asyncio
async def test_rk4_tracker_decelerates_to_critical_point(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """A mission stopping point becomes a dependency the coordinator holds
    the tracker at (unlike a raw ``setCriticalPoint`` call, which the next
    control tick would immediately override since nothing else justifies it)
    — the tracker must decelerate to a stop near there, then resume once the
    stopping duration elapses.

    Note (ported from Java verbatim): the stopping-point wait timer is
    proximity-triggered (``abs(pathIndex - stoppingPoint) <= 1``), not
    motion-triggered — it can start counting down while the robot is still
    braking, before velocity actually reaches 0. A duration long enough for
    the robot's own deceleration profile to finish is required for the robot
    to actually come to rest during the hold; a too-short duration (e.g. the
    100ms Java uses for its own internal direction-change stopping points)
    can expire before the robot fully stops.
    """
    coordinator.setFootprint(1, *footprint)
    coordinator.setRobotMaxVelocity(1, 1.5)
    coordinator.setRobotMaxAcceleration(1, 1.0)
    path = line_path(0.0, 0.0, 10.0, 0.0, step=0.5)
    stopping_pose = path[10].getPose()

    mission = Mission(1, path)
    mission.setStoppingPoint(stopping_pose, 3000)

    coordinator.placeRobot(1, path[0].getPose())
    coordinator.addMissions(mission)

    observed_stop = False
    for _ in range(1000):
        await asyncio.sleep(0.02)
        rr = coordinator.getRobotReport(1)
        if rr is not None and rr.getVelocity() <= 0.0 and 0 < rr.getPathIndex() < len(path) - 1:
            observed_stop = True
            break

    assert observed_stop, "robot never came to rest at the stopping point"

    await wait_until_idle(coordinator, timeout=10.0)
    rr = coordinator.getRobotReport(1)
    assert rr is not None and rr.getPathIndex() == -1


@pytest.mark.asyncio
async def test_rk4_two_robot_crossing_collision_free(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    coordinator.setRobotMaxVelocity(1, 1.5)
    coordinator.setRobotMaxAcceleration(1, 1.0)
    coordinator.setRobotMaxVelocity(2, 1.5)
    coordinator.setRobotMaxAcceleration(2, 1.0)
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))

    stop = asyncio.Event()
    monitor = asyncio.create_task(_assert_no_kinematic_collisions(coordinator, stop))
    try:
        await asyncio.sleep(0.1)
        assert len(coordinator.allCriticalSections) == 1
        await wait_until_idle(coordinator, timeout=20.0)
    finally:
        stop.set()
        await monitor


async def _assert_no_kinematic_collisions(
    tec: TrajectoryEnvelopeCoordinatorSimulation, stop: asyncio.Event
) -> None:
    """Stricter than the discrete monitor: check the *interpolated* footprint
    at the tracker's current arclength, not just the nearest waypoint footprint."""
    while not stop.is_set():
        live = []
        for robotID, tracker in tec.trackers.items():
            rr = tracker.getRobotReport()
            if rr is None or rr.getPose() is None:
                continue
            fp = place_footprint(tec.getFootprint(robotID), rr.getPose())
            live.append((robotID, fp))
        for i in range(len(live)):
            for j in range(i + 1, len(live)):
                rid_a, fa = live[i]
                rid_b, fb = live[j]
                assert not fa.intersects(fb), f"interpolated collision: r{rid_a} vs r{rid_b}"
        await asyncio.sleep(0.005)
