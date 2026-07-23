"""Dynamic missions: robots are given new work after their previous envelope
completes. Verifies the coordinator handles envelope lifecycle correctly:
no leaked critical sections, no leaked precedence orders.
"""

from __future__ import annotations

import asyncio
import math

import pytest

from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from tests.conftest import assert_no_collisions, wait_until_idle
from tests.paths import line_path, shuttle_path

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


async def test_truncate_mid_drive_parks_at_stopping_point(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """Re-tasking a driving robot: after truncateEnvelope the robot must brake,
    come to rest at the truncation point, and be parked *there* — not teleport
    to the superseded mission goal (regression: the end-parking envelope was
    captured at mission start and ignored mid-drive envelope replacement).
    """
    coordinator.setFootprint(1, *footprint)
    # 1 m/s at the default 1 m/s^2 deceleration -> braking distance ~0.5 m,
    # so the parked-near-braking-point bound below is physically principled.
    coordinator.setRobotMaxVelocity(1, 1.0)
    path = line_path(-8.0, 0.0, 8.0, 0.0)
    goal = path[-1].getPose()
    coordinator.placeRobot(1, path[0].getPose())
    coordinator.addMissions(Mission(1, path))

    # Let the robot get properly underway.
    deadline = asyncio.get_running_loop().time() + 10.0
    while True:
        report = coordinator.trackers[1].getRobotReport()
        if report is not None and report.getPose().getX() > -5.0:
            break
        assert asyncio.get_running_loop().time() < deadline, "robot never got underway"
        await asyncio.sleep(0.01)

    while not coordinator.truncateEnvelope(1):
        await asyncio.sleep(0.01)
    pose_at_truncation = coordinator.trackers[1].getRobotReport().getPose()

    # Watch the robot come to rest: the pose must evolve continuously (no
    # teleports) until it parks.
    last = pose_at_truncation
    deadline = asyncio.get_running_loop().time() + 10.0
    while coordinator.isDrivingRobot(1):
        assert asyncio.get_running_loop().time() < deadline, "robot did not stop after truncation"
        await asyncio.sleep(0.01)
        now = coordinator.trackers[1].getRobotReport().getPose()
        step = math.hypot(now.getX() - last.getX(), now.getY() - last.getY())
        assert step < 0.5, f"pose teleported by {step:.2f} m while stopping"
        last = now

    parked = coordinator.trackers[1].getRobotReport().getPose()
    dist_to_goal = math.hypot(parked.getX() - goal.getX(), parked.getY() - goal.getY())
    dist_from_truncation = math.hypot(
        parked.getX() - pose_at_truncation.getX(), parked.getY() - pose_at_truncation.getY()
    )
    assert dist_to_goal > 2.0, f"robot parked at the superseded goal (within {dist_to_goal:.2f} m)"
    assert dist_from_truncation < 1.5, (
        f"robot parked {dist_from_truncation:.2f} m from its braking point"
    )

    # Re-task from the rest pose: the follow-up mission must complete.
    assert coordinator.addMissions(Mission(1, line_path(parked.getX(), parked.getY(), 0.0, 6.0)))
    deadline = asyncio.get_running_loop().time() + 5.0
    while not coordinator.isDrivingRobot(1):
        assert asyncio.get_running_loop().time() < deadline, "re-tasked mission never dispatched"
        await asyncio.sleep(0.01)
    await wait_until_idle(coordinator, timeout=20.0)
    final = coordinator.trackers[1].getRobotReport().getPose()
    assert math.hypot(final.getX() - 0.0, final.getY() - 6.0) < 0.5


async def test_finished_mission_retires_envelopes(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """A finished mission's driving envelope and the parking envelope it
    superseded must both retire (cleanUp: solver.mark_completed, dropped
    from currentParkingEnvelopes) — otherwise every mission, forever, stays
    "active" in the solver and can spuriously conflict with robots long
    gone from that space (the ghost-critical-section class of bug)."""
    coordinator.setFootprint(1, *footprint)
    start_pose = (-4.0, 0.0)
    path = shuttle_path(start_pose, (4.0, 0.0))
    coordinator.placeRobot(1, path[0].getPose())
    start_parking = coordinator.trackers[1].getTrajectoryEnvelope()

    coordinator.addMissions(Mission(1, path))
    deadline = asyncio.get_running_loop().time() + 5.0
    while not coordinator.isDrivingRobot(1):
        assert asyncio.get_running_loop().time() < deadline, "mission never dispatched"
        await asyncio.sleep(0.01)
    driving_te = coordinator.trackers[1].getTrajectoryEnvelope()
    assert driving_te.envelope_id != start_parking.envelope_id

    await wait_until_idle(coordinator, timeout=10.0)

    assert start_parking.completed, "the superseded parking envelope was never retired"
    assert driving_te.completed, "the finished driving envelope was never retired"
    assert start_parking not in coordinator.currentParkingEnvelopes
    assert driving_te not in coordinator.currentParkingEnvelopes
    active_ids = {te.envelope_id for te in coordinator.solver.envelopes()}
    assert start_parking.envelope_id not in active_ids
    assert driving_te.envelope_id not in active_ids
    # The envelope_id/TrajectoryEnvelope objects themselves stay around (for
    # introspection, all_envelopes()) — only their underlying STP variables
    # are actually detached (test_stp_working_set_stays_bounded_across_
    # sequential_missions below proves that half of it).
    all_ids = {te.envelope_id for te in coordinator.solver.all_envelopes()}
    assert {start_parking.envelope_id, driving_te.envelope_id} <= all_ids


async def test_stp_working_set_stays_bounded_across_sequential_missions(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """cleanUp must actually detach a retired envelope's STP variables
    (STPSolver.remove_variable), not just flag it completed: this port's
    STPSolver is a dense distance matrix sized for the live variable count,
    unlike Java's sparse ConstraintNetwork graph (verified against the real
    org.metacsp.framework source) — without freeing slots for reuse, the
    working set grows without bound for every mission a robot ever runs
    over the coordinator's lifetime, exactly the "matrix keeps every
    completed mission's entries forever" bug this guards against.
    """
    coordinator.setFootprint(1, *footprint)
    coordinator.placeRobot(1, Pose(-4.0, 0.0, 0.0))

    # Two missions is enough for num_variables to plateau instead of
    # climbing: each mission allocates 2 STP nodes (start/end) for its
    # driving envelope plus 2 for the parking envelope placeRobot creates
    # at the end -- if those aren't freed, num_variables keeps climbing by
    # ~4 every mission forever.
    after_first: int | None = None
    for i in range(5):
        x = -4.0 if i % 2 == 0 else 4.0
        goal_x = 4.0 if i % 2 == 0 else -4.0
        assert coordinator.addMissions(Mission(1, line_path(x, 0.0, goal_x, 0.0)))
        deadline = asyncio.get_running_loop().time() + 5.0
        while not coordinator.isDrivingRobot(1):
            assert asyncio.get_running_loop().time() < deadline, f"mission {i} never dispatched"
            await asyncio.sleep(0.01)
        await wait_until_idle(coordinator, timeout=10.0)
        if i == 0:
            after_first = coordinator.solver.stp.num_variables
        else:
            assert coordinator.solver.stp.num_variables == after_first, (
                f"STP working set grew after mission {i}: "
                f"{coordinator.solver.stp.num_variables} vs {after_first} after mission 0 "
                "-- retired envelopes' variables are not being freed"
            )
    # All_envelopes() bookkeeping is expected to grow (cheap Python objects,
    # kept for introspection) -- only the STP working set must stay flat.
    assert len(coordinator.solver.all_envelopes()) >= 10



async def test_cleanup_stale_parking_envelope_helper(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """cleanUpStaleParkingEnvelope(robotID) -- used by callers that end a
    mission a different way than onTrackingFinished (e.g. a hard-drop
    cancel) -- retires a robot's pre-mission parking envelope on demand."""
    coordinator.setFootprint(1, *footprint)
    coordinator.placeRobot(1, Pose(0.0, 0.0, 0.0))
    parking = coordinator.trackers[1].getTrajectoryEnvelope()
    assert parking in coordinator.currentParkingEnvelopes
    assert not parking.completed

    coordinator.cleanUpStaleParkingEnvelope(1)

    assert parking.completed
    assert parking not in coordinator.currentParkingEnvelopes
    # idempotent: a robot with no stale parking envelope left is a no-op.
    coordinator.cleanUpStaleParkingEnvelope(1)
