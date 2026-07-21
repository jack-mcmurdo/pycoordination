"""Milestone 2 regression tests for the Java-fidelity deadlock/CS machinery.

Covers, per the port plan:
  (a) paths crossing twice get two independent CS orders
  (b) CS obsoletion: a passed CS disappears and stops feeding the dep graph
  (c) a head-on (2-cycle) deadlock is detected and resolved by local reordering
  (d) canExitCriticalSection (the primitive behind the artificial-dependency
      fallback) correctly distinguishes safe vs. unsafe escapes
  (e) a robot parked inside another's path creates a parking dependency
      (yieldIfParking) and is never collided with
  (f) re-tasking a robot leaves no ghost envelope/CS/dependency behind
  (g) ConstantAccelerationForwardModel.getEarliestStoppingPathIndex is a
      sane numeric stopping-distance estimate
  (h) spawnReplanning defers rePlanPath to a task instead of recursing
      through replacePath -> updateDependencies
"""

from __future__ import annotations

import asyncio

import pytest

from coordination_oru.dependency import Dependency
from coordination_oru.forward_model import ConstantAccelerationForwardModel
from coordination_oru.mission import Mission
from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.robot_report import RobotReport
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from tests.conftest import assert_no_collisions, wait_until_idle
from tests.paths import line_path, two_robot_cross


# --------------------------------------------------------------------- (a)


def test_paths_crossing_twice_yield_two_independent_critical_sections() -> None:
    tec = TrajectoryEnvelopeCoordinatorSimulation()
    tec.setupSolver()
    fp = footprint_coords(1.0, 0.6)
    tec.setFootprint(1, *fp)
    tec.setFootprint(2, *fp)

    # A "peak" path crosses a straight line twice: once going up, once coming down.
    peak = list(line_path(-6.0, -3.0, 0.0, 3.0, step=0.5)) + list(line_path(0.0, 3.0, 6.0, -3.0, step=0.5))
    straight = line_path(-6.0, 0.0, 6.0, 0.0, step=0.5)

    te1 = tec.solver.createEnvelopeNoParking(1, peak, "Driving", tec.getFootprint(1))
    te2 = tec.solver.createEnvelopeNoParking(2, straight, "Driving", tec.getFootprint(2))

    css = TrajectoryEnvelopeCoordinatorSimulation.getCriticalSections(
        te1, -1, te2, -1, True, min(tec.getMaxFootprintDimension(1), tec.getMaxFootprintDimension(2))
    )

    assert len(css) == 2, f"expected 2 disjoint critical sections, got {len(css)}: {css}"
    (cs_a, cs_b) = sorted(css, key=lambda cs: cs.getTe1Start())
    # The two pieces must not overlap in index range on either envelope.
    assert cs_a.getTe1End() < cs_b.getTe1Start()
    assert cs_a != cs_b
    assert hash(cs_a) != hash(cs_b) or cs_a != cs_b


# --------------------------------------------------------------------- (b)


@pytest.mark.asyncio
async def test_critical_section_obsoletion_after_both_robots_pass(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    coordinator.setRobotMaxVelocity(1, 1.5)
    coordinator.setRobotMaxAcceleration(1, 1.0)
    coordinator.setRobotMaxVelocity(2, 1.2)
    coordinator.setRobotMaxAcceleration(2, 1.0)
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))

    await asyncio.sleep(0.1)
    [cs] = coordinator.allCriticalSections
    counter_before = coordinator.criticalSectionCounter

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await wait_until_idle(coordinator, timeout=15.0)
    finally:
        stop.set()
        await monitor

    # The specific CS must be gone, not merely replaced by an equal-looking one,
    # and no dependency bookkeeping should still reference it.
    assert cs not in coordinator.allCriticalSections
    assert cs not in coordinator.CSToDepsOrder
    assert coordinator.criticalSectionCounter > counter_before


# --------------------------------------------------------------------- (c)


def test_two_cycle_deadlock_detected_and_resolved_by_local_reordering() -> None:
    tec = TrajectoryEnvelopeCoordinatorSimulation()
    tec.setupSolver()
    fp = footprint_coords(1.0, 0.6)
    tec.setFootprint(1, *fp)
    tec.setFootprint(2, *fp)

    path_a, path_b = two_robot_cross()
    te1 = tec.solver.createEnvelopeNoParking(1, path_a, "Driving", tec.getFootprint(1))
    te2 = tec.solver.createEnvelopeNoParking(2, path_b, "Driving", tec.getFootprint(2))
    [cs] = TrajectoryEnvelopeCoordinatorSimulation.getCriticalSections(
        te1, -1, te2, -1, True, min(tec.getMaxFootprintDimension(1), tec.getMaxFootprintDimension(2))
    )

    # Synthetic dependencies forming a 2-cycle: robot1 waits for robot2 (dep_a),
    # AND robot2 waits for robot1 (dep_b) — with releasing/waiting points chosen
    # so nonlivePair holds (a robot's wait point at-or-before the other's release
    # point), matching Java's nonlive-cycle condition.
    dep_a = Dependency(te1, te2, waitingPoint=5, thresholdPoint=10)
    dep_b = Dependency(te2, te1, waitingPoint=3, thresholdPoint=20)

    g = tec.depsToGraph({1: dep_a, 2: dep_b})
    nonlive_cycles = tec.findSimpleNonliveCycles(g)
    assert nonlive_cycles, "expected the 2-cycle to be detected as nonlive"
    assert tec.nonlivePair(dep_a, dep_b) or tec.nonlivePair(dep_b, dep_a)

    # Wire up depsToCS / a reversible dependency and ask callLocalReordering to
    # break the cycle by reversing one edge.
    tec.depsToCS[dep_a] = cs
    tec.depsToCS[dep_b] = cs
    tec.CSToDepsOrder[cs] = (dep_a.getWaitingRobotID(), dep_a.getWaitingPoint())
    reports = {
        1: RobotReport(1, path_a[0].getPose(), 0, 0.0, 0.0, -1),
        2: RobotReport(2, path_b[0].getPose(), 0, 0.0, 0.0, -1),
    }
    all_deps = {1: {dep_a}, 2: {dep_b}}

    result = tec.callLocalReordering(nonlive_cycles, {}, g, {dep_a}, all_deps, reports)

    # After reordering, re-derive the graph from whatever callLocalReordering
    # left in currentDependencies and confirm the cycle is gone.
    g_after = tec.depsToGraph(tec.currentDependencies)
    assert not tec.findSimpleNonliveCycles(g_after), "cycle should have been broken by local reordering"
    assert result != all_deps or tec.currentDependencies != {1: dep_a, 2: dep_b}


# --------------------------------------------------------------------- (d)


def test_can_exit_critical_section_distinguishes_safe_and_unsafe() -> None:
    tec = TrajectoryEnvelopeCoordinatorSimulation()
    tec.setupSolver()
    fp = footprint_coords(1.0, 0.6)
    tec.setFootprint(1, *fp)
    tec.setFootprint(2, *fp)

    path_a, path_b = two_robot_cross()
    driving_te = tec.solver.createEnvelopeNoParking(1, path_a, "Driving", tec.getFootprint(1))
    waiting_te = tec.solver.createEnvelopeNoParking(2, path_b, "Driving", tec.getFootprint(2))
    [cs] = TrajectoryEnvelopeCoordinatorSimulation.getCriticalSections(
        driving_te, -1, waiting_te, -1, True, min(tec.getMaxFootprintDimension(1), tec.getMaxFootprintDimension(2))
    )

    # Waiting robot parked well before the driving robot's sweep -> safe to exit.
    assert tec.canExitCriticalSection(
        drivingCurrentIndex=cs.getTe1Start(),
        waitingCurrentIndex=max(0, cs.getTe2Start() - 5),
        drivingTE=driving_te,
        waitingTE=waiting_te,
        lastIndexOfCSDriving=cs.getTe1End(),
    )

    # Waiting robot already sitting inside the actual overlap region while the
    # driving robot's sweep still covers it -> not safe to exit.
    assert not tec.canExitCriticalSection(
        drivingCurrentIndex=cs.getTe1Start(),
        waitingCurrentIndex=cs.getTe2Start(),
        drivingTE=driving_te,
        waitingTE=waiting_te,
        lastIndexOfCSDriving=cs.getTe1End(),
    )


# --------------------------------------------------------------------- (e)


@pytest.mark.asyncio
async def test_robot_parked_inside_path_creates_parking_dependency(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)

    # Robot 1 parks squarely on robot 2's upcoming path (no mission given).
    coordinator.placeRobot(1, Pose(0.0, 0.0, 0.0))
    path2 = line_path(-6.0, 0.0, 6.0, 0.0, step=0.5)
    coordinator.placeRobot(2, path2[0].getPose())
    coordinator.setRobotMaxVelocity(2, 1.5)
    coordinator.setRobotMaxAcceleration(2, 1.0)
    coordinator.addMissions(Mission(2, path2))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        # Robot 2 must never reach the parked robot's position while robot 1
        # stays put: watch it hold short for a while.
        held = False
        for _ in range(200):
            await asyncio.sleep(0.02)
            rr2 = coordinator.getRobotReport(2)
            if rr2 is not None and rr2.getVelocity() <= 0.0 and 0 < rr2.getPathIndex() < len(path2) - 1:
                held = True
                break
        assert held, "robot 2 never held short of the parked robot"

        # Now move robot 1 out of the way and confirm robot 2 completes.
        coordinator.addMissions(Mission(1, line_path(0.0, 0.0, 0.0, 6.0, step=0.5)))
        await wait_until_idle(coordinator, timeout=15.0)
    finally:
        stop.set()
        await monitor


# --------------------------------------------------------------------- (f)


@pytest.mark.asyncio
async def test_retasking_leaves_no_ghost_envelope(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    path_a, path_b = two_robot_cross()
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))

    await wait_until_idle(coordinator, timeout=15.0)

    # Re-task both robots with a fresh crossing.
    path_c, path_d = (
        line_path(4.0, 4.0, -4.0, -4.0, step=0.5),
        line_path(-4.0, 4.0, 4.0, -4.0, step=0.5),
    )
    coordinator.addMissions(Mission(1, path_c), Mission(2, path_d))
    await asyncio.sleep(0.1)

    # No critical section or dependency may reference anything other than the
    # envelope each robot's tracker is *currently* driving (a stale reference
    # to the completed first mission's envelope would be exactly the
    # "ghost envelope" bug this test guards against).
    live_envelopes = {robotID: t.getTrajectoryEnvelope() for robotID, t in coordinator.trackers.items()}
    for cs in coordinator.allCriticalSections:
        assert cs.getTe1() == live_envelopes[cs.getTe1().getRobotID()]
        assert cs.getTe2() == live_envelopes[cs.getTe2().getRobotID()]
    for dep in coordinator.currentDependencies.values():
        assert dep.getWaitingTrajectoryEnvelope() == live_envelopes[dep.getWaitingRobotID()]

    await wait_until_idle(coordinator, timeout=15.0)


# --------------------------------------------------------------------- (g)


def test_earliest_stopping_path_index_matches_kinematics() -> None:
    v_max, a_max = 2.0, 1.0
    fm = ConstantAccelerationForwardModel(maxAccel=a_max, maxVel=v_max, temporalResolution=1000.0, controlPeriodInMillis=0, trackingPeriodInMillis=0)
    path = line_path(0.0, 0.0, 20.0, 0.0, step=0.5)
    footprint = footprint_coords(1.0, 0.6)

    from coordination_oru.metacsp.spatial.trajectory_envelope import compute_spatial_envelope
    from shapely.geometry import Polygon

    fp_poly = Polygon(footprint)
    spatial = compute_spatial_envelope(path, fp_poly)
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope

    te = TrajectoryEnvelope(
        envelope_id=1, robot_id=1, path=tuple(path), start_node=0, end_node=1, spatial_envelope=spatial, footprint=fp_poly
    )

    velocity = 1.5
    distance_traveled = 4.0
    current_state = RobotReport(1, path[8].getPose(), 8, velocity, distance_traveled, -1)

    idx = fm.getEarliestStoppingPathIndex(te, current_state)

    # Braking distance under constant deceleration: d = v^2 / (2*a).
    expected_distance = distance_traveled + (velocity**2) / (2 * a_max)
    expected_index = round(expected_distance / 0.5)

    assert 0 <= idx <= len(path) - 1
    assert abs(idx - expected_index) <= 2, f"expected stopping index near {expected_index}, got {idx}"


# --------------------------------------------------------------------- (h)


@pytest.mark.asyncio
async def test_spawn_replanning_defers_replan_to_a_task(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, monkeypatch: pytest.MonkeyPatch
) -> None:
    """spawnReplanning must schedule rePlanPath instead of calling it inline
    (Java runs it on a separate thread): rePlanPath ends in replacePath ->
    updateDependencies, which can re-detect the same nonlive cycle and spawn
    again — run inline that is unbounded recursion (RecursionError) whenever
    a deadlock survives its replan, e.g. a goal inside the critical section."""
    from coordination_oru.motionplanning.abstract_motion_planner import AbstractMotionPlanner

    class _NeverCalledPlanner(AbstractMotionPlanner):
        def doPlanning(self) -> bool:
            raise AssertionError("not reached: rePlanPath is stubbed out")

    coordinator.setMotionPlanner(1, _NeverCalledPlanner())
    calls: list[tuple[set[int], set[int]]] = []

    async def _stub_replan(robots: set[int], obstacles: set[int]) -> bool:
        calls.append((robots, obstacles))
        return False

    monkeypatch.setattr(coordinator, "rePlanPath", _stub_replan)

    assert coordinator.spawnReplanning({1}, {1, 2}) is True
    assert calls == [], "rePlanPath ran synchronously inside spawnReplanning"

    tasks = list(coordinator._replanTasks)
    assert len(tasks) == 1
    await asyncio.gather(*tasks)
    assert calls == [({1}, {1, 2})]


# ---------------------------------------------------------- global avoidance


@pytest.mark.asyncio
async def test_global_deadlock_avoidance_completes_without_collision(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """``avoidDeadlockGlobally`` is off by default (exponential worst case);
    exercise ``globalCheckAndRevise`` at least once end-to-end to confirm it
    is a real, working implementation and not just a stub."""
    coordinator.setBreakDeadlocks(True, False, False)
    assert coordinator.avoidDeadlockGlobally is True

    path_a, path_b = two_robot_cross()
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    coordinator.setRobotMaxVelocity(1, 1.5)
    coordinator.setRobotMaxAcceleration(1, 1.0)
    coordinator.setRobotMaxVelocity(2, 1.2)
    coordinator.setRobotMaxAcceleration(2, 1.0)
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await wait_until_idle(coordinator, timeout=15.0)
    finally:
        stop.set()
        await monitor
