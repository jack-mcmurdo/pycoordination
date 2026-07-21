"""Tests for the 0.7.0 additions:

- ``criticalPointSafetyMargin``: index math of ``_indexAtMarginBefore``
  (including margin longer than the available path -> 0) and its effect on
  ``getCriticalPoint``.
- ``computeIsDeadlocked`` recording ``deadlockedCycles``.
- ``rePlanPath`` releasing the coordinator lock while a slow plan runs in
  its worker thread (three-phase lock scope).
"""

from __future__ import annotations

import asyncio
import time

import pytest

from coordination_oru.dependency import Dependency
from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.motionplanning.abstract_motion_planner import AbstractMotionPlanner
from coordination_oru.robot_report import RobotReport
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from tests.paths import line_path, two_robot_cross


# ------------------------------------------------------------------ margin


def test_index_at_margin_before_walks_back_by_arc_length() -> None:
    tec = TrajectoryEnvelopeCoordinatorSimulation()
    poses = [ps.getPose() for ps in line_path(0.0, 0.0, 10.0, 0.0, step=0.5)]

    # margin 0 (default): identity
    assert tec._indexAtMarginBefore(poses, 10) == 10
    assert tec._indexAtMarginBefore(poses, 0) == 0

    tec.setCriticalPointSafetyMargin(1.0)
    assert tec._indexAtMarginBefore(poses, 10) == 8  # exactly 2 x 0.5m steps

    tec.setCriticalPointSafetyMargin(1.1)
    assert tec._indexAtMarginBefore(poses, 10) == 7  # rounds outward, never short

    # margin longer than the whole path before the index -> clamps to 0
    tec.setCriticalPointSafetyMargin(1e6)
    assert tec._indexAtMarginBefore(poses, 10) == 0
    assert tec._indexAtMarginBefore(poses, len(poses) - 1) == 0

    with pytest.raises(ValueError):
        tec.setCriticalPointSafetyMargin(-0.1)


def test_critical_point_margin_moves_waiting_point_back() -> None:
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

    leading_index = 0  # robot 1 still before the CS
    baseline = tec.getCriticalPoint(2, cs, leading_index)

    margin = 2.0
    tec.setCriticalPointSafetyMargin(margin)
    with_margin = tec.getCriticalPoint(2, cs, leading_index)

    assert with_margin <= baseline

    # Arc length from the returned waiting point to the CS start must cover
    # the margin (unless clamped at the path start).
    poses = [ps.getPose() for ps in path_b]
    yielding_start = cs.getTe2Start() if cs.getTe2().getRobotID() == 2 else cs.getTe1Start()
    if with_margin > 0:
        arc = sum(poses[i].distanceTo(poses[i + 1]) for i in range(with_margin, yielding_start))
        assert arc >= margin


# --------------------------------------------------------- deadlockedCycles


class _FakeTracker:
    def __init__(self, rr: RobotReport) -> None:
        self._rr = rr

    def getLastRobotReport(self) -> RobotReport:
        return self._rr


def test_compute_is_deadlocked_records_cycles() -> None:
    tec = TrajectoryEnvelopeCoordinatorSimulation()
    tec.setupSolver()
    fp = footprint_coords(1.0, 0.6)
    tec.setFootprint(1, *fp)
    tec.setFootprint(2, *fp)

    path_a, path_b = two_robot_cross()
    te1 = tec.solver.createEnvelopeNoParking(1, path_a, "Driving", tec.getFootprint(1))
    te2 = tec.solver.createEnvelopeNoParking(2, path_b, "Driving", tec.getFootprint(2))

    # Same synthetic nonlive 2-cycle as the reordering regression test.
    dep_a = Dependency(te1, te2, waitingPoint=5, thresholdPoint=10)
    dep_b = Dependency(te2, te1, waitingPoint=3, thresholdPoint=20)
    tec.currentDependencies = {1: dep_a, 2: dep_b}

    tracker1 = _FakeTracker(RobotReport(1, Pose(0.0, 0.0, 0.0), 5, 0.0, 0.0, 5))
    tracker2 = _FakeTracker(RobotReport(2, Pose(1.0, 1.0, 0.0), 3, 0.0, 0.0, 3))
    tec.trackers = {1: tracker1, 2: tracker2}
    tec.communicatedCPs = {tracker1: (5, 0), tracker2: (3, 0)}

    assert tec.computeIsDeadlocked() is True
    assert len(tec.deadlockedCycles) == 1
    assert sorted(tec.deadlockedCycles[0]) == [1, 2]

    # Robot 1 not yet at its communicated CP -> nonlive cycle exists but is
    # not (yet) a deadlock; deadlockedCycles must be empty again.
    tracker1._rr = RobotReport(1, Pose(0.0, 0.0, 0.0), 2, 0.4, 0.0, 5)
    assert tec.computeIsDeadlocked() is False
    assert tec.deadlockedCycles == []


# ------------------------------------------------- lock scope during replan


@pytest.mark.asyncio
async def test_lock_released_while_slow_plan_runs(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation, footprint: tuple[tuple[float, float], ...]
) -> None:
    """The three-phase rePlanPath must not hold the coordinator lock while
    the motion planner works: a plan sleeping in its worker thread may not
    block other lock acquirers (inference, mission dispatch, preemption)."""

    class _SlowFailingPlanner(AbstractMotionPlanner):
        def doPlanning(self) -> bool:
            time.sleep(0.6)  # in asyncio.to_thread, so it must not block the loop
            return False

    coordinator.setFootprint(1, *footprint)
    path_a, _ = two_robot_cross()
    te1 = coordinator.solver.createEnvelopeNoParking(1, path_a, "Driving", coordinator.getFootprint(1))
    coordinator.replanningStoppingPoints[1] = Dependency(te1, None, 5, 0)
    coordinator.setMotionPlanner(1, _SlowFailingPlanner())

    replan = asyncio.create_task(coordinator.rePlanPath({1}, {1}))
    await asyncio.sleep(0.15)  # the plan is now sleeping in its thread
    assert not replan.done()

    loop = asyncio.get_running_loop()
    t0 = loop.time()
    async with coordinator._lock:
        waited = loop.time() - t0
    assert waited < 0.2, f"coordinator lock was held during the plan (waited {waited:.3f}s)"

    assert await replan is False
    assert 1 not in coordinator.replanningStoppingPoints
