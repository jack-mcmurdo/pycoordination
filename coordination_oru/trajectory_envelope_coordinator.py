"""``TrajectoryEnvelopeCoordinator``: the full dependency/deadlock pipeline.

Ported from Java's ``TrajectoryEnvelopeCoordinator``. Two mutually exclusive
strategies for ``updateDependencies()``:

* ``localCheckAndRevise`` (default) — per-critical-section local ordering,
  with local reordering (``callLocalReordering``) and/or one-path replanning
  (``callOnePathReplan``) to break detected nonlive cycles.
* ``globalCheckAndRevise`` (behind ``avoidDeadlockGlobally``, off by default
  — exponential worst case) — precedence pre-loaded via FCFS/previous
  decisions, then revised per a global heuristic while checking that no
  reversal introduces a nonlive cycle across the whole fleet.

``breakDeadlocksByReplanning`` calls into ``AbstractMotionPlanner`` — with no
planner injected (the default), replanning attempts simply fail and the
local-reordering / artificial-dependency fallback takes over, exactly as in
Java when no planner is configured for a robot.
"""

from __future__ import annotations

import asyncio
import itertools
from typing import TYPE_CHECKING, Callable

import networkx as nx

from coordination_oru.abstract_trajectory_envelope_coordinator import (
    AbstractTrajectoryEnvelopeCoordinator,
)
from coordination_oru.critical_section import CriticalSection
from coordination_oru.dependency import Dependency
from coordination_oru.robot_at_critical_section import RobotAtCriticalSection
from coordination_oru.robot_report import RobotReport
from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)
from coordination_oru.util.logging import get_logger

if TYPE_CHECKING:
    from coordination_oru.abstract_trajectory_envelope_tracker import (
        AbstractTrajectoryEnvelopeTracker,
    )
    from coordination_oru.metacsp.spatial.pose import PoseSteering
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope

log = get_logger(__name__)


class TrajectoryEnvelopeCoordinator(AbstractTrajectoryEnvelopeCoordinator):
    def __init__(self, CONTROL_PERIOD: int = 1000, TEMPORAL_RESOLUTION: float = 1000.0) -> None:
        super().__init__(CONTROL_PERIOD, TEMPORAL_RESOLUTION)

        # currentOrdersGraph/currentCyclesList mirror allCriticalSections (kept
        # in sync via addEdges/deleteEdges) and are only load-bearing when
        # avoidDeadlockGlobally is enabled.
        self.currentOrdersGraph: nx.DiGraph = nx.DiGraph()
        self.currentCyclesList: dict[tuple[int, int], set[tuple[int, ...]]] = {}

        self.replanningStoppingPoints: dict[int, Dependency] = {}
        self._replanTasks: set[asyncio.Task[None]] = set()

        self.breakDeadlocksByReordering = True
        self.breakDeadlocksByReplanning = True
        self.avoidDeadlockGlobally = False

        self.nonliveStatesDetected = 0
        self.nonliveStatesAvoided = 0
        self.currentOrdersHeurusticallyDecided = 0
        self.nonliveCyclesOld: list[list[int]] = []
        self.replanningTrialsCounter = 0
        self._noPlannerWarned: set[int] = set()
        self.successfulReplanningTrialsCounter = 0
        self.forceCriticalPointReTransmission: dict[int, bool] = {}

        self.staticReplan = False
        self.isBlocked = False
        self.isDeadlockedFlag = False
        self.deadlockedCycles: list[list[int]] = []

        self.deadlockedCallback: Callable[[], None] | None = None
        self.replanFailedCallback: Callable[[set[int]], None] | None = None
        self.fake = False

    # ---------------------------------------------------------------- config

    def isBlockedFleet(self) -> bool:
        return self.isBlocked

    def setBreakDeadlocks(self, global_: bool, reorder: bool, replan: bool) -> None:
        if global_ and (reorder or replan):
            log.error(
                "invalid_break_deadlocks_config",
                msg="Enable either the global or the local strategies, not both. Using defaults (local reorder+replan).",
            )
            self.avoidDeadlockGlobally = False
            self.breakDeadlocksByReordering = True
            self.breakDeadlocksByReplanning = True
            return
        self.avoidDeadlockGlobally = global_
        self.breakDeadlocksByReordering = reorder
        self.breakDeadlocksByReplanning = replan

    def setDeadlockedCallback(self, cb: Callable[[], None]) -> None:
        self.deadlockedCallback = cb

    def setReplanFailedCallback(self, cb: Callable[[set[int]], None]) -> None:
        """Called from :meth:`rePlanPath` — after the
        ``replanningStoppingPoints`` pins are popped — when no robot in the
        set could be replanned, with the ``robotsToReplan`` set. Lets the
        deployment decide what to do with an unresolvable deadlock (e.g.
        cancel the missions and park the robots). Not called when the replan
        task is cancelled."""
        self.replanFailedCallback = cb

    def setStaticReplan(self, value: bool) -> None:
        self.staticReplan = value

    def setFakeCoordination(self, fake: bool) -> None:
        self.fake = fake

    # ------------------------------------------------------------------ cycles

    def depsToGraph(self, deps: dict[int, Dependency]) -> nx.DiGraph:
        g = nx.DiGraph()
        for dep in deps.values():
            g.add_node(dep.getWaitingRobotID())
            g.add_node(dep.getDrivingRobotID())
            g.add_edge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), dep=dep)
        return g

    def nonlivePair(self, dep1: Dependency, dep2: Dependency) -> bool:
        return dep2.getWaitingPoint() <= dep1.getReleasingPoint()

    def findSimpleNonliveCycles(self, g: nx.DiGraph) -> list[list[int]]:
        nonlive_cycles: list[list[int]] = []
        for cycle in nx.simple_cycles(g):
            if len(cycle) < 2:
                continue
            edges_along_cycle: list[Dependency] = []
            for i in range(len(cycle)):
                j = i + 1 if i < len(cycle) - 1 else 0
                data = g.get_edge_data(cycle[i], cycle[j])
                if data is not None:
                    edges_along_cycle.append(data["dep"])
            is_nonlive = False
            for i in range(len(edges_along_cycle)):
                j = i + 1 if i < len(edges_along_cycle) - 1 else 0
                if self.nonlivePair(edges_along_cycle[i], edges_along_cycle[j]):
                    is_nonlive = True
                    break
            if is_nonlive:
                nonlive_cycles.append(cycle)
        return nonlive_cycles

    def computeClosestDependencies(
        self,
        allDeps: dict[int, set[Dependency]],
        artificialDeps: dict[int, set[Dependency]],
    ) -> dict[int, Dependency]:
        closestDeps: dict[int, Dependency] = {}
        if not allDeps and not artificialDeps:
            return closestDeps

        robotIDs = set(allDeps.keys()) | set(artificialDeps.keys())
        for robotID in robotIDs:
            firstDep = min(allDeps[robotID]) if allDeps.get(robotID) else None
            firstArtificialDep = min(artificialDeps[robotID]) if artificialDeps.get(robotID) else None
            if firstDep is not None:
                depToSend = (
                    firstDep if firstArtificialDep is None or firstDep.compareTo(firstArtificialDep) < 0 else firstArtificialDep
                )
            else:
                depToSend = firstArtificialDep
            assert depToSend is not None
            closestDeps[depToSend.getWaitingRobotID()] = depToSend
        return closestDeps

    def computeIsDeadlocked(self) -> bool:
        """Recompute :attr:`isDeadlockedFlag` without firing the
        deadlocked callback — safe for observers (e.g. viewers) to poll.
        Also records :attr:`deadlockedCycles`: every nonlive cycle whose
        robots have all come to rest at their communicated critical points
        (``[]`` when none has)."""
        g = self.depsToGraph(self.currentDependencies)
        nonlive_cycles = self.findSimpleNonliveCycles(g)
        deadlockedCycles: list[list[int]] = []
        for cycle in nonlive_cycles:
            allStopped = True
            for robotID in cycle:
                tracker = self.trackers[robotID]
                rr = tracker.getLastRobotReport()
                communicated = self.communicatedCPs.get(tracker)
                if not (
                    communicated is not None
                    and communicated[0] == rr.getCriticalPoint()
                    and rr.getCriticalPoint() == rr.getPathIndex()
                ):
                    allStopped = False
                    break
            if allStopped:
                deadlockedCycles.append(list(cycle))
        self.deadlockedCycles = deadlockedCycles
        self.isDeadlockedFlag = bool(deadlockedCycles)
        return self.isDeadlockedFlag

    def isDeadlocked(self) -> bool:
        self.computeIsDeadlocked()
        if self.deadlockedCallback is not None and self.isDeadlockedFlag:
            self.deadlockedCallback()
        return self.isDeadlockedFlag

    # ------------------------------------------------------ local reordering

    def findAndRepairNonliveCycles(
        self,
        currentDeps: dict[int, set[Dependency]],
        artificialDeps: dict[int, set[Dependency]],
        reversibleDeps: set[Dependency],
        currentReports: dict[int, RobotReport],
    ) -> dict[int, set[Dependency]]:
        allDeps = {robotID: set(deps) for robotID, deps in currentDeps.items()}

        g = self.depsToGraph(self.currentDependencies)
        nonlive_cycles = self.findSimpleNonliveCycles(g)
        self.nonliveStatesDetected += len(nonlive_cycles)

        if self.breakDeadlocksByReordering:
            allDeps = self.callLocalReordering(nonlive_cycles, artificialDeps, g, reversibleDeps, allDeps, currentReports)

        if self.breakDeadlocksByReplanning:
            for cycle in nonlive_cycles:
                self.callOnePathReplan(cycle, g)

        return allDeps

    def callLocalReordering(
        self,
        nonlive_cycles: list[list[int]],
        artificialDeps: dict[int, set[Dependency]],
        g: nx.DiGraph,
        reversibleDeps: set[Dependency],
        allDeps: dict[int, set[Dependency]],
        currentReports: dict[int, RobotReport],
    ) -> dict[int, set[Dependency]]:
        self.nonliveCyclesOld = [c for c in nonlive_cycles]

        counter = 0
        while counter < len(nonlive_cycles):
            cycle = nonlive_cycles[counter]
            counter += 1

            reversible_deps_along_cycle: list[Dependency] = []
            for i in range(len(cycle)):
                j = i + 1 if i < len(cycle) - 1 else 0
                data = g.get_edge_data(cycle[i], cycle[j])
                dep = data["dep"] if data is not None else None
                if dep is not None and dep in reversibleDeps and dep.getWaitingRobotID() not in artificialDeps:
                    reversible_deps_along_cycle.append(dep)

            for dep in reversible_deps_along_cycle:
                if dep not in self.depsToCS:
                    continue
                cs = self.depsToCS[dep]

                waitingRobotID = dep.getDrivingRobotID()
                waitingTE = dep.getDrivingTrajectoryEnvelope()
                drivingRobotID = dep.getWaitingRobotID()
                drivingTE = dep.getWaitingTrajectoryEnvelope()
                drivingCurrentIndex = currentReports[drivingRobotID].getPathIndex()
                waitingPoint = self.getCriticalPoint(waitingRobotID, cs, drivingCurrentIndex)
                drivingCSEnd = cs.getTe1End() if drivingRobotID == cs.getTe1().getRobotID() else cs.getTe2End()

                assert waitingTE is not None
                # Only reverse if the newly-waiting robot can still honour the
                # new waiting point (which sits TRAILING_PATH_POINTS before
                # the conflict, i.e. earlier than the CS start its
                # reversibility was judged against). Otherwise its tracker
                # would silently reject the critical point and both robots
                # would enter the critical section.
                newWaitingTracker = self.trackers.get(waitingRobotID)
                if newWaitingTracker is not None:
                    newWaitingReport = currentReports[waitingRobotID]
                    earliest = {
                        waitingRobotID: self.getForwardModel(waitingRobotID).getEarliestStoppingPathIndex(
                            waitingTE, newWaitingReport
                        )
                    }
                    if not self._canStop(newWaitingTracker, newWaitingReport, waitingPoint + 1, earliest):
                        continue
                revDep = Dependency(waitingTE, drivingTE, waitingPoint, drivingCSEnd)

                allDepsTmp = {rid: set(deps) for rid, deps in allDeps.items()}
                allDepsTmp.setdefault(revDep.getWaitingRobotID(), set()).add(revDep)
                if dep.getWaitingRobotID() in allDepsTmp:
                    allDepsTmp[dep.getWaitingRobotID()].discard(dep)
                    if not allDepsTmp[dep.getWaitingRobotID()]:
                        del allDepsTmp[dep.getWaitingRobotID()]

                CSToDepsOrderTmp = dict(self.CSToDepsOrder)
                CSToDepsOrderTmp[cs] = (revDep.getWaitingRobotID(), revDep.getWaitingPoint())

                depsToCSTmp = dict(self.depsToCS)
                depsToCSTmp.pop(dep, None)
                depsToCSTmp[revDep] = cs

                currentDepsTmp = self.computeClosestDependencies(allDepsTmp, artificialDeps)
                gTmp = self.depsToGraph(currentDepsTmp)

                nonlive_cycles_tmp = self.findSimpleNonliveCycles(gTmp)
                if len(nonlive_cycles_tmp) < len(nonlive_cycles):
                    self.nonliveStatesAvoided += 1
                    log.info(
                        "reversed_precedence_to_break_deadlock",
                        waiting=revDep.getWaitingRobotID(),
                        driving=revDep.getDrivingRobotID(),
                    )

                    counter = 0
                    g = gTmp
                    self.currentDependencies.clear()
                    self.currentDependencies.update(currentDepsTmp)
                    allDeps = allDepsTmp
                    nonlive_cycles = nonlive_cycles_tmp
                    self.CSToDepsOrder.clear()
                    self.CSToDepsOrder.update(CSToDepsOrderTmp)
                    self.depsToCS.clear()
                    self.depsToCS.update(depsToCSTmp)
                    break

        return allDeps

    # ------------------------------------------------------------- replanning

    def replanEnvelope(self, robotID: int, onlyIfDeadlocks: bool = False) -> bool:
        g = self.depsToGraph(self.currentDependencies)
        if onlyIfDeadlocks:
            nonlive_cycles = self.findSimpleNonliveCycles(g)
            self.nonliveStatesDetected += len(nonlive_cycles)
            for cycle in nonlive_cycles:
                if robotID in cycle:
                    return self.callOnePathReplan(cycle, g)
        return self.callOnePathReplan([robotID], g)

    def callOnePathReplan(self, cycle: list[int], g: nx.DiGraph) -> bool:
        robotsToReplan: set[int] = set()
        for i in range(len(cycle)):
            robotID = cycle[i]
            j = i + 1 if i < len(cycle) - 1 else 0
            data = g.get_edge_data(cycle[i], cycle[j]) if len(cycle) > 1 else None
            dep = data["dep"] if data is not None else None
            if dep is None:
                tracker = self.trackers.get(robotID)
                if isinstance(tracker, TrajectoryEnvelopeTrackerDummy):
                    return False
                assert tracker is not None
                te = tracker.getTrajectoryEnvelope()
                waitingPoint = self.getForwardModel(robotID).getEarliestStoppingPathIndex(te, tracker.getLastRobotReport())
                dep = Dependency(te, None, waitingPoint, 0)
                log.info("added_replan_stopping_point", robotID=robotID, waitingPoint=waitingPoint)
                self.currentDependencies[robotID] = dep
                self.setCriticalPoint(robotID, waitingPoint, True)

            if dep.getDrivingTrajectoryEnvelope() is not None:
                robotsToReplan.add(dep.getDrivingRobotID())
            robotsToReplan.add(dep.getWaitingRobotID())
            rrWaiting = self.getRobotReport(dep.getWaitingRobotID())
            assert rrWaiting is not None

            drivingParked = dep.getDrivingTrajectoryEnvelope() is not None and self.inParkingPose(dep.getDrivingRobotID())
            waitingParked = self.inParkingPose(dep.getWaitingRobotID())
            static_wait = self.staticReplan and (
                (rrWaiting.getPathIndex() < 0)
                if dep.getWaitingPoint() == 0
                else (rrWaiting.getPathIndex() < dep.getWaitingPoint() - 1)
            )
            if drivingParked or waitingParked or static_wait:
                return False

        undirected = g.to_undirected()
        allConnectedRobots: set[int] = (
            set(nx.node_connected_component(undirected, cycle[0])) if cycle and cycle[0] in undirected else set()
        )
        return self.spawnReplanning(robotsToReplan, allConnectedRobots)

    def spawnReplanning(self, robotsToReplan: set[int], allConnectedRobots: set[int]) -> bool:
        if not any(robotID in self.motionPlanners for robotID in robotsToReplan):
            # Nothing can be replanned. Without this guard a plannerless sim
            # (e.g. the examples) re-logs the spawn every control period for
            # as long as the blocked cycle persists.
            unwarned = robotsToReplan - self._noPlannerWarned
            if unwarned:
                log.warning("no_motion_planner_for_replan", robots=sorted(robotsToReplan))
                self._noPlannerWarned |= unwarned
            return False
        if self.setMaxCPDependencies(robotsToReplan):
            log.info("spawning_replan", robots=sorted(robotsToReplan), connected=sorted(allConnectedRobots))
            # Deferred like Java's replanning thread: rePlanPath ends in
            # replacePath -> updateDependencies, which can re-detect the same
            # nonlive cycle and spawn again — run inline that is unbounded
            # recursion. The robots stay locked via replanningStoppingPoints
            # until the task runs, so no duplicate spawn can sneak in between.
            task = asyncio.get_running_loop().create_task(
                self._rePlanPathTask(robotsToReplan, allConnectedRobots),
                name=f"replan-{'-'.join(str(r) for r in sorted(robotsToReplan))}",
            )
            self._replanTasks.add(task)
            task.add_done_callback(self._replanTasks.discard)
            return True
        return False

    async def _rePlanPathTask(self, robotsToReplan: set[int], allConnectedRobots: set[int]) -> None:
        # rePlanPath scopes the lock itself (gather -> plan unlocked -> commit),
        # so a slow or unplannable plan never freezes the rest of the location.
        await self.rePlanPath(robotsToReplan, allConnectedRobots)

    async def stopInference(self) -> None:
        await super().stopInference()
        for task in list(self._replanTasks):
            task.cancel()
        if self._replanTasks:
            await asyncio.gather(*self._replanTasks, return_exceptions=True)
        # a cancelled replan task never reached its unlock in rePlanPath
        self.replanningStoppingPoints.clear()

    def setMaxCPDependencies(self, robotIDs: set[int]) -> bool:
        for robotID in robotIDs:
            if robotID in self.replanningStoppingPoints:
                return False
        currentDeps = self.getCurrentDependencies()
        for robotID in robotIDs:
            if robotID in currentDeps:
                dep = currentDeps[robotID]
                self.replanningStoppingPoints[dep.getWaitingRobotID()] = Dependency(
                    dep.getWaitingTrajectoryEnvelope(), None, dep.getWaitingPoint(), 0
                )
        return True

    async def rePlanPath(self, robotsToReplan: set[int], robotsAsObstacles: set[int]) -> bool:
        """Three-phase lock scope per robot: gather dep/pose/obstacle info
        under ``self._lock``, run the (possibly slow, possibly remote) plan
        with the lock released — the involved robots stay pinned via
        ``replanningStoppingPoints`` — and re-acquire only for the
        ``replacePath`` commit."""
        ret = False
        try:
            for robotID in sorted(robotsToReplan):
                async with self._lock:
                    currentDeps = self.getCurrentDependencies()
                    dep: Dependency | None
                    if len(robotsToReplan) == 1:
                        dep = self.replanningStoppingPoints.get(robotID)
                        if dep is None:
                            log.info("invalid_replan", robotID=robotID)
                            continue
                    else:
                        dep = currentDeps.get(robotID)
                        if dep is None:
                            log.info("robot_not_deadlocked", robotID=robotID)
                            continue

                    currentWaitingIndex = dep.getWaitingPoint()
                    currentWaitingPose = dep.getWaitingPose()
                    oldPath: tuple["PoseSteering", ...] = dep.getWaitingTrajectoryEnvelope().getTrajectory().getPoseSteering()
                    currentWaitingGoal = oldPath[-1].getPose()

                    obstacles = []
                    otherRobotIDs = {rid for rid in robotsAsObstacles if rid != robotID}
                    if otherRobotIDs:
                        obstacles = self.getObstaclesInCriticalPoints(list(otherRobotIDs))

                    mp = self.motionPlanners.get(robotID)
                    if mp is None:
                        log.warning("no_motion_planner_for_replan", robotID=robotID)
                        continue

                newPath = await self.doReplanning(mp, currentWaitingPose, currentWaitingGoal, obstacles)
                self.replanningTrialsCounter += 1
                if newPath:
                    newCompletePath = list(oldPath[:currentWaitingIndex]) + list(newPath)
                    async with self._lock:
                        self.replacePath(robotID, newCompletePath, currentWaitingIndex, robotsToReplan)
                    self.successfulReplanningTrialsCounter += 1
                    log.info("replan_succeeded", robotID=robotID)
                    ret = True
                    break
                log.info("replan_failed", robotID=robotID)
        finally:
            # No await between here and the pops: runs even when the task is
            # cancelled mid-plan, without re-entering the lock.
            for robotID in robotsToReplan:
                self.replanningStoppingPoints.pop(robotID, None)
        # Past the finally, so a cancelled replan never reports failure.
        if not ret and self.replanFailedCallback is not None:
            self.replanFailedCallback(set(robotsToReplan))
        return ret

    def replacePath(
        self,
        robotID: int,
        newPath: list["PoseSteering"],
        breakingPathIndex: int,
        lockedRobotIDs: set[int] | None,
        concatenatePaths: bool = True,
    ) -> None:
        if robotID not in self.trackers:
            log.warning("invalid_robot_for_replace_path", robotID=robotID)
            return

        te = self.getCurrentTrajectoryEnvelope(robotID)
        log.info("replacing_envelope", robotID=robotID, breakingPathIndex=breakingPathIndex)

        holdingCS: dict[CriticalSection, tuple[int, int]] = {}
        for cs, order in self.CSToDepsOrder.items():
            if (cs.getTe1().getRobotID() == robotID and cs.getTe1Start() <= breakingPathIndex) or (
                cs.getTe2().getRobotID() == robotID and cs.getTe2Start() <= breakingPathIndex
            ):
                holdingCS[cs] = order

        self.cleanUpRobotCS(robotID, breakingPathIndex)

        assert self.solver is not None
        newTE = self.solver.createEnvelopeNoParking(robotID, newPath, "Driving", self.getFootprint(robotID))
        self.trackers[robotID].updateTrajectoryEnvelope(newTE)

        self.envelopesToTrack.append(newTE)
        self.computeCriticalSections()

        offset = 0 if concatenatePaths else -breakingPathIndex
        for cs1 in self.allCriticalSections:
            if not (
                (cs1.getTe1().getRobotID() == robotID and cs1.getTe1Start() <= breakingPathIndex + offset)
                or (cs1.getTe2().getRobotID() == robotID and cs1.getTe2Start() <= breakingPathIndex + offset)
            ):
                continue
            for cs2, order in holdingCS.items():
                same_pair = (
                    cs1.getTe1().getRobotID() == cs2.getTe1().getRobotID()
                    and cs1.getTe2().getRobotID() == cs2.getTe2().getRobotID()
                ) or (
                    cs1.getTe1().getRobotID() == cs2.getTe2().getRobotID()
                    and cs1.getTe2().getRobotID() == cs2.getTe1().getRobotID()
                )
                if not same_pair:
                    continue

                start11 = cs1.getTe1Start() if cs1.getTe1().getRobotID() == robotID else cs1.getTe2Start()
                start12 = cs1.getTe2Start() if cs1.getTe1().getRobotID() == robotID else cs1.getTe1Start()
                end11 = cs1.getTe1End() if cs1.getTe1().getRobotID() == robotID else cs1.getTe2End()
                end12 = cs1.getTe2End() if cs1.getTe1().getRobotID() == robotID else cs1.getTe1End()
                start21 = (cs2.getTe1Start() + offset) if cs2.getTe1().getRobotID() == robotID else cs2.getTe2Start()
                start22 = (cs2.getTe2Start() + offset) if cs2.getTe1().getRobotID() == robotID else cs2.getTe1Start()
                end21 = (cs2.getTe1End() + offset) if cs2.getTe1().getRobotID() == robotID else cs2.getTe2End()
                end22 = (cs2.getTe2End() + offset) if cs2.getTe1().getRobotID() == robotID else cs2.getTe1End()

                if (start21 <= start11 <= end21) and not (end22 < start12 or end12 < start22):
                    self.CSToDepsOrder[cs1] = order
                    if self.avoidDeadlockGlobally:
                        waitingRobotID = order[0]
                        drivingRobotID = (
                            cs2.getTe2().getRobotID() if waitingRobotID == cs2.getTe1().getRobotID() else cs2.getTe1().getRobotID()
                        )
                        self.addEdges({(waitingRobotID, drivingRobotID): 1})
                    break

        if newTE in self.envelopesToTrack:
            self.envelopesToTrack.remove(newTE)

        if lockedRobotIDs is not None:
            for rid in lockedRobotIDs:
                self.replanningStoppingPoints.pop(rid, None)

        self.forceCriticalPointReTransmission[robotID] = True
        self.updateDependencies()

    def truncateEnvelope(self, robotID: int) -> bool:
        return self.truncateEnvelopeAt(robotID, -1)

    def truncateEnvelopeAt(self, robotID: int, pathIndex: int) -> bool:
        if robotID in self.replanningStoppingPoints:
            return False

        te = self.getCurrentTrajectoryEnvelope(robotID)
        tet = self.trackers.get(robotID)
        if isinstance(tet, TrajectoryEnvelopeTrackerDummy) or tet is None:
            return True

        earliestStoppingPathIndex = self.getForwardModel(robotID).getEarliestStoppingPathIndex(te, self.getRobotReport(robotID))
        lastCommunicatedCP = self.communicatedCPs.get(tet, (-1, 0))[0]
        if lastCommunicatedCP != -1:
            earliestStoppingPathIndex = min(lastCommunicatedCP, earliestStoppingPathIndex)
        if pathIndex != -1 and pathIndex < earliestStoppingPathIndex:
            return False

        stoppingIndex = pathIndex if pathIndex != -1 else earliestStoppingPathIndex
        truncatedPath = list(te.getTrajectory().getPoseSteering()[: stoppingIndex + 1])
        self.replacePath(robotID, truncatedPath, len(truncatedPath) - 1, {robotID})
        log.info("truncated_envelope", robotID=robotID, stoppingIndex=stoppingIndex)
        return True

    def reverseEnvelope(self, robotID: int) -> bool:
        if robotID in self.replanningStoppingPoints:
            return False

        te = self.getCurrentTrajectoryEnvelope(robotID)
        tet = self.trackers.get(robotID)
        if isinstance(tet, TrajectoryEnvelopeTrackerDummy) or tet is None:
            return True

        earliestStoppingPathIndex = self.getForwardModel(robotID).getEarliestStoppingPathIndex(te, self.getRobotReport(robotID))
        lastCommunicatedCP = self.communicatedCPs.get(tet, (-1, 0))[0]
        if lastCommunicatedCP != -1:
            earliestStoppingPathIndex = min(lastCommunicatedCP, earliestStoppingPathIndex)

        if earliestStoppingPathIndex != -1:
            truncatedPath = list(te.getTrajectory().getPoseSteering()[: earliestStoppingPathIndex + 1])
            overallPath = list(truncatedPath) + list(reversed(truncatedPath[:-1]))
            self.replacePath(robotID, overallPath, len(truncatedPath) - 1, {robotID})
            log.info("reversed_envelope", robotID=robotID, earliestStoppingPathIndex=earliestStoppingPathIndex)
        return True

    # -------------------------------------------------------------- ordering

    def getOrder(
        self,
        robotTracker1: "AbstractTrajectoryEnvelopeTracker",
        robotReport1: RobotReport,
        robotTracker2: "AbstractTrajectoryEnvelopeTracker",
        robotReport2: RobotReport,
        cs: CriticalSection,
    ) -> bool:
        te1, te2 = cs.getTe1(), cs.getTe2()
        assert te1 is not None and te2 is not None

        if self.yieldIfParking:
            robot1ParksInCS = cs.getTe1End() == te1.getPathLength() - 1
            robot2ParksInCS = cs.getTe2End() == te2.getPathLength() - 1
            if robot1ParksInCS and not robot2ParksInCS:
                return False
            if not robot1ParksInCS and robot2ParksInCS:
                return True

        r1atcs = RobotAtCriticalSection(robotReport1, cs)
        r2atcs = RobotAtCriticalSection(robotReport2, cs)
        if self.comparators:
            ret = self.comparators[0](r1atcs, r2atcs) < 0
            for comparator in self.comparators[1:]:
                result = comparator(r1atcs, r2atcs)
                if result != 0:
                    ret = result < 0
                    break
        else:
            ret = (cs.getTe2Start() - robotReport2.getPathIndex()) > (cs.getTe1Start() - robotReport1.getPathIndex())

        if ret and robotReport2.getRobotID() in self.muted:
            return False
        if not ret and robotReport1.getRobotID() in self.muted:
            return True
        return ret

    # ------------------------------------------------------------ dependencies

    def updateDependencies(self) -> None:
        if self.fake:
            for robotID in self.trackers:
                self.setCriticalPoint(robotID, -1, True)
            return
        if self.avoidDeadlockGlobally:
            self.globalCheckAndRevise()
        else:
            self.localCheckAndRevise()

    def _makeStoppingPointDeps(self) -> tuple[dict[int, RobotReport], dict[int, set[Dependency]]]:
        currentReports: dict[int, RobotReport] = {}
        currentDeps: dict[int, set[Dependency]] = {}
        for robotID, robotTracker in self.trackers.items():
            robotReport = robotTracker.getRobotReport()
            currentReports[robotID] = robotReport
            if robotID in self.stoppingPoints:
                for i, stoppingPoint in enumerate(self.stoppingPoints[robotID]):
                    duration = self.stoppingTimes[robotID][i]
                    if robotReport.getPathIndex() <= stoppingPoint:
                        dep = Dependency(robotTracker.getTrajectoryEnvelope(), None, stoppingPoint, 0)
                        currentDeps.setdefault(robotID, set()).add(dep)
                    if (
                        abs(robotReport.getPathIndex() - stoppingPoint) <= 1
                        and robotReport.getCriticalPoint() <= stoppingPoint
                        and robotID not in self.stoppingPointTimers
                    ):
                        self.spawnWaitingThread(robotID, i, duration)
            if robotID in self.replanningStoppingPoints:
                currentDeps.setdefault(robotID, set()).add(self.replanningStoppingPoints[robotID])
        return currentReports, currentDeps

    def localCheckAndRevise(self) -> None:
        currentReports, currentDeps = self._makeStoppingPointDeps()
        artificialDependencies: dict[int, set[Dependency]] = {}
        currentReversibleDependencies: set[Dependency] = set()

        robotIDs = set(self.trackers.keys())
        earliestStoppingPoints: dict[int, int] = {}
        if self.allCriticalSections:
            for robotID in robotIDs:
                earliestStoppingPoints[robotID] = self.getForwardModel(robotID).getEarliestStoppingPathIndex(
                    self.trackers[robotID].getTrajectoryEnvelope(), currentReports[robotID]
                )

        self.depsToCS.clear()
        self.isBlocked = False

        toRemove: set[CriticalSection] = set()
        for cs in self.allCriticalSections:
            te1, te2 = cs.getTe1(), cs.getTe2()
            assert te1 is not None and te2 is not None
            robotTracker1 = self.trackers[te1.getRobotID()]
            robotReport1 = currentReports[te1.getRobotID()]
            robotTracker2 = self.trackers[te2.getRobotID()]
            robotReport2 = currentReports[te2.getRobotID()]

            if robotReport1.getPathIndex() > cs.getTe1End() or robotReport2.getPathIndex() > cs.getTe2End():
                toRemove.add(cs)
                continue

            waitingPoint = -1
            drivingTracker = None
            waitingTracker = None
            drivingCurrentIndex = -1

            if isinstance(robotTracker1, TrajectoryEnvelopeTrackerDummy) or isinstance(robotTracker2, TrajectoryEnvelopeTrackerDummy):
                self.isBlocked = True
                createAParkingDep = False

                if isinstance(robotTracker1, TrajectoryEnvelopeTrackerDummy):
                    drivingCurrentIndex = robotReport1.getPathIndex()
                    drivingTracker, waitingTracker = robotTracker1, robotTracker2
                    waitingPoint = self.getCriticalPoint(robotReport2.getRobotID(), cs, drivingCurrentIndex)
                    comm = self.communicatedCPs.get(waitingTracker)
                    if (comm is None and robotReport1.getPathIndex() <= waitingPoint) or (
                        comm is not None and comm[0] != -1 and comm[0] <= waitingPoint
                    ):
                        createAParkingDep = True
                elif isinstance(robotTracker2, TrajectoryEnvelopeTrackerDummy):
                    drivingCurrentIndex = robotReport2.getPathIndex()
                    drivingTracker, waitingTracker = robotTracker2, robotTracker1
                    waitingPoint = self.getCriticalPoint(robotReport1.getRobotID(), cs, drivingCurrentIndex)
                    comm = self.communicatedCPs.get(waitingTracker)
                    if (comm is None and robotReport2.getPathIndex() <= waitingPoint) or (
                        comm is not None and comm[0] != -1 and comm[0] <= waitingPoint
                    ):
                        createAParkingDep = True

                if createAParkingDep:
                    assert drivingTracker is not None and waitingTracker is not None
                    drivingCSEnd = cs.getTe1End() if drivingTracker.getTrajectoryEnvelope().getRobotID() == te1.getRobotID() else cs.getTe2End()
                    dep = Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd)
                    currentDeps.setdefault(waitingTracker.getTrajectoryEnvelope().getRobotID(), set()).add(dep)
                    self.CSToDepsOrder[cs] = (dep.getWaitingRobotID(), dep.getWaitingPoint())
                    self.depsToCS[dep] = cs
                continue

            canStopRobot1 = self._canStop(robotTracker1, robotReport1, cs.getTe1Start(), earliestStoppingPoints)
            canStopRobot2 = self._canStop(robotTracker2, robotReport2, cs.getTe2Start(), earliestStoppingPoints)

            wakeUpinCSRobot1 = False
            wakeUpinCSRobot2 = False

            if canStopRobot1 and canStopRobot2:
                if self.getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs):
                    drivingCurrentIndex, drivingTracker, waitingTracker = robotReport1.getPathIndex(), robotTracker1, robotTracker2
                else:
                    drivingCurrentIndex, drivingTracker, waitingTracker = robotReport2.getPathIndex(), robotTracker2, robotTracker1
            elif canStopRobot1 and not canStopRobot2:
                drivingCurrentIndex, drivingTracker, waitingTracker = robotReport2.getPathIndex(), robotTracker2, robotTracker1
            elif not canStopRobot1 and canStopRobot2:
                drivingCurrentIndex, drivingTracker, waitingTracker = robotReport1.getPathIndex(), robotTracker1, robotTracker2
            else:
                drivingRobotID = -1
                waitingRobotID = -1
                wakeUpinCSRobot1 = robotTracker1 not in self.communicatedCPs and max(robotReport1.getPathIndex(), 0) >= cs.getTe1Start()
                wakeUpinCSRobot2 = robotTracker2 not in self.communicatedCPs and max(robotReport2.getPathIndex(), 0) >= cs.getTe2Start()

                if wakeUpinCSRobot1 or wakeUpinCSRobot2:
                    drivingRobotID = robotReport1.getRobotID() if wakeUpinCSRobot1 else robotReport2.getRobotID()
                    waitingRobotID = robotReport2.getRobotID() if wakeUpinCSRobot1 else robotReport1.getRobotID()
                elif cs in self.CSToDepsOrder and self.CSToDepsOrder[cs] is not None:
                    waitingRobotID = self.CSToDepsOrder[cs][0]
                    drivingRobotID = robotReport2.getRobotID() if waitingRobotID == robotReport1.getRobotID() else robotReport1.getRobotID()
                else:
                    log.error("lost_dependency_order", te1=te1.getID(), te2=te2.getID())
                    ahead = 0
                    c1 = self.communicatedCPs.get(robotTracker1)
                    c2 = self.communicatedCPs.get(robotTracker2)
                    if c1 is not None and (c1[0] == -1 or c1[0] > cs.getTe1End()):
                        ahead = 1
                    elif c2 is not None and (c2[0] == -1 or c2[0] > cs.getTe2End()):
                        ahead = -1
                    if ahead == 0:
                        ahead = self.isAhead(cs, robotReport1, robotReport2)
                    if ahead == 0:
                        if cs not in self.CSToDepsOrder:
                            raise RuntimeError(
                                f"FIXME! Lost dependency and order cannot be restored! Key value not found. "
                                f"RobotReport1: {robotReport1}, RobotReport2: {robotReport2}, cs: {cs}"
                            )
                        if self.CSToDepsOrder[cs] is None:
                            raise RuntimeError("FIXME! Lost dependency and order cannot be restored! Empty value.")
                    else:
                        drivingRobotID = robotReport1.getRobotID() if ahead == 1 else robotReport2.getRobotID()
                        waitingRobotID = robotReport2.getRobotID() if drivingRobotID == robotReport1.getRobotID() else robotReport1.getRobotID()
                        log.info("restored_lost_order_via_estimation", drivingRobotID=drivingRobotID)

                drivingCurrentIndex = robotReport1.getPathIndex() if drivingRobotID == robotReport1.getRobotID() else robotReport2.getPathIndex()
                drivingTracker = robotTracker1 if drivingRobotID == robotReport1.getRobotID() else robotTracker2
                waitingTracker = robotTracker1 if waitingRobotID == robotReport1.getRobotID() else robotTracker2

                waitingCurrentIndex = (
                    self.communicatedCPs[waitingTracker][0]
                    if waitingTracker in self.communicatedCPs
                    else (robotReport1.getPathIndex() if waitingRobotID == robotReport1.getRobotID() else robotReport2.getPathIndex())
                )
                lastIndexOfCSDriving = cs.getTe1End() if drivingRobotID == te1.getRobotID() else cs.getTe2End()
                if not self.canExitCriticalSection(
                    drivingCurrentIndex, waitingCurrentIndex, drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), lastIndexOfCSDriving
                ):
                    artWaitingPoint = (
                        self.communicatedCPs[drivingTracker][0]
                        if drivingTracker in self.communicatedCPs
                        else (robotReport1.getPathIndex() if drivingRobotID == robotReport1.getRobotID() else robotReport2.getPathIndex())
                    )
                    artDrivingCSEnd = cs.getTe1End() if waitingRobotID == te1.getRobotID() else cs.getTe2End()
                    dep = Dependency(
                        drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), max(0, artWaitingPoint), artDrivingCSEnd
                    )
                    artificialDependencies.setdefault(drivingRobotID, set()).add(dep)
                    log.info("cannot_escape_cs_creating_artificial_dependency", drivingRobotID=drivingRobotID, waitingPoint=dep.getWaitingPoint())

            assert drivingTracker is not None and waitingTracker is not None
            waitingPoint = self.getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex)

            if wakeUpinCSRobot1 and robotTracker2 in self.communicatedCPs:
                if self.communicatedCPs[robotTracker2][0] > waitingPoint:
                    waitingPoint = self.communicatedCPs[robotTracker2][0]
                    self.escapingCSToWaitingRobotIDandCP[cs] = (robotReport2.getRobotID(), waitingPoint)
            elif wakeUpinCSRobot2 and robotTracker1 in self.communicatedCPs:
                if self.communicatedCPs[robotTracker1][0] > waitingPoint:
                    waitingPoint = self.communicatedCPs[robotTracker1][0]
                    self.escapingCSToWaitingRobotIDandCP[cs] = (robotReport1.getRobotID(), waitingPoint)

            escaping = self.escapingCSToWaitingRobotIDandCP.get(cs)
            if escaping is not None and escaping[0] == waitingTracker.getTrajectoryEnvelope().getRobotID():
                waitingPoint = escaping[1]

            if waitingPoint >= 0:
                drivingCSEnd = cs.getTe1End() if drivingTracker.getTrajectoryEnvelope().getRobotID() == te1.getRobotID() else cs.getTe2End()
                dep = Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd)
                currentDeps.setdefault(waitingTracker.getTrajectoryEnvelope().getRobotID(), set()).add(dep)
                self.CSToDepsOrder[cs] = (dep.getWaitingRobotID(), dep.getWaitingPoint())
                self.depsToCS[dep] = cs
                if canStopRobot1 and canStopRobot2:
                    currentReversibleDependencies.add(dep)
            else:
                raise RuntimeError(f"Waiting point < 0 for critical section {cs}")

        for cs in toRemove:
            self.allCriticalSections.discard(cs)
            self.escapingCSToWaitingRobotIDandCP.pop(cs, None)
        self.criticalSectionCounter += len(toRemove)

        closestDeps = self.computeClosestDependencies(currentDeps, artificialDependencies)
        self.currentDependencies.clear()
        self.currentDependencies.update(closestDeps)

        currentDeps = self.findAndRepairNonliveCycles(currentDeps, artificialDependencies, currentReversibleDependencies, currentReports)

        for robotID in robotIDs:
            self.sendCriticalPoint(robotID, currentReports)

        self.isDeadlocked()

    def _canStop(
        self,
        tracker: "AbstractTrajectoryEnvelopeTracker",
        report: RobotReport,
        csStart: int,
        earliestStoppingPoints: dict[int, int],
    ) -> bool:
        comm = self.communicatedCPs.get(tracker)
        if (comm is not None and comm[0] != -1 and comm[0] < csStart) or (comm is None and max(0, report.getPathIndex()) < csStart):
            return True
        return earliestStoppingPoints[report.getRobotID()] < csStart

    def sendCriticalPoint(self, robotID: int, currentReports: dict[int, RobotReport]) -> None:
        tracker = self.trackers[robotID]
        maxDelay = 2 * (self.MAX_TX_DELAY + self.CONTROL_PERIOD + tracker.getTrackingPeriodInMillis()) + self.CONTROL_PERIOD
        retransmitt = self.forceCriticalPointReTransmission.get(robotID, False)
        if robotID in self.currentDependencies:
            dep = self.currentDependencies[robotID]
            comm = self.communicatedCPs.get(tracker)
            retransmitt = retransmitt or (
                comm is not None
                and comm[0] == dep.getWaitingPoint()
                and currentReports[robotID].getCriticalPoint() != dep.getWaitingPoint()
                and (self.getCurrentTimeInMillis() - comm[1]) > maxDelay
            )
            self.setCriticalPoint(dep.getWaitingRobotID(), dep.getWaitingPoint(), retransmitt)
        else:
            comm = self.communicatedCPs.get(tracker)
            retransmitt = retransmitt or (
                comm is not None
                and comm[0] == -1
                and currentReports[robotID].getCriticalPoint() != -1
                and (self.getCurrentTimeInMillis() - comm[1]) > maxDelay
            )
            self.setCriticalPoint(robotID, -1, retransmitt)
        self.forceCriticalPointReTransmission[robotID] = False

    # -------------------------------------------------------- cleanup override

    def cleanUpRobotCS(self, robotID: int, lastWaitingPoint: int) -> None:
        toRemove: set[CriticalSection] = set()
        for cs in self.CSToDepsOrder:
            if cs.getTe1().getRobotID() == robotID or cs.getTe2().getRobotID() == robotID:
                toRemove.add(cs)
        for cs in self.allCriticalSections:
            if (cs.getTe1().getRobotID() == robotID or cs.getTe2().getRobotID() == robotID) and cs not in toRemove:
                toRemove.add(cs)
                if (
                    cs.getTe1().getRobotID() == robotID
                    and (cs.getTe1Start() <= lastWaitingPoint or lastWaitingPoint == -1)
                    or cs.getTe2().getRobotID() == robotID
                    and (cs.getTe2Start() <= lastWaitingPoint or lastWaitingPoint == -1)
                ):
                    self.criticalSectionCounter += 1

        for cs in toRemove:
            if self.avoidDeadlockGlobally and cs in self.CSToDepsOrder:
                waitingRobID = self.CSToDepsOrder[cs][0]
                drivingRobID = cs.getTe2().getRobotID() if cs.getTe1().getRobotID() == waitingRobID else cs.getTe1().getRobotID()
                self.deleteEdge((waitingRobID, drivingRobID))
            self.CSToDepsOrder.pop(cs, None)
            self.allCriticalSections.discard(cs)
            self.escapingCSToWaitingRobotIDandCP.pop(cs, None)

    # ------------------------------------------------------------- graph ops

    def deleteEdge(self, edge: tuple[int, int]) -> None:
        self.deleteEdges({edge: 1})

    def deleteEdges(self, edgesToDelete: dict[tuple[int, int], int] | None) -> None:
        if not edgesToDelete:
            return
        for edge, occurrence in edgesToDelete.items():
            if not occurrence:
                continue
            if self.currentOrdersGraph.has_edge(*edge):
                numEdge = self.currentOrdersGraph[edge[0]][edge[1]]["weight"]
                if numEdge > occurrence:
                    self.currentOrdersGraph[edge[0]][edge[1]]["weight"] = numEdge - occurrence
                else:
                    self.currentOrdersGraph.remove_edge(*edge)
                    if edge in self.currentCyclesList:
                        toRemove: dict[tuple[int, int], set[tuple[int, ...]]] = {}
                        for cycle in self.currentCyclesList[edge]:
                            for i in range(len(cycle)):
                                j = i + 1 if i < len(cycle) - 1 else 0
                                key = (cycle[i], cycle[j])
                                toRemove.setdefault(key, set()).add(cycle)
                        for key, cycles in toRemove.items():
                            if key in self.currentCyclesList:
                                self.currentCyclesList[key] -= cycles
                                if not self.currentCyclesList[key]:
                                    del self.currentCyclesList[key]

    def addEdges(self, edgesToAdd: dict[tuple[int, int], int] | None) -> None:
        if not edgesToAdd:
            return
        toAdd: set[tuple[int, int]] = set()
        for edge, occurrence in edgesToAdd.items():
            if not occurrence:
                continue
            if not self.currentOrdersGraph.has_edge(*edge):
                toAdd.add(edge)
                self.currentOrdersGraph.add_edge(edge[0], edge[1], weight=occurrence)
            else:
                self.currentOrdersGraph[edge[0]][edge[1]]["weight"] += occurrence
        if not toAdd:
            return

        sccs = [
            self.currentOrdersGraph.subgraph(nodes).copy()
            for nodes in nx.strongly_connected_components(self.currentOrdersGraph)
        ]
        for edge in toAdd:
            for scc in sccs:
                if edge[0] in scc or edge[1] in scc:
                    if edge[0] in scc and edge[1] in scc:
                        for cycle in nx.simple_cycles(scc):
                            if len(cycle) < 2:
                                continue
                            for i in range(len(cycle)):
                                j = i + 1 if i < len(cycle) - 1 else 0
                                key = (cycle[i], cycle[j])
                                self.currentCyclesList.setdefault(key, set()).add(tuple(cycle))
                    break

    def updateGraph(
        self, edgesToDelete: dict[tuple[int, int], int] | None, edgesToAdd: dict[tuple[int, int], int] | None
    ) -> None:
        edgesToDelete = edgesToDelete or {}
        edgesToAdd = edgesToAdd or {}
        toDelete: dict[tuple[int, int], int] = {}
        for edge, count_ in edgesToDelete.items():
            if edge in edgesToAdd and edgesToAdd[edge] < count_:
                toDelete[edge] = count_ - edgesToAdd[edge]
            else:
                toDelete[edge] = count_
        toAdd: dict[tuple[int, int], int] = {}
        for edge, count_ in edgesToAdd.items():
            if edge in edgesToDelete and edgesToDelete[edge] < count_:
                toAdd[edge] = count_ - edgesToDelete[edge]
            else:
                toAdd[edge] = count_
        self.deleteEdges(toDelete)
        self.addEdges(toAdd)

    # ------------------------------------------------------------------ global

    def globalCheckAndRevise(self) -> None:
        currentReports, currentDeps = self._makeStoppingPointDeps()
        artificialDependencies: dict[int, set[Dependency]] = {}
        depsGraph: nx.MultiDiGraph = nx.MultiDiGraph()
        askForReplan: set[int] = set()
        earliestStoppingPoints: dict[int, int] = {}
        edgesToDelete: dict[tuple[int, int], int] = {}
        edgesToAdd: dict[tuple[int, int], int] = {}
        reversibleCS: set[CriticalSection] = set()

        robotIDs = set(self.trackers.keys())
        if self.allCriticalSections:
            for robotID in robotIDs:
                earliestStoppingPoints[robotID] = self.getForwardModel(robotID).getEarliestStoppingPathIndex(
                    self.trackers[robotID].getTrajectoryEnvelope(), currentReports[robotID]
                )

        self.depsToCS.clear()
        self.isBlocked = False

        toRemove: set[CriticalSection] = set()
        for cs in self.allCriticalSections:
            te1, te2 = cs.getTe1(), cs.getTe2()
            assert te1 is not None and te2 is not None
            robotTracker1 = self.trackers[te1.getRobotID()]
            robotReport1 = currentReports[te1.getRobotID()]
            robotTracker2 = self.trackers[te2.getRobotID()]
            robotReport2 = currentReports[te2.getRobotID()]

            if robotReport1.getPathIndex() > cs.getTe1End() or robotReport2.getPathIndex() > cs.getTe2End():
                toRemove.add(cs)
                continue

            waitingPoint = -1
            drivingCurrentIndex = -1
            drivingTracker = None
            waitingTracker = None

            if isinstance(robotTracker1, TrajectoryEnvelopeTrackerDummy) or isinstance(robotTracker2, TrajectoryEnvelopeTrackerDummy):
                self.isBlocked = True
                createAParkingDep = False
                if isinstance(robotTracker1, TrajectoryEnvelopeTrackerDummy):
                    drivingCurrentIndex = robotReport1.getPathIndex()
                    drivingTracker, waitingTracker = robotTracker1, robotTracker2
                    waitingPoint = self.getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex)
                    comm = self.communicatedCPs.get(waitingTracker)
                    if (comm is None and robotReport1.getPathIndex() <= waitingPoint) or (
                        comm is not None and comm[0] != -1 and comm[0] <= waitingPoint
                    ):
                        createAParkingDep = True
                elif isinstance(robotTracker2, TrajectoryEnvelopeTrackerDummy):
                    drivingCurrentIndex = robotReport2.getPathIndex()
                    drivingTracker, waitingTracker = robotTracker2, robotTracker1
                    waitingPoint = self.getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex)
                    comm = self.communicatedCPs.get(waitingTracker)
                    if (comm is None and robotReport2.getPathIndex() <= waitingPoint) or (
                        comm is not None and comm[0] != -1 and comm[0] <= waitingPoint
                    ):
                        createAParkingDep = True

                if createAParkingDep:
                    assert drivingTracker is not None and waitingTracker is not None
                    drivingCSEnd = cs.getTe1End() if drivingTracker.getTrajectoryEnvelope().getRobotID() == te1.getRobotID() else cs.getTe2End()
                    dep = Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd)
                    currentDeps.setdefault(waitingTracker.getTrajectoryEnvelope().getRobotID(), set()).add(dep)
                    if cs not in self.CSToDepsOrder:
                        edge = (dep.getWaitingRobotID(), dep.getDrivingRobotID())
                        edgesToAdd[edge] = edgesToAdd.get(edge, 0) + 1
                    depsGraph.add_edge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), key=id(dep), dep=dep)
                    self.CSToDepsOrder[cs] = (dep.getWaitingRobotID(), dep.getWaitingPoint())
                    self.depsToCS[dep] = cs
                continue

            canStopRobot1 = self._canStop(robotTracker1, robotReport1, cs.getTe1Start(), earliestStoppingPoints)
            canStopRobot2 = self._canStop(robotTracker2, robotReport2, cs.getTe2Start(), earliestStoppingPoints)
            wakeUpinCSRobot1 = False
            wakeUpinCSRobot2 = False

            if canStopRobot1 and canStopRobot2:
                reversibleCS.add(cs)
                if cs not in self.CSToDepsOrder:
                    robot2Yields = robotTracker1.getStartingTimeInMillis() < robotTracker2.getStartingTimeInMillis() or (
                        robotTracker1.getStartingTimeInMillis() == robotTracker2.getStartingTimeInMillis()
                        and robotReport1.getRobotID() < robotReport2.getRobotID()
                    )
                else:
                    robot2Yields = self.CSToDepsOrder[cs][0] == robotReport2.getRobotID()
                if robot2Yields:
                    drivingCurrentIndex, drivingTracker, waitingTracker = robotReport1.getPathIndex(), robotTracker1, robotTracker2
                else:
                    drivingCurrentIndex, drivingTracker, waitingTracker = robotReport2.getPathIndex(), robotTracker2, robotTracker1
            elif canStopRobot1 and not canStopRobot2:
                drivingCurrentIndex, drivingTracker, waitingTracker = robotReport2.getPathIndex(), robotTracker2, robotTracker1
            elif not canStopRobot1 and canStopRobot2:
                drivingCurrentIndex, drivingTracker, waitingTracker = robotReport1.getPathIndex(), robotTracker1, robotTracker2
            else:
                drivingRobotID = -1
                waitingRobotID = -1
                wakeUpinCSRobot1 = robotTracker1 not in self.communicatedCPs and max(robotReport1.getPathIndex(), 0) >= cs.getTe1Start()
                wakeUpinCSRobot2 = robotTracker2 not in self.communicatedCPs and max(robotReport2.getPathIndex(), 0) >= cs.getTe2Start()
                if wakeUpinCSRobot1 or wakeUpinCSRobot2:
                    drivingRobotID = robotReport1.getRobotID() if wakeUpinCSRobot1 else robotReport2.getRobotID()
                    waitingRobotID = robotReport2.getRobotID() if wakeUpinCSRobot1 else robotReport1.getRobotID()
                elif cs in self.CSToDepsOrder and self.CSToDepsOrder[cs] is not None:
                    waitingRobotID = self.CSToDepsOrder[cs][0]
                    drivingRobotID = robotReport2.getRobotID() if waitingRobotID == robotReport1.getRobotID() else robotReport1.getRobotID()
                else:
                    log.error("lost_dependency_order_global", te1=te1.getID(), te2=te2.getID())
                    ahead = 0
                    c1 = self.communicatedCPs.get(robotTracker1)
                    c2 = self.communicatedCPs.get(robotTracker2)
                    if c1 is not None and (c1[0] == -1 or c1[0] > cs.getTe1End()):
                        ahead = 1
                    elif c2 is not None and (c2[0] == -1 or c2[0] > cs.getTe2End()):
                        ahead = -1
                    if ahead == 0:
                        ahead = self.isAhead(cs, robotReport1, robotReport2)
                    if ahead == 0:
                        if cs not in self.CSToDepsOrder:
                            raise RuntimeError(
                                f"FIXME! Lost dependency and order cannot be restored! Key value not found. "
                                f"RobotReport1: {robotReport1}, RobotReport2: {robotReport2}, cs: {cs}"
                            )
                        if self.CSToDepsOrder[cs] is None:
                            raise RuntimeError("FIXME! Lost dependency and order cannot be restored! Empty value.")
                    else:
                        drivingRobotID = robotReport1.getRobotID() if ahead == 1 else robotReport2.getRobotID()
                        waitingRobotID = robotReport2.getRobotID() if drivingRobotID == robotReport1.getRobotID() else robotReport1.getRobotID()
                        log.info("restored_lost_order_via_estimation_global", drivingRobotID=drivingRobotID)

                drivingCurrentIndex = robotReport1.getPathIndex() if drivingRobotID == robotReport1.getRobotID() else robotReport2.getPathIndex()
                waitingTracker = robotTracker1 if waitingRobotID == robotReport1.getRobotID() else robotTracker2
                drivingTracker = robotTracker1 if drivingRobotID == robotReport1.getRobotID() else robotTracker2

                waitingCurrentIndex = (
                    self.communicatedCPs[waitingTracker][0]
                    if waitingTracker in self.communicatedCPs
                    else (robotReport1.getPathIndex() if waitingRobotID == robotReport1.getRobotID() else robotReport2.getPathIndex())
                )
                lastIndexOfCSDriving = cs.getTe1End() if drivingRobotID == te1.getRobotID() else cs.getTe2End()
                if not self.canExitCriticalSection(
                    drivingCurrentIndex, waitingCurrentIndex, drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), lastIndexOfCSDriving
                ):
                    artWaitingPoint = (
                        self.communicatedCPs[drivingTracker][0]
                        if drivingTracker in self.communicatedCPs
                        else (robotReport1.getPathIndex() if drivingRobotID == robotReport1.getRobotID() else robotReport2.getPathIndex())
                    )
                    artDrivingCSEnd = cs.getTe1End() if waitingRobotID == te1.getRobotID() else cs.getTe2End()
                    dep = Dependency(
                        drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), max(0, artWaitingPoint), artDrivingCSEnd
                    )
                    artificialDependencies.setdefault(drivingRobotID, set()).add(dep)
                    askForReplan.add(drivingRobotID)
                    depsGraph.add_edge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), key=id(dep), dep=dep)

            if cs in reversibleCS and canStopRobot1 and canStopRobot2:
                # precedence pre-loaded above; still need waitingPoint + bookkeeping below
                pass

            if drivingTracker is None or waitingTracker is None:
                continue

            waitingPoint = self.getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex)

            if wakeUpinCSRobot1 and robotTracker2 in self.communicatedCPs and self.communicatedCPs[robotTracker2][0] > waitingPoint:
                waitingPoint = self.communicatedCPs[robotTracker2][0]
                self.escapingCSToWaitingRobotIDandCP[cs] = (robotReport2.getRobotID(), waitingPoint)
            elif wakeUpinCSRobot2 and robotTracker1 in self.communicatedCPs and self.communicatedCPs[robotTracker1][0] > waitingPoint:
                waitingPoint = self.communicatedCPs[robotTracker1][0]
                self.escapingCSToWaitingRobotIDandCP[cs] = (robotReport1.getRobotID(), waitingPoint)

            escaping = self.escapingCSToWaitingRobotIDandCP.get(cs)
            if escaping is not None and escaping[0] == waitingTracker.getTrajectoryEnvelope().getRobotID():
                waitingPoint = escaping[1]

            if waitingPoint < 0:
                raise RuntimeError(f"Waiting point < 0 for critical section {cs}")

            drivingCSEnd = cs.getTe1End() if drivingTracker.getTrajectoryEnvelope().getRobotID() == te1.getRobotID() else cs.getTe2End()
            dep = Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd)
            currentDeps.setdefault(waitingTracker.getTrajectoryEnvelope().getRobotID(), set()).add(dep)
            if cs not in self.CSToDepsOrder:
                edge = (dep.getWaitingRobotID(), dep.getDrivingRobotID())
                edgesToAdd[edge] = edgesToAdd.get(edge, 0) + 1
            depsGraph.add_edge(dep.getWaitingRobotID(), dep.getDrivingRobotID(), key=id(dep), dep=dep)
            self.CSToDepsOrder[cs] = (dep.getWaitingRobotID(), dep.getWaitingPoint())
            self.depsToCS[dep] = cs

        for cs in toRemove:
            self.allCriticalSections.discard(cs)
            if cs in self.CSToDepsOrder:
                waitingRobID = self.CSToDepsOrder[cs][0]
                drivingRobID = cs.getTe2().getRobotID() if cs.getTe1().getRobotID() == waitingRobID else cs.getTe1().getRobotID()
                edge = (waitingRobID, drivingRobID)
                edgesToDelete[edge] = edgesToDelete.get(edge, 0) + 1
            self.escapingCSToWaitingRobotIDandCP.pop(cs, None)
        self.criticalSectionCounter += len(toRemove)

        self.updateGraph(edgesToDelete, edgesToAdd)

        self._reviseReversibleByHeuristic(reversibleCS, currentReports, currentDeps, depsGraph)

        closestDeps = self.computeClosestDependencies(currentDeps, artificialDependencies)
        self.currentDependencies.clear()
        self.currentDependencies.update(closestDeps)

        for robotID in askForReplan:
            self.replanEnvelope(robotID, True)

        for robotID in robotIDs:
            self.sendCriticalPoint(robotID, currentReports)

        self.isDeadlocked()

    def _reviseReversibleByHeuristic(
        self,
        reversibleCS: set[CriticalSection],
        currentReports: dict[int, RobotReport],
        currentDeps: dict[int, set[Dependency]],
        depsGraph: nx.MultiDiGraph,
    ) -> None:
        for cs in reversibleCS:
            edgesToDelete: dict[tuple[int, int], int] = {}
            edgesToAdd: dict[tuple[int, int], int] = {}

            te1, te2 = cs.getTe1(), cs.getTe2()
            assert te1 is not None and te2 is not None
            robotTracker1 = self.trackers[te1.getRobotID()]
            robotReport1 = currentReports[te1.getRobotID()]
            robotTracker2 = self.trackers[te2.getRobotID()]
            robotReport2 = currentReports[te2.getRobotID()]

            robot2Yields = self.getOrder(robotTracker1, robotReport1, robotTracker2, robotReport2, cs)
            order = self.CSToDepsOrder.get(cs)
            if order is None:
                continue
            robot2YieldsOld = order[0] == robotReport2.getRobotID()

            if robot2YieldsOld == robot2Yields:
                continue

            drivingCurrentIndex = robotReport1.getPathIndex() if robot2Yields else robotReport2.getPathIndex()
            drivingTracker = robotTracker1 if robot2Yields else robotTracker2
            waitingTracker = robotTracker2 if robot2Yields else robotTracker1
            waitingPoint = self.getCriticalPoint(waitingTracker.getTrajectoryEnvelope().getRobotID(), cs, drivingCurrentIndex)
            if waitingPoint < 0:
                continue
            # same guard as in the nonlive-cycle repair: never reverse onto a
            # robot that can no longer stop at the new waiting point
            newWaitingRobotID = waitingTracker.getTrajectoryEnvelope().getRobotID()
            newWaitingReport = currentReports[newWaitingRobotID]
            earliest = {
                newWaitingRobotID: self.getForwardModel(newWaitingRobotID).getEarliestStoppingPathIndex(
                    waitingTracker.getTrajectoryEnvelope(), newWaitingReport
                )
            }
            if not self._canStop(waitingTracker, newWaitingReport, waitingPoint + 1, earliest):
                continue

            backupGraph = self.currentOrdersGraph.copy()
            backupCycles = {k: set(v) for k, v in self.currentCyclesList.items()}
            backupDepsGraph = depsGraph.copy()

            edgesToDelete[(drivingTracker.getTrajectoryEnvelope().getRobotID(), waitingTracker.getTrajectoryEnvelope().getRobotID())] = 1
            newEdge = (waitingTracker.getTrajectoryEnvelope().getRobotID(), drivingTracker.getTrajectoryEnvelope().getRobotID())
            edgesToAdd[newEdge] = 1
            self.updateGraph(edgesToDelete, edgesToAdd)

            drivingCSEnd = cs.getTe1End() if drivingTracker.getTrajectoryEnvelope().getRobotID() == te1.getRobotID() else cs.getTe2End()
            drivingCSEndOld = cs.getTe2End() if drivingTracker.getTrajectoryEnvelope().getRobotID() == te1.getRobotID() else cs.getTe1End()
            depNew = Dependency(waitingTracker.getTrajectoryEnvelope(), drivingTracker.getTrajectoryEnvelope(), waitingPoint, drivingCSEnd)
            depOld = Dependency(drivingTracker.getTrajectoryEnvelope(), waitingTracker.getTrajectoryEnvelope(), order[1], drivingCSEndOld)

            safe = True
            if newEdge in self.currentCyclesList:
                for cycle in self.currentCyclesList[newEdge]:
                    edges_along_cycle: list[list[Dependency]] = []
                    for i in range(len(cycle)):
                        j = i + 1 if i < len(cycle) - 1 else 0
                        if cycle[i] == depNew.getWaitingRobotID() and cycle[j] == depNew.getDrivingRobotID():
                            edges_along_cycle.append([depNew])
                        else:
                            all_edges = [
                                data["dep"]
                                for _, _, data in depsGraph.edges(data=True)
                                if data["dep"].getWaitingRobotID() == cycle[i] and data["dep"].getDrivingRobotID() == cycle[j]
                            ]
                            edges_along_cycle.append(all_edges)

                    if any(len(e) == 0 for e in edges_along_cycle):
                        continue

                    for combo in itertools.product(*edges_along_cycle):
                        for i in range(len(combo)):
                            j = i + 1 if i < len(combo) - 1 else 0
                            if self.nonlivePair(combo[i], combo[j]):
                                safe = False
                                break
                        if not safe:
                            break
                    if not safe:
                        break

            if not safe:
                self.currentOrdersGraph = backupGraph
                self.currentCyclesList = backupCycles
                depsGraph.clear()
                depsGraph.add_edges_from(backupDepsGraph.edges(data=True, keys=True))
                self.nonliveStatesAvoided += 1
            else:
                if depOld.getWaitingRobotID() in currentDeps:
                    currentDeps[depOld.getWaitingRobotID()].discard(depOld)
                    if not currentDeps[depOld.getWaitingRobotID()]:
                        del currentDeps[depOld.getWaitingRobotID()]
                currentDeps.setdefault(waitingTracker.getTrajectoryEnvelope().getRobotID(), set()).add(depNew)
                self.CSToDepsOrder[cs] = (depNew.getWaitingRobotID(), depNew.getWaitingPoint())
                self.depsToCS[depNew] = cs
                self.currentOrdersHeurusticallyDecided += 1
