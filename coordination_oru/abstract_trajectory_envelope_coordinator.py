"""``AbstractTrajectoryEnvelopeCoordinator``: coordination for a fleet of robots.

Ported from the Java class of the same name. Java's ``synchronized(solver)``
(which nearly every public method wraps its whole body in, and which nests
``synchronized(trackers)``/``synchronized(allCriticalSections)``/
``synchronized(currentDependencies)``/``synchronized(stoppingPoints)``
inside it) becomes a single ``asyncio.Lock`` acquired at the outer entry
points (the coordination tick, ``addMissions``, ``placeRobot``, and the
tracker-finished callback); asyncio is cooperative and none of the ported
logic below awaits mid-section, so one lock at the outer boundary gives the
same atomicity as Java's nested monitors without risking deadlock from
non-reentrant re-acquisition.

Java's per-robot ``Thread``s (the coordinator inference thread, tracker
threads, stopping-point waiting threads) become ``asyncio.Task``s.

Java's ground-envelope/sub-envelope temporal dispatch machinery (deadlines,
``getAllSubEnvelopes``) is dropped — missions here are always a single flat
envelope, so it has no observable effect; see
:mod:`coordination_oru.abstract_trajectory_envelope_tracker`.
"""

from __future__ import annotations

import abc
import asyncio
import math
from typing import TYPE_CHECKING, Callable, Sequence

from shapely.affinity import rotate, translate
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from coordination_oru import network_configuration
from coordination_oru.critical_section import CriticalSection
from coordination_oru.dependency import Dependency
from coordination_oru.forward_model import ForwardModel
from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
from coordination_oru.metacsp.spatial.trajectory_envelope_solver import (
    TrajectoryEnvelopeSolver,
)
from coordination_oru.mission import Mission
from coordination_oru.robot_report import RobotReport
from coordination_oru.tracking_callback import TrackingCallback
from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)
from coordination_oru.util.logging import get_logger

if TYPE_CHECKING:
    from coordination_oru.abstract_trajectory_envelope_tracker import (
        AbstractTrajectoryEnvelopeTracker,
    )
    from coordination_oru.motionplanning.abstract_motion_planner import (
        AbstractMotionPlanner,
    )
    from coordination_oru.robot_at_critical_section import RobotAtCriticalSection

log = get_logger(__name__)

PARKING_DURATION = 3000
DEFAULT_STOPPING_TIME = 5000
DEFAULT_ROBOT_TRACKING_PERIOD = 30

TRAILING_PATH_POINTS = 3


class AbstractTrajectoryEnvelopeCoordinator(abc.ABC):
    def __init__(self, CONTROL_PERIOD: int = 1000, TEMPORAL_RESOLUTION: float = 1000.0) -> None:
        self.CONTROL_PERIOD = CONTROL_PERIOD
        self.TEMPORAL_RESOLUTION = TEMPORAL_RESOLUTION
        self.DEFAULT_ROBOT_TRACKING_PERIOD = DEFAULT_ROBOT_TRACKING_PERIOD

        self.overlay = False
        self.quiet = False

        self.totalMsgsSent = 0
        self.totalMsgsReTx = 0
        self.criticalSectionCounter = 0

        self.solver: TrajectoryEnvelopeSolver | None = None
        self._stopInference = True
        self._inference_task: asyncio.Task[None] | None = None

        self.missionsPool: list[tuple[TrajectoryEnvelope, int]] = []
        self.envelopesToTrack: list[TrajectoryEnvelope] = []
        self.currentParkingEnvelopes: list[TrajectoryEnvelope] = []
        self.allCriticalSections: set[CriticalSection] = set()
        self.CSToDepsOrder: dict[CriticalSection, tuple[int, int]] = {}
        self.depsToCS: dict[Dependency, CriticalSection] = {}
        self.escapingCSToWaitingRobotIDandCP: dict[CriticalSection, tuple[int, int]] = {}
        self.stoppingPoints: dict[int, list[int]] = {}
        self.stoppingTimes: dict[int, list[int]] = {}
        self.stoppingPointTimers: dict[int, asyncio.Task[None]] = {}

        self.trackers: dict[int, "AbstractTrajectoryEnvelopeTracker"] = {}
        self.currentDependencies: dict[int, Dependency] = {}

        self.communicatedCPs: dict["AbstractTrajectoryEnvelopeTracker", tuple[int, int]] = {}
        self.externalCPCounters: dict["AbstractTrajectoryEnvelopeTracker", int] = {}

        self.comparators: list[Callable[["RobotAtCriticalSection", "RobotAtCriticalSection"], int]] = []
        self.forwardModels: dict[int, ForwardModel] = {}

        self.footprints: dict[int, Polygon] = {}
        self.maxFootprintDimensions: dict[int, float] = {}

        self.robotTrackingPeriodInMillis: dict[int, int] = {}
        self.robotMaxVelocity: dict[int, float] = {}
        self.robotMaxAcceleration: dict[int, float] = {}

        self.muted: set[int] = set()

        self.yieldIfParking = True
        self.checkEscapePoses = True

        self.trackingCallbacks: dict[int, TrackingCallback] = {}
        self.inferenceCallback: Callable[[], None] | None = None

        self.motionPlanners: dict[int, "AbstractMotionPlanner"] = {}

        self.packetLossProbability = network_configuration.PROBABILITY_OF_PACKET_LOSS
        self.MAX_TX_DELAY = network_configuration.getMaximumTxDelay()
        self.maxFaultsProbability = network_configuration.PROBABILITY_OF_PACKET_LOSS
        self.numberOfReplicas = 1

        self.isDriving: dict[int, bool] = {}

        self._lock = asyncio.Lock()

    # -------------------------------------------------------------- footprint

    def getMaxFootprintDimension(self, robotID: int) -> float | None:
        if robotID in self.footprints:
            return self.maxFootprintDimensions.get(robotID)
        return None

    def getFootprint(self, robotID: int) -> Polygon | None:
        return self.footprints.get(robotID)

    def getFootprintPolygon(self, robotID: int) -> BaseGeometry | None:
        fp = self.getFootprint(robotID)
        return fp

    def setFootprint(self, robotID: int, *coordinates: tuple[float, float]) -> None:
        footprint = Polygon(coordinates)
        self.footprints[robotID] = footprint
        self.maxFootprintDimensions[robotID] = self.computeMaxFootprintDimension(coordinates)

    def computeMaxFootprintDimension(self, coords: Sequence[tuple[float, float]]) -> float:
        xs = sorted(c[0] for c in coords)
        ys = sorted(c[1] for c in coords)
        return max(xs[-1] - xs[0], ys[-1] - ys[0])

    # ------------------------------------------------------------ forward model

    def setForwardModel(self, robotID: int, fm: ForwardModel) -> None:
        self.forwardModels[robotID] = fm

    def getForwardModel(self, robotID: int) -> ForwardModel:
        if robotID in self.forwardModels:
            return self.forwardModels[robotID]
        return _DefaultForwardModel()

    # ------------------------------------------------------------ robot config

    def setRobotTrackingPeriodInMillis(self, robotID: int, trackingPeriodInMillis: int) -> None:
        self.robotTrackingPeriodInMillis[robotID] = trackingPeriodInMillis

    def getRobotTrackingPeriodInMillis(self, robotID: int) -> int:
        if robotID in self.robotTrackingPeriodInMillis:
            return self.robotTrackingPeriodInMillis[robotID]
        return self.DEFAULT_ROBOT_TRACKING_PERIOD

    def setRobotMaxVelocity(self, robotID: int, maxVelocity: float) -> None:
        self.robotMaxVelocity[robotID] = maxVelocity

    def setRobotMaxAcceleration(self, robotID: int, maxAcceleration: float) -> None:
        self.robotMaxAcceleration[robotID] = maxAcceleration

    def getRobotMaxVelocity(self, robotID: int) -> float | None:
        return self.robotMaxVelocity.get(robotID)

    def getRobotMaxAcceleration(self, robotID: int) -> float | None:
        return self.robotMaxAcceleration.get(robotID)

    # ----------------------------------------------------------------- network

    def setNetworkParameters(
        self, packetLossProbability: float, max_tx_delay: int, maxFaultsProbability: float
    ) -> None:
        self.packetLossProbability = packetLossProbability
        network_configuration.setDelays(network_configuration.getMinimumTxDelay(), max_tx_delay)
        self.MAX_TX_DELAY = max_tx_delay
        self.maxFaultsProbability = maxFaultsProbability
        self.numberOfReplicas = (
            int(
                math.ceil(
                    math.log(1 - math.sqrt(1 - maxFaultsProbability)) / math.log(packetLossProbability)
                )
            )
            if packetLossProbability > 0 and maxFaultsProbability > 0
            else 1
        )

    def setInferenceCallback(self, cb: Callable[[], None]) -> None:
        self.inferenceCallback = cb

    def getControlPeriod(self) -> int:
        return self.CONTROL_PERIOD

    def getTemporalResolution(self) -> float:
        return self.TEMPORAL_RESOLUTION

    def setYieldIfParking(self, value: bool) -> None:
        self.yieldIfParking = value

    def setCheckEscapePoses(self, value: bool) -> None:
        self.checkEscapePoses = value

    # ----------------------------------------------------------------- muting

    def toggleMute(self, robotID: int) -> None:
        if robotID in self.muted:
            self.muted.remove(robotID)
        else:
            self.muted.add(robotID)

    def mute(self, robotID: int) -> None:
        self.muted.add(robotID)

    def unMute(self, robotID: int) -> None:
        self.muted.discard(robotID)

    def getMuted(self) -> list[int]:
        return list(self.muted)

    # --------------------------------------------------------------- lifecycle

    @abc.abstractmethod
    def getCurrentTimeInMillis(self) -> int: ...

    def setupSolver(self, max_envelopes: int = 64) -> None:
        self.solver = TrajectoryEnvelopeSolver(max_envelopes=max_envelopes)

    async def startInference(self) -> None:
        if self.solver is None:
            raise RuntimeError("Solver not initialized, please call setupSolver() first!")
        if not self._stopInference:
            return
        self._stopInference = False
        self._inference_task = asyncio.create_task(self._inference_loop(), name="coordinator-inference")

    async def stopInference(self) -> None:
        self._stopInference = True
        if self._inference_task is not None:
            self._inference_task.cancel()
            try:
                await self._inference_task
            except asyncio.CancelledError:
                pass
            self._inference_task = None

    def isStartedInference(self) -> bool:
        return not self._stopInference

    async def _inference_loop(self) -> None:
        MAX_ADDED_MISSIONS = 1
        thread_last_update = self.getCurrentTimeInMillis()
        try:
            while not self._stopInference:
                async with self._lock:
                    if self.missionsPool:
                        added = 0
                        while self.missionsPool and added < MAX_ADDED_MISSIONS:
                            te = self._pollMissionsPool()
                            self.envelopesToTrack.append(te)
                            self.onNewMissionDispatched(te.getRobotID())
                            added += 1
                        self.computeCriticalSections()
                        self.startTrackingAddedMissions()
                    self.updateDependencies()

                if self.CONTROL_PERIOD > 0:
                    elapsed = self.getCurrentTimeInMillis() - thread_last_update
                    await asyncio.sleep(max(0, self.CONTROL_PERIOD - elapsed) / 1000.0)
                thread_last_update = self.getCurrentTimeInMillis()
                if self.inferenceCallback is not None:
                    self.inferenceCallback()
        except asyncio.CancelledError:
            raise

    def _pollMissionsPool(self) -> TrajectoryEnvelope:
        # Java's missionsPool is a TreeSet with a comparator that inverts
        # timestamp order, so pollFirst() returns the *latest*-submitted
        # envelope (a LIFO quirk of the original) — ported verbatim.
        idx = max(range(len(self.missionsPool)), key=lambda i: self.missionsPool[i][1])
        te, _ts = self.missionsPool.pop(idx)
        return te

    def onNewMissionDispatched(self, robotID: int) -> None:
        pass

    def onCriticalSectionUpdate(self) -> None:
        pass

    # ------------------------------------------------------------- robot state

    def getDrivingEnvelopes(self) -> list[TrajectoryEnvelope]:
        return [
            t.getTrajectoryEnvelope()
            for t in self.trackers.values()
            if not isinstance(t, TrajectoryEnvelopeTrackerDummy)
        ]

    def isParked(self, robotID: int) -> bool:
        return robotID in self.isDriving and not self.isDriving[robotID]

    def isDrivingRobot(self, robotID: int) -> bool:
        return self.isDriving.get(robotID, False)

    def getIdleRobots(self) -> list[int]:
        if self.solver is None:
            raise RuntimeError("Solver not initialized, please call setupSolver() first!")
        return [robotID for robotID in self.trackers if self.isFree(robotID)]

    def getAllRobotIDs(self) -> set[int]:
        return set(self.trackers.keys())

    def getRobotReport(self, robotID: int) -> RobotReport | None:
        if robotID not in self.trackers:
            return None
        return self.trackers[robotID].getLastRobotReport()

    def getCurrentDependencies(self) -> dict[int, Dependency]:
        return self.currentDependencies

    def getCurrentSuperEnvelope(self, robotID: int) -> TrajectoryEnvelope:
        return self.trackers[robotID].getTrajectoryEnvelope()

    def getCurrentTrajectoryEnvelope(self, robotID: int) -> TrajectoryEnvelope:
        return self.trackers[robotID].getTrajectoryEnvelope()

    def addTrackingCallback(self, robotID: int, cb: TrackingCallback) -> None:
        self.trackingCallbacks[robotID] = cb

    def setVisualization(self, viz: object) -> None:
        self.viz = viz

    def addComparator(self, c: Callable[["RobotAtCriticalSection", "RobotAtCriticalSection"], int]) -> None:
        self.comparators.append(c)

    def setMotionPlanner(self, robotID: int, mp: "AbstractMotionPlanner") -> None:
        self.motionPlanners[robotID] = mp

    def getMotionPlanner(self, robotID: int) -> "AbstractMotionPlanner | None":
        return self.motionPlanners.get(robotID)

    def inParkingPose(self, robotID: int) -> bool:
        rr = self.getRobotReport(robotID)
        return rr is not None and rr.getPathIndex() == -1

    # --------------------------------------------------------------- messaging

    def setCriticalPoint(self, robotID: int, criticalPoint: int, retransmitt: bool) -> None:
        tracker = self.trackers.get(robotID)
        if tracker is not None and robotID not in self.muted and not isinstance(tracker, TrajectoryEnvelopeTrackerDummy):
            prev = self.communicatedCPs.get(tracker)
            if prev is None or prev[0] != criticalPoint or retransmitt:
                self.communicatedCPs[tracker] = (criticalPoint, self.getCurrentTimeInMillis())
                self.externalCPCounters[tracker] = self.externalCPCounters.get(tracker, -1) + 1
                tracker.setCriticalPointWithCounter(criticalPoint, self.externalCPCounters[tracker] % 2147483647)
                self.totalMsgsSent += 1
                if retransmitt:
                    self.totalMsgsReTx += 1

    # ------------------------------------------------------------- placement

    def placeRobot(
        self,
        robotID: int,
        currentPose: Pose | None = None,
        parking: TrajectoryEnvelope | None = None,
        location: str | None = None,
    ) -> None:
        if self.solver is None:
            raise RuntimeError("Solver not initialized, please call setupSolver() first!")

        if parking is None:
            assert currentPose is not None
            parking = self.solver.createParkingEnvelope(
                robotID, PARKING_DURATION, currentPose, location or str(currentPose), self.getFootprint(robotID)
            )
        else:
            currentPose = parking.getTrajectory().getPose()[0]

        self.isDriving[robotID] = False

        outer_cb_holder = self.trackingCallbacks

        class _PlacementCallback(TrackingCallback):
            def beforeTrackingStart(self_inner) -> None:
                existing = outer_cb_holder.get(robotID)
                if existing is not None:
                    existing.myTE = self_inner.myTE
                    existing.beforeTrackingStart()

            def onTrackingStart(self_inner) -> None:
                existing = outer_cb_holder.get(robotID)
                if existing is not None:
                    existing.onTrackingStart()

            def onNewGroundEnvelope(self_inner) -> None:
                existing = outer_cb_holder.get(robotID)
                if existing is not None:
                    existing.onNewGroundEnvelope()

            def beforeTrackingFinished(self_inner) -> None:
                existing = outer_cb_holder.get(robotID)
                if existing is not None:
                    existing.beforeTrackingFinished()

            def onTrackingFinished(self_inner) -> None:
                existing = outer_cb_holder.get(robotID)
                if existing is not None:
                    existing.onTrackingFinished()

            def onPositionUpdate(self_inner) -> list[str] | None:
                existing = outer_cb_holder.get(robotID)
                if existing is not None:
                    return existing.onPositionUpdate()
                return None

        cb = _PlacementCallback(parking)
        tracker = TrajectoryEnvelopeTrackerDummy(parking, 300, self.TEMPORAL_RESOLUTION, self, cb)

        self.currentParkingEnvelopes.append(tracker.getTrajectoryEnvelope())

        old = self.trackers.get(robotID)
        if old is not None:
            self.externalCPCounters.pop(old, None)
        self.trackers.pop(robotID, None)
        self.trackers[robotID] = tracker
        self.externalCPCounters[tracker] = -1

    # ----------------------------------------------------------- free/busy

    def isFree(self, robotID: int) -> bool:
        if self.solver is None:
            raise RuntimeError("Solver not initialized, please call setupSolver() first!")
        if robotID in self.muted:
            return False
        if any(te.getRobotID() == robotID for te, _ts in self.missionsPool):
            return False
        tracker = self.trackers.get(robotID)
        if not isinstance(tracker, TrajectoryEnvelopeTrackerDummy):
            return False
        return not tracker.isParkingFinished()

    # ---------------------------------------------------------- stopping points

    def atStoppingPoint(self, robotID: int) -> bool:
        return robotID in self.stoppingPointTimers

    def spawnWaitingThread(self, robotID: int, index: int, duration: int) -> None:
        # `index` is the position within stoppingPoints[robotID]/stoppingTimes[robotID]
        # (matching Java's List.remove(int index), a positional removal), not the
        # waypoint value stored at that position.
        async def _wait() -> None:
            await asyncio.sleep(duration / 1000.0)
            async with self._lock:
                points = self.stoppingPoints.get(robotID)
                times = self.stoppingTimes.get(robotID)
                if points is not None and times is not None and index < len(points):
                    points.pop(index)
                    times.pop(index)
                self.stoppingPointTimers.pop(robotID, None)
                self.updateDependencies()

        self.stoppingPointTimers[robotID] = asyncio.create_task(_wait(), name=f"stopping-point-Robot{robotID}")

    # ------------------------------------------------------------ obstacles

    def getObstaclesInCriticalPoints(self, robotIDs: Sequence[int]) -> list[BaseGeometry]:
        ret: list[BaseGeometry] = []
        for robotID in robotIDs:
            tracker = self.trackers.get(robotID)
            if isinstance(tracker, TrajectoryEnvelopeTrackerDummy):
                rr = self.getRobotReport(robotID)
                assert rr is not None and rr.getPose() is not None
                ret.append(self.makeObstacles(robotID, rr.getPose())[0])
                continue
            assert tracker is not None
            currentDeps = self.getCurrentDependencies()
            dep = currentDeps.get(robotID)
            if dep is None:
                path = tracker.getTrajectoryEnvelope().getTrajectory().getPose()
                waitingPose = path[-1]
            else:
                waitingPose = dep.getWaitingPose()
            currentFP = self.makeObstacles(robotID, waitingPose)[0]
            rr = self.getRobotReport(robotID)
            currentPoint = rr.getPathIndex() if rr is not None else -1
            if currentPoint != -1 and dep is not None and currentPoint > dep.getWaitingPoint():
                currentPose = dep.getWaitingTrajectoryEnvelope().getTrajectory().getPose()[currentPoint]
                currentFP = self.makeObstacles(robotID, currentPose)[0]
            ret.append(currentFP)
        return ret

    def getObstaclesFromWaitingRobots(self, robotID: int) -> list[BaseGeometry]:
        dep = self.getCurrentDependencies().get(robotID)
        if dep is None:
            return []
        waitingPose = dep.getWaitingTrajectoryEnvelope().getTrajectory().getPose()[dep.getWaitingPoint()]
        return [self.makeObstacles(robotID, waitingPose)[0]]

    def makeObstacles(self, robotID: int, *obstaclePoses: Pose) -> list[BaseGeometry]:
        footprint = self.getFootprint(robotID)
        assert footprint is not None
        ret = []
        for p in obstaclePoses:
            obstacle = rotate(footprint, p.getTheta(), origin=(0.0, 0.0), use_radians=True)
            obstacle = translate(obstacle, p.getX(), p.getY())
            ret.append(obstacle)
        return ret

    def doReplanning(
        self,
        mp: "AbstractMotionPlanner | None",
        fromPose: Pose,
        toPose: Pose,
        obstaclesToConsider: Sequence[BaseGeometry] = (),
    ) -> tuple | None:
        if mp is None:
            return None
        mp.setStart(fromPose)
        mp.setGoals(toPose)
        if obstaclesToConsider:
            mp.addObstacles(obstaclesToConsider)
        successful = mp.plan()
        if obstaclesToConsider:
            mp.clearObstacles()
        if successful:
            return mp.getPath()
        return None

    # ------------------------------------------------------------ core algorithm

    @abc.abstractmethod
    def updateDependencies(self) -> None: ...

    def canExitCriticalSection(
        self,
        drivingCurrentIndex: int,
        waitingCurrentIndex: int,
        drivingTE: TrajectoryEnvelope,
        waitingTE: TrajectoryEnvelope,
        lastIndexOfCSDriving: int,
    ) -> bool:
        drivingCurrentIndex = max(drivingCurrentIndex, 0)
        waitingCurrentIndex = max(waitingCurrentIndex, 0)
        placementWaiting = waitingTE.makeFootprint(waitingTE.getTrajectory().getPoseSteering()[waitingCurrentIndex])
        for i in range(drivingCurrentIndex, lastIndexOfCSDriving + 1):
            placementDriving = drivingTE.makeFootprint(drivingTE.getTrajectory().getPoseSteering()[i])
            if placementWaiting.intersects(placementDriving):
                return False
        return True

    def getCriticalPoint(self, yieldingRobotID: int, cs: CriticalSection, leadingRobotCurrentPathIndex: int) -> int:
        te1, te2 = cs.getTe1(), cs.getTe2()
        assert te1 is not None and te2 is not None
        if te1.getRobotID() == yieldingRobotID:
            leadingRobotStart, yieldingRobotStart = cs.getTe2Start(), cs.getTe1Start()
            leadingRobotEnd, yieldingRobotEnd = cs.getTe2End(), cs.getTe1End()
            leadingRobotTE, yieldingRobotTE = te2, te1
        else:
            leadingRobotStart, yieldingRobotStart = cs.getTe1Start(), cs.getTe2Start()
            leadingRobotEnd, yieldingRobotEnd = cs.getTe1End(), cs.getTe2End()
            leadingRobotTE, yieldingRobotTE = te1, te2

        if leadingRobotCurrentPathIndex < leadingRobotStart:
            return max(0, yieldingRobotStart - TRAILING_PATH_POINTS)

        leadingPoses = leadingRobotTE.getTrajectory().getPose()
        leadingRobotInPose = translate(
            rotate(leadingRobotTE.getFootprint(), leadingPoses[leadingRobotCurrentPathIndex].getTheta(), origin=(0.0, 0.0), use_radians=True),
            leadingPoses[leadingRobotCurrentPathIndex].getX(),
            leadingPoses[leadingRobotCurrentPathIndex].getY(),
        )
        if leadingRobotCurrentPathIndex <= leadingRobotEnd:
            polys = [leadingRobotInPose]
            for i in range(leadingRobotCurrentPathIndex + 1, leadingRobotEnd + 1):
                p = leadingPoses[i]
                polys.append(
                    translate(
                        rotate(leadingRobotTE.getFootprint(), p.getTheta(), origin=(0.0, 0.0), use_radians=True),
                        p.getX(),
                        p.getY(),
                    )
                )
            leadingRobotInPose = unary_union(polys) if len(polys) > 1 else polys[0]

        yieldingPoses = yieldingRobotTE.getTrajectory().getPose()
        for i in range(yieldingRobotStart, yieldingRobotEnd + 1):
            p = yieldingPoses[i]
            yieldingRobotInPose = translate(
                rotate(yieldingRobotTE.getFootprint(), p.getTheta(), origin=(0.0, 0.0), use_radians=True),
                p.getX(),
                p.getY(),
            )
            if leadingRobotInPose.intersects(yieldingRobotInPose):
                return max(0, i - TRAILING_PATH_POINTS)

        return max(0, yieldingRobotStart - TRAILING_PATH_POINTS)

    def isAhead(self, cs: CriticalSection, rr1: RobotReport, rr2: RobotReport) -> int:
        te1, te2 = cs.getTe1(), cs.getTe2()
        assert te1 is not None and te2 is not None
        if cs not in self.allCriticalSections or rr1.getPathIndex() > cs.getTe1End() or rr2.getPathIndex() > cs.getTe2End():
            return -2
        if rr1.getPathIndex() >= cs.getTe1Start() and rr2.getPathIndex() >= cs.getTe2Start():
            path1 = te1.getTrajectory().getPoseSteering()
            path2 = te2.getTrajectory().getPoseSteering()
            dist1 = sum(
                path1[i].getPose().distanceTo(path1[i + 1].getPose())
                for i in range(cs.getTe1Start(), rr1.getPathIndex() - 1)
            )
            dist2 = sum(
                path2[i].getPose().distanceTo(path2[i + 1].getPose())
                for i in range(cs.getTe2Start(), rr2.getPathIndex() - 1)
            )
            return 1 if dist1 > dist2 else -1
        return 0

    # ------------------------------------------------------- critical sections

    def computeCriticalSections(self) -> None:
        currentReports: dict[int, RobotReport | None] = {
            robotID: self.getRobotReport(robotID) for robotID in self.trackers
        }
        drivingEnvelopes = self.getDrivingEnvelopes()

        def minStartFor(te: TrajectoryEnvelope) -> int:
            rr = currentReports.get(te.getRobotID())
            return rr.getPathIndex() if rr is not None else -1

        def maxDim(a: TrajectoryEnvelope, b: TrajectoryEnvelope) -> float:
            da = self.getMaxFootprintDimension(a.getRobotID())
            db = self.getMaxFootprintDimension(b.getRobotID())
            assert da is not None and db is not None
            return min(da, db)

        for te_a in drivingEnvelopes:
            for te_b in self.envelopesToTrack:
                if te_a.getRobotID() == te_b.getRobotID():
                    continue
                for cs in self.getCriticalSections(
                    te_a, minStartFor(te_a), te_b, minStartFor(te_b), self.checkEscapePoses, maxDim(te_a, te_b)
                ):
                    self.allCriticalSections.add(cs)

        for i, te_a in enumerate(self.envelopesToTrack):
            for te_b in self.envelopesToTrack[i + 1 :]:
                if te_a.getRobotID() == te_b.getRobotID():
                    continue
                for cs in self.getCriticalSections(
                    te_a, minStartFor(te_a), te_b, minStartFor(te_b), self.checkEscapePoses, maxDim(te_a, te_b)
                ):
                    self.allCriticalSections.add(cs)

        for te_a in drivingEnvelopes:
            for te_b in self.currentParkingEnvelopes:
                if te_a.getRobotID() == te_b.getRobotID():
                    continue
                for cs in self.getCriticalSections(
                    te_a, minStartFor(te_a), te_b, minStartFor(te_b), self.checkEscapePoses, maxDim(te_a, te_b)
                ):
                    self.allCriticalSections.add(cs)

        for te_a in self.envelopesToTrack:
            for te_b in self.currentParkingEnvelopes:
                if te_a.getRobotID() == te_b.getRobotID():
                    continue
                for cs in self.getCriticalSections(
                    te_a, minStartFor(te_a), te_b, minStartFor(te_b), self.checkEscapePoses, maxDim(te_a, te_b)
                ):
                    self.allCriticalSections.add(cs)

        self.filterCriticalSections()
        self.onCriticalSectionUpdate()

    def filterCriticalSections(self) -> None:
        # allCriticalSections is a set keyed by CriticalSection.__eq__/__hash__,
        # so duplicates (Java's redundant post-hoc sweep) can't occur here.
        pass

    @staticmethod
    def getCriticalSections(
        te1: TrajectoryEnvelope,
        minStart1: int,
        te2: TrajectoryEnvelope,
        minStart2: int,
        checkEscapePoses: bool,
        maxDimensionOfSmallestRobot: float,
    ) -> list[CriticalSection]:
        css: list[CriticalSection] = []
        se1 = te1.getSpatialEnvelope()
        se2 = te2.getSpatialEnvelope()
        shape1 = se1.getPolygon()
        shape2 = se2.getPolygon()

        if not shape1.intersects(shape2):
            return css

        path1 = se1.getPath()
        path2 = se2.getPath()

        def placement(se, ps):
            fp = se.getFootprint()
            pose = ps.getPose()
            return translate(rotate(fp, pose.getTheta(), origin=(0.0, 0.0), use_radians=True), pose.getX(), pose.getY())

        if checkEscapePoses:
            safe = any(not placement(se1, ps).intersects(shape2) for ps in path1)
            if len(path1) == 1 or len(path2) == 1:
                safe = True
            if not safe:
                log.warning("cannot_coordinate_fully_overlapped", te1=te1.getID(), te2=te2.getID())

            safe = any(not placement(se2, ps).intersects(shape1) for ps in path2)
            if len(path1) == 1 or len(path2) == 1:
                safe = True
            if not safe:
                log.warning("cannot_coordinate_fully_overlapped", te1=te1.getID(), te2=te2.getID())

        gc = shape1.intersection(shape2)
        geoms = list(gc.geoms) if hasattr(gc, "geoms") else [gc]
        geoms = [g for g in geoms if not g.is_empty and g.area > 0]
        if not geoms:
            return css

        all_intersections: list[BaseGeometry] = []
        if len(geoms) == 1:
            all_intersections.append(geoms[0])
        else:
            for i in range(1, len(geoms)):
                prev, nxt = geoms[i - 1], geoms[i]
                if prev.distance(nxt) < maxDimensionOfSmallestRobot:
                    all_intersections.append(unary_union([prev, nxt]).convex_hull)
                else:
                    all_intersections.append(prev)
                    if i == len(geoms) - 1:
                        all_intersections.append(nxt)

        placements1 = [placement(se1, ps) for ps in path1]
        placements2 = [placement(se2, ps) for ps in path2]

        for g in all_intersections:
            te1Starts: list[int] = []
            te1Ends: list[int] = []
            te2Starts: list[int] = []
            te2Ends: list[int] = []

            started = False
            for j, pl in enumerate(placements1):
                intersects = pl.intersects(g)
                if not started and intersects:
                    started = True
                    te1Starts.append(j)
                elif started and not intersects:
                    te1Ends.append(max(j - 1, 0))
                    started = False
                if started and j == len(placements1) - 1:
                    te1Ends.append(len(placements1) - 1)

            started = False
            for j, pl in enumerate(placements2):
                intersects = pl.intersects(g)
                if not started and intersects:
                    started = True
                    te2Starts.append(j)
                elif started and not intersects:
                    te2Ends.append(max(j - 1, 0))
                    started = False
                if started and j == len(placements2) - 1:
                    te2Ends.append(len(placements2) - 1)

            cssOneIntersectionPiece: list[CriticalSection] = []
            for k1 in range(len(te1Starts)):
                for k2 in range(len(te2Starts)):
                    if te1Ends[k1] >= max(0, minStart1) and te2Ends[k2] >= max(0, minStart2):
                        cssOneIntersectionPiece.append(
                            CriticalSection(te1, te2, te1Starts[k1], te2Starts[k2], te1Ends[k1], te2Ends[k2])
                        )

            r_te1Starts = [cs.getTe1Start() for cs in cssOneIntersectionPiece]
            r_te2Starts = [cs.getTe2Start() for cs in cssOneIntersectionPiece]

            if len(r_te1Starts) == 0 or len(r_te2Starts) == 0:
                cssOneIntersectionPiece = []
            elif len(r_te1Starts) != len(r_te2Starts):
                first, last = cssOneIntersectionPiece[0], cssOneIntersectionPiece[-1]
                cssOneIntersectionPiece = [
                    CriticalSection(te1, te2, first.getTe1Start(), first.getTe2Start(), last.getTe1End(), last.getTe2End())
                ]

            css.extend(cssOneIntersectionPiece)

        return css

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
            self.CSToDepsOrder.pop(cs, None)
            self.allCriticalSections.discard(cs)
            self.escapingCSToWaitingRobotIDandCP.pop(cs, None)

    # ------------------------------------------------------------------ missions

    def startTrackingAddedMissions(self) -> None:
        for te in self.envelopesToTrack:
            robotID = te.getRobotID()
            startParkingTracker = self.trackers.get(robotID)
            assert isinstance(startParkingTracker, TrajectoryEnvelopeTrackerDummy)
            startParking = startParkingTracker.getTrajectoryEnvelope()
            assert self.solver is not None
            endParking = self.solver.createParkingEnvelope(
                robotID, PARKING_DURATION, te.getTrajectory().getPose()[-1], "whatever", self.getFootprint(robotID)
            )

            coordinator = self

            class _DrivingCallback(TrackingCallback):
                async def beforeTrackingStart(self_inner) -> None:
                    existing = coordinator.trackingCallbacks.get(robotID)
                    if existing is not None:
                        existing.myTE = self_inner.myTE
                        existing.beforeTrackingStart()
                    tracker = coordinator.trackers.get(robotID)
                    if tracker is not None:
                        await tracker.waitUntilCanStartTracking()

                def onTrackingStart(self_inner) -> None:
                    existing = coordinator.trackingCallbacks.get(robotID)
                    if existing is not None:
                        existing.onTrackingStart()

                def onNewGroundEnvelope(self_inner) -> None:
                    existing = coordinator.trackingCallbacks.get(robotID)
                    if existing is not None:
                        existing.onNewGroundEnvelope()

                def beforeTrackingFinished(self_inner) -> None:
                    existing = coordinator.trackingCallbacks.get(robotID)
                    if existing is not None:
                        existing.beforeTrackingFinished()

                def onTrackingFinished(self_inner) -> None:
                    existing = coordinator.trackingCallbacks.get(robotID)
                    if existing is not None:
                        existing.onTrackingFinished()

                    coordinator.stoppingPoints.pop(robotID, None)
                    coordinator.stoppingTimes.pop(robotID, None)

                    coordinator.cleanUpRobotCS(robotID, -1)

                    if startParking in coordinator.currentParkingEnvelopes:
                        coordinator.currentParkingEnvelopes.remove(startParking)

                    old_tracker = coordinator.trackers.get(robotID)
                    if old_tracker is not None:
                        coordinator.communicatedCPs.pop(old_tracker, None)

                    coordinator.placeRobot(robotID, None, endParking, None)
                    coordinator.computeCriticalSections()
                    coordinator.updateDependencies()

                def onPositionUpdate(self_inner) -> list[str] | None:
                    existing = coordinator.trackingCallbacks.get(robotID)
                    if existing is not None:
                        return existing.onPositionUpdate()
                    return None

            cb = _DrivingCallback(te)

            old = self.trackers.get(robotID)
            if old is not None:
                self.externalCPCounters.pop(old, None)
            self.trackers.pop(robotID, None)

            tracker = self.getNewTracker(te, cb)
            self.trackers[robotID] = tracker
            self.externalCPCounters[tracker] = -1

            startParkingTracker.finishParking()
            self.isDriving[robotID] = True

        self.envelopesToTrack.clear()

    def addMissions(self, *missions: Mission) -> bool:
        if self.solver is None:
            raise RuntimeError("Solvers not initialized, please call setupSolver()")

        robotsToMissions: dict[int, list[Mission]] = {}
        for m in missions:
            robotsToMissions.setdefault(m.getRobotID(), []).append(m)

        for robotID in robotsToMissions:
            if not self.isFree(robotID):
                return False

        for robotID, robot_missions in robotsToMissions.items():
            startParkingTracker = self.trackers.get(robotID)
            assert isinstance(startParkingTracker, TrajectoryEnvelopeTrackerDummy)

            overallPath = [ps for m in robot_missions for ps in m.getPath()]
            te = self.solver.createEnvelopeNoParking(robotID, overallPath, "Driving", self.getFootprint(robotID))

            for m in robot_missions:
                for pose, duration in m.getStoppingPoints().items():
                    stoppingPoint = te.getSequenceNumber(pose.getX(), pose.getY())
                    if stoppingPoint == te.getPathLength() - 1:
                        stoppingPoint -= 2
                    self.stoppingPoints.setdefault(robotID, [])
                    self.stoppingTimes.setdefault(robotID, [])
                    if stoppingPoint not in self.stoppingPoints[robotID]:
                        self.stoppingPoints[robotID].append(stoppingPoint)
                        self.stoppingTimes[robotID].append(duration)

            for m in robot_missions[:-1]:
                destPose = m.getToPose()
                assert destPose is not None
                stoppingPoint = te.getSequenceNumber(destPose.getX(), destPose.getY())
                self.stoppingPoints.setdefault(robotID, [])
                self.stoppingTimes.setdefault(robotID, [])
                if stoppingPoint not in self.stoppingPoints[robotID]:
                    self.stoppingPoints[robotID].append(stoppingPoint)
                    self.stoppingTimes[robotID].append(DEFAULT_STOPPING_TIME)

            self.missionsPool.append((te, self.getCurrentTimeInMillis()))

        return True

    @abc.abstractmethod
    def getNewTracker(self, te: TrajectoryEnvelope, cb: TrackingCallback) -> "AbstractTrajectoryEnvelopeTracker": ...


class _DefaultForwardModel(ForwardModel):
    def canStop(self, te, currentState, targetPathIndex, useVelocity) -> bool:  # noqa: D102
        del te, currentState, targetPathIndex, useVelocity
        return True

    def getEarliestStoppingPathIndex(self, te, currentState) -> int:  # noqa: D102
        del te, currentState
        return 0
