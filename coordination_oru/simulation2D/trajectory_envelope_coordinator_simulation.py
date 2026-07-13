"""``TrajectoryEnvelopeCoordinatorSimulation``: simulated-robot fleet coordinator.

Ported from Java's ``TrajectoryEnvelopeCoordinatorSimulation``. Wires up
``TrajectoryEnvelopeTrackerRK4`` as the tracker factory, provides sane
default footprint/kinodynamic parameters for robots that don't configure
their own, and mirrors ``getCurrentTimeInMillis()`` off the wall clock.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from shapely.geometry import Polygon

from coordination_oru.abstract_trajectory_envelope_tracker import (
    AbstractTrajectoryEnvelopeTracker,
)
from coordination_oru.collision_event import CollisionEvent
from coordination_oru.forward_model import ConstantAccelerationForwardModel
from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_tracker_rk4 import (
    TrajectoryEnvelopeTrackerRK4,
)
from coordination_oru.trajectory_envelope_coordinator import TrajectoryEnvelopeCoordinator

if TYPE_CHECKING:
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
    from coordination_oru.tracking_callback import TrackingCallback

DEFAULT_FOOTPRINT: tuple[tuple[float, float], ...] = (
    (-1.7, 0.7),
    (-1.7, -0.7),
    (2.7, -0.7),
    (2.7, 0.7),
)


class TrajectoryEnvelopeCoordinatorSimulation(TrajectoryEnvelopeCoordinator):
    def __init__(
        self,
        CONTROL_PERIOD: int = 1000,
        TEMPORAL_RESOLUTION: float = 1000.0,
        MAX_VELOCITY: float = 10.0,
        MAX_ACCELERATION: float = 1.0,
        DEFAULT_ROBOT_TRACKING_PERIOD: int = 30,
    ) -> None:
        super().__init__(CONTROL_PERIOD, TEMPORAL_RESOLUTION)
        self.DEFAULT_ROBOT_TRACKING_PERIOD = DEFAULT_ROBOT_TRACKING_PERIOD
        self.DEFAULT_MAX_VELOCITY = MAX_VELOCITY
        self.DEFAULT_MAX_ACCELERATION = MAX_ACCELERATION

        self._start_time = time.monotonic()
        self.useInternalCPs = True
        self.checkCollisions = False
        self.collisionsList: list[CollisionEvent] = []
        self.totalMsgsLost = 0
        self.totalPacketsLost = 0
        self.DEFAULT_FOOTPRINT: Polygon = Polygon(DEFAULT_FOOTPRINT)
        self.MAX_DEFAULT_FOOTPRINT_DIMENSION = self.computeMaxFootprintDimension(DEFAULT_FOOTPRINT)

    def setCheckCollisions(self, enable: bool) -> None:
        self.checkCollisions = enable

    def incrementLostMsgsCounter(self) -> None:
        self.totalMsgsLost += 1

    def incrementLostPacketsCounter(self) -> None:
        self.totalPacketsLost += 1

    def setUseInternalCriticalPoints(self, value: bool) -> None:
        self.useInternalCPs = value

    def getRobotMaxVelocity(self, robotID: int) -> float:
        return self.robotMaxVelocity.get(robotID, self.DEFAULT_MAX_VELOCITY)

    def getRobotMaxAcceleration(self, robotID: int) -> float:
        return self.robotMaxAcceleration.get(robotID, self.DEFAULT_MAX_ACCELERATION)

    def getMaxFootprintDimension(self, robotID: int) -> float:
        if robotID in self.footprints:
            dim = self.maxFootprintDimensions.get(robotID)
            if dim is not None:
                return dim
        return self.MAX_DEFAULT_FOOTPRINT_DIMENSION

    def getDefaultFootprint(self) -> Polygon:
        return self.DEFAULT_FOOTPRINT

    def getFootprint(self, robotID: int) -> Polygon:
        return self.footprints.get(robotID, self.DEFAULT_FOOTPRINT)

    def setDefaultFootprint(self, *coordinates: tuple[float, float]) -> None:
        self.DEFAULT_FOOTPRINT = Polygon(coordinates)
        self.MAX_DEFAULT_FOOTPRINT_DIMENSION = self.computeMaxFootprintDimension(coordinates)

    def getNewTracker(self, te: "TrajectoryEnvelope", cb: "TrackingCallback") -> AbstractTrajectoryEnvelopeTracker:
        robotID = te.getRobotID()
        trackingPeriod = self.getRobotTrackingPeriodInMillis(robotID)
        maxVel = self.getRobotMaxVelocity(robotID)
        maxAccel = self.getRobotMaxAcceleration(robotID)
        if not self.forwardModels.get(robotID):
            self.setForwardModel(
                robotID,
                ConstantAccelerationForwardModel(maxAccel, maxVel, self.TEMPORAL_RESOLUTION, self.CONTROL_PERIOD, trackingPeriod),
            )
        tracker = TrajectoryEnvelopeTrackerRK4(te, trackingPeriod, self.TEMPORAL_RESOLUTION, maxVel, maxAccel, self, cb)
        tracker.setUseInternalCriticalPoints(False)
        return tracker

    def getCurrentTimeInMillis(self) -> int:
        return int((time.monotonic() - self._start_time) * 1000)

    def addMissions(self, *missions: Mission) -> bool:
        user_stopping_points: dict[Mission, dict] = {}
        if self.useInternalCPs:
            for m in missions:
                path = m.getPath()
                sps = _compute_stopping_points(path)
                user_stopping_points[m] = m.getStoppingPoints()
                for i in sps:
                    m.setStoppingPoint(path[i - 1].getPose(), 100)
        if not super().addMissions(*missions):
            if self.useInternalCPs:
                for m in missions:
                    m.clearStoppingPoints()
                    for pose, duration in user_stopping_points[m].items():
                        m.setStoppingPoint(pose, duration)
            return False
        return True

    def onCriticalSectionUpdate(self) -> None:
        pass


def _compute_stopping_points(poses) -> list[int]:
    import math

    ret: list[int] = []
    prev_theta = poses[0].getTheta()
    if len(poses) > 1:
        prev_theta = math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX())
    for i in range(len(poses) - 1):
        theta = math.atan2(poses[i + 1].getY() - poses[i].getY(), poses[i + 1].getX() - poses[i].getX())
        delta_theta = theta - prev_theta
        prev_theta = theta
        if abs(delta_theta) > math.pi / 2 and abs(delta_theta) < 1.9 * math.pi:
            ret.append(i)
    return ret
