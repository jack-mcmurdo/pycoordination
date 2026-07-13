"""``RobotReport``: telemetry snapshot issued by a tracker."""

from __future__ import annotations

from coordination_oru.metacsp.spatial.pose import Pose


class RobotReport:
    def __init__(
        self,
        robotID: int,
        pose: Pose | None,
        pathIndex: int,
        velocity: float,
        distanceTraveled: float,
        criticalPoint: int,
    ) -> None:
        self.robotID = robotID
        self.pose = pose
        self.pathIndex = pathIndex
        self.velocity = velocity
        self.distanceTraveled = distanceTraveled
        self.criticalPoint = criticalPoint

    def getRobotID(self) -> int:
        return self.robotID

    def getPose(self) -> Pose | None:
        return self.pose

    def getPathIndex(self) -> int:
        return self.pathIndex

    def getVelocity(self) -> float:
        return self.velocity

    def getDistanceTraveled(self) -> float:
        return self.distanceTraveled

    def getCriticalPoint(self) -> int:
        return self.criticalPoint

    def __str__(self) -> str:
        return (
            f"Distance: {self.distanceTraveled:.4f}  Pose: {self.pose}  "
            f"Index: {self.pathIndex}  Velocity: {self.velocity:.4f}"
        )

    __repr__ = __str__
