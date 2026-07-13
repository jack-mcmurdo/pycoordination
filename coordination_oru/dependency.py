"""``Dependency``: the tuple ``(teWaiting, teDriving, waitingPoint, thresholdPoint)``.

The robot navigating ``teWaiting`` should not go beyond path index
``waitingPoint`` until the robot navigating ``teDriving`` reaches path index
``thresholdPoint``. ``teDriving`` may be ``None`` (a stopping-point-only
dependency), in which case ``robotIDDriving`` is 0, matching Java.

Note (ported verbatim from the Java javadoc): ``__eq__`` and ``__lt__`` give
different results for dependencies involving different robot pairs with the
same critical point — ``__lt__`` orders by waiting/threshold point only,
while ``__eq__`` also compares the envelope pair. Be sure to use the right
one when adding/removing dependencies from a given data structure.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from coordination_oru.metacsp.spatial.pose import Pose

if TYPE_CHECKING:
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


class Dependency:
    def __init__(
        self,
        teWaiting: "TrajectoryEnvelope",
        teDriving: "TrajectoryEnvelope | None",
        waitingPoint: int,
        thresholdPoint: int,
    ) -> None:
        self.teWaiting = teWaiting
        self.teDriving = teDriving
        self.waitingPoint = waitingPoint
        self.thresholdPoint = thresholdPoint
        self.robotIDWaiting = teWaiting.getRobotID()
        self.robotIDDriving = teDriving.getRobotID() if teDriving is not None else 0

    def __hash__(self) -> int:
        code = int(
            f"{self.robotIDWaiting}0{self.waitingPoint}0{self.robotIDDriving}0{self.thresholdPoint}"
        )
        return code % 2147483647  # Integer.MAX_VALUE

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Dependency):
            return False
        return (
            self.teWaiting == other.teWaiting
            and self.teDriving == other.teDriving
            and self.waitingPoint == other.waitingPoint
            and self.thresholdPoint == other.thresholdPoint
        )

    def __lt__(self, other: "Dependency") -> bool:
        return self.compareTo(other) < 0

    def compareTo(self, other: "Dependency") -> int:
        if self.waitingPoint != other.waitingPoint:
            return self.waitingPoint - other.waitingPoint
        return self.thresholdPoint - other.thresholdPoint

    def __str__(self) -> str:
        drivingTEID = 0 if self.teDriving is None else self.teDriving.getID()
        return (
            f"{self.getWaitingRobotID()}/{self.waitingPoint}(TE{self.teWaiting.getID()})-"
            f"{self.getDrivingRobotID()}/{self.thresholdPoint}(TE{drivingTEID})"
        )

    __repr__ = __str__

    # --------------------------------------------------------------- getters

    def getWaitingPose(self) -> Pose:
        return self.teWaiting.getTrajectory().getPose()[self.getWaitingPoint()]

    def getReleasingPose(self) -> Pose:
        assert self.teDriving is not None
        return self.teDriving.getTrajectory().getPose()[self.getReleasingPoint()]

    def getWaitingTrajectoryEnvelope(self) -> "TrajectoryEnvelope":
        return self.teWaiting

    def getDrivingTrajectoryEnvelope(self) -> "TrajectoryEnvelope | None":
        return self.teDriving

    def getWaitingPoint(self) -> int:
        return self.waitingPoint

    def getReleasingPoint(self) -> int:
        return self.thresholdPoint

    def getWaitingRobotID(self) -> int:
        return self.robotIDWaiting

    def getDrivingRobotID(self) -> int:
        return self.robotIDDriving
