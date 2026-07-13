"""``Mission``: a goal for a robot, reached via a path connecting two poses."""

from __future__ import annotations

from itertools import count
from typing import Sequence

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering

_mission_order_counter = count(0)


class Mission:
    def __init__(
        self,
        robotID: int,
        path: Sequence[PoseSteering],
        fromLocation: str | None = None,
        toLocation: str | None = None,
        fromPose: Pose | None = None,
        toPose: Pose | None = None,
    ) -> None:
        self.robotID = robotID
        self.path: tuple[PoseSteering, ...] = tuple(path)
        self.order = next(_mission_order_counter)
        self.fromLocation = fromLocation if fromLocation is not None else str(self.path[0].getPose())
        self.toLocation = toLocation if toLocation is not None else str(self.path[-1].getPose())
        self.fromPose = fromPose if fromPose is not None else self.path[0].getPose()
        self.toPose = toPose if toPose is not None else self.path[-1].getPose()
        self.stoppingPoints: list[Pose] = []
        self.stoppingPointDurations: list[int] = []

    def __lt__(self, other: "Mission") -> bool:
        return self.order < other.order

    def compareTo(self, other: "Mission") -> int:
        return self.order - other.order

    def setStoppingPoint(self, pose: Pose, duration: int) -> None:
        self.stoppingPoints.append(pose)
        self.stoppingPointDurations.append(duration)

    def clearStoppingPoints(self) -> None:
        self.stoppingPoints.clear()
        self.stoppingPointDurations.clear()

    def getStoppingPoints(self) -> dict[Pose, int]:
        return dict(zip(self.stoppingPoints, self.stoppingPointDurations))

    def setToLocation(self, location: str) -> None:
        self.toLocation = location

    def setFromLocation(self, location: str) -> None:
        self.fromLocation = location

    def getRobotID(self) -> int:
        return self.robotID

    def getPath(self) -> tuple[PoseSteering, ...]:
        return self.path

    def setPath(self, path: Sequence[PoseSteering]) -> None:
        self.path = tuple(path)

    def getFromLocation(self) -> str | None:
        return self.fromLocation

    def getToLocation(self) -> str | None:
        return self.toLocation

    def getFromPose(self) -> Pose | None:
        return self.fromPose

    def setFromPose(self, fromPose: Pose) -> None:
        self.fromPose = fromPose

    def getToPose(self) -> Pose | None:
        return self.toPose

    def setToPose(self, toPose: Pose) -> None:
        self.toPose = toPose

    def __str__(self) -> str:
        length = f" (path length: {len(self.path)})" if self.path else ""
        return f"Robot{self.robotID}: {self.fromLocation} --> {self.toLocation}{length}"

    __repr__ = __str__
