"""``AbstractMotionPlanner``: pluggable re-planning interface.

Ported from Java's ``AbstractMotionPlanner``, trimmed to the surface the
coordinator's ``callOnePathReplan``/``doReplanning`` flow actually calls.
No planner implementation is bundled — ``breakDeadlocksByReplanning`` is a
no-op unless a concrete subclass is injected via ``setMotionPlanner``.
"""

from __future__ import annotations

import abc
from typing import Sequence

from shapely.geometry.base import BaseGeometry

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering


class AbstractMotionPlanner(abc.ABC):
    def __init__(self) -> None:
        self.start: Pose | None = None
        self.goal: tuple[Pose, ...] = ()
        self.footprintCoords: Sequence[tuple[float, float]] | None = None
        self.pathPS: tuple[PoseSteering, ...] | None = None
        self._obstacles: list[BaseGeometry] = []

    def setFootprint(self, *coords: tuple[float, float]) -> None:
        self.footprintCoords = coords

    def setStart(self, p: Pose) -> None:
        self.start = p

    def setGoals(self, *poses: Pose) -> None:
        self.goal = tuple(poses)

    def getPath(self) -> tuple[PoseSteering, ...] | None:
        return self.pathPS

    def addObstacles(self, geoms: Sequence[BaseGeometry]) -> None:
        self._obstacles.extend(geoms)

    def clearObstacles(self) -> None:
        self._obstacles.clear()

    def getObstacles(self) -> list[BaseGeometry]:
        return list(self._obstacles)

    @abc.abstractmethod
    def doPlanning(self) -> bool:
        """Populate ``self.pathPS``; return ``True`` iff planning succeeded."""

    def plan(self) -> bool:
        successful = self.doPlanning()
        if not successful:
            return False
        if self.pathPS is None:
            return False
        return True

    def writeDebugImage(self) -> None:
        pass
