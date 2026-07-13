"""``Trajectory``: the path view of a trajectory envelope.

Mirrors the metaCSP ``Trajectory`` class just enough for the coordinator
code, which accesses paths via ``te.getTrajectory().getPose()[i]`` and
``te.getTrajectory().getPoseSteering()``.
"""

from __future__ import annotations

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering


class Trajectory:
    __slots__ = ("_poseSteering", "_poses")

    def __init__(self, poseSteering: tuple[PoseSteering, ...]) -> None:
        self._poseSteering = tuple(poseSteering)
        self._poses = tuple(ps.pose for ps in self._poseSteering)

    def getPoseSteering(self) -> tuple[PoseSteering, ...]:
        return self._poseSteering

    def getPose(self) -> tuple[Pose, ...]:
        return self._poses

    def __len__(self) -> int:
        return len(self._poseSteering)
