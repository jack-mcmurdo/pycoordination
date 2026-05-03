"""``Mission``: an externally-supplied job for a single robot.

A mission is the user-facing thing that turns into a
:class:`~coordination_oru.metacsp.spatial.trajectory_envelope.TrajectoryEnvelope`
once the coordinator accepts it.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import count

from shapely.geometry import Polygon

from coordination_oru.metacsp.spatial.pose import PoseSteering


_mission_id_counter = count(1)


@dataclass(frozen=True, slots=True)
class Mission:
    mission_id: int
    robot_id: int
    path: tuple[PoseSteering, ...]
    footprint: Polygon

    @staticmethod
    def make(robot_id: int, path: list[PoseSteering] | tuple[PoseSteering, ...], footprint: Polygon) -> "Mission":
        return Mission(
            mission_id=next(_mission_id_counter),
            robot_id=robot_id,
            path=tuple(path),
            footprint=footprint,
        )
