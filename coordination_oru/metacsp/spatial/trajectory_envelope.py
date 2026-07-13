"""Trajectory envelope: a swept-area polygon plus its STP timing variables.

A :class:`TrajectoryEnvelope` is the runtime unit the coordinator reasons
over. It bundles:

* The full path the robot intends to follow (``tuple[PoseSteering, ...]``).
* The robot id that owns it.
* Two STP node indices (``start_node`` and ``end_node``) that participate
  in the all-pairs distance matrix.
* The :class:`SpatialEnvelope` — the union of per-waypoint footprints, plus
  the per-waypoint footprints themselves so we can localise where two
  envelopes start to interfere.

Java-named accessors (``getRobotID()``, ``getTrajectory()``,
``makeFootprint()``, ...) are provided so the ported coordinator code reads
like the Java original. Envelope identity (``__eq__``/``__hash__``) is by
``envelope_id``, matching Java object identity of metaCSP variables.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.metacsp.spatial.trajectory import Trajectory
from coordination_oru.util.geometry import place_footprint


@dataclass(frozen=True, slots=True)
class SpatialEnvelope:
    """Pre-computed swept geometry of an envelope's path.

    Mirrors Java's ``TrajectoryEnvelope.SpatialEnvelope`` (polygon + path +
    footprint), with the per-waypoint footprints cached in addition.
    """

    geometry: BaseGeometry  # union of all per-waypoint footprints
    footprints: tuple[Polygon, ...]  # per-waypoint footprints in path order
    path: tuple[PoseSteering, ...]
    footprint: Polygon  # the robot footprint centered at the origin

    # ------------------------------------------------- Java-named accessors

    def getPolygon(self) -> BaseGeometry:
        return self.geometry

    def getPath(self) -> tuple[PoseSteering, ...]:
        return self.path

    def getFootprint(self) -> Polygon:
        return self.footprint


def compute_spatial_envelope(
    path: Sequence[PoseSteering], footprint: Polygon
) -> SpatialEnvelope:
    """Sweep ``footprint`` along ``path`` and union the result."""
    if not path:
        raise ValueError("path must contain at least one PoseSteering")
    polys = tuple(place_footprint(footprint, ps.pose) for ps in path)
    union = unary_union(polys)
    return SpatialEnvelope(
        geometry=union, footprints=polys, path=tuple(path), footprint=footprint
    )


@dataclass(slots=True, eq=False)
class TrajectoryEnvelope:
    """A robot's planned trajectory expressed as an STP-aware swept envelope.

    ``envelope_id`` is assigned by the
    :class:`~coordination_oru.metacsp.spatial.trajectory_envelope_solver.TrajectoryEnvelopeSolver`
    that creates it; ``start_node`` / ``end_node`` are the STP variable indices
    for this envelope's start and end times.
    """

    envelope_id: int
    robot_id: int
    path: tuple[PoseSteering, ...]
    start_node: int
    end_node: int
    spatial_envelope: SpatialEnvelope
    footprint: Polygon
    component: str = "Driving"
    nominal_duration: float = 0.0
    completed: bool = False
    metadata: dict[str, object] = field(default_factory=dict)
    _trajectory: Trajectory | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if len(self.path) < 1:
            raise ValueError("envelope path must be non-empty")

    # identity semantics: metaCSP variables compare by ID
    def __eq__(self, other: object) -> bool:
        if not isinstance(other, TrajectoryEnvelope):
            return NotImplemented
        return self.envelope_id == other.envelope_id

    def __hash__(self) -> int:
        return hash(self.envelope_id)

    def __str__(self) -> str:
        return f"TE{self.envelope_id} (Robot{self.robot_id}, {self.component})"

    __repr__ = __str__

    # --------------------------------------------------------------- lookups

    def pose_at(self, index: int) -> Pose:
        return self.path[index].pose

    @property
    def length(self) -> int:
        return len(self.path)

    def waypoint_footprint(self, index: int) -> Polygon:
        return self.spatial_envelope.footprints[index]

    # ------------------------------------------------- Java-named accessors

    def getID(self) -> int:
        return self.envelope_id

    def getRobotID(self) -> int:
        return self.robot_id

    def getPathLength(self) -> int:
        return len(self.path)

    def getSpatialEnvelope(self) -> SpatialEnvelope:
        return self.spatial_envelope

    def getFootprint(self) -> Polygon:
        return self.footprint

    def getTrajectory(self) -> Trajectory:
        if self._trajectory is None:
            self._trajectory = Trajectory(self.path)
        return self._trajectory

    def makeFootprint(self, ps: PoseSteering) -> Polygon:
        return place_footprint(self.footprint, ps.pose)

    def getComponent(self) -> str:
        return f"Robot{self.robot_id}#{self.envelope_id}"

    def getSequenceNumberStart(self) -> int:
        return 0

    def getSequenceNumberEnd(self) -> int:
        return len(self.path) - 1

    def getSequenceNumber(self, x: float, y: float) -> int:
        """Index of the path point closest to ``(x, y)``.

        Mirrors Java's ``getSequenceNumber(Coordinate)`` used to locate
        stopping points along the path.
        """
        best, best_d2 = 0, float("inf")
        for i, ps in enumerate(self.path):
            d2 = (ps.pose.x - x) ** 2 + (ps.pose.y - y) ** 2
            if d2 < best_d2:
                best, best_d2 = i, d2
        return best
