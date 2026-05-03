"""Trajectory envelope: a swept-area polygon plus its STP timing variables.

A :class:`TrajectoryEnvelope` is the runtime unit the coordinator reasons
over. It bundles:

* The full path the robot intends to follow (``list[PoseSteering]``).
* The robot id that owns it.
* Two STP node indices (``start_node`` and ``end_node``) that participate
  in the all-pairs distance matrix.
* The :class:`SpatialEnvelope` — the union of per-waypoint footprints, plus
  the per-waypoint footprints themselves so we can localise where two
  envelopes start to interfere.

The Java original lazily computes the swept area; we eagerly compute it once
at construction time. Paths are immutable for the lifetime of the envelope,
so caching is correct and avoids mid-coordination stalls.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from shapely.geometry import Polygon
from shapely.ops import unary_union

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.util.geometry import place_footprint


@dataclass(frozen=True, slots=True)
class SpatialEnvelope:
    """Pre-computed swept geometry of an envelope's path."""

    geometry: Polygon  # union of all per-waypoint footprints
    footprints: tuple[Polygon, ...]  # per-waypoint footprints in path order


def compute_spatial_envelope(
    path: Sequence[PoseSteering], footprint: Polygon
) -> SpatialEnvelope:
    """Sweep ``footprint`` along ``path`` and union the result."""
    if not path:
        raise ValueError("path must contain at least one PoseSteering")
    polys = tuple(place_footprint(footprint, ps.pose) for ps in path)
    union = unary_union(polys)
    if not isinstance(union, Polygon):
        # A MultiPolygon means the sweep is disjoint along the path — unusual
        # but legal. Buffer-by-0 collapses adjacent components into a single
        # Polygon when geometrically possible; otherwise we keep the original.
        repaired = union.buffer(0.0)
        if isinstance(repaired, Polygon):
            return SpatialEnvelope(geometry=repaired, footprints=polys)
        return SpatialEnvelope(geometry=Polygon(union.convex_hull), footprints=polys)
    return SpatialEnvelope(geometry=union, footprints=polys)


@dataclass(slots=True)
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
    nominal_duration: float = 0.0
    completed: bool = False
    metadata: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if len(self.path) < 1:
            raise ValueError("envelope path must be non-empty")

    # --------------------------------------------------------------- lookups

    def pose_at(self, index: int) -> Pose:
        return self.path[index].pose

    @property
    def length(self) -> int:
        return len(self.path)

    def waypoint_footprint(self, index: int) -> Polygon:
        return self.spatial_envelope.footprints[index]
