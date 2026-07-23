"""Registry that owns trajectory envelopes and wires them to the STP network.

This is the surface the coordinator uses to:

* Create a new envelope from a path + footprint.
* Add ordering constraints between two envelopes (``A BEFORE B``, etc).
* Query the earliest/latest start/end of any envelope.
* Mark an envelope as completed (releases its STP variables in spirit; we
  keep the matrix nodes but flag the envelope as inactive so the
  coordination loop ignores it).

The original Java ``TrajectoryEnvelopeSolver`` also exposes Allen-relation
metaprogramming. We expose a thin ``add_allen_constraint`` helper for the
small subset the coordinator actually exercises.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import count
from typing import Sequence

from shapely.geometry import Polygon

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.metacsp.spatial.trajectory_envelope import (
    TrajectoryEnvelope,
    compute_spatial_envelope,
)
from coordination_oru.metacsp.temporal.allen import AllenType, to_diff_constraints
from coordination_oru.metacsp.temporal.bounds import Bounds
from coordination_oru.metacsp.temporal.stp import STPSolver


@dataclass
class TrajectoryEnvelopeSolver:
    """Tracks envelopes and exposes timing queries through an STP network."""

    max_envelopes: int = 64
    stp: STPSolver = field(init=False)
    _envelopes: dict[int, TrajectoryEnvelope] = field(default_factory=dict, init=False)
    _ids: count[int] = field(default_factory=lambda: count(1), init=False)

    def __post_init__(self) -> None:
        # Each envelope adds 2 STP variables; keep a margin for the origin.
        self.stp = STPSolver(max_nodes=self.max_envelopes * 2 + 4)

    # --------------------------------------------------------------- creation

    def create_envelope(
        self,
        robot_id: int,
        path: Sequence[PoseSteering],
        footprint: Polygon,
        *,
        nominal_duration: float = math.nan,
        earliest_start: float | None = None,
        latest_start: float | None = None,
    ) -> TrajectoryEnvelope:
        envelope_id = next(self._ids)
        spatial = compute_spatial_envelope(path, footprint)
        start_node = self.stp.new_variable()
        end_node = self.stp.new_variable()

        if math.isnan(nominal_duration):
            # heuristic: total path length / 1 m·s⁻¹ default
            length = sum(
                math.hypot(
                    path[i + 1].pose.x - path[i].pose.x,
                    path[i + 1].pose.y - path[i].pose.y,
                )
                for i in range(len(path) - 1)
            )
            nominal_duration = max(length, 1e-3)

        # encode the duration as start_node + nominal_duration <= end_node
        # (we allow the actual end to slip later, but never finish earlier)
        self.stp.add_constraint(end_node, start_node, -nominal_duration)

        if earliest_start is not None:
            self.stp.add_release_time(start_node, earliest_start)
        if latest_start is not None:
            self.stp.add_deadline(start_node, latest_start)

        envelope = TrajectoryEnvelope(
            envelope_id=envelope_id,
            robot_id=robot_id,
            path=tuple(path),
            start_node=start_node,
            end_node=end_node,
            spatial_envelope=spatial,
            footprint=footprint,
            nominal_duration=nominal_duration,
        )
        self._envelopes[envelope_id] = envelope
        return envelope

    # ------------------------------------------------------- Java-named factories

    def createEnvelopeNoParking(
        self,
        robotID: int,
        path: Sequence[PoseSteering],
        component: str,
        footprint: Polygon,
    ) -> TrajectoryEnvelope:
        te = self.create_envelope(robotID, path, footprint)
        te.component = component
        return te

    def createParkingEnvelope(
        self,
        robotID: int,
        duration: int,
        pose: "Pose",
        location: str,
        footprint: Polygon,
    ) -> TrajectoryEnvelope:
        te = self.create_envelope(robotID, (PoseSteering(pose),), footprint, nominal_duration=max(duration / 1000.0, 1e-3))
        te.component = location
        return te

    # ---------------------------------------------------------- registry view

    def envelopes(self) -> list[TrajectoryEnvelope]:
        return [te for te in self._envelopes.values() if not te.completed]

    def all_envelopes(self) -> list[TrajectoryEnvelope]:
        return list(self._envelopes.values())

    def get(self, envelope_id: int) -> TrajectoryEnvelope:
        return self._envelopes[envelope_id]

    def mark_completed(self, envelope_id: int) -> None:
        """Retire an envelope: flag it (``envelopes()`` excludes it from the
        active set consumed by critical-section/dependency computation, but
        it stays in ``all_envelopes()`` for introspection) and detach its
        two STP variables (``remove_variable`` — ports Java's
        ``removeConstraints`` + ``removeVariable``), freeing their slots for
        reuse rather than letting the temporal network's working set grow
        for the coordinator's entire lifetime.
        """
        envelope = self._envelopes[envelope_id]
        envelope.completed = True
        self.stp.remove_variable(envelope.start_node)
        self.stp.remove_variable(envelope.end_node)

    # ------------------------------------------------------------- ordering

    def add_ordering(
        self,
        first: TrajectoryEnvelope,
        second: TrajectoryEnvelope,
        *,
        gap_lb: float = 0.0,
        gap_ub: float = math.inf,
    ) -> None:
        """Constrain ``first`` to finish before ``second`` starts.

        ``gap_lb`` / ``gap_ub`` define the allowed gap between
        ``end(first)`` and ``start(second)``. Defaults to "any non-negative
        gap" — the most common case for critical-section serialisation.
        """
        self.stp.add_interval(first.end_node, second.start_node, gap_lb, gap_ub)

    def add_allen_constraint(
        self,
        rel: AllenType,
        a: TrajectoryEnvelope,
        b: TrajectoryEnvelope,
        bounds: Bounds | None = None,
    ) -> None:
        diffs = to_diff_constraints(
            rel, a.start_node, a.end_node, b.start_node, b.end_node, bounds
        )
        for d in diffs:
            self.stp.add_constraint(d.src, d.dst, d.weight)

    # ---------------------------------------------------------------- queries

    def earliest_start(self, envelope: TrajectoryEnvelope) -> float:
        return self.stp.get_earliest(envelope.start_node)

    def latest_start(self, envelope: TrajectoryEnvelope) -> float:
        return self.stp.get_latest(envelope.start_node)

    def earliest_end(self, envelope: TrajectoryEnvelope) -> float:
        return self.stp.get_earliest(envelope.end_node)

    def latest_end(self, envelope: TrajectoryEnvelope) -> float:
        return self.stp.get_latest(envelope.end_node)

    def is_consistent(self) -> bool:
        return self.stp.is_consistent()
