"""``CriticalSection``: a contiguous range of overlapping waypoints between
two trajectory envelopes.

The Java original tracks per-envelope index ranges along with the two
envelopes; we keep the same shape. ``a_start..a_end`` (inclusive) is the
range of waypoint indices on envelope A whose per-waypoint footprint
intersects at least one waypoint footprint on envelope B in
``b_start..b_end``.
"""

from __future__ import annotations

from dataclasses import dataclass

from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


@dataclass(frozen=True, slots=True)
class CriticalSection:
    envelope_a: TrajectoryEnvelope
    envelope_b: TrajectoryEnvelope
    a_start: int
    a_end: int
    b_start: int
    b_end: int

    @property
    def key(self) -> frozenset[int]:
        """Stable identity of this CS, ignoring envelope ordering."""
        return frozenset({self.envelope_a.envelope_id, self.envelope_b.envelope_id})

    def cs_range_for(self, envelope_id: int) -> tuple[int, int]:
        if envelope_id == self.envelope_a.envelope_id:
            return self.a_start, self.a_end
        if envelope_id == self.envelope_b.envelope_id:
            return self.b_start, self.b_end
        raise ValueError(f"envelope {envelope_id} is not part of this CS")

    def other(self, envelope_id: int) -> TrajectoryEnvelope:
        if envelope_id == self.envelope_a.envelope_id:
            return self.envelope_b
        if envelope_id == self.envelope_b.envelope_id:
            return self.envelope_a
        raise ValueError(f"envelope {envelope_id} is not part of this CS")
