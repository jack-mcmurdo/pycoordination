"""Closed temporal interval `[lb, ub]` (seconds).

Mirrors the Java meta-csp ``Bounds`` value type: a flyweight pair used by the
STP layer and trajectory-envelope plumbing whenever a temporal interval needs
to be passed around (durations, release times, deadlines, etc.).
"""

from __future__ import annotations

import math
from typing import NamedTuple


class Bounds(NamedTuple):
    """Closed temporal interval. ``ub`` may be ``math.inf`` for unbounded."""

    lb: float
    ub: float

    def __repr__(self) -> str:
        ub = "inf" if math.isinf(self.ub) else f"{self.ub:g}"
        return f"Bounds({self.lb:g}, {ub})"

    def intersect(self, other: "Bounds") -> "Bounds | None":
        lb = max(self.lb, other.lb)
        ub = min(self.ub, other.ub)
        if lb > ub:
            return None
        return Bounds(lb, ub)

    def is_singleton(self) -> bool:
        return self.lb == self.ub


UNBOUNDED: Bounds = Bounds(0.0, math.inf)
