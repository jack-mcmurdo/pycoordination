"""Allen interval-algebra relations.

The Java meta-csp framework exposes ``AllenIntervalConstraint.Type`` with the
13 jointly-exhaustive, pairwise-disjoint relations between two intervals
plus a few syntactic-sugar variants. We port only the 13 base relations and a
helper that converts each to the difference constraints needed by the STP
layer.

Naming convention: the relation is read ``A REL B``. So ``BEFORE`` means
A ends before B starts, ``MET_BY`` means B ends exactly when A starts, etc.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from coordination_oru.metacsp.temporal.bounds import Bounds


class AllenType(Enum):
    BEFORE = auto()
    MEETS = auto()
    OVERLAPS = auto()
    STARTS = auto()
    DURING = auto()
    FINISHES = auto()
    EQUALS = auto()
    AFTER = auto()
    MET_BY = auto()
    OVERLAPPED_BY = auto()
    STARTED_BY = auto()
    CONTAINS = auto()
    FINISHED_BY = auto()


@dataclass(frozen=True, slots=True)
class DiffConstraint:
    """A single ``x_dst - x_src <= weight`` STP edge."""

    src: int
    dst: int
    weight: float


def to_diff_constraints(
    rel: AllenType,
    a_start: int,
    a_end: int,
    b_start: int,
    b_end: int,
    bounds: "Bounds | None" = None,
) -> list[DiffConstraint]:
    """Translate ``A REL B`` into the STP edges that encode it.

    Some relations admit an optional gap or overlap interval ``bounds``:
    e.g. ``BEFORE`` with bounds ``(2, 5)`` means there is a gap of at least 2
    and at most 5 between ``end(A)`` and ``start(B)``. When ``bounds`` is
    ``None`` we use ``[0, inf)`` for the gap-bearing relations.

    Bounds use a closed interval ``[lb, ub]``. ``ub`` may be ``math.inf``.
    """
    lb, ub = (0.0, math.inf) if bounds is None else (bounds.lb, bounds.ub)

    def edge(src: int, dst: int, w: float) -> DiffConstraint:
        return DiffConstraint(src, dst, w)

    def interval(x: int, y: int, lo: float, hi: float) -> list[DiffConstraint]:
        # encode lo <= y - x <= hi
        return [edge(x, y, hi), edge(y, x, -lo)]

    match rel:
        case AllenType.BEFORE:
            return interval(a_end, b_start, lb, ub)
        case AllenType.AFTER:
            return interval(b_end, a_start, lb, ub)
        case AllenType.MEETS:
            return interval(a_end, b_start, 0.0, 0.0)
        case AllenType.MET_BY:
            return interval(b_end, a_start, 0.0, 0.0)
        case AllenType.OVERLAPS:
            return [
                *interval(a_start, b_start, lb, ub),
                edge(a_end, b_end, math.inf),
                edge(b_end, a_end, 0.0),  # b_end >= a_end
                edge(b_start, a_end, math.inf),
                edge(a_end, b_start, 0.0),  # a_end >= b_start (overlap)
            ]
        case AllenType.OVERLAPPED_BY:
            return [
                *interval(b_start, a_start, lb, ub),
                edge(b_end, a_end, math.inf),
                edge(a_end, b_end, 0.0),
                edge(a_start, b_end, math.inf),
                edge(b_end, a_start, 0.0),
            ]
        case AllenType.STARTS:
            return [
                *interval(a_start, b_start, 0.0, 0.0),
                edge(a_end, b_end, math.inf),
                edge(b_end, a_end, 0.0),  # b_end >= a_end (strict during)
            ]
        case AllenType.STARTED_BY:
            return [
                *interval(a_start, b_start, 0.0, 0.0),
                edge(b_end, a_end, math.inf),
                edge(a_end, b_end, 0.0),
            ]
        case AllenType.FINISHES:
            return [
                *interval(a_end, b_end, 0.0, 0.0),
                edge(b_start, a_start, math.inf),
                edge(a_start, b_start, 0.0),
            ]
        case AllenType.FINISHED_BY:
            return [
                *interval(a_end, b_end, 0.0, 0.0),
                edge(a_start, b_start, math.inf),
                edge(b_start, a_start, 0.0),
            ]
        case AllenType.DURING:
            return [
                edge(b_start, a_start, math.inf),
                edge(a_start, b_start, 0.0),  # a_start > b_start (lb 0 ok for closed)
                edge(a_end, b_end, math.inf),
                edge(b_end, a_end, 0.0),
            ]
        case AllenType.CONTAINS:
            return [
                edge(a_start, b_start, math.inf),
                edge(b_start, a_start, 0.0),
                edge(b_end, a_end, math.inf),
                edge(a_end, b_end, 0.0),
            ]
        case AllenType.EQUALS:
            return [
                *interval(a_start, b_start, 0.0, 0.0),
                *interval(a_end, b_end, 0.0, 0.0),
            ]
