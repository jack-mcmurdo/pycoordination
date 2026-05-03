"""Synthetic test paths.

The original Java demos load paths from text files in ``coordination_oru``'s
``paths/`` directory. Those files aren't part of this repo, so we generate
deterministic equivalents in code. Each helper returns a tuple of
``PoseSteering`` ready to feed into a ``Mission``.
"""

from __future__ import annotations

import math

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering


def line_path(
    x0: float, y0: float, x1: float, y1: float, *, step: float = 0.5
) -> tuple[PoseSteering, ...]:
    """Sample a straight segment from (x0,y0) to (x1,y1) at ``step``-metre spacing."""
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length == 0:
        return (PoseSteering(Pose(x0, y0, 0.0)),)
    n = max(2, int(math.ceil(length / step)) + 1)
    theta = math.atan2(dy, dx)
    out: list[PoseSteering] = []
    for i in range(n):
        t = i / (n - 1)
        out.append(PoseSteering(Pose(x0 + t * dx, y0 + t * dy, theta)))
    return tuple(out)


def two_robot_cross() -> tuple[tuple[PoseSteering, ...], tuple[PoseSteering, ...]]:
    """Two perpendicular straight paths that cross at the origin."""
    return (
        line_path(-5.0, 0.0, 5.0, 0.0),
        line_path(0.0, -5.0, 0.0, 5.0),
    )


def three_robot_intersection() -> tuple[
    tuple[PoseSteering, ...], tuple[PoseSteering, ...], tuple[PoseSteering, ...]
]:
    """Three paths that all pass through the origin from different directions."""
    return (
        line_path(-8.0, 0.0, 8.0, 0.0),  # west → east
        line_path(0.0, -8.0, 0.0, 8.0),  # south → north
        line_path(-6.0, -6.0, 6.0, 6.0),  # SW → NE diagonal
    )


def shuttle_path(start: tuple[float, float], end: tuple[float, float]) -> tuple[PoseSteering, ...]:
    return line_path(start[0], start[1], end[0], end[1])
