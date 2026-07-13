"""Footprint and geometry helpers built on shapely."""

from __future__ import annotations

from typing import TYPE_CHECKING

import shapely.affinity
from shapely.geometry import Polygon

if TYPE_CHECKING:
    from coordination_oru.metacsp.spatial.pose import Pose


def rectangular_footprint(length: float, width: float) -> Polygon:
    """Axis-aligned rectangular footprint centered at the origin.

    ``length`` is along the +x axis (forward), ``width`` along the y axis.
    """
    if length <= 0 or width <= 0:
        raise ValueError("length and width must be positive")
    half_l, half_w = length / 2.0, width / 2.0
    return Polygon(
        [
            (-half_l, -half_w),
            (half_l, -half_w),
            (half_l, half_w),
            (-half_l, half_w),
        ]
    )


def footprint_coords(length: float, width: float) -> tuple[tuple[float, float], ...]:
    """Coordinates of :func:`rectangular_footprint`, for ``setFootprint(robotID, *coords)``."""
    return tuple(rectangular_footprint(length, width).exterior.coords)[:-1]


def place_footprint(footprint: Polygon, pose: "Pose") -> Polygon:
    """Rotate ``footprint`` (centered at origin) by ``theta`` then translate to ``pose``."""
    rotated = shapely.affinity.rotate(footprint, pose.theta, origin=(0.0, 0.0), use_radians=True)
    return shapely.affinity.translate(rotated, pose.x, pose.y)
