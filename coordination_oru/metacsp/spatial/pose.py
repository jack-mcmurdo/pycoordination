"""Spatial primitives: ``Pose`` and ``PoseSteering``.

Mirrors the meta-csp ``Pose`` class. The 3-D fields (``z``, ``roll``,
``pitch``) are optional — left as ``NaN`` for planar workloads, which is the
common case in coordination_oru.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Pose:
    x: float
    y: float
    theta: float  # radians, planar yaw
    z: float = math.nan
    roll: float = math.nan
    pitch: float = math.nan

    def is_3d(self) -> bool:
        return not math.isnan(self.z)

    def distance_xy(self, other: "Pose") -> float:
        dx, dy = self.x - other.x, self.y - other.y
        return math.hypot(dx, dy)


@dataclass(frozen=True, slots=True)
class PoseSteering:
    """A pose with the steering angle the robot should apply at that pose."""

    pose: Pose
    steering: float = 0.0
