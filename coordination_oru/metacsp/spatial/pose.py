"""Spatial primitives: ``Pose`` and ``PoseSteering``.

Mirrors the metaCSP ``Pose``/``PoseSteering`` classes. Java-named accessors
(``getX()``, ``distanceTo()``, ``interpolate()``, ...) are provided so ported
coordinator code reads like the Java original; the pythonic attribute access
(``pose.x``) remains available.
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

    # ------------------------------------------------- Java-named accessors

    def getX(self) -> float:
        return self.x

    def getY(self) -> float:
        return self.y

    def getTheta(self) -> float:
        return self.theta

    def distanceTo(self, other: "Pose") -> float:
        return self.distance_xy(other)

    def interpolate(self, other: "Pose", ratio: float) -> "Pose":
        """Linear interpolation towards ``other``; theta via shortest arc."""
        return Pose(
            x=self.x + ratio * (other.x - self.x),
            y=self.y + ratio * (other.y - self.y),
            theta=_lerp_angle(self.theta, other.theta, ratio),
        )

    def __str__(self) -> str:
        return f"({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})"


@dataclass(frozen=True, slots=True)
class PoseSteering:
    """A pose with the steering angle the robot should apply at that pose."""

    pose: Pose
    steering: float = 0.0

    # ------------------------------------------------- Java-named accessors

    def getPose(self) -> Pose:
        return self.pose

    def getSteering(self) -> float:
        return self.steering

    def getX(self) -> float:
        return self.pose.x

    def getY(self) -> float:
        return self.pose.y

    def getTheta(self) -> float:
        return self.pose.theta


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Shortest-arc interpolation between two angles in radians."""
    diff = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + diff * t
