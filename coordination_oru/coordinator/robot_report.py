"""``RobotReport``: telemetry packet emitted by a tracker."""

from __future__ import annotations

from dataclasses import dataclass

from coordination_oru.metacsp.spatial.pose import Pose


@dataclass(frozen=True, slots=True)
class RobotReport:
    robot_id: int
    envelope_id: int
    current_pose: Pose
    path_index: int  # index into the envelope's path (0..length-1)
    timestamp: float  # monotonic seconds
    completed: bool = False
