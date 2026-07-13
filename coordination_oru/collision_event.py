"""``CollisionEvent``: a recorded collision between two robots (simulation stats)."""

from __future__ import annotations

from dataclasses import dataclass

from coordination_oru.robot_report import RobotReport


@dataclass(frozen=True, slots=True)
class CollisionEvent:
    timestamp: int
    robot_report_1: RobotReport
    robot_report_2: RobotReport

    def __str__(self) -> str:
        return (
            f"Collision @ {self.timestamp}: "
            f"Robot{self.robot_report_1.robotID} <-> Robot{self.robot_report_2.robotID}"
        )
