"""``RobotAtCriticalSection``: a (report, CS) pair used by ordering heuristics."""

from __future__ import annotations

from coordination_oru.critical_section import CriticalSection
from coordination_oru.robot_report import RobotReport


class RobotAtCriticalSection:
    def __init__(self, rr: RobotReport, cs: CriticalSection) -> None:
        self.rr = rr
        self.cs = cs

    def getRobotReport(self) -> RobotReport:
        return self.rr

    def getCriticalSection(self) -> CriticalSection:
        return self.cs
