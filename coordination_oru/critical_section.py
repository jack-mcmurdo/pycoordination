"""``CriticalSection``: a quadruple ``(te1, te2, [te1Start,te1End], [te2Start,te2End])``.

``te1``/``te2`` overlap when robot 1 is between path indices ``te1Start`` and
``te1End`` while robot 2 is between ``te2Start`` and ``te2End``. Ported
verbatim from Java's ``CriticalSection``, including the symmetric
``te1``/``te2`` swap in ``__eq__``/``__hash__`` and the exclusion of the
mutable ``te1Break``/``te2Break`` fields from the hash (they mutate after
construction, so including them would break hash stability while the CS
lives in a set/dict — this is the fix for the "CS-identity/priority keying
bug" the Java original avoids too).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


class CriticalSection:
    def __init__(
        self,
        te1: "TrajectoryEnvelope | None",
        te2: "TrajectoryEnvelope | None",
        te1Start: int,
        te2Start: int,
        te1End: int,
        te2End: int,
    ) -> None:
        self.te1 = te1
        self.te2 = te2
        self.te1Start = te1Start
        self.te2Start = te2Start
        self.te1End = te1End
        self.te2End = te2End
        self.te1Break = -1
        self.te2Break = -1

    # ------------------------------------------------------------- equality

    def __eq__(self, other: object) -> bool:
        if self is other:
            return True
        if not isinstance(other, CriticalSection):
            return False
        if self.te1 is None or self.te2 is None:
            if other.te1 is not None and other.te2 is not None:
                return False
            if self.te1 is None:
                mine = self.te2
                theirs = other.te1 if other.te1 is not None else other.te2
                return mine == theirs
            return True
        if self.te1 == other.te1 and self.te2 == other.te2:
            return (
                self.te1End == other.te1End
                and self.te2End == other.te2End
                and self.te1Start == other.te1Start
                and self.te2Start == other.te2Start
            )
        if self.te1 == other.te2 and self.te2 == other.te1:
            return (
                self.te1End == other.te2End
                and self.te2End == other.te1End
                and self.te1Start == other.te2Start
                and self.te2Start == other.te1Start
            )
        return False

    def __hash__(self) -> int:
        prime = 31
        result = 1
        te1_hash = 0 if self.te1 is None else hash(self.te1)
        te2_hash = 0 if self.te2 is None else hash(self.te2)
        result = prime * result + te1_hash + te2_hash
        result = prime * result + self.te1End + self.te2End
        result = prime * result + abs(self.te1End - self.te2End)
        result = prime * result + self.te1Start + self.te2Start
        result = prime * result + abs(self.te1Start - self.te2Start)
        return result

    # --------------------------------------------------------------- getters

    def getTe1(self) -> "TrajectoryEnvelope | None":
        return self.te1

    def getTe2(self) -> "TrajectoryEnvelope | None":
        return self.te2

    def getTe1Start(self) -> int:
        return self.te1Start

    def getTe2Start(self) -> int:
        return self.te2Start

    def getTe1End(self) -> int:
        return self.te1End

    def getTe2End(self) -> int:
        return self.te2End

    def getTe1Break(self) -> int:
        return self.te1Break

    def getTe2Break(self) -> int:
        return self.te2Break

    def setTe1Break(self, te1Break: int) -> None:
        self.te1Break = te1Break

    def setTe2Break(self, te2Break: int) -> None:
        self.te2Break = te2Break

    def __str__(self) -> str:
        robot1 = "null" if self.te1 is None else f"Robot{self.te1.getRobotID()}"
        robot2 = "null" if self.te2 is None else f"Robot{self.te2.getRobotID()}"
        return (
            f"CriticalSection ({robot1} [{self.te1Start};{self.te1End}], "
            f"{robot2} [{self.te2Start};{self.te2End}])"
        )

    __repr__ = __str__
