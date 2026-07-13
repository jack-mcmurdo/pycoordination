"""``State``: 1-D arclength position/velocity pair used by the RK4 integrator."""

from __future__ import annotations


class State:
    __slots__ = ("position", "velocity")

    def __init__(self, distance: float, velocity: float) -> None:
        self.position = distance
        self.velocity = velocity

    def getPosition(self) -> float:
        return self.position

    def setPosition(self, distance: float) -> None:
        self.position = distance

    def getVelocity(self) -> float:
        return self.velocity

    def setVelocity(self, velocity: float) -> None:
        self.velocity = velocity

    def clone(self) -> "State":
        return State(self.position, self.velocity)

    def __str__(self) -> str:
        return f"Pos: {self.position} Vel: {self.velocity}"
