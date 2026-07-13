"""``Derivative``: (velocity, acceleration) pair for RK4 integration steps."""

from __future__ import annotations

from coordination_oru.simulation2D.state import State

EPSILON = 0.0001


class Derivative:
    __slots__ = ("velocity", "acceleration")

    def __init__(self, velocity: float = 0.0, acceleration: float = 0.0) -> None:
        self.velocity = velocity
        self.acceleration = acceleration

    def getVelocity(self) -> float:
        return self.velocity

    def getAcceleration(self) -> float:
        return self.acceleration

    @staticmethod
    def evaluate(
        initial_state: State,
        time: float,
        delta_time: float,
        deriv: "Derivative",
        slow_down: bool,
        max_velocity: float,
        max_velocity_dampening_factor: float,
        max_acceleration: float,
    ) -> "Derivative":
        position = initial_state.getPosition() + deriv.getVelocity() * delta_time
        velocity = initial_state.getVelocity() + deriv.getAcceleration() * delta_time
        new_state = State(position, velocity)
        new_velocity = new_state.getVelocity()
        new_acceleration = Derivative.compute_acceleration(
            new_state, time + delta_time, slow_down, max_velocity, max_velocity_dampening_factor, max_acceleration
        )
        return Derivative(new_velocity, new_acceleration)

    @staticmethod
    def compute_acceleration(
        state: State,
        time: float,
        slow_down: bool,
        max_velocity: float,
        max_velocity_dampening_factor: float,
        max_acceleration: float,
    ) -> float:
        del time
        if not slow_down:
            if state.getVelocity() > max_velocity_dampening_factor * max_velocity:
                return 0.0
            return max_acceleration
        return -max_acceleration
