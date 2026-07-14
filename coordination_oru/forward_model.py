"""``ForwardModel``: predicts whether/where a robot can stop.

``ConstantAccelerationForwardModel`` matches the Java class of the same
name. The Java version forward-simulates with the RK4 integrator at 0.1 ms
steps; since the dynamics it integrates are piecewise-constant acceleration
(``Derivative.compute_acceleration`` is bang-bang toward a velocity cap),
the trajectory has an exact closed form, which is used here instead: the
RK4 loop cost ~100 ms per call in pure Python and starved the asyncio
event loop that the simulation and the web viewer share.
"""

from __future__ import annotations

import abc
from typing import TYPE_CHECKING

from coordination_oru import network_configuration
from coordination_oru.simulation2D.state import State
from coordination_oru.simulation2D.trajectory_envelope_tracker_rk4 import (
    _robot_report_at,
    computeDistance,
)


def _accelerate_capped(v0: float, accel: float, vcap: float, duration: float) -> tuple[float, float]:
    """Distance travelled and final velocity after ``duration`` seconds of
    acceleration ``accel`` toward the velocity cap (zero acceleration at or
    above it) — the closed form of what the Java RK4 loop integrates."""
    if v0 >= vcap:
        return v0 * duration, v0
    t_cap = (vcap - v0) / accel
    if duration <= t_cap:
        return v0 * duration + 0.5 * accel * duration * duration, v0 + accel * duration
    return v0 * t_cap + 0.5 * accel * t_cap * t_cap + vcap * (duration - t_cap), vcap

if TYPE_CHECKING:
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
    from coordination_oru.robot_report import RobotReport


class ForwardModel(abc.ABC):
    @abc.abstractmethod
    def canStop(
        self, te: "TrajectoryEnvelope", currentState: "RobotReport", targetPathIndex: int, useVelocity: bool
    ) -> bool: ...

    @abc.abstractmethod
    def getEarliestStoppingPathIndex(self, te: "TrajectoryEnvelope", currentState: "RobotReport") -> int: ...


class ConstantAccelerationForwardModel(ForwardModel):
    def __init__(
        self,
        maxAccel: float,
        maxVel: float,
        temporalResolution: float,
        controlPeriodInMillis: int,
        trackingPeriodInMillis: int,
    ) -> None:
        self.maxAccel = maxAccel
        self.maxVel = maxVel
        self.temporalResolution = temporalResolution
        self.controlPeriodInMillis = controlPeriodInMillis
        self.trackingPeriodInMillis = trackingPeriodInMillis

    def _lookahead_seconds(self) -> float:
        lookahead_millis = self.controlPeriodInMillis + 2 * (
            network_configuration.getMaximumTxDelay() + self.trackingPeriodInMillis
        )
        return max(0.0, lookahead_millis / self.temporalResolution)

    def canStop(
        self, te: "TrajectoryEnvelope", currentState: "RobotReport", targetPathIndex: int, useVelocity: bool
    ) -> bool:
        if useVelocity and currentState.getVelocity() <= 0.0:
            return True
        distance = computeDistance(
            te.getTrajectory(),
            currentState.getPathIndex() if currentState.getPathIndex() != -1 else 0,
            targetPathIndex,
        )
        # keep accelerating for the lookahead window, then brake to a halt
        position, velocity = _accelerate_capped(
            currentState.getVelocity(), self.maxAccel, self.maxVel, self._lookahead_seconds()
        )
        position += velocity * velocity / (2.0 * self.maxAccel)
        return position <= distance

    def getEarliestStoppingPathIndex(self, te: "TrajectoryEnvelope", currentState: "RobotReport") -> int:
        # optimistic acceleration (×1.1) during the lookahead window,
        # pessimistic braking (×0.9) after — same margins as the Java model
        travelled, velocity = _accelerate_capped(
            currentState.getVelocity(), self.maxAccel * 1.1, self.maxVel, self._lookahead_seconds()
        )
        position = currentState.getDistanceTraveled() + travelled
        position += velocity * velocity / (2.0 * self.maxAccel * 0.9)

        rr = _robot_report_at(te.getTrajectory(), State(position, 0.0), -1, -1)
        return rr.getPathIndex() if rr is not None else -1
