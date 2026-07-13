"""``ForwardModel``: predicts whether/where a robot can stop.

``ConstantAccelerationForwardModel`` is a faithful numeric port of the Java
class of the same name, reusing the RK4 integrator from
:mod:`coordination_oru.simulation2D.trajectory_envelope_tracker_rk4` exactly
as the Java version calls into ``TrajectoryEnvelopeTrackerRK4`` statically.
"""

from __future__ import annotations

import abc
from typing import TYPE_CHECKING

from coordination_oru import network_configuration
from coordination_oru.simulation2D.state import State
from coordination_oru.simulation2D.trajectory_envelope_tracker_rk4 import (
    _robot_report_at,
    computeDistance,
    integrateRK4,
)

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
        state = State(0.0, currentState.getVelocity())
        time = 0.0
        delta_time = 0.0001
        lookahead_millis = self.controlPeriodInMillis + 2 * (
            network_configuration.getMaximumTxDelay() + self.trackingPeriodInMillis
        )
        if lookahead_millis > 0:
            while time * self.temporalResolution < lookahead_millis:
                integrateRK4(state, time, delta_time, False, self.maxVel, 1.0, self.maxAccel)
                time += delta_time

        while state.getVelocity() > 0:
            if state.getPosition() > distance:
                return False
            integrateRK4(state, time, delta_time, True, self.maxVel, 1.0, self.maxAccel)
            time += delta_time
        return True

    def getEarliestStoppingPathIndex(self, te: "TrajectoryEnvelope", currentState: "RobotReport") -> int:
        state = State(currentState.getDistanceTraveled(), currentState.getVelocity())
        time = 0.0
        delta_time = 0.0001
        lookahead_millis = self.controlPeriodInMillis + 2 * (
            network_configuration.getMaximumTxDelay() + self.trackingPeriodInMillis
        )
        if lookahead_millis > 0:
            while time * self.temporalResolution < lookahead_millis:
                integrateRK4(state, time, delta_time, False, self.maxVel, 1.0, self.maxAccel * 1.1)
                time += delta_time

        while state.getVelocity() > 0:
            integrateRK4(state, time, delta_time, True, self.maxVel, 1.0, self.maxAccel * 0.9)
            time += delta_time

        rr = _robot_report_at(te.getTrajectory(), state, -1, -1)
        return rr.getPathIndex() if rr is not None else -1
