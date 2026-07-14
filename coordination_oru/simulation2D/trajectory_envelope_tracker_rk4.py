"""``TrajectoryEnvelopeTrackerRK4``: simulated robot tracked via RK4 integration.

Ported from Java's ``TrajectoryEnvelopeTrackerRK4``. Simplifications versus
the Java original (documented, not silently dropped):

* No simulated network delay/packet loss on reports (``getLastRobotReport``
  always returns the freshest report) — :mod:`coordination_oru.network_configuration`
  defaults to zero delay/loss, so this only changes behaviour if a caller
  deliberately configures nonzero values, which this port does not yet wire
  up for the report path.
* No ground-envelope / sub-envelope deadline bookkeeping (see
  :mod:`coordination_oru.abstract_trajectory_envelope_tracker`).

The kinodynamic core — ``computeDistance``, ``integrateRK4``, the slow-down
profile, ``setCriticalPoint`` honouring the critical point by decelerating
to a stop *at* it — is a faithful numeric port.
"""

from __future__ import annotations

import asyncio
import bisect
import time
from typing import TYPE_CHECKING

from coordination_oru.abstract_trajectory_envelope_tracker import (
    AbstractTrajectoryEnvelopeTracker,
)
from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.metacsp.spatial.trajectory import Trajectory
from coordination_oru.robot_report import RobotReport
from coordination_oru.simulation2D.derivative import Derivative
from coordination_oru.simulation2D.state import State
from coordination_oru.tracking_callback import TrackingCallback
from coordination_oru.util.logging import get_logger

if TYPE_CHECKING:
    from coordination_oru.abstract_trajectory_envelope_coordinator import (
        AbstractTrajectoryEnvelopeCoordinator,
    )
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope

log = get_logger(__name__)

WAIT_AMOUNT_AT_END_MILLIS = 3000
EPSILON = 0.01


def computeDistance(traj: Trajectory, startIndex: int, endIndex: int) -> float:
    poses = traj.getPose()
    ret = 0.0
    for i in range(startIndex, min(endIndex, len(poses) - 1)):
        ret += poses[i].distanceTo(poses[i + 1])
    return ret


def integrateRK4(
    state: State,
    time_: float,
    deltaTime: float,
    slowDown: bool,
    MAX_VELOCITY: float,
    MAX_VELOCITY_DAMPENING_FACTOR: float,
    MAX_ACCELERATION: float,
) -> None:
    a = Derivative.evaluate(
        state, time_, 0.0, Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION
    )
    b = Derivative.evaluate(
        state, time_, deltaTime / 2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION
    )
    c = Derivative.evaluate(
        state, time_, deltaTime / 2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION
    )
    d = Derivative.evaluate(
        state, time_, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION
    )

    dxdt = (1.0 / 6.0) * (a.getVelocity() + 2.0 * (b.getVelocity() + c.getVelocity()) + d.getVelocity())
    dvdt = (1.0 / 6.0) * (
        a.getAcceleration() + 2.0 * (b.getAcceleration() + c.getAcceleration()) + d.getAcceleration()
    )

    state.setPosition(state.getPosition() + dxdt * deltaTime)
    state.setVelocity(state.getVelocity() + dvdt * deltaTime)


def _robot_report_at(traj: Trajectory, aux_state: State | None, robot_id: int, critical_point: int) -> RobotReport | None:
    if aux_state is None:
        return None
    poses = traj.getPose()
    accumulated = 0.0
    pose = None
    current_index = -1
    for i in range(len(poses) - 1):
        delta_s = poses[i].distanceTo(poses[i + 1])
        accumulated += delta_s
        if accumulated > aux_state.getPosition():
            ratio = 1.0 - (accumulated - aux_state.getPosition()) / delta_s
            pose = poses[i].interpolate(poses[i + 1], ratio)
            current_index = i
            break
    if current_index == -1:
        current_index = len(poses) - 1
        pose = poses[current_index]
    return RobotReport(robot_id, pose, current_index, aux_state.getVelocity(), aux_state.getPosition(), critical_point)


class TrajectoryEnvelopeTrackerRK4(AbstractTrajectoryEnvelopeTracker):
    def __init__(
        self,
        te: "TrajectoryEnvelope",
        timeStep: int,
        temporalResolution: float,
        maxVelocity: float,
        maxAcceleration: float,
        tec: "AbstractTrajectoryEnvelopeCoordinator",
        cb: TrackingCallback | None,
    ) -> None:
        self.MAX_VELOCITY = maxVelocity
        self.MAX_ACCELERATION = maxAcceleration
        self.state = State(0.0, 0.0)
        self._useInternalCPs = True
        self._internalCriticalPoints: list[int] = []
        self._curvatureDampening: list[float] = []
        self._run_task: asyncio.Task[None] | None = None
        self._internal_cp_task: asyncio.Task[None] | None = None
        # super().__init__ schedules (but, being asyncio, does not yet run) the
        # monitor task, which will eventually call startTracking(); it needs
        # self.te/self.traj, which the base constructor sets. We then finish
        # precomputing totalDistance/slowdown profile before yielding control
        # back to the event loop, so no task can observe a half-built tracker.
        super().__init__(te, temporalResolution, tec, timeStep, cb)
        self._precompute(self.traj)

    def _precompute(self, traj: Trajectory) -> None:
        self.totalDistance = computeDistance(traj, 0, len(traj.getPose()) - 1)
        self.overallDistance = self.totalDistance
        self._computeInternalCriticalPoints(traj)
        self._slowDownProfile = self._getSlowdownProfile()
        self.positionToSlowDown = self._computePositionToSlowDown()

    def setUseInternalCriticalPoints(self, value: bool) -> None:
        self._useInternalCPs = value

    def _computeInternalCriticalPoints(self, traj: Trajectory) -> None:
        import math

        poses = traj.getPose()
        self._curvatureDampening = [1.0] * len(poses)
        self._internalCriticalPoints = []
        prev_theta = poses[0].getTheta()
        if len(poses) > 1:
            prev_theta = math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX())
        for i in range(len(poses) - 1):
            theta = math.atan2(poses[i + 1].getY() - poses[i].getY(), poses[i + 1].getX() - poses[i].getX())
            delta_theta = theta - prev_theta
            prev_theta = theta
            if abs(delta_theta) > math.pi / 2 and abs(delta_theta) < 1.9 * math.pi:
                self._internalCriticalPoints.append(i)

    def getCurvatureDampening(self, index: int, backwards: bool) -> float:
        if not backwards:
            return self._curvatureDampening[index]
        return self._curvatureDampening[len(self.traj.getPose()) - 1 - index]

    def onTrajectoryEnvelopeUpdate(self) -> None:
        self.totalDistance = computeDistance(self.traj, 0, len(self.traj.getPose()) - 1)
        self.overallDistance = self.totalDistance
        self._computeInternalCriticalPoints(self.traj)
        self._slowDownProfile = self._getSlowdownProfile()
        self.positionToSlowDown = self._computePositionToSlowDown()

    def startTracking(self) -> None:
        self._run_task = asyncio.create_task(self._run(), name=f"rk4-Robot{self.te.getRobotID()}")
        if self._useInternalCPs:
            self._internal_cp_task = asyncio.create_task(
                self._internal_cp_loop(), name=f"rk4-internal-cp-Robot{self.te.getRobotID()}"
            )

    async def _internal_cp_loop(self) -> None:
        user_cp_replacements: dict[int, int] = {}
        while self._run_task is not None and not self._run_task.done():
            to_remove: list[int] = []
            for i in list(self._internalCriticalPoints):
                if self.getRobotReport().getPathIndex() >= i:
                    to_remove.append(i)
                    self.setCriticalPoint(user_cp_replacements.get(i, -1))
                    break
                else:
                    if self.criticalPoint == -1 or self.criticalPoint > i:
                        user_cp_replacements[i] = self.criticalPoint
                        self.setCriticalPoint(i)
                        break
            for i in to_remove:
                self._internalCriticalPoints.remove(i)
            await asyncio.sleep(self.trackingPeriodInMillis / 1000.0)

    # ------------------------------------------------------------- slowdown

    def _getSlowdownProfile(self) -> list[tuple[float, float]]:
        ret: list[tuple[float, float]] = []
        temp = State(0.0, 0.0)
        ret.append((temp.getVelocity(), temp.getPosition()))
        t = 0.0
        delta_time = 0.5 * (self.trackingPeriodInMillis / self.temporalResolution)
        while temp.getVelocity() < self.MAX_VELOCITY * 1.1:
            dampening = self.getCurvatureDampening(
                _robot_report_at(self.traj, temp, self.te.getRobotID(), -1).getPathIndex(), True
            )
            integrateRK4(temp, t, delta_time, False, self.MAX_VELOCITY * 1.1, dampening, self.MAX_ACCELERATION)
            t += delta_time
            ret.append((temp.getVelocity(), temp.getPosition()))
        # Java keeps a TreeMap sorted by speed descending; a list sorted the same way
        # supports the same "first speed <= current" scan in _computePositionToSlowDown.
        ret.sort(key=lambda kv: -kv[0])
        return ret

    def _computePositionToSlowDown(self) -> float:
        temp = State(self.state.getPosition(), self.state.getVelocity())
        t = 0.0
        delta_time = 0.5 * (self.trackingPeriodInMillis / self.temporalResolution)
        while temp.getPosition() < self.totalDistance:
            prev_speed = -1.0
            first_time = True
            found_break = False
            for speed, space_needed in self._slowDownProfile:
                if temp.getVelocity() > speed:
                    landing_position = temp.getPosition() + (0.0 if first_time else self._space_for(prev_speed))
                    if landing_position > self.totalDistance:
                        return temp.getPosition()
                    found_break = True
                    break
                first_time = False
                prev_speed = speed
            del found_break
            dampening = self.getCurvatureDampening(
                _robot_report_at(self.traj, temp, self.te.getRobotID(), -1).getPathIndex(), True
            )
            integrateRK4(temp, t, delta_time, False, self.MAX_VELOCITY, dampening, self.MAX_ACCELERATION)
            t += delta_time
        return -self.totalDistance

    def _space_for(self, speed: float) -> float:
        for s, space in self._slowDownProfile:
            if s == speed:
                return space
        return 0.0

    # ------------------------------------------------------------- critical point

    def setCriticalPoint(self, criticalPointToSet: int) -> None:
        if self.criticalPoint != criticalPointToSet:
            current_path_index = self.getRobotReport().getPathIndex()
            # ``>=`` (Java uses ``>``): a robot standing exactly at the
            # requested waypoint can still honour the hold — this matters for
            # missions that begin inside a critical section, where the first
            # critical point equals the start index 0. The rollback below
            # still rejects the request when the robot is already rolling
            # past the waypoint.
            if criticalPointToSet != -1 and criticalPointToSet >= current_path_index:
                total_distance_bkp = self.totalDistance
                critical_point_bkp = self.criticalPoint
                position_to_slow_down_bkp = self.positionToSlowDown

                self.criticalPoint = criticalPointToSet
                self.totalDistance = computeDistance(self.traj, 0, criticalPointToSet)
                self.positionToSlowDown = self._computePositionToSlowDown()

                if self.positionToSlowDown < self.state.getPosition():
                    self.criticalPoint = critical_point_bkp
                    self.totalDistance = total_distance_bkp
                    self.positionToSlowDown = position_to_slow_down_bkp
            elif criticalPointToSet != -1 and criticalPointToSet < current_path_index:
                log.warning(
                    "ignored_late_critical_point",
                    robotID=self.te.getRobotID(),
                    criticalPoint=criticalPointToSet,
                    pathIndex=current_path_index,
                )
            elif criticalPointToSet == -1:
                self.criticalPoint = criticalPointToSet
                self.totalDistance = computeDistance(self.traj, 0, len(self.traj.getPose()) - 1)
                self.positionToSlowDown = self._computePositionToSlowDown()

    def setCriticalPointWithCounter(self, criticalPointToSet: int, externalCPCounter: int) -> None:
        # Java's RK4 tracker overrides the 2-arg (network-timestamped) form to
        # simulate delay/loss and, on every delivered message, flips
        # canStartTracking — this is what lets the monitor loop's
        # beforeTrackingStart() proceed and spawn the run()/internal-CP tasks.
        # This port drops the delay/loss simulation (see module docstring)
        # but keeps the canStartTracking trigger, which is load-bearing.
        super().setCriticalPointWithCounter(criticalPointToSet, externalCPCounter)
        if not self.canStartTracking():
            self.setCanStartTracking()

    def getRobotReport(self) -> RobotReport:
        return _robot_report_at(self.traj, self.state, self.te.getRobotID(), self.criticalPoint) or RobotReport(
            self.te.getRobotID(), self.traj.getPose()[0], -1, 0.0, 0.0, -1
        )

    def getCurrentTimeInMillis(self) -> int:
        return self.tec.getCurrentTimeInMillis()

    # ------------------------------------------------------------- main loop

    async def _run(self) -> None:
        elapsed_tracking_time = 0.0
        delta_time = 0.0
        at_cp = False

        while True:
            skip_integration = False
            if self.state.getPosition() >= self.positionToSlowDown and self.state.getVelocity() < 0.0:
                if self.criticalPoint == -1 and not at_cp:
                    self.state = State(self.totalDistance, 0.0)
                    self.onPositionUpdate()
                    break
                if not at_cp:
                    at_cp = True
                skip_integration = True

            time_start = time.monotonic()

            if not skip_integration:
                if at_cp:
                    at_cp = False
                slowing_down = self.state.getPosition() >= self.positionToSlowDown
                dampening = self.getCurvatureDampening(self.getRobotReport().getPathIndex(), False)
                integrateRK4(
                    self.state, elapsed_tracking_time, delta_time, slowing_down, self.MAX_VELOCITY, dampening, self.MAX_ACCELERATION
                )

            self.onPositionUpdate()

            await asyncio.sleep(self.trackingPeriodInMillis / 1000.0)

            delta_time = (time.monotonic() - time_start)
            elapsed_tracking_time += delta_time

        await asyncio.sleep(WAIT_AMOUNT_AT_END_MILLIS / 1000.0)
