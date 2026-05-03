"""RK4 kinematic simulation tracker.

A drop-in replacement for the discrete waypoint-hop
:class:`~coordination_oru.simulation.sim_tracker.SimulationTracker` that
integrates a 1-D arclength model

    ds/dt = v
    dv/dt = a(s, v)

with classic 4th-order Runge-Kutta. The acceleration is bang-bang:

* Brake at ``-a_max`` if continuing at ``+a_max`` (or cruising) would
  overshoot the permit boundary, given the current ``v`` and the room
  needed to decelerate to zero.
* Otherwise accelerate at ``+a_max`` while ``v < v_max``.
* Cruise at zero acceleration while ``v == v_max``.

The permit boundary is read from ``permit_index_until`` (set by the
coordinator) and converted to a target arclength ``cum_s[permit]``. Path
positions and per-waypoint indices are recovered from the integrated
arclength via a precomputed cumulative-length table.

This matches the behaviour of the Java ``TrajectoryEnvelopeTrackerRK4``:
robots accelerate to a fixed cruise speed, decelerate at a fixed rate to
stop just short of any held critical section, and resume when the loser
is unblocked.
"""

from __future__ import annotations

import bisect
import math

from coordination_oru.coordinator.robot_report import RobotReport
from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
from coordination_oru.simulation.sim_tracker import SimulationTracker


class RK4SimulationTracker(SimulationTracker):
    def __init__(
        self,
        robot_id: int,
        envelope: TrajectoryEnvelope,
        coordinator: object,
        *,
        v_max: float = 1.0,
        a_max: float = 0.5,
    ) -> None:
        if v_max <= 0:
            raise ValueError("v_max must be positive")
        if a_max <= 0:
            raise ValueError("a_max must be positive")
        super().__init__(robot_id, envelope, coordinator)
        self.v_max = v_max
        self.a_max = a_max
        self.s: float = 0.0
        self.v: float = 0.0
        self._cum_s: list[float] = self._build_cumulative_arclength(envelope)
        self._total_s: float = self._cum_s[-1]

    # --------------------------------------------------------------- setup

    @staticmethod
    def _build_cumulative_arclength(envelope: TrajectoryEnvelope) -> list[float]:
        cum: list[float] = [0.0]
        for i in range(1, envelope.length):
            p0 = envelope.path[i - 1].pose
            p1 = envelope.path[i].pose
            cum.append(cum[-1] + math.hypot(p1.x - p0.x, p1.y - p0.y))
        return cum

    # ------------------------------------------------------------ queries

    @property
    def path_index(self) -> int:
        return self._index_at(self.s)

    @property
    def current_pose(self) -> Pose:
        return self._pose_at(self.s)

    def _index_at(self, s: float) -> int:
        if s >= self._total_s:
            return self.envelope.length - 1
        if s <= 0:
            return 0
        # cum_s is strictly non-decreasing; bisect_right gives the first
        # index whose arclength is strictly greater than s, so subtracting
        # one yields the last waypoint we have already reached.
        i = bisect.bisect_right(self._cum_s, s) - 1
        return max(0, min(i, self.envelope.length - 1))

    def _pose_at(self, s: float) -> Pose:
        if s <= 0:
            return self.envelope.path[0].pose
        if s >= self._total_s:
            return self.envelope.path[-1].pose
        i = self._index_at(s)
        if i >= self.envelope.length - 1:
            return self.envelope.path[-1].pose
        seg_len = self._cum_s[i + 1] - self._cum_s[i]
        if seg_len <= 0:
            return self.envelope.path[i].pose
        t = (s - self._cum_s[i]) / seg_len
        p0 = self.envelope.path[i].pose
        p1 = self.envelope.path[i + 1].pose
        return Pose(
            x=p0.x + t * (p1.x - p0.x),
            y=p0.y + t * (p1.y - p0.y),
            theta=_lerp_angle(p0.theta, p1.theta, t),
        )

    # ----------------------------------------------------- dynamics

    def _target_arclength(self) -> float:
        permit = max(0, min(self.permit_index_until, self.envelope.length - 1))
        return self._cum_s[permit]

    def _accel(self, s: float, v: float, target_s: float) -> float:
        # past target — emergency brake
        if s >= target_s:
            return -self.a_max if v > 0.0 else 0.0
        remaining = target_s - s
        # distance needed to decelerate to a stop at -a_max
        brake_distance = (v * v) / (2.0 * self.a_max) if v > 0.0 else 0.0
        if brake_distance >= remaining:
            return -self.a_max
        if v < self.v_max:
            return self.a_max
        return 0.0  # cruise

    def advance(self, dt: float, sim_clock: float) -> RobotReport:
        target_s = self._target_arclength()
        s, v = self.s, self.v

        # RK4 over (s, v) with derivatives (v, a(s, v))
        k1_s, k1_v = v, self._accel(s, v, target_s)
        k2_s, k2_v = (
            v + 0.5 * dt * k1_v,
            self._accel(s + 0.5 * dt * k1_s, v + 0.5 * dt * k1_v, target_s),
        )
        k3_s, k3_v = (
            v + 0.5 * dt * k2_v,
            self._accel(s + 0.5 * dt * k2_s, v + 0.5 * dt * k2_v, target_s),
        )
        k4_s, k4_v = (
            v + dt * k3_v,
            self._accel(s + dt * k3_s, v + dt * k3_v, target_s),
        )

        new_s = s + (dt / 6.0) * (k1_s + 2.0 * k2_s + 2.0 * k3_s + k4_s)
        new_v = v + (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)

        # Clamp velocity into [0, v_max] and arclength into [0, total].
        new_v = max(0.0, min(self.v_max, new_v))
        new_s = max(0.0, min(self._total_s, new_s))
        # Don't permit advancing past the target; if RK4 overshoots due to
        # the discontinuous control, snap to the target and zero v.
        if new_s > target_s:
            new_s = target_s
            new_v = 0.0
        if new_s >= self._total_s:
            new_v = 0.0

        self.s, self.v = new_s, new_v

        completed = new_s >= self._total_s - 1e-9
        return RobotReport(
            robot_id=self.robot_id,
            envelope_id=self.envelope.envelope_id,
            current_pose=self._pose_at(new_s),
            path_index=self._index_at(new_s),
            timestamp=sim_clock,
            completed=completed,
        )


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Shortest-arc interpolation between two angles in radians."""
    diff = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + diff * t
