"""Simulation tracker.

The tracker advances itself one waypoint per call to :meth:`advance`. The
:class:`~coordination_oru.simulation.sim_coordinator.SimulationCoordinator`
calls ``advance`` on every sim tick, gets back a :class:`RobotReport`, and
submits that report to the tracker so the coordinator's report-handling
path runs end-to-end (including critical-section bookkeeping).

Subclasses (e.g. :class:`RK4SimulationTracker`) override ``advance`` to
implement different motion models while keeping the same
``path_index``-based contract with the coordinator.
"""

from __future__ import annotations

from coordination_oru.coordinator.abstract_tracker import AbstractTrajectoryEnvelopeTracker
from coordination_oru.coordinator.robot_report import RobotReport
from coordination_oru.metacsp.spatial.pose import Pose
from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


class SimulationTracker(AbstractTrajectoryEnvelopeTracker):
    """Discrete waypoint-hop simulation tracker."""

    def __init__(self, robot_id: int, envelope: TrajectoryEnvelope, coordinator: object) -> None:
        super().__init__(robot_id, envelope, coordinator)  # type: ignore[arg-type]
        self._path_index: int = 0

    @property
    def path_index(self) -> int:
        return self._path_index

    @property
    def current_pose(self) -> Pose:
        return self.envelope.path[self._path_index].pose

    def advance(self, dt: float, sim_clock: float) -> RobotReport:
        """Advance one waypoint per tick if the permit allows it."""
        del dt  # unused; the discrete model ignores time step length
        permit = self.permit_index_until
        if self._path_index < permit and self._path_index < self.envelope.length - 1:
            self._path_index += 1
        completed = self._path_index >= self.envelope.length - 1
        return RobotReport(
            robot_id=self.robot_id,
            envelope_id=self.envelope.envelope_id,
            current_pose=self.current_pose,
            path_index=self._path_index,
            timestamp=sim_clock,
            completed=completed,
        )

    async def _on_report(self, report: RobotReport) -> None:
        if report.completed:
            self.log.info("sim_tracker_completed", path_index=report.path_index)
