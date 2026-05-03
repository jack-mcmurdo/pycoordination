"""Abstract per-robot async tracker.

Replaces the Java ``AbstractTrajectoryEnvelopeTracker`` thread. The tracker
is an :class:`asyncio.Task` that:

1. Holds the envelope it's executing for one robot.
2. Receives :class:`RobotReport` updates externally (from a real robot driver
   or from the simulator) via :meth:`submit_report`.
3. Forwards each report to the coordinator so coordination decisions can be
   refreshed.
4. Honours :attr:`permit_index_until` — the highest waypoint index the
   coordinator currently allows the robot to advance to. Trackers must NOT
   advance past this index. Subclasses translate this into a real-world stop
   command (e.g. emit a velocity-zero on a ROS topic).

The base class only handles the lifecycle and report-routing — the actual
robot-driving loop lives in subclasses (see ``simulation.SimulationTracker``).
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from coordination_oru.coordinator.robot_report import RobotReport
from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
from coordination_oru.util.logging import get_logger

if TYPE_CHECKING:
    from coordination_oru.coordinator.abstract_coordinator import (
        AbstractTrajectoryEnvelopeCoordinator,
    )


class AbstractTrajectoryEnvelopeTracker:
    def __init__(
        self,
        robot_id: int,
        envelope: TrajectoryEnvelope,
        coordinator: "AbstractTrajectoryEnvelopeCoordinator",
    ) -> None:
        self.robot_id = robot_id
        self.envelope = envelope
        self.coordinator = coordinator
        self._reports: asyncio.Queue[RobotReport] = asyncio.Queue()
        self._task: asyncio.Task[None] | None = None
        self._stopped = asyncio.Event()
        # default: the robot may proceed all the way to the goal until the
        # coordinator says otherwise.
        self.permit_index_until: int = envelope.length - 1
        self.log = get_logger(__name__, robot_id=robot_id, envelope_id=envelope.envelope_id)

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        if self._task is not None:
            raise RuntimeError("tracker already started")
        self._task = asyncio.create_task(self._run(), name=f"tracker-{self.robot_id}")

    async def stop(self) -> None:
        self._stopped.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    # ------------------------------------------------------------ reporting

    async def submit_report(self, report: RobotReport) -> None:
        await self._reports.put(report)

    # ------------------------------------------------------------ main loop

    async def _run(self) -> None:
        try:
            while not self._stopped.is_set():
                report = await self._reports.get()
                await self.coordinator.on_robot_report(report)
                await self._on_report(report)
                if report.completed:
                    return
        except asyncio.CancelledError:
            raise

    async def _on_report(self, report: RobotReport) -> None:
        """Hook for subclasses. The default implementation is a no-op."""
        del report
