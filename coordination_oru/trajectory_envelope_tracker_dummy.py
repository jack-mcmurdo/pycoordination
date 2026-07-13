"""``TrajectoryEnvelopeTrackerDummy``: represents a robot parked in a pose."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from coordination_oru.abstract_trajectory_envelope_tracker import (
    AbstractTrajectoryEnvelopeTracker,
)
from coordination_oru.robot_report import RobotReport
from coordination_oru.tracking_callback import TrackingCallback

if TYPE_CHECKING:
    from coordination_oru.abstract_trajectory_envelope_coordinator import (
        AbstractTrajectoryEnvelopeCoordinator,
    )
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


class TrajectoryEnvelopeTrackerDummy(AbstractTrajectoryEnvelopeTracker):
    def __init__(
        self,
        te: "TrajectoryEnvelope",
        timeStep: int,
        temporalResolution: float,
        tec: "AbstractTrajectoryEnvelopeCoordinator",
        cb: TrackingCallback | None,
    ) -> None:
        self._parkingFinished = False
        self._currentIndex = -1
        self._finish_event = asyncio.Event()
        super().__init__(te, temporalResolution, tec, timeStep, cb)
        self._run_task: asyncio.Task[None] = asyncio.create_task(
            self.run(), name=f"parking-Robot{te.getRobotID()}"
        )

    def onTrajectoryEnvelopeUpdate(self) -> None:
        pass

    def startTracking(self) -> None:
        pass

    def setCriticalPoint(self, criticalPoint: int) -> None:
        pass

    def getRobotReport(self) -> RobotReport:
        return RobotReport(
            self.te.getRobotID(), self.te.getTrajectory().getPose()[0], self._currentIndex, 0.0, 0.0, -1
        )

    def finishParking(self) -> None:
        self._parkingFinished = True
        self._finish_event.set()

    def isParkingFinished(self) -> bool:
        return self._parkingFinished

    async def run(self) -> None:
        await self._finish_event.wait()
        self._currentIndex += 1

    def onPositionUpdate(self) -> list[str] | None:
        return None

    def getCurrentTimeInMillis(self) -> int:
        return self.tec.getCurrentTimeInMillis()
