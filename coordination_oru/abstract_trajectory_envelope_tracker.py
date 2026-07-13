"""``AbstractTrajectoryEnvelopeTracker``: base class for per-robot trackers.

Ported from Java's ``AbstractTrajectoryEnvelopeTracker``. The Java monitor
thread additionally manages ground-envelope (sub-envelope) dispatch and STP
deadlines for chained real-robot missions; this port targets flat,
single-envelope missions (as the rest of this codebase does), so that
bookkeeping is dropped — the causally-relevant behaviour (wait for tracking
to be enabled, fire lifecycle callbacks, detect path completion, fire
``onTrackingFinished``) is preserved. Java's ``Thread`` becomes an
``asyncio.Task``; ``synchronized`` sections that never straddle an ``await``
don't need a lock (asyncio is cooperative), so only genuinely shared,
cross-task mutable state (``communicatedCPs`` et al. on the coordinator)
uses ``asyncio.Lock``.
"""

from __future__ import annotations

import abc
import asyncio
import inspect
from typing import TYPE_CHECKING, Any

from coordination_oru.robot_report import RobotReport
from coordination_oru.tracking_callback import TrackingCallback

if TYPE_CHECKING:
    from coordination_oru.abstract_trajectory_envelope_coordinator import (
        AbstractTrajectoryEnvelopeCoordinator,
    )
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


class AbstractTrajectoryEnvelopeTracker(abc.ABC):
    def __init__(
        self,
        te: "TrajectoryEnvelope",
        temporalResolution: float,
        tec: "AbstractTrajectoryEnvelopeCoordinator",
        trackingPeriodInMillis: int,
        cb: TrackingCallback | None,
    ) -> None:
        self.te = te
        self.traj = te.getTrajectory()
        self.temporalResolution = temporalResolution
        self.externalCPCounter = -1
        self.reportCounter = -1
        self.criticalPoint = -1
        self.trackingPeriodInMillis = trackingPeriodInMillis
        self.cb = cb
        self.tec = tec
        self._calledOnTrackingStart = False
        self._calledStartTracking = False
        self._can_start_tracking = asyncio.Event()
        self.startingTimeInMillis = tec.getCurrentTimeInMillis()
        self._monitor_task: asyncio.Task[None] = asyncio.create_task(
            self._monitor_loop(), name=f"tracker-monitor-Robot{te.getRobotID()}"
        )

    # ------------------------------------------------------------- lifecycle

    def getTrackingPeriod(self) -> int:
        return self.trackingPeriodInMillis

    def getStartingTimeInMillis(self) -> int:
        return self.startingTimeInMillis

    def resetStartingTimeInMillis(self) -> None:
        self.startingTimeInMillis = self.tec.getCurrentTimeInMillis()

    @abc.abstractmethod
    def onTrajectoryEnvelopeUpdate(self) -> None: ...

    def updateTrajectoryEnvelope(self, te: "TrajectoryEnvelope") -> None:
        self.te = te
        if self.cb is not None:
            self.cb.updateTrajectoryEnvelope(te)
        self.traj = te.getTrajectory()
        self.onTrajectoryEnvelopeUpdate()

    def setCanStartTracking(self) -> None:
        self._can_start_tracking.set()

    def canStartTracking(self) -> bool:
        return self._can_start_tracking.is_set()

    async def waitUntilCanStartTracking(self) -> None:
        await self._can_start_tracking.wait()

    # ------------------------------------------------------------ critical point

    @abc.abstractmethod
    def setCriticalPoint(self, criticalPoint: int) -> None: ...

    def setCriticalPointWithCounter(self, criticalPointToSet: int, externalCPCounter: int) -> None:
        half_max = 2147483647 / 2.0
        stale = (
            externalCPCounter < self.externalCPCounter
            and externalCPCounter - self.externalCPCounter > half_max
        ) or (
            self.externalCPCounter > externalCPCounter
            and self.externalCPCounter - externalCPCounter < half_max
        )
        if stale:
            return
        self.setCriticalPoint(criticalPointToSet)
        self.externalCPCounter = externalCPCounter

    def setReportCounter(self, reportCounter: int) -> None:
        self.reportCounter = reportCounter

    def getReportCounter(self) -> int:
        return self.reportCounter

    def getCriticalPoint(self) -> int:
        return self.criticalPoint

    def getTrackingPeriodInMillis(self) -> int:
        return self.trackingPeriodInMillis

    def getLastRobotReport(self) -> RobotReport:
        return self.getRobotReport()

    @abc.abstractmethod
    def getRobotReport(self) -> RobotReport: ...

    def onPositionUpdate(self) -> list[str] | None:
        if self.cb is not None:
            return self.cb.onPositionUpdate()
        return None

    @abc.abstractmethod
    def getCurrentTimeInMillis(self) -> int: ...

    @abc.abstractmethod
    def startTracking(self) -> None: ...

    def trackingStarted(self) -> bool:
        return self._calledStartTracking

    def getTrajectoryEnvelope(self) -> "TrajectoryEnvelope":
        return self.te

    # ------------------------------------------------------------ monitoring

    async def _call_maybe_async(self, fn: Any) -> Any:
        result = fn()
        if inspect.isawaitable(result):
            return await result
        return result

    async def _monitor_loop(self) -> None:
        if self.cb is not None:
            await self._call_maybe_async(self.cb.beforeTrackingStart)

        if self.cb is not None and not self._calledOnTrackingStart:
            self._calledOnTrackingStart = True
            self.cb.onTrackingStart()

        if not self._calledStartTracking:
            self._calledStartTracking = True
            self.startTracking()

        prev_seq_number = -1
        while True:
            rr = self.tec.getRobotReport(self.te.getRobotID())
            while rr is None:
                await asyncio.sleep(0.1)
                rr = self.tec.getRobotReport(self.te.getRobotID())
            current_seq_number = rr.getPathIndex()

            if self.te.getSequenceNumberEnd() == current_seq_number or (
                current_seq_number < prev_seq_number and current_seq_number <= 0
            ):
                break
            prev_seq_number = current_seq_number
            await asyncio.sleep(self.trackingPeriodInMillis / 1000.0)

        if self.cb is not None:
            self.cb.beforeTrackingFinished()
        self.finishTracking()
        if self.cb is not None:
            self.cb.onTrackingFinished()

    def finishTracking(self) -> None:
        pass

    def cancel(self) -> None:
        self._monitor_task.cancel()
