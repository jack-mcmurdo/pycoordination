"""In-process simulator built on top of the abstract coordinator.

Drives a fleet by calling :meth:`SimulationTracker.advance` every
``sim_step_period`` seconds and then submitting the resulting
:class:`RobotReport` back to the tracker (so the coordinator's
report-handling path runs end-to-end).

Two cooperating asyncio tasks:

1. The coordination loop (inherited from ``AbstractTrajectoryEnvelopeCoordinator``).
2. The simulation step loop that ticks every tracker.

Per-robot motion state lives on the tracker (``path_index``, plus any
extra dynamics state in subclasses). The coordinator holds no robot state
beyond the registry.
"""

from __future__ import annotations

import asyncio
from typing import Callable

from coordination_oru.coordinator.abstract_coordinator import (
    COORDINATION_PERIOD,
    AbstractTrajectoryEnvelopeCoordinator,
)
from coordination_oru.coordinator.abstract_tracker import (
    AbstractTrajectoryEnvelopeTracker,
)
from coordination_oru.coordinator.mission import Mission
from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
from coordination_oru.simulation.sim_tracker import SimulationTracker


TrackerFactory = Callable[
    [int, TrajectoryEnvelope, "SimulationCoordinator"],
    AbstractTrajectoryEnvelopeTracker,
]


class SimulationCoordinator(AbstractTrajectoryEnvelopeCoordinator):
    def __init__(
        self,
        period: float = COORDINATION_PERIOD,
        sim_step_period: float = 0.02,
    ) -> None:
        super().__init__(period=period)
        self.sim_step_period = sim_step_period
        self._sim_task: asyncio.Task[None] | None = None
        self._sim_clock: float = 0.0

    # ------------------------------------------------------------ lifecycle

    async def start(self) -> None:
        await super().start()
        self._sim_task = asyncio.create_task(self._sim_loop(), name="sim-loop")

    async def stop(self) -> None:
        if self._sim_task is not None:
            self._sim_task.cancel()
            try:
                await self._sim_task
            except asyncio.CancelledError:
                pass
            self._sim_task = None
        for tracker in list(self._trackers.values()):
            await tracker.stop()
        await super().stop()

    # ------------------------------------------------------------ wiring

    def add_robot(
        self,
        mission: Mission,
        tracker_factory: TrackerFactory | None = None,
    ) -> TrajectoryEnvelope:
        """Submit a mission and start a fresh tracker for the robot.

        ``tracker_factory`` defaults to the discrete waypoint-hop
        :class:`SimulationTracker`. Pass a different factory (e.g. one that
        builds an ``RK4SimulationTracker``) to swap motion models per robot.
        """
        prev_tracker = self._trackers.pop(mission.robot_id, None)
        if prev_tracker is not None:
            asyncio.create_task(prev_tracker.stop(), name=f"stop-prev-{mission.robot_id}")

        envelope = self.submit_mission(mission)
        tracker: AbstractTrajectoryEnvelopeTracker
        if tracker_factory is None:
            tracker = SimulationTracker(
                robot_id=mission.robot_id, envelope=envelope, coordinator=self
            )
        else:
            tracker = tracker_factory(mission.robot_id, envelope, self)
        self.register_tracker(tracker)
        tracker.start()
        return envelope

    def add_rk4_robot(
        self,
        mission: Mission,
        *,
        v_max: float = 1.0,
        a_max: float = 0.5,
    ) -> TrajectoryEnvelope:
        """Like :meth:`add_robot` but uses an RK4-integrated kinematic tracker."""
        from coordination_oru.simulation.rk4_tracker import RK4SimulationTracker

        return self.add_robot(
            mission,
            tracker_factory=lambda robot_id, envelope, coord: RK4SimulationTracker(
                robot_id=robot_id,
                envelope=envelope,
                coordinator=coord,
                v_max=v_max,
                a_max=a_max,
            ),
        )

    # ------------------------------------------------------------ snapshots

    @property
    def trackers(self) -> dict[int, AbstractTrajectoryEnvelopeTracker]:
        return dict(self._trackers)

    @property
    def envelopes_by_robot(self) -> dict[int, TrajectoryEnvelope]:
        return dict(self._envelopes)

    # ------------------------------------------------------------ sim loop

    async def _sim_loop(self) -> None:
        try:
            while self._running:
                await self._sim_step()
                await asyncio.sleep(self.sim_step_period)
                self._sim_clock += self.sim_step_period
        except asyncio.CancelledError:
            raise

    async def _sim_step(self) -> None:
        items = list(self._trackers.items())
        for robot_id, tracker in items:
            envelope = self._envelopes.get(robot_id)
            if envelope is None or envelope.completed:
                continue
            advance = getattr(tracker, "advance", None)
            if advance is None:
                continue
            report = advance(self.sim_step_period, self._sim_clock)
            await tracker.submit_report(report)

    # ----------------------------------------------------- test helpers

    async def run_until_idle(self, timeout: float = 30.0) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            active = [e for e in self._envelopes.values() if not e.completed]
            if not active:
                return
            if loop.time() > deadline:
                raise TimeoutError(
                    f"simulation did not complete in {timeout}s; still active: "
                    f"{[e.envelope_id for e in active]}"
                )
            await asyncio.sleep(self.sim_step_period)

    def current_path_index(self, robot_id: int) -> int:
        tracker = self._trackers.get(robot_id)
        if tracker is None:
            raise KeyError(robot_id)
        idx = getattr(tracker, "path_index", None)
        if idx is None:
            raise AttributeError(f"tracker for robot {robot_id} has no path_index")
        return int(idx)
