"""Shared runner for the example scripts.

``run(tec, scenario, ...)`` opens the pyglet viewer when pyglet is installed
(``pip install -e .[viz]``); otherwise it falls back to a headless run that
prints per-robot progress about once a second and a summary at the end.

Both modes drive the same asyncio coordinator and are bounded by
``wait_until_idle(timeout=120.0)`` so a coordination bug can never hang the
terminal.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Awaitable, Callable

from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)
from coordination_oru.util.logging import configure_logging

Scenario = Callable[[TrajectoryEnvelopeCoordinatorSimulation], Awaitable[None]]

IDLE_TIMEOUT = 120.0


async def wait_until_idle(tec: TrajectoryEnvelopeCoordinatorSimulation, timeout: float = IDLE_TIMEOUT) -> None:
    """Block until every known robot is parked (not driving)."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        active = [robotID for robotID in tec.trackers if tec.isDrivingRobot(robotID)]
        if not active and tec.trackers:
            return
        if loop.time() > deadline:
            raise TimeoutError(f"simulation did not complete in {timeout}s; still active: {active}")
        await asyncio.sleep(0.02)


def run(
    tec: TrajectoryEnvelopeCoordinatorSimulation,
    scenario: Scenario,
    *,
    world_size: float = 20.0,
    world_center: tuple[float, float] = (0.0, 0.0),
    title: str = "coordination_oru",
    width: int = 800,
    height: int = 800,
) -> None:
    """Run a scenario with the pyglet viewer, or headless if pyglet is absent."""
    configure_logging()
    try:
        import pyglet  # noqa: F401
    except ImportError:
        print(
            f"[{title}] pyglet not installed — running headless. "
            "For the animated viewer: pip install -e .[viz]"
        )
        asyncio.run(_run_headless(tec, scenario, title=title))
        return
    _run_viz(
        tec,
        scenario,
        world_size=world_size,
        world_center=world_center,
        title=title,
        width=width,
        height=height,
    )


async def _run_headless(tec: TrajectoryEnvelopeCoordinatorSimulation, scenario: Scenario, *, title: str) -> None:
    await tec.startInference()
    try:
        await scenario(tec)
        progress = asyncio.create_task(_print_progress(tec))
        try:
            await wait_until_idle(tec, IDLE_TIMEOUT)
        finally:
            progress.cancel()
        _print_summary(tec, title=title)
    finally:
        await tec.stopInference()


async def _print_progress(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    while True:
        parts = []
        for robotID, tracker in sorted(tec.trackers.items()):
            if isinstance(tracker, TrajectoryEnvelopeTrackerDummy):
                parts.append(f"robot {robotID}: parked")
                continue
            rr = tracker.getRobotReport()
            te = tracker.getTrajectoryEnvelope()
            parts.append(f"robot {robotID}: {rr.getPathIndex() + 1}/{te.getPathLength()}")
        if parts:
            print("  " + "   ".join(parts))
        await asyncio.sleep(1.0)


def _print_summary(tec: TrajectoryEnvelopeCoordinatorSimulation, *, title: str) -> None:
    print(
        f"[{title}] finished: all robots parked, "
        f"{len(tec.allCriticalSections)} critical sections still open, "
        f"{len(tec.CSToDepsOrder)} precedence decisions still held"
    )


def _run_viz(
    tec: TrajectoryEnvelopeCoordinatorSimulation,
    scenario: Scenario,
    *,
    world_size: float,
    world_center: tuple[float, float],
    title: str,
    width: int,
    height: int,
) -> None:
    from coordination_oru.viz.pyglet_viewer import PygletViewer

    async def driver() -> None:
        await tec.startInference()
        try:
            await scenario(tec)
            await wait_until_idle(tec, IDLE_TIMEOUT)
        finally:
            await tec.stopInference()

    loop = asyncio.new_event_loop()

    def thread_target() -> None:
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(driver())
        finally:
            loop.close()

    thread = threading.Thread(target=thread_target, daemon=True)
    thread.start()

    viewer = PygletViewer(
        tec,
        world_size=world_size,
        world_center=world_center,
        title=title,
        width=width,
        height=height,
    )
    viewer.stop_when_idle()
    try:
        viewer.run()
    finally:
        thread.join(timeout=2.0)
