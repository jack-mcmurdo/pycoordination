"""Shared runner for the example scripts.

``run(sim, scenario, ...)`` opens the pyglet viewer when pyglet is installed
(``pip install -e .[viz]``); otherwise it falls back to a headless run that
prints per-robot progress about once a second and a summary at the end.

Both modes drive the same asyncio coordinator and are bounded by
``run_until_idle(timeout=120.0)`` so a coordination bug can never hang the
terminal.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Awaitable, Callable

from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.logging import configure_logging

Scenario = Callable[[SimulationCoordinator], Awaitable[None]]

IDLE_TIMEOUT = 120.0


def run(
    sim: SimulationCoordinator,
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
        asyncio.run(_run_headless(sim, scenario, title=title))
        return
    _run_viz(
        sim,
        scenario,
        world_size=world_size,
        world_center=world_center,
        title=title,
        width=width,
        height=height,
    )


async def _run_headless(
    sim: SimulationCoordinator, scenario: Scenario, *, title: str
) -> None:
    await sim.start()
    try:
        await scenario(sim)
        progress = asyncio.create_task(_print_progress(sim))
        try:
            await sim.run_until_idle(timeout=IDLE_TIMEOUT)
        finally:
            progress.cancel()
        _print_summary(sim, title=title)
    finally:
        await sim.stop()


async def _print_progress(sim: SimulationCoordinator) -> None:
    while True:
        parts = []
        for robot_id, envelope in sorted(sim.envelopes_by_robot.items()):
            if envelope.completed:
                parts.append(f"robot {robot_id}: done")
                continue
            try:
                idx = sim.current_path_index(robot_id)
            except KeyError:
                continue
            total = len(envelope.spatial_envelope.footprints)
            parts.append(f"robot {robot_id}: {idx + 1}/{total}")
        if parts:
            print("  " + "   ".join(parts))
        await asyncio.sleep(1.0)


def _print_summary(sim: SimulationCoordinator, *, title: str) -> None:
    envelopes = sim.solver.all_envelopes()
    done = sum(1 for e in envelopes if e.completed)
    print(
        f"[{title}] finished: {done}/{len(envelopes)} envelopes completed, "
        f"{len(sim.critical_sections)} critical sections still open, "
        f"{len(sim.priorities)} priority decisions still held"
    )


def _run_viz(
    sim: SimulationCoordinator,
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
        await sim.start()
        try:
            await scenario(sim)
            await sim.run_until_idle(timeout=IDLE_TIMEOUT)
        finally:
            await sim.stop()

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
        sim,
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
