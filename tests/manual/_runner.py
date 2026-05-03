"""Shared runner for manual viz scripts.

Spins up an asyncio loop in a daemon thread, runs the supplied async
``setup`` coroutine to completion (the sim runs to idle), then opens a
pyglet window in the main thread that displays state read from the
coordinator.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Awaitable, Callable

from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.logging import configure_logging
from coordination_oru.viz.pyglet_viewer import PygletViewer


def run_viz(
    sim: SimulationCoordinator,
    scenario: Callable[[SimulationCoordinator], Awaitable[None]],
    *,
    world_size: float = 20.0,
    world_center: tuple[float, float] = (0.0, 0.0),
    title: str = "coordination_oru",
    width: int = 800,
    height: int = 800,
) -> None:
    configure_logging()

    async def driver() -> None:
        await sim.start()
        try:
            await scenario(sim)
            await sim.run_until_idle(timeout=120.0)
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
