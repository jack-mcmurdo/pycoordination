"""Shared runner for the example scripts and the installed demo.

``run(tec, scenario, ...)`` picks a viewer from the command line — every
example accepts the same flags without declaring them itself:

    python examples/two_robots.py                # pyglet if installed, else headless
    python examples/two_robots.py --web-viewer   # browser viewer (starlette+uvicorn)
    python examples/two_robots.py --pyglet       # force pyglet
    python examples/two_robots.py --headless     # force headless

``default_viewer`` sets the mode used when no flag is given: the examples
keep the historical ``"auto"`` (pyglet if installed, else headless), while
the ``coordination-oru-demo`` console script defaults to ``"web"``.

All modes drive the same asyncio coordinator and the mission phase is
bounded by ``wait_until_idle(timeout=120.0)`` so a coordination bug can
never hang the terminal. The web viewer keeps serving the finished state
until Ctrl+C.
"""

from __future__ import annotations

import argparse
import asyncio
import threading
import webbrowser
from typing import TYPE_CHECKING, Awaitable, Callable

from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)

if TYPE_CHECKING:
    from coordination_oru.motionplanning.occupancy_map import OccupancyMap
from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)
from coordination_oru.util.logging import configure_logging

Scenario = Callable[[TrajectoryEnvelopeCoordinatorSimulation], Awaitable[None]]
OnGoal = Callable[[int, float, float, float], Awaitable[None]]

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


def _parse_args(title: str, default_viewer: str) -> argparse.Namespace:
    epilog = (
        "With no viewer flag: the browser-based web viewer."
        if default_viewer == "web"
        else "With no viewer flag: pyglet if installed, otherwise headless."
    )
    parser = argparse.ArgumentParser(
        description=f"{title} — coordination_oru example",
        epilog=epilog,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--web-viewer",
        action="store_const",
        dest="viewer",
        const="web",
        help="serve a browser-based viewer (requires starlette+uvicorn and a frontend build)",
    )
    mode.add_argument(
        "--pyglet",
        action="store_const",
        dest="viewer",
        const="pyglet",
        help="force the pyglet window",
    )
    mode.add_argument(
        "--headless",
        action="store_const",
        dest="viewer",
        const="headless",
        help="force the headless text run",
    )
    parser.set_defaults(viewer=default_viewer)
    parser.add_argument(
        "--port", type=int, default=8723, help="web viewer port (default: 8723)"
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="web viewer: do not open the system browser",
    )
    return parser.parse_args()


def run(
    tec: TrajectoryEnvelopeCoordinatorSimulation,
    scenario: Scenario,
    *,
    world_size: float = 20.0,
    world_center: tuple[float, float] = (0.0, 0.0),
    title: str = "coordination_oru",
    width: int = 800,
    height: int = 800,
    occupancy_map: "OccupancyMap | None" = None,
    on_goal: OnGoal | None = None,
    interactive: bool = False,
    default_viewer: str = "auto",
) -> None:
    """Run a scenario with the viewer selected on the command line."""
    configure_logging()
    args = _parse_args(title, default_viewer)

    viewer = args.viewer
    if viewer == "auto":
        try:
            import pyglet  # noqa: F401

            viewer = "pyglet"
        except ImportError:
            print(
                f"[{title}] pyglet not importable — running headless. "
                "pyglet ships with the package; try --web-viewer for the "
                "browser viewer instead"
            )
            viewer = "headless"

    if viewer != "web" and interactive:
        print(f"[{title}] interactive goal posting needs --web-viewer; running the scripted scenario")

    if viewer == "headless":
        asyncio.run(_run_headless(tec, scenario, title=title))
    elif viewer == "web":
        _run_web(
            tec,
            scenario,
            world_size=world_size,
            world_center=world_center,
            title=title,
            port=args.port,
            open_browser=not args.no_browser,
            occupancy_map=occupancy_map,
            on_goal=on_goal,
            interactive=interactive,
        )
    else:
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


def _run_web(
    tec: TrajectoryEnvelopeCoordinatorSimulation,
    scenario: Scenario,
    *,
    world_size: float,
    world_center: tuple[float, float],
    title: str,
    port: int,
    open_browser: bool,
    occupancy_map: "OccupancyMap | None" = None,
    on_goal: OnGoal | None = None,
    interactive: bool = False,
) -> None:
    """Sim and websocket server share one asyncio loop — no threads."""
    try:
        from coordination_oru.viz.web_viewer import WebViewer
    except ImportError as exc:
        raise SystemExit(
            f"[{title}] web viewer import failed ({exc}) — "
            "reinstall the package: pip install --force-reinstall coordination-oru"
        ) from None

    async def main() -> None:
        viewer = WebViewer(
            tec,
            port=port,
            world_size=world_size,
            world_center=world_center,
            title=title,
            map=occupancy_map,
            on_goal=on_goal,
        )
        server_task = asyncio.create_task(viewer.serve())
        # fail fast on a missing frontend build or an occupied port
        await asyncio.wait({server_task}, timeout=0.5)
        if server_task.done():
            server_task.result()
            return
        if open_browser:
            webbrowser.open(f"http://127.0.0.1:{port}/")

        await tec.startInference()
        try:
            await scenario(tec)
            if interactive:
                print("select a robot, then press-drag-release to post a goal pose (Ctrl+C to exit)")
                await server_task  # inference keeps running for posted goals
                return
            await wait_until_idle(tec, IDLE_TIMEOUT)
        finally:
            await tec.stopInference()
        _print_summary(tec, title=title)
        await server_task  # keep serving the finished state until Ctrl+C

    try:
        asyncio.run(main())
    except RuntimeError as exc:
        # missing frontend build or occupied port — one clean line, no traceback
        raise SystemExit(f"[{title}] {exc}") from None
