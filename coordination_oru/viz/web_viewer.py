"""Browser-based live viewer for a running
:class:`TrajectoryEnvelopeCoordinatorSimulation`.

A small starlette + uvicorn server serves the prebuilt Vite/React frontend
(shipped inside the wheel at ``coordination_oru/viz/static/``) and streams
coordinator state over a ``/ws`` websocket. Like :class:`PygletViewer` it is
a pure polling observer — it reads public coordinator state and never calls
back into the core.

Wire protocol (all messages carry ``seq``, monotonic int, and ``ts``, unix
ms):

- ``{"kind": "static", "title", "world": {"size", "center"}, "robots":
  [{"id", "envelopeID", "path": [[x, y], ...], "envelope": [ring, ...]}],
  "footprints": [{"id", "ring"}], "interactive": bool}`` — per-robot path
  polyline and swept-envelope polygon rings for every *driving* robot, plus
  each known robot's footprint outline centered at the origin. Sent on
  client connect and whenever the set of robots or driving envelopes
  changes (missions start/finish). Paths are the heavy payload, so they
  are only re-sent on change. When the viewer has an occupancy map, the
  message also carries ``"map": {"dataUri", "resolution", "origin":
  [x, y], "width", "height"}`` (a base64 PNG data URI plus the world-frame
  placement); ``interactive`` tells the frontend to enable the
  goal-posting UX.
- ``{"kind": "state", "robots": [{"id", "driving", "pose": [x, y, theta],
  "pathIndex", "pathLength", "velocity", "criticalPoint"}],
  "criticalSections": [{"robot1", "start1", "end1", "robot2", "start2",
  "end2"}], "dependencies": [{"waiting", "driving", "waitingPoint"}],
  "counts": {"driving", "parked", "criticalSections", "orders"},
  "deadlocked": bool}`` — sent every poll tick (``poll_hz``). The frontend
  places the static footprint outline at ``pose`` (translate+rotate, cheap
  enough to CSS-animate); critical sections reference path indices into
  the static paths, the frontend slices the highlight segments from those;
  ``dependencies`` are the current yielder → leader precedence orders;
  ``deadlocked`` is true while the coordinator sees a nonlive cycle whose
  robots are all stopped at their critical points.

Inbound messages: ``{"kind": "postGoal", "robot": int, "goal": [x, y,
theta]}`` — a goal pose for a robot, dispatched to the ``on_goal``
callback when one is configured. Malformed or unknown inbound messages
are silently ignored.

The server runs *inside* the simulation's asyncio event loop (the
coordinator is asyncio-native, so no thread bridge is needed): create the
viewer, then ``await viewer.serve()`` alongside the sim driver.
"""

from __future__ import annotations

import asyncio
import base64
import importlib.resources
import json
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any, AsyncIterator, Awaitable, Callable

import uvicorn
from starlette.applications import Starlette
from starlette.routing import WebSocketRoute
from starlette.staticfiles import StaticFiles
from starlette.websockets import WebSocket, WebSocketDisconnect

from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)

if TYPE_CHECKING:
    from shapely.geometry.base import BaseGeometry

    from coordination_oru.abstract_trajectory_envelope_coordinator import (
        AbstractTrajectoryEnvelopeCoordinator,
    )
    from coordination_oru.motionplanning.occupancy_map import OccupancyMap

__all__ = ["WebViewer", "build_static_message", "build_state_message"]


def _now_ms() -> int:
    return int(time.time() * 1000)


def _static_dir() -> Path | None:
    """The frontend build directory, if one has been built into the
    installed package (see the npm build step in
    ``.github/workflows/deploy.yml``)."""
    files = importlib.resources.files("coordination_oru.viz") / "static"
    path = Path(str(files))
    return path if (path / "index.html").is_file() else None


STATIC_MISSING_MESSAGE = (
    "coordination_oru/viz/static/ is missing an index.html (this looks like "
    "a source checkout without a frontend build). Build it with:\n"
    "    npm --prefix frontend install && npm --prefix frontend run build"
)


def _rings(geometry: "BaseGeometry") -> list[list[list[float]]]:
    """Exterior rings of a (Multi)Polygon as ``[[x, y], ...]`` lists,
    coordinates rounded to 3 decimals (mm resolution) to slim the JSON."""
    geoms = getattr(geometry, "geoms", [geometry])
    rings: list[list[list[float]]] = []
    for geom in geoms:
        exterior = getattr(geom, "exterior", None)
        if exterior is None:
            continue
        ring = [[round(x, 3), round(y, 3)] for x, y in exterior.coords[:-1]]
        if len(ring) >= 3:
            rings.append(ring)
    return rings


def driving_envelope_ids(
    coordinator: "AbstractTrajectoryEnvelopeCoordinator",
) -> dict[int, int]:
    """robotID → envelope ID for every robot currently driving a mission."""
    return {
        robotID: tracker.getTrajectoryEnvelope().getID()
        for robotID, tracker in dict(coordinator.trackers).items()
        if not isinstance(tracker, TrajectoryEnvelopeTrackerDummy)
    }


def static_content_key(
    coordinator: "AbstractTrajectoryEnvelopeCoordinator",
) -> tuple[frozenset[int], tuple[tuple[int, int], ...]]:
    """Changes exactly when the static message content would: a robot
    appears, or the set of driving envelopes changes."""
    return (
        frozenset(coordinator.trackers),
        tuple(sorted(driving_envelope_ids(coordinator).items())),
    )


def build_static_message(
    coordinator: "AbstractTrajectoryEnvelopeCoordinator",
    *,
    title: str = "coordination_oru",
    world_size: float = 20.0,
    world_center: tuple[float, float] = (0.0, 0.0),
    occupancy_map: "OccupancyMap | None" = None,
    map_data_uri: str | None = None,
    interactive: bool = False,
) -> dict[str, Any]:
    """The per-mission payload: paths and swept envelopes of driving robots."""
    robots = []
    for robotID, tracker in sorted(dict(coordinator.trackers).items()):
        if isinstance(tracker, TrajectoryEnvelopeTrackerDummy):
            continue
        e = tracker.getTrajectoryEnvelope()
        robots.append(
            {
                "id": robotID,
                "envelopeID": e.getID(),
                "path": [
                    [round(ps.pose.x, 3), round(ps.pose.y, 3)] for ps in e.path
                ],
                "envelope": _rings(e.getSpatialEnvelope().getPolygon()),
            }
        )
    footprints = []
    for robotID in sorted(dict(coordinator.trackers)):
        outline = coordinator.getFootprint(robotID)
        if outline is None:
            continue
        footprints.append(
            {
                "id": robotID,
                "ring": [
                    [round(x, 3), round(y, 3)] for x, y in outline.exterior.coords[:-1]
                ],
            }
        )
    message: dict[str, Any] = {
        "kind": "static",
        "title": title,
        "world": {"size": world_size, "center": list(world_center)},
        "robots": robots,
        "footprints": footprints,
        "interactive": interactive,
    }
    if occupancy_map is not None and map_data_uri is not None:
        message["map"] = {
            "dataUri": map_data_uri,
            "resolution": occupancy_map.resolution,
            "origin": [occupancy_map.origin[0], occupancy_map.origin[1]],
            "width": occupancy_map.width,
            "height": occupancy_map.height,
        }
    return message


def build_state_message(
    coordinator: "AbstractTrajectoryEnvelopeCoordinator",
) -> dict[str, Any]:
    """The per-tick payload: placed footprints, reports, critical sections."""
    trackers = dict(coordinator.trackers)
    css = list(coordinator.allCriticalSections)
    driving = {
        robotID
        for robotID, tracker in trackers.items()
        if not isinstance(tracker, TrajectoryEnvelopeTrackerDummy)
    }

    robots = []
    for robotID, tracker in sorted(trackers.items()):
        rr = tracker.getRobotReport()
        pose = rr.getPose() if rr is not None else None
        if pose is None:
            continue
        entry: dict[str, Any] = {
            "id": robotID,
            "driving": robotID in driving,
            "pose": [round(pose.getX(), 4), round(pose.getY(), 4), round(pose.getTheta(), 4)],
            "pathIndex": rr.getPathIndex(),
            "velocity": round(rr.getVelocity(), 3),
            "criticalPoint": rr.getCriticalPoint(),
        }
        if robotID in driving:
            entry["pathLength"] = tracker.getTrajectoryEnvelope().getPathLength()
        robots.append(entry)

    dependencies = [
        {
            "waiting": dep.getWaitingRobotID(),
            "driving": dep.getDrivingRobotID(),
            "waitingPoint": dep.getWaitingPoint(),
        }
        for dep in coordinator.getCurrentDependencies().values()
        if dep.getDrivingRobotID() != 0  # 0 = stopping-point-only dependency
    ]

    sections = []
    for cs in css:
        te1, te2 = cs.getTe1(), cs.getTe2()
        if te1 is None or te2 is None:
            continue
        sections.append(
            {
                "robot1": te1.getRobotID(),
                "start1": max(0, cs.getTe1Start()),
                "end1": min(cs.getTe1End(), te1.getPathLength() - 1),
                "robot2": te2.getRobotID(),
                "start2": max(0, cs.getTe2Start()),
                "end2": min(cs.getTe2End(), te2.getPathLength() - 1),
            }
        )

    # side-effect-free deadlock probe (TrajectoryEnvelopeCoordinator only;
    # other coordinator subclasses simply report False)
    compute_deadlocked = getattr(coordinator, "computeIsDeadlocked", None)
    deadlocked = bool(compute_deadlocked()) if callable(compute_deadlocked) else False

    return {
        "kind": "state",
        "robots": robots,
        "criticalSections": sections,
        "dependencies": dependencies,
        "counts": {
            "driving": len(driving),
            "parked": len(trackers) - len(driving),
            "criticalSections": len(css),
            "orders": len(coordinator.CSToDepsOrder),
        },
        "deadlocked": deadlocked,
    }


class WebViewer:
    """Read-only browser viewer; runs in the simulation's asyncio loop.

    Usage::

        viewer = WebViewer(tec, world_size=14.0)
        server_task = asyncio.create_task(viewer.serve())
        ...  # drive the sim in the same loop
        await server_task  # serves until Ctrl+C or viewer.request_stop()
    """

    def __init__(
        self,
        coordinator: "AbstractTrajectoryEnvelopeCoordinator",
        *,
        host: str = "127.0.0.1",
        port: int = 8723,
        poll_hz: float = 20.0,
        world_size: float = 20.0,
        world_center: tuple[float, float] = (0.0, 0.0),
        title: str = "coordination_oru",
        map: "OccupancyMap | None" = None,
        on_goal: "Callable[[int, float, float, float], Awaitable[None]] | None" = None,
    ) -> None:
        self.coordinator = coordinator
        self.host = host
        self.port = port
        self.poll_hz = poll_hz
        self.world_size = world_size
        self.world_center = world_center
        self.title = title
        self.map = map
        self.on_goal = on_goal
        self._map_data_uri: str | None = None
        if map is not None:
            self._map_data_uri = "data:image/png;base64," + base64.b64encode(
                map.to_png_bytes()
            ).decode("ascii")

        self._seq = 0
        self._clients: set[WebSocket] = set()
        self._known_static_key: object = None
        self._poll_task: asyncio.Task[None] | None = None
        self._uvicorn_server: uvicorn.Server | None = None

        self.app = self._build_app()

    # --------------------------------------------------------------- app

    def _build_app(self) -> Starlette:
        app = Starlette(
            routes=[WebSocketRoute("/ws", self._ws_endpoint)],
            lifespan=self._lifespan,
        )
        static_dir = _static_dir()
        if static_dir is not None:
            app.mount("/", StaticFiles(directory=str(static_dir), html=True), name="static")
        return app

    @asynccontextmanager
    async def _lifespan(self, app: Starlette) -> AsyncIterator[None]:
        self._poll_task = asyncio.get_running_loop().create_task(self._poll_loop())
        try:
            yield
        finally:
            self._poll_task.cancel()

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _stamp(self, message: dict[str, Any]) -> dict[str, Any]:
        message["seq"] = self._next_seq()
        message["ts"] = _now_ms()
        return message

    def _static_message(self) -> dict[str, Any]:
        return self._stamp(
            build_static_message(
                self.coordinator,
                title=self.title,
                world_size=self.world_size,
                world_center=self.world_center,
                occupancy_map=self.map,
                map_data_uri=self._map_data_uri,
                interactive=self.on_goal is not None,
            )
        )

    # --------------------------------------------------------- websocket

    async def _ws_endpoint(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._clients.add(websocket)
        try:
            await websocket.send_json(self._static_message())
            await websocket.send_json(self._stamp(build_state_message(self.coordinator)))
            while True:
                raw = await websocket.receive_text()
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if not isinstance(msg, dict):
                    continue
                goal = msg.get("goal")
                if (
                    msg.get("kind") == "postGoal"
                    and self.on_goal is not None
                    and isinstance(msg.get("robot"), int)
                    and isinstance(goal, list)
                    and len(goal) == 3
                    and all(isinstance(v, (int, float)) for v in goal)
                ):
                    await self.on_goal(msg["robot"], float(goal[0]), float(goal[1]), float(goal[2]))
        except WebSocketDisconnect:
            pass
        finally:
            self._clients.discard(websocket)

    async def _broadcast(self, message: dict[str, Any]) -> None:
        for client in list(self._clients):
            try:
                await client.send_json(message)
            except Exception:
                self._clients.discard(client)

    async def _poll_loop(self) -> None:
        period = 1.0 / self.poll_hz
        while True:
            await asyncio.sleep(period)
            if not self._clients:
                continue
            key = static_content_key(self.coordinator)
            if key != self._known_static_key:
                self._known_static_key = key
                await self._broadcast(self._static_message())
            await self._broadcast(self._stamp(build_state_message(self.coordinator)))

    # --------------------------------------------------------- lifecycle

    async def serve(self) -> None:
        """Serve until Ctrl+C (SIGINT/SIGTERM) or :meth:`request_stop`.

        Raises :class:`RuntimeError` when the frontend build is missing —
        i.e. a source checkout where ``npm run build`` has not been run.
        """
        if _static_dir() is None:
            raise RuntimeError(STATIC_MISSING_MESSAGE)
        config = uvicorn.Config(self.app, host=self.host, port=self.port, log_level="warning")
        self._uvicorn_server = uvicorn.Server(config)
        print(f"[{self.title}] web viewer on http://{self.host}:{self.port}/ (Ctrl+C to exit)")
        try:
            await self._uvicorn_server.serve()
        except SystemExit as exc:
            # uvicorn sys.exit(1)s on startup failure (e.g. port in use); as a
            # task exception SystemExit would tear through the event loop
            # before callers can inspect the task, so convert it here.
            raise RuntimeError(
                f"web viewer failed to start on {self.host}:{self.port} "
                f"(port already in use?)"
            ) from exc

    def request_stop(self) -> None:
        """Ask the running server to shut down gracefully."""
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True
