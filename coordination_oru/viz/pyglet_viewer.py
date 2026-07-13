"""Pyglet-based visualisation for a running :class:`TrajectoryEnvelopeCoordinatorSimulation`.

The viewer takes a *snapshot* of coordinator state on every draw frame
(60 Hz by default) and re-creates a small set of pyglet ``shapes`` from
it. State is read without locking — the brief race against the asyncio
sim loop is harmless for visualisation purposes (worst case: one frame's
worth of jitter).

Usage pattern (run sim in a daemon thread, viewer in the main thread):

.. code-block:: python

    tec = TrajectoryEnvelopeCoordinatorSimulation(...)

    async def run_sim():
        await tec.startInference()
        tec.addMissions(...)
        ...

    loop = asyncio.new_event_loop()
    threading.Thread(
        target=lambda: (asyncio.set_event_loop(loop), loop.run_until_complete(run_sim())),
        daemon=True,
    ).start()

    viewer = PygletViewer(tec, world_size=15.0)
    viewer.run()

Layers drawn (back to front):

1. Path polylines (faint, per-robot colour).
2. Swept-envelope outlines (very faint fill).
3. Critical-section index ranges highlighted in red on each path.
4. Current footprints filled in robot colour.
5. Status text (top-left).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pyglet
from pyglet import shapes
from shapely.geometry import MultiPolygon, Polygon

from coordination_oru.trajectory_envelope_tracker_dummy import (
    TrajectoryEnvelopeTrackerDummy,
)
from coordination_oru.util.geometry import place_footprint

if TYPE_CHECKING:
    from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
        TrajectoryEnvelopeCoordinatorSimulation,
    )


ROBOT_COLORS: tuple[tuple[int, int, int], ...] = (
    (255, 99, 71),    # tomato
    (51, 168, 255),   # sky blue
    (255, 195, 51),   # gold
    (130, 255, 51),   # lime
    (200, 51, 255),   # violet
    (51, 255, 200),   # mint
)

CS_HIGHLIGHT: tuple[int, int, int, int] = (240, 90, 90, 200)
PATH_COLOR: tuple[int, int, int, int] = (180, 180, 180, 120)
SWEPT_OPACITY: int = 28
TEXT_COLOR: tuple[int, int, int, int] = (220, 220, 220, 255)
LABEL_COLOR: tuple[int, int, int, int] = (10, 10, 10, 255)
BACKGROUND: tuple[int, int, int, int] = (24, 24, 28, 255)


class PygletViewer:
    """Read-only viewer for a :class:`TrajectoryEnvelopeCoordinatorSimulation`."""

    def __init__(
        self,
        coordinator: "TrajectoryEnvelopeCoordinatorSimulation",
        *,
        width: int = 800,
        height: int = 800,
        world_size: float = 20.0,
        world_center: tuple[float, float] = (0.0, 0.0),
        title: str = "coordination_oru",
        draw_swept_envelope: bool = True,
    ) -> None:
        self.coordinator = coordinator
        self.world_size = world_size
        self.world_center = world_center
        self.draw_swept_envelope = draw_swept_envelope
        self.window = pyglet.window.Window(width=width, height=height, caption=title)
        self.batch = pyglet.graphics.Batch()
        self._shapes: list[object] = []
        self._labels: list[pyglet.text.Label] = []
        self._stop_when_idle = False
        self._has_been_active = False

        self.window.push_handlers(on_draw=self._on_draw)
        pyglet.gl.glClearColor(*[c / 255.0 for c in BACKGROUND])

    # ------------------------------------------------------------ controls

    def stop_when_idle(self) -> None:
        """Auto-close the window once every robot is parked (idle)."""
        self._stop_when_idle = True

    def run(self) -> None:
        # one heartbeat callback so pyglet doesn't go fully idle when the sim
        # thread is the only thing changing state
        pyglet.clock.schedule_interval(lambda dt: None, 1.0 / 60.0)
        pyglet.app.run()

    # ------------------------------------------------------------ rendering

    def _scale(self) -> float:
        return float(min(self.window.width, self.window.height)) / self.world_size

    def _world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        scale = self._scale()
        cx = self.window.width / 2.0
        cy = self.window.height / 2.0
        wcx, wcy = self.world_center
        return cx + (x - wcx) * scale, cy + (y - wcy) * scale

    def _color_for_robot(self, robot_id: int) -> tuple[int, int, int]:
        return ROBOT_COLORS[(robot_id - 1) % len(ROBOT_COLORS)]

    def _on_draw(self) -> None:
        self.window.clear()
        self._shapes.clear()
        self._labels.clear()

        trackers = dict(self.coordinator.trackers)
        css = list(self.coordinator.allCriticalSections)

        driving = {
            robotID: tracker
            for robotID, tracker in trackers.items()
            if not isinstance(tracker, TrajectoryEnvelopeTrackerDummy)
        }
        if driving:
            self._has_been_active = True

        # 1. paths — faint polylines
        for tracker in driving.values():
            e = tracker.getTrajectoryEnvelope()
            for i in range(e.length - 1):
                p0 = e.path[i].pose
                p1 = e.path[i + 1].pose
                x1, y1 = self._world_to_screen(p0.x, p0.y)
                x2, y2 = self._world_to_screen(p1.x, p1.y)
                self._shapes.append(
                    shapes.Line(x1, y1, x2, y2, thickness=1.0, color=PATH_COLOR, batch=self.batch)
                )

        # 2. swept envelope outlines (faint fill)
        if self.draw_swept_envelope:
            for robotID, tracker in driving.items():
                e = tracker.getTrajectoryEnvelope()
                geom = e.getSpatialEnvelope().getPolygon()
                polys = list(geom.geoms) if isinstance(geom, MultiPolygon) else [geom]
                rgb = self._color_for_robot(robotID)
                for poly_geom in polys:
                    if not isinstance(poly_geom, Polygon):
                        continue
                    pts = list(poly_geom.exterior.coords)
                    if len(pts) < 3:
                        continue
                    screen_pts = [self._world_to_screen(x, y) for x, y in pts[:-1]]
                    poly = shapes.Polygon(*screen_pts, color=rgb, batch=self.batch)
                    poly.opacity = SWEPT_OPACITY
                    self._shapes.append(poly)

        # 3. CS index ranges highlighted on each side
        for cs in css:
            for te, start, end in (
                (cs.getTe1(), cs.getTe1Start(), cs.getTe1End()),
                (cs.getTe2(), cs.getTe2Start(), cs.getTe2End()),
            ):
                if te is None or te.getRobotID() not in driving:
                    continue
                start = max(0, start)
                end = min(end, te.getPathLength() - 1)
                for i in range(start, end):
                    p0 = te.path[i].pose
                    p1 = te.path[i + 1].pose
                    x1, y1 = self._world_to_screen(p0.x, p0.y)
                    x2, y2 = self._world_to_screen(p1.x, p1.y)
                    self._shapes.append(
                        shapes.Line(
                            x1, y1, x2, y2, thickness=3.0, color=CS_HIGHLIGHT, batch=self.batch
                        )
                    )

        # 4. current footprints filled (both driving and parked robots)
        for robotID, tracker in trackers.items():
            rr = tracker.getRobotReport()
            pose = rr.getPose() if rr is not None else None
            if pose is None:
                continue
            footprint = self.coordinator.getFootprint(robotID)
            fp = place_footprint(footprint, pose)
            pts = list(fp.exterior.coords)
            if len(pts) < 3:
                continue
            screen_pts = [self._world_to_screen(x, y) for x, y in pts[:-1]]
            rgb = self._color_for_robot(robotID)
            poly = shapes.Polygon(*screen_pts, color=rgb, batch=self.batch)
            poly.opacity = 230 if robotID in driving else 120
            self._shapes.append(poly)

            cxw, cyw = fp.centroid.coords[0]
            sx, sy = self._world_to_screen(cxw, cyw)
            label = pyglet.text.Label(
                f"R{robotID}",
                x=sx,
                y=sy,
                anchor_x="center",
                anchor_y="center",
                font_size=11,
                color=LABEL_COLOR,
                batch=self.batch,
            )
            self._labels.append(label)

        # 5. status text
        status = (
            f"driving: {len(driving)}   parked: {len(trackers) - len(driving)}   "
            f"CSes: {len(css)}   orders: {len(self.coordinator.CSToDepsOrder)}"
        )
        self._labels.append(
            pyglet.text.Label(
                status,
                x=12,
                y=self.window.height - 22,
                font_size=12,
                color=TEXT_COLOR,
                batch=self.batch,
            )
        )

        # per-robot velocity/critical-point readout
        line_y = self.window.height - 44
        for robotID, tracker in sorted(trackers.items()):
            state = getattr(tracker, "state", None)
            v = state.getVelocity() if state is not None else None
            cp = getattr(tracker, "criticalPoint", None)
            if v is None and cp is None:
                continue
            r, g, b = self._color_for_robot(robotID)
            text = f"R{robotID}"
            if v is not None:
                text += f"  v={v:.2f}"
            if cp is not None:
                text += f"  cp={cp}"
            self._labels.append(
                pyglet.text.Label(
                    text,
                    x=12,
                    y=line_y,
                    font_size=11,
                    color=(r, g, b, 255),
                    batch=self.batch,
                )
            )
            line_y -= 18

        self.batch.draw()

        if self._stop_when_idle and self._has_been_active and not driving:
            pyglet.app.exit()
