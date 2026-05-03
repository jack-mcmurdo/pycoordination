"""Pyglet-based visualisation for a running :class:`SimulationCoordinator`.

The viewer takes a *snapshot* of coordinator state on every draw frame
(60 Hz by default) and re-creates a small set of pyglet ``shapes`` from
it. State is read without locking — the brief race against the asyncio
sim loop is harmless for visualisation purposes (worst case: one frame's
worth of jitter).

Usage pattern (run sim in a daemon thread, viewer in the main thread):

.. code-block:: python

    sim = SimulationCoordinator(...)

    async def run_sim():
        await sim.start()
        sim.add_rk4_robot(...)
        await sim.run_until_idle(timeout=60.0)
        await sim.stop()

    loop = asyncio.new_event_loop()
    threading.Thread(
        target=lambda: (asyncio.set_event_loop(loop), loop.run_until_complete(run_sim())),
        daemon=True,
    ).start()

    viewer = PygletViewer(sim, world_size=15.0)
    viewer.stop_when_idle()
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
from shapely.geometry import Polygon

from coordination_oru.util.geometry import place_footprint

if TYPE_CHECKING:
    from coordination_oru.simulation.sim_coordinator import SimulationCoordinator


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
    """Read-only viewer for a :class:`SimulationCoordinator`."""

    def __init__(
        self,
        coordinator: "SimulationCoordinator",
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
        """Auto-close the window once every active envelope is completed."""
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

        envelopes = list(self.coordinator.envelopes_by_robot.values())
        trackers = self.coordinator.trackers
        css = self.coordinator.critical_sections
        priorities = self.coordinator.priorities

        active_envelopes = [e for e in envelopes if not e.completed]
        if active_envelopes:
            self._has_been_active = True

        # 1. paths — faint polylines
        for e in active_envelopes:
            color = PATH_COLOR
            for i in range(e.length - 1):
                p0 = e.path[i].pose
                p1 = e.path[i + 1].pose
                x1, y1 = self._world_to_screen(p0.x, p0.y)
                x2, y2 = self._world_to_screen(p1.x, p1.y)
                self._shapes.append(
                    shapes.Line(x1, y1, x2, y2, thickness=1.0, color=color, batch=self.batch)
                )

        # 2. swept envelope outlines (faint fill)
        if self.draw_swept_envelope:
            for e in active_envelopes:
                geom = e.spatial_envelope.geometry
                if not isinstance(geom, Polygon):
                    continue
                pts = list(geom.exterior.coords)
                if len(pts) < 3:
                    continue
                screen_pts = [self._world_to_screen(x, y) for x, y in pts[:-1]]
                rgb = self._color_for_robot(e.robot_id)
                poly = shapes.Polygon(*screen_pts, color=rgb, batch=self.batch)
                poly.opacity = SWEPT_OPACITY
                self._shapes.append(poly)

        # 3. CS index ranges highlighted on each side
        for cs in css:
            for envelope in (cs.envelope_a, cs.envelope_b):
                if envelope.completed:
                    continue
                start, end = cs.cs_range_for(envelope.envelope_id)
                start = max(0, start)
                end = min(end, envelope.length - 1)
                for i in range(start, end):
                    p0 = envelope.path[i].pose
                    p1 = envelope.path[i + 1].pose
                    x1, y1 = self._world_to_screen(p0.x, p0.y)
                    x2, y2 = self._world_to_screen(p1.x, p1.y)
                    self._shapes.append(
                        shapes.Line(
                            x1, y1, x2, y2, thickness=3.0, color=CS_HIGHLIGHT, batch=self.batch
                        )
                    )

        # 4. current footprints filled
        for e in active_envelopes:
            tracker = trackers.get(e.robot_id)
            if tracker is None:
                continue
            pose = getattr(tracker, "current_pose", None)
            if pose is None:
                continue
            fp = place_footprint(e.footprint, pose)
            pts = list(fp.exterior.coords)
            if len(pts) < 3:
                continue
            screen_pts = [self._world_to_screen(x, y) for x, y in pts[:-1]]
            rgb = self._color_for_robot(e.robot_id)
            poly = shapes.Polygon(*screen_pts, color=rgb, batch=self.batch)
            poly.opacity = 230
            self._shapes.append(poly)

            # robot id label centred on the footprint
            cxw, cyw = fp.centroid.coords[0]
            sx, sy = self._world_to_screen(cxw, cyw)
            label = pyglet.text.Label(
                f"R{e.robot_id}",
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
        completed_count = sum(1 for e in envelopes if e.completed)
        active_count = len(active_envelopes)
        status = (
            f"active: {active_count}   completed: {completed_count}   "
            f"CSes: {len(css)}   priorities: {len(priorities)}"
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

        # per-robot velocity readout (RK4 trackers expose ``v``)
        line_y = self.window.height - 44
        for robot_id, tracker in sorted(trackers.items()):
            v = getattr(tracker, "v", None)
            permit = getattr(tracker, "permit_index_until", None)
            if v is None and permit is None:
                continue
            r, g, b = self._color_for_robot(robot_id)
            text = f"R{robot_id}"
            if v is not None:
                text += f"  v={v:.2f}"
            if permit is not None:
                text += f"  permit={permit}"
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

        if self._stop_when_idle and self._has_been_active and active_count == 0:
            pyglet.app.exit()
