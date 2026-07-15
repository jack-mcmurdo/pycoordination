# Visualization

Every example accepts the same flags — no per-script setup:

```bash
python examples/two_robots.py                # pyglet if installed, else headless
python examples/two_robots.py --web-viewer   # browser viewer
python examples/two_robots.py --pyglet       # force the pyglet window
python examples/two_robots.py --headless     # force text-only
```

## Web viewer

`WebViewer` (`coordination_oru/viz/web_viewer.py`) serves a Vite/React
frontend (starlette + uvicorn, port 8723 by default; `--port`,
`--no-browser`) and streams coordinator state over a websocket. It shows live
paths, swept envelopes, critical-section highlights, current dependencies,
footprints, zoom/pan, dark mode — and, when the example provides an
`on_goal` callback, interactive goal posting (click a robot, press-drag-release
to set the goal pose).

It is a pure polling observer: it reads public coordinator state and never
calls into the core. The server shares the simulation's asyncio loop — no
threads. PyPI wheels ship the frontend prebuilt; in a source checkout build
it once with `npm --prefix frontend install && npm --prefix frontend run build`.

## Pyglet viewer

`PygletViewer` (`coordination_oru/viz/pyglet_viewer.py`) is a zero-dependency
(beyond `pyglet`) local window with the same essentials: footprints,
envelopes, critical sections, per-robot progress. The simulation runs on a
background thread's event loop; the window closes itself when the fleet goes
idle.

## Headless

No extra dependencies: per-robot progress lines once a second and a final
summary (open critical sections, held precedence decisions). All example
runs are bounded by a 120 s idle timeout, so a coordination bug can never
hang a terminal or CI job.
