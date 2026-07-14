---
name: verify
description: Build, run, and observe coordination_oru changes end-to-end (examples, web viewer, headless).
---

# Verifying coordination_oru changes

## Build / install

- `pip install -e ".[viz,dev]"` — core + both viewers + test deps.
- Web viewer frontend: `npm --prefix frontend install && npm --prefix frontend run build`
  (outputs to `coordination_oru/viz/static/`, gitignored; the web viewer
  refuses to start without it).

## Surfaces

- **Examples (CLI):** `python examples/two_robots.py [--web-viewer|--pyglet|--headless] [--port N] [--no-browser]`.
  Headless prints per-robot progress and a summary. All examples share the
  flags via `examples/_common.py`.
- **Web viewer:** serves http://127.0.0.1:8723/ by default; websocket at `/ws`
  streams `static` (paths/envelopes) and `state` (~20 Hz) messages — protocol
  in the `coordination_oru/viz/web_viewer.py` docstring. The `websockets`
  package makes a fine test client.

## Gotchas

- **The sim runs in real time and two_robots finishes in ~9 s.** Connect the
  websocket client / fire the screenshot *before or right after* launching the
  example or you'll only see the parked end-state (server keeps serving it).
- Foreground `python examples/...--web-viewer` blocks forever after finish
  (Ctrl+C to exit) — always background it and `kill -INT` when done.
- `pkill -f <pattern>` self-matches the harness's zsh eval wrapper — kill by
  pid (`pgrep` the python process) or `fuser -k PORT/tcp` instead.
- Headless browser render: `firefox --headless --profile "$(mktemp -d)"
  --window-size=1100,800 --screenshot out.png http://127.0.0.1:PORT/`
  (no chromium/playwright on this machine). Firefox takes ~3-5 s to load —
  launch the example ~1 s before it to catch robots mid-drive.
- Don't run `--pyglet` from an agent session: it pops a window on the user's
  desktop.
