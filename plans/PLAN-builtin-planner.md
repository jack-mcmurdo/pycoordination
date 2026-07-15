# Plan: built-in Hybrid A\* planner (Reeds-Shepp) + point-and-click goals

**Goal:** Ship a built-in, pure-Python **Hybrid A\*** motion planner producing Reeds-Shepp-style car paths (with a small reversing penalty) on ROS-style occupancy-grid maps (YAML + PGM), using a circumcircle footprint for collision checking, and rework the dynamic-missions example so full goal poses can be posted per robot by press-drag-release in the web viewer (RViz "2D Nav Goal" style).

## Approach

Circumcircle footprint ⇒ collision checking is heading-independent: a state `(x, y, θ)` is valid iff its cell is free in the **inflated** grid (obstacles dilated by the circumradius). Hybrid A\* searches over continuous states binned into a `(row, col, θ-bin)` closed set, expanding 6 kinematic primitives ({forward, reverse} × {max-left, straight, max-right} arcs), with reverse motion costed at `reverse_cost ×` length plus a `gear_switch_cost` per direction change. Heuristic = max(obstacle-aware 2D Dijkstra distance field, obstacle-free Reeds-Shepp distance) — both admissible (RS curves are computed without the reverse penalty, and true cost ≥ RS length since multipliers ≥ 1). Termination is primarily by **analytic expansion**: a collision-checked Reeds-Shepp shot from a popped node exactly onto the goal pose. A standalone `reeds_shepp.py` module (48-word closed form) powers the shot, the heuristic, and is reusable for a future RRT\* variant. Everything is deterministic — no RNG anywhere.

The planner subclasses the existing `AbstractMotionPlanner`, which also makes the coordinator's `doReplanning` deadlock-breaking flow functional (it passes shapely obstacles; we rasterize them onto the inflated grid). The web viewer gains a map image layer and claims its reserved inbound websocket channel for `postGoal` messages carrying `[x, y, theta]`, dispatched to an `on_goal` callback wired by the example.

**Conventions that apply to every step:** Python ≥3.12, `mypy --strict` must stay green (`ignore_missing_imports` is already on). Java-style camelCase only where mirroring the Java port's API; new modules use snake_case like `coordination_oru/util/`. All new public functions get docstrings in the style of `coordination_oru/util/paths.py`. No randomness: two identical `plan()` calls must return identical paths.

## Changes

- `coordination_oru/motionplanning/occupancy_map.py` — **new**: `OccupancyMap` (YAML+PGM loader, world↔grid transforms, inflation, PNG export), `load_bundled_map()`.
- `coordination_oru/motionplanning/reeds_shepp.py` — **new**: closed-form Reeds-Shepp curves (solve, sample, lengths).
- `coordination_oru/motionplanning/hybrid_astar_planner.py` — **new**: `HybridAStarPlanner(AbstractMotionPlanner)`.
- `coordination_oru/motionplanning/__init__.py` — re-export `OccupancyMap`, `load_bundled_map`, `HybridAStarPlanner`.
- `scripts/gen_demo_map.py` — **new**: deterministic generator for the bundled demo map.
- `coordination_oru/data/maps/demo.yaml`, `demo.pgm` — **new**: generated, committed.
- `pyproject.toml` — deps `pyyaml`, `scipy`; package-data `maps/*`; version → 0.6.0.
- `coordination_oru/viz/web_viewer.py` — map in static message; inbound `postGoal` → `on_goal` callback.
- `examples/_common.py` — plumb `occupancy_map`, `on_goal`, `interactive` through `run()`/`_run_web`.
- `examples/dynamic_missions.py` — rewrite onto the map + planner; interactive in web mode.
- `frontend/src/lib/protocol.ts`, `lib/ws.ts`, `store.ts`, `components/WorldView.tsx` — map layer, robot selection, drag-to-set-heading goal posting.
- `tests/test_occupancy_map.py`, `tests/test_reeds_shepp.py`, `tests/test_hybrid_astar_planner.py` — **new**; `tests/test_web_viewer.py` — extend.

---

## Milestone 1 — map + Reeds-Shepp + Hybrid A\* (headless, fully tested). One commit.

### `coordination_oru/motionplanning/occupancy_map.py`

- [ ] Implement `_read_pgm(path: pathlib.Path) -> np.ndarray` returning `uint8` array of shape `(height, width)`:
  - Read the file as bytes. Tokenize the header: split on whitespace, but any token starting with `#` discards the rest of that line (comments). The first 4 tokens are: magic (`P5` or `P2`), width, height, maxval. Raise `ValueError` for any other magic or `maxval > 255`.
  - `P5`: pixel data is the `width*height` bytes immediately after the single whitespace byte that follows the maxval token; `np.frombuffer(..., dtype=np.uint8).reshape(height, width)`.
  - `P2`: remaining tokens are ASCII ints; `np.array(tokens, dtype=np.uint8).reshape(height, width)`.
- [ ] Implement `class OccupancyMap` (plain class, not a dataclass — it carries a cache):
  - `__init__(self, image: np.ndarray, resolution: float, origin: tuple[float, float], occupied: np.ndarray) -> None`:
    - `self.image`: `uint8` array `(height, width)` **already flipped to y-up** (row index increases with world +y). The loader does the flip; `__init__` stores as given.
    - `self.resolution: float` (metres/pixel), `self.origin: tuple[float, float]` (world x,y of the map's lower-left corner).
    - `self.occupied: np.ndarray` (bool, same shape).
    - `self._inflated_cache: dict[int, np.ndarray]` = `{}` (key: radius in pixels).
  - `@classmethod from_yaml(cls, yaml_path: str | pathlib.Path, *, unknown_is_occupied: bool = True) -> "OccupancyMap"`:
    - `yaml.safe_load` the file. Required keys: `image`, `resolution`, `origin`. Optional with defaults: `negate=0`, `occupied_thresh=0.65`, `free_thresh=0.196`, `mode="trinary"`. Raise `ValueError` if `mode != "trinary"` or `origin[2]` (yaw) is nonzero (message: rotated maps unsupported).
    - Resolve the image path relative to the YAML file's parent directory. If suffix is `.pgm` (case-insensitive) use `_read_pgm`; otherwise `try: from PIL import Image` and `np.asarray(Image.open(p).convert("L"), dtype=np.uint8)`; on `ImportError` raise `ValueError("non-PGM maps need Pillow: pip install pillow")`.
    - Flip to y-up: `img = np.flipud(raw).copy()`.
    - Occupancy probability per ROS map_server: `p = img / 255.0` if `negate` else `(255 - img) / 255.0`.
    - `occupied = p > occupied_thresh`; if `unknown_is_occupied`: `occupied |= (p > free_thresh)` (unknown band counts as occupied).
    - Return `cls(img, resolution, (origin[0], origin[1]), occupied)`.
  - Properties: `height`/`width` (from `image.shape`), `bounds -> tuple[float, float, float, float]` = `(ox, oy, ox + width*res, oy + height*res)`.
  - `world_to_grid(self, x: float, y: float) -> tuple[int, int]` returning `(row, col)` = `(floor((y - oy)/res), floor((x - ox)/res))` (plain `int(math.floor(...))`).
  - `grid_to_world(self, row: int, col: int) -> tuple[float, float]` returning the **cell center**: `(ox + (col + 0.5)*res, oy + (row + 0.5)*res)`.
  - `in_bounds(self, row: int, col: int) -> bool`.
  - `inflated(self, radius: float) -> np.ndarray` (bool; callers must `.copy()` before writing):
    - `k = math.ceil(radius / self.resolution)`; return cached if `k` in `self._inflated_cache`.
    - Disk structuring element: `yy, xx = np.mgrid[-k:k+1, -k:k+1]; disk = (xx*xx + yy*yy) <= k*k`.
    - `scipy.ndimage.binary_dilation(self.occupied, structure=disk)`; cache and return.
  - `to_png_bytes(self) -> bytes` — grayscale 8-bit PNG of `np.flipud(self.image)` (back to image orientation, top row first), pure stdlib:
    ```python
    def _chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))
    # body of to_png_bytes:
    img = np.flipud(self.image)
    h, w = img.shape
    raw = b"".join(b"\x00" + img[r].tobytes() for r in range(h))
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)  # bit depth 8, color type 0 (gray)
    return (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(raw)) + _chunk(b"IEND", b""))
    ```
- [ ] Implement `load_bundled_map(name: str = "demo.yaml") -> OccupancyMap`: `path = pathlib.Path(str(importlib.resources.files("coordination_oru.data") / "maps" / name))`, then `OccupancyMap.from_yaml(path)` (same `Path(str(files(...)))` trick as `_static_dir()` in `web_viewer.py`).

### `coordination_oru/motionplanning/reeds_shepp.py`

- [ ] Port the classical 48-word closed-form Reeds-Shepp solution from **PythonRobotics** `PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py` (MIT licence — compatible with this project's GPL; keep an attribution comment with the PythonRobotics copyright line at the top of the file). Port the math verbatim (the `SCS`/`CSC`/`CCC`/`CCCC`/`CCSC`/`CCSCC` word families with their timeflip/reflect transforms); adapt naming and types to this codebase. Public API:
  ```python
  @dataclass(frozen=True)
  class RSPath:
      lengths: tuple[float, ...]   # per-segment signed lengths in METRES (< 0 = reverse)
      ctypes: tuple[str, ...]      # per-segment "L" | "S" | "R", same arity as lengths
      total_length: float          # sum of abs(lengths), metres

  def solve(q0: tuple[float, float, float], q1: tuple[float, float, float],
            turning_radius: float) -> RSPath:
      """Shortest Reeds-Shepp path q0 -> q1 (poses as (x, y, theta))."""

  def sample_path(q0: tuple[float, float, float], path: RSPath, turning_radius: float,
                  step: float) -> list[tuple[float, float, float, int]]:
      """Poses along the path every `step` metres of arc length, as
      (x, y, theta, gear) with gear +1 forward / -1 reverse. Includes the
      start pose and the exact endpoint. theta normalized to [-pi, pi)."""

  def reverse_length(path: RSPath) -> float:
      """Sum of abs(length) over segments with negative length."""
  ```
  Implementation notes: internally normalize to unit turning radius (scale x,y by `1/R`, solve, scale lengths back by `R`); enumerate all candidate words and return the one minimizing `total_length`; `solve` never returns `None` (a valid word always exists for distinct poses; for `q0 == q1` return an `RSPath((), (), 0.0)`).
- [ ] `tests/test_reeds_shepp.py` (all with `turning_radius=1.0` unless stated):
  - **endpoint accuracy**: for every `(x, y, theta)` in the fixed grid `x, y ∈ {-4.0, -1.0, 0.5, 3.0}`, `theta ∈ {0.0, 1.2, math.pi/2, -2.5}` (64 goals from `q0=(0,0,0)`): last sample of `sample_path(q0, solve(q0, q1, 1.0), 1.0, 0.1)` matches `q1` within `1e-6` (x, y, and angle wrapped).
  - **straight cases**: `solve((0,0,0), (5,0,0), 1.0).total_length == pytest.approx(5.0)`; `solve((0,0,0), (-3,0,0), 1.0).total_length == pytest.approx(3.0)` and its `reverse_length == pytest.approx(3.0)` (backs straight up).
  - **lower bound**: `total_length >= euclidean(q0, q1) - 1e-9` for every grid case above.
  - **symmetry**: `solve(q0, q1, R).total_length == pytest.approx(solve(q1, q0, R).total_length)` for 5 of the grid cases.
  - **radius scaling**: `solve(q0, (0,4,math.pi), 2.0).total_length == pytest.approx(2 * solve((0,0,0), (0,2,math.pi), 1.0).total_length)`.
  - **gear flags**: the `(-3,0,0)` case yields samples whose `gear` is `-1`.

### `coordination_oru/motionplanning/hybrid_astar_planner.py`

- [ ] `class HybridAStarPlanner(AbstractMotionPlanner)`:
  - `__init__(self, occupancy_map: OccupancyMap, *, turning_radius: float = 1.0, path_step: float = 0.25, prim_step: float = 0.5, reverse_cost: float = 1.5, gear_switch_cost: float = 1.0, angle_bins: int = 72, heuristic_inflation: float = 1.3, max_expansions: int = 100_000) -> None` — call `super().__init__()`, store all. Validate `reverse_cost >= 1.0` and `turning_radius > 0` (raise `ValueError`).
  - Class docstring documents: car-like Reeds-Shepp model; collision via circumcircle on the inflated grid (heading-independent); start/goal **theta is honored**; reverse arcs cost `reverse_cost ×` length plus `gear_switch_cost` per gear change; `heuristic_inflation > 1` trades optimality for speed; output `PoseSteering.steering` is always `0.0`; deterministic.
  - `_circumradius(self) -> float`: `max(math.hypot(x, y) for x, y in self.footprintCoords)`; raise `RuntimeError("setFootprint(...) before planning")` if `footprintCoords` is `None` or empty.
  - **`doPlanning(self) -> bool`:**
    1. Raise `RuntimeError` if `self.start` is `None` or `self.goal` is empty.
    2. `r = self._circumradius()`; `grid = self._map.inflated(r)`.
    3. If `self.getObstacles()` is non-empty: `grid = grid.copy()`; for each obstacle geometry `g`: `buf = g.buffer(r)`; clamp `buf.bounds` to map bounds; for the affected cell sub-rectangle build meshgrids of **cell-center** world coordinates and `grid[rows, cols] |= shapely.contains_xy(buf, xs, ys)` (shapely ≥2.0 vectorized). Skip obstacles whose bbox misses the map.
    4. Waypoint chain: `poses = [self.start, *self.goal]`; for each consecutive pair call `_plan_segment(grid, p0, p1)`; if any returns `None`, set `self.pathPS = None`, return `False`. Concatenate segment pose lists, dropping each segment's first pose after the first segment (duplicate joint).
    5. `self.pathPS = tuple(PoseSteering(Pose(x, y, th), 0.0) for (x, y, th) in chain)`; return `True`.
  - **`_plan_segment(self, grid, p0: Pose, p1: Pose) -> list[tuple[float, float, float]] | None`:**
    1. `start = (p0.x, p0.y, wrap(p0.theta))`, `goal = (p1.x, p1.y, wrap(p1.theta))` where `wrap` normalizes to `[-pi, pi)`. If either cell (`world_to_grid`) is out of bounds or occupied in `grid` → return `None`. If `math.hypot(dx, dy) < 1e-9` and angle diff `< 2*pi/angle_bins` → return `[start]` … actually return `None` (degenerate; nothing to drive) — document this.
    2. **2D distance field** `dist2d`: Dijkstra from the goal cell over free cells of `grid`, 8-connected, step costs `res` / `res*sqrt(2)`, `heapq` with `(d, counter, cell)`; store a float `np.full(shape, np.inf)` array. If `dist2d[start_cell]` is `inf` → return `None` (goal unreachable even holonomically — fail fast).
    3. **Heuristic** `h(x, y, theta)`: `e = euclid((x,y), goal)`; `rs = reeds_shepp.solve((x,y,theta), goal, R).total_length if e <= 6*R else e` (avoids thousands of RS solves far from the goal; `e` is a valid lower bound for RS length); return `heuristic_inflation * max(dist2d[cell(x,y)], rs)`.
    4. **Search state**: continuous `(x, y, theta)` + `gear` (`0` at start, else `±1`). Closed-set key: `(row, col, theta_bin)` with `theta_bin = int(((theta + pi) / (2*pi)) * angle_bins) % angle_bins`. Dicts: `g_cost: dict[key, float]`, `entry: dict[key, tuple[x, y, theta, gear]]`, `parent: dict[key, key | None]`, `parent_motion: dict[key, list[pose_samples]]` (the collision-checked samples of the primitive that reached this key — used for path reconstruction). Open list: `heapq` entries `(f, counter, key)` with `counter = next(itertools.count_instance)`; skip stale pops (`g` in entry no longer matches `g_cost[key]` — simplest: also store `g` in the heap tuple and compare).
    5. **Primitives** from state `(x, y, theta)` with gear `gear_p`: for `direction in (+1, -1)` and `curvature in (+1/R, 0.0, -1/R)`: signed arc length `d = direction * prim_step`;
       - straight (`curvature == 0`): `x' = x + d*cos(theta)`, `y' = y + d*sin(theta)`, `theta' = theta`;
       - arc: `theta' = wrap(theta + d*curvature)`, `x' = x + (sin(theta') - sin(theta))/curvature`, `y' = y - (cos(theta') - cos(theta))/curvature`.
       - **Collision check + samples**: interpolate the primitive at every `res/2` of arc length including the endpoint (for the arc, interpolate `theta` linearly along the signed arc and recompute x,y with the same closed forms per sample). Every sample's cell must be in-bounds and free in `grid`; else discard the primitive.
       - **Cost**: `prim_step * (1.0 if direction > 0 else reverse_cost)` `+ (gear_switch_cost if gear_p != 0 and direction != gear_p else 0.0)`.
    6. **Analytic expansion**: keep a pop counter; on each popped state, if `euclid <= 3*R` or `pops % 20 == 0`: `rs = solve(state, goal, R)`; sample at `res/2`; if all samples' cells free → **success**: reconstruct (walk `parent`/`parent_motion` back to start, concatenating primitive samples in order, then append the RS shot samples). Deduplicate consecutive identical points.
    7. Also succeed if a popped state is within `path_step` (xy) and one `theta_bin` of the goal.
    8. Give up (`return None`) when the open list empties or `pops > max_expansions`.
    9. **Output resampling**: the reconstructed sample list is spaced ~`res/2`; walk its cumulative arc length (xy distance between consecutive samples) and keep one pose every `path_step` metres, **plus**: always keep the first pose, the last pose, and every pose where the gear flips (direction of xy motion reverses w.r.t. heading) so cusps survive resampling. Return the pose list as `(x, y, theta)` tuples. Headings come from the sampled states, never recomputed from tangents (reversing means heading ≠ travel direction).
- [ ] Update `coordination_oru/motionplanning/__init__.py` to export `OccupancyMap`, `load_bundled_map`, `HybridAStarPlanner` (keep the existing `AbstractMotionPlanner` export).

### Demo map

- [ ] Write `scripts/gen_demo_map.py` (stdlib + numpy only, no argparse — fixed output paths `coordination_oru/data/maps/demo.{pgm,yaml}`):
  - 400×400 px, `resolution = 0.05` (20×20 m), `origin = [-10.0, -10.0, 0.0]`. Pixel values: free `254`, occupied `0`. Work in a y-up numpy array, then `np.flipud` before writing (PGM row 0 is the map's top).
  - `fill(x0, y0, x1, y1)` helper marks occupied every pixel whose cell center lies inside the world rect. Obstacles (world coords):
    - border walls 0.2 m thick along all four edges;
    - block: `(-6, -2) → (-2, 2)`;
    - block: `(2, -6) → (6, -2)`;
    - L-wall: `(-2, 4) → (6, 4.4)` and `(5.6, 0) → (6, 4.4)`.
  - (Corridors are ≥ 3.5 m wide — comfortable for `turning_radius=1.0` and circumradius ≈ 0.6.)
  - Write PGM P5: header `b"P5\n400 400\n255\n"` + `img.tobytes()`. Write `demo.yaml`:
    ```yaml
    image: demo.pgm
    resolution: 0.05
    origin: [-10.0, -10.0, 0.0]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
    ```
- [ ] Run `python scripts/gen_demo_map.py`; commit the two generated files.

### Packaging

- [ ] `pyproject.toml`: add `"pyyaml>=6.0"` and `"scipy>=1.12"` to `[project] dependencies`; change the `"coordination_oru.data"` package-data line to `["*.path", "maps/*"]`; bump `version` to `"0.6.0"`.

### Tests (milestone 1)

- [ ] `tests/test_occupancy_map.py` (build a tiny map in `tmp_path`: write a 10×8 P5 PGM — 8 wide, 10 tall — all `254` except a 2×2 block of `0` at image rows 0–1, cols 0–1, i.e. the **top-left**; a YAML with `resolution: 0.5`, `origin: [1.0, 2.0, 0.0]`):
  - loader: `width == 8`, `height == 10`, `bounds == (1.0, 2.0, 5.0, 7.0)`.
  - y-flip: the occupied block is image-top-left ⇒ world-top-left ⇒ `occupied[9, 0] and occupied[8, 1]` are `True`, `occupied[0, 0]` is `False`.
  - transforms: `world_to_grid(*grid_to_world(r, c)) == (r, c)` for corner and center cells.
  - `negate: 1` variant flips which pixels are occupied; a mid-gray (`205`) pixel is occupied by default and free with `unknown_is_occupied=False`.
  - `inflated(0.5)` marks strictly more cells than `occupied`; `inflated(0.0)` equals `occupied`.
  - `to_png_bytes()` starts with `b"\x89PNG\r\n\x1a\n"` and contains `b"IEND"`.
  - `from_yaml` raises `ValueError` on nonzero yaw and on `mode: raw`.
  - `load_bundled_map()` loads and is 400×400.
- [ ] `tests/test_hybrid_astar_planner.py` (use `load_bundled_map()`; footprint `footprint_coords(1.0, 0.6)` from `coordination_oru.util.geometry`, circumradius ≈ 0.583; planner defaults unless stated):
  - **basic**: start `Pose(-8, -8, 0)`, goal `Pose(8, 8, math.pi/2)`: `plan()` is `True`; first path pose equals start (x, y, wrapped theta within `1e-6`); last pose equals goal likewise (analytic expansion is exact); consecutive xy spacing ≤ `path_step * 1.5`; every waypoint's cell is free in `inflated(circumradius)`; consecutive heading change `abs(wrap(dtheta)) ≤ spacing/turning_radius + 0.05` (curvature bound).
  - **determinism**: two identical `plan()` calls yield equal `pathPS`.
  - **reversing works**: open-area segment in the NW corner — start `Pose(-7, 8, 0)`, goal `Pose(-8.5, 8, 0)` (1.5 m directly **behind**, same heading): `plan()` succeeds and total path xy length `< 2 * math.pi * 1.0` (it backs up rather than driving a loop); at least one consecutive pair moves opposite the heading (dot product of the xy-delta with `(cos θ, sin θ)` is negative) — proves a reverse segment exists.
  - **reverse penalty**: same segment with `reverse_cost=1.5` vs `reverse_cost=1.0`: both succeed; the `1.0` run's summed reverse distance ≥ the `1.5` run's − 1e-6 (penalty never increases reversing).
  - **failure modes**: goal inside a block (e.g. `Pose(-4, 0, 0)`) → `False`; goal outside map bounds → `False`; goal walled off by a dynamic obstacle: `addObstacles([shapely box sealing the map's NE quadrant])` → `False`; then `clearObstacles()` → `True` again.
  - **multi-goal**: `setGoals(Pose(0, -8, 0), Pose(8, 8, math.pi/2))` → path passes within `0.5` m of `(0, -8)`.
  - **dynamic obstacle detour**: `addObstacles([shapely.geometry.box(-1.0, 2.0, 1.0, 8.0)])` on a start/goal pair whose unobstructed path crosses that region; `plan()` succeeds and every waypoint keeps `box.distance(shapely.geometry.Point(x, y)) >= circumradius - 2*resolution`.
- [ ] Run `python -m pytest` and `python -m mypy coordination_oru` — both green. **Commit 1** (message: `Add built-in Hybrid A* planner with Reeds-Shepp paths and ROS-style maps`).

---

## Milestone 2 — web viewer point-and-click goal poses + interactive example. One commit.

### Backend: `coordination_oru/viz/web_viewer.py`

- [ ] `build_static_message(...)` gains keywords `occupancy_map: "OccupancyMap | None" = None`, `map_data_uri: str | None = None`, and `interactive: bool = False`. When map + uri are given, add to the returned dict:
  `"map": {"dataUri": map_data_uri, "resolution": m.resolution, "origin": [ox, oy], "width": m.width, "height": m.height}`. Always add `"interactive": interactive` — the frontend uses it to enable/disable the goal-posting UX, so read-only examples keep today's behavior exactly.
- [ ] `WebViewer.__init__` gains `map: "OccupancyMap | None" = None` and `on_goal: "Callable[[int, float, float, float], Awaitable[None]] | None" = None`. If `map` is not `None`, precompute once: `self._map_data_uri = "data:image/png;base64," + base64.b64encode(map.to_png_bytes()).decode("ascii")`. `_static_message()` forwards `occupancy_map`, `map_data_uri`, and `interactive=self.on_goal is not None` to `build_static_message`.
- [ ] `_ws_endpoint`: replace the `receive_text()` no-op loop with:
  ```python
  while True:
      raw = await websocket.receive_text()
      try:
          msg = json.loads(raw)
      except json.JSONDecodeError:
          continue
      goal = msg.get("goal")
      if (msg.get("kind") == "postGoal" and self.on_goal is not None
              and isinstance(msg.get("robot"), int)
              and isinstance(goal, list) and len(goal) == 3
              and all(isinstance(v, (int, float)) for v in goal)):
          await self.on_goal(msg["robot"], float(goal[0]), float(goal[1]), float(goal[2]))
  ```
  Malformed/unknown messages are silently ignored.
- [ ] Update the module docstring's wire-protocol section: `static` gains optional `map`; document inbound `{"kind": "postGoal", "robot": int, "goal": [x, y, theta]}`.

### Runner: `examples/_common.py`

- [ ] `run(...)` gains keywords `occupancy_map=None`, `on_goal=None`, `interactive: bool = False`; forward all three to `_run_web`. In pyglet/headless modes, if `interactive` print one line: `"interactive goal posting needs --web-viewer; running the scripted scenario"` and proceed as today.
- [ ] `_run_web`: pass `map=occupancy_map, on_goal=on_goal` to `WebViewer`. After `await scenario(tec)`: if `interactive`, print `"select a robot, then press-drag-release to post a goal pose (Ctrl+C to exit)"` and `await server_task` directly (inference keeps running; the existing `finally: await tec.stopInference()` handles shutdown); else keep the current `wait_until_idle` → `_print_summary` → `await server_task` flow.

### Example: `examples/dynamic_missions.py` (rewrite)

- [ ] Setup shared by both scenarios: `omap = load_bundled_map()`; `fp = footprint_coords(1.0, 0.6)`; robots 1–3 with start poses R1 `Pose(-8, -8, 0.0)`, R2 `Pose(8, -8, math.pi)`, R3 `Pose(-8, 8, 0.0)`. Per robot `i`: `tec.setFootprint(i, *fp)`; `planners[i] = HybridAStarPlanner(omap, turning_radius=1.0)`; `planners[i].setFootprint(*fp)`; `tec.setMotionPlanner(i, planners[i])`; `tec.placeRobot(i, start_i)`.
- [ ] Helper `def plan_path(planner: HybridAStarPlanner, start: Pose, goal: Pose) -> tuple[PoseSteering, ...]`: `setStart(start)`, `setGoals(goal)`, `planner.plan()`; raise `RuntimeError(f"no path to {goal}")` on `False`; return `planner.getPath()`.
- [ ] `scenario_scripted` (headless/pyglet): mission set 1 — R1→`Pose(8, 8, 0.0)`, R2→`Pose(-8, 8, math.pi)`, R3→`Pose(8, -8, 0.0)` (crossing routes through the corridors); `tec.addMissions(...)`, `await wait_until_idle(tec, timeout=90.0)`, print a progress line; mission set 2 — each robot back to its start pose; `tec.addMissions(...)` (the runner's own `wait_until_idle` bounds it).
- [ ] `scenario_interactive` (web): setup only — no missions; goals come from clicks.
- [ ] `async def on_goal(robotID: int, x: float, y: float, theta: float) -> None`: if `robotID not in tec.trackers`: return. If `tec.isDrivingRobot(robotID)`: `print(f"robot {robotID} is driving — goal ignored")`; return. Current pose: `tec.trackers[robotID].getRobotReport().getPose()`. `try: path = await asyncio.to_thread(plan_path, planners[robotID], pose, Pose(x, y, theta))` `except RuntimeError as exc: print(exc); return`. Then `tec.addMissions(Mission(robotID, path))` and print `f"robot {robotID} → ({x:.1f}, {y:.1f}, {theta:.2f})"`.
- [ ] `__main__`: define both scenarios; `interactive = "--web-viewer" in sys.argv` (one-line comment: mirrors the flag `_common._parse_args` parses). Call `run(tec, scenario_interactive if interactive else scenario_scripted, occupancy_map=omap, on_goal=on_goal, interactive=interactive, world_size=21.0, world_center=(0.0, 0.0), title="dynamic missions")`.
- [ ] Update the module docstring: what it shows, how to run each mode, the click UX (`click a robot → press-drag-release to set the goal pose; plain click aims the goal heading from the robot toward the click; Esc deselects`).

### Frontend

- [ ] `frontend/src/lib/protocol.ts`: add
  ```ts
  export interface MapInfo { dataUri: string; resolution: number; origin: Point; width: number; height: number }
  // StaticMessage gains: map?: MapInfo; interactive?: boolean
  export interface PostGoalMessage { kind: "postGoal"; robot: number; goal: [number, number, number] }
  ```
- [ ] `frontend/src/lib/ws.ts`: keep a module-level `let activeSocket: WebSocket | null`; set in `connect()`, clear on close; export `function sendPostGoal(msg: PostGoalMessage): void` sending `JSON.stringify(msg)` only when `activeSocket?.readyState === WebSocket.OPEN`.
- [ ] `frontend/src/store.ts`: add `selectedRobot: number | null` (init `null`) and `setSelectedRobot: (id: number | null) => void`.
- [ ] `frontend/src/components/WorldView.tsx`:
  - **Coordinate helper** inside the component: `function clientToWorld(e): {x, y}` using `new DOMPoint(e.clientX, e.clientY).matrixTransform(svg.getScreenCTM()!.inverse())` — returns `{x: pt.x, y: -pt.y}` (undo the y-negation; exact under `preserveAspectRatio` letterboxing).
  - **Map layer** (first child inside `<svg>`, before `StaticLayers`): when `staticData.map` exists,
    ```tsx
    <image href={map.dataUri} x={map.origin[0]} y={-(map.origin[1] + map.height * map.resolution)}
           width={map.width * map.resolution} height={map.height * map.resolution}
           preserveAspectRatio="none" opacity={0.85}
           style={{ imageRendering: "pixelated" }} />
    ```
    (top of the map in world y = `origin[1] + height·resolution`, negated for the y-down SVG.)
  - **Interactivity gate**: `const interactive = staticData.interactive === true;` — when false, none of the selection/goal handlers below are attached and no hint renders; the view behaves exactly as today (drag always pans).
  - **Robot selection** (only when `interactive`): on each robot `<g>` add `onPointerDown={(e) => e.stopPropagation()}` and `onClick={(e) => { e.stopPropagation(); setSelectedRobot(robot.id); }}`, `className="cursor-pointer"`. Selected robot gets a dashed `<circle>` inside its `<g>` (inherits the pose transform): radius = 1.4 × footprint circumradius (add a `circumradii: Map<number, number>` memo beside `outlines`: `max(hypot(x, y))` over the raw ring points), `fill="none"`, `stroke={robotColor(robot.id)}`, `strokeDasharray={stroke * 4}`, `strokeWidth={stroke}`.
  - **Modal drag rule**: when `selectedRobot === null`, pointer drag pans exactly as today. When a robot **is** selected, background drag becomes **goal posting** (pan is unavailable; wheel zoom still works; Esc restores pan):
    - `onPointerDown` (background): store `goalDrag = { anchor: clientToWorld(e) }` in a `useRef` + a `useState` for the live preview endpoint; capture the pointer.
    - `onPointerMove`: update the preview endpoint (`clientToWorld(e)`).
    - `onPointerUp`: `dx = cur.x - anchor.x, dy = cur.y - anchor.y`; if pointer moved ≥ 8 **screen px** since down: `theta = Math.atan2(dy, dx)`; else (plain click) `theta = Math.atan2(anchor.y - robotY, anchor.x - robotX)` using the selected robot's live pose. `sendPostGoal({ kind: "postGoal", robot: selectedRobot, goal: [r3(anchor.x), r3(anchor.y), r3(theta)] })` (`r3` = round to 3 decimals). Clear the drag state; **keep** the selection.
    - **Preview** while dragging: a line from anchor to cursor with the existing arrow marker style, stroke `robotColor(selectedRobot)`, plus a small circle at the anchor.
  - **Escape**: `useEffect` window keydown listener — Esc clears any active goal drag and calls `setSelectedRobot(null)`.
  - **Hint overlay**: when a robot is selected, absolutely-positioned muted text bottom-center: `Robot N — press-drag-release to send a goal pose · Esc to cancel`.
- [ ] Build: `npm --prefix frontend install && npm --prefix frontend run build` (must succeed; fix TS errors).

### Tests (milestone 2)

- [ ] Extend `tests/test_web_viewer.py`, following its existing patterns for constructing messages/apps:
  - `build_static_message(..., occupancy_map=m, map_data_uri="data:image/png;base64,AAA=")` includes a `map` dict with the right `resolution`/`origin`/`width`/`height`; without the kwargs there is no `"map"` key. `"interactive"` is `False` by default and `True` when passed; a `WebViewer` constructed with an `on_goal` produces a static message with `"interactive": True`.
  - inbound handling: `WebViewer` with an `on_goal` that appends to a list; drive the websocket via the file's existing test approach; send `{"kind": "postGoal", "robot": 1, "goal": [2.5, -3.0, 1.57]}` → recorded `(1, 2.5, -3.0, 1.57)`; send garbage (`"not json"`, `{"kind": "postGoal", "robot": "x"}`, 2-element goal) → no call, no exception.
- [ ] Verify end-to-end: `python -m pytest`, `python -m mypy coordination_oru`, then `python examples/dynamic_missions.py --headless` (scripted sets complete) and `python examples/dynamic_missions.py --web-viewer --no-browser` (manual: map renders; select robot, drag a goal pose, robot drives a smooth car-like path; two robots given crossing goals coordinate without collision; a goal behind a robot produces a short reversing maneuver). **Commit 2** (message: `Add point-and-click goal poses on the demo map (web viewer + dynamic_missions)`).

## Edge cases & risks

- **Pure-Python Hybrid A\* speed**: the demo map (400×400, 72 θ-bins, `prim_step=0.5`) should plan in roughly 0.5–3 s per query; `heuristic_inflation=1.3`, the 2D Dijkstra field, and the RS-heuristic distance gate keep it bounded, and `max_expansions` guarantees termination. The example plans in `asyncio.to_thread`, so the viewer never stalls. Very large maps will be slow — documented in the planner docstring.
- **Reeds-Shepp closed form is the riskiest code** — hence porting the battle-tested PythonRobotics math verbatim (MIT, attribution comment required) and the endpoint-accuracy grid test at 1e-6.
- **Cusps must survive resampling** (step 9 keeps gear-flip poses) or reversal segments degrade; the tracker/coordinator already handles reversal paths (see commit `00b8fc7`).
- **Reverse penalty vs heuristic admissibility**: the RS heuristic ignores the penalty, which keeps it admissible (`reverse_cost ≥ 1` enforced).
- **Goal for a driving robot** is ignored with a printed notice (no queueing).
- **Rotated maps** (origin yaw ≠ 0) and non-trinary modes raise `ValueError` up front; **unknown cells** are occupied by default (`unknown_is_occupied=True`).
- **Pan while a robot is selected** is unavailable (drag = goal posting); Esc restores it. Acceptable modal trade-off; revisit if it annoys.
- **`scipy` dependency** (~40 MB wheel) added to core deps; the planner is a core feature so an extra would hurt more than the size does.
