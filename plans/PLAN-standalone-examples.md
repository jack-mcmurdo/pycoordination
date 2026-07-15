# Plan: standalone runnable examples + PyPI deploy

**Goal:** Give newcomers directly runnable `python examples/<scenario>.py` demos so nobody has to reverse-engineer the pytest suite to learn the coordinator (the pytest scenario tests stay as the regression suite), then add a GitHub Actions workflow that publishes the package to PyPI on version tags.

## Approach
Create a top-level `examples/` directory of plain scripts (not a package), one per scenario, each opening the pyglet viewer by default and falling back to a headless console run with a clear "install `.[viz]` for the animation" message when pyglet is missing. The existing `tests/manual/` viz scripts already prove out the runner pattern — fold them into `examples/` and delete `tests/manual/`. Promote the path helpers out of `tests/paths/` into the package (`coordination_oru/util/paths.py`) so both tests and examples import them without `tests.` on the path.

Only `debug1/2/3.path` are actually consumed (convoy + oldpath scenarios); the rest of `paths/` is unreferenced legacy. Ship those three as package data (`coordination_oru/data/`) loaded via `importlib.resources`, so every example — including the path-file ones — runs from an installed wheel, not just a source checkout. Generating the debug paths in code instead was rejected: they are recorded planner outputs whose corridor geometry the convoy semantics depend on; hand-built equivalents would silently change the scenario. The synthetic scenarios are already code-generated.

## Changes
- `coordination_oru/data/` — **new** subpackage (`__init__.py` empty); `git mv` `debug1.path`, `debug2.path`, `debug3.path` here from `paths/` (move, not copy — a single canonical copy, nothing to drift). Delete the rest of `paths/` and the directory itself: every remaining file is referenced by no code, and all are recoverable from git history.
- `coordination_oru/util/paths.py` — **new**; move `two_robot_cross`, `shuttle_path`, and the other synthetic generators here verbatim from `tests/paths/__init__.py`. `load_path_file(name)` keeps its one-arg signature but loads from `importlib.resources.files("coordination_oru.data")`; a separate `load_path(file: Path)` handles arbitrary user files (same parser, split out of the current function).
- `tests/paths/__init__.py` — becomes a thin shim re-exporting everything from `coordination_oru.util.paths`, so no test bodies change.
- `examples/_common.py` — **new**; the `run_viz` runner moved from `tests/manual/_runner.py`, extended with the headless fallback: `try: import pyglet` — on `ImportError`, print the install hint, run the sim to idle while printing per-robot path-index progress about once a second, then print a summary (envelopes completed, critical sections seen, priorities decided). Both paths keep `run_until_idle(timeout=120.0)` so a coordinator regression can never hang the terminal. Example scripts import only `coordination_oru.*` and sibling `_common` — never `tests.*` (which isn't on `sys.path` when running `python examples/x.py`).
- `examples/two_robots.py` — from `tests/manual/two_robots_viz.py`: two RK4 robots, perpendicular cross. Add a short module docstring saying what the scenario shows and how to run it (`python examples/two_robots.py`).
- `examples/three_robots.py` — from `tests/manual/three_robots_viz.py`, same treatment.
- `examples/three_robots_oldpath.py` — from `tests/manual/three_robots_oldpath_viz.py`; loads `debug1/2/3.path` via `load_path_file` (package data — works installed).
- `examples/convoy.py` — **new** script derived from the scenario in `tests/test_convoy.py` (leader/yielder through the shared y≈8.7 corridor), minus assertions.
- `examples/dynamic_missions.py` — **new** script derived from `tests/test_dynamic_missions.py`: robots receive a second mission after the first completes.
- `tests/manual/` — **delete** (fully superseded by `examples/`).
- `pyproject.toml` — drop `norecursedirs = ["tests/manual"]`; add `[tool.setuptools.package-data]` with `"coordination_oru.data" = ["*.path"]`.
- `README.md` — add an **Examples** section right after Install (list the five scripts with one line each, note the `[viz]` extra and headless fallback); update the Layout tree to include `examples/`.
- `.github/workflows/deploy.yml` — **new** (milestone 2); on push of `v*` tags, one job with these steps in order:
  1. checkout; set up Python 3.12; `pip install build twine`.
  2. **Tag/version guard:** extract `version` from `pyproject.toml` (`python -c "import tomllib; print(tomllib.load(open('pyproject.toml','rb'))['project']['version'])"`), compare to `${GITHUB_REF_NAME#v}`; `exit 1` with a clear message on mismatch. A forgotten version bump fails the run before anything is built or published.
  3. `python -m build`, then `twine check dist/*`.
  4. **Wheel smoke test:** `pip install dist/*.whl` and run `python -c "from coordination_oru.util.paths import load_path_file; assert len(load_path_file('debug1.path')) > 0"` — proves the wheel is importable and the package data made it in, before anything is published.
  5. `twine upload dist/*`. Job declares `environment: deploy`, with `TWINE_USERNAME: __token__` and `TWINE_PASSWORD: ${{ secrets.PYPI_API_KEY }}`. Top-of-file comment: the `deploy` environment and `PYPI_API_KEY` secret must exist on GitHub before tagging a release (created manually by the repo owner).

  Only step 5 is unexercisable before the first real tag; steps 1–4 are the identical commands run locally in the verification steps below, so the only untested code is the `twine upload` invocation itself, which is standard boilerplate.

## Milestone 1 — standalone examples

### Steps
- [x] Create `coordination_oru/data/`: `git mv` the three debug `.path` files in, delete the rest of `paths/`, add the package-data entry to `pyproject.toml`.
- [x] Move path helpers to `coordination_oru/util/paths.py` (loader switched to `importlib.resources`); shim `tests/paths/__init__.py`; run `pytest` to confirm the suite is green untouched. This step must precede the example scripts — it is what lets them avoid `tests.*` imports.
- [x] Create `examples/_common.py` with `run_viz` + headless fallback; delete `tests/manual/`.
- [x] Port the three viz scripts to `examples/` and write the two new example scripts (`convoy.py`, `dynamic_missions.py`).
- [x] Verify: `grep -rn "tests" examples/` returns nothing; each example runs headless (`python examples/<name>.py` with pyglet absent or a forced ImportError) and with viz; `three_robots_oldpath.py` works from a wheel install (`pip install .` into a scratch venv).
- [x] Update `README.md`.

## Milestone 2 — PyPI deploy workflow

### Steps
- [x] Add `.github/workflows/deploy.yml` exactly as specified in Changes (tag/version guard, build, `twine check`, wheel smoke test, upload).
- [x] Run workflow steps 2–4 locally verbatim (with `GITHUB_REF_NAME=v0.1.0`): version guard passes, `python -m build` + `twine check dist/*` pass, wheel smoke test confirms `coordination_oru/data/*.path` ships in the wheel. Also run the guard once with a deliberately wrong tag to confirm it fails.

## Protocol
One commit per milestone, two total.
