"""Manual viz for the three debug paths from the original Java repo.

Paths span roughly x ∈ [15, 42], y ∈ [3, 17]; the viewer is centred on the
midpoint of that bounding box.

Run from the repo root:

    .venv/bin/python -m tests.manual.three_robots_oldpath_viz
"""

from __future__ import annotations

import asyncio

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.geometry import rectangular_footprint
from tests.manual._runner import run_viz
from tests.paths import load_path_file


def _bbox(paths: list[tuple]) -> tuple[float, float, float, float]:
    xs = [ps.pose.x for p in paths for ps in p]
    ys = [ps.pose.y for p in paths for ps in p]
    return min(xs), min(ys), max(xs), max(ys)


async def scenario(sim: SimulationCoordinator) -> None:
    fp = rectangular_footprint(1.0, 0.6)
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    sim.add_rk4_robot(Mission.make(1, paths[0], fp), v_max=2.0, a_max=1.0)
    sim.add_rk4_robot(Mission.make(2, paths[1], fp), v_max=2.0, a_max=1.0)
    sim.add_rk4_robot(Mission.make(3, paths[2], fp), v_max=2.0, a_max=1.0)
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    minx, miny, maxx, maxy = _bbox(paths)
    cx = (minx + maxx) / 2.0
    cy = (miny + maxy) / 2.0
    span = max(maxx - minx, maxy - miny) * 1.1  # 10% margin

    sim = SimulationCoordinator(period=0.02, sim_step_period=0.02)
    run_viz(
        sim,
        scenario,
        world_size=span,
        world_center=(cx, cy),
        title="three robots — debug1/2/3.path",
    )