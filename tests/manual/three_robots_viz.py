"""Manual viz scenario: three RK4 robots crossing at the origin.

Run from the repo root:

    .venv/bin/python -m tests.manual.three_robots_viz
"""

from __future__ import annotations

import asyncio

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.geometry import rectangular_footprint
from tests.manual._runner import run_viz
from tests.paths import three_robot_intersection


async def scenario(sim: SimulationCoordinator) -> None:
    fp = rectangular_footprint(1.0, 0.6)
    p1, p2, p3 = three_robot_intersection()
    sim.add_rk4_robot(Mission.make(1, p1, fp), v_max=1.5, a_max=1.0)
    sim.add_rk4_robot(Mission.make(2, p2, fp), v_max=1.2, a_max=1.0)
    sim.add_rk4_robot(Mission.make(3, p3, fp), v_max=1.0, a_max=1.0)
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    sim = SimulationCoordinator(period=0.02, sim_step_period=0.02)
    run_viz(sim, scenario, world_size=22.0, title="three robots — RK4")
