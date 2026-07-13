"""Two RK4 robots with perpendicular paths crossing at the origin.

The coordinator detects the single critical section at the crossing, gives
one robot priority, and holds the other short of the intersection until the
winner clears it.

Run:

    python examples/two_robots.py
"""

from __future__ import annotations

import asyncio

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.geometry import rectangular_footprint
from coordination_oru.util.paths import two_robot_cross

from _common import run


async def scenario(sim: SimulationCoordinator) -> None:
    fp = rectangular_footprint(1.0, 0.6)
    path_a, path_b = two_robot_cross()
    sim.add_rk4_robot(
        Mission.make(robot_id=1, path=path_a, footprint=fp), v_max=1.5, a_max=0.8
    )
    sim.add_rk4_robot(
        Mission.make(robot_id=2, path=path_b, footprint=fp), v_max=1.2, a_max=0.8
    )
    # let the coordinator detect the CS before the sim ramps up
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    sim = SimulationCoordinator(period=0.02, sim_step_period=0.02)
    run(sim, scenario, world_size=14.0, title="two robots — RK4")
