"""Convoy following: a yielder trails the leader inside a shared corridor.

The debug paths share a long east-bound corridor at y ≈ 8.7. When two robots
traverse a critical section in the same direction, the yielder is not parked
at the entrance until the leader fully exits — it follows the leader through
at a fixed trailing buffer (the Java ``getCriticalPoint`` convoy semantics).
Watch the trailing robots advance while the leader is still inside the
corridor.

Run:

    python examples/convoy.py
"""

from __future__ import annotations

import asyncio

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.geometry import rectangular_footprint
from coordination_oru.util.paths import load_path_file

from _common import run


async def scenario(sim: SimulationCoordinator) -> None:
    fp = rectangular_footprint(1.0, 0.6)
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robot_id, path in enumerate(paths, start=1):
        sim.add_robot(Mission.make(robot_id=robot_id, path=path, footprint=fp))
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    sim = SimulationCoordinator(period=0.02, sim_step_period=0.02)
    run(
        sim,
        scenario,
        world_size=32.0,
        world_center=(28.0, 10.0),
        title="convoy — shared corridor",
    )
