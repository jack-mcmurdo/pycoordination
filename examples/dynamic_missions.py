"""Dynamic missions: robots receive new work after finishing their first run.

Two robots first cross at the origin (horizontal vs vertical). Once both
missions complete, the same robots are given a second, diagonal crossing
pair. The coordinator must clean up the finished envelopes — no leaked
critical sections or priorities — and coordinate the new pair from scratch.

Run:

    python examples/dynamic_missions.py
"""

from __future__ import annotations

from coordination_oru.coordinator.mission import Mission
from coordination_oru.simulation.sim_coordinator import SimulationCoordinator
from coordination_oru.util.geometry import rectangular_footprint
from coordination_oru.util.paths import shuttle_path

from _common import run


async def scenario(sim: SimulationCoordinator) -> None:
    fp = rectangular_footprint(1.0, 0.6)

    # Mission set 1: perpendicular cross at the origin.
    sim.add_robot(
        Mission.make(robot_id=1, path=shuttle_path((-4.0, 0.0), (4.0, 0.0)), footprint=fp)
    )
    sim.add_robot(
        Mission.make(robot_id=2, path=shuttle_path((0.0, -4.0), (0.0, 4.0)), footprint=fp)
    )
    await sim.run_until_idle(timeout=30.0)
    print("mission set 1 complete — dispatching set 2")

    # Mission set 2: the same robots cross again on the diagonals.
    sim.add_robot(
        Mission.make(robot_id=1, path=shuttle_path((4.0, 4.0), (-4.0, -4.0)), footprint=fp)
    )
    sim.add_robot(
        Mission.make(robot_id=2, path=shuttle_path((-4.0, 4.0), (4.0, -4.0)), footprint=fp)
    )


if __name__ == "__main__":
    sim = SimulationCoordinator(period=0.02, sim_step_period=0.02)
    run(sim, scenario, world_size=14.0, title="dynamic missions")
