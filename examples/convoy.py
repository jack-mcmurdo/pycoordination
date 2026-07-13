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

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.paths import load_path_file

from _common import run


async def scenario(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    fp = footprint_coords(1.0, 0.6)
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robotID, path in enumerate(paths, start=1):
        tec.setFootprint(robotID, *fp)
        tec.placeRobot(robotID, path[0].getPose())
    tec.addMissions(*[Mission(robotID, path) for robotID, path in enumerate(paths, start=1)])
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    run(
        tec,
        scenario,
        world_size=32.0,
        world_center=(28.0, 10.0),
        title="convoy — shared corridor",
    )
