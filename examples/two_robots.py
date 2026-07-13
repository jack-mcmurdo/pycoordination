"""Two RK4 robots with perpendicular paths crossing at the origin.

The coordinator detects the single critical section at the crossing, gives
one robot priority, and holds the other short of the intersection until the
winner clears it.

Run:

    python examples/two_robots.py
"""

from __future__ import annotations

import asyncio

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.paths import two_robot_cross

from _common import run


async def scenario(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    fp = footprint_coords(1.0, 0.6)
    path_a, path_b = two_robot_cross()

    tec.setFootprint(1, *fp)
    tec.setFootprint(2, *fp)
    tec.setRobotMaxVelocity(1, 1.5)
    tec.setRobotMaxAcceleration(1, 0.8)
    tec.setRobotMaxVelocity(2, 1.2)
    tec.setRobotMaxAcceleration(2, 0.8)

    tec.placeRobot(1, path_a[0].getPose())
    tec.placeRobot(2, path_b[0].getPose())

    tec.addMissions(Mission(1, path_a), Mission(2, path_b))
    # let the coordinator detect the CS before the sim ramps up
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    run(tec, scenario, world_size=14.0, title="two robots — RK4")
