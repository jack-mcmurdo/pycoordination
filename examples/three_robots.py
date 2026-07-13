"""Three RK4 robots whose paths all pass through the origin.

Each pair of paths crosses at the origin, so the coordinator must resolve
three pairwise critical sections and serialise the robots through the
intersection without deadlock.

Run:

    python examples/three_robots.py
"""

from __future__ import annotations

import asyncio

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.paths import three_robot_intersection

from _common import run


async def scenario(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    fp = footprint_coords(1.0, 0.6)
    p1, p2, p3 = three_robot_intersection()

    for robotID, (path, v_max) in enumerate(((p1, 1.5), (p2, 1.2), (p3, 1.0)), start=1):
        tec.setFootprint(robotID, *fp)
        tec.setRobotMaxVelocity(robotID, v_max)
        tec.setRobotMaxAcceleration(robotID, 1.0)
        tec.placeRobot(robotID, path[0].getPose())

    tec.addMissions(Mission(1, p1), Mission(2, p2), Mission(3, p3))
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    run(tec, scenario, world_size=22.0, title="three robots — RK4")
