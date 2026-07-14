"""Five RK4 robots on interleaved sine and cosine waves.

All robots start at x = 0 with y offsets 4, 8, 12, 16 and 20. Odd robots
follow a sine, even robots a cosine (a 90° phase shift), all with a 3 m
amplitude. Amplitudes sum to 6 m across a 4 m offset, so every adjacent
pair of curves intersects (4 < 3√2) and the coordinator has to thread all
five robots through a lattice of critical sections.

Note the offset/amplitude interplay: the cosine robots start at their
*peak*, i.e. only ``|offset − 3|`` metres from the sine neighbour above —
the closest the fleet ever gets to itself is at t = 0, and some robots
begin inside a critical section (held at index 0 until it clears).

Run:

    python examples/five_robots_sine.py
"""

from __future__ import annotations

import asyncio
import math

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.paths import sine_path

from _common import run

V_MAX = (1.5, 1.2, 1.4, 1.3, 1.6)


async def scenario(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    fp = footprint_coords(1.0, 0.6)
    missions = []
    for i in range(5):
        robotID = i + 1
        # odd robots: sine; even robots: cosine (90° phase shift)
        path = sine_path(4.0 * (i + 1), phase=0.0 if i % 2 == 0 else math.pi / 2.0)
        tec.setFootprint(robotID, *fp)
        tec.setRobotMaxVelocity(robotID, V_MAX[i])
        tec.setRobotMaxAcceleration(robotID, 1.0)
        tec.placeRobot(robotID, path[0].getPose())
        missions.append(Mission(robotID, path))

    tec.addMissions(*missions)
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    run(tec, scenario, world_size=30.0, world_center=(12.0, 12.0), title="five robots — sine waves")
