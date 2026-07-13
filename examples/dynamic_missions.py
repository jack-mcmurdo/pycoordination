"""Dynamic missions: robots receive new work after finishing their first run.

Two robots first cross at the origin (horizontal vs vertical). Once both
missions complete, the same robots are given a second, diagonal crossing
pair. The coordinator must clean up the finished envelopes — no leaked
critical sections or precedence orders — and coordinate the new pair from
scratch.

Run:

    python examples/dynamic_missions.py
"""

from __future__ import annotations

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.paths import shuttle_path

from _common import run, wait_until_idle


async def scenario(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    fp = footprint_coords(1.0, 0.6)
    tec.setFootprint(1, *fp)
    tec.setFootprint(2, *fp)

    # Mission set 1: perpendicular cross at the origin.
    path1a = shuttle_path((-4.0, 0.0), (4.0, 0.0))
    path2a = shuttle_path((0.0, -4.0), (0.0, 4.0))
    tec.placeRobot(1, path1a[0].getPose())
    tec.placeRobot(2, path2a[0].getPose())
    tec.addMissions(Mission(1, path1a), Mission(2, path2a))
    await wait_until_idle(tec, timeout=30.0)
    print("mission set 1 complete — dispatching set 2")

    # Mission set 2: the same robots cross again on the diagonals.
    path1b = shuttle_path((4.0, 4.0), (-4.0, -4.0))
    path2b = shuttle_path((-4.0, 4.0), (4.0, -4.0))
    tec.addMissions(Mission(1, path1b), Mission(2, path2b))


if __name__ == "__main__":
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    run(tec, scenario, world_size=14.0, title="dynamic missions")
