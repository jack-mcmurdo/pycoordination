"""Three robots on the original Java repo's debug1/2/3.path files.

The three paths enter from the north at different x offsets, drop down, and
share a long east-bound corridor at y ≈ 8.7 — overlapping convoy lanes rather
than a perpendicular crossing. The path files ship as package data, so this
also works from an installed wheel.

Run:

    python examples/three_robots_oldpath.py
"""

from __future__ import annotations

import asyncio

from coordination_oru.mission import Mission
from coordination_oru.metacsp.spatial.pose import PoseSteering
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords
from coordination_oru.util.paths import load_path_file

from _common import run


def _bbox(
    paths: list[tuple[PoseSteering, ...]],
) -> tuple[float, float, float, float]:
    xs = [ps.pose.x for p in paths for ps in p]
    ys = [ps.pose.y for p in paths for ps in p]
    return min(xs), min(ys), max(xs), max(ys)


async def scenario(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    fp = footprint_coords(1.0, 0.6)
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    for robotID, path in enumerate(paths, start=1):
        tec.setFootprint(robotID, *fp)
        tec.setRobotMaxVelocity(robotID, 2.0)
        tec.setRobotMaxAcceleration(robotID, 1.0)
        tec.placeRobot(robotID, path[0].getPose())
    tec.addMissions(*[Mission(robotID, path) for robotID, path in enumerate(paths, start=1)])
    await asyncio.sleep(0.1)


if __name__ == "__main__":
    paths = [load_path_file(f"debug{i}.path") for i in (1, 2, 3)]
    minx, miny, maxx, maxy = _bbox(paths)
    cx = (minx + maxx) / 2.0
    cy = (miny + maxy) / 2.0
    span = max(maxx - minx, maxy - miny) * 1.1  # 10% margin

    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    run(
        tec,
        scenario,
        world_size=span,
        world_center=(cx, cy),
        title="three robots — debug1/2/3.path",
    )
