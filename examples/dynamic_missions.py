"""Dynamic missions on the bundled demo map, planned by the built-in
Hybrid A* planner.

Three car-like robots live on a 20x20 m occupancy-grid map. In headless or
pyglet mode a scripted scenario runs: the robots swap corners through the
map's corridors (crossing routes the coordinator must sequence), then
drive back to their start poses. With ``--web-viewer`` the example is
**interactive** instead: no scripted missions — click a robot in the
browser, then press-drag-release on the map to post a goal pose (RViz
"2D Nav Goal" style; a plain click aims the goal heading from the robot
toward the click; Esc deselects). Each goal is planned with Hybrid A* and
dispatched as a mission; goals for a driving robot are ignored.

Run:

    python examples/dynamic_missions.py               # scripted (pyglet/headless)
    python examples/dynamic_missions.py --web-viewer  # interactive, point-and-click
"""

from __future__ import annotations

import asyncio
import math
import sys

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.mission import Mission
from coordination_oru.motionplanning import HybridAStarPlanner, load_bundled_map
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.geometry import footprint_coords

from _common import run, wait_until_idle

OMAP = load_bundled_map()
FOOTPRINT = footprint_coords(1.0, 0.6)
STARTS = {
    1: Pose(-8.0, -8.0, 0.0),
    2: Pose(8.0, -8.0, math.pi),
    3: Pose(-8.0, 8.0, 0.0),
}
planners: dict[int, HybridAStarPlanner] = {}


def setup_robots(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    for robotID, start in STARTS.items():
        tec.setFootprint(robotID, *FOOTPRINT)
        planner = HybridAStarPlanner(OMAP, turning_radius=1.0)
        planner.setFootprint(*FOOTPRINT)
        planners[robotID] = planner
        tec.setMotionPlanner(robotID, planner)
        tec.placeRobot(robotID, start)


def plan_path(planner: HybridAStarPlanner, start: Pose, goal: Pose) -> tuple[PoseSteering, ...]:
    """Plan start -> goal with the robot's planner; raise on failure."""
    planner.setStart(start)
    planner.setGoals(goal)
    if not planner.plan():
        raise RuntimeError(f"no path to {goal}")
    path = planner.getPath()
    assert path is not None
    return path


async def scenario_scripted(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    setup_robots(tec)

    # Mission set 1: crossing routes through the corridors.
    goals1 = {1: Pose(8.0, 8.0, 0.0), 2: Pose(-8.0, 8.0, math.pi), 3: Pose(8.0, -8.0, 0.0)}
    tec.addMissions(
        *(Mission(rid, plan_path(planners[rid], STARTS[rid], goal)) for rid, goal in goals1.items())
    )
    await wait_until_idle(tec, timeout=90.0)
    print("mission set 1 complete — sending the robots home")

    # Mission set 2: each robot back to its start pose (the runner's own
    # wait_until_idle bounds it).
    tec.addMissions(
        *(Mission(rid, plan_path(planners[rid], goals1[rid], STARTS[rid])) for rid in goals1)
    )


async def scenario_interactive(tec: TrajectoryEnvelopeCoordinatorSimulation) -> None:
    setup_robots(tec)  # no missions — goals come from clicks in the browser


if __name__ == "__main__":
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=20, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()

    async def on_goal(robotID: int, x: float, y: float, theta: float) -> None:
        if robotID not in tec.trackers:
            return
        if tec.isDrivingRobot(robotID):
            print(f"robot {robotID} is driving — goal ignored")
            return
        report = tec.trackers[robotID].getRobotReport()
        pose = report.getPose() if report is not None else None
        if pose is None:
            return
        try:
            path = await asyncio.to_thread(plan_path, planners[robotID], pose, Pose(x, y, theta))
        except RuntimeError as exc:
            print(exc)
            return
        tec.addMissions(Mission(robotID, path))
        print(f"robot {robotID} → ({x:.1f}, {y:.1f}, {theta:.2f})")

    # mirrors the flag _common._parse_args parses
    interactive = "--web-viewer" in sys.argv
    run(
        tec,
        scenario_interactive if interactive else scenario_scripted,
        occupancy_map=OMAP,
        on_goal=on_goal,
        interactive=interactive,
        world_size=21.0,
        world_center=(0.0, 0.0),
        title="dynamic missions",
    )
