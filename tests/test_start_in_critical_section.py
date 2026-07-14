"""Regression: missions that begin inside a critical section must not collide.

A sine robot and a cosine robot (90° phase shift) whose start poses sit
1 m apart: the cosine robot starts at its peak, inside the pair's first
critical section (CS start index 0). Two historical bugs made this pair
collide within the first metre:

- the RK4 tracker dropped a critical point equal to the robot's current
  path index (``cp=0`` at index 0), so the hold at the start was ignored;
- precedence reversals could pick a newly-waiting robot that was unable to
  stop by the reversed waiting point, which its tracker then silently
  rejected, letting both robots into the critical section.
"""

from __future__ import annotations

import asyncio
import math

import pytest

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.util.paths import sine_path
from tests.conftest import assert_no_collisions, wait_until_idle

pytestmark = pytest.mark.asyncio


async def test_start_inside_critical_section_no_collision(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation,
    footprint: tuple[tuple[float, float], ...],
) -> None:
    # one period is enough — the conflict is at the very start
    path_sine = sine_path(6.0, phase=0.0, length=12.0)
    path_cos = sine_path(4.0, phase=math.pi / 2.0, length=12.0)

    coordinator.setFootprint(2, *footprint)
    coordinator.setFootprint(3, *footprint)
    coordinator.setRobotMaxVelocity(2, 1.2)
    coordinator.setRobotMaxAcceleration(2, 1.0)
    coordinator.setRobotMaxVelocity(3, 1.4)
    coordinator.setRobotMaxAcceleration(3, 1.0)
    coordinator.placeRobot(2, path_cos[0].getPose())
    coordinator.placeRobot(3, path_sine[0].getPose())
    coordinator.addMissions(Mission(2, path_cos), Mission(3, path_sine))

    await asyncio.sleep(0.1)
    # precondition: at least one robot really does start inside a CS,
    # otherwise this test no longer exercises the scenario
    assert any(
        cs.getTe1Start() <= 0 or cs.getTe2Start() <= 0 for cs in coordinator.allCriticalSections
    ), "geometry drift: no critical section includes a start index"

    stop = asyncio.Event()
    monitor = asyncio.create_task(assert_no_collisions(coordinator, stop))
    try:
        await wait_until_idle(coordinator, timeout=30.0)
    finally:
        stop.set()
        await monitor
