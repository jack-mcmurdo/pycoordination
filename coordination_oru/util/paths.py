"""Demo paths: bundled ``.path`` files and synthetic generators.

The bundled files (``debug1/2/3.path``) are recorded planner outputs from the
original Java ``coordination_oru`` repo, shipped as package data so they load
from an installed wheel. The synthetic helpers generate deterministic paths in
code. Each helper returns a tuple of ``PoseSteering`` ready to feed into a
``Mission``.
"""

from __future__ import annotations

import math
import pathlib
from importlib import resources
from typing import Iterable

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering


def _parse_path_lines(lines: Iterable[str], source: str) -> tuple[PoseSteering, ...]:
    """Parse ``x y theta [steering]`` lines (blank/# lines ignored)."""
    out: list[PoseSteering] = []
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if len(parts) < 3:
            continue
        x, y, theta = float(parts[0]), float(parts[1]), float(parts[2])
        steering = float(parts[3]) if len(parts) > 3 else 0.0
        out.append(PoseSteering(Pose(x, y, theta), steering))
    if not out:
        raise ValueError(f"no waypoints parsed from {source}")
    return tuple(out)


def load_path(file: pathlib.Path) -> tuple[PoseSteering, ...]:
    """Load a coordination_oru ``.path`` file from an arbitrary location."""
    with file.open() as f:
        return _parse_path_lines(f, str(file))


def load_path_file(name: str) -> tuple[PoseSteering, ...]:
    """Load a bundled ``.path`` file (e.g. ``"debug1.path"``) from package data."""
    text = resources.files("coordination_oru.data").joinpath(name).read_text()
    return _parse_path_lines(text.splitlines(), f"coordination_oru.data/{name}")


def line_path(
    x0: float, y0: float, x1: float, y1: float, *, step: float = 0.5
) -> tuple[PoseSteering, ...]:
    """Sample a straight segment from (x0,y0) to (x1,y1) at ``step``-metre spacing."""
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length == 0:
        return (PoseSteering(Pose(x0, y0, 0.0)),)
    n = max(2, int(math.ceil(length / step)) + 1)
    theta = math.atan2(dy, dx)
    out: list[PoseSteering] = []
    for i in range(n):
        t = i / (n - 1)
        out.append(PoseSteering(Pose(x0 + t * dx, y0 + t * dy, theta)))
    return tuple(out)


def two_robot_cross() -> tuple[tuple[PoseSteering, ...], tuple[PoseSteering, ...]]:
    """Two perpendicular straight paths that cross at the origin."""
    return (
        line_path(-5.0, 0.0, 5.0, 0.0),
        line_path(0.0, -5.0, 0.0, 5.0),
    )


def three_robot_intersection() -> tuple[
    tuple[PoseSteering, ...], tuple[PoseSteering, ...], tuple[PoseSteering, ...]
]:
    """Three paths that all pass through the origin from different directions."""
    return (
        line_path(-8.0, 0.0, 8.0, 0.0),  # west → east
        line_path(0.0, -8.0, 0.0, 8.0),  # south → north
        line_path(-6.0, -6.0, 6.0, 6.0),  # SW → NE diagonal
    )


def shuttle_path(start: tuple[float, float], end: tuple[float, float]) -> tuple[PoseSteering, ...]:
    return line_path(start[0], start[1], end[0], end[1])


def sine_path(
    y_offset: float,
    *,
    amplitude: float = 3.0,
    phase: float = 0.0,
    length: float = 24.0,
    period: float = 12.0,
    step: float = 0.25,
) -> tuple[PoseSteering, ...]:
    """Sample ``y = y_offset + amplitude * sin(2πx/period + phase)`` from
    x = 0 to ``length`` at ``step``-metre x-spacing, headings tangent to
    the curve."""
    k = 2.0 * math.pi / period
    n = max(2, int(math.ceil(length / step)) + 1)
    out: list[PoseSteering] = []
    for i in range(n):
        x = i * (length / (n - 1))
        y = y_offset + amplitude * math.sin(k * x + phase)
        theta = math.atan2(amplitude * k * math.cos(k * x + phase), 1.0)
        out.append(PoseSteering(Pose(x, y, theta)))
    return tuple(out)
