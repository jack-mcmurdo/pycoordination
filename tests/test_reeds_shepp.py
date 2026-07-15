"""Closed-form Reeds-Shepp curves: endpoint accuracy, bounds, symmetry."""

from __future__ import annotations

import math

import pytest

from coordination_oru.motionplanning import reeds_shepp

Q0 = (0.0, 0.0, 0.0)
GRID_XY = [-4.0, -1.0, 0.5, 3.0]
GRID_THETA = [0.0, 1.2, math.pi / 2, -2.5]
GRID = [(x, y, theta) for x in GRID_XY for y in GRID_XY for theta in GRID_THETA]


def _angle_diff(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def test_endpoint_accuracy() -> None:
    for q1 in GRID:
        path = reeds_shepp.solve(Q0, q1, 1.0)
        samples = reeds_shepp.sample_path(Q0, path, 1.0, 0.1)
        x, y, theta, _ = samples[-1]
        assert abs(x - q1[0]) < 1e-6, q1
        assert abs(y - q1[1]) < 1e-6, q1
        assert _angle_diff(theta, q1[2]) < 1e-6, q1


def test_straight_cases() -> None:
    forward = reeds_shepp.solve(Q0, (5.0, 0.0, 0.0), 1.0)
    assert forward.total_length == pytest.approx(5.0)

    backward = reeds_shepp.solve(Q0, (-3.0, 0.0, 0.0), 1.0)
    assert backward.total_length == pytest.approx(3.0)
    assert reeds_shepp.reverse_length(backward) == pytest.approx(3.0)


def test_lower_bound() -> None:
    for q1 in GRID:
        path = reeds_shepp.solve(Q0, q1, 1.0)
        assert path.total_length >= math.hypot(q1[0], q1[1]) - 1e-9, q1


def test_symmetry() -> None:
    for q1 in GRID[::13][:5]:
        forward = reeds_shepp.solve(Q0, q1, 1.0)
        backward = reeds_shepp.solve(q1, Q0, 1.0)
        assert forward.total_length == pytest.approx(backward.total_length), q1


def test_radius_scaling() -> None:
    scaled = reeds_shepp.solve(Q0, (0.0, 4.0, math.pi), 2.0)
    unit = reeds_shepp.solve(Q0, (0.0, 2.0, math.pi), 1.0)
    assert scaled.total_length == pytest.approx(2.0 * unit.total_length)


def test_gear_flags() -> None:
    path = reeds_shepp.solve(Q0, (-3.0, 0.0, 0.0), 1.0)
    samples = reeds_shepp.sample_path(Q0, path, 1.0, 0.1)
    assert all(gear == -1 for _, _, _, gear in samples)
