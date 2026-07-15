"""Closed-form Reeds-Shepp curves: solve, sample, lengths.

The 48-word closed-form solution is ported from PythonRobotics
``PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py``
(Copyright (c) 2016 Atsushi Sakai, MIT licence), which itself follows the
word families of Reeds & Shepp (1990) as organized in OMPL: ``CSC``,
``CCC``, ``CCCC``, ``CCSC`` and ``CCSCC`` base words, each expanded by the
timeflip, reflect and backwards transforms. The math is kept verbatim;
naming and types are adapted to this codebase. Everything is
deterministic and pure Python.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["RSPath", "solve", "sample_path", "reverse_length"]

_ZERO = 1e-10
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class RSPath:
    """A Reeds-Shepp word: per-segment signed lengths and curvature types."""

    lengths: tuple[float, ...]  # per-segment signed lengths in METRES (< 0 = reverse)
    ctypes: tuple[str, ...]  # per-segment "L" | "S" | "R", same arity as lengths
    total_length: float  # sum of abs(lengths), metres


def _mod2pi(x: float) -> float:
    """Wrap ``x`` to ``[-pi, pi]`` (the PythonRobotics/OMPL convention)."""
    v = math.fmod(x, _TWO_PI)
    if v < -math.pi:
        v += _TWO_PI
    elif v > math.pi:
        v -= _TWO_PI
    return v


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


# Each word solver returns (ok, t, u, v) for the canonical "p" (forward-first)
# variant; the enumeration in ``_all_words`` applies timeflip / reflect /
# backwards transforms to cover all 48 words.


def _LpSpLp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= -_ZERO:
        v = _mod2pi(phi - t)
        if v >= -_ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpSpRp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 * u1
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= -_ZERO and v >= -_ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmL(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = _mod2pi(theta + 0.5 * u + math.pi)
        v = _mod2pi(phi - t + u)
        if t >= -_ZERO and u <= _ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _tau_omega(u: float, v: float, xi: float, eta: float, phi: float) -> tuple[float, float]:
    delta = _mod2pi(u - v)
    a = math.sin(u) - math.sin(delta)
    b = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * a - xi * b, xi * a + eta * b)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = _mod2pi(t1 + math.pi) if t2 < 0.0 else _mod2pi(t1)
    omega = _mod2pi(tau - u + v - phi)
    return tau, omega


def _LpRupLumRm(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = 0.25 * (2.0 + math.hypot(xi, eta))
    if rho <= 1.0:
        u = math.acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi)
        if t >= -_ZERO and v <= _ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRumLumRp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -0.5 * math.pi:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -_ZERO and v >= -_ZERO:
                return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmSmLm(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _mod2pi(theta + math.atan2(r, -2.0))
        v = _mod2pi(phi - 0.5 * math.pi - t)
        if t >= -_ZERO and u <= _ZERO and v <= _ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmSmRm(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = _mod2pi(t + 0.5 * math.pi - phi)
        if t >= -_ZERO and u <= _ZERO and v <= _ZERO:
            return True, t, u, v
    return False, 0.0, 0.0, 0.0


def _LpRmSLmRp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, _ = _polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= _ZERO:
            t = _mod2pi(math.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
            v = _mod2pi(t - phi)
            if t >= -_ZERO and v >= -_ZERO:
                return True, t, u, v
    return False, 0.0, 0.0, 0.0


_Word = tuple[tuple[float, ...], tuple[str, ...]]


def _all_words(x: float, y: float, phi: float) -> list[_Word]:
    """Every feasible word for the normalized problem, as (lengths, ctypes)
    in unit-turning-radius units. Timeflip negates (x, phi) and all
    lengths; reflect negates (y, phi) and swaps L <-> R; backwards mirrors
    the problem through the goal frame and reverses the segment order."""
    words: list[_Word] = []
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)

    def add(lengths: tuple[float, ...], ctypes: tuple[str, ...]) -> None:
        words.append((lengths, ctypes))

    # --- CSC -------------------------------------------------------------
    ok, t, u, v = _LpSpLp(x, y, phi)
    if ok:
        add((t, u, v), ("L", "S", "L"))
    ok, t, u, v = _LpSpLp(-x, y, -phi)  # timeflip
    if ok:
        add((-t, -u, -v), ("L", "S", "L"))
    ok, t, u, v = _LpSpLp(x, -y, -phi)  # reflect
    if ok:
        add((t, u, v), ("R", "S", "R"))
    ok, t, u, v = _LpSpLp(-x, -y, phi)  # timeflip + reflect
    if ok:
        add((-t, -u, -v), ("R", "S", "R"))
    ok, t, u, v = _LpSpRp(x, y, phi)
    if ok:
        add((t, u, v), ("L", "S", "R"))
    ok, t, u, v = _LpSpRp(-x, y, -phi)
    if ok:
        add((-t, -u, -v), ("L", "S", "R"))
    ok, t, u, v = _LpSpRp(x, -y, -phi)
    if ok:
        add((t, u, v), ("R", "S", "L"))
    ok, t, u, v = _LpSpRp(-x, -y, phi)
    if ok:
        add((-t, -u, -v), ("R", "S", "L"))

    # --- CCC -------------------------------------------------------------
    ok, t, u, v = _LpRmL(x, y, phi)
    if ok:
        add((t, u, v), ("L", "R", "L"))
    ok, t, u, v = _LpRmL(-x, y, -phi)
    if ok:
        add((-t, -u, -v), ("L", "R", "L"))
    ok, t, u, v = _LpRmL(x, -y, -phi)
    if ok:
        add((t, u, v), ("R", "L", "R"))
    ok, t, u, v = _LpRmL(-x, -y, phi)
    if ok:
        add((-t, -u, -v), ("R", "L", "R"))
    # backwards
    ok, t, u, v = _LpRmL(xb, yb, phi)
    if ok:
        add((v, u, t), ("L", "R", "L"))
    ok, t, u, v = _LpRmL(-xb, yb, -phi)
    if ok:
        add((-v, -u, -t), ("L", "R", "L"))
    ok, t, u, v = _LpRmL(xb, -yb, -phi)
    if ok:
        add((v, u, t), ("R", "L", "R"))
    ok, t, u, v = _LpRmL(-xb, -yb, phi)
    if ok:
        add((-v, -u, -t), ("R", "L", "R"))

    # --- CCCC ------------------------------------------------------------
    ok, t, u, v = _LpRupLumRm(x, y, phi)
    if ok:
        add((t, u, -u, v), ("L", "R", "L", "R"))
    ok, t, u, v = _LpRupLumRm(-x, y, -phi)
    if ok:
        add((-t, -u, u, -v), ("L", "R", "L", "R"))
    ok, t, u, v = _LpRupLumRm(x, -y, -phi)
    if ok:
        add((t, u, -u, v), ("R", "L", "R", "L"))
    ok, t, u, v = _LpRupLumRm(-x, -y, phi)
    if ok:
        add((-t, -u, u, -v), ("R", "L", "R", "L"))
    ok, t, u, v = _LpRumLumRp(x, y, phi)
    if ok:
        add((t, u, u, v), ("L", "R", "L", "R"))
    ok, t, u, v = _LpRumLumRp(-x, y, -phi)
    if ok:
        add((-t, -u, -u, -v), ("L", "R", "L", "R"))
    ok, t, u, v = _LpRumLumRp(x, -y, -phi)
    if ok:
        add((t, u, u, v), ("R", "L", "R", "L"))
    ok, t, u, v = _LpRumLumRp(-x, -y, phi)
    if ok:
        add((-t, -u, -u, -v), ("R", "L", "R", "L"))

    # --- CCSC ------------------------------------------------------------
    half_pi = 0.5 * math.pi
    ok, t, u, v = _LpRmSmLm(x, y, phi)
    if ok:
        add((t, -half_pi, u, v), ("L", "R", "S", "L"))
    ok, t, u, v = _LpRmSmLm(-x, y, -phi)
    if ok:
        add((-t, half_pi, -u, -v), ("L", "R", "S", "L"))
    ok, t, u, v = _LpRmSmLm(x, -y, -phi)
    if ok:
        add((t, -half_pi, u, v), ("R", "L", "S", "R"))
    ok, t, u, v = _LpRmSmLm(-x, -y, phi)
    if ok:
        add((-t, half_pi, -u, -v), ("R", "L", "S", "R"))
    ok, t, u, v = _LpRmSmRm(x, y, phi)
    if ok:
        add((t, -half_pi, u, v), ("L", "R", "S", "R"))
    ok, t, u, v = _LpRmSmRm(-x, y, -phi)
    if ok:
        add((-t, half_pi, -u, -v), ("L", "R", "S", "R"))
    ok, t, u, v = _LpRmSmRm(x, -y, -phi)
    if ok:
        add((t, -half_pi, u, v), ("R", "L", "S", "L"))
    ok, t, u, v = _LpRmSmRm(-x, -y, phi)
    if ok:
        add((-t, half_pi, -u, -v), ("R", "L", "S", "L"))
    # backwards
    ok, t, u, v = _LpRmSmLm(xb, yb, phi)
    if ok:
        add((v, u, -half_pi, t), ("L", "S", "R", "L"))
    ok, t, u, v = _LpRmSmLm(-xb, yb, -phi)
    if ok:
        add((-v, -u, half_pi, -t), ("L", "S", "R", "L"))
    ok, t, u, v = _LpRmSmLm(xb, -yb, -phi)
    if ok:
        add((v, u, -half_pi, t), ("R", "S", "L", "R"))
    ok, t, u, v = _LpRmSmLm(-xb, -yb, phi)
    if ok:
        add((-v, -u, half_pi, -t), ("R", "S", "L", "R"))
    ok, t, u, v = _LpRmSmRm(xb, yb, phi)
    if ok:
        add((v, u, -half_pi, t), ("R", "S", "R", "L"))
    ok, t, u, v = _LpRmSmRm(-xb, yb, -phi)
    if ok:
        add((-v, -u, half_pi, -t), ("R", "S", "R", "L"))
    ok, t, u, v = _LpRmSmRm(xb, -yb, -phi)
    if ok:
        add((v, u, -half_pi, t), ("L", "S", "L", "R"))
    ok, t, u, v = _LpRmSmRm(-xb, -yb, phi)
    if ok:
        add((-v, -u, half_pi, -t), ("L", "S", "L", "R"))

    # --- CCSCC -----------------------------------------------------------
    ok, t, u, v = _LpRmSLmRp(x, y, phi)
    if ok:
        add((t, -half_pi, u, -half_pi, v), ("L", "R", "S", "L", "R"))
    ok, t, u, v = _LpRmSLmRp(-x, y, -phi)
    if ok:
        add((-t, half_pi, -u, half_pi, -v), ("L", "R", "S", "L", "R"))
    ok, t, u, v = _LpRmSLmRp(x, -y, -phi)
    if ok:
        add((t, -half_pi, u, -half_pi, v), ("R", "L", "S", "R", "L"))
    ok, t, u, v = _LpRmSLmRp(-x, -y, phi)
    if ok:
        add((-t, half_pi, -u, half_pi, -v), ("R", "L", "S", "R", "L"))

    return words


def solve(
    q0: tuple[float, float, float], q1: tuple[float, float, float], turning_radius: float
) -> RSPath:
    """Shortest Reeds-Shepp path q0 -> q1 (poses as (x, y, theta))."""
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    c, s = math.cos(q0[2]), math.sin(q0[2])
    # normalize to the start frame and unit turning radius
    x = (c * dx + s * dy) / turning_radius
    y = (-s * dx + c * dy) / turning_radius
    phi = _mod2pi(q1[2] - q0[2])
    if math.hypot(x, y) < _ZERO and abs(phi) < _ZERO:
        return RSPath((), (), 0.0)
    best: _Word | None = None
    best_length = math.inf
    for lengths, ctypes in _all_words(x, y, phi):
        length = sum(abs(l) for l in lengths)
        if length < best_length - _ZERO:
            best_length = length
            best = (lengths, ctypes)
    assert best is not None, "no Reeds-Shepp word found (unreachable for distinct poses)"
    # drop zero-length segments and scale back to metres
    lengths_m = tuple(l * turning_radius for l in best[0])
    kept = [(l, ct) for l, ct in zip(lengths_m, best[1]) if abs(l) > _ZERO * turning_radius]
    return RSPath(
        tuple(l for l, _ in kept),
        tuple(ct for _, ct in kept),
        sum(abs(l) for l, _ in kept),
    )


def _advance(
    pose: tuple[float, float, float], d: float, ctype: str, turning_radius: float
) -> tuple[float, float, float]:
    """Pose after driving signed arc length ``d`` metres along one segment."""
    x, y, theta = pose
    if ctype == "S":
        return x + d * math.cos(theta), y + d * math.sin(theta), theta
    phi = d / turning_radius
    if ctype == "L":
        return (
            x + turning_radius * (math.sin(theta + phi) - math.sin(theta)),
            y - turning_radius * (math.cos(theta + phi) - math.cos(theta)),
            theta + phi,
        )
    return (
        x - turning_radius * (math.sin(theta - phi) - math.sin(theta)),
        y + turning_radius * (math.cos(theta - phi) - math.cos(theta)),
        theta - phi,
    )


def _wrap(theta: float) -> float:
    """Normalize an angle to ``[-pi, pi)``."""
    return (theta + math.pi) % _TWO_PI - math.pi


def sample_path(
    q0: tuple[float, float, float], path: RSPath, turning_radius: float, step: float
) -> list[tuple[float, float, float, int]]:
    """Poses along the path every `step` metres of arc length, as
    (x, y, theta, gear) with gear +1 forward / -1 reverse. Includes the
    start pose and the exact endpoint. theta normalized to [-pi, pi)."""
    if not path.lengths:
        return [(q0[0], q0[1], _wrap(q0[2]), 1)]
    samples: list[tuple[float, float, float, int]] = []
    seg_start = (q0[0], q0[1], q0[2])
    for length, ctype in zip(path.lengths, path.ctypes):
        gear = 1 if length >= 0.0 else -1
        arc = abs(length)
        n = max(1, math.ceil(arc / step))
        first = 0 if not samples else 1  # segment start duplicates the previous end
        for i in range(first, n + 1):
            d = math.copysign(min(i * step, arc), length)
            x, y, theta = _advance(seg_start, d, ctype, turning_radius)
            samples.append((x, y, _wrap(theta), gear))
        seg_start = _advance(seg_start, length, ctype, turning_radius)
    return samples


def reverse_length(path: RSPath) -> float:
    """Sum of abs(length) over segments with negative length."""
    return sum(-l for l in path.lengths if l < 0.0)
