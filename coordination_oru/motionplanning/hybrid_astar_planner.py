"""Built-in Hybrid A* motion planner producing Reeds-Shepp-style car paths."""

from __future__ import annotations

import heapq
import itertools
import math
from typing import TYPE_CHECKING

import numpy as np
import numpy.typing as npt
import shapely

from coordination_oru.metacsp.spatial.pose import Pose, PoseSteering
from coordination_oru.motionplanning import reeds_shepp
from coordination_oru.motionplanning.abstract_motion_planner import AbstractMotionPlanner
from coordination_oru.motionplanning.occupancy_map import OccupancyMap

if TYPE_CHECKING:
    from shapely.geometry.base import BaseGeometry

__all__ = ["HybridAStarPlanner"]

_TWO_PI = 2.0 * math.pi

# search-internal aliases
_Key = tuple[int, int, int]  # (row, col, theta_bin)
_State = tuple[float, float, float, int]  # (x, y, theta, gear)
_Sample = tuple[float, float, float, int]  # pose sample + gear


def _wrap(theta: float) -> float:
    """Normalize an angle to ``[-pi, pi)``."""
    return (theta + math.pi) % _TWO_PI - math.pi


class HybridAStarPlanner(AbstractMotionPlanner):
    """Hybrid A* over a ROS-style occupancy grid, car-like Reeds-Shepp model.

    The robot is a car with minimum turning radius ``turning_radius`` that
    may drive forward and in reverse. Collision checking uses the robot's
    circumcircle (the smallest origin-centered circle containing the
    footprint) against the grid inflated by that radius, so validity of a
    state is heading-independent. Start and goal **theta are honored**:
    termination is by a collision-checked analytic Reeds-Shepp expansion
    exactly onto the goal pose.

    Reverse arcs cost ``reverse_cost x`` their length plus
    ``gear_switch_cost`` per direction change (``reverse_cost >= 1`` keeps
    the Reeds-Shepp heuristic admissible). ``heuristic_inflation > 1``
    trades optimality for speed. Output poses carry
    ``PoseSteering.steering == 0.0``. Planning is fully deterministic: two
    identical ``plan()`` calls return identical paths. Pure Python — large
    maps (millions of cells) will be slow.
    """

    def __init__(
        self,
        occupancy_map: OccupancyMap,
        *,
        turning_radius: float = 1.0,
        path_step: float = 0.25,
        prim_step: float = 0.5,
        reverse_cost: float = 1.5,
        gear_switch_cost: float = 1.0,
        angle_bins: int = 72,
        heuristic_inflation: float = 1.3,
        max_expansions: int = 100_000,
    ) -> None:
        super().__init__()
        if reverse_cost < 1.0:
            raise ValueError("reverse_cost must be >= 1.0 (heuristic admissibility)")
        if turning_radius <= 0.0:
            raise ValueError("turning_radius must be positive")
        self._map = occupancy_map
        self.turning_radius = turning_radius
        self.path_step = path_step
        self.prim_step = prim_step
        self.reverse_cost = reverse_cost
        self.gear_switch_cost = gear_switch_cost
        self.angle_bins = angle_bins
        self.heuristic_inflation = heuristic_inflation
        self.max_expansions = max_expansions

    def _circumradius(self) -> float:
        if not self.footprintCoords:
            raise RuntimeError("setFootprint(...) before planning")
        return max(math.hypot(x, y) for x, y in self.footprintCoords)

    # ------------------------------------------------------------ planning

    def doPlanning(self) -> bool:
        if self.start is None or not self.goal:
            raise RuntimeError("setStart(...) and setGoals(...) before planning")
        r = self._circumradius()
        grid = self._map.inflated(r)
        obstacles = self.getObstacles()
        if obstacles:
            grid = grid.copy()
            for g in obstacles:
                self._rasterize_obstacle(grid, g, r)

        chain: list[tuple[float, float, float]] = []
        poses = [self.start, *self.goal]
        for p0, p1 in zip(poses, poses[1:]):
            segment = self._plan_segment(grid, p0, p1)
            if segment is None:
                self.pathPS = None
                return False
            chain.extend(segment if not chain else segment[1:])
        self.pathPS = tuple(PoseSteering(Pose(x, y, th), 0.0) for (x, y, th) in chain)
        return True

    def _rasterize_obstacle(
        self, grid: npt.NDArray[np.bool_], geometry: "BaseGeometry", radius: float
    ) -> None:
        """Mark occupied every cell whose center lies within ``radius`` of
        the obstacle (i.e. inside the obstacle buffered by the circumradius)."""
        m = self._map
        buf = geometry.buffer(radius)
        minx, miny, maxx, maxy = buf.bounds
        bx0, by0, bx1, by1 = m.bounds
        if maxx < bx0 or minx > bx1 or maxy < by0 or miny > by1:
            return
        r0, c0 = m.world_to_grid(max(minx, bx0), max(miny, by0))
        r1, c1 = m.world_to_grid(min(maxx, bx1), min(maxy, by1))
        r0, c0 = max(r0, 0), max(c0, 0)
        r1, c1 = min(r1, m.height - 1), min(c1, m.width - 1)
        if r0 > r1 or c0 > c1:
            return
        xs = m.origin[0] + (np.arange(c0, c1 + 1) + 0.5) * m.resolution
        ys = m.origin[1] + (np.arange(r0, r1 + 1) + 0.5) * m.resolution
        xx, yy = np.meshgrid(xs, ys)
        grid[r0 : r1 + 1, c0 : c1 + 1] |= shapely.contains_xy(buf, xx, yy)

    def _theta_bin(self, theta: float) -> int:
        return int(((theta + math.pi) / _TWO_PI) * self.angle_bins) % self.angle_bins

    def _distance_field(
        self, grid: npt.NDArray[np.bool_], goal_cell: tuple[int, int]
    ) -> npt.NDArray[np.float64]:
        """8-connected Dijkstra distance (metres) from the goal cell over
        free cells; unreachable cells stay ``inf``."""
        res = self._map.resolution
        height, width = grid.shape
        dist = np.full((height, width), np.inf)
        occ = grid  # bool: True = blocked
        counter = itertools.count()
        heap: list[tuple[float, int, tuple[int, int]]] = [(0.0, next(counter), goal_cell)]
        dist[goal_cell] = 0.0
        diag = res * math.sqrt(2.0)
        neighbours = (
            (-1, -1, diag), (-1, 0, res), (-1, 1, diag),
            (0, -1, res), (0, 1, res),
            (1, -1, diag), (1, 0, res), (1, 1, diag),
        )
        while heap:
            d, _, (row, col) = heapq.heappop(heap)
            if d > dist[row, col]:
                continue
            for dr, dc, step in neighbours:
                nr, nc = row + dr, col + dc
                if 0 <= nr < height and 0 <= nc < width and not occ[nr, nc]:
                    nd = d + step
                    if nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        heapq.heappush(heap, (nd, next(counter), (nr, nc)))
        return dist

    def _check_primitive(
        self,
        grid: npt.NDArray[np.bool_],
        x: float,
        y: float,
        theta: float,
        direction: int,
        curvature: float,
    ) -> list[_Sample] | None:
        """Collision-checked samples (every res/2 of arc length, endpoint
        included) of one primitive, or ``None`` if it collides."""
        m = self._map
        step = m.resolution / 2.0
        n = max(1, math.ceil(self.prim_step / step))
        samples: list[_Sample] = []
        for i in range(1, n + 1):
            d = direction * min(i * step, self.prim_step)
            if curvature == 0.0:
                sx, sy, st = x + d * math.cos(theta), y + d * math.sin(theta), theta
            else:
                st = theta + d * curvature
                sx = x + (math.sin(st) - math.sin(theta)) / curvature
                sy = y - (math.cos(st) - math.cos(theta)) / curvature
            row, col = m.world_to_grid(sx, sy)
            if not (0 <= row < m.height and 0 <= col < m.width) or grid[row, col]:
                return None
            samples.append((sx, sy, _wrap(st), direction))
        return samples

    def _rs_shot(
        self, grid: npt.NDArray[np.bool_], state: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> list[_Sample] | None:
        """Collision-checked analytic Reeds-Shepp expansion onto the goal."""
        m = self._map
        path = reeds_shepp.solve(state, goal, self.turning_radius)
        samples = reeds_shepp.sample_path(state, path, self.turning_radius, m.resolution / 2.0)
        for sx, sy, _, _ in samples:
            row, col = m.world_to_grid(sx, sy)
            if not (0 <= row < m.height and 0 <= col < m.width) or grid[row, col]:
                return None
        return samples

    def _plan_segment(
        self, grid: npt.NDArray[np.bool_], p0: Pose, p1: Pose
    ) -> list[tuple[float, float, float]] | None:
        m = self._map
        R = self.turning_radius
        start = (p0.x, p0.y, _wrap(p0.theta))
        goal = (p1.x, p1.y, _wrap(p1.theta))
        bin_width = _TWO_PI / self.angle_bins
        for x, y, _ in (start, goal):
            row, col = m.world_to_grid(x, y)
            if not m.in_bounds(row, col) or grid[row, col]:
                return None
        if (
            math.hypot(goal[0] - start[0], goal[1] - start[1]) < 1e-9
            and abs(_wrap(goal[2] - start[2])) < bin_width
        ):
            # degenerate segment: nothing to drive
            return None

        goal_cell = m.world_to_grid(goal[0], goal[1])
        dist2d = self._distance_field(grid, goal_cell)
        start_cell = m.world_to_grid(start[0], start[1])
        if not np.isfinite(dist2d[start_cell]):
            return None  # goal unreachable even holonomically

        def heuristic(x: float, y: float, theta: float) -> float:
            e = math.hypot(goal[0] - x, goal[1] - y)
            # Euclidean distance is a valid lower bound for the RS length,
            # so the exact solve is only worth its cost near the goal.
            rs = reeds_shepp.solve((x, y, theta), goal, R).total_length if e <= 6.0 * R else e
            row, col = m.world_to_grid(x, y)
            return self.heuristic_inflation * max(float(dist2d[row, col]), rs)

        start_key = (start_cell[0], start_cell[1], self._theta_bin(start[2]))
        g_cost: dict[_Key, float] = {start_key: 0.0}
        entry: dict[_Key, _State] = {start_key: (start[0], start[1], start[2], 0)}
        parent: dict[_Key, _Key | None] = {start_key: None}
        parent_motion: dict[_Key, list[_Sample]] = {
            start_key: [(start[0], start[1], start[2], 0)]
        }
        counter = itertools.count()
        heap: list[tuple[float, int, float, _Key]] = [
            (heuristic(*start[:3]), next(counter), 0.0, start_key)
        ]
        curvatures = (1.0 / R, 0.0, -1.0 / R)
        goal_bin = self._theta_bin(goal[2])
        pops = 0

        def reconstruct(key: _Key, shot: list[_Sample]) -> list[tuple[float, float, float]]:
            motions: list[list[_Sample]] = []
            walk: _Key | None = key
            while walk is not None:
                motions.append(parent_motion[walk])
                walk = parent[walk]
            samples: list[_Sample] = []
            for motion in reversed(motions):
                samples.extend(motion)
            samples.extend(shot)
            deduped: list[_Sample] = []
            for s in samples:
                if deduped and abs(s[0] - deduped[-1][0]) < 1e-12 and abs(s[1] - deduped[-1][1]) < 1e-12 and abs(s[2] - deduped[-1][2]) < 1e-12:
                    continue
                deduped.append(s)
            return self._resample(deduped)

        while heap:
            _, _, g, key = heapq.heappop(heap)
            if g > g_cost.get(key, math.inf):
                continue  # stale entry
            pops += 1
            if pops > self.max_expansions:
                return None
            x, y, theta, gear = entry[key]

            # analytic expansion onto the exact goal pose
            e = math.hypot(goal[0] - x, goal[1] - y)
            if e <= 3.0 * R or pops % 20 == 0:
                shot = self._rs_shot(grid, (x, y, theta), goal)
                if shot is not None:
                    return reconstruct(key, shot)

            # near-goal fallback (RS shot blocked but we are already there)
            if e <= self.path_step and (key[2] - goal_bin) % self.angle_bins in (0, 1, self.angle_bins - 1):
                return reconstruct(key, [(goal[0], goal[1], goal[2], gear if gear != 0 else 1)])

            for direction in (1, -1):
                for curvature in curvatures:
                    samples = self._check_primitive(grid, x, y, theta, direction, curvature)
                    if samples is None:
                        continue
                    nx, ny, ntheta, _ = samples[-1]
                    nkey = (*m.world_to_grid(nx, ny), self._theta_bin(ntheta))
                    cost = self.prim_step * (1.0 if direction > 0 else self.reverse_cost)
                    if gear != 0 and direction != gear:
                        cost += self.gear_switch_cost
                    ng = g + cost
                    if ng >= g_cost.get(nkey, math.inf):
                        continue
                    h = heuristic(nx, ny, ntheta)
                    if not math.isfinite(h):
                        continue
                    g_cost[nkey] = ng
                    entry[nkey] = (nx, ny, ntheta, direction)
                    parent[nkey] = key
                    parent_motion[nkey] = samples
                    heapq.heappush(heap, (ng + h, next(counter), ng, nkey))
        return None

    def _resample(self, samples: list[_Sample]) -> list[tuple[float, float, float]]:
        """Thin the ~res/2-spaced samples to one pose per ``path_step`` of
        arc length, always keeping the first pose, the last pose and every
        gear flip so cusps survive. Headings come from the sampled states
        (reversing means heading != travel direction)."""
        if not samples:
            return []
        out: list[tuple[float, float, float]] = [samples[0][:3]]
        since_kept = 0.0
        for prev, cur in zip(samples, samples[1:]):
            since_kept += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            gear_flip = prev[3] != 0 and cur[3] != 0 and cur[3] != prev[3]
            if gear_flip or since_kept >= self.path_step:
                out.append(cur[:3])
                since_kept = 0.0
        last = samples[-1][:3]
        if out[-1] != last:
            out.append(last)
        return out
