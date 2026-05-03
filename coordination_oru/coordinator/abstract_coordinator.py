"""Multi-robot trajectory-envelope coordinator.

This is the runtime brain. It:

* Owns a :class:`TrajectoryEnvelopeSolver` (the STP-aware envelope registry).
* Accepts a :class:`Mission` per robot and turns it into an envelope.
* Periodically (re)detects pairwise critical sections via shapely.
* Decides priority on each new CS using the closest-to-CS-entry heuristic.
* Builds a waiting graph and uses :func:`networkx.find_cycle` to spot
  deadlocks; if a cycle appears, swap priority on one of its edges to break
  it (a minimal but workable resolution policy).
* Publishes ``permit_index_until`` to each tracker so robots stop short of
  any CS where they don't have priority and the other robot hasn't cleared.

Subclasses customise tracker construction (e.g. a real-robot driver vs. the
simulator). The base class is fully async, takes no global locks beyond a
single ``asyncio.Lock`` around the coordination cycle, and keeps state in
plain dicts.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

import networkx as nx

from coordination_oru.coordinator.critical_section import CriticalSection
from coordination_oru.coordinator.mission import Mission
from coordination_oru.coordinator.robot_report import RobotReport
from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope
from coordination_oru.metacsp.spatial.trajectory_envelope_solver import (
    TrajectoryEnvelopeSolver,
)
from coordination_oru.util.logging import get_logger

if TYPE_CHECKING:
    from coordination_oru.coordinator.abstract_tracker import (
        AbstractTrajectoryEnvelopeTracker,
    )


COORDINATION_PERIOD: float = 0.05  # seconds


class AbstractTrajectoryEnvelopeCoordinator:
    """Drive a fleet of trajectory envelopes safely through critical sections."""

    def __init__(self, period: float = COORDINATION_PERIOD) -> None:
        self.solver = TrajectoryEnvelopeSolver()
        self.period = period

        # registries keyed by robot_id (one active envelope per robot at a time)
        self._trackers: dict[int, "AbstractTrajectoryEnvelopeTracker"] = {}
        self._envelopes: dict[int, TrajectoryEnvelope] = {}
        self._reports: dict[int, RobotReport] = {}

        # critical-section state
        self._critical_sections: list[CriticalSection] = []
        # cs.key -> envelope_id of the envelope that has priority through this CS
        self._priority: dict[frozenset[int], int] = {}

        self._lock = asyncio.Lock()
        self._loop_task: asyncio.Task[None] | None = None
        self._running = False

        self.log = get_logger(__name__)

    # ----------------------------------------------------------- public API

    def submit_mission(self, mission: Mission) -> TrajectoryEnvelope:
        """Convert the mission into an envelope and register it as active."""
        envelope = self.solver.create_envelope(
            robot_id=mission.robot_id,
            path=mission.path,
            footprint=mission.footprint,
        )
        self._envelopes[mission.robot_id] = envelope
        self.log.info(
            "mission_submitted",
            mission_id=mission.mission_id,
            robot_id=mission.robot_id,
            envelope_id=envelope.envelope_id,
            length=envelope.length,
        )
        return envelope

    def register_tracker(self, tracker: "AbstractTrajectoryEnvelopeTracker") -> None:
        if tracker.robot_id in self._trackers:
            raise RuntimeError(f"tracker for robot {tracker.robot_id} already registered")
        self._trackers[tracker.robot_id] = tracker

    def get_envelope(self, robot_id: int) -> TrajectoryEnvelope | None:
        return self._envelopes.get(robot_id)

    @property
    def critical_sections(self) -> list[CriticalSection]:
        return list(self._critical_sections)

    @property
    def priorities(self) -> dict[frozenset[int], int]:
        return dict(self._priority)

    # ------------------------------------------------------------ lifecycle

    async def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._loop_task = asyncio.create_task(self._coordination_loop(), name="coordinator-loop")

    async def stop(self) -> None:
        self._running = False
        if self._loop_task is not None:
            self._loop_task.cancel()
            try:
                await self._loop_task
            except asyncio.CancelledError:
                pass
            self._loop_task = None

    # ------------------------------------------------------- report ingress

    async def on_robot_report(self, report: RobotReport) -> None:
        async with self._lock:
            self._reports[report.robot_id] = report
            envelope = self._envelopes.get(report.robot_id)
            if envelope is None:
                return
            if report.completed or report.path_index >= envelope.length - 1:
                if not envelope.completed:
                    self.solver.mark_completed(envelope.envelope_id)
                    self.log.info(
                        "envelope_completed",
                        robot_id=report.robot_id,
                        envelope_id=envelope.envelope_id,
                    )
                    # forget priority entries that referenced it
                    self._priority = {
                        k: v
                        for k, v in self._priority.items()
                        if envelope.envelope_id not in k
                    }
                    self._critical_sections = [
                        cs
                        for cs in self._critical_sections
                        if envelope.envelope_id not in cs.key
                    ]

    # ------------------------------------------------------------ main loop

    async def _coordination_loop(self) -> None:
        try:
            while self._running:
                async with self._lock:
                    self._recompute_critical_sections()
                    self._update_permits()
                await asyncio.sleep(self.period)
        except asyncio.CancelledError:
            raise

    # ---------------------------------------------------------- CS detection

    def _recompute_critical_sections(self) -> None:
        envelopes = self.solver.envelopes()
        new_css = find_critical_sections(envelopes)

        # Carry over priority decisions for surviving CSes; decide for new ones.
        new_priority: dict[frozenset[int], int] = {}
        for cs in new_css:
            key = cs.key
            if key in self._priority:
                new_priority[key] = self._priority[key]
            else:
                new_priority[key] = self._decide_priority(cs)

        self._critical_sections = new_css
        self._priority = new_priority

        self._break_deadlock_if_any()

    def _decide_priority(self, cs: CriticalSection) -> int:
        """Closest-to-CS-entry wins. Ties broken by lower robot id."""
        report_a = self._reports.get(cs.envelope_a.robot_id)
        report_b = self._reports.get(cs.envelope_b.robot_id)
        idx_a = report_a.path_index if report_a is not None else 0
        idx_b = report_b.path_index if report_b is not None else 0
        dist_a = max(0, cs.a_start - idx_a)
        dist_b = max(0, cs.b_start - idx_b)
        if dist_a < dist_b:
            return cs.envelope_a.envelope_id
        if dist_b < dist_a:
            return cs.envelope_b.envelope_id
        # tie-break on robot id (deterministic)
        return (
            cs.envelope_a.envelope_id
            if cs.envelope_a.robot_id <= cs.envelope_b.robot_id
            else cs.envelope_b.envelope_id
        )

    def _break_deadlock_if_any(self) -> None:
        """Detect a cycle in the waiting graph; if found, swap one priority.

        Edge ``A -> B`` means "A is waiting for B". A cycle ⇒ deadlock.
        We resolve by flipping priority on the most recently added CS in the
        cycle (deterministic via sorting).
        """
        graph = nx.DiGraph()
        for cs in self._critical_sections:
            winner_id = self._priority[cs.key]
            loser = cs.envelope_a if cs.envelope_b.envelope_id == winner_id else cs.envelope_b
            winner = cs.envelope_b if loser is cs.envelope_a else cs.envelope_a
            graph.add_edge(loser.envelope_id, winner.envelope_id, cs_key=cs.key)
        try:
            cycle = nx.find_cycle(graph, orientation="original")
        except nx.NetworkXNoCycle:
            return
        # cycle is a list of (u, v, key) edges; pick the lexicographically max
        # cs_key edge to flip for determinism.
        edge = max(cycle, key=lambda e: tuple(sorted(e[2]["cs_key"])))
        flipped_key: frozenset[int] = edge[2]["cs_key"]
        envelope_ids = list(flipped_key)
        current_winner = self._priority[flipped_key]
        new_winner = next(eid for eid in envelope_ids if eid != current_winner)
        self._priority[flipped_key] = new_winner
        self.log.warning(
            "deadlock_resolved_by_swap",
            cs=tuple(sorted(flipped_key)),
            old_winner=current_winner,
            new_winner=new_winner,
        )

    # -------------------------------------------------------- permits update

    def _update_permits(self) -> None:
        """For each tracker, set ``permit_index_until`` based on active CSes."""
        for robot_id, tracker in self._trackers.items():
            envelope = self._envelopes.get(robot_id)
            if envelope is None or envelope.completed:
                continue
            permit = envelope.length - 1
            for cs in self._critical_sections:
                if envelope.envelope_id not in cs.key:
                    continue
                self_start, _self_end = cs.cs_range_for(envelope.envelope_id)
                other = cs.other(envelope.envelope_id)
                _other_start, other_end = cs.cs_range_for(other.envelope_id)
                priority_winner = self._priority[cs.key]
                if priority_winner == envelope.envelope_id:
                    continue  # we own this CS, no restriction from it
                # We must hold short of self_start until the other has cleared
                # past other_end.
                other_report = self._reports.get(other.robot_id)
                other_passed = (
                    other.completed
                    or (other_report is not None and other_report.path_index > other_end)
                )
                if not other_passed:
                    permit = min(permit, max(0, self_start - 1))
            tracker.permit_index_until = permit


# --------------------------------------------------------------- detection


def find_critical_sections(
    envelopes: list[TrajectoryEnvelope],
) -> list[CriticalSection]:
    """Detect all pairwise critical sections among the given envelopes.

    For each pair of envelopes whose swept-area polygons intersect, find every
    pair of waypoint indices ``(k, l)`` whose per-waypoint footprints
    intersect, then group those into 8-neighbourhood-connected components in
    index space. Each component becomes one CS bounded by the min/max indices
    along each axis.
    """
    sections: list[CriticalSection] = []
    n = len(envelopes)
    for i in range(n):
        te_a = envelopes[i]
        for j in range(i + 1, n):
            te_b = envelopes[j]
            if te_a.robot_id == te_b.robot_id:
                continue
            geom_a = te_a.spatial_envelope.geometry
            geom_b = te_b.spatial_envelope.geometry
            if not geom_a.intersects(geom_b):
                continue
            sections.extend(_pairwise_critical_sections(te_a, te_b))
    return sections


def _pairwise_critical_sections(
    te_a: TrajectoryEnvelope, te_b: TrajectoryEnvelope
) -> list[CriticalSection]:
    fa = te_a.spatial_envelope.footprints
    fb = te_b.spatial_envelope.footprints
    pairs: set[tuple[int, int]] = set()
    # Per-waypoint pairwise intersect tests. With L_a · L_b small (tens) this
    # is faster than building an STRtree and dispatching queries; for larger
    # paths a tree pre-pass would help.
    for k, poly_a in enumerate(fa):
        for l, poly_b in enumerate(fb):
            if poly_a.intersects(poly_b):
                pairs.add((k, l))
    if not pairs:
        return []

    visited: set[tuple[int, int]] = set()
    sections: list[CriticalSection] = []
    for seed in sorted(pairs):
        if seed in visited:
            continue
        stack = [seed]
        comp: list[tuple[int, int]] = []
        while stack:
            p = stack.pop()
            if p in visited:
                continue
            visited.add(p)
            comp.append(p)
            k, l = p
            for dk in (-1, 0, 1):
                for dl in (-1, 0, 1):
                    if dk == 0 and dl == 0:
                        continue
                    nb = (k + dk, l + dl)
                    if nb in pairs and nb not in visited:
                        stack.append(nb)
        ks = [p[0] for p in comp]
        ls = [p[1] for p in comp]
        sections.append(
            CriticalSection(
                envelope_a=te_a,
                envelope_b=te_b,
                a_start=min(ks),
                a_end=max(ks),
                b_start=min(ls),
                b_end=max(ls),
            )
        )
    return sections


__all__ = [
    "AbstractTrajectoryEnvelopeCoordinator",
    "COORDINATION_PERIOD",
    "find_critical_sections",
]
