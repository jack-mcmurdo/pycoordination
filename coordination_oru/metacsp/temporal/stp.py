"""Simple Temporal Problem solver (Floyd-Warshall on a numpy distance matrix).

Replaces the meta-csp ``APSPSolver``. Constraints are difference constraints
of the form ``x_dst - x_src <= weight``. Consistency is the absence of any
negative-weight cycle, equivalently ``d[i, i] >= 0`` for every node.

Node 0 is reserved as the temporal origin (``t = 0``). The solver allocates
it in the constructor so that ``get_earliest`` / ``get_latest`` always have a
reference frame.

The Java implementation supports incremental constraint removal via a kept
copy of the original constraint graph. We start with that pattern: every
``add_constraint`` records the edge and runs an incremental update; a removal
or backtrack triggers a full rebuild.

``remove_variable`` actually detaches a node (drops every constraint that
touches it, then frees its slot) rather than merely hiding it: a node's
slot is reused by a later ``new_variable()`` call instead of the matrix
growing without bound for the process's entire lifetime. The caller must
never reference a removed node again — its slot may already mean something
else by the time a stale reference is queried.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

INF = math.inf
ORIGIN: int = 0


@dataclass(frozen=True, slots=True)
class _Edge:
    src: int
    dst: int
    weight: float


class STPInconsistent(RuntimeError):
    """Raised when a constraint addition produces a negative-weight cycle."""


class STPSolver:
    """All-pairs-shortest-path STP solver.

    The distance matrix ``_d`` has shape ``(max_nodes, max_nodes)``; only the
    top-left ``(_n, _n)`` block is meaningful. ``_d[i, j]`` is the tightest
    known upper bound on ``x_j - x_i``.
    """

    def __init__(self, max_nodes: int = 256) -> None:
        if max_nodes < 2:
            raise ValueError("max_nodes must allow the origin plus at least one variable")
        self._max = max_nodes
        self._d: npt.NDArray[np.float64] = np.full((max_nodes, max_nodes), INF, dtype=np.float64)
        np.fill_diagonal(self._d, 0.0)
        self._n = 0
        self._edges: list[_Edge] = []
        #: node indices freed by remove_variable, available for reuse
        self._free: list[int] = []
        # allocate the origin
        self._origin = self.new_variable()
        assert self._origin == ORIGIN

    # --------------------------------------------------------------- variables

    @property
    def num_variables(self) -> int:
        """Live node count (allocated minus freed) — the working-set size
        that actually drives rebuild()'s cost, not the lifetime total."""
        return self._n - len(self._free)

    def new_variable(self) -> int:
        if self._free:
            idx = self._free.pop()
            self._d[idx, : self._n] = INF
            self._d[: self._n, idx] = INF
            self._d[idx, idx] = 0.0
            return idx
        if self._n >= self._max:
            self._grow(max(self._max * 2, self._n + 1))
        idx = self._n
        self._n += 1
        return idx

    def remove_variable(self, node: int) -> None:
        """Detach ``node`` from the network: drop every constraint that
        touches it (as either endpoint) and free its slot for a future
        ``new_variable()`` to reuse. Ports Java's ``removeConstraints`` +
        ``removeVariable`` pair — this solver has no separate incident-edge
        query, so one call does both. A no-op if already removed.
        """
        self._check_node(node)
        if node == ORIGIN:
            raise ValueError("cannot remove the origin")
        if node in self._free:
            return
        self._edges = [e for e in self._edges if e.src != node and e.dst != node]
        self._free.append(node)
        self.rebuild()

    def _grow(self, new_size: int) -> None:
        bigger = np.full((new_size, new_size), INF, dtype=np.float64)
        np.fill_diagonal(bigger, 0.0)
        bigger[: self._max, : self._max] = self._d[: self._max, : self._max]
        self._d = bigger
        self._max = new_size

    # ------------------------------------------------------------- constraints

    def add_constraint(self, src: int, dst: int, weight: float) -> None:
        """Add ``x_dst - x_src <= weight``.

        Performs an incremental tightening: for any pair ``(u, v)``, the new
        edge can only shorten ``d[u, v]`` via the path ``u -> src -> dst -> v``.
        Raises :class:`STPInconsistent` if the result has a negative cycle.
        """
        self._check_node(src)
        self._check_node(dst)
        self._edges.append(_Edge(src, dst, weight))

        if weight >= self._d[src, dst]:
            return  # not tighter, no propagation needed

        n = self._n
        d = self._d
        d[src, dst] = weight
        # vector update: d[u, v] = min(d[u, v], d[u, src] + weight + d[dst, v])
        col_src = d[:n, src : src + 1]
        row_dst = d[dst : dst + 1, :n]
        candidate = col_src + weight + row_dst
        np.minimum(d[:n, :n], candidate, out=d[:n, :n])

        if not self.is_consistent():
            raise STPInconsistent(
                f"adding {src}->{dst} <= {weight} produced a negative-weight cycle"
            )

    def add_interval(self, src: int, dst: int, lb: float, ub: float) -> None:
        """Encode ``lb <= x_dst - x_src <= ub`` as two difference constraints."""
        if lb > ub:
            raise ValueError(f"empty interval [{lb}, {ub}]")
        self.add_constraint(src, dst, ub)
        self.add_constraint(dst, src, -lb)

    def add_release_time(self, node: int, earliest: float) -> None:
        """Constrain ``earliest <= x_node`` (relative to the origin)."""
        self.add_constraint(node, ORIGIN, -earliest)

    def add_deadline(self, node: int, latest: float) -> None:
        """Constrain ``x_node <= latest`` (relative to the origin)."""
        self.add_constraint(ORIGIN, node, latest)

    # ----------------------------------------------------------------- queries

    def is_consistent(self) -> bool:
        n = self._n
        return bool(np.all(np.diag(self._d[:n, :n]) >= 0))

    def get_earliest(self, node: int) -> float:
        """Earliest feasible time of ``node`` relative to the origin."""
        self._check_node(node)
        return float(-self._d[node, ORIGIN])

    def get_latest(self, node: int) -> float:
        """Latest feasible time of ``node`` relative to the origin."""
        self._check_node(node)
        return float(self._d[ORIGIN, node])

    def get_distance(self, src: int, dst: int) -> float:
        """Tightest known upper bound on ``x_dst - x_src``."""
        self._check_node(src)
        self._check_node(dst)
        return float(self._d[src, dst])

    # ----------------------------------------------------------------- internal

    def _check_node(self, node: int) -> None:
        if not 0 <= node < self._n:
            raise IndexError(f"node {node} is out of range [0, {self._n})")
        if node in self._free:
            raise IndexError(f"node {node} was removed (remove_variable) — stale reference")

    def rebuild(self) -> None:
        """Recompute the distance matrix from scratch — used after removals."""
        n = self._n
        d = np.full((self._max, self._max), INF, dtype=np.float64)
        np.fill_diagonal(d, 0.0)
        for e in self._edges:
            if e.weight < d[e.src, e.dst]:
                d[e.src, e.dst] = e.weight
        # full Floyd-Warshall on the active block
        block = d[:n, :n]
        for k in range(n):
            block = np.minimum(block, block[:, k : k + 1] + block[k : k + 1, :])
        d[:n, :n] = block
        self._d = d
        if not self.is_consistent():
            raise STPInconsistent("rebuild produced a negative-weight cycle")
