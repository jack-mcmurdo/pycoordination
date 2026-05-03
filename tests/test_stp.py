"""STP solver sanity checks."""

from __future__ import annotations

import math

import pytest

from coordination_oru.metacsp.temporal.stp import STPInconsistent, STPSolver


def test_origin_is_node_zero() -> None:
    stp = STPSolver()
    assert stp.num_variables == 1
    assert stp.get_earliest(0) == 0.0
    assert stp.get_latest(0) == 0.0


def test_release_and_deadline_propagate() -> None:
    stp = STPSolver()
    a = stp.new_variable()
    stp.add_release_time(a, 5.0)
    stp.add_deadline(a, 12.0)
    assert stp.get_earliest(a) == 5.0
    assert stp.get_latest(a) == 12.0


def test_negative_cycle_raises() -> None:
    stp = STPSolver()
    a = stp.new_variable()
    b = stp.new_variable()
    # b - a <= -1, a - b <= -1  =>  cycle of weight -2
    stp.add_constraint(a, b, -1.0)
    with pytest.raises(STPInconsistent):
        stp.add_constraint(b, a, -1.0)


def test_ordering_propagates_to_end_times() -> None:
    stp = STPSolver()
    a_start = stp.new_variable()
    a_end = stp.new_variable()
    b_start = stp.new_variable()
    b_end = stp.new_variable()
    # release a at t=0, duration [3, 3]
    stp.add_release_time(a_start, 0.0)
    stp.add_interval(a_start, a_end, 3.0, 3.0)
    # b duration [2, 2]
    stp.add_interval(b_start, b_end, 2.0, 2.0)
    # ordering: a finishes before b starts
    stp.add_interval(a_end, b_start, 0.0, math.inf)
    assert stp.get_earliest(b_start) >= 3.0
    assert stp.get_earliest(b_end) >= 5.0
