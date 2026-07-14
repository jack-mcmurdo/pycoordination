"""Web viewer: wire-message composition and websocket roundtrip.

The message builders are exercised against a live two-robot coordinator;
the websocket test drives the starlette app headlessly via TestClient, so
no frontend build is needed.
"""

from __future__ import annotations

import asyncio

import pytest
from starlette.testclient import TestClient

from coordination_oru.mission import Mission
from coordination_oru.simulation2D.trajectory_envelope_coordinator_simulation import (
    TrajectoryEnvelopeCoordinatorSimulation,
)
from coordination_oru.viz.web_viewer import (
    WebViewer,
    build_state_message,
    build_static_message,
    driving_envelope_ids,
)
from tests.paths import two_robot_cross


def _add_cross_missions(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation,
    footprint: tuple[tuple[float, float], ...],
) -> None:
    path_a, path_b = two_robot_cross()
    coordinator.setFootprint(1, *footprint)
    coordinator.setFootprint(2, *footprint)
    coordinator.placeRobot(1, path_a[0].getPose())
    coordinator.placeRobot(2, path_b[0].getPose())
    coordinator.addMissions(Mission(1, path_a), Mission(2, path_b))


@pytest.mark.asyncio
async def test_message_composition(
    coordinator: TrajectoryEnvelopeCoordinatorSimulation,
    footprint: tuple[tuple[float, float], ...],
) -> None:
    # before any mission: no driving robots, nothing static to draw
    assert driving_envelope_ids(coordinator) == {}
    assert build_static_message(coordinator)["robots"] == []

    _add_cross_missions(coordinator, footprint)
    await asyncio.sleep(0.05)  # let the coordinator pick up the missions

    envelopes = driving_envelope_ids(coordinator)
    assert set(envelopes) == {1, 2}

    static = build_static_message(coordinator, title="test", world_size=14.0)
    assert static["kind"] == "static"
    assert static["world"] == {"size": 14.0, "center": [0.0, 0.0]}
    by_id = {r["id"]: r for r in static["robots"]}
    assert set(by_id) == {1, 2}
    for robotID, entry in by_id.items():
        assert entry["envelopeID"] == envelopes[robotID]
        assert len(entry["path"]) >= 2
        assert all(len(pt) == 2 for pt in entry["path"])
        assert len(entry["envelope"]) >= 1
        assert all(len(ring) >= 3 for ring in entry["envelope"])
    outlines = {f["id"]: f["ring"] for f in static["footprints"]}
    assert set(outlines) == {1, 2}
    assert all(len(ring) >= 3 for ring in outlines.values())

    state = build_state_message(coordinator)
    assert state["kind"] == "state"
    robots = {r["id"]: r for r in state["robots"]}
    assert set(robots) == {1, 2}
    for entry in robots.values():
        assert entry["driving"] is True
        assert len(entry["pose"]) == 3
        assert entry["pathLength"] >= 2
    # one CS between two robots → exactly one yielder → leader dependency
    deps = state["dependencies"]
    assert len(deps) == 1
    assert {deps[0]["waiting"], deps[0]["driving"]} == {1, 2}
    assert deps[0]["waitingPoint"] >= 0
    # the crossing paths must have produced the one critical section
    assert state["counts"]["criticalSections"] == 1
    [cs] = state["criticalSections"]
    assert {cs["robot1"], cs["robot2"]} == {1, 2}
    assert 0 <= cs["start1"] <= cs["end1"]
    assert 0 <= cs["start2"] <= cs["end2"]


def test_websocket_sends_static_then_state() -> None:
    tec = TrajectoryEnvelopeCoordinatorSimulation(CONTROL_PERIOD=10, TEMPORAL_RESOLUTION=1000.0)
    tec.setupSolver()
    viewer = WebViewer(tec, title="ws-test", world_size=14.0)

    with TestClient(viewer.app) as client:
        with client.websocket_connect("/ws") as ws:
            first = ws.receive_json()
            assert first["kind"] == "static"
            assert first["title"] == "ws-test"
            second = ws.receive_json()
            assert second["kind"] == "state"
            assert second["seq"] == first["seq"] + 1
            assert second["ts"] >= first["ts"]
