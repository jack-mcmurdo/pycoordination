"""``TrackingCallback``: per-tracker lifecycle hooks.

Mirrors Java's abstract ``TrackingCallback`` class. Subclass and override
the hooks you need; all are no-ops by default. ``myTE`` is rebound by the
tracker whenever its trajectory envelope changes (``updateTrajectoryEnvelope``).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from coordination_oru.metacsp.spatial.trajectory_envelope import TrajectoryEnvelope


class TrackingCallback:
    def __init__(self, te: "TrajectoryEnvelope | None" = None) -> None:
        self.myTE = te

    def updateTrajectoryEnvelope(self, te: "TrajectoryEnvelope") -> None:
        self.myTE = te

    def beforeTrackingStart(self) -> None:
        pass

    def onTrackingStart(self) -> None:
        pass

    def onNewGroundEnvelope(self) -> None:
        pass

    def beforeTrackingFinished(self) -> None:
        pass

    def onTrackingFinished(self) -> None:
        pass

    def onPositionUpdate(self) -> list[str] | None:
        return None
