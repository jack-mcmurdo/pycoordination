"""``NetworkConfiguration``: shared network-delay/packet-loss parameters.

Module-level state mirrors Java's static fields on the class of the same
name (shared process-wide, exactly like a Java ``static`` field).
"""

from __future__ import annotations

PROBABILITY_OF_PACKET_LOSS: float = 0.0
_maximum_tx_delay: int = 0
_minimum_tx_delay: int = 0


def setDelays(minimum: int, maximum: int) -> None:
    global _maximum_tx_delay, _minimum_tx_delay
    mx = max(0, max(minimum, maximum))
    mn = max(0, min(minimum, maximum))
    _maximum_tx_delay = mx
    _minimum_tx_delay = mn


def getMaximumTxDelay() -> int:
    return _maximum_tx_delay


def getMinimumTxDelay() -> int:
    return _minimum_tx_delay
