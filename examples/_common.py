"""Shared runner for the example scripts.

The implementation lives in :mod:`coordination_oru.util.example_runner` so
the installed ``coordination-oru-demo`` console script can use it too; this
module just re-exports it for the example scripts' historical import.
"""

from coordination_oru.util.example_runner import IDLE_TIMEOUT, run, wait_until_idle

__all__ = ["IDLE_TIMEOUT", "run", "wait_until_idle"]
