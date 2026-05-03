"""Async-safe structured logging via structlog.

The Java framework uses ad-hoc ``System.out`` printing. We use ``structlog``
so log records carry the robot id, envelope id, and event keys — easy to grep
in tests and easy to ship to a JSON log aggregator in production.
"""

from __future__ import annotations

import logging
import sys
from typing import Any

import structlog


def configure_logging(level: int = logging.INFO) -> None:
    """Idempotent global setup. Safe to call from tests and from main."""
    logging.basicConfig(stream=sys.stderr, level=level, format="%(message)s")
    structlog.configure(
        processors=[
            structlog.contextvars.merge_contextvars,
            structlog.processors.add_log_level,
            structlog.processors.TimeStamper(fmt="iso", utc=True),
            structlog.dev.ConsoleRenderer(colors=False),
        ],
        wrapper_class=structlog.make_filtering_bound_logger(level),
        cache_logger_on_first_use=True,
    )


def get_logger(name: str | None = None, **bindings: Any) -> Any:
    return structlog.get_logger(name).bind(**bindings)
