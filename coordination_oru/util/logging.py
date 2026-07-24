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
    """A lazy proxy, not a materialized logger: don't call ``.bind()`` here.

    ``BoundLoggerLazyProxy.bind()`` immediately freezes in whatever
    ``structlog.configure()`` last set (processors, wrapper class, logger
    factory) onto a concrete ``BoundLogger``. Every call site does
    ``log = get_logger(__name__)`` at module import time, which for
    downstream consumers happens before their own ``structlog.configure()``
    runs — eagerly binding would permanently lock these loggers onto
    structlog's unconfigured defaults (console-only, print-based), deaf to
    any later reconfiguration. ``structlog.get_logger(name, **bindings)``
    passes bindings through as initial context instead, keeping the proxy
    lazy so it materializes against whatever config is active at first log
    call.
    """
    return structlog.get_logger(name, **bindings)
