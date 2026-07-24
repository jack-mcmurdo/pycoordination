import structlog

from coordination_oru.util.logging import get_logger


def _capturing_processors(seen):
    def capture(_logger, _name, event_dict):
        seen.append(dict(event_dict))
        return event_dict

    return [capture, structlog.processors.JSONRenderer()]


def test_get_logger_stays_lazy_across_a_later_configure():
    """Call sites do ``log = get_logger(__name__)`` at *module import time* —
    often before the embedding application's own ``structlog.configure()``
    has run (e.g. dyno_coordination's ``logging_setup.configure()``, called
    from ``main()`` after all top-level imports). If ``get_logger()``
    eagerly materialized a bound logger, it would freeze onto whatever
    config was active at import time (structlog's unconfigured defaults)
    and stay deaf to the real configuration forever."""
    log = get_logger("some.module")  # simulates the pre-configure() import

    seen = []
    structlog.configure(
        processors=_capturing_processors(seen),
        wrapper_class=structlog.make_filtering_bound_logger(0),
        logger_factory=structlog.PrintLoggerFactory(),
        cache_logger_on_first_use=False,
    )
    try:
        log.info("hello", x=1)
    finally:
        structlog.reset_defaults()

    assert seen == [{"event": "hello", "x": 1}]


def test_get_logger_bindings_survive_as_initial_context():
    seen = []
    structlog.configure(
        processors=_capturing_processors(seen),
        wrapper_class=structlog.make_filtering_bound_logger(0),
        logger_factory=structlog.PrintLoggerFactory(),
        cache_logger_on_first_use=False,
    )
    try:
        get_logger("some.module", robot_id="r0").info("hello")
    finally:
        structlog.reset_defaults()

    assert seen == [{"event": "hello", "robot_id": "r0"}]
