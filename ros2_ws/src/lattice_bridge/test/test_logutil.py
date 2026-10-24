"""
lattice_bridge/test/test_logutil.py -- PURPOSE: prove the loud-once warnings: the altitude warning is logged
exactly once when no geoid undulation is configured, never when one is (even 0.0), other keys are independent,
the memory is bounded, and a failing log handler cannot raise into the caller.
"""

import logging
import threading

import pytest

from lattice_bridge import logutil
from lattice_bridge.entities import ALTITUDE_WARNING


class Collector(logging.Handler):
    def __init__(self):
        super().__init__(level=logging.DEBUG)
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture
def logger():
    """A private logger with its own handler (see test_jsonl_sink for why caplog is avoided)."""
    log = logging.getLogger('lattice_bridge.test_logutil')
    handler = Collector()
    log.addHandler(handler)
    log.setLevel(logging.DEBUG)
    log.propagate = False
    logutil.reset_warnings()
    yield log, handler
    log.removeHandler(handler)
    logutil.reset_warnings()


def test_the_altitude_warning_is_logged_once_when_nothing_is_configured(logger):
    log, handler = logger
    assert logutil.warn_if_altitude_approximate(None, log) is True
    assert logutil.warn_if_altitude_approximate(None, log) is False
    assert logutil.warn_if_altitude_approximate(None, log) is False
    assert len(handler.records) == 1
    record = handler.records[0]
    assert record.levelno == logging.WARNING and record.getMessage() == ALTITUDE_WARNING
    assert 'AMSL' in record.getMessage() and 'geoid_undulation_m' in record.getMessage()


@pytest.mark.parametrize('configured', [0.0, -29.0, 25.5, 0])
def test_a_configured_value_is_silent(logger, configured):
    log, handler = logger
    assert logutil.warn_if_altitude_approximate(configured, log) is False
    assert handler.records == []


def test_reset_allows_the_warning_again(logger):
    log, handler = logger
    logutil.warn_if_altitude_approximate(None, log)
    logutil.reset_warnings()
    assert logutil.warn_if_altitude_approximate(None, log) is True
    assert len(handler.records) == 2


def test_keys_are_independent(logger):
    log, handler = logger
    assert logutil.warn_once('a', 'first', log) and logutil.warn_once('b', 'second', log)
    assert not logutil.warn_once('a', 'first again', log)
    assert [r.getMessage() for r in handler.records] == ['first', 'second']


def test_text_with_percent_signs_is_not_treated_as_a_format_string(logger):
    log, handler = logger
    assert logutil.warn_once('pct', '100% sure %s %d {0}', log)
    assert handler.records[0].getMessage() == '100% sure %s %d {0}'


def test_memory_is_bounded(logger):
    log, handler = logger
    logged = sum(logutil.warn_once(f'key{i}', 'x', log) for i in range(500))
    assert logged == 100 and len(handler.records) == 100


def test_a_broken_handler_cannot_raise_into_the_caller(logger):
    log, _ = logger

    class Broken(logging.Handler):
        def emit(self, record):
            raise RuntimeError('disk on fire')

        def handleError(self, record):                  # logging would print a traceback; keep the test quiet
            pass

    broken = Broken()
    log.addHandler(broken)
    try:
        assert logutil.warn_once('boom', 'x', log) in (True, False)         # no exception either way
    finally:
        log.removeHandler(broken)


def test_thread_safe_exactly_one_winner(logger):
    log, handler = logger
    results = []

    def worker():
        results.append(logutil.warn_once('race', 'x', log))

    threads = [threading.Thread(target=worker) for _ in range(20)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert results.count(True) == 1 and len(handler.records) == 1


def test_default_logger_is_the_package_logger():
    assert logutil.LOG.name == 'lattice_bridge'
