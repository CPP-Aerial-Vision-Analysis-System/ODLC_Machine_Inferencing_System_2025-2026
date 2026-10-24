"""
Unit tests for AsyncParameterReader in wp_sender/parameter.py (the non-blocking parameter reader).

test_async_reader.py -- PURPOSE: drive the reader that main_controller uses for its mission indices
with a fake node, a fake service client and a clock the test moves by hand (no ROS graph, nothing
spins), and check the October 2026 crash fix from end to end:
  * a read that gets no answer is given up after timeout_s; the caller's handler is NOT called, so
    the caller keeps its previous values, and nothing raises (the old code returned None here and
    main_controller died on it);
  * the read is tried again after retry_delay_s, a late answer to the abandoned request is ignored,
    and the next answer is delivered;
  * a failing service call, an incomplete answer and a handler that raises are reported, never
    raised;
  * the log says when reading works again ("... again after N failed attempt(s)").

Needs rclpy / rcl_interfaces (ROS 2 sourced), otherwise the tests are skipped:
    source /opt/ros/humble/setup.bash
    python3 -m pytest ros2_ws/src/wp_sender/test/test_async_reader.py -v
"""

import os
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy', reason='ROS 2 (rclpy) is not sourced')

from rcl_interfaces.msg import ParameterValue  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from wp_sender.parameter import AsyncParameterReader  # noqa: E402

NAMES = ['rtl_index', 'last_before_rtl']


class FakeLogger:
    """Collect log lines as (level, text)."""

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    warning = warn

    def error(self, text):
        self.lines.append(('error', text))

    def text(self):
        return '\n'.join(t for _, t in self.lines)


class FakeFuture:
    """Like an rclpy Future: done-callbacks run when it is answered, failed or cancelled."""

    def __init__(self):
        self._callbacks, self._response, self._error, self._cancelled = [], None, None, False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def _finish(self):
        for callback in self._callbacks:
            callback(self)

    def answer(self, response):
        self._response = response
        self._finish()

    def fail(self, error):
        self._error = error
        self._finish()

    def cancel(self):
        self._cancelled = True
        self._finish()

    def cancelled(self):
        return self._cancelled

    def result(self):
        if self._error is not None:
            raise self._error
        return self._response


class FakeClient:
    """A GetParameters client: remembers every request and the future it returned."""

    def __init__(self):
        self.ready = True
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = FakeFuture()
        self.futures.append(future)
        return future


class FakeNode:
    """The three node methods the reader uses."""

    def __init__(self):
        self.client = FakeClient()
        self.logger = FakeLogger()

    def create_client(self, _srv_type, _name):
        return self.client

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback, cancel=lambda: None)

    def get_logger(self):
        return self.logger


def answer_with(rtl, last):
    """Build a GetParameters response with two integer values."""
    return SimpleNamespace(values=[ParameterValue(type=2, integer_value=rtl),
                                   ParameterValue(type=2, integer_value=last)])


@pytest.fixture
def setup():
    """Return (reader, node, got, clock): got collects the handler's values, clock[0] is now."""
    node = FakeNode()
    got = []
    reader = AsyncParameterReader(node, 'waypoint_manager', NAMES, got.append,
                                  timeout_s=5.0, retry_delay_s=2.0, tick_s=0.5)
    clock = [100.0]
    reader._now = lambda: clock[0]
    return reader, node, got, clock


def test_a_good_read_is_delivered_without_extra_messages(setup):
    reader, node, got, _clock = setup
    reader.request('start-up')
    assert len(node.client.futures) == 1
    node.client.futures[0].answer(answer_with(11, 10))
    assert got == [{'rtl_index': 11, 'last_before_rtl': 10}]
    assert 'again after' not in node.logger.text()       # nothing failed, nothing to announce


def test_a_read_without_answer_keeps_the_old_values_and_is_retried(setup):
    reader, node, got, clock = setup
    reader.request('mission indices may have changed')
    first = node.client.futures[0]
    clock[0] += 4.9
    reader._tick()
    assert reader.failures == 0                          # not given up yet
    clock[0] += 0.1                                      # 5 s without an answer
    reader._tick()                                       # must not raise (the old crash)
    assert reader.failures == 1 and got == []            # handler not called: old values kept
    text = node.logger.text()
    assert 'no answer within 5 s' in text and 'Keeping the previous values' in text
    clock[0] += 1.9
    reader._tick()
    assert len(node.client.futures) == 1                 # waits the 2 s back-off
    clock[0] += 0.1
    reader._tick()
    assert len(node.client.futures) == 2                 # second attempt sent
    first.answer(answer_with(99, 98))                    # a LATE answer to the abandoned read
    assert got == []                                     # ... is ignored
    node.client.futures[1].answer(answer_with(11, 10))
    assert got == [{'rtl_index': 11, 'last_before_rtl': 10}] and reader.failures == 0
    assert 'again after 1 failed attempt(s)' in node.logger.text()


def test_two_timeouts_in_a_row_then_success(setup):
    reader, node, got, clock = setup
    reader.request('start-up')
    for attempt in (1, 2):
        clock[0] += 5.0
        reader._tick()                                   # attempt times out
        assert reader.failures == attempt and got == []
        clock[0] += 2.0
        reader._tick()                                   # next attempt
    node.client.futures[-1].answer(answer_with(11, 10))
    assert len(node.client.futures) == 3 and len(got) == 1
    assert 'again after 2 failed attempt(s)' in node.logger.text()


def test_a_failing_service_call_is_reported_not_raised(setup):
    reader, node, got, clock = setup
    reader.request('start-up')
    node.client.futures[0].fail(RuntimeError('waypoint_manager went away'))
    assert got == [] and reader.failures == 1
    assert 'the service call failed' in node.logger.text()
    clock[0] += 2.0
    reader._tick()
    node.client.futures[1].answer(answer_with(11, 10))
    assert len(got) == 1


def test_an_incomplete_answer_counts_as_a_failure(setup):
    reader, node, got, _clock = setup
    reader.request('start-up')
    node.client.futures[0].answer(SimpleNamespace(values=[ParameterValue(type=2, integer_value=11),
                                                          ParameterValue(type=0)]))
    assert got == [] and reader.failures == 1
    assert 'not set (yet): last_before_rtl' in node.logger.text()


def test_a_handler_that_raises_does_not_escape(setup):
    reader, node, _got, _clock = setup

    def broken_handler(_values):
        raise ValueError('bug in the caller')

    reader._on_values = broken_handler
    reader.request('start-up')
    node.client.futures[0].answer(answer_with(11, 10))  # must not raise
    assert 'raised: bug in the caller' in node.logger.text()


def test_nothing_is_sent_while_the_service_is_missing(setup):
    reader, node, got, clock = setup
    node.client.ready = False
    reader.request('start-up')
    clock[0] += 30.0
    reader._tick()
    assert node.client.futures == [] and got == []
    assert node.logger.text().count('Waiting for /waypoint_manager/get_parameters') == 1
    node.client.ready = True
    reader._tick()
    node.client.futures[0].answer(answer_with(11, 10))
    assert len(got) == 1
