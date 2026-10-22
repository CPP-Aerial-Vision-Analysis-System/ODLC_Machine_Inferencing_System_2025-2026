"""
Unit tests for wp_sender/param_tracking.py (non-blocking parameter reads).

test_parameter_reader.py -- PURPOSE: unit tests for the bookkeeping that lets main_controller read
waypoint_manager's mission indices WITHOUT blocking (the fix for the main_controller crash of
October 2026). Checks: parameter values are decoded to the right Python types, NOT_SET becomes
None, many requests collapse into one follow-up read, a read with no answer is abandoned after the
timeout, and a failed read is retried after the back-off until one succeeds.

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/wp_sender/test/test_parameter_reader.py -v
"""

import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from wp_sender.param_tracking import (  # noqa: E402
    ACTION_SEND,
    ACTION_TIMEOUT,
    PARAMETER_BOOL,
    PARAMETER_DOUBLE,
    PARAMETER_INTEGER,
    PARAMETER_INTEGER_ARRAY,
    PARAMETER_NOT_SET,
    PARAMETER_STRING,
    ParameterReadTracker,
    decode_parameter_values,
    parameter_value_to_python,
)


def pv(kind, **fields):
    """Build a stand-in for rcl_interfaces/ParameterValue with the fields a test needs."""
    base = dict(type=kind, bool_value=False, integer_value=0, double_value=0.0, string_value='',
                byte_array_value=[], bool_array_value=[], integer_array_value=[],
                double_array_value=[], string_array_value=[])
    base.update(fields)
    return SimpleNamespace(**base)


# ------------------------------------------------------------------ value decoding
def test_integer_bool_double_string_are_decoded():
    assert parameter_value_to_python(pv(PARAMETER_INTEGER, integer_value=11)) == 11
    assert parameter_value_to_python(pv(PARAMETER_BOOL, bool_value=True)) is True
    assert parameter_value_to_python(pv(PARAMETER_DOUBLE, double_value=2.5)) == 2.5
    assert parameter_value_to_python(pv(PARAMETER_STRING, string_value='hi')) == 'hi'
    ints = pv(PARAMETER_INTEGER_ARRAY, integer_array_value=[1, 2])
    assert parameter_value_to_python(ints) == [1, 2]


def test_not_set_is_none():
    # waypoint_manager answers NOT_SET for a parameter it never declared.
    assert parameter_value_to_python(pv(PARAMETER_NOT_SET)) is None


def test_decode_pairs_names_with_values_and_marks_missing_ones():
    names = ['takeoff_index', 'rtl_index', 'last_before_rtl']
    values = [pv(PARAMETER_INTEGER, integer_value=1), pv(PARAMETER_INTEGER, integer_value=11)]
    assert decode_parameter_values(names, values) == {
        'takeoff_index': 1, 'rtl_index': 11, 'last_before_rtl': None}


def test_decode_of_an_empty_answer():
    assert decode_parameter_values(['a'], None) == {'a': None}


# ------------------------------------------------------------------ tracker state machine
def test_nothing_is_sent_until_requested():
    t = ParameterReadTracker()
    assert t.next_action(0.0, service_ready=True) is None


def test_request_waits_for_the_service():
    t = ParameterReadTracker()
    t.request()
    assert t.next_action(0.0, service_ready=False) is None
    assert t.next_action(1.0, service_ready=True) == ACTION_SEND


def test_many_requests_while_in_flight_give_one_follow_up_read():
    t = ParameterReadTracker()
    t.request()
    assert t.next_action(0.0, True) == ACTION_SEND
    t.sent(0.0)
    for _ in range(6):                       # six parameter events for one mission
        t.request()
        assert t.next_action(0.1, True) is None   # still in flight: never a second request
    t.finished(0.2, ok=True)
    assert t.next_action(0.2, True) == ACTION_SEND  # exactly one follow-up read
    t.sent(0.2)
    t.finished(0.3, ok=True)
    assert t.next_action(0.3, True) is None
    assert t.successes == 2 and t.failures == 0


def test_a_read_without_answer_times_out():
    t = ParameterReadTracker(timeout_s=5.0, retry_delay_s=2.0)
    t.request()
    t.sent(10.0)
    assert t.next_action(14.9, True) is None
    assert t.next_action(15.0, True) == ACTION_TIMEOUT


def test_failure_is_retried_after_the_back_off_until_success():
    t = ParameterReadTracker(timeout_s=5.0, retry_delay_s=2.0)
    t.request()
    t.sent(0.0)
    t.finished(1.0, ok=False)                # e.g. the call failed / values not set yet
    assert t.failures == 1
    assert t.next_action(2.9, True) is None  # back-off
    assert t.next_action(3.0, True) == ACTION_SEND
    t.sent(3.0)
    assert t.next_action(8.0, True) == ACTION_TIMEOUT
    t.finished(8.0, ok=False)
    assert t.failures == 2
    assert t.next_action(10.0, True) == ACTION_SEND
    t.sent(10.0)
    t.finished(10.5, ok=True)
    assert t.failures == 0 and t.successes == 1
    assert t.next_action(20.0, True) is None  # nothing more wanted


def test_tracker_never_asks_for_two_requests_at_once():
    t = ParameterReadTracker(timeout_s=5.0, retry_delay_s=0.0)
    in_flight = 0
    now = 0.0
    for step in range(200):
        now += 0.25
        if step % 3 == 0:
            t.request()
        action = t.next_action(now, True)
        if action == ACTION_SEND:
            in_flight += 1
            assert in_flight == 1
            t.sent(now)
        elif action == ACTION_TIMEOUT:
            in_flight -= 1
            t.finished(now, ok=False)
        elif t.in_flight and step % 5 == 0:
            in_flight -= 1
            t.finished(now, ok=True)
        assert in_flight in (0, 1)
