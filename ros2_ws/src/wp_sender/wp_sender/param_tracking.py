"""
Pure bookkeeping for reading another node's ROS parameters without ever blocking.

wp_sender/param_tracking.py -- PURPOSE: the pure-Python logic behind reading another node's ROS
parameters asynchronously. It has NO ROS imports, so it is unit-tested with plain pytest
(test/test_parameter_reader.py) and can be reused by any node.

Two pieces:

  parameter_value_to_python(value)
      Turns one rcl_interfaces/ParameterValue (or anything with the same attribute names) into a
      plain Python value: 2 -> int, 3 -> float, 4 -> str, ... and NOT_SET (0) -> None.

  ParameterReadTracker
      Decides WHEN to send a remote parameter read, when to give up on one that takes too long,
      and when to try again after a failure. wp_sender/parameter.py (AsyncParameterReader) drives
      it from a ROS timer and does the actual sending.

Why it exists (October 2026): main_controller read waypoint_manager's mission indices with a
helper that spun the executor INSIDE a callback and returned None after a 10 s timeout.
main_controller then crashed on the None (2 of 4 simulator runs: once right after start-up, once
at take-off). The fix is to read asynchronously; this tracker is the small state machine that
makes that safe:
  * many read requests while one read is in flight collapse into ONE follow-up read
    (waypoint_manager can announce up to six parameter changes for one mission);
  * a read that gets no answer within `timeout_s` is abandoned (the caller cancels it);
  * a failed or abandoned read is retried after `retry_delay_s`, again and again, until one
    succeeds; the caller keeps its previous values in the meantime.
"""

# rcl_interfaces/msg/ParameterType constants (copied so this module needs no ROS install).
PARAMETER_NOT_SET = 0
PARAMETER_BOOL = 1
PARAMETER_INTEGER = 2
PARAMETER_DOUBLE = 3
PARAMETER_STRING = 4
PARAMETER_BYTE_ARRAY = 5
PARAMETER_BOOL_ARRAY = 6
PARAMETER_INTEGER_ARRAY = 7
PARAMETER_DOUBLE_ARRAY = 8
PARAMETER_STRING_ARRAY = 9

# Tracker actions returned by ParameterReadTracker.next_action().
ACTION_SEND = 'send'
ACTION_TIMEOUT = 'timeout'


def parameter_value_to_python(value):
    """
    Convert one ParameterValue message into a plain Python value.

    Parameters
    ----------
    value
        An rcl_interfaces/msg/ParameterValue, or any object with the same attributes
        (`type`, `bool_value`, `integer_value`, `double_value`, `string_value` and the
        `*_array_value` fields).

    Returns
    -------
    object
        bool, int, float, str, bytes or a list, matching the parameter's type; None when the
        parameter is not set (type 0) or the type is unknown.

    """
    kind = getattr(value, 'type', PARAMETER_NOT_SET)
    if kind == PARAMETER_BOOL:
        return bool(value.bool_value)
    if kind == PARAMETER_INTEGER:
        return int(value.integer_value)
    if kind == PARAMETER_DOUBLE:
        return float(value.double_value)
    if kind == PARAMETER_STRING:
        return str(value.string_value)
    if kind == PARAMETER_BYTE_ARRAY:
        return bytes(value.byte_array_value)
    if kind == PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    if kind == PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if kind == PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if kind == PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    return None


def decode_parameter_values(names, values):
    """
    Pair the requested parameter names with the values a GetParameters call returned.

    Parameters
    ----------
    names
        The parameter names, in the order they were requested.
    values
        The `values` list of the GetParameters response (same order).

    Returns
    -------
    dict
        {name: python value}; a name whose value is missing or not set maps to None.

    """
    values = list(values or [])
    out = {}
    for i, name in enumerate(names):
        out[name] = parameter_value_to_python(values[i]) if i < len(values) else None
    return out


class ParameterReadTracker:
    """
    Decide when to send, abandon and retry one remote parameter read (pure; no ROS).

    How a caller uses it, on every timer tick and right after request():
        action = tracker.next_action(now, service_ready)
        'send'    -> send a GetParameters request, then call tracker.sent(now)
        'timeout' -> the request in flight took too long: cancel it, then
                     tracker.finished(now, False)
        None      -> nothing to do now
    and when an answer (or an error) arrives: tracker.finished(now, ok).

    `now` is any steadily increasing clock in seconds (time.monotonic() in the node, a plain
    number in the tests).
    """

    def __init__(self, timeout_s: float = 5.0, retry_delay_s: float = 2.0):
        """
        Set up an idle tracker.

        Parameters
        ----------
        timeout_s
            A request with no answer after this many seconds is abandoned.
        retry_delay_s
            After a failure, wait this long before the next attempt.

        """
        self.timeout_s = float(timeout_s)
        self.retry_delay_s = float(retry_delay_s)
        self.wanted = False          # somebody asked for a (new) read that has not been sent yet
        self.in_flight = False       # a request has been sent and not answered yet
        self.sent_at = None          # when the request in flight was sent
        self.not_before = 0.0        # no new attempt before this time (back-off after a failure)
        self.failures = 0            # failures in a row (0 after any success)
        self.successes = 0           # successful reads so far

    def request(self) -> None:
        """Ask for a fresh read; several calls before it is sent count as one."""
        self.wanted = True

    def next_action(self, now: float, service_ready: bool):
        """Tell the caller what to do now: ACTION_SEND, ACTION_TIMEOUT or None."""
        if self.in_flight:
            if self.sent_at is not None and now - self.sent_at >= self.timeout_s:
                return ACTION_TIMEOUT
            return None
        if self.wanted and service_ready and now >= self.not_before:
            return ACTION_SEND
        return None

    def sent(self, now: float) -> None:
        """Record that a request was just sent."""
        self.in_flight = True
        self.sent_at = now
        self.wanted = False

    def finished(self, now: float, ok: bool) -> None:
        """
        Record the end of the request in flight.

        After a failure another read is wanted automatically (after retry_delay_s), so the
        caller keeps retrying until one read succeeds.

        Parameters
        ----------
        now
            Current time.
        ok
            True if usable values arrived; False for an error, an incomplete answer or a
            timeout.

        """
        self.in_flight = False
        self.sent_at = None
        if ok:
            self.failures = 0
            self.successes += 1
        else:
            self.failures += 1
            self.wanted = True
            self.not_before = now + self.retry_delay_s
