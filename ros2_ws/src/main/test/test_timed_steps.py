"""
Unit tests for main/src/timed_steps.py (non-blocking timed steps).

test_timed_steps.py -- PURPOSE: unit tests for the one-shot-timer step runner that replaced
main_controller's time.sleep() calls (October 2026). A fake node with a manual clock stands in for
ROS: the tests check that steps run in order at the right times, that nothing runs early, that a
second sequence cannot start while one runs, that a failing step does not stop the rest, that every
timer is destroyed after use, and main_controller's real drop sequence timing
(6 s loiter -> open -> 3 s -> close -> 6 s -> RTL).

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/main/test/test_timed_steps.py -v
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from timed_steps import StepRunner, call_later  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))

    def error(self, msg):
        self.lines.append(('error', msg))


class FakeTimer:
    def __init__(self, node, period, callback):
        self.node, self.period, self.callback = node, period, callback
        self.next_due = node.now + period
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    """create_timer/destroy_timer/get_logger with a clock the test moves by hand."""

    def __init__(self):
        self.now = 0.0
        self.timers = []
        self.logger = FakeLogger()

    def create_timer(self, period, callback):
        t = FakeTimer(self, period, callback)
        self.timers.append(t)
        return t

    def destroy_timer(self, timer):
        self.timers.remove(timer)

    def get_logger(self):
        return self.logger

    def advance(self, dt, step=0.01):
        """Move the clock forward, firing due timers like an executor would (repeating timers)."""
        end = self.now + dt
        while self.now < end - 1e-9:
            self.now = min(end, self.now + step)
            for t in list(self.timers):
                if t in self.timers and not t.cancelled and self.now >= t.next_due - 1e-9:
                    t.next_due += t.period
                    t.callback()


def test_steps_run_in_order_at_the_right_times():
    node = FakeNode()
    log = []
    r = StepRunner(node)
    assert r.start('demo', [(2.0, 'a', lambda: log.append(('a', round(node.now, 2)))),
                            (1.0, 'b', lambda: log.append(('b', round(node.now, 2)))),
                            (0.0, 'c', lambda: log.append(('c', round(node.now, 2))))])
    assert r.busy
    node.advance(1.9)
    assert log == []                          # nothing early
    node.advance(0.2)
    assert log == [('a', 2.0)]
    node.advance(1.0)
    assert log == [('a', 2.0), ('b', 3.0), ('c', 3.0)]   # zero delay = right after b
    assert not r.busy and r.name == ''
    assert node.timers == []                  # every one-shot timer was destroyed


def test_each_step_runs_once_although_ros_timers_repeat():
    node = FakeNode()
    count = []
    r = StepRunner(node)
    r.start('once', [(1.0, 'x', lambda: count.append(1))])
    node.advance(10.0)
    assert count == [1]


def test_second_sequence_is_refused_while_one_runs():
    node = FakeNode()
    r = StepRunner(node)
    r.start('first', [(5.0, 'x', lambda: None)])
    assert r.start('second', [(1.0, 'y', lambda: None)]) is False
    assert any('not started' in m for lvl, m in node.logger.lines if lvl == 'warn')
    node.advance(5.1)
    assert r.start('second', [(1.0, 'y', lambda: None)]) is True


def test_a_failing_step_does_not_stop_the_rest():
    node = FakeNode()
    log = []

    def boom():
        raise RuntimeError('servo service gone')
    r = StepRunner(node)
    r.start('drop', [(1.0, 'open', boom), (1.0, 'rtl', lambda: log.append('rtl'))])
    node.advance(2.1)
    assert log == ['rtl']
    assert any('open' in m for lvl, m in node.logger.lines if lvl == 'error')


def test_cancel_drops_the_remaining_steps():
    node = FakeNode()
    log = []
    r = StepRunner(node)
    r.start('x', [(1.0, 'a', lambda: log.append('a')), (1.0, 'b', lambda: log.append('b'))])
    node.advance(1.1)
    r.cancel()
    node.advance(5.0)
    assert log == ['a'] and not r.busy and node.timers == []


def test_call_later():
    node = FakeNode()
    log = []
    call_later(node, 3.0, lambda: log.append(node.now))
    node.advance(2.9)
    assert log == []
    node.advance(0.2)
    assert len(log) == 1 and abs(log[0] - 3.0) < 0.02


def test_main_controller_drop_sequence_timing():
    """The last target: loiter 6 s, open, 3 s, close, 6 s, RTL (same timing as the old sleeps)."""
    node = FakeNode()
    events = []
    loiter, servo_open = 6.0, 3.0
    r = StepRunner(node)
    r.start('drop on the person', [
        (loiter, 'open', lambda: events.append(('open', round(node.now, 2)))),
        (servo_open, 'close', lambda: events.append(('close', round(node.now, 2)))),
        (loiter, 'rtl', lambda: events.append(('rtl', round(node.now, 2)))),
    ])
    node.advance(20.0)
    assert events == [('open', 6.0), ('close', 9.0), ('rtl', 15.0)]
