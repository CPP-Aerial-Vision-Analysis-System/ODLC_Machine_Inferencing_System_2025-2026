"""
Run a short list of timed steps on a ROS node without ever blocking it.

timed_steps.py -- PURPOSE: run steps like "wait 6 s, open the servo, wait 3 s, close it, ..." one
after another on a ROS node WITHOUT blocking it. main_controller.py uses it for its
loiter-and-drop and resume-AUTO sequences.

Why (October 2026): main_controller used time.sleep() inside its callbacks (6 s loiter, 3 s servo
opening, 6 s before RTL, 4 s + 1 s before resuming AUTO). A sleeping callback freezes the whole
node: no detections, no waypoint messages, no service answers are handled meanwhile. Here every
wait is a one-shot ROS timer instead, so the node keeps working between the steps.

How to use:
    runner = StepRunner(node)
    runner.start('drop on the tent', [
        (6.0, 'open the release servo', open_fn),     # wait 6 s, then call open_fn()
        (3.0, 'close the release servo', close_fn),   # 3 s after that, call close_fn()
    ])
    runner.busy   -> True until the last step has run

Works with any object that has ROS-style create_timer(period_s, callback) / destroy_timer(timer)
and get_logger() methods (a real rclpy Node in flight, a fake clock node in the unit tests,
../test/test_timed_steps.py). No ROS imports here.
"""

import traceback


class StepRunner:
    """Run (delay_s, label, function) steps in order, each after its delay, never blocking."""

    def __init__(self, node):
        """
        Set up an idle runner.

        Parameters
        ----------
        node
            The node whose timers are used (an rclpy Node).

        """
        self._node = node
        self._steps = []
        self._timer = None
        self.name = ''               # name of the sequence that is running ('' when idle)
        self.last_label = ''         # label of the last step that ran

    @property
    def busy(self) -> bool:
        """Tell whether a sequence is running (a step is waiting or about to run)."""
        return self._timer is not None or bool(self._steps)

    def start(self, name: str, steps) -> bool:
        """
        Start a sequence; return False (and change nothing) if another one is still running.

        Parameters
        ----------
        name
            Short name for the log, e.g. 'drop on the tent'.
        steps
            List of (delay_s, label, function); each function is called with no arguments
            delay_s seconds after the previous step (0 = right away).

        Returns
        -------
        bool
            True if started.

        """
        if self.busy:
            self._node.get_logger().warn(
                f"Sequence '{name}' not started: '{self.name}' is still running")
            return False
        self.name = name
        self._steps = list(steps)
        self._schedule_next()
        return True

    def cancel(self) -> None:
        """Stop the running sequence; its remaining steps are dropped."""
        self._drop_timer()
        self._steps = []
        self.name = ''

    def _drop_timer(self) -> None:
        timer, self._timer = self._timer, None
        if timer is not None:
            timer.cancel()
            self._node.destroy_timer(timer)

    def _schedule_next(self) -> None:
        """Run every step that is due now; arm a one-shot timer for the next delayed one."""
        while self._steps:
            delay = float(self._steps[0][0])
            if delay > 0.0:
                self._timer = self._node.create_timer(delay, self._on_timer)
                return
            _delay, label, function = self._steps.pop(0)
            self._run(label, function)
            if self._timer is not None:
                return                   # that step started a new sequence itself
        self.name = ''

    def _on_timer(self) -> None:
        # ROS timers repeat: this one has done its job, so remove it before running the step.
        self._drop_timer()
        if not self._steps:
            self.name = ''
            return
        _delay, label, function = self._steps.pop(0)
        self._run(label, function)
        if self._timer is None:          # (unless that step started a new sequence itself)
            self._schedule_next()

    def _run(self, label: str, function) -> None:
        self.last_label = label
        try:
            function()
        except Exception:  # noqa: BLE001 - one failing step must not stop the rest (e.g. RTL)
            self._node.get_logger().error(
                f"Step '{label}' of '{self.name}' failed: {traceback.format_exc()}")


def call_later(node, delay_s: float, function):
    """
    Call function() once, delay_s seconds from now, on node's executor (a one-shot timer).

    Returns
    -------
    StepRunner
        The runner doing it (keep it if you want to cancel()).

    """
    runner = StepRunner(node)
    runner.start('call later', [(delay_s, 'call', function)])
    return runner
