"""
Read waypoint_manager's mission-index parameters from another node.

wp_sender/parameter.py -- PURPOSE: read the mission-index parameters that waypoint_manager
publishes (num_waypoints, takeoff_index, rtl_index, next_after_takeoff, last_before_rtl,
buffer_wp) from another node. Two tools, for two different situations:

  AsyncParameterReader  USE THIS INSIDE A RUNNING NODE (main_controller does). It never blocks
                        and never spins: it sends the request with call_async and handles the
                        answer in a done-callback on the caller's OWN executor. When a read fails
                        or times out, the caller simply keeps its previous values; the reader logs
                        why and tries again until a read succeeds.

  ParameterManager      the original blocking helper (a second node +
                        spin_until_future_complete). Only for command-line tools such as
                        `ros2 run wp_sender test_param`. NEVER call get_param() from inside a
                        callback: spinning inside a callback re-enters the executor, the read
                        times out after 10 s and returns None. That is exactly how
                        main_controller crashed (AttributeError on None) in 2 of 4 simulator runs
                        in October 2026.

The pure bookkeeping (value decoding, when to send/abandon/retry) lives in
wp_sender/param_tracking.py and is unit-tested in test/test_parameter_reader.py.

Example (inside a node's __init__):
    self.reader = AsyncParameterReader(self, 'waypoint_manager',
                                       ['takeoff_index', 'rtl_index'], self.on_indices)
    self.reader.request('start-up')      # and again whenever the values may have changed
    ...
    def on_indices(self, values):        # called on this node's executor with a dict
        self.rtl_index = values['rtl_index']
"""

import os
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from wp_sender.param_tracking import (
    ACTION_SEND,
    ACTION_TIMEOUT,
    ParameterReadTracker,
    decode_parameter_values,
)


class AsyncParameterReader:
    """
    Read parameters of another node without blocking (safe to use from any callback).

    The service client and a small timer are created ON THE CALLER'S NODE, so the answers are
    handled by the caller's own executor: no second node, no nested spinning.
    """

    def __init__(self, node, remote_node: str, names, on_values,
                 timeout_s: float = 5.0, retry_delay_s: float = 2.0, tick_s: float = 0.5):
        """
        Create the client and the timer on `node`.

        Parameters
        ----------
        node
            The rclpy Node that owns the client (e.g. main_controller).
        remote_node
            Name of the node whose parameters are read, e.g. 'waypoint_manager'.
        names
            List of parameter names to read together.
        on_values
            function(dict) called with {name: value} after every successful read. A read only
            counts as successful when EVERY name has a value.
        timeout_s
            Abandon a request with no answer after this many seconds.
        retry_delay_s
            Wait this long after a failure before trying again.
        tick_s
            How often the reader checks for timeouts and retries.

        """
        self._node = node
        self._names = list(names)
        self._on_values = on_values
        self._service = '/' + remote_node.strip('/') + '/get_parameters'
        self._client = node.create_client(GetParameters, self._service)
        self._tracker = ParameterReadTracker(timeout_s, retry_delay_s)
        self._future = None
        self._reason = ''
        self._waiting_logged = False
        self._timer = node.create_timer(tick_s, self._tick)

    @property
    def failures(self) -> int:
        """Count failed reads in a row (0 after a success)."""
        return self._tracker.failures

    def request(self, reason: str = '') -> None:
        """Ask for a fresh read (many calls while one read runs give ONE extra read)."""
        if reason:
            self._reason = reason
        self._tracker.request()
        self._tick()

    def _now(self) -> float:
        return time.monotonic()

    def _log(self):
        return self._node.get_logger()

    def _tick(self) -> None:
        """Send, abandon or wait, as the tracker says (timer callback and request() hook)."""
        ready = self._client.service_is_ready()
        action = self._tracker.next_action(self._now(), ready)
        if action == ACTION_SEND:
            self._send()
        elif action == ACTION_TIMEOUT:
            future, self._future = self._future, None
            if future is not None:
                future.cancel()            # a late answer to it is ignored
            self._tracker.finished(self._now(), ok=False)
            self._log_failure(f'no answer within {self._tracker.timeout_s:.0f} s')
        elif self._tracker.wanted and not ready and not self._waiting_logged:
            self._waiting_logged = True
            self._log().info(
                f'Waiting for {self._service} to read {", ".join(self._names)} ...')

    def _send(self) -> None:
        request = GetParameters.Request()
        request.names = list(self._names)
        now = self._now()
        try:
            future = self._client.call_async(request)
        except Exception as exc:  # noqa: BLE001 - never let a client error kill the caller
            self._tracker.sent(now)
            self._tracker.finished(now, ok=False)
            self._log_failure(f'could not send the request ({exc})')
            return
        self._tracker.sent(now)
        self._waiting_logged = False
        self._future = future
        future.add_done_callback(self._on_done)

    def _on_done(self, future) -> None:
        """Handle the answer (or the cancellation) on the caller's executor."""
        if future is not self._future:
            return                          # an abandoned (timed-out) request answering late
        self._future = None
        if future.cancelled():
            return
        try:
            response = future.result()
            values = decode_parameter_values(self._names, response.values)
        except Exception as exc:  # noqa: BLE001
            self._tracker.finished(self._now(), ok=False)
            self._log_failure(f'the service call failed ({exc})')
            return
        missing = [n for n in self._names if values.get(n) is None]
        if missing:
            self._tracker.finished(self._now(), ok=False)
            self._log_failure(f'not set (yet): {", ".join(missing)}')
            return
        self._tracker.finished(self._now(), ok=True)
        try:
            self._on_values(values)
        except Exception as exc:  # noqa: BLE001 - a bug in the caller's handler must not kill us
            self._log().error(f'Handler for {self._service} values raised: {exc}')

    def _log_failure(self, why: str) -> None:
        n = self._tracker.failures
        # The first few failures are all logged; after that only every tenth, so a missing
        # waypoint_manager cannot flood the log.
        if n <= 3 or n % 10 == 0:
            reason = f' (asked because: {self._reason})' if self._reason else ''
            self._log().warn(
                f'Could not read {", ".join(self._names)} from {self._service}: {why}{reason}. '
                f'Keeping the previous values; trying again in '
                f'{self._tracker.retry_delay_s:.0f} s (failure {n} in a row).')


class ParameterManager(Node):
    """Blocking parameter reader for COMMAND-LINE tools only (see the module docstring)."""

    def __init__(self, node_name=None):
        # Node names must be unique across the whole ROS graph. Several
        # processes in tracker.launch.xml build a ParameterManager, so a fixed
        # name gave two live /parameter_manager nodes -- undefined behaviour in
        # ROS 2, and service replies could be matched to the wrong client,
        # hanging the loser forever.
        super().__init__(node_name or 'parameter_manager_%d' % os.getpid())

        self.waypoint_client = self.create_client(GetParameters, "waypoint_manager/get_parameters")

        self._wait_for_services()

        # Wait for all services to be available
    def _wait_for_services(self, overall_timeout_sec=30.0):
        # Bounded: an unbounded wait here blocks whatever thread built us.
        clients = [('waypoint_manager/get_parameters', self.waypoint_client),]
        for name, client in clients:
            waited = 0.0
            while not client.wait_for_service(timeout_sec=1.0):
                waited += 1.0
                if waited >= overall_timeout_sec:
                    self.get_logger().error(
                        f'{name} not available after {overall_timeout_sec:.0f}s, giving up')
                    return False
                self.get_logger().info(f'{name} service not available, waiting...')
        return True

    def get_param(self, client, list_params, timeout_sec=10.0):
        """
        Read parameters, BLOCKING up to timeout_sec; return a dict, or None on failure.

        Command-line use only. Inside a node use AsyncParameterReader instead: this method
        spins the executor, which must never happen inside a callback.
        """
        req = GetParameters.Request()
        req.names = list_params

        try:
            future = client.call_async(req)
            # Always bounded. An untimed wait here is what left the DigiCamCtrl
            # trigger node alive but never spinning.
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if not future.done():
                future.cancel()
                self.get_logger().error(
                    f'get_parameters timed out after {timeout_sec:.0f}s')
                return None

            result = future.result()
            if result is None or not hasattr(result, 'values'):
                self.get_logger().error("No result or invalid response")
                return None

            # Build a dict of {param_name: python_value} (shared, unit-tested decoder).
            return decode_parameter_values(list_params, result.values)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None


def main():
    rclpy.init()

    param_manager = ParameterManager()
    response = param_manager.get_param(param_manager.waypoint_client, list_params=['num_waypoints', 'takeoff_index', 'rtl_index', 'next_after_takeoff', 'last_before_rtl'])
    print(response)

    param_manager.destroy_node()
    rclpy.shutdown()
