"""Unit tests for mission_logic.find_last_two_nav_waypoints.

These are REAL unit tests: each one feeds the function a known input and
asserts the exact output. They need no ROS, no drone, no network — so they
run in milliseconds on any machine, including CI.

Run locally with:   pytest tests/unit
"""

import os
import sys

# Make the pure-logic module importable without installing the ROS package.
# (mission_logic.py lives next to main_controller.py in the `main` package.)
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "ros2_ws", "src", "main", "src"
    ),
)

from mission_logic import find_last_two_nav_waypoints  # noqa: E402

# Command IDs used below: 16 = NAV_WAYPOINT, 183 = DO_SET_SERVO (not a NAV cmd).


def test_finds_last_two_nav_waypoints():
    # indices:   0   1   2    3    4
    commands = [16, 16, 16, 183, 20]  # mission with a servo + a loiter mixed in
    rtl_index = 4
    last_nav, buffer_wp = find_last_two_nav_waypoints(commands, rtl_index)
    assert last_nav == 2   # index 3 is DO_SET_SERVO (skipped), so last NAV is 2
    assert buffer_wp == 1  # the NAV before that


def test_servo_items_are_not_counted_as_nav():
    commands = [16, 183, 183, 16, 183]
    rtl_index = 5
    last_nav, buffer_wp = find_last_two_nav_waypoints(commands, rtl_index)
    assert last_nav == 3
    assert buffer_wp == 0


def test_only_one_nav_waypoint_leaves_buffer_unset():
    commands = [183, 183, 16]
    rtl_index = 3
    last_nav, buffer_wp = find_last_two_nav_waypoints(commands, rtl_index)
    assert last_nav == 2
    assert buffer_wp == -1


def test_no_nav_waypoints_returns_minus_one():
    commands = [183, 183, 183]
    rtl_index = 3
    assert find_last_two_nav_waypoints(commands, rtl_index) == (-1, -1)


def test_rtl_index_zero_returns_minus_one():
    # rtl_index must be > 0 for there to be anything before it.
    assert find_last_two_nav_waypoints([16, 16], 0) == (-1, -1)


def test_empty_mission_returns_minus_one():
    assert find_last_two_nav_waypoints([], 5) == (-1, -1)


def test_loiter_commands_count_as_nav():
    # 17-21 are NAV_LOITER_* and should be treated as navigation waypoints.
    commands = [18, 19, 183]
    rtl_index = 3
    last_nav, buffer_wp = find_last_two_nav_waypoints(commands, rtl_index)
    assert last_nav == 1
    assert buffer_wp == 0
