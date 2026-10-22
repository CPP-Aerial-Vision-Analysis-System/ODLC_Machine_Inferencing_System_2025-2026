"""
Unit tests for waypoint_mavros/mission_sync.py (keeping the mission copy fresh).

test_mission_sync.py -- PURPOSE: unit tests for the rules that keep waypoint_manager's copy of the
mission fresh (the "stale mission" fix of October 2026): the mission indices are derived from
scratch for every list (same arithmetic as before), only changed indices are re-published,
"Flight plan received" is recognised, and the re-read policy waits, merges triggers, ignores the
echo of waypoint_manager's own upload, waits for MAVROS, and retries failed reads.

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/waypoint_mavros/test/test_mission_sync.py -v
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from waypoint_mavros.mission_sync import (  # noqa: E402
    ACTION_PULL,
    NAV_RETURN_TO_LAUNCH,
    NAV_TAKEOFF,
    MissionRepullPolicy,
    changed_indices,
    derive_mission_indices,
    is_new_mission_text,
)

WP, DIGICAM, SPEED = 16, 203, 178
# The simulator's runway survey mission (sim_worlds/missions/runway_survey.waypoints):
# 0 home, 1 TAKEOFF, 2 WP, 3 DO_DIGICAM_CONTROL, 4 DO_CHANGE_SPEED, 5..10 WP, 11 RTL.
RUNWAY = [WP, NAV_TAKEOFF, WP, DIGICAM, SPEED, WP, WP, WP, WP, WP, WP, NAV_RETURN_TO_LAUNCH]


# ------------------------------------------------------------------ indices
def test_runway_mission_indices():
    assert derive_mission_indices(RUNWAY) == {
        'num_waypoints': 12, 'takeoff_index': 1, 'next_after_takeoff': 2, 'rtl_index': 11,
        'last_before_rtl': 10, 'buffer_wp': 9}


def test_empty_mission_gives_the_not_known_values():
    # What MAVROS reports before any upload: nothing (or only home).
    for cmds in ([], [WP]):
        idx = derive_mission_indices(cmds)
        assert idx['takeoff_index'] == -1 and idx['rtl_index'] == -1
        assert idx['last_before_rtl'] == -1 and idx['buffer_wp'] == -2


def test_indices_are_not_left_over_from_the_previous_mission():
    derive_mission_indices(RUNWAY)
    # A new mission without TAKEOFF / RTL must not keep the old 1 / 11 (the old code did).
    idx = derive_mission_indices([WP, WP, WP])
    assert idx['takeoff_index'] == -1 and idx['rtl_index'] == -1 and idx['buffer_wp'] == -2


def test_inserted_targets_move_rtl_down():
    # main_controller inserts tent@11 and person@12 before RTL: RTL becomes 13.
    with_targets = RUNWAY[:11] + [WP, WP] + [NAV_RETURN_TO_LAUNCH]
    idx = derive_mission_indices(with_targets)
    assert idx['rtl_index'] == 13 and idx['last_before_rtl'] == 12 and idx['buffer_wp'] == 11


def test_rtl_right_after_home_has_no_last_before_rtl():
    idx = derive_mission_indices([WP, NAV_RETURN_TO_LAUNCH])
    assert idx['rtl_index'] == 1 and idx['last_before_rtl'] == -1 and idx['buffer_wp'] == -2


def test_takeoff_as_last_item_has_no_next():
    idx = derive_mission_indices([WP, NAV_TAKEOFF])
    assert idx['takeoff_index'] == 1 and idx['next_after_takeoff'] == -1


def test_changed_indices_only_lists_differences():
    old = derive_mission_indices(RUNWAY)
    assert changed_indices(old, dict(old)) == {}
    new = dict(old, rtl_index=13, num_waypoints=14)
    assert changed_indices(old, new) == {'rtl_index': 13, 'num_waypoints': 14}
    assert changed_indices({}, {'a': 1}) == {'a': 1}


# ------------------------------------------------------------------ autopilot text
def test_flight_plan_received_is_recognised():
    assert is_new_mission_text('Flight plan received')
    assert is_new_mission_text('flight plan received')
    assert not is_new_mission_text('Mission: 3 DigiCamCtrl')
    assert not is_new_mission_text('')
    assert not is_new_mission_text(None)


# ------------------------------------------------------------------ re-read policy
def test_upload_text_schedules_one_pull_after_the_delay():
    p = MissionRepullPolicy(delay_s=2.0)
    assert p.on_new_mission_text(100.0) is True
    assert p.next_action(101.9, True) is None
    assert p.next_action(102.0, True) == ACTION_PULL
    p.pulled(102.0)
    assert p.next_action(102.5, True) is None          # in flight
    assert p.finished(102.6, ok=True) == 'ok'
    assert p.next_action(110.0, True) is None          # nothing more wanted
    assert p.pulls_ok == 1


def test_triggers_close_together_merge_into_one_pull():
    p = MissionRepullPolicy(delay_s=2.0)
    p.on_new_mission_text(10.0)
    p.on_armed(10.5)
    p.on_new_mission_text(11.0)
    assert p.next_action(12.0, True) == ACTION_PULL
    assert 'armed' in p.reason and 'uploaded' in p.reason
    p.pulled(12.0)
    p.finished(12.3, ok=True)
    assert p.next_action(20.0, True) is None


def test_trigger_during_a_pull_gives_one_more_pull_afterwards():
    p = MissionRepullPolicy(delay_s=1.0)
    p.on_armed(0.0)
    assert p.next_action(0.5, True) == ACTION_PULL
    p.pulled(0.5)
    p.on_new_mission_text(0.6)                         # arrives while the pull runs
    p.finished(0.8, ok=True)
    assert p.next_action(1.7, True) is None
    assert p.next_action(1.8, True) == ACTION_PULL


def test_echo_of_our_own_push_is_ignored():
    p = MissionRepullPolicy(delay_s=2.0, own_push_quiet_s=15.0)
    p.on_own_push(50.0)                                # main_controller inserted the targets
    assert p.on_new_mission_text(50.3) is False        # ArduPilot's echo of that upload
    assert p.next_action(60.0, True) is None
    assert p.on_new_mission_text(70.0) is True         # a real upload later is handled again


def test_no_pull_while_our_own_push_may_be_running():
    p = MissionRepullPolicy(delay_s=0.5, own_push_quiet_s=15.0)
    p.on_armed(100.0)
    p.on_own_push(100.2)
    assert p.next_action(101.0, True) is None          # postponed to the end of the quiet window
    assert p.next_action(115.1, True) is None
    assert p.next_action(115.3, True) == ACTION_PULL


def test_waits_for_the_mavros_service():
    p = MissionRepullPolicy(delay_s=1.0, retry_delay_s=2.0)
    p.on_new_mission_text(0.0)
    assert p.next_action(1.0, service_ready=False) is None
    assert p.next_action(2.9, True) is None
    assert p.next_action(3.0, True) == ACTION_PULL


def test_failed_pull_is_retried_then_given_up():
    p = MissionRepullPolicy(delay_s=1.0, retry_delay_s=2.0, max_attempts=3)
    p.on_new_mission_text(0.0)
    t = 1.0
    outcomes = []
    for _ in range(3):
        assert p.next_action(t, True) == ACTION_PULL
        p.pulled(t)
        outcomes.append(p.finished(t + 0.1, ok=False))
        t += 2.5                                       # past the 2 s back-off
    assert outcomes == ['retry', 'retry', 'gave_up']
    assert p.next_action(t + 10, True) is None
