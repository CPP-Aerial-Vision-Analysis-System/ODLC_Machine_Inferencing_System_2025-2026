"""
sim_demo/test/test_phases.py -- PURPOSE: unit tests of the mission phase tracker (phases.py).
It replays the messages a full search mission produces - the exact status texts the team's
main_controller, mapping trigger and video_cam send, ArduPilot's "Mission: N <type>" texts,
/mavros/state and /mavros/mission/reached - and checks every phase on the strip.
"""
from sim_demo.mission_file import analyze_items, load_waypoints_file
from sim_demo.phases import (DROPPING, FLYING_TO_TARGET, LANDED, PROCESSING, RETURNING, SEARCHING,
                             TAKING_OFF, WAITING, PhaseTracker)


def _tracker(world):
    tr = PhaseTracker()
    tr.set_mission(analyze_items(load_waypoints_file(world.mission_path)))
    return tr


def test_full_mission_sequence(world):
    tr = _tracker(world)
    t = 0.0
    tr.on_state(t, False, False, '')
    assert tr.phase == WAITING and 'MAVROS' in tr.detail
    tr.on_state(t, True, False, 'STABILIZE')
    assert tr.detail == 'on the ground, motors off'
    tr.on_state(1, True, True, 'GUIDED')
    assert tr.phase == TAKING_OFF
    tr.on_statustext(2, 'Mission: 1 Takeoff')
    tr.on_rel_alt(3, 12.0)
    assert tr.phase == TAKING_OFF and 'of 30 m' in tr.detail
    tr.on_statustext(4, 'Mission: 2 DigiCamCtrl')
    assert tr.phase == SEARCHING and tr.detail.startswith('flying to the search area')
    tr.on_statustext(4.1, 'Camera trigger STARTED')
    tr.on_statustext(5, 'SUCCESS: Captured 2K image #1')
    tr.on_statustext(5.5, 'Zoom set to 5.0x')
    assert tr.photos_captured == 1 and tr.zoom == 5.0
    tr.on_reached(6, 3)
    assert tr.detail == 'lane 1 of 3'
    tr.on_reached(7, 4)
    tr.on_reached(8, 5)
    assert tr.detail == 'lane 2 of 3'
    tr.on_statustext(9, 'Detected tent')
    assert 'tent' in tr.detected_classes
    tr.on_reached(10, 8)
    assert tr.phase == SEARCHING and tr.detail == 'end of the search pattern'
    tr.on_statustext(10.5, 'Reached last WP. Holding to finish image processing before RTL')
    assert tr.phase == PROCESSING
    tr.on_state(10.6, True, True, 'GUIDED')
    tr.on_statustext(20, 'All 42 images processed. Selecting targets.')
    assert tr.phase == PROCESSING and 'choosing' in tr.detail
    tr.on_statustext(21, 'Targets: tent@9 -> person@10. Diverting.')
    assert tr.phase == FLYING_TO_TARGET and tr.target_plan == [('tent', 9), ('person', 10)]
    tr.on_statustext(26, 'Set current WP -> 9')
    tr.on_statustext(27, 'Resumed AUTO -> WP 9')
    tr.on_state(27, True, True, 'AUTO')
    assert tr.phase == FLYING_TO_TARGET and 'tent' in tr.detail
    tr.on_reached(40, 9)
    assert tr.phase == DROPPING and tr.current_target == 'tent'
    tr.on_statustext(40.1, 'Over tent: loiter 6s then drop')
    tr.on_state(40.2, True, True, 'GUIDED')
    tr.on_statustext(46.5, 'Servo 13 -> 1400')
    assert tr.drops == ['tent'] and 'released' in tr.detail
    tr.on_statustext(49.5, 'Servo 13 -> 500')
    tr.on_state(49.6, True, True, 'AUTO')                 # off to the next target
    assert tr.phase == FLYING_TO_TARGET and tr.current_target == 'person'
    tr.on_statustext(60, 'Over person: loiter 6s then drop')
    assert tr.phase == DROPPING
    tr.on_statustext(66, 'Servo 11 -> 1400')
    assert tr.drops == ['tent', 'person']
    tr.on_statustext(69, 'All targets done. Waiting 6s then RTL.')
    tr.on_state(75, True, True, 'RTL')
    assert tr.phase == RETURNING
    tr.on_statustext(75.1, 'RTL')
    tr.on_rel_alt(90, 0.1)
    tr.on_state(95, True, False, 'LAND')
    assert tr.phase == LANDED
    phases = [e.data['phase'] for e in tr.history]
    assert phases == [TAKING_OFF, SEARCHING, PROCESSING, FLYING_TO_TARGET, DROPPING,
                      FLYING_TO_TARGET, DROPPING, RETURNING, LANDED]
    drops = [e for e in tr.events if e.kind == 'drop']
    assert [(d.data['cls'], d.data['channel']) for d in drops] == [('tent', 13), ('person', 11)]


def test_no_targets_goes_home(world):
    tr = _tracker(world)
    tr.on_state(0, True, True, 'AUTO')
    tr.on_rel_alt(1, 30)
    tr.on_statustext(2, 'Mission: 2 DigiCamCtrl')
    tr.on_statustext(3, 'Reached last WP. Holding to finish image processing before RTL')
    tr.on_statustext(4, 'No valid targets found. Returning to launch.')
    assert tr.phase == RETURNING and 'no targets' in tr.detail
    tr.on_state(5, True, True, 'RTL')
    assert tr.phase == RETURNING


def test_disarm_on_the_ground_returns_to_waiting(world):
    tr = _tracker(world)
    tr.on_state(0, True, True, 'GUIDED')
    assert tr.phase == TAKING_OFF
    tr.on_state(5, True, False, 'GUIDED')
    assert tr.phase == WAITING


def test_reached_takeoff_starts_search_without_digicam(world):
    tr = _tracker(world)
    tr.on_state(0, True, True, 'AUTO')
    tr.on_rel_alt(1, 29.0)
    tr.on_reached(2, 1)
    assert tr.phase == SEARCHING


def test_late_servo_text_after_auto_still_moves_on(world):
    """Real-simulator order: main_controller sleeps in its callback, so AUTO (next target)
    reaches us before its "Servo 13 -> 1400" text."""
    tr = _tracker(world)
    tr.on_state(0, True, True, 'AUTO')
    tr.on_rel_alt(1, 30)
    tr.on_statustext(2, 'Targets: tent@11 -> person@12. Diverting.')
    tr.on_statustext(10, 'Over tent: loiter 6s then drop')
    tr.on_state(10.1, True, True, 'GUIDED')
    tr.on_state(19.0, True, True, 'AUTO')          # already flying to the person ...
    assert tr.phase == DROPPING                     # (no drop seen yet)
    tr.on_statustext(19.1, 'Servo 13 -> 1400')      # ... when the late text arrives
    assert tr.phase == FLYING_TO_TARGET and tr.current_target == 'person'
    tr.on_statustext(35, 'Over person: loiter 6s then drop')
    assert tr.phase == DROPPING and tr.current_target == 'person'


def test_mission_rtl_text_waits_for_main_controller(world):
    """ArduPilot says "Mission: 11 RTL" right when the last search waypoint is reached; one
    second later main_controller switches to GUIDED to hold: no "Returning" in between."""
    tr = _tracker(world)
    tr.on_state(0, True, True, 'AUTO')
    tr.on_rel_alt(1, 30)
    tr.on_statustext(2, 'Mission: 2 DigiCamCtrl')
    tr.on_reached(50, 8)
    tr.on_statustext(50.1, 'Mission: 9 RTL')
    tr.on_state(51.0, True, True, 'GUIDED')
    tr.on_statustext(51.1, 'Reached last WP. Holding to finish image processing before RTL')
    tr.on_rel_alt(60.0, 30)
    assert RETURNING not in [e.data['phase'] for e in tr.history]
    assert tr.phase == PROCESSING


def test_mission_rtl_without_main_controller_becomes_returning(world):
    tr = _tracker(world)
    tr.on_state(0, True, True, 'AUTO')
    tr.on_rel_alt(1, 30)
    tr.on_statustext(2, 'Mission: 2 DigiCamCtrl')
    tr.on_statustext(50, 'Mission: 9 RTL')
    tr.on_rel_alt(52, 30)
    assert tr.phase == SEARCHING                    # still within the grace time
    tr.on_rel_alt(55, 29)
    assert tr.phase == RETURNING and 'mission RTL' in tr.detail


def test_failed_servo_is_not_a_drop():
    tr = PhaseTracker()
    tr.on_statustext(1, 'Servo 13 move FAILED')
    assert tr.drops == []
