"""
sim_bringup/test/test_phase_monitor.py -- PURPOSE: unit tests for the phase monitor's state machine (no ROS): a whole
mission is replayed as the texts and numbers MAVROS and the team's nodes publish, and the printed phases, detections,
drops and the "landed" end condition are checked.
"""

from sim_bringup.phase_monitor import PHASES, MissionState


def phases_of(events):
    return [e.phase for e in events if e.kind == 'phase']


def test_full_mission_goes_through_every_phase_in_order():
    s = MissionState()
    seen = []
    seen += phases_of(s.on_state(armed=True, mode='GUIDED'))
    seen += phases_of(s.on_state(armed=True, mode='AUTO'))
    seen += phases_of(s.on_autopilot_text('Mission: 1 Takeoff'))
    seen += phases_of(s.on_altitude(10.0))
    seen += phases_of(s.on_autopilot_text('Mission: 3 DigiCamCtrl'))
    seen += phases_of(s.on_team_text('Camera trigger STARTED'))
    seen += phases_of(s.on_team_text('Reached last WP. Holding to finish image processing before RTL'))
    seen += phases_of(s.on_team_text('Targets: tent@9 -> person@10. Diverting.'))
    seen += phases_of(s.on_team_text('Over tent: loiter 6s then drop'))
    seen += phases_of(s.on_team_text('Over person: loiter 6s then drop'))
    seen += phases_of(s.on_state(armed=True, mode='RTL'))
    seen += phases_of(s.on_altitude(0.2))
    assert seen == ['ARMED', 'TAKEOFF', 'SEARCH', 'HOLD', 'TARGETS', 'DROP', 'RTL']
    assert not s.finished
    final = phases_of(s.on_state(armed=False, mode='RTL'))
    assert final == ['LANDED'] and s.finished


def test_phases_never_go_backwards_except_drop_to_targets():
    s = MissionState()
    s.on_state(True, 'AUTO')
    s.on_altitude(20.0)
    s.on_team_text('Targets: tent@9 -> person@10. Diverting.')
    assert phases_of(s.on_autopilot_text('Mission: 3 DigiCamCtrl')) == []      # no going back to SEARCH
    assert phases_of(s.on_team_text('Over tent: loiter 6s then drop')) == ['DROP']
    assert s.phase == 'DROP'


def test_mission_rtl_item_is_not_rtl_when_main_controller_holds():
    # Seen in the simulator: "Mission: 7 RTL" arrives the moment the last waypoint is reached, but the team's
    # main_controller switches to GUIDED at the same time to hold while the detector finishes.
    s = MissionState()
    s.on_state(True, 'AUTO', now=0.0)
    s.on_altitude(20.0)
    assert phases_of(s.on_autopilot_text('Mission: 7 RTL', now=10.0)) == []
    s.on_team_text('Reached last WP. Holding to finish image processing before RTL')
    assert phases_of(s.on_state(True, 'GUIDED', now=11.0)) == []
    assert phases_of(s.on_state(True, 'GUIDED', now=20.0)) == [] and s.phase == 'HOLD'
    assert phases_of(s.on_state(True, 'RTL', now=30.0)) == ['RTL']


def test_mission_rtl_item_counts_when_the_drone_stays_in_auto():
    # Without the team stack (fly_mission.sh alone) the mission's own RTL item flies the drone home in AUTO.
    s = MissionState()
    s.on_state(True, 'AUTO', now=0.0)
    s.on_altitude(20.0)
    s.on_autopilot_text('Mission: 7 RTL', now=10.0)
    assert phases_of(s.on_state(True, 'AUTO', now=11.0)) == []
    assert phases_of(s.on_state(True, 'AUTO', now=13.5)) == ['RTL']


def test_servo_texts_become_payload_lines():
    s = MissionState()
    events = s.on_team_text('Servo 13 -> 1400')
    assert any('tent payload' in e.text for e in events) and s.drops == ['tent payload']
    assert s.on_team_text('Servo 13 -> 500') == []                               # closing is not a drop
    assert s.on_team_text('Servo 11 -> 1400') and s.drops[-1].startswith('person')


def test_detection_lines_and_counts():
    s = MissionState()
    events = s.on_detection('36.2 , -96.0.jpg', [('person', 0.87), ('tent', 0.55)], 36.2, -96.0)
    assert len(events) == 2 and 'PERSON' in events[0].text and '87%' in events[0].text
    assert s.detections['person'] == [0.87]
    no_gps = s.on_detection('IMG_0001.jpg', [('tent', 0.4)], 0.0, 0.0)
    assert 'NO GPS' in no_gps[0].text


def test_landing_needs_a_flight_first():
    s = MissionState()
    assert s.on_state(armed=False, mode='STABILIZE') == []          # sitting on the ground is not "landed"
    s.on_state(True, 'GUIDED')
    s.on_landed_state(2)                                           # MAVROS: in air
    s.on_landed_state(1)                                           # on ground, but still armed
    assert not s.finished
    assert phases_of(s.on_state(False, 'LAND')) == ['LANDED']


def test_photo_counter_prints_first_and_every_tenth():
    s = MissionState()
    assert len(s.on_photo_count(1)) == 1
    assert s.on_photo_count(5) == []
    assert len(s.on_photo_count(10)) == 1
    assert s.on_photo_count(10) == []


def test_summary_mentions_everything():
    s = MissionState()
    s.on_detection('a.jpg', [('person', 0.9)], 1.0, 2.0)
    s.on_team_text('Targets: person@9. Diverting.')
    text = ' '.join(s.summary())
    assert 'person: 1' in text and 'Targets: person@9' in text and len(PHASES) == 9
