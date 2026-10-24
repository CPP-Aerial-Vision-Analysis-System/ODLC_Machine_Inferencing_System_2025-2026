"""
lattice_bridge/test/test_mavlink_decoder.py -- PURPOSE: prove that the MAVLink decoder turns the messages Mission
Planner sees into the right drone position, mode, battery, target reports and releases. It uses REAL pymavlink
message objects (so a renamed field would be caught), and checks the places where a unit or a sentinel value could
quietly put the drone in the wrong place in Lattice: degrees x 1e7, millimetres, centi-degrees, "heading unknown",
"battery unknown", position (0, 0), the autopilot's clock being minutes off, the autopilot rebooting, other vehicles
and the ground station's / Jetson's heartbeats. Skipped when pymavlink is not installed.
"""

from datetime import datetime, timedelta, timezone

import pytest

pytest.importorskip('pymavlink')
from pymavlink import mavutil                                    # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mav         # noqa: E402

from lattice_bridge.events import DroneFix, DropEvent, SourceNotice, TargetSighting      # noqa: E402
from lattice_bridge.sources.mavlink_source import (                                      # noqa: E402
    ARDUCOPTER_MODES, MavlinkDecoder, MavlinkDecoderConfig, copter_mode_name)

T0 = datetime(2026, 10, 24, 15, 30, 0, tzinfo=timezone.utc)


def at(seconds):
    return T0 + timedelta(seconds=seconds)


def stamped(message, sysid=1, compid=1):
    """Give a hand-made message the sender address a received one has."""
    message._header.srcSystem = sysid
    message._header.srcComponent = compid
    return message


def position(boot_ms=1000, lat=36.214935, lon=-96.006203, alt_m=240.5, rel_m=30.0, v=(100, -50, -20), hdg=9000, **kw):
    return stamped(mav.MAVLink_global_position_int_message(
        boot_ms, round(lat * 1e7), round(lon * 1e7), round(alt_m * 1000), round(rel_m * 1000), v[0], v[1], v[2], hdg), **kw)


def heartbeat(mode=3, armed=True, vehicle_type=mav.MAV_TYPE_QUADROTOR, autopilot=mav.MAV_AUTOPILOT_ARDUPILOTMEGA, **kw):
    base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
    return stamped(mav.MAVLink_heartbeat_message(vehicle_type, autopilot, base, mode, mav.MAV_STATE_ACTIVE, 3), **kw)


def statustext(text, severity=6, **kw):
    return stamped(mav.MAVLink_statustext_message(severity, text.encode()), **kw)


@pytest.fixture()
def decoder():
    return MavlinkDecoder(MavlinkDecoderConfig(time_mode='arrival'))


def fixes(events):
    return [e for e in events if isinstance(e, DroneFix)]


# ------------------------------------------------------------------------------------------ the position
def test_the_position_message_is_converted_with_the_right_units(decoder):
    [fix] = fixes(decoder.feed(position(), at(0)))
    assert (fix.lat_deg, fix.lon_deg) == (pytest.approx(36.214935), pytest.approx(-96.006203))
    assert fix.alt_amsl_m == pytest.approx(240.5) and fix.alt_hae_m is None       # MAVLink height is above SEA LEVEL
    assert fix.rel_alt_m == pytest.approx(30.0) and fix.heading_deg == pytest.approx(90.0)
    assert fix.velocity_ned_mps == (pytest.approx(1.0), pytest.approx(-0.5), pytest.approx(-0.2))
    assert fix.arrival == at(0) and fix.fix_time == at(0)


@pytest.mark.parametrize('lat,lon', [(0.0, 0.0), (95.0, 10.0), (36.0, 200.0), (-91.0, 0.5)])
def test_a_position_that_is_not_a_real_place_is_dropped_and_counted(decoder, lat, lon):
    message = stamped(mav.MAVLink_global_position_int_message(1000, round(lat * 1e7), round(lon * 1e7), 0, 0, 0, 0, 0, 0))
    assert decoder.feed(message, at(0)) == [] and decoder.stats['bad_fixes'] == 1


def test_heading_unknown_is_left_out_not_published_as_655_degrees(decoder):
    [fix] = fixes(decoder.feed(position(hdg=65535), at(0)))
    assert fix.heading_deg is None


def test_an_absurd_height_is_left_out_but_the_position_is_kept(decoder):
    [fix] = fixes(decoder.feed(position(alt_m=900000.0, rel_m=900000.0), at(0)))
    assert fix.alt_amsl_m is None and fix.rel_alt_m is None and fix.lat_deg == pytest.approx(36.214935)


def test_the_largest_speed_the_message_can_carry_is_passed_through_unchanged(decoder):
    [fix] = fixes(decoder.feed(position(v=(32767, -32768, 0)), at(0)))
    assert fix.velocity_ned_mps == (pytest.approx(327.67), pytest.approx(-327.68), 0.0)


# ------------------------------------------------------------------------------------------ mode, armed, battery
def test_the_flight_mode_and_armed_flag_come_from_the_autopilot_heartbeat_and_ride_on_the_next_fix(decoder):
    notices = decoder.feed(heartbeat(mode=3, armed=True), at(0))
    assert [type(n) for n in notices] == [SourceNotice] and 'vehicle found' in notices[0].message
    [fix] = fixes(decoder.feed(position(), at(0.5)))
    assert fix.mode == 'AUTO' and fix.armed is True
    decoder.feed(heartbeat(mode=9, armed=False), at(1))
    [later] = fixes(decoder.feed(position(boot_ms=2000), at(1.5)))
    assert later.mode == 'LAND' and later.armed is False


@pytest.mark.parametrize('kwargs', [
    {'vehicle_type': mav.MAV_TYPE_GCS, 'autopilot': mav.MAV_AUTOPILOT_INVALID},              # Mission Planner
    {'vehicle_type': mav.MAV_TYPE_ONBOARD_CONTROLLER, 'autopilot': mav.MAV_AUTOPILOT_INVALID},   # the Jetson's MAVROS
    {'vehicle_type': mav.MAV_TYPE_GIMBAL, 'autopilot': mav.MAV_AUTOPILOT_INVALID},
    {'vehicle_type': mav.MAV_TYPE_CAMERA, 'autopilot': mav.MAV_AUTOPILOT_INVALID},
])
def test_heartbeats_of_the_ground_station_the_jetson_and_gimbals_are_not_the_vehicle(decoder, kwargs):
    assert decoder.feed(heartbeat(**kwargs), at(0)) == [] and decoder.vehicle_sysid is None
    assert decoder.stats['other_heartbeats'] == 1


def test_the_real_vehicle_is_still_found_after_those_other_heartbeats(decoder):
    decoder.feed(heartbeat(vehicle_type=mav.MAV_TYPE_GCS, autopilot=mav.MAV_AUTOPILOT_INVALID, sysid=255, compid=190), at(0))
    decoder.feed(heartbeat(vehicle_type=mav.MAV_TYPE_ONBOARD_CONTROLLER, autopilot=mav.MAV_AUTOPILOT_INVALID, compid=191), at(0))
    decoder.feed(heartbeat(mode=5), at(0.1))
    assert decoder.vehicle_sysid == 1
    [fix] = fixes(decoder.feed(position(), at(0.2)))
    assert fix.mode == 'LOITER'


def test_messages_of_another_vehicle_are_ignored_once_one_is_locked(decoder):
    decoder.feed(heartbeat(), at(0))
    assert decoder.feed(position(sysid=2), at(1)) == [] and decoder.stats['other_vehicle_messages'] == 1
    assert decoder.feed(heartbeat(mode=9, sysid=2), at(1)) == []
    [fix] = fixes(decoder.feed(position(boot_ms=2000), at(2)))
    assert fix.mode == 'AUTO'                                                                 # sysid 2 did not change it


def test_a_configured_system_id_is_used_from_the_start():
    decoder = MavlinkDecoder(MavlinkDecoderConfig(vehicle_sysid=7, time_mode='arrival'))
    assert decoder.feed(position(sysid=1), at(0)) == []
    assert len(fixes(decoder.feed(position(sysid=7), at(1)))) == 1


def test_a_non_copter_or_non_ardupilot_vehicle_gets_no_invented_mode(decoder):
    plane = heartbeat(vehicle_type=mav.MAV_TYPE_FIXED_WING)
    px4 = heartbeat(autopilot=mav.MAV_AUTOPILOT_PX4)
    assert copter_mode_name(plane) is None and copter_mode_name(px4) is None


def test_an_unknown_copter_mode_number_is_reported_as_the_number():
    assert copter_mode_name(heartbeat(mode=99)) == 'MODE 99'


def test_the_mode_table_matches_pymavlinks_own_table():
    theirs = dict(mavutil.mode_mapping_acm)
    assert {number: theirs.get(number) for number in ARDUCOPTER_MODES} == ARDUCOPTER_MODES


def test_battery_percent_rides_on_the_next_fix_and_minus_one_means_unknown(decoder):
    unknown = stamped(mav.MAVLink_sys_status_message(0, 0, 0, 0, 12000, -1, -1, 0, 0, 0, 0, 0, 0))
    known = stamped(mav.MAVLink_sys_status_message(0, 0, 0, 0, 12000, -1, 76, 0, 0, 0, 0, 0, 0))
    decoder.feed(heartbeat(), at(0))
    decoder.feed(unknown, at(0.1))
    assert fixes(decoder.feed(position(), at(0.2)))[0].battery_percent is None
    decoder.feed(known, at(0.3))
    assert fixes(decoder.feed(position(boot_ms=2000), at(0.4)))[0].battery_percent == 76.0


def test_a_heading_from_the_hud_or_attitude_fills_in_when_the_position_message_has_none(decoder):
    decoder.feed(heartbeat(), at(0))
    hud = stamped(mav.MAVLink_vfr_hud_message(10.0, 10.0, 270, 50, 30.0, 0.0))
    decoder.feed(hud, at(0.1))
    [fix] = fixes(decoder.feed(position(hdg=65535), at(0.2)))
    assert fix.heading_deg == pytest.approx(270.0)
    attitude = stamped(mav.MAVLink_attitude_message(0, 0.0, 0.0, 3.14159265 / 2, 0.0, 0.0, 0.0))
    decoder.feed(attitude, at(0.3))
    [again] = fixes(decoder.feed(position(boot_ms=2000, hdg=65535), at(0.4)))
    assert again.heading_deg == pytest.approx(90.0, abs=0.01)


# ------------------------------------------------------------------------------------------ clocks
def system_time(unix_s, boot_ms):
    return stamped(mav.MAVLink_system_time_message(int(unix_s * 1e6), boot_ms))


def test_the_autopilots_own_clock_stamps_the_position_while_it_agrees_with_ours():
    decoder = MavlinkDecoder(MavlinkDecoderConfig(time_mode='auto'))
    decoder.feed(heartbeat(), at(0))
    decoder.feed(system_time(at(0).timestamp() - 0.4, 1000), at(0))
    [fix] = fixes(decoder.feed(position(boot_ms=3000), at(2.5)))                   # measured at autopilot time 3.0 s
    assert fix.time_source == 'autopilot'
    assert abs((fix.fix_time - at(1.6)).total_seconds()) < 0.01                     # the autopilot's own measurement time


def test_an_autopilot_clock_that_is_far_off_is_not_believed_and_the_arrival_time_is_used():
    decoder = MavlinkDecoder(MavlinkDecoderConfig(time_mode='auto', max_clock_skew_s=2.0))
    decoder.feed(heartbeat(), at(0))
    decoder.feed(system_time(at(0).timestamp() - 34.0, 1000), at(0))                # 34 s behind, like the simulator
    [fix] = fixes(decoder.feed(position(boot_ms=3000), at(2.5)))
    assert fix.time_source == 'arrival' and fix.fix_time == at(2.5) and decoder.stats['clock_fallbacks'] >= 1


def test_a_position_is_never_stamped_in_the_future():
    decoder = MavlinkDecoder(MavlinkDecoderConfig(time_mode='auto'))
    decoder.feed(heartbeat(), at(0))
    decoder.feed(system_time(at(0).timestamp() + 600.0, 1000), at(0))
    [fix] = fixes(decoder.feed(position(boot_ms=3000), at(2.5)))
    assert fix.fix_time <= fix.arrival


def test_a_clock_that_means_no_real_time_yet_is_ignored():
    decoder = MavlinkDecoder(MavlinkDecoderConfig(time_mode='auto'))
    decoder.feed(heartbeat(), at(0))
    decoder.feed(stamped(mav.MAVLink_system_time_message(0, 1000)), at(0))
    [fix] = fixes(decoder.feed(position(boot_ms=3000), at(2.5)))
    assert fix.time_source == 'arrival'


def test_an_autopilot_reboot_is_noticed_by_the_clock_going_backwards(decoder):
    decoder.feed(position(boot_ms=600000), at(0))
    decoder.feed(position(boot_ms=2000), at(1))
    assert decoder.stats['reboots'] == 1


# ------------------------------------------------------------------------------------------ the radio texts
def test_a_target_report_on_the_radio_becomes_a_sighting(decoder):
    decoder.feed(heartbeat(), at(0))
    [sighting] = [e for e in decoder.feed(statustext('TGT p 36.214935 -96.006203 95 12'), at(3)) if isinstance(e, TargetSighting)]
    assert (sighting.kind, sighting.confidence, sighting.seq) == ('person', 0.95, 12)
    assert (sighting.lat_deg, sighting.lon_deg) == (pytest.approx(36.214935), pytest.approx(-96.006203))
    assert sighting.measured_time == at(3)                                          # the message has no time of its own


def test_a_tent_report_and_a_report_at_the_wrong_severity(decoder):
    tent = [e for e in decoder.feed(statustext('TGT t 36.215000 -96.007000 80 1'), at(0)) if isinstance(e, TargetSighting)]
    assert tent and tent[0].kind == 'tent'
    assert decoder.feed(statustext('TGT p 36.215000 -96.007000 80 2', severity=2), at(1)) == []       # only INFO is accepted


@pytest.mark.parametrize('text', [
    'hello', 'TGT', 'TGT x 36.215000 -96.006000 80 1',                      # not our message / unknown kind letter
    'TGT p 99.000000 -96.006000 80 1', 'TGT p 36.215000 -196.000000 80 1',  # a latitude / longitude that cannot exist
    'TGT p 36.215000 -96.006000 180 1', 'TGT p 36.215000 -96.006000 80',    # confidence over 100 / counter missing
    'TGT p 36.2 -96.0 80 1', 'TGT p nan -96.006000 80 1', 'tgt p 36.215000 -96.006000 80 1',
    'TGT p 36.215000 -96.006000 80 1 extra', 'TGT p 036.215000 -96.006000 80 1'])
def test_texts_that_are_not_valid_target_reports_make_no_sighting(decoder, text):
    assert [e for e in decoder.feed(statustext(text), at(0)) if isinstance(e, TargetSighting)] == []


def test_the_release_line_becomes_a_drop_at_the_last_known_position(decoder):
    decoder.feed(heartbeat(), at(0))
    decoder.feed(position(lat=36.2150, lon=-96.0060), at(1))
    drops = [e for e in decoder.feed(statustext('Servo 13 -> 1400', severity=6), at(2)) if isinstance(e, DropEvent)]
    assert len(drops) == 1 and drops[0].kind == 'tent' and drops[0].lat_deg == pytest.approx(36.2150)


def test_the_release_line_with_no_known_position_is_still_a_drop_event_without_a_position(decoder):
    drops = [e for e in decoder.feed(statustext('Servo 11 -> 1400'), at(0)) if isinstance(e, DropEvent)]
    assert len(drops) == 1 and drops[0].kind == 'person' and not drops[0].has_position


def test_a_servo_line_for_another_channel_or_a_closing_pulse_is_not_a_release(decoder):
    assert [e for e in decoder.feed(statustext('Servo 5 -> 1400'), at(0)) if isinstance(e, DropEvent)] == []
    assert [e for e in decoder.feed(statustext('Servo 13 -> 1900'), at(0)) if isinstance(e, DropEvent)] == []


# ------------------------------------------------------------------------------------------ junk
@pytest.mark.parametrize('junk', [object(), None, 'text', 42, {'a': 1}])
def test_something_that_is_not_a_message_is_survived_and_counted(decoder, junk):
    assert decoder.feed(junk, at(0)) == [] and decoder.stats['bad_messages'] + decoder.stats['ignored'] >= 1


def test_messages_we_do_not_use_are_ignored_quickly(decoder):
    ping = stamped(mav.MAVLink_attitude_quaternion_message(0, 1, 0, 0, 0, 0, 0, 0))
    assert decoder.feed(ping, at(0)) == [] and decoder.stats['ignored'] == 1


def test_reset_forgets_the_vehicle_but_keeps_a_configured_one():
    free = MavlinkDecoder()
    free.feed(heartbeat(), at(0))
    assert free.vehicle_sysid == 1
    free.reset()
    assert free.vehicle_sysid is None
    fixed = MavlinkDecoder(MavlinkDecoderConfig(vehicle_sysid=1))
    fixed.reset()
    assert fixed.vehicle_sysid == 1


def test_merged_stats_collect_the_counters_of_all_parts(decoder):
    decoder.feed(statustext('TGT p 36.2150 -96.0070 80 1'), at(0))
    stats = decoder.merged_stats()
    assert stats['messages'] == 1 and stats['statustexts'] == 1 and any(k.startswith('text_') for k in stats)
