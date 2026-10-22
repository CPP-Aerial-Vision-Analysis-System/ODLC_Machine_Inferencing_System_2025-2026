"""
sim_demo/test/test_mission_control.py -- PURPOSE: tests of the panel's "UPLOAD MISSION + START"
and "RTL" worker (mission_control.py):
  * the MAVROS fallback path with a fake MAVROS adapter (arming refused twice = the SITL
    settling window, then accepted);
  * the MAVLink path against a small FAKE AUTOPILOT speaking real MAVLink over UDP on
    localhost (heartbeats, mission upload protocol, COMMAND_ACKs, "PreArm" texts). Skipped if
    pymavlink is not installed.
"""
import socket
import threading
import time

import pytest

from sim_demo.mission_control import MissionCommander
from sim_demo.mission_file import load_waypoints_file


def _wait(cond, timeout=20.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


class FakeMavros:
    """Records calls; refuses to arm twice, like SITL during its settling window."""

    def __init__(self):
        self.calls = []
        self.arm_attempts = 0

    def available(self):
        return True

    def push_mission(self, items):
        self.calls.append(('push', len(items)))
        return True

    def pull(self):
        self.calls.append(('pull',))
        return True

    def set_mode(self, name):
        self.calls.append(('mode', name))
        return True

    def arm(self):
        self.arm_attempts += 1
        self.calls.append(('arm', self.arm_attempts))
        return (self.arm_attempts >= 3, '' if self.arm_attempts >= 3 else 'Arm: Gyros inconsistent')

    def command_long(self, cmd, params):
        self.calls.append(('cmd', cmd))
        return True


def test_mavros_path(world):
    items = load_waypoints_file(world.mission_path)
    log = []
    fake = FakeMavros()
    mc = MissionCommander('', lambda t, lvl: log.append((lvl, t)), fake, link='mavros')
    mc.retry_interval_s = 0.05
    mc.start()
    before = []
    assert mc.request_start(items, settle_timeout_s=10.0, before_arm=lambda: before.append(1))
    assert not mc.request_start(items)                  # second press while busy is refused
    assert _wait(lambda: not mc.busy)
    mc.stop()
    assert fake.calls[0] == ('push', 10) and ('pull',) in fake.calls
    assert ('mode', 'GUIDED') in fake.calls and fake.arm_attempts == 3
    assert fake.calls[-1] == ('cmd', 300)               # MAV_CMD_MISSION_START
    assert before == [1]
    assert any('Mission started' in t for _, t in log)
    assert any('Gyros' in t for _, t in log)            # the settling reason is shown


def test_nothing_reachable_reports_error(world):
    items = load_waypoints_file(world.mission_path)
    log = []

    class Down(FakeMavros):
        def available(self):
            return False

    mc = MissionCommander('', lambda t, lvl: log.append((lvl, t)), Down(), link='mavros')
    mc.start()
    mc.request_start(items)
    assert _wait(lambda: not mc.busy)
    mc.stop()
    assert log[-1][0] == 'error'


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeAutopilot(threading.Thread):
    """Speaks just enough ArduPilot MAVLink: heartbeat, mission upload, commands."""

    def __init__(self, port):
        super().__init__(daemon=True)
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=1,
                                            source_component=1)
        self.items = {}
        self.count = 0
        self.commands = []
        self.arm_refusals = 2
        self.stop_flag = False

    def run(self):
        m, mav = self.m, self.m.mav
        last_hb = 0.0
        while not self.stop_flag:
            if time.time() - last_hb > 0.2:
                mav.heartbeat_send(2, 3, 0, 4, 3)       # quadrotor, ArduPilot
                last_hb = time.time()
            msg = m.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'MISSION_COUNT':
                self.count = msg.count
                mav.mission_request_int_send(msg.get_srcSystem(), msg.get_srcComponent(), 0, 0)
            elif t == 'MISSION_ITEM_INT':
                self.items[msg.seq] = msg
                nxt = msg.seq + 1
                if nxt < self.count:
                    mav.mission_request_int_send(msg.get_srcSystem(), msg.get_srcComponent(), nxt, 0)
                else:
                    mav.mission_ack_send(msg.get_srcSystem(), msg.get_srcComponent(), 0, 0)
            elif t == 'COMMAND_LONG':
                self.commands.append((msg.command, msg.param1, msg.param2))
                result = 0
                if msg.command == 400 and self.arm_refusals > 0:
                    self.arm_refusals -= 1
                    mav.statustext_send(4, b'PreArm: Gyros inconsistent')
                    result = 4                           # MAV_RESULT_FAILED
                mav.command_ack_send(msg.command, result)


def test_mavlink_path_against_fake_autopilot(world):
    pytest.importorskip('pymavlink')
    items = load_waypoints_file(world.mission_path)
    port = _free_udp_port()
    log = []
    mc = MissionCommander(f'udpin:127.0.0.1:{port}', lambda t, lvl: log.append((lvl, t)), None,
                          link='mavlink')
    mc.retry_interval_s = 0.1
    mc.start()
    ap = FakeAutopilot(port)
    ap.start()
    try:
        assert mc.request_start(items, settle_timeout_s=20.0)
        assert _wait(lambda: not mc.busy, 40.0), log
        assert len(ap.items) == len(items)
        it3 = ap.items[3]
        assert it3.command == 16 and it3.x == round(items[3].lat * 1e7)
        cmds = [c for c, _, _ in ap.commands]
        assert cmds.count(400) == 3                     # two refusals, then armed
        assert (176, 1.0, 4.0) in ap.commands           # DO_SET_MODE GUIDED (custom mode 4)
        assert cmds[-1] == 300                          # MISSION_START
        assert any('Gyros' in t for _, t in log)
        mc.request_rtl()
        assert _wait(lambda: (176, 1.0, 6.0) in ap.commands, 10.0)   # RTL = custom mode 6
    finally:
        ap.stop_flag = True
        mc.stop()
