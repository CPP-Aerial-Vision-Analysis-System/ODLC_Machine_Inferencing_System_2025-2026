"""
sim_demo/mission_control.py -- PURPOSE: what the panel's "UPLOAD MISSION + START" and "RTL"
buttons do, on a background thread so the window never freezes.

Start sequence (the job a ground-station operator does with Mission Planner last year):
  1. connect to the autopilot (pymavlink, default udpin:127.0.0.1:14553 = an extra MAVProxy
     output the bringup adds: --out udp:127.0.0.1:14553);
  2. upload the world's .waypoints mission (MAVLink mission protocol, MISSION_ITEM_INT);
  3. ask MAVROS to re-read the mission (/mavros/mission/pull) so the team's waypoint_manager
     and main_controller learn the new mission indices;
  4. switch to GUIDED and wait out the SITL settling window: retry arming until ArduPilot
     accepts ("Arm: Gyros inconsistent" for the first ~30 s after boot is normal);
  5. MAV_CMD_MISSION_START: ArduCopter switches to AUTO and flies the mission (takeoff first).
If nothing answers on the MAVLink port, the same steps run through MAVROS services instead
(/mavros/mission/push, /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/command).

RTL: set mode RTL (MAVLink if connected, else MAVROS /mavros/set_mode).
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Callable

from .mission_file import MAV_CMD_MISSION_START, MissionItem

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176
PREARM_BIT = 0x10000000          # MAV_SYS_STATUS_PREARM_CHECK in SYS_STATUS health flags
COPTER_MODES = {'STABILIZE': 0, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5, 'RTL': 6,
                'LAND': 9, 'BRAKE': 17}


class MissionCommander:
    """Background worker for mission upload/start and RTL (thread-safe request methods).

    Args:
        mavlink_url: pymavlink connection string ('' disables the MAVLink path).
        status: callback(text, level) for progress ('info', 'ok', 'warn', 'error').
        mavros: optional adapter with push_mission(items), pull(), set_mode(name), arm(),
            command_long(cmd, params), available() -> bool  (see demo_panel.MavrosAdapter).
        link: 'auto' (MAVLink, else MAVROS), 'mavlink' or 'mavros'.
    """

    def __init__(self, mavlink_url: str, status: Callable[[str, str], None], mavros=None,
                 link: str = 'auto', source_system: int = 252, source_component: int = 191):
        self.url = mavlink_url
        self.status = status
        self.mavros = mavros
        self.link = link
        self.source_system = source_system
        self.source_component = source_component
        self.master = None
        self.busy = False
        self.last_reason = ''
        self.retry_interval_s = 3.0      # pause between arming attempts [s]
        self.connect_timeout_s = 8.0     # wait this long for a heartbeat in 'auto' mode [s]
        # One MAVLink exchange at a time (the RTL thread and the start thread share the link,
        # and each waits for its own COMMAND_ACK).
        self._ml_lock = threading.RLock()
        self._q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='mission_commander', daemon=True)

    # ------------------------------------------------------------------ public
    def start(self) -> None:
        """Start the worker thread."""
        self._thread.start()

    def stop(self) -> None:
        """Stop the worker thread (pending requests are dropped)."""
        self._stop.set()
        self._q.put(('quit',))
        try:
            if self.master is not None:
                self.master.close()
        except Exception:  # noqa: BLE001
            pass

    def request_start(self, items: list[MissionItem], settle_timeout_s: float = 150.0,
                      before_arm: Callable[[], None] | None = None) -> bool:
        """Queue the start sequence. Returns False if a sequence is already running."""
        if self.busy:
            return False
        self.busy = True
        self._q.put(('start', items, settle_timeout_s, before_arm))
        return True

    def request_rtl(self) -> None:
        """Queue an RTL command (runs even while a start sequence is waiting)."""
        threading.Thread(target=self._rtl, name='rtl', daemon=True).start()

    # ------------------------------------------------------------------ worker
    def _run(self) -> None:
        while not self._stop.is_set():
            job = self._q.get()
            if job[0] == 'quit':
                return
            if job[0] == 'start':
                try:
                    self._start_sequence(*job[1:])
                except Exception as exc:  # noqa: BLE001 - report, never kill the thread
                    self.status(f'Start sequence failed: {exc}', 'error')
                finally:
                    self.busy = False

    def _use_mavlink(self) -> bool:
        if self.link == 'mavros' or not self.url:
            return False
        if self.master is not None:
            return True
        self.status(f'Connecting to the autopilot on {self.url} ...', 'info')
        self.master = self._connect(self.connect_timeout_s if self.link == 'auto' else 30.0)
        if self.master is None and self.link == 'auto':
            self.status('No MAVLink heartbeat on that port - using MAVROS services instead.', 'warn')
        return self.master is not None

    def _connect(self, timeout: float):
        try:
            from pymavlink import mavutil
        except ImportError:
            self.status('pymavlink is not installed - using MAVROS services.', 'warn')
            return None
        try:
            m = mavutil.mavlink_connection(self.url, source_system=self.source_system,
                                           source_component=self.source_component,
                                           autoreconnect=True)
            hb = m.wait_heartbeat(timeout=timeout)
            if hb is None:
                m.close()
                return None
            self.status(f'Autopilot found (system {m.target_system}).', 'ok')
            return m
        except Exception as exc:  # noqa: BLE001
            self.status(f'MAVLink connection failed: {exc}', 'warn')
            return None

    def _start_sequence(self, items, settle_timeout_s, before_arm) -> None:
        if not items:
            self.status('No mission to upload (mission file missing?).', 'error')
            return
        use_ml = self._use_mavlink()
        if not use_ml and (self.mavros is None or not self.mavros.available()):
            self.status('Neither MAVLink nor MAVROS is reachable - is the simulator running?',
                        'error')
            return
        self.status(f'Uploading the mission ({len(items)} items) ...', 'info')
        ok = self._upload_mavlink(items) if use_ml else self.mavros.push_mission(items)
        if not ok:
            self.status('Mission upload FAILED.', 'error')
            return
        self.status('Mission uploaded.', 'ok')
        if self.mavros is not None and self.mavros.available():
            self.mavros.pull()     # let MAVROS + waypoint_manager + main_controller see it
        if before_arm is not None:
            before_arm()
        self._set_mode('GUIDED', use_ml)
        self.status('Waiting for the autopilot to be ready to arm (settling window, ~30 s '
                    'after the simulator starts) ...', 'info')
        if not self._arm_with_retries(settle_timeout_s, use_ml):
            self.status(f'Could not arm within {settle_timeout_s:.0f} s. Last reason: '
                        f'{self.last_reason or "unknown"}', 'error')
            return
        self.status('ARMED. Starting the mission (AUTO) ...', 'ok')
        started = (self._command_long(MAV_CMD_MISSION_START, [0, 0, 0, 0, 0, 0, 0]) if use_ml
                   else self.mavros.command_long(MAV_CMD_MISSION_START, [0] * 7))
        if not started:
            self._set_mode('AUTO', use_ml)
        self.status('Mission started: taking off.', 'ok')

    # ------------------------------------------------------------------ MAVLink helpers
    def _upload_mavlink(self, items: list[MissionItem]) -> bool:
        with self._ml_lock:
            return self._upload_mavlink_locked(items)

    def _upload_mavlink_locked(self, items: list[MissionItem]) -> bool:
        m = self.master
        ts, tc = m.target_system, m.target_component or 1
        m.mav.mission_count_send(ts, tc, len(items), 0)
        sent = set()
        deadline = time.time() + 30.0
        while time.time() < deadline:
            msg = m.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                               blocking=True, timeout=3.0)
            if msg is None:
                if not sent:
                    m.mav.mission_count_send(ts, tc, len(items), 0)   # first request lost: retry
                continue
            if msg.get_type() == 'MISSION_ACK':
                if msg.type == 0:
                    return len(sent) >= len(items) or len(items) == 0
                self.status(f'Autopilot rejected the mission (MAV_MISSION_RESULT {msg.type}).',
                            'error')
                return False
            seq = int(msg.seq)
            if 0 <= seq < len(items):
                it = items[seq]
                m.mav.mission_item_int_send(
                    ts, tc, seq, it.frame, it.command, 1 if seq == 0 else 0, it.autocontinue,
                    it.param1, it.param2, it.param3, it.param4,
                    int(round(it.lat * 1e7)), int(round(it.lon * 1e7)), float(it.alt), 0)
                sent.add(seq)
        return False

    def _command_long(self, cmd: int, params: list[float], timeout: float = 3.0) -> bool:
        """Send COMMAND_LONG and wait for its COMMAND_ACK. True if ACCEPTED (result 0)."""
        with self._ml_lock:
            m = self.master
            p = (list(params) + [0.0] * 7)[:7]
            m.mav.command_long_send(m.target_system, m.target_component or 1, cmd, 0, *p)
            end = time.time() + timeout
            while time.time() < end:
                msg = m.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=0.5)
                if msg is None:
                    continue
                if msg.get_type() == 'STATUSTEXT':
                    self._note_reason(msg.text)
                    continue
                if int(msg.command) == cmd:
                    return int(msg.result) == 0
            return False

    def _note_reason(self, text: str) -> None:
        t = str(text)
        if t.startswith(('PreArm', 'Arm', 'Prearm')):
            self.last_reason = t

    def _set_mode(self, name: str, use_ml: bool) -> bool:
        if not use_ml:
            return bool(self.mavros and self.mavros.set_mode(name))
        mode_id = COPTER_MODES.get(name.upper())
        if mode_id is None:
            return False
        return self._command_long(MAV_CMD_DO_SET_MODE, [1, mode_id, 0, 0, 0, 0, 0])

    def _arm_with_retries(self, timeout: float, use_ml: bool) -> bool:
        end = time.time() + timeout
        tries = 0
        while time.time() < end and not self._stop.is_set():
            tries += 1
            if use_ml:
                ok = self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0], 3.0)
            else:
                ok, reason = self.mavros.arm()
                if reason:
                    self.last_reason = reason
            if ok:
                return True
            if tries % 3 == 1:
                left = max(0, int(end - time.time()))
                why = f' ({self.last_reason})' if self.last_reason else ''
                self.status(f'Not ready to arm yet{why}; retrying ({left} s left) ...', 'info')
            time.sleep(self.retry_interval_s)
        return False

    def _rtl(self) -> None:
        try:
            if self.master is not None:
                ok = self._set_mode('RTL', True)
            elif self.mavros is not None and self.mavros.available():
                ok = self._set_mode('RTL', False)
            elif self._use_mavlink():
                ok = self._set_mode('RTL', True)
            else:
                ok = False
            self.status('RTL commanded: flying home.' if ok else 'RTL command FAILED.',
                        'ok' if ok else 'error')
        except Exception as exc:  # noqa: BLE001
            self.status(f'RTL failed: {exc}', 'error')
