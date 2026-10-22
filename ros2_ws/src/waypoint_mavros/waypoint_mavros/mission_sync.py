"""
Rules that keep waypoint_manager's copy of the autopilot mission up to date.

waypoint_mavros/mission_sync.py -- PURPOSE: the pure-Python rules waypoint_manager (waypoint.py)
uses to keep its copy of the autopilot's mission fresh. No ROS imports, so everything here is
unit-tested with plain pytest (test/test_mission_sync.py).

The problem it solves (October 2026, "stale mission"): waypoint_manager asked MAVROS for the
mission only once, at start-up. MAVROS does not re-read the mission by itself after a ground
station UPLOADS a new one, so when the pilot uploaded the mission after starting ./launcher.sh,
waypoint_manager kept the old (often empty) mission: buffer_wp stayed -2, the 1 Hz photos never
stopped and main_controller never held at the last waypoint. Now waypoint_manager asks MAVROS to
read the mission again (service /mavros/mission/pull) whenever:
  * ArduPilot says "Flight plan received" (the STATUSTEXT it sends when a mission upload ends),
  * the drone is ARMED (the last moment before take-off; this also covers a real drone where
    the ground station's radio link is not the link MAVROS listens to, so MAVROS never sees
    that text), or
  * waypoint_manager starts and MAVROS is connected.

Pieces:
  derive_mission_indices(commands)  the mission indices other nodes read as parameters, worked
                                    out FROM SCRATCH for every list (the old code kept stale
                                    values when a new mission had no TAKEOFF / RTL item)
  changed_indices(old, new)         only the indices whose value changed (one parameter event)
  is_new_mission_text(text)         is this autopilot text "a new mission was uploaded"?
  MissionRepullPolicy               WHEN to ask MAVROS for the mission: waits a moment after
                                    the trigger, merges triggers that come close together,
                                    ignores the "Flight plan received" that our OWN target
                                    waypoint upload causes (re-reading during our upload would
                                    make MAVROS reject the next push or set_current), and
                                    retries a failed read a few times.
"""

from typing import Dict, Optional, Sequence

# MAVLink command ids (mavros_msgs/CommandCode) used to find the mission's landmarks.
NAV_RETURN_TO_LAUNCH = 20
NAV_TAKEOFF = 22

# The parameters waypoint_manager publishes, in the order it always declared them.
INDEX_NAMES = ('num_waypoints', 'takeoff_index', 'next_after_takeoff', 'rtl_index',
               'last_before_rtl', 'buffer_wp')

# Policy actions returned by MissionRepullPolicy.next_action().
ACTION_PULL = 'pull'


def derive_mission_indices(commands: Sequence[int]) -> Dict[str, int]:
    """
    Work out the mission indices from the mission's command list.

    Same arithmetic as waypoint_manager has always used, but starting from -1 for every list:
        takeoff_index       index of the (last) NAV_TAKEOFF item, -1 if none
        next_after_takeoff  takeoff_index + 1 if that item exists, else -1
        rtl_index           index of the (last) NAV_RETURN_TO_LAUNCH item, -1 if none
        last_before_rtl     rtl_index - 1 if that is > 0, else -1
        buffer_wp           last_before_rtl - 1 (the photo loop stops there; -2 without RTL)
        num_waypoints       number of items, home (item 0) included

    Parameters
    ----------
    commands
        The MAVLink command id of each mission item, item 0 (home) first.

    Returns
    -------
    dict
        The six INDEX_NAMES and their values.

    """
    n = len(commands)
    takeoff = next_after = rtl = last_before = -1
    for i, command in enumerate(commands):
        if int(command) == NAV_TAKEOFF:
            takeoff = i
            if i + 1 < n:
                next_after = i + 1
        if int(command) == NAV_RETURN_TO_LAUNCH:
            rtl = i
            if i - 1 > 0:
                last_before = i - 1
    return {
        'num_waypoints': n,
        'takeoff_index': takeoff,
        'next_after_takeoff': next_after,
        'rtl_index': rtl,
        'last_before_rtl': last_before,
        'buffer_wp': last_before - 1,
    }


def changed_indices(old: Dict[str, int], new: Dict[str, int]) -> Dict[str, int]:
    """Return the entries of `new` whose value differs from `old` (missing = changed)."""
    return {k: v for k, v in new.items() if old.get(k) != v}


def is_new_mission_text(text: str) -> bool:
    """
    Tell whether an autopilot text means "a new mission was just uploaded".

    ArduPilot (MissionItemProtocol_Waypoints::complete) sends exactly "Flight plan received"
    when a ground station finishes uploading a mission. Compared without case, anywhere in the
    text.
    """
    return 'flight plan received' in (text or '').lower()


class MissionRepullPolicy:
    """
    Decide when waypoint_manager asks MAVROS to read the mission again (pure; no ROS).

    Use: call the on_*() methods when things happen and next_action(now, service_ready) from a
    timer; when it returns ACTION_PULL, send /mavros/mission/pull, call pulled(now), and call
    finished(now, ok) with the result. `now` is any steady clock in seconds.
    """

    def __init__(self, delay_s: float = 2.0, own_push_quiet_s: float = 15.0,
                 retry_delay_s: float = 2.0, max_attempts: int = 3):
        """
        Set up a policy with nothing to do yet.

        Parameters
        ----------
        delay_s
            Wait this long after a trigger before pulling (merges triggers that come close
            together and lets the autopilot finish its side of an upload).
        own_push_quiet_s
            A "Flight plan received" this soon after waypoint_manager's OWN push is that push's
            echo, not a ground-station upload: it is ignored.
        retry_delay_s
            Wait this long before retrying a failed pull.
        max_attempts
            Give up on one trigger after this many failed pulls in a row.

        """
        self.delay_s = float(delay_s)
        self.own_push_quiet_s = float(own_push_quiet_s)
        self.retry_delay_s = float(retry_delay_s)
        self.max_attempts = int(max_attempts)
        self.due_at: Optional[float] = None     # time a pull should be sent (None = none wanted)
        self.reason = ''                        # why the wanted pull was asked for
        self.in_flight = False
        self.pending_reason = ''                # a trigger that arrived while a pull was running
        self.attempts = 0                       # failed attempts for the current trigger
        self.last_own_push: Optional[float] = None
        self.pulls_ok = 0

    # ---------------------------------------------------------------- triggers
    def request(self, now: float, reason: str, delay_s: Optional[float] = None) -> None:
        """Ask for a pull `delay_s` from now (default delay_s), merged with a wanted one."""
        delay = self.delay_s if delay_s is None else float(delay_s)
        if self.in_flight:
            self.pending_reason = self.pending_reason or reason
            return
        if self.due_at is None:
            self.due_at = now + delay
            self.reason = reason
            self.attempts = 0
        elif reason not in self.reason:
            self.reason = f'{self.reason}; {reason}' if self.reason else reason

    def on_new_mission_text(self, now: float) -> bool:
        """Handle "Flight plan received"; return False if it was ignored (our own push)."""
        if self.last_own_push is not None and now - self.last_own_push < self.own_push_quiet_s:
            return False
        self.request(now, 'a new mission was uploaded ("Flight plan received")')
        return True

    def on_armed(self, now: float) -> None:
        """Handle the arming of the drone: read the mission once more before take-off."""
        self.request(now, 'the drone was armed', delay_s=0.5)

    def on_own_push(self, now: float) -> None:
        """Note that waypoint_manager itself is uploading the mission (target waypoints)."""
        self.last_own_push = now

    # ---------------------------------------------------------------- driving it
    def next_action(self, now: float, service_ready: bool):
        """Return ACTION_PULL when a pull should be sent now, else None."""
        if self.in_flight or self.due_at is None or now < self.due_at:
            return None
        if self.last_own_push is not None and now - self.last_own_push < self.own_push_quiet_s:
            # Never read the mission while our own upload may still be running: MAVROS refuses
            # a push or set_current during a pull (and the other way round).
            self.due_at = self.last_own_push + self.own_push_quiet_s
            return None
        if not service_ready:
            self.due_at = now + self.retry_delay_s
            return None
        return ACTION_PULL

    def pulled(self, now: float) -> None:
        """Record that the pull request was just sent."""
        self.in_flight = True
        self.due_at = None

    def finished(self, now: float, ok: bool) -> str:
        """
        Record the pull's result.

        Parameters
        ----------
        now
            Current time.
        ok
            True if MAVROS read the mission.

        Returns
        -------
        str
            'ok', 'retry' (another attempt is scheduled) or 'gave_up'.

        """
        self.in_flight = False
        outcome = 'ok'
        if ok:
            self.attempts = 0
            self.pulls_ok += 1
        else:
            self.attempts += 1
            if self.attempts < self.max_attempts:
                self.due_at = now + self.retry_delay_s
                outcome = 'retry'
            else:
                self.attempts = 0
                outcome = 'gave_up'
        if self.pending_reason and self.due_at is None:
            reason, self.pending_reason = self.pending_reason, ''
            self.request(now, reason)
        return outcome
