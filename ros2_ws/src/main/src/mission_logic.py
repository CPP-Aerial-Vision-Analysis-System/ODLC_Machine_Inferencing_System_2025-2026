"""Pure mission-planning helpers, with NO ROS dependencies.

Why this file exists
--------------------
The logic here used to live inside ``MainController`` in
``main_controller.py``. That class subclasses a ROS 2 ``Node``, so you
cannot import it (let alone test it) without a full ROS environment.

By pulling the *pure* logic out into plain functions that take ordinary
Python values (ints, lists) and return ordinary values, we can unit-test
it on any machine — including a CI runner that has no ROS installed.

This is the general lesson: **logic that is decoupled from its framework
is logic you can test.**
"""

# Mission-item command IDs that count as real navigation waypoints.
# (MAVLink: 16=NAV_WAYPOINT, 17-21=NAV_LOITER_*, 22=NAV_TAKEOFF)
NAV_COMMANDS = {16, 17, 18, 19, 20, 21, 22}


def find_last_two_nav_waypoints(commands, rtl_index, nav_commands=NAV_COMMANDS):
    """Find the last two navigation waypoints before the RTL waypoint.

    Scans backwards from just before ``rtl_index`` and returns the indices
    of the last NAV waypoint and the one before it.

    DO_* commands (e.g. DO_SET_SERVO, DigiCamCtrl) do not trigger a
    ``WaypointReached`` event, so to react "at the last real waypoint" we
    need the index of the last *physical* navigation waypoint, plus the
    one before it (the "buffer" waypoint used for the GUIDED processing
    hold).

    Args:
        commands: list of MAVLink command IDs, one per mission item,
            in mission order.
        rtl_index: index of the RETURN_TO_LAUNCH item. Scanning starts at
            ``rtl_index - 1`` and goes downward.
        nav_commands: set of command IDs treated as navigation waypoints.

    Returns:
        (last_nav_before_rtl, buffer_wp) as a tuple of ints. Either value
        is ``-1`` if not found.
    """
    last_nav_before_rtl = -1
    buffer_wp = -1

    if rtl_index > 0 and len(commands) > 0:
        found_last = False
        for i in range(rtl_index - 1, -1, -1):
            if i >= len(commands):
                continue
            if commands[i] in nav_commands:
                if not found_last:
                    last_nav_before_rtl = i
                    found_last = True
                else:
                    buffer_wp = i
                    break

    return last_nav_before_rtl, buffer_wp
