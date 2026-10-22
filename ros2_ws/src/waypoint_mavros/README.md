# waypoint_mavros: the mission copy (waypoint_manager)

`waypoint_mavros/waypoint.py` runs the ROS node `waypoint_manager`. It

* keeps a copy of the autopilot's mission (MAVROS topic `/mavros/mission/waypoints`),
* publishes the mission's landmarks as its own ROS parameters, which the other nodes read:

  | Parameter | Meaning | Runway example |
  |---|---|---|
  | `num_waypoints` | items in the mission (home = item 0 included) | 12 |
  | `takeoff_index` | the TAKEOFF item | 1 |
  | `next_after_takeoff` | the item after it | 2 |
  | `rtl_index` | the RETURN_TO_LAUNCH item | 11 |
  | `last_before_rtl` | the item before RTL (main_controller inserts targets after it) | 10 |
  | `buffer_wp` | `last_before_rtl - 1`: the photo trigger stops when the drone reaches it | 9 |

  (`-1` / `-2` mean "not known": no mission yet.)
* serves `/addWaypoint`: inserts a waypoint into the copy and uploads the whole mission again
  (main_controller uses it for the target waypoints).

## What changed in October 2026: the "stale mission" bug

**Before:** the node asked MAVROS for the mission only once, when it started. MAVROS does not read the
mission again by itself after a ground station (Mission Planner) UPLOADS one. So if you uploaded the
mission after starting `./launcher.sh`, the team code kept the old (empty) mission: `buffer_wp` stayed
`-2`, the photos never stopped and main_controller never held at the last waypoint.

**Now** it asks MAVROS to read the mission again (`/mavros/mission/pull`, without ever blocking) when

1. ArduPilot says **"Flight plan received"** (it sends this text when an upload finishes),
2. the drone is **armed** (last chance before take-off; also covers a real drone whose ground-station
   radio is not the link MAVROS listens to, so MAVROS never sees that text), and
3. at start-up, once MAVROS is connected.

After each re-read you see in the log `Re-read the mission from the autopilot: 12 items (because ...)`
and the ground station gets `Mission re-read (12 items)`. The indices are then worked out again
from scratch, and only the ones that changed are published (one parameter event instead of six).
The "Flight plan received" that comes back after the node's OWN upload of the target waypoints is
ignored, so a re-read never gets in the way of main_controller's divert.

The rules (when to re-read, how the indices are computed) are in `waypoint_mavros/mission_sync.py`,
pure Python, tested by:

```bash
python3 -m pytest ros2_ws/src/waypoint_mavros/test/test_mission_sync.py -v    # no ROS needed
```
