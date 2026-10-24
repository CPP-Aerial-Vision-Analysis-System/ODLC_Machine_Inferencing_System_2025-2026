"""
sim_plugins/contracts.py -- PURPOSE: the written CONTRACT of every slot, as data.

A contract says exactly what a component must do to replace the team's current one in that slot:
which ROS topics it publishes / subscribes (name, message type, QoS), which services it serves or calls,
which files it reads or writes, which network ports it uses, and the behaviour rules in plain words.
Everything here was taken from the team's code (detection/new_od.py, main/src/main_controller.py,
video_cam/siyi_node.py, waypoint_mavros/waypoint.py) and the study reports (understand_odlc/SYNTHESIS.md).

The SAME data is used three ways:
  * check_slot.py checks a running component against it (the machine-checkable rows),
  * `profile_tool contract --slot detector` prints it,
  * docs/simulation/PLUGINS.md shows it as tables (test_contracts.py keeps them in step).

QoS words: "reliable" = the component must use RELIABLE (a BEST_EFFORT publisher never reaches the team's
RELIABLE subscriber); "any" = either works.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

PHOTO_DIR = '<ws>/src/video_cam/mapping_photos'     # <ws> = the ros2_ws folder of the team repo


@dataclass(frozen=True)
class TopicRule:
    """A ROS topic the slot's component publishes or subscribes."""
    name: str
    type: str
    role: str                 # 'publish' | 'subscribe'   (what the COMPONENT does)
    qos: str = 'any'          # 'reliable' | 'best_effort' | 'any'
    required: bool = True
    note: str = ''


@dataclass(frozen=True)
class ServiceRule:
    """A ROS service the component serves ('server') or calls ('client')."""
    name: str
    type: str
    role: str                 # 'server' | 'client'
    required: bool = True
    note: str = ''


@dataclass(frozen=True)
class FileRule:
    """A file or folder the component reads or writes."""
    path: str
    role: str                 # 'reads' | 'writes'
    note: str = ''


@dataclass(frozen=True)
class PortRule:
    """A network port the component listens on or talks to."""
    what: str
    role: str                 # 'listens' | 'talks to'
    note: str = ''


@dataclass(frozen=True)
class SlotContract:
    """Everything a replacement for one slot must do."""
    slot: str
    title: str
    default: str
    alternatives: str
    summary: str
    node_names: Tuple[str, ...] = ()
    topics: Tuple[TopicRule, ...] = ()
    services: Tuple[ServiceRule, ...] = ()
    files: Tuple[FileRule, ...] = ()
    ports: Tuple[PortRule, ...] = ()
    rules: Tuple[str, ...] = ()


DETECTOR = SlotContract(
    slot='detector',
    title='Detector (finds the hiker and the tent in the photos)',
    default='team: detection/new_od (YOLO26m on 640 px tiles), started by main/launch/tracker.launch.xml',
    alternatives='truth (sim_demo truth_detector, the answer key) | custom (your detector: replaces new_od)',
    summary='Reads every new photo that video_cam saves in the team photo folder and publishes ONE '
            'interfaces/ImageResult per photo on /image_detection. main_controller uses those messages to '
            'choose where to drop.',
    node_names=('new_od',),
    topics=(
        TopicRule('/image_detection', 'interfaces/msg/ImageResult', 'publish', 'reliable', True,
                  'one message per photo (RELIABLE, depth 10, like new_od); main_controller subscribes RELIABLE'),
        TopicRule('/sahi_detection_info', 'std_msgs/msg/String', 'publish', 'any', False,
                  'optional JSON per photo: image, timestamp, num_people, num_tents, num_objects, '
                  'total_detections, method'),
        TopicRule('/mavros/mission/reached', 'mavros_msgs/msg/WaypointReached', 'subscribe', 'any', False,
                  'optional: new_od copies the last reached item into ImageResult.waypoint_index'),
        TopicRule('/sim/camera/photo', 'sensor_msgs/msg/Image', 'subscribe', 'any', False,
                  'SIMULATOR ONLY, optional extra input: every photo the camera emulator renders (the photo '
                  'FILE stays the real input: main_controller matches results by file name)'),
    ),
    services=(
        ServiceRule('/sahi/get_health', 'std_srvs/srv/Trigger', 'server', False,
                    'recommended: message "Healthy: True, ConsecErrors: 0, ModelLoaded: True, Format: <name>"; '
                    'stack_check --need-detector waits for ModelLoaded: True'),
        ServiceRule('/sahi/get_statistics', 'std_srvs/srv/Trigger', 'server', False, 'optional'),
    ),
    files=(
        FileRule(f'{PHOTO_DIR}/"<lat> , <lon>.jpg"', 'reads',
                 'the photos (6 decimals, e.g. "36.215026 , -96.006800.jpg"); "IMG_NNNN.jpg" / "sim_*.jpg" when '
                 'there was no GPS fix. Written atomically as "<name>.tmp.jpg" then renamed: ignore *.tmp.*'),
        FileRule(f'{PHOTO_DIR}/"<lat> , <lon>.json"', 'reads',
                 'optional sidecar (format astra.photo_metadata.v1): height above home, heading, gimbal angles, '
                 'zoom, field of view at the shutter. Not an image: skip .json files'),
        FileRule('<ws>/src/video_cam/detection_results_sahi/', 'writes',
                 'optional annotated images (demo.sh copies them into the run folder)'),
    ),
    rules=(
        'Publish exactly ONE ImageResult per photo file, ALSO when nothing was found: at the last survey '
        'waypoint main_controller waits (up to 90 s) until every photo name in the folder was reported.',
        'image_name = the photo\'s file name exactly as in the folder (no folder part).',
        'detections = vision_msgs/Detection2DArray with ONLY persons and tents: results[0].hypothesis.class_id '
        '"0" = person, "1" = tent; .score = confidence 0..1; bbox.center.position.x/.y = box centre and '
        'bbox.size_x/.size_y = box size, in pixels of the FULL-SIZE photo.',
        'latitude/longitude = the GPS in the file name (float32); 0.0 when the name has none.',
        'method = your detector\'s name. It must NOT start with "truth_detector" (the demo panel would show '
        'the answer-key banner).',
        'header.frame_id "camera"; timestamp = ISO wall-clock text; num_detections; classes / confidences / '
        'areas / descriptions = parallel lists over every detection (information only).',
        'Run ONE detector at a time: main_controller cannot tell two detectors apart.',
        'main_controller keeps the highest score per class (strictly higher wins) and aims at that box\'s centre '
        '(using the sidecar); a photo without GPS is ignored.',
    ),
)

MISSION_BRAIN = SlotContract(
    slot='mission_brain',
    title='Mission brain (decides where to drop, flies there, drops, goes home)',
    default='team: main/src/main_controller.py (node main_controller)',
    alternatives='custom (your mission logic: replaces main_controller)',
    summary='ArduPilot flies the uploaded survey by itself (AUTO). The mission brain listens to the detector, '
            'and after the last survey waypoint inserts one waypoint per target, flies there, opens the '
            'release servos and sends the drone home.',
    node_names=('main_controller',),
    topics=(
        TopicRule('/image_detection', 'interfaces/msg/ImageResult', 'subscribe', 'reliable', True,
                  'the detector\'s results (see the detector contract)'),
        TopicRule('/mavros/mission/waypoints', 'mavros_msgs/msg/WaypointList', 'subscribe', 'any', True,
                  'the mission on the autopilot (find the last NAV item before RTL)'),
        TopicRule('/mavros/mission/reached', 'mavros_msgs/msg/WaypointReached', 'subscribe', 'any', True,
                  'which mission item was just reached (wp_seq)'),
        TopicRule('/parameter_events', 'rcl_interfaces/msg/ParameterEvent', 'subscribe', 'any', False,
                  'optional: waypoint_manager publishes the mission indices as its parameters'),
        TopicRule('/mavros/statustext/send', 'mavros_msgs/msg/StatusText', 'publish', 'any', False,
                  'recommended: short texts (max 50 characters) that Mission Planner and the phase monitor show'),
    ),
    services=(
        ServiceRule('/addWaypoint', 'interfaces/srv/AddWaypoint', 'client', True,
                    'waypoint_manager INSERTS one item at `index` and pushes the whole mission '
                    '(command 16 = NAV_WAYPOINT, altitude relative to home)'),
        ServiceRule('/mavros/set_mode', 'mavros_msgs/srv/SetMode', 'client', True, 'custom_mode GUIDED / AUTO / RTL'),
        ServiceRule('/mavros/mission/set_current', 'mavros_msgs/srv/WaypointSetCurrent', 'client', True,
                    'point the mission at the first inserted target'),
        ServiceRule('/mavros/cmd/command', 'mavros_msgs/srv/CommandLong', 'client', True,
                    'MAV_CMD_DO_SET_SERVO (183): param1 = output, param2 = PWM'),
        ServiceRule('/waypoint_manager/get_parameters', 'rcl_interfaces/srv/GetParameters', 'client', False,
                    'optional: num_waypoints, takeoff_index, rtl_index, next_after_takeoff, last_before_rtl'),
    ),
    files=(
        FileRule(f'{PHOTO_DIR}/', 'reads', 'optional: which photos exist (to know when the detector is done)'),
        FileRule(f'{PHOTO_DIR}/"<lat> , <lon>.json"', 'reads',
                 'optional: the photo\'s sidecar, to turn a box centre into latitude/longitude'),
    ),
    rules=(
        'Never arm, take off or upload the survey: the ground station (demo.sh / Mission Planner) does that.',
        'At the last survey waypoint switch to GUIDED (hover) and wait for the detector (main_controller: '
        'until every photo was reported, at most 90 s).',
        'Insert one NAV_WAYPOINT (16) per target right before the mission\'s RETURN_TO_LAUNCH item '
        '(main_controller: index = last_before_rtl + 1, tent first, altitude 49 m), set the current item to '
        'the first target, then AUTO.',
        'Over each target: GUIDED, hold, DO_SET_SERVO output 13 (tent) or 11 (person) to 1400 us (open) for '
        '3 s, then 500 us (closed). After the last target: RTL.',
        'Phase monitor texts (optional): "Reached last WP ...", "All N images processed ..." or "Processing '
        'timeout ...", "Targets: ...", "No valid targets ...", "Over tent|person ...", "Servo 13 -> 1400", "RTL".',
        'Run ONE mission brain at a time (two would fight over the flight mode).',
    ),
)

CAMERA = SlotContract(
    slot='camera',
    title='Camera (the SIYI gimbal camera on the drone\'s network)',
    default='siyi_emulator: pretends to be the SIYI camera at 192.168.144.25 (profiles zr10 = 2K photos, '
            'a8mini = 4K; strict = silent like a real A8 mini on commands it does not support)',
    alternatives='custom (your camera program) | none (no camera)',
    summary='The team\'s video_cam talks to "the camera" exactly like on the drone: SIYI SDK over UDP and the '
            'media web API over HTTP. In the simulator the camera renders its photos in Gazebo and moves the '
            'Gazebo gimbal.',
    node_names=('siyi_emulator',),
    topics=(
        TopicRule('/sim/camera/photo_info', 'std_msgs/msg/String', 'publish', 'reliable', False,
                  'optional (demo tools): JSON record of every photo (RELIABLE + TRANSIENT_LOCAL, depth 10)'),
        TopicRule('/sim/camera/emulator_state', 'std_msgs/msg/String', 'publish', 'any', False,
                  'optional (demo panel): JSON zoom + gimbal, 2 Hz'),
    ),
    ports=(
        PortRule('UDP <camera_ip>:37260', 'listens',
                 'SIYI SDK: frames 55 66 | ctrl | len | seq | cmd | data | CRC16-XMODEM; video_cam sends 0x02 '
                 '(hardware id), 0x0A (system info), 0x0C (photo), 0x0D/0x0E (gimbal attitude / angles), 0x0F '
                 '(zoom), 0x05/0x07/0x08, 0x18 (zoom read-back) and 0x04/0x06 (focus; an A8 mini ignores 0x04 '
                 '0x06 0x16 0x18 0x19)'),
        PortRule('TCP <camera_ip>:82', 'listens',
                 'HTTP media API: GET /cgi-bin/media.cgi/api/v1/getdirectories?media_type=0, '
                 'getmedialist?media_type=0&path=<dir>&start=0&count=<n>, then GET each photo url (JPEG)'),
        PortRule('gz /sim/camera/photo/trigger -> /sim/camera/photo/image', 'talks to',
                 'Gazebo (gz-transport, partition $GZ_PARTITION): one gz.msgs.Boolean = one rendered photo'),
        PortRule('gz /sim/gimbal/{yaw,roll,pitch}/cmd_pos', 'talks to',
                 'gz.msgs.Double, radians; yaw + = left, pitch -pi/2 = straight down; '
                 '/sim/drone/odometry for stabilising'),
    ),
    rules=(
        'Photo size = the camera profile: zr10 2560x1440, a8mini 3840x2160 (video_cam checks the size).',
        'After 0x0C the new photo must appear in getmedialist within 15 s (video_cam polls every 0.5 s).',
        'Send the 0x0B "photo OK" feedback within 0.8 s, or nothing (silence = success for video_cam).',
        'Listen on camera_ip (192.168.144.25 is added to this computer by simulation/setup/camera_ip_alias.sh), '
        'or on camera_bind (0.0.0.0 in containers).',
    ),
)

AUTOPILOT = SlotContract(
    slot='autopilot',
    title='Autopilot (the Pixhawk firmware: ArduCopter)',
    default='ArduCopter SITL 4.6.3 (~/sitl/arducopter) + copter.parm, gazebo-iris.parm, astra_sitl.parm',
    alternatives='another version (fetch_sitl.sh), a source build (/opt/ardupilot), extra parameter files, '
                 'or external (another container / PC)',
    summary='Flies the drone in Gazebo through the ArduPilot plugin\'s JSON physics link and talks MAVLink to '
            'MAVProxy, which copies it to MAVROS (the team stack), our scripts and the demo panel.',
    topics=(
        TopicRule('/mavros/state', 'mavros_msgs/msg/State', 'publish', 'any', True,
                  'published by MAVROS once ./launcher.sh sim runs: connected must become true'),
    ),
    ports=(
        PortRule('UDP 9002+10*I (Gazebo computer)', 'talks to',
                 'JSON physics link: the drone\'s ArduPilotPlugin listens (lock-step); SITL sends servo outputs'),
        PortRule('TCP 5760+10*I', 'listens', 'SITL serial0: MAVProxy connects here (5762/5763+10*I are spare)'),
        PortRule('UDP 127.0.0.1:14550 / 14552 / 14553', 'talks to',
                 'MAVProxy outputs: ground station + our scripts / MAVROS / demo panel (+ Windows ground station)'),
    ),
    rules=(
        'Behave like ArduCopter: modes GUIDED, AUTO and RTL; MISSION_ITEM_REACHED; the text '
        '"Mission: <n> DigiCamCtrl" when the DO_DIGICAM_CONTROL item starts (it starts the photos).',
        'Accept MAV_CMD_DO_SET_SERVO on outputs 11 and 13 (SERVO11/13_FUNCTION 0).',
        'Arm and take off in AUTO without RC sticks (AUTO_OPTIONS 3); no geofence, no GCS failsafe in the sim.',
        'Home = the world\'s home (sim_worlds/config/<world>.yaml), so GPS and the Gazebo picture agree.',
        'Wait ~30 s after boot before arming: "Arm: Gyros inconsistent" until then is normal.',
    ),
)

WORLD = SlotContract(
    slot='world',
    title='World (the field, the targets and the survey mission)',
    default='runway (sim_worlds/config/runway.yaml)',
    alternatives='suas_field | suas_field_real (real terrain; being built) | any .sdf + config .yaml',
    summary='A Gazebo world plus one config file that tells the bringup where the world file, the SITL home, '
            'the spawn pose, the survey mission and the answer key (truth) are.',
    topics=(
        TopicRule('/sim/drone/odometry', 'nav_msgs/msg/Odometry', 'publish', 'any', True,
                  'bridged from Gazebo while the simulator runs (shows the world is loaded with the drone)'),
        TopicRule('/sim/camera/image', 'sensor_msgs/msg/Image', 'publish', 'any', False, 'live camera (bridge)'),
    ),
    files=(
        FileRule('sim_worlds/config/<name>.yaml', 'reads',
                 'world_file, home "lat,lon,alt_msl,heading" (= the SDF\'s <spherical_coordinates>), spawn '
                 '{x,y,z,yaw}, mission (.waypoints), truth (answer key)'),
        FileRule('<world>.sdf', 'reads', 'systems Physics (max_step_size 0.001), UserCommands, SceneBroadcaster, '
                 'Sensors (ogre2), Imu, NavSat; <spherical_coordinates> EARTH_WGS84 ENU'),
        FileRule('missions/<world>_survey.waypoints', 'reads',
                 'QGC WPL 110: TAKEOFF, DO_DIGICAM_CONTROL, survey waypoints, RETURN_TO_LAUNCH last '
                 '(check: ros2 run sim_bringup mission_tool check-mission --mission FILE)'),
    ),
    rules=(
        'Any name with sim_worlds/config/<name>.yaml works as world.name (suas_field_real too, once it exists).',
        'A bare .sdf works as well: SITL home comes from its <spherical_coordinates>; give the mission with '
        'world.mission (demo.sh --mission).',
    ),
)

EXTRAS = SlotContract(
    slot='extras',
    title='Extras (anything else you want running: a logger, a monitor, a test driver)',
    default='nothing',
    alternatives='launch files, single nodes or plain programs, started with the team stack or the simulator',
    summary='Extras have no fixed contract. They should only listen, unless they replace something on purpose.',
    rules=(
        'Do not publish on a topic another slot owns (/image_detection, /camera/trigger, /mavros/...) unless '
        'that is what you are testing.',
        'Subscribe to /mavros topics with BEST_EFFORT QoS: it receives from every publisher.',
        'An extra with start_with: team_stack starts and stops with ./launcher.sh sim; start_with: simulator '
        'starts with sim.launch.py.',
    ),
)

NETWORK = SlotContract(
    slot='network',
    title='Network (ports and addresses; Docker and multi-computer setups)',
    default='everything on 127.0.0.1 inside one computer / container',
    alternatives='MAVProxy outputs to other hosts (mavros_out, gcs_out, extra_outs), the camera emulator on '
                 'another address (camera_bind), an external autopilot (mavlink_master, fdm_addr)',
    summary='Where MAVLink goes and where the camera listens. The defaults are today\'s values.',
    ports=(
        PortRule('MAVProxy --master tcp:127.0.0.1:5760+10*I', 'talks to', 'launch argument mavlink_master'),
        PortRule('MAVProxy --out udp:127.0.0.1:14550', 'talks to', 'gcs_port / gcs_out: our scripts (mission_tool)'),
        PortRule('MAVProxy --out udp:127.0.0.1:14552', 'talks to',
                 'mavros_port / mavros_out: MAVROS (launcher.sh sim fcu_url udp://:14552@localhost:14552)'),
        PortRule('MAVProxy --out udp:127.0.0.1:14553', 'talks to', 'panel_port: demo panel buttons (the panel listens on 127.0.0.1)'),
        PortRule('MAVProxy --out udp:<Windows host>:14550', 'talks to', 'windows_gcs / windows_gcs_addr'),
        PortRule('siyi_emulator UDP 37260 + TCP 82 on 192.168.144.25', 'listens', 'camera_ip / camera_bind'),
        PortRule('Gazebo plugin UDP 9002+10*I on 127.0.0.1', 'listens', 'fdm_addr (0.0.0.0 for an external SITL)'),
    ),
)

CONTRACTS: Dict[str, SlotContract] = {c.slot: c for c in (AUTOPILOT, CAMERA, DETECTOR, MISSION_BRAIN, WORLD,
                                                           EXTRAS, NETWORK)}


def contract(slot: str) -> SlotContract:
    """The contract of one slot (KeyError with the known names for a typo)."""
    if slot not in CONTRACTS:
        raise KeyError(f'unknown slot "{slot}" (known: {", ".join(CONTRACTS)})')
    return CONTRACTS[slot]


def render_text(c: SlotContract) -> str:
    """The contract as plain text (profile_tool contract)."""
    lines: List[str] = [f'SLOT {c.slot}: {c.title}', f'  default:      {c.default}',
                        f'  alternatives: {c.alternatives}', f'  {c.summary}']
    if c.node_names:
        lines.append(f'  node name(s) the stack waits for: {", ".join(c.node_names)}')
    if c.topics:
        lines.append('  ROS topics:')
        for t in c.topics:
            need = 'REQUIRED' if t.required else 'optional'
            lines.append(f'    {t.role:9s} {t.name}  [{t.type}]  QoS {t.qos}  ({need}) {t.note}')
    if c.services:
        lines.append('  ROS services:')
        for s in c.services:
            need = 'REQUIRED' if s.required else 'optional'
            lines.append(f'    {s.role:9s} {s.name}  [{s.type}]  ({need}) {s.note}')
    if c.files:
        lines.append('  files:')
        lines += [f'    {f.role:9s} {f.path}  {f.note}' for f in c.files]
    if c.ports:
        lines.append('  network:')
        lines += [f'    {p.role:9s} {p.what}  {p.note}' for p in c.ports]
    if c.rules:
        lines.append('  rules:')
        lines += [f'    - {r}' for r in c.rules]
    return '\n'.join(lines)


def _md_escape(text: str) -> str:
    return text.replace('|', '\\|')


#: The verb for a topic rule's role in a sentence ("The component ... publishes").
ROLE_VERBS = {'publish': 'publishes', 'subscribe': 'subscribes to'}


def render_markdown(c: SlotContract) -> str:
    """The contract as Markdown tables (used to write docs/simulation/PLUGINS.md)."""
    out: List[str] = [f'**Default:** {c.default}', '', f'**Alternatives:** {c.alternatives}', '', c.summary, '']
    if c.node_names:
        out += [f'Node name the stack waits for: `{"`, `".join(c.node_names)}` (a custom component says its own '
                'with `node_name`).', '']
    if c.topics:
        out += ['| Topic | Type | The component ... | QoS | Required | Notes |', '|---|---|---|---|---|---|']
        out += [f'| `{t.name}` | `{t.type}` | {ROLE_VERBS.get(t.role, t.role)} | {t.qos} | '
                f'{"yes" if t.required else "no"} | {_md_escape(t.note)} |' for t in c.topics]
        out.append('')
    if c.services:
        out += ['| Service | Type | The component is the ... | Required | Notes |', '|---|---|---|---|---|']
        out += [f'| `{s.name}` | `{s.type}` | {s.role} | {"yes" if s.required else "no"} | {_md_escape(s.note)} |'
                for s in c.services]
        out.append('')
    if c.files:
        out += ['| File / folder | The component ... | Notes |', '|---|---|---|']
        out += [f'| `{_md_escape(f.path)}` | {f.role} | {_md_escape(f.note)} |' for f in c.files]
        out.append('')
    if c.ports:
        out += ['| Network | The component ... | Notes |', '|---|---|---|']
        out += [f'| `{_md_escape(p.what)}` | {p.role} | {_md_escape(p.note)} |' for p in c.ports]
        out.append('')
    if c.rules:
        out += ['Rules:', ''] + [f'{i}. {r}' for i, r in enumerate(c.rules, start=1)] + ['']
    return '\n'.join(out)
