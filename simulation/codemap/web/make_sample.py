"""
simulation/codemap/web/make_sample.py -- PURPOSE: make the two SAMPLE files that the explorer page (index.html)
is tested with, so the page can be tried and tested without running the full extractor or the simulator:

  * sample_codemap.json  - a small code map that follows the codemap.json contract (schema 1): 3 systems
    (team flight code, simulator, ground station), the 5 outside programs, 7 packages, 14 files, about 120
    classes / functions / methods, 9 ROS topics and labelled links (calls, imports, publishes, subscribes,
    launches, runs, mavlink, http, data). Names, first lines (decorators included, like Python's
    co_firstlineno), end lines, docstrings, signatures and source code are READ FROM THE REAL FILES of this
    repo. Call links are found in the source (self.x(...), module.x(...), x(...)); a few extra links are kept
    only when a proof string is found in the caller's source. Plain-language explanations come from
    simulation/codemap/explanations/*.json when an explainer already wrote one for that id; the few top-level
    texts below were written for this sample. The map says "sample": true, so the page shows SAMPLE DATA.
  * sample_recording.jsonl - about 72 seconds of SYNTHETIC events in the recording format of server.py
    ({"event":"calls","t":..,"calls":{fn id: count}} and {"event":"procs","t":..,"running":[ids]}), shaped like
    the start of a search flight (start-up, 1 photo per second, detections, hold, divert, one drop). It was
    NOT recorded from a real run: the first line says so ({"event":"meta","note":...}).

Run (Windows or Linux, Python 3.10+, standard library only):
    python simulation/codemap/web/make_sample.py          (writes both files next to this script)
    python simulation/codemap/web/make_sample.py --check  (only report what would be written)
Then test the page: node simulation/codemap/web/run_selftest.js
"""

import argparse
import ast
import datetime
import glob
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))
EXPLANATIONS = os.path.join(HERE, os.pardir, 'explanations')

# ---------------------------------------------------------------------------------------------------------
# The top level: systems, outside programs, packages (texts written for this sample)
# ---------------------------------------------------------------------------------------------------------
SYSTEMS = [
    ('sys:team_stack', 'Team flight code',
     'The team\'s ROS 2 programs that fly on the drone\'s Jetson computer: they take a photo every second, '
     'find the hiker (mannequin) and the tent in the photos, and tell the autopilot where to drop the payloads. '
     'In the simulator the very same code runs unchanged (./launcher.sh sim).'),
    ('sys:simulator', 'Simulator (Gazebo digital twin)',
     'Everything that pretends to be the real world: Gazebo draws the field and the drone, ArduCopter SITL is '
     'the autopilot, MAVProxy passes the MAVLink messages on, and the SIYI camera emulator answers like the '
     'real camera at 192.168.144.25. Started by simulation/scripts/demo.sh.'),
    ('sys:ground_station', 'Ground station',
     'Mission Planner on the Windows laptop: the pilot\'s screen. The ASTRA plugin adds the drone camera, the '
     'mission phase and the target cards, using only what arrives over MAVLink.'),
]
EXTERNALS = [
    ('ext:gazebo', 'Gazebo Harmonic',
     'The 3D world simulator (not our code). It draws the field, the hiker, the tent and the drone, and moves '
     'the drone the way real physics would.'),
    ('ext:ardupilot_sitl', 'ArduCopter SITL 4.6.3',
     'The real Pixhawk autopilot firmware, compiled to run on the laptop ("software in the loop"). It flies '
     'the mission exactly like the real drone would.'),
    ('ext:mavproxy', 'MAVProxy',
     'A small MAVLink relay (not our code). It takes the autopilot\'s messages from TCP 5760 and copies them '
     'to MAVROS (UDP 14552), the scripts (UDP 14550) and Mission Planner on Windows.'),
    ('ext:mavros', 'MAVROS',
     'The bridge between MAVLink (the autopilot\'s language) and ROS 2 topics and services (not our code).'),
    ('ext:mission_planner', 'Mission Planner',
     'The ground-station program on Windows (not our code). The pilot uploads the mission, arms and starts '
     'AUTO here; the ASTRA plugin runs inside it.'),
]
PACKAGES = [
    ('pkg:ros2_ws/src/main', 'sys:team_stack', 'main'),
    ('pkg:ros2_ws/src/mapping', 'sys:team_stack', 'mapping'),
    ('pkg:ros2_ws/src/video_cam', 'sys:team_stack', 'video_cam'),
    ('pkg:ros2_ws/src/simulation/siyi_emulator', 'sys:simulator', 'siyi_emulator'),
    ('pkg:ros2_ws/src/simulation/sim_bringup', 'sys:simulator', 'sim_bringup'),
    ('pkg:simulation/scripts', 'sys:simulator', 'simulation/scripts'),
    ('pkg:simulation/windows/mission_planner_plugin', 'sys:ground_station', 'mission_planner_plugin'),
]
TOPICS = {
    '/image_detection': 'One message per photo from the detector: what it found (person "0" or tent "1"), how sure it is, where in the photo, and the GPS of the photo.',
    '/mavros/mission/reached': 'MAVROS tells everyone which mission item the autopilot just reached.',
    '/mavros/mission/waypoints': 'The mission as the autopilot has it, sent by MAVROS.',
    '/mavros/statustext/recv': 'Text messages from the autopilot, such as "Mission: 3 DigiCamCtrl".',
    '/mavros/statustext/send': 'Text messages from the team code to the ground station (shown in Mission Planner).',
    '/mavros/global_position/global': 'The drone\'s GPS position (about once a second).',
    '/parameter_events': 'ROS 2 announces every changed parameter here; the team code watches the mission landmarks.',
    '/camera/trigger': '"Take one photo now" (sent once a second during the survey).',
    '/camera/command': 'Lens and gimbal commands for the camera driver, such as capture_setup or autofocus.',
}

# Files and which of their classes / functions go into the sample ("*" = every one; "*.name" = that method
# name in any class). Parents of chosen methods are added automatically.
MODULES = [
    ('ros2_ws/src/main/src/main_controller.py', 'pkg:ros2_ws/src/main', 'main_controller.py', [
        'MainController.__init__', 'MainController._number_parameter', 'MainController.fetch_mission_indices',
        'MainController._on_mission_indices', 'MainController.parameter_event_cb', 'MainController.update_waypoint_reached',
        'MainController._divert_to_targets', 'MainController._loiter_and_drop', 'MainController._loiter_and_drop.after_close',
        'MainController._loiter_and_drop.go_home', 'MainController.valid_detection', 'MainController.waypoints_cb',
        'MainController._update_last_nav_before_rtl', 'MainController._check_all_images_processed',
        'MainController._end_processing_hold', 'MainController.locate_detection', 'MainController._latlon_from_name',
        'MainController.image_result_cb', 'MainController.change_mode', 'MainController._on_mode_result',
        'MainController.send_waypoint_data', 'MainController.set_current_wp', 'MainController.divert_and_resume',
        'MainController.divert_and_resume.resume', 'MainController.open_servos', 'MainController.close_servos',
        'MainController.actuate_servos', 'MainController.move_servo', 'MainController._on_servo_result',
        'MainController.send_ack', 'Detection_Object', 'Detection_Object.__init__']),
    ('ros2_ws/src/main/src/target_geolocation.py', 'pkg:ros2_ws/src/main', 'target_geolocation.py', ['*']),
    ('ros2_ws/src/main/launch/tracker.launch.xml', 'pkg:ros2_ws/src/main', 'tracker.launch.xml', []),
    ('ros2_ws/src/main/test/test_target_geolocation.py', 'pkg:ros2_ws/src/main', 'test_target_geolocation.py', ['first:4']),
    ('ros2_ws/src/mapping/mapping/do_digi_cam_trigger.py', 'pkg:ros2_ws/src/mapping', 'do_digi_cam_trigger.py', ['*']),
    ('ros2_ws/src/video_cam/video_cam/siyi_node.py', 'pkg:ros2_ws/src/video_cam', 'siyi_node.py', [
        'SIYINode.__init__', 'SIYINode._apply_camera_model', 'SIYINode._apply_capture_zoom', 'SIYINode._service_capture_queue',
        'SIYINode._query_gimbal_attitude', 'SIYINode._capture_metadata', 'SIYINode._capture_metadata.age',
        'SIYINode._execute_real_camera_capture', 'SIYINode.camera_trigger_callback', 'SIYINode.camera_command_callback',
        'SIYINode.gps_cb']),
    ('ros2_ws/src/video_cam/video_cam/pipeline_orchestrator.py', 'pkg:ros2_ws/src/video_cam', 'pipeline_orchestrator.py', ['*']),
    ('ros2_ws/src/video_cam/video_cam/camera_interface.py', 'pkg:ros2_ws/src/video_cam', 'camera_interface.py', [
        '*.send_capture_command', '*.absolute_zoom_autofocus', '*.get_current_zoom_magnification', '*.detect_model',
        '*.get_hardware_id', '*.request_gimbal_attitude', '*.get_media_list', '*.get_new_file', '*.download_image',
        '*.decode_image', '*.initialize_sd_card']),
    ('ros2_ws/src/simulation/siyi_emulator/siyi_emulator/command_handler.py', 'pkg:ros2_ws/src/simulation/siyi_emulator', 'command_handler.py', [
        'HandlerHooks', 'HandlerHooks.take_photo', 'CommandHandler.__init__', 'CommandHandler.handle', 'CommandHandler._heartbeat',
        'CommandHandler._hardware_id', 'CommandHandler._capture', 'CommandHandler._attitude', 'CommandHandler._set_angles',
        'CommandHandler._absolute_zoom', 'CommandHandler._current_zoom']),
    ('ros2_ws/src/simulation/siyi_emulator/siyi_emulator/node.py', 'pkg:ros2_ws/src/simulation/siyi_emulator', 'node.py', ['main']),
    ('ros2_ws/src/simulation/siyi_emulator/siyi_emulator/media_server.py', 'pkg:ros2_ws/src/simulation/siyi_emulator', 'media_server.py', []),
    ('ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'pkg:ros2_ws/src/simulation/sim_bringup', 'sim.launch.py', [
        'share_or_none', '_running_in_wsl', '_launch_setup', '_launch_setup.add_mavproxy', 'generate_launch_description']),
    ('simulation/scripts/demo.sh', 'pkg:simulation/scripts', 'demo.sh', ['port_busy', 'problem', 'stop_group', 'finish', 'on_interrupt']),
    ('simulation/windows/mission_planner_plugin/astra_gcs.cs', 'pkg:simulation/windows/mission_planner_plugin', 'astra_gcs.cs', [
        'AstraPlugin.Init', 'AstraPlugin.Loaded', 'AstraPlugin.Loop', 'AstraPlugin.Exit', 'AstraPlugin.OnStatusText',
        'AstraPlugin.Tick', 'AstraPlugin.HandleText']),
]
ENTRY_POINTS = {
    'ros2_ws/src/main/src/main_controller.py': 'ros2 run main main_controller.py',
    'ros2_ws/src/mapping/mapping/do_digi_cam_trigger.py': 'ros2 run mapping trigger',
    'ros2_ws/src/video_cam/video_cam/siyi_node.py': 'ros2 run video_cam siyi',
    'ros2_ws/src/simulation/siyi_emulator/siyi_emulator/node.py': 'ros2 run siyi_emulator siyi_emulator',
    'simulation/scripts/demo.sh': 'bash simulation/scripts/demo.sh runway',
}
PY_MODULE_NAMES = {   # Python's __name__ when the file is imported (trace_index.py_modules)
    'ros2_ws/src/main/src/target_geolocation.py': 'target_geolocation',
    'ros2_ws/src/mapping/mapping/do_digi_cam_trigger.py': 'mapping.do_digi_cam_trigger',
    'ros2_ws/src/video_cam/video_cam/siyi_node.py': 'video_cam.siyi_node',
    'ros2_ws/src/video_cam/video_cam/pipeline_orchestrator.py': 'video_cam.pipeline_orchestrator',
    'ros2_ws/src/video_cam/video_cam/camera_interface.py': 'video_cam.camera_interface',
    'ros2_ws/src/simulation/siyi_emulator/siyi_emulator/command_handler.py': 'siyi_emulator.command_handler',
    'ros2_ws/src/simulation/siyi_emulator/siyi_emulator/node.py': 'siyi_emulator.node',
    'ros2_ws/src/simulation/siyi_emulator/siyi_emulator/media_server.py': 'siyi_emulator.media_server',
}

M = 'fn:ros2_ws/src/main/src/main_controller.py::MainController.'
TRG = 'fn:ros2_ws/src/mapping/mapping/do_digi_cam_trigger.py::MissionCameraTrigger.'
SN = 'fn:ros2_ws/src/video_cam/video_cam/siyi_node.py::SIYINode.'
PO = 'fn:ros2_ws/src/video_cam/video_cam/pipeline_orchestrator.py::PipelineOrchestrator.'
CH = 'fn:ros2_ws/src/simulation/siyi_emulator/siyi_emulator/command_handler.py::'
CAM_MOD = 'ros2_ws/src/video_cam/video_cam/camera_interface.py'

# (caller, callee, label, proof): kept only when `proof` is found in the caller's source code.
EXTRA_CALLS = [
    (SN + '_execute_real_camera_capture', PO + 'capture_and_index', '', 'self.pipeline.capture_and_index('),
    (SN + '_execute_real_camera_capture', PO + 'download_and_save', '', 'self.pipeline.download_and_save('),
    (SN + '_query_gimbal_attitude', 'CAM.request_gimbal_attitude', '', 'self.camera.request_gimbal_attitude('),
    (SN + '_apply_capture_zoom', 'CAM.absolute_zoom_autofocus', '', 'self.camera.absolute_zoom_autofocus('),
    (SN + '_apply_capture_zoom', 'CAM.get_current_zoom_magnification', '', 'self.camera.get_current_zoom_magnification('),
    (SN + '__init__', 'CAM.detect_model', '', 'self.camera.detect_model('),
    (PO + '_phase1_capture', 'CAM.send_capture_command', '', 'send_capture_command('),
    (PO + '_phase2_index', 'CAM.get_new_file', '', 'get_new_file('),
    (PO + '_download_bytes', 'CAM.download_image', '', 'download_image('),
    (PO + '_decode_and_verify', 'CAM.decode_image', '', 'decode_image('),
    (PO + 'initialize_sd_card', 'CAM.initialize_sd_card', '', '.initialize_sd_card('),   # the call, not its own def line
    (CH + 'CommandHandler._capture', CH + 'HandlerHooks.take_photo', '', 'take_photo('),
]
# (owner, kind, topic, proof in the owner's FILE): which function subscribes / publishes; ext:mavros is outside.
TOPIC_LINKS = [
    (M + 'image_result_cb', 'subscribes', '/image_detection', 'self.image_result_cb'),
    (M + 'update_waypoint_reached', 'subscribes', '/mavros/mission/reached', 'self.update_waypoint_reached'),
    (M + 'parameter_event_cb', 'subscribes', '/parameter_events', 'self.parameter_event_cb'),
    (M + 'waypoints_cb', 'subscribes', '/mavros/mission/waypoints', 'self.waypoints_cb'),
    (M + 'send_ack', 'publishes', '/mavros/statustext/send', 'self.status_publisher.publish'),
    (TRG + 'statustext_callback', 'subscribes', '/mavros/statustext/recv', 'self.statustext_callback'),
    (TRG + 'update_waypoint_reached', 'subscribes', '/mavros/mission/reached', 'self.update_waypoint_reached'),
    (TRG + 'parameter_event_cb', 'subscribes', '/parameter_events', 'self.parameter_event_cb'),
    (TRG + 'send_ack', 'publishes', '/mavros/statustext/send', 'self.status_publisher.publish'),
    (TRG + 'trigger_camera', 'publishes', '/camera/trigger', 'camera_trigger_pub'),
    (TRG + 'request_capture_setup', 'publishes', '/camera/command', 'camera_command_pub'),
    (TRG + 'request_autofocus', 'publishes', '/camera/command', 'camera_command_pub'),
    (SN + 'camera_trigger_callback', 'subscribes', '/camera/trigger', 'self.camera_trigger_callback'),
    (SN + 'camera_command_callback', 'subscribes', '/camera/command', 'self.camera_command_callback'),
    (SN + 'gps_cb', 'subscribes', '/mavros/global_position/global', 'self.gps_cb'),
    ('ext:mavros', 'publishes', '/mavros/statustext/recv', None),
    ('ext:mavros', 'publishes', '/mavros/mission/reached', None),
    ('ext:mavros', 'publishes', '/mavros/mission/waypoints', None),
    ('ext:mavros', 'publishes', '/mavros/global_position/global', None),
    ('ext:mavros', 'subscribes', '/mavros/statustext/send', None),
]
# Links between programs and files (facts from AGENTS.md sections 3 and 4).
LINKS = [
    ('ext:ardupilot_sitl', 'ext:mavproxy', 'mavlink', 'TCP 5760'),
    ('ext:mavproxy', 'ext:mavros', 'mavlink', 'UDP 14552'),
    ('ext:mavproxy', 'ext:mission_planner', 'mavlink', 'UDP 14550 (Windows)'),
    ('ext:gazebo', 'ext:ardupilot_sitl', 'data', 'physics, JSON over UDP 9002'),
    ('ext:mission_planner', 'mod:simulation/windows/mission_planner_plugin/astra_gcs.cs', 'runs', 'loads the plugin'),
    ('ext:mavproxy', 'fn:simulation/windows/mission_planner_plugin/astra_gcs.cs::AstraPlugin.OnStatusText', 'mavlink', 'STATUSTEXT'),
    ('mod:simulation/scripts/demo.sh', 'mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'runs', 'step 1: the simulator'),
    ('mod:simulation/scripts/demo.sh', 'mod:ros2_ws/src/main/launch/tracker.launch.xml', 'runs', './launcher.sh sim'),
    ('mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'ext:gazebo', 'launches', 'gz sim'),
    ('mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'ext:ardupilot_sitl', 'launches', 'arducopter'),
    ('mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'ext:mavproxy', 'launches', 'mavproxy.py'),
    ('mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'mod:ros2_ws/src/simulation/siyi_emulator/siyi_emulator/node.py', 'launches', 'siyi_emulator'),
    ('mod:ros2_ws/src/main/launch/tracker.launch.xml', 'mod:ros2_ws/src/main/src/main_controller.py', 'launches', 'main_controller'),
    ('mod:ros2_ws/src/main/launch/tracker.launch.xml', 'mod:ros2_ws/src/mapping/mapping/do_digi_cam_trigger.py', 'launches', 'mission_camera_trigger'),
    ('mod:ros2_ws/src/main/launch/tracker.launch.xml', 'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py', 'launches', 'siyi'),
    ('mod:ros2_ws/src/main/launch/tracker.launch.xml', 'ext:mavros', 'launches', 'via launcher.sh'),
    ('CAM.send_capture_command', CH + 'CommandHandler.handle', 'data', 'SIYI SDK 0x0C, UDP 37260'),
    ('CAM.request_gimbal_attitude', CH + 'CommandHandler.handle', 'data', 'SIYI SDK 0x0D, UDP 37260'),
    ('CAM.download_image', 'mod:ros2_ws/src/simulation/siyi_emulator/siyi_emulator/media_server.py', 'http', 'HTTP :82 photo download'),
    ('pkg:ros2_ws/src/simulation/siyi_emulator', 'ext:gazebo', 'data', 'photo trigger + camera image (gz topics)'),
    (CH + 'CommandHandler.handle', CH + 'CommandHandler._capture', 'calls', 'via the command table'),
    (CH + 'CommandHandler.handle', CH + 'CommandHandler._attitude', 'calls', 'via the command table'),
    (CH + 'CommandHandler.handle', CH + 'CommandHandler._absolute_zoom', 'calls', 'via the command table'),
    (CH + 'CommandHandler.handle', CH + 'CommandHandler._hardware_id', 'calls', 'via the command table'),
    (CH + 'CommandHandler.handle', CH + 'CommandHandler._current_zoom', 'calls', 'via the command table'),
]


def repo_file(rel):
    return os.path.join(REPO, *rel.split('/'))


def read_text(rel):
    with open(repo_file(rel), encoding='utf-8') as fh:
        return fh.read()


def lang_of(rel):
    if rel.endswith('.py'):
        return 'py'
    if rel.endswith('.sh'):
        return 'sh'
    if rel.endswith('.cs'):
        return 'cs'
    if rel.endswith('.xml'):
        return 'xml'
    return ''


def header_doc(rel, src):
    """The file's PURPOSE header: the module docstring (Python) or the first comment block (other files)."""
    if rel.endswith('.py'):
        return (ast.get_docstring(ast.parse(src)) or '').strip()
    lines = src.splitlines()
    if rel.endswith('.xml'):
        m = re.search(r'<!--(.*?)-->', src, re.S)
        return m.group(1).strip() if m else ''
    if rel.endswith('.cs'):
        m = re.match(r'\s*/\*(.*?)\*/', src, re.S)
        return re.sub(r'^\s*\* ?', '', m.group(1), flags=re.M).strip() if m else ''
    out = []
    for ln in lines[1:]:   # shell: the # block right after the shebang
        if ln.startswith('#'):
            out.append(ln[1:].strip())
        else:
            break
    return '\n'.join(out).strip()


def py_defs(rel, src):
    """Every class / function of a Python file with contract qualnames (nested: outer.inner, never <locals>)."""
    tree = ast.parse(src)
    lines = src.splitlines()
    out = []

    def visit(body, prefix, parent_id, parent_kind):
        for node in body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                qual = prefix + node.name
                fid = 'fn:' + rel + '::' + qual
                first = min([d.lineno for d in node.decorator_list] + [node.lineno])
                end = getattr(node, 'end_lineno', node.lineno)
                if isinstance(node, ast.ClassDef):
                    kind = 'class'
                    bases = ', '.join(ast.unparse(b) for b in node.bases)
                    sig = 'class ' + node.name + ('(' + bases + ')' if bases else '')
                else:
                    kind = 'method' if parent_kind == 'class' else 'function'
                    sig = ('async def ' if isinstance(node, ast.AsyncFunctionDef) else 'def ') + node.name + '(' + ast.unparse(node.args) + ')'
                    if node.returns is not None:
                        sig += ' -> ' + ast.unparse(node.returns)
                out.append({'id': fid, 'qual': qual, 'kind': kind, 'parent': parent_id, 'line': first, 'end_line': end,
                            'signature': sig, 'doc': (ast.get_docstring(node) or '').strip(),
                            'source': '\n'.join(lines[first - 1:end]), 'node': node})
                visit(node.body, qual + '.', fid, kind)

    visit(tree.body, '', 'mod:' + rel, 'module')
    return out, tree


def bash_defs(rel, src):
    lines = src.splitlines()
    out = []
    for i, ln in enumerate(lines):
        m = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)\(\)\s*\{', ln)
        if not m:
            continue
        end = i
        if ln.rstrip().endswith('}') and ln.count('{') == ln.count('}'):
            end = i
        else:
            for j in range(i + 1, len(lines)):
                if lines[j].rstrip() == '}':
                    end = j
                    break
        out.append({'qual': m.group(1), 'line': i + 1, 'end_line': end + 1, 'source': '\n'.join(lines[i:end + 1]),
                    'signature': m.group(1) + '()'})
    return out


def cs_defs(rel, src, wanted):
    """Find Class.Method in a C# file: the method's signature line, then match its braces."""
    lines = src.splitlines()
    out = []
    cls = None
    for i, ln in enumerate(lines):
        mc = re.match(r'\s*(?:public |internal |private |static |sealed |partial )*class\s+(\w+)', ln)
        if mc:
            cls = mc.group(1)
            continue
        mm = re.match(r'\s*(?:public|private|protected|internal)[^=;(]*?\s(\w+)\s*\(', ln)
        if not mm or not cls or (cls + '.' + mm.group(1)) not in wanted:
            continue
        depth, started, end = 0, False, i
        for j in range(i, len(lines)):
            depth += lines[j].count('{') - lines[j].count('}')
            if '{' in lines[j]:
                started = True
            if started and depth <= 0:
                end = j
                break
        out.append({'qual': cls + '.' + mm.group(1), 'line': i + 1, 'end_line': end + 1, 'signature': ln.strip(),
                    'source': '\n'.join(lines[i:end + 1]), 'cls': cls})
    return out


def load_explanations():
    entries = {}
    for path in sorted(glob.glob(os.path.join(EXPLANATIONS, '*.json'))):
        try:
            with open(path, encoding='utf-8') as fh:
                data = json.load(fh)
        except (OSError, ValueError) as err:
            print('  (skipping unreadable explanations file ' + os.path.basename(path) + ': ' + str(err) + ')')
            continue
        for k, v in (data.get('entries') or {}).items():
            if isinstance(v, dict) and v.get('explain'):
                entries.setdefault(k, v)
    return entries


def call_edges(rel, tree, defs, chosen, module_funcs):
    """Calls written as self.m(...) (same class), f(...) (same file / imported name) or alias.f(...) (imported file)."""
    alias_to_rel, name_to_id = {}, {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                for other in module_funcs:
                    if os.path.splitext(os.path.basename(other))[0] == a.name.split('.')[-1]:
                        alias_to_rel[a.asname or a.name] = other
        elif isinstance(node, ast.ImportFrom) and node.module:
            for other in module_funcs:
                if os.path.splitext(os.path.basename(other))[0] == node.module.split('.')[-1]:
                    for a in node.names:
                        name_to_id[a.asname or a.name] = 'fn:' + other + '::' + a.name
    edges = []
    for d in defs:
        if d['id'] not in chosen or d['kind'] == 'class':
            continue
        cls_prefix = d['qual'].split('.')[0] + '.' if d['kind'] == 'method' or '.' in d['qual'] else ''
        own = d['node']
        nested = {id(n) for n in ast.walk(own) if n is not own and isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))}
        skip = set()
        for n in ast.walk(own):
            if id(n) in nested:
                skip.update(id(x) for x in ast.walk(n) if x is not n)
        for n in ast.walk(own):
            if id(n) in skip or not isinstance(n, ast.Call):
                continue
            f, target = n.func, None
            if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) and f.value.id == 'self' and cls_prefix:
                target = 'fn:' + rel + '::' + cls_prefix + f.attr
            elif isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) and f.value.id in alias_to_rel:
                target = 'fn:' + alias_to_rel[f.value.id] + '::' + f.attr
            elif isinstance(f, ast.Name):
                for cand in ('fn:' + rel + '::' + d['qual'] + '.' + f.id, 'fn:' + rel + '::' + f.id, name_to_id.get(f.id)):
                    if cand and cand in chosen:
                        target = cand
                        break
            if target and target in chosen and target != d['id']:
                edges.append((d['id'], target))
    return edges


def build_codemap():
    explained = load_explanations()
    nodes, edges = [], []
    trace_fns, py_files = {}, {}

    def add_node(node):
        ex = explained.get(node['id'])
        if ex:
            for key in ('explain', 'called_when', 'inputs', 'outputs', 'effects', 'how_it_fits'):
                if ex.get(key):
                    node[key] = ex[key]
        nodes.append(node)

    for sid, label, text in SYSTEMS:
        add_node({'id': sid, 'kind': 'system', 'label': label, 'parent': None, 'summary': text})
    for eid, label, text in EXTERNALS:
        add_node({'id': eid, 'kind': 'external', 'label': label, 'parent': None, 'summary': text})
    for pid, parent, label in PACKAGES:
        add_node({'id': pid, 'kind': 'package', 'label': label, 'parent': parent, 'path': pid[4:]})
    for name, text in TOPICS.items():
        add_node({'id': 'topic:' + name, 'kind': 'topic', 'label': name, 'parent': None, 'summary': text})

    chosen_all, per_module = set(), {}
    cam_class = None
    for rel, pkg, label, want in MODULES:
        src = read_text(rel)
        mid = 'mod:' + rel
        mod = {'id': mid, 'kind': 'module', 'label': label, 'parent': pkg, 'path': rel, 'lang': lang_of(rel),
               'line': 1, 'end_line': len(src.splitlines()), 'doc': header_doc(rel, src),
               'is_test': '/test/' in rel or os.path.basename(rel).startswith('test_')}
        if rel in ENTRY_POINTS:
            mod['entry_point'] = ENTRY_POINTS[rel]
        add_node(mod)
        if rel.endswith('.py'):
            py_files[rel] = mid
            defs, tree = py_defs(rel, src)
            if want == ['*']:
                picked = [d for d in defs]
            elif want and want[0].startswith('first:'):
                picked = [d for d in defs if d['kind'] != 'class'][:int(want[0].split(':')[1])]
            else:
                picked = []
                for d in defs:
                    for w in want:
                        if w == d['qual'] or (w.startswith('*.') and d['kind'] == 'method' and d['qual'].endswith(w[1:])):
                            picked.append(d)
                            break
            ids = {d['id'] for d in picked}
            for d in list(picked):   # parents of chosen methods / nested functions come along
                p = d['parent']
                while p.startswith('fn:') and p not in ids:
                    parent = next(x for x in defs if x['id'] == p)
                    picked.append(parent)
                    ids.add(p)
                    p = parent['parent']
            if rel == CAM_MOD:
                cam_class = next(d['qual'].split('.')[0] for d in picked if d['kind'] == 'method')
            picked.sort(key=lambda d: d['line'])
            for d in picked:
                add_node({'id': d['id'], 'kind': d['kind'], 'label': d['qual'].split('.')[-1], 'parent': d['parent'],
                          'path': rel, 'lang': 'py', 'line': d['line'], 'end_line': d['end_line'], 'signature': d['signature'],
                          'doc': d['doc'], 'source': d['source'], 'is_test': mod['is_test']})
                trace_fns.setdefault(mid, {})[str(d['line'])] = d['id']
            chosen_all.update(ids)
            per_module[rel] = (tree, defs)
        elif rel.endswith('.sh'):
            for d in bash_defs(rel, src):
                if d['qual'] in want:
                    add_node({'id': 'fn:' + rel + '::' + d['qual'], 'kind': 'function', 'label': d['qual'], 'parent': mid,
                              'path': rel, 'lang': 'sh', 'line': d['line'], 'end_line': d['end_line'],
                              'signature': d['signature'], 'source': d['source']})
        elif rel.endswith('.cs'):
            found = cs_defs(rel, src, set(want))
            classes = sorted({d['cls'] for d in found})
            for c in classes:
                first = min(d['line'] for d in found if d['cls'] == c)
                add_node({'id': 'fn:' + rel + '::' + c, 'kind': 'class', 'label': c, 'parent': mid, 'path': rel, 'lang': 'cs', 'line': first})
            for d in found:
                add_node({'id': 'fn:' + rel + '::' + d['qual'], 'kind': 'method', 'label': d['qual'].split('.')[-1],
                          'parent': 'fn:' + rel + '::' + d['cls'], 'path': rel, 'lang': 'cs', 'line': d['line'],
                          'end_line': d['end_line'], 'signature': d['signature'], 'source': d['source']})

    ids = {n['id'] for n in nodes}
    by_id = {n['id']: n for n in nodes}
    cam = lambda x: x.replace('CAM.', 'fn:' + CAM_MOD + '::' + str(cam_class) + '.')
    py_mods = {rel: per_module[rel] for rel in per_module}
    seen = set()

    def add_edge(a, b, kind, label=''):
        if a not in ids or b not in ids:
            raise SystemExit('sample link to an unknown id: ' + a + ' -> ' + b)
        key = (a, b, kind)
        if key in seen:
            return
        seen.add(key)
        e = {'from': a, 'to': b, 'kind': kind}
        if label:
            e['label'] = label
        edges.append(e)

    for rel, (tree, defs) in py_mods.items():
        for a, b in call_edges(rel, tree, defs, chosen_all, list(py_mods)):
            add_edge(a, b, 'calls')
    for caller, callee, label, proof in EXTRA_CALLS:
        caller, callee = cam(caller), cam(callee)
        if caller not in by_id or callee not in by_id:
            print('  skipped extra call (not in the sample): ' + caller + ' -> ' + callee)
        elif proof not in by_id[caller].get('source', ''):
            print('  skipped extra call (proof "' + proof + '" not in the source): ' + caller)
        else:
            add_edge(caller, callee, 'calls', label)
    for rel in ['ros2_ws/src/main/src/main_controller.py', 'ros2_ws/src/video_cam/video_cam/siyi_node.py',
                'ros2_ws/src/video_cam/video_cam/pipeline_orchestrator.py', 'ros2_ws/src/main/test/test_target_geolocation.py']:
        src = read_text(rel)
        for other in py_mods:
            base = os.path.splitext(os.path.basename(other))[0]
            if other != rel and re.search(r'^\s*(import\s+(\w+\.)*' + base + r'\b|from\s+\.*(\w+\.)*' + base + r'\s+import)', src, re.M):
                add_edge('mod:' + rel, 'mod:' + other, 'imports')
    for owner, kind, topic, proof in TOPIC_LINKS:
        if proof is not None:
            n = by_id.get(owner)
            if not n:
                print('  skipped topic link (not in the sample): ' + owner)
                continue
            src = read_text(n['path'])
            if topic not in src or proof not in src:
                print('  skipped topic link (proof not in ' + n['path'] + '): ' + owner + ' ' + kind + ' ' + topic)
                continue
        add_edge(owner, 'topic:' + topic, kind)
    for a, b, kind, label in LINKS:
        a, b = cam(a), cam(b)
        add_edge(a, b, kind, label)

    commit = ''
    try:
        commit = subprocess.run(['git', '-C', REPO, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        commit = ''
    py_modules = {name: 'mod:' + rel for rel, name in PY_MODULE_NAMES.items()}
    return {
        'schema': 1,
        'generated': datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0).isoformat(),
        'commit': commit or 'unknown',
        'sample': True,
        'note': 'SAMPLE map for testing the explorer page, made by simulation/codemap/web/make_sample.py from real files of '
                'this repo (a small subset). It is not the full code map; extract.py makes that.',
        'nodes': nodes,
        'edges': edges,
        'trace_index': {'py_modules': py_modules, 'py_files': py_files, 'functions': trace_fns},
    }


def build_recording(cm):
    """About 72 s of SYNTHETIC events shaped like the start of a search flight (see the PURPOSE above)."""
    ids = {n['id'] for n in cm['nodes']}
    cam_cls = next(n['id'] for n in cm['nodes'] if n['id'].startswith('fn:' + CAM_MOD + '::') and n['kind'] == 'class')
    CAM = cam_cls + '.'
    TG = 'fn:ros2_ws/src/main/src/target_geolocation.py::'
    SL = 'fn:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py::'
    DS = 'fn:simulation/scripts/demo.sh::'
    buckets = {}

    def at(t, fid, n=1):
        if fid not in ids:
            raise SystemExit('recording uses an id that is not in the sample map: ' + fid)
        b = buckets.setdefault(round(t / 0.2), {})
        b[fid] = b.get(fid, 0) + n

    at(0.2, DS + 'port_busy', 4)
    at(0.6, SL + 'generate_launch_description')
    at(0.8, SL + '_running_in_wsl')
    at(0.8, SL + 'share_or_none', 3)
    at(0.8, SL + '_launch_setup')
    at(1.0, SL + '_launch_setup.add_mavproxy')
    at(1.2, 'fn:ros2_ws/src/simulation/siyi_emulator/siyi_emulator/node.py::main')
    at(1.4, CH + 'CommandHandler.__init__')
    at(4.2, M + '__init__')
    at(4.2, M + '_number_parameter', 5)
    at(4.2, M + 'fetch_mission_indices')
    at(4.4, TRG + '__init__')
    at(4.4, TRG + 'number_parameter', 2)
    at(4.4, TRG + 'fetch_mission_indices')
    at(4.6, SN + '__init__')
    at(4.6, SN + '_apply_camera_model')
    at(4.6, CAM + 'detect_model')
    at(4.6, CAM + 'get_hardware_id')
    at(4.6, CH + 'CommandHandler.handle', 2)
    at(4.6, CH + 'CommandHandler._hardware_id')
    at(4.8, PO + '__init__')
    at(4.8, PO + 'initialize_sd_card')
    at(4.8, CAM + 'initialize_sd_card')
    at(5.2, M + '_on_mission_indices')
    at(5.2, TRG + '_buffer_wp_cb')
    for k in range(23):
        t = 5.4 + 3 * k
        at(t, M + 'parameter_event_cb')
        at(t, TRG + 'parameter_event_cb')
    at(6.0, TRG + 'statustext_callback')
    at(6.0, TRG + 'request_capture_setup')
    at(6.0, TRG + 'start_autofocus_loop')
    at(6.2, SN + 'camera_command_callback')
    at(6.2, SN + '_apply_capture_zoom')
    at(6.4, CAM + 'absolute_zoom_autofocus')
    at(6.4, CAM + 'get_current_zoom_magnification', 3)
    at(6.4, CH + 'CommandHandler.handle', 4)
    at(6.4, CH + 'CommandHandler._absolute_zoom')
    at(6.4, CH + 'CommandHandler._current_zoom', 3)
    for k in range(6):
        at(11.0 + 5 * k, TRG + 'request_autofocus')
        at(11.2 + 5 * k, SN + 'camera_command_callback')
    for k in range(33):   # one photo per second from 7 s to 39 s
        t = 7.0 + k
        at(t, TRG + 'trigger_camera')
        at(t + 0.2, SN + 'camera_trigger_callback')
        at(t + 0.2, SN + '_service_capture_queue')
        at(t + 0.4, SN + '_execute_real_camera_capture')
        at(t + 0.4, SN + '_capture_metadata')
        at(t + 0.4, SN + '_capture_metadata.age', 3)
        at(t + 0.4, SN + '_query_gimbal_attitude')
        at(t + 0.4, CAM + 'request_gimbal_attitude')
        at(t + 0.4, PO + 'capture_and_index')
        at(t + 0.4, PO + '_phase1_capture')
        at(t + 0.4, CAM + 'send_capture_command')
        at(t + 0.4, CH + 'CommandHandler.handle', 2)
        at(t + 0.4, CH + 'CommandHandler._capture')
        at(t + 0.4, CH + 'CommandHandler._attitude')
        at(t + 0.4, CH + 'HandlerHooks.take_photo')
        at(t + 0.6, PO + '_phase2_index')
        at(t + 0.6, CAM + 'get_media_list', 2)
        at(t + 0.6, CAM + 'get_new_file')
        at(t + 0.6, SN + 'gps_cb')
        at(t + 0.8, PO + 'download_and_save')
        at(t + 0.8, PO + '_phase3_download')
        at(t + 0.8, PO + '_download_bytes')
        at(t + 0.8, CAM + 'download_image')
        at(t + 0.8, PO + '_decode_and_verify')
        at(t + 0.8, CAM + 'decode_image')
        at(t + 1.6, M + 'image_result_cb')
        if k % 6 == 3:   # a detection: aim at the box
            at(t + 1.6, M + 'valid_detection')
            at(t + 1.6, M + 'locate_detection')
            at(t + 1.6, M + '_latlon_from_name')
            at(t + 1.6, TG + 'read_photo_sidecar')
            at(t + 1.6, TG + 'photo_position')
            at(t + 1.6, TG + 'aim_at_box')
            at(t + 1.6, TG + 'pixel_to_ground_offset')
            at(t + 1.6, TG + 'offset_to_latlon')
            at(t + 1.6, TG + 'valid_latlon', 2)
            at(t + 1.8, M + 'send_ack')
    for t in (12.0, 20.0, 28.0, 36.0):
        at(t, M + 'update_waypoint_reached')
        at(t, TRG + 'update_waypoint_reached')
    at(40.0, TRG + 'update_waypoint_reached')
    at(40.0, TRG + 'stop_camera_trigger')
    at(40.0, TRG + 'stop_autofocus_loop')
    at(40.2, TRG + 'send_ack')
    at(42.0, M + 'update_waypoint_reached')
    at(42.0, M + 'change_mode')
    at(42.2, M + 'send_ack')
    at(42.4, M + '_on_mode_result')
    for t in (44.0, 46.0, 48.0):
        at(t, M + '_check_all_images_processed')
    at(48.2, M + '_end_processing_hold')
    at(48.2, M + '_divert_to_targets')
    at(48.4, M + 'send_waypoint_data', 2)
    at(48.4, M + 'send_ack', 3)
    at(52.2, M + 'divert_and_resume')
    at(52.2, M + 'set_current_wp')
    at(53.2, M + 'divert_and_resume.resume')
    at(53.2, M + 'change_mode')
    at(53.6, M + '_on_mode_result')
    at(60.0, M + 'update_waypoint_reached')
    at(60.0, M + '_loiter_and_drop')
    at(60.0, M + 'change_mode')
    at(60.2, M + 'send_ack')
    at(60.4, M + '_on_mode_result')
    at(66.0, M + 'actuate_servos')
    at(66.0, M + 'open_servos')
    at(66.0, M + 'move_servo')
    at(66.4, M + '_on_servo_result')
    at(66.2, M + 'send_ack')
    at(69.0, M + 'close_servos')
    at(69.0, M + 'move_servo')
    at(69.4, M + '_on_servo_result')
    at(69.4, M + '_loiter_and_drop.after_close')

    t0 = datetime.datetime(2026, 10, 31, 15, 0, 0, tzinfo=datetime.timezone.utc).timestamp()
    sim = ['mod:simulation/scripts/demo.sh', 'mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'ext:gazebo',
           'ext:ardupilot_sitl', 'ext:mavproxy', 'ext:mission_planner']
    team = ['mod:ros2_ws/src/main/src/main_controller.py', 'mod:ros2_ws/src/mapping/mapping/do_digi_cam_trigger.py',
            'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py']
    events = [{'event': 'meta', 'note': 'SYNTHETIC sample made by simulation/codemap/web/make_sample.py to test the page; '
                                        'it is shaped like the start of a search flight but was NOT recorded from a real run.'}]
    timeline = []
    for b, calls in buckets.items():
        timeline.append((b * 0.2, {'event': 'calls', 't': round(t0 + b * 0.2, 3), 'calls': dict(sorted(calls.items()))}))
    for k in range(37):
        t = 2.0 * k
        running = list(sim)
        if t >= 1.2:
            running.append('mod:ros2_ws/src/simulation/siyi_emulator/siyi_emulator/node.py')
        if t >= 3.0:
            running.append('ext:mavros')
        if t >= 4.0:
            running += team
        timeline.append((t + 0.1, {'event': 'procs', 't': round(t0 + t + 0.1, 3), 'running': running}))
    timeline.sort(key=lambda x: x[0])
    events += [ev for _, ev in timeline]
    for ev in events:
        for fid in ev.get('running', []):
            if fid not in ids:
                raise SystemExit('recording lists a program that is not in the sample map: ' + fid)
    return events


def main(argv=None):
    ap = argparse.ArgumentParser(description='Make sample_codemap.json and sample_recording.jsonl for the explorer page.')
    ap.add_argument('--check', action='store_true', help='only print what would be written')
    args = ap.parse_args(argv)
    cm = build_codemap()
    events = build_recording(cm)
    kinds = {}
    for n in cm['nodes']:
        kinds[n['kind']] = kinds.get(n['kind'], 0) + 1
    ekinds = {}
    for e in cm['edges']:
        ekinds[e['kind']] = ekinds.get(e['kind'], 0) + 1
    explained = sum(1 for n in cm['nodes'] if n.get('explain') or n.get('summary'))
    print('nodes: ' + ', '.join(k + ' ' + str(v) for k, v in sorted(kinds.items())))
    print('links: ' + ', '.join(k + ' ' + str(v) for k, v in sorted(ekinds.items())))
    print('with a plain explanation or summary: ' + str(explained) + ' of ' + str(len(cm['nodes'])))
    print('recording: ' + str(len(events)) + ' lines, ' + str(sum(1 for e in events if e['event'] == 'calls')) + ' calls events')
    if args.check:
        return 0
    with open(os.path.join(HERE, 'sample_codemap.json'), 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(cm, fh, ensure_ascii=False, indent=1)
        fh.write('\n')
    with open(os.path.join(HERE, 'sample_recording.jsonl'), 'w', encoding='utf-8', newline='\n') as fh:
        for ev in events:
            fh.write(json.dumps(ev, ensure_ascii=False, separators=(',', ':')) + '\n')
    print('wrote ' + os.path.join(HERE, 'sample_codemap.json') + ' and sample_recording.jsonl')
    return 0


if __name__ == '__main__':
    sys.exit(main())
