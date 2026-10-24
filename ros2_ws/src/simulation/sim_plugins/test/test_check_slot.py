"""
sim_plugins/test/test_check_slot.py -- PURPOSE: unit tests of the conformance checker (check_slot.py) with FAKES:
a FakeGraph stands in for the live ROS graph and SimpleNamespace objects for messages, so every rule is tested
without ROS: the team's new_od passes, a BEST_EFFORT / wrong-type / missing publisher fails, a second detector
warns, the mission brain's service clients are checked, sample messages are sanity-checked, and the camera's
network probe results are reported.
"""

from types import SimpleNamespace as NS

import pytest

from sim_plugins import check_slot as cs
from sim_plugins.check_slot import Endpoint as EP
from sim_plugins.check_slot import FakeGraph

IR = 'interfaces/msg/ImageResult'
TRIG = 'std_srvs/srv/Trigger'


def team_graph(**changes):
    """A graph like the running demo: new_od, main_controller, MAVROS, waypoint_manager, the panel."""
    g = dict(
        nodes=['new_od', 'main_controller', 'mavros', 'waypoint_manager', 'demo_panel', 'siyi', 'sim_bridge'],
        pubs={'/image_detection': [EP('new_od', IR, 'reliable')],
              '/mavros/state': [EP('mavros', 'mavros_msgs/msg/State', 'reliable')],
              '/sim/drone/odometry': [EP('sim_bridge', 'nav_msgs/msg/Odometry', 'reliable')],
              '/mavros/statustext/send': [EP('main_controller', 'mavros_msgs/msg/StatusText')]},
        subs={'/image_detection': [EP('main_controller', IR), EP('demo_panel', IR)],
              '/mavros/mission/reached': [EP('new_od', 'mavros_msgs/msg/WaypointReached'),
                                          EP('main_controller', 'mavros_msgs/msg/WaypointReached')],
              '/mavros/mission/waypoints': [EP('main_controller', 'mavros_msgs/msg/WaypointList')],
              '/parameter_events': [EP('main_controller', 'rcl_interfaces/msg/ParameterEvent')]},
        servers={'new_od': {'/sahi/get_health': [TRIG], '/sahi/get_statistics': [TRIG]},
                 'waypoint_manager': {'/addWaypoint': ['interfaces/srv/AddWaypoint']},
                 'mavros': {'/mavros/set_mode': ['mavros_msgs/srv/SetMode'],
                            '/mavros/cmd/command': ['mavros_msgs/srv/CommandLong'],
                            '/mavros/mission/set_current': ['mavros_msgs/srv/WaypointSetCurrent']}},
        clients={'main_controller': {'/addWaypoint': ['interfaces/srv/AddWaypoint'],
                                     '/mavros/set_mode': ['mavros_msgs/srv/SetMode'],
                                     '/mavros/cmd/command': ['mavros_msgs/srv/CommandLong'],
                                     '/mavros/mission/set_current': ['mavros_msgs/srv/WaypointSetCurrent'],
                                     '/waypoint_manager/get_parameters': ['rcl_interfaces/srv/GetParameters']}},
    )
    g.update(changes)
    return FakeGraph(**g)


def levels(report):
    return [f.level for f in report.findings]


def texts(report):
    return '\n'.join(f.render() for f in report.findings)


# ---- detector ----------------------------------------------------------------------------------------

def test_team_detector_passes():
    r = cs.evaluate(team_graph(), 'detector')
    assert r.node == 'new_od' and not r.failed, texts(r)
    assert 'OK' in r.render() and 'RESULT: PASS' in r.render()


def test_custom_detector_is_found_by_its_output():
    g = team_graph(nodes=['example_detector', 'main_controller'],
                   pubs={'/image_detection': [EP('example_detector', IR, 'reliable')]},
                   servers={'example_detector': {'/sahi/get_health': [TRIG]}})
    r = cs.evaluate(g, 'detector')
    assert r.node == 'example_detector' and not r.failed, texts(r)


def test_best_effort_publisher_fails():
    g = team_graph(nodes=['my_det', 'main_controller'], pubs={'/image_detection': [EP('my_det', IR, 'best_effort')]})
    r = cs.evaluate(g, 'detector')
    assert r.failed and 'BEST_EFFORT' in texts(r) and 'ReliabilityPolicy.RELIABLE' in texts(r)


def test_wrong_message_type_fails():
    g = team_graph(nodes=['my_det'], pubs={'/image_detection': [EP('my_det', 'std_msgs/msg/String')]})
    r = cs.evaluate(g, 'detector', node='my_det')
    assert r.failed and 'must be interfaces/msg/ImageResult' in texts(r)


def test_no_detector_fails_with_a_hint():
    r = cs.evaluate(team_graph(nodes=['main_controller'], pubs={}), 'detector')
    assert r.failed and 'no node publishes /image_detection' in texts(r)


def test_named_node_must_exist():
    r = cs.evaluate(team_graph(), 'detector', node='nope')
    assert r.failed and 'not running' in texts(r)


def test_two_detectors_warn():
    g = team_graph(nodes=['new_od', 'truth_detector', 'main_controller'],
                   pubs={'/image_detection': [EP('new_od', IR), EP('truth_detector', IR)]})
    r = cs.evaluate(g, 'detector')
    assert not r.failed and 'warn' in levels(r) and 'truth_detector' in texts(r)


def test_optional_rules_are_info_not_failures():
    g = team_graph(nodes=['my_det'], pubs={'/image_detection': [EP('my_det', IR)]}, subs={}, servers={})
    r = cs.evaluate(g, 'detector', node='my_det')
    assert not r.failed and levels(r).count('info') >= 3


def photo_result(**over):
    det = NS(results=[NS(hypothesis=NS(class_id='1', score=0.83))],
             bbox=NS(center=NS(position=NS(x=1200.0, y=700.0)), size_x=80.0, size_y=60.0))
    msg = NS(image_name='36.215026 , -96.006800.jpg', method='my_det v1', num_detections=1,
             detections=NS(detections=[det]), latitude=36.215026, longitude=-96.0068)
    for k, v in over.items():
        setattr(msg, k, v)
    return msg


def test_good_sample_passes(tmp_path):
    (tmp_path / '36.215026 , -96.006800.jpg').write_bytes(b'x')
    r = cs.Report('detector')
    cs.check_image_result(photo_result(), r, photo_dir=str(tmp_path))
    assert not r.failed and 'tent' in texts(r) and 'photo exists' in texts(r)


@pytest.mark.parametrize('change, words', [
    ({'image_name': ''}, 'image_name is empty'),
    ({'image_name': 'mapping_photos/a.jpg'}, 'contains a folder'),
    ({'method': 'truth_detector (stand-in)'}, 'starts with truth_detector'),
    ({'latitude': 123.0}, 'not a position'),
])
def test_bad_samples_fail(change, words):
    r = cs.Report('detector')
    cs.check_image_result(photo_result(**change), r)
    assert r.failed and words in texts(r)


def test_bad_detection_fields_fail():
    det = NS(results=[NS(hypothesis=NS(class_id='person', score=1.7))],
             bbox=NS(center=NS(position=NS(x=-5.0, y=10.0)), size_x=0.0, size_y=10.0))
    r = cs.Report('detector')
    cs.check_image_result(photo_result(detections=NS(detections=[det])), r)
    t = texts(r)
    assert r.failed and 'class_id "person"' in t and 'score 1.7' in t and 'box size' in t and 'outside' in t


def test_sample_warnings():
    r = cs.Report('detector')
    cs.check_image_result(photo_result(latitude=0.0, longitude=0.0, num_detections=3, method=''), r,
                          photo_dir='/definitely/not/here')
    assert not r.failed and levels(r).count('warn') == 4


def test_new_od_counting_other_objects_is_fine():
    # new_od: num_detections and classes count EVERY detection, the "object" class too, while the detections
    # list holds only persons and tents (detection/new_od.py _publish_results). That is the contract, no warning.
    r = cs.Report('detector')
    cs.check_image_result(photo_result(num_detections=2, classes=['tent', 'object']), r)
    assert not r.failed and 'warn' not in levels(r), texts(r)
    r = cs.Report('detector')
    cs.check_image_result(photo_result(num_detections=0, classes=[]), r)      # fewer than the list: warn
    assert 'warn' in levels(r) and 'num_detections = 0' in texts(r)


def test_empty_photo_result_is_fine():
    r = cs.Report('detector')
    cs.check_image_result(photo_result(detections=NS(detections=[]), num_detections=0), r)
    assert not r.failed


def test_sample_from_truth_detector_in_truth_mode():
    g = team_graph(nodes=['new_od', 'truth_detector'],
                   pubs={'/image_detection': [EP('new_od', IR), EP('truth_detector', IR)]})
    r = cs.evaluate(g, 'detector', sample=photo_result(method='truth_detector (stand-in)'), sample_wanted=True)
    assert not r.failed and 'came from truth_detector' in texts(r)


def test_missing_sample_is_a_warning_with_a_hint():
    r = cs.evaluate(team_graph(), 'detector', sample=None, sample_wanted=True)
    assert not r.failed and '--take-photo' in texts(r)


# ---- mission brain ------------------------------------------------------------------------------------

def test_team_mission_brain_passes():
    r = cs.evaluate(team_graph(), 'mission_brain')
    assert r.node == 'main_controller' and not r.failed, texts(r)


def test_mission_brain_without_clients_fails():
    clients = {'my_brain': {'/mavros/set_mode': ['mavros_msgs/srv/SetMode'],
                            '/addWaypoint': ['interfaces/srv/AddWaypoint']}}
    subs = {'/image_detection': [EP('my_brain', IR)],
            '/mavros/mission/reached': [EP('my_brain', 'mavros_msgs/msg/WaypointReached')],
            '/mavros/mission/waypoints': [EP('my_brain', 'mavros_msgs/msg/WaypointList')]}
    g = team_graph(nodes=['my_brain', 'mavros', 'waypoint_manager'], clients=clients, subs=subs)
    r = cs.evaluate(g, 'mission_brain')
    t = texts(r)
    assert r.node == 'my_brain' and r.failed
    assert '/mavros/cmd/command: my_brain never calls it' in t and 'Create the client when your node starts' in t


def test_mission_brain_found_by_add_waypoint_client():
    g = team_graph(nodes=['brain2'], clients={'brain2': {'/addWaypoint': ['interfaces/srv/AddWaypoint']}})
    assert cs.find_component(g, 'mission_brain') == ['brain2']


# ---- autopilot / world / camera / extras --------------------------------------------------------------

def test_autopilot_and_world_any_node():
    r = cs.evaluate(team_graph(), 'autopilot', sample=NS(connected=True, mode='GUIDED', armed=False),
                    sample_wanted=True)
    assert not r.failed and 'connected to the autopilot' in texts(r)
    r = cs.evaluate(team_graph(), 'autopilot', sample=NS(connected=False), sample_wanted=True)
    assert r.failed and 'NOT connected' in texts(r)
    odom = NS(pose=NS(pose=NS(position=NS(x=1.0, y=2.0, z=0.2))))
    r = cs.evaluate(team_graph(), 'world', sample=odom, sample_wanted=True)
    assert not r.failed and 'in the world' in texts(r)
    r = cs.evaluate(team_graph(pubs={}), 'world')
    assert r.failed and '/sim/drone/odometry' in texts(r)


class FakeProbe:
    def __init__(self, sdk, media):
        self.sdk, self.media = sdk, media

    def probe_sdk(self, ip, timeout=2.0):
        return self.sdk, f'sdk {ip}'

    def probe_media(self, ip, timeout=3.0):
        return self.media, f'media {ip}'


def test_camera_slot_uses_the_network_probe():
    g = team_graph(nodes=['siyi_emulator'],
                   pubs={'/sim/camera/photo_info': [EP('siyi_emulator', 'std_msgs/msg/String', 'reliable')]})
    r = cs.evaluate(g, 'camera', camera_probe=lambda: cs.camera_findings('1.2.3.4', probe=FakeProbe(True, True)))
    assert r.node == 'siyi_emulator' and not r.failed, texts(r)
    r = cs.evaluate(team_graph(nodes=[], pubs={}), 'camera',
                    camera_probe=lambda: cs.camera_findings('1.2.3.4', probe=FakeProbe(True, False)))
    assert r.failed and 'no camera node in the ROS graph' in texts(r)


def test_extras():
    r = cs.evaluate(team_graph(), 'extras')
    assert r.failed and '--node' in texts(r)
    g = team_graph(nodes=['logger'], pubs={'/camera/trigger': [EP('logger', 'std_msgs/msg/Bool')]})
    r = cs.evaluate(g, 'extras', node='logger')
    assert not r.failed and 'warn' in levels(r) and '/camera/trigger' in texts(r)
    g = team_graph(nodes=['quiet'], pubs={})
    r = cs.evaluate(g, 'extras', node='quiet')
    assert not r.failed and 'does not publish on topics other slots own' in texts(r)


def test_report_render_and_counts():
    r = cs.Report('detector', node='x')
    r.add('ok', 'fine')
    r.add('warn', 'hmm', 'do this')
    assert r.counts() == {'ok': 1, 'fail': 0, 'warn': 1, 'info': 0}
    out = r.render()
    assert out.splitlines()[0] == 'check_slot: detector (component node: x)'
    assert '-> do this' in out and out.endswith('RESULT: PASS (1 check(s) passed, 1 warning(s))')
    r.add('fail', 'broken')
    assert r.failed and r.render().endswith('RESULT: FAIL (1 problem(s), 1 warning(s))')


def test_parser_rejects_unknown_slot():
    with pytest.raises(SystemExit):
        cs.build_parser().parse_args(['--slot', 'network'])
    args = cs.build_parser().parse_args(['--slot', 'detector', '--take-photo', '--timeout', '5'])
    assert args.take_photo and args.timeout == 5.0
