"""test_ros_tap.py -- PURPOSE: unit tests of the ROS tap's pure parts (no ROS needed; runs on plain Windows Python and
in WSL): the frame format both ends speak, the topic list file tap_topics.txt (the real one is parsed too), the QoS
choice that matches every publisher, the presence-node bookkeeping (no false "crashed" during a link drop), the
address parser and the command line's exit code for a typo.

Run:  python -m pytest simulation/jetson/pc/test/test_ros_tap.py -q
"""
import importlib.util
import os
import struct

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PC = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location('ros_tap', os.path.join(PC, 'ros_tap.py'))
ros_tap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ros_tap)


# ---- frames ---------------------------------------------------------------------------------------------------------

def test_frame_layout_is_length_kind_payload():
    f = ros_tap.encode_frame(ros_tap.KIND_PING)
    assert f == struct.pack('>IB', 1, 3)
    f = ros_tap.data_frame(7, b'abc')
    assert f[:4] == struct.pack('>I', 1 + 2 + 3)
    assert f[4] == ros_tap.KIND_DATA
    assert ros_tap.split_data(f[5:]) == (7, b'abc')


def test_reader_handles_frames_in_pieces_and_together():
    frames = [ros_tap.json_frame(ros_tap.KIND_HELLO, {'proto': ros_tap.PROTO}),
              ros_tap.data_frame(1, b'\x00\x01' * 100), ros_tap.encode_frame(ros_tap.KIND_PING)]
    stream = b''.join(frames)
    r = ros_tap.FrameReader()
    got = []
    for i in range(0, len(stream), 7):                    # 7-byte pieces: frames split anywhere
        got += r.feed(stream[i:i + 7])
    assert [k for k, _ in got] == [0, 2, 3]
    assert ros_tap.split_data(got[1][1]) == (1, b'\x00\x01' * 100)
    assert r.pending == 0
    r2 = ros_tap.FrameReader()
    assert len(r2.feed(stream)) == 3                       # all at once


def test_reader_rejects_impossible_length():
    r = ros_tap.FrameReader(max_frame=1000)
    with pytest.raises(ValueError):
        r.feed(struct.pack('>IB', 0, 1))
    r = ros_tap.FrameReader(max_frame=1000)
    with pytest.raises(ValueError):
        r.feed(struct.pack('>IB', 5000, 2))


def test_data_frame_id_must_fit_16_bits():
    with pytest.raises(ValueError):
        ros_tap.data_frame(70000, b'')
    with pytest.raises(ValueError):
        ros_tap.split_data(b'\x01')


# ---- topic file -----------------------------------------------------------------------------------------------------

def test_real_topic_file_parses_and_has_what_the_pc_tools_need():
    with open(os.path.join(PC, 'tap_topics.txt'), encoding='utf-8') as f:
        sec = ros_tap.parse_topic_file(f.read())
    common = dict(sec['common'])
    # what sim_demo ros_common.FlightData (score_run) and phase_monitor subscribe to (HIL_DESIGN.md section 7)
    for topic in ('/mavros/state', '/mavros/extended_state', '/mavros/global_position/global',
                  '/mavros/global_position/rel_alt', '/mavros/global_position/compass_hdg', '/mavros/vfr_hud',
                  '/mavros/local_position/pose', '/mavros/mission/reached', '/mavros/mission/waypoints',
                  '/mavros/statustext/recv', '/mavros/statustext/send'):
        assert topic in common, topic
    legacy = dict(sec['legacy'])
    assert legacy['/image_detection'] == 'interfaces/msg/ImageResult'
    assert '/camera/command' in legacy
    new = dict(sec['new'])
    assert new['/detection/results'] == 'vision_msgs/msg/Detection2DArray'
    # VIDEO_PIPELINE.md section 14 item 4: the twin's detector output and, with the proposals, the adapter's
    # /image_detection (also in [legacy]; select_topics copies each topic once)
    assert new['/yolo26/detections'] == 'vision_msgs/msg/Detection2DArray'
    assert new['/image_detection'] == 'interfaces/msg/ImageResult'
    assert [t for t, _ in ros_tap.select_topics(sec, ['common', 'new', 'legacy'])].count('/image_detection') == 1
    for n in ('zr10_camera', 'zr10_stream', 'detector_node', 'gimbal_pointing', 'detection_adapter'):
        assert n in sec['nodes']
    # never raw or annotated video
    every = [t for part in ('common', 'new', 'legacy') for t, _ in sec[part]]
    for banned in ('/zr10/image_raw', '/detection/annotated', '/image_raw'):
        assert banned not in every
    # presence nodes: stack_check's TEAM_NODES + mavros
    for n in ('waypoint_manager', 'main_controller', 'siyi', 'mission_camera_trigger', 'new_od',
              'gps_mavros_service_node', 'mavros'):
        assert n in sec['nodes']


def test_topic_file_errors_name_the_line():
    with pytest.raises(ValueError, match='line 2'):
        ros_tap.parse_topic_file('[common]\n/a std_msgs/msg/String extra\n')
    with pytest.raises(ValueError, match='must start with'):
        ros_tap.parse_topic_file('a std_msgs/msg/String\n')
    with pytest.raises(ValueError, match='pkg/msg/Name'):
        ros_tap.parse_topic_file('/a String\n')
    with pytest.raises(ValueError, match='unknown section'):
        ros_tap.parse_topic_file('[images]\n')
    with pytest.raises(ValueError, match='not a ROS node name'):
        ros_tap.parse_topic_file('[nodes]\nmain-controller\n')


def test_select_topics_keeps_file_order_and_unique():
    sec = ros_tap.parse_topic_file('/a std_msgs/msg/String\n[new]\n/b std_msgs/msg/Int8\n'
                                   '[legacy]\n/a std_msgs/msg/String\n/c std_msgs/msg/Float64\n')
    assert ros_tap.select_topics(sec, ['common', 'legacy']) == [('/a', 'std_msgs/msg/String'),
                                                                ('/c', 'std_msgs/msg/Float64')]
    assert [t for t, _ in ros_tap.select_topics(sec, ['common', 'new', 'legacy'])] == ['/a', '/b', '/c']
    with pytest.raises(ValueError):
        ros_tap.select_topics(sec, ['images'])


# ---- QoS ------------------------------------------------------------------------------------------------------------

def test_qos_matches_every_publisher():
    rel = {'reliability': 'reliable', 'durability': 'volatile', 'depth': 0}
    be = {'reliability': 'best_effort', 'durability': 'volatile', 'depth': 5}
    tl = {'reliability': 'reliable', 'durability': 'transient_local', 'depth': 1}
    assert ros_tap.choose_qos([rel]) == {'reliability': 'reliable', 'durability': 'volatile', 'depth': 10}
    assert ros_tap.choose_qos([rel, be])['reliability'] == 'best_effort'     # a reliable reader misses best effort
    assert ros_tap.choose_qos([tl])['durability'] == 'transient_local'
    assert ros_tap.choose_qos([tl, rel])['durability'] == 'volatile'         # transient reader misses volatile
    assert ros_tap.choose_qos([be])['depth'] == 5
    assert ros_tap.choose_qos([])['reliability'] == 'reliable'


# ---- presence -------------------------------------------------------------------------------------------------------

def test_presence_added_at_once_removed_after_two_missing_lists():
    p = ros_tap.PresenceTracker()
    assert p.update(['main_controller', 'new_od']) == (['main_controller', 'new_od'], [])
    assert p.update(['main_controller']) == ([], [])                 # one list without new_od: kept
    assert p.update(['main_controller', 'new_od']) == ([], [])       # back: the count starts again
    assert p.update(['main_controller']) == ([], [])
    assert p.update(['main_controller']) == ([], ['new_od'])         # two in a row: removed
    assert p.update([]) == ([], [])
    assert p.update([]) == ([], ['main_controller'])


def test_presence_untouched_while_no_lists_arrive():
    """A link drop sends no NODES lists at all: nothing is removed (no false 'main_controller crashed')."""
    p = ros_tap.PresenceTracker()
    p.update(['main_controller'])
    # (no update calls during the drop)
    assert 'main_controller' in p.present
    assert p.update(['main_controller']) == ([], [])


def test_filter_nodes():
    graph = [('main_controller', '/'), ('astra_tap_send', '/'), ('_ros2cli_123', '/'), ('mavros', '/mavros'),
             ('main_controller', '/')]
    assert ros_tap.filter_nodes(graph, ['main_controller', 'mavros', 'astra_tap_send'], own='astra_tap_send') == \
        [['main_controller', '/'], ['mavros', '/mavros']]


# ---- addresses, status, command line --------------------------------------------------------------------------------

def test_parse_endpoint():
    assert ros_tap.parse_endpoint('unix:/run/astra_hil/tap.sock') == ('unix', '/run/astra_hil/tap.sock')
    assert ros_tap.parse_endpoint('tcp:127.0.0.1:7461') == ('tcp', '127.0.0.1', 7461)
    for bad in ('tcp:127.0.0.1', 'tcp::7461', 'udp:1.2.3.4:5', 'unix:', '/x.sock', 'tcp:h:99999'):
        with pytest.raises(ValueError):
            ros_tap.parse_endpoint(bad)


def test_write_status_atomic(tmp_path):
    f = tmp_path / 'state' / 'tap.env'
    ros_tap.write_status(str(f), {'CONNECTED': 1, 'MSGS': 5})
    assert f.read_text().splitlines() == ['CONNECTED=1', 'MSGS=5']
    assert not [p for p in f.parent.iterdir() if '.tmp' in p.name]


def test_typo_on_command_line_exits_2():
    assert ros_tap.main(['nonsense']) == 2
    assert ros_tap.main(['send', '--bogus']) == 2


def test_bad_topic_file_exits_4(tmp_path, capsys):
    bad = tmp_path / 'topics.txt'
    bad.write_text('/a not-a-type\n')
    with pytest.raises(SystemExit) as e:
        ros_tap.main(['send', '--topics', str(bad), '--listen', f'unix:{tmp_path}/t.sock'])
    assert e.value.code == 4
    err = capsys.readouterr().err
    assert 'ERROR' in err and 'Likely cause' in err and 'Fix' in err
