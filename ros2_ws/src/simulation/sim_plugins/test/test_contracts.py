"""
sim_plugins/test/test_contracts.py -- PURPOSE: keep the written slot contracts (contracts.py) honest:
  * every contract is complete and its message / service types are well-formed;
  * the detector and mission-brain contracts match the TEAM'S CODE (the topic names, QoS and services are
    looked up in detection/new_od.py and main/src/main_controller.py, and every ImageResult field a rule
    mentions exists in interfaces/msg/ImageResult.msg), so a change in the team code that breaks the contract
    shows up here;
  * docs/simulation/PLUGINS.md lists every topic and service of every contract.
"""

import re
from pathlib import Path

import pytest

from sim_plugins import contracts

SRC = Path(__file__).resolve().parents[3]           # ros2_ws/src
REPO = SRC.parent.parent
TYPE_RE = re.compile(r'^[a-z_]+/(msg|srv)/[A-Z][A-Za-z0-9]+$')


def read(rel):
    path = SRC / rel
    if not path.is_file():
        pytest.skip(f'{rel} is not in this checkout')
    return path.read_text(encoding='utf-8')


def test_every_contract_is_complete():
    assert set(contracts.CONTRACTS) == {'autopilot', 'camera', 'detector', 'mission_brain', 'world', 'extras',
                                        'network'}
    for c in contracts.CONTRACTS.values():
        assert c.title and c.default and c.alternatives and c.summary
        for t in c.topics:
            assert t.name.startswith('/') and TYPE_RE.match(t.type) and '/msg/' in t.type, t
            assert t.role in ('publish', 'subscribe') and t.qos in ('reliable', 'best_effort', 'any')
        for s in c.services:
            assert s.name.startswith('/') and TYPE_RE.match(s.type) and '/srv/' in s.type, s
            assert s.role in ('server', 'client')


def test_detector_contract_matches_new_od():
    code = read('detection/detection/new_od.py')
    c = contracts.contract('detector')
    main = [t for t in c.topics if t.required][0]
    assert main.name == '/image_detection' and main.type == 'interfaces/msg/ImageResult' and main.qos == 'reliable'
    assert "create_publisher(ImageResult, '/image_detection', qos)" in code
    assert 'QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)' in code
    assert "'sahi/get_health'" in code and "'sahi/get_statistics'" in code
    assert "'/sahi_detection_info'" in code
    assert c.node_names == ('new_od',)


def test_detector_rules_name_real_imageresult_fields():
    msg = read('interfaces/msg/ImageResult.msg')
    fields = set(re.findall(r'^\s*[\w/\[\]]+\s+(\w+)\s*$', msg, re.M))
    for name in ('image_name', 'method', 'latitude', 'longitude', 'num_detections', 'timestamp', 'classes',
                 'confidences', 'areas', 'descriptions', 'detections', 'header'):
        assert name in fields, name
    rules = ' '.join(contracts.contract('detector').rules)
    for name in ('image_name', 'method', 'latitude/longitude', 'num_detections'):
        assert name in rules


def test_mission_brain_contract_matches_main_controller():
    code = read('main/src/main_controller.py')
    c = contracts.contract('mission_brain')
    for s in c.services:
        if s.required:
            assert f'"{s.name}"' in code or f"'{s.name}'" in code, s.name
    for t in c.topics:
        if t.required:
            assert f'"{t.name}"' in code or f"'{t.name}'" in code, t.name
    assert 'detection_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)' in code


def test_photo_folder_matches_the_team_code():
    assert contracts.PHOTO_DIR.endswith('video_cam/mapping_photos')
    assert '"video_cam", "mapping_photos"' in read('detection/detection/new_od.py')


def test_render_text_and_markdown():
    for c in contracts.CONTRACTS.values():
        assert contracts.render_text(c).startswith(f'SLOT {c.slot}')
        md = contracts.render_markdown(c)
        assert md.startswith('**Default:**')
    with pytest.raises(KeyError, match='known'):
        contracts.contract('detecter')


def test_plugins_doc_lists_every_contract_item():
    doc = REPO / 'docs' / 'simulation' / 'PLUGINS.md'
    if not doc.is_file():
        pytest.skip('docs/simulation/PLUGINS.md not written yet')
    text = doc.read_text(encoding='utf-8')
    for c in contracts.CONTRACTS.values():
        for item in list(c.topics) + list(c.services):
            assert f'`{item.name}`' in text, f'{c.slot}: {item.name} missing from PLUGINS.md'
