"""
lattice_bridge/test/test_target_locator.py -- PURPOSE: prove that the reporter puts a target at the SAME place
main_controller would aim at, because it calls the SAME function (main/src/target_geolocation.py) on the same
files, and that every kind of bad photo or detection is skipped with a reason instead of raising:

  * finding and loading the team's file (installed package, source checkout, missing, broken, changed shape);
  * extract_job(): plain numbers out of a message, whatever shape it has;
  * TargetLocator on real files in a temporary photo folder: good photo, no sidecar, unreadable sidecar, NaN
    everywhere, strange photo names, the settings that change the aim;
  * PARITY: for 400 random photos the answer equals the answer of the REAL MainController.locate_detection
    (needs ROS and cv_bridge; skipped without them);
  * the photo folder is found exactly like main_controller finds it.
"""

import json
import math
import os
import struct
import sys
import types
from pathlib import Path

import pytest

from lattice_bridge import target_locator as T
from lattice_bridge.geo import haversine_m
from lattice_bridge.reporter_logic import AimSettings
from support import rng

W, H = 2560, 1440
GOOD_LAT, GOOD_LON = 36.2150000, -96.0068000


@pytest.fixture(scope='module')
def team_path():
    path, how = T.find_team_module()
    if path is None:
        pytest.skip(f'the team file target_geolocation.py cannot be found here: {how}')
    return path


@pytest.fixture(scope='module')
def geo(team_path):
    return T.load_team_module(team_path)


def write_photo(folder, name, meta=None, size=(W, H)):
    """A tiny JPEG header with the given size (all the reader needs), plus an optional sidecar."""
    sof = b'\xff\xc0' + struct.pack('>HBHH', 11, 8, size[1], size[0]) + b'\x03\x01\x22\x00'
    (Path(folder) / name).write_bytes(b'\xff\xd8' + sof + b'\xff\xd9')
    side = Path(folder) / (os.path.splitext(name)[0] + '.json')
    if meta is not None:
        side.write_text(json.dumps(meta))
    elif side.exists():
        side.unlink()                                                  # no stale sidecar of an earlier case


def sidecar(**over):
    """A good sidecar: 30 m up, heading north, 81 degree view, camera straight down."""
    meta = {'format': 'astra.photo_metadata.v1', 'rel_alt_m': 30.0, 'heading_deg': 0.0,
            'position': {'lat': GOOD_LAT, 'lon': GOOD_LON},
            'camera': {'hfov_deg': 81.0, 'zoom_x': 1.0, 'zoom_changing': False, 'image_top_direction_deg': 0.0},
            'gimbal': {'yaw_deg': 0.0, 'pitch_deg': -90.0, 'roll_deg': 0.0}}
    meta.update(over)
    return meta


GOOD_NAME = f'{GOOD_LAT:.6f} , {GOOD_LON:.6f}.jpg'


def make_message(name, boxes, lat=GOOD_LAT, lon=GOOD_LON):
    """An ImageResult look-alike: boxes = [(class_id, score, u, v)]."""
    detections = []
    for class_id, score, u, v in boxes:
        hypothesis = types.SimpleNamespace(class_id=class_id, score=score)
        detections.append(types.SimpleNamespace(
            bbox=types.SimpleNamespace(center=types.SimpleNamespace(position=types.SimpleNamespace(x=u, y=v))),
            results=[types.SimpleNamespace(hypothesis=hypothesis)]))
    return types.SimpleNamespace(image_name=name, latitude=lat, longitude=lon,
                                 detections=types.SimpleNamespace(detections=detections))


def locate_one(geo, folder, name, u=W / 2 + 100, v=H / 2, lat=GOOD_LAT, lon=GOOD_LON, aim=None, allow=False):
    """The whole path for one detection: message -> job -> photo -> position."""
    locator = T.TargetLocator(geo, str(folder), aim or AimSettings(), allow)
    job, _ = T.extract_job(make_message(name, [('0', 0.9, u, v)], lat, lon))
    return locator.locate(locator.open_photo(job), job.detections[0].u, job.detections[0].v)


# ------------------------------------------------------------------------------ finding the team's file
def make_prefix(tmp_path, with_marker=True, with_file=True):
    prefix = tmp_path / 'install' / 'main'
    (prefix / 'lib' / 'main').mkdir(parents=True)
    if with_file:
        (prefix / 'lib' / 'main' / 'target_geolocation.py').write_text('x = 1\n')
    if with_marker:
        marker = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
        marker.mkdir(parents=True)
        (marker / 'main').write_text('')
    return prefix


def test_the_installed_package_is_found_through_ament_prefix_path(tmp_path):
    prefix = make_prefix(tmp_path)
    other = tmp_path / 'install' / 'other'
    other.mkdir(parents=True)
    path, how = T.find_team_module({'AMENT_PREFIX_PATH': os.pathsep.join(['', str(other), str(prefix)])},
                                   search_from=str(tmp_path))
    assert path == str(prefix / 'lib' / 'main' / 'target_geolocation.py') and 'installed package main' in how


@pytest.mark.parametrize('marker, file', [(False, True), (True, False)])
def test_a_prefix_without_the_marker_or_the_file_is_not_main(tmp_path, marker, file):
    prefix = make_prefix(tmp_path, with_marker=marker, with_file=file)
    path, how = T.find_team_module({'AMENT_PREFIX_PATH': str(prefix)}, search_from=str(tmp_path))
    assert path is None and 'not installed' in how and 'source' in how


def test_a_source_checkout_is_found_by_walking_up(tmp_path):
    src = tmp_path / 'ros2_ws' / 'src'
    (src / 'main' / 'src').mkdir(parents=True)
    (src / 'main' / 'src' / 'target_geolocation.py').write_text('x = 1\n')
    deep = src / 'lattice_bridge' / 'lattice_bridge'
    deep.mkdir(parents=True)
    path, how = T.find_team_module({}, search_from=str(deep / 'target_locator.py'))
    assert path == str(src / 'main' / 'src' / 'target_geolocation.py') and 'source tree' in how


def test_the_real_file_is_found_and_loads(team_path, geo):
    assert os.path.basename(team_path) == 'target_geolocation.py'
    for name in T.REQUIRED_NAMES:
        assert hasattr(geo, name), name
    assert T.load_team_module(team_path) is geo                         # cached


def test_loading_a_broken_or_changed_file_is_a_clear_error(tmp_path):
    broken = tmp_path / 'broken_a.py'
    broken.write_text('this is not python (\n')
    with pytest.raises(T.TeamModuleError, match='cannot load'):
        T.load_team_module(str(broken))
    boom = tmp_path / 'broken_b.py'
    boom.write_text('raise RuntimeError("boom")\n')
    with pytest.raises(T.TeamModuleError, match='boom'):
        T.load_team_module(str(boom))
    short = tmp_path / 'short.py'
    short.write_text('def aim_at_box():\n    pass\n')
    with pytest.raises(T.TeamModuleError) as caught:
        T.load_team_module(str(short))
    assert 'photo_position' in str(caught.value) and 'team code changed' in str(caught.value)
    with pytest.raises(T.TeamModuleError, match='not a Python file'):
        T.load_team_module(str(tmp_path / 'file.txt'))


def test_the_team_module_is_never_copied():
    """The maths must stay in main/src/target_geolocation.py: the reporter's files have no copy of its formulas."""
    for name in ('target_locator.py', 'reporter_logic.py', 'reporter_node.py'):
        text = (Path(T.__file__).parent / name).read_text(encoding='utf-8')
        for formula in ('math.tan(', 'math.atan', 'def pixel_to_ground_offset', 'def aim_at_box',
                        'def offset_to_latlon', 'METRES_PER_DEGREE'):
            assert formula not in text, f'{name} contains {formula}'


# ------------------------------------------------------------------------------------------ small helpers
@pytest.mark.parametrize('name, expected', [
    ('36.214935 , -96.006203.jpg', (36.214935, -96.006203)),
    ('/x/y/36.2 , -96.0.jpg', (36.2, -96.0)),
    ('IMG_0079.jpg', (None, None)), ('36.2,-96.0.jpg', (None, None)), ('a , b.jpg', (None, None)),
    ('', (None, None)), (None, (None, None)),
])
def test_the_gps_name_rule_is_main_controllers(name, expected):
    assert T.latlon_from_photo_name(name) == expected


def test_nonfinite_names_parse_like_main_controller_and_are_refused_later(geo):
    lat, lon = T.latlon_from_photo_name('nan , inf.jpg')
    assert math.isnan(lat) and math.isinf(lon)                           # same as main_controller ...
    assert geo.photo_position(None, lat, lon) is None                    # ... and refused by the team code


@pytest.mark.parametrize('name, ok', [
    ('36.2 , -96.0.jpg', True), ('IMG_1.jpg', True), ('a' * 255, True), ('a' * 256, False), ('', False),
    ('.', False), ('..', False), ('../x.jpg', False), ('dir/x.jpg', False), ('dir\\x.jpg', False),
    ('x\x00.jpg', False), (None, False), (5, False), (b'x.jpg', False),
])
def test_only_plain_file_names_are_accepted(name, ok):
    assert T.is_plain_file_name(name) is ok


# ---------------------------------------------------------------------------------------------- extract_job
def test_extract_job_copies_plain_numbers():
    msg = make_message('a.jpg', [('0', 0.9, 100.0, 200.0), ('1', 0.4, 300.5, 400.5)], 36.5, -96.5)
    job, malformed = T.extract_job(msg)
    assert malformed == 0 and job.image_name == 'a.jpg'
    assert (job.msg_lat, job.msg_lon) == (36.5, -96.5)
    assert job.detections == (T.RawDetection('0', 0.9, 100.0, 200.0), T.RawDetection('1', 0.4, 300.5, 400.5))


def test_extract_job_keeps_nonfinite_numbers_so_they_can_be_reported_by_name():
    job, malformed = T.extract_job(make_message('a.jpg', [('0', float('nan'), float('inf'), 1.0)]))
    assert malformed == 0 and math.isnan(job.detections[0].score) and math.isinf(job.detections[0].u)


@pytest.mark.parametrize('msg', [
    None, 5, 'text', types.SimpleNamespace(), types.SimpleNamespace(image_name=5, detections=None),
    types.SimpleNamespace(image_name='a.jpg', detections=types.SimpleNamespace(detections=5)),
    types.SimpleNamespace(image_name='a.jpg', detections=types.SimpleNamespace(detections=[object()])),
    types.SimpleNamespace(image_name='a.jpg', latitude='north', longitude=None,
                          detections=types.SimpleNamespace(detections=[
                              types.SimpleNamespace(bbox=None, results=[])])),
])
def test_extract_job_never_raises_on_a_strange_message(msg):
    job, malformed = T.extract_job(msg)
    assert isinstance(job, T.PhotoJob) and isinstance(job.detections, tuple) and malformed >= 0
    assert isinstance(job.image_name, str)


def test_extract_job_counts_unreadable_boxes_and_bounds_the_work():
    good = make_message('a.jpg', [('0', 0.9, 1.0, 2.0)]).detections.detections[0]
    junk = types.SimpleNamespace(bbox=types.SimpleNamespace(center=None), results=[1])
    msg = types.SimpleNamespace(image_name='a.jpg', latitude=1.0, longitude=2.0,
                                detections=types.SimpleNamespace(detections=[junk, good, junk]))
    job, malformed = T.extract_job(msg)
    assert malformed == 2 and len(job.detections) == 1
    many = make_message('b.jpg', [('0', 0.5, 1.0, 2.0)] * 500)
    assert len(T.extract_job(many)[0].detections) == T.MAX_DETECTIONS_PER_PHOTO
    long_name = T.extract_job(make_message('x' * 5000, []))[0].image_name
    assert len(long_name) == T.MAX_NAME_LENGTH + 1 and not T.is_plain_file_name(long_name)


# ------------------------------------------------------------------------------------- locating, real files
def test_a_good_photo_is_projected_with_the_team_function(tmp_path, geo):
    write_photo(tmp_path, GOOD_NAME, sidecar())
    result = locate_one(geo, tmp_path, GOOD_NAME, u=W / 2 + 300)
    assert result.ok and result.method == 'box' and 'box centre' in result.note
    oracle = geo.aim_at_box(sidecar(), GOOD_LAT, GOOD_LON, W / 2 + 300, H / 2, (W, H))
    assert (result.lat, result.lon) == (oracle.lat, oracle.lon) and oracle.method == 'box'
    # 300 px right of the centre at 30 m with an 81 degree view is about 6 m east of the drone
    east = haversine_m(GOOD_LAT, GOOD_LON, result.lat, result.lon)
    assert 4.0 < east < 8.0 and result.lon > GOOD_LON                     # heading 0: right of centre is due east


def test_the_target_is_never_the_drone_position_for_a_box_away_from_the_centre(tmp_path, geo):
    write_photo(tmp_path, GOOD_NAME, sidecar())
    result = locate_one(geo, tmp_path, GOOD_NAME, u=W / 2 + 600, v=H / 2 - 300)
    assert result.ok and haversine_m(GOOD_LAT, GOOD_LON, result.lat, result.lon) > 10.0


@pytest.mark.parametrize('u, v', [(float('nan'), 100.0), (100.0, float('inf')), (float('-inf'), float('nan'))])
def test_a_box_centre_that_is_not_a_number_is_skipped_by_name(tmp_path, geo, u, v):
    write_photo(tmp_path, GOOD_NAME, sidecar())
    result = locate_one(geo, tmp_path, GOOD_NAME, u=u, v=v)
    assert not result.ok and result.skip_key == T.SKIP_BAD_BOX and 'not a finite number' in result.skip_text


def test_no_sidecar_means_the_drone_position_and_is_skipped_unless_allowed(tmp_path, geo):
    write_photo(tmp_path, GOOD_NAME)                                      # photo, no sidecar
    skipped = locate_one(geo, tmp_path, GOOD_NAME)
    assert not skipped.ok and skipped.skip_key == T.SKIP_PHOTO_FALLBACK
    assert 'drone position' in skipped.skip_text and 'no sidecar file' in skipped.skip_text
    allowed = locate_one(geo, tmp_path, GOOD_NAME, allow=True)
    assert allowed.ok and allowed.method == 'photo' and (allowed.lat, allowed.lon) == (GOOD_LAT, GOOD_LON)


@pytest.mark.parametrize('text', ['{not json', '[1, 2]', '{"format": "something.else", "rel_alt_m": 30}', ''])
def test_an_unreadable_or_foreign_sidecar_counts_as_none(tmp_path, geo, text):
    write_photo(tmp_path, GOOD_NAME)
    (tmp_path / (os.path.splitext(GOOD_NAME)[0] + '.json')).write_text(text)
    result = locate_one(geo, tmp_path, GOOD_NAME)
    assert not result.ok and result.skip_key == T.SKIP_PHOTO_FALLBACK
    assert 'unreadable or in another format' in result.skip_text


@pytest.mark.parametrize('over', [
    {'rel_alt_m': float('nan')}, {'heading_deg': float('inf')}, {'rel_alt_m': 1.0}, {'rel_alt_m': None},
    {'camera': {'hfov_deg': 81.0, 'zoom_x': 1.0, 'zoom_changing': True, 'image_top_direction_deg': 0.0}},
])
def test_a_sidecar_that_cannot_be_used_falls_back_and_is_skipped(tmp_path, geo, over):
    write_photo(tmp_path, GOOD_NAME, sidecar(**over))
    result = locate_one(geo, tmp_path, GOOD_NAME)
    assert not result.ok and result.skip_key == T.SKIP_PHOTO_FALLBACK


@pytest.mark.parametrize('name', ['nan , nan.jpg', 'inf , -96.006800.jpg', '95.000000 , -96.006800.jpg',
                                  '0.000000 , 0.000000.jpg'])
def test_a_photo_without_a_usable_gps_position_is_skipped(tmp_path, geo, name):
    write_photo(tmp_path, name)
    result = locate_one(geo, tmp_path, name, lat=float('nan'), lon=float('nan'))
    assert not result.ok and result.skip_key == T.SKIP_NO_POSITION and 'no usable GPS position' in result.skip_text


def test_the_sidecar_position_beats_the_file_name(tmp_path, geo):
    write_photo(tmp_path, 'IMG_0001.jpg', sidecar())                       # no GPS in the name at all
    result = locate_one(geo, tmp_path, 'IMG_0001.jpg', lat=float('nan'), lon=float('nan'))
    assert result.ok and result.method == 'box'


def test_the_message_position_is_used_when_the_name_has_none(tmp_path, geo):
    write_photo(tmp_path, 'IMG_0002.jpg')
    result = locate_one(geo, tmp_path, 'IMG_0002.jpg', lat=36.25, lon=-96.1, allow=True)
    assert result.ok and result.method == 'photo' and result.lat == pytest.approx(36.25)


@pytest.mark.parametrize('name', ['../x.jpg', '/etc/passwd', 'a/b.jpg', 'x' * 400, 'bad\x00name.jpg', '..'])
def test_a_photo_name_that_is_not_a_plain_file_name_is_skipped_without_touching_the_disk(tmp_path, geo, name,
                                                                                          monkeypatch):
    touched = []
    monkeypatch.setattr(geo, 'read_photo_sidecar', lambda path: touched.append(path))
    result = locate_one(geo, tmp_path, name)
    assert not result.ok and result.skip_key == T.SKIP_BAD_NAME and touched == []


def test_an_empty_photo_name_uses_the_message_position_like_main_controller(tmp_path, geo):
    skipped = locate_one(geo, tmp_path, '', lat=36.25, lon=-96.1)
    assert not skipped.ok and skipped.skip_key == T.SKIP_PHOTO_FALLBACK
    allowed = locate_one(geo, tmp_path, '', lat=36.25, lon=-96.1, allow=True)
    assert allowed.ok and allowed.method == 'photo'


def test_box_aiming_off_gives_the_photo_position_and_is_skipped_unless_allowed(tmp_path, geo):
    write_photo(tmp_path, GOOD_NAME, sidecar())
    aim = AimSettings(aim_at_detection_box=False)
    assert locate_one(geo, tmp_path, GOOD_NAME, aim=aim).skip_key == T.SKIP_PHOTO_FALLBACK
    result = locate_one(geo, tmp_path, GOOD_NAME, aim=aim, allow=True)
    assert result.ok and result.method == 'photo' and 'aim_at_detection_box is false' in result.note


def test_the_aim_settings_change_the_answer_like_they_do_for_main_controller(tmp_path, geo):
    write_photo(tmp_path, GOOD_NAME, sidecar())
    base = locate_one(geo, tmp_path, GOOD_NAME, u=W / 2 + 600, v=H / 2)
    higher_ground = locate_one(geo, tmp_path, GOOD_NAME, u=W / 2 + 600, v=H / 2,
                               aim=AimSettings(target_ground_height_m=10.0))
    assert base.ok and higher_ground.ok and (base.lat, base.lon) != (higher_ground.lat, higher_ground.lon)
    assert haversine_m(GOOD_LAT, GOOD_LON, higher_ground.lat, higher_ground.lon) < \
        haversine_m(GOOD_LAT, GOOD_LON, base.lat, base.lon)               # a closer ground means a shorter reach
    tiny_limit = locate_one(geo, tmp_path, GOOD_NAME, u=W / 2 + 600, v=H / 2, aim=AimSettings(max_aim_offset_m=2.0))
    assert not tiny_limit.ok and tiny_limit.skip_key == T.SKIP_PHOTO_FALLBACK
    locator = T.TargetLocator(geo, str(tmp_path))
    assert locator.aim == AimSettings()
    locator.set_aim(AimSettings(target_ground_height_m=3.0))
    assert locator.aim.target_ground_height_m == 3.0


def test_the_photo_is_read_once_for_many_detections(tmp_path, geo, monkeypatch):
    write_photo(tmp_path, GOOD_NAME, sidecar())
    reads = []
    real = geo.read_photo_sidecar
    monkeypatch.setattr(geo, 'read_photo_sidecar', lambda path: reads.append(path) or real(path))
    locator = T.TargetLocator(geo, str(tmp_path))
    job, _ = T.extract_job(make_message(GOOD_NAME, [('0', 0.9, 100.0 * k, 50.0 * k) for k in range(1, 6)]))
    photo = locator.open_photo(job)
    results = [locator.locate(photo, d.u, d.v) for d in job.detections]
    assert len(reads) == 1 and all(r.ok for r in results)


# --------------------------------------------------------------------------------- the photo folder
def test_the_photo_folder_is_found_like_main_controller_finds_it(tmp_path):
    workspace = tmp_path / 'ros2_ws'
    lib = workspace / 'install' / 'main' / 'lib' / 'main'
    lib.mkdir(parents=True)
    (workspace / 'src').mkdir()
    assert T.default_photo_dir(str(lib / 'target_geolocation.py')) == str(workspace / 'src' / 'video_cam' / 'mapping_photos')
    lonely = tmp_path / 'x' / 'main' / 'src'
    lonely.mkdir(parents=True)
    fallback = T.default_photo_dir(str(lonely / 'target_geolocation.py')).replace('\\', '/')   # Windows uses backslashes
    assert fallback.endswith('video_cam/mapping_photos')


# ---------------------------------------------------------------------- PARITY with the real main_controller
def random_case(r, folder, index):
    """One random photo (files written to `folder`) and its detection box. Returns (name, lat, lon, u, v)."""
    lat, lon = GOOD_LAT + r.uniform(-0.002, 0.002), GOOD_LON + r.uniform(-0.002, 0.002)
    style = r.choice(['gps', 'gps', 'gps', 'gps', 'sd', 'nan'])
    name = {'gps': f'{lat:.6f} , {lon:.6f}.jpg', 'sd': f'IMG_{index:04d}.jpg', 'nan': 'nan , nan.jpg'}[style]
    msg_lat, msg_lon = (lat, lon) if r.random() < 0.7 else (float('nan'), float('nan'))
    mode = r.choice(['good'] * 8 + ['none', 'broken', 'alien', 'weird'])
    meta = None
    if mode in ('good', 'weird'):
        meta = sidecar(rel_alt_m=r.choice([30.0, 55.0, 100.0, 30.0, 55.0, 2.0, None, float('nan')]),
                       heading_deg=r.choice([0.0, 33.3, 90.0, 270.0, 359.9, 12.5, 180.0, None, float('inf')]),
                       position={'lat': lat + r.uniform(-1e-5, 1e-5), 'lon': lon + r.uniform(-1e-5, 1e-5)})
        meta['gimbal'] = r.choice([{'yaw_deg': r.uniform(-20, 20), 'pitch_deg': r.uniform(-92, -80), 'roll_deg': 0.0},
                                   {'yaw_deg': 0.0, 'pitch_deg': 12.0, 'roll_deg': 0.0}, None])
        meta['camera'] = {'hfov_deg': r.choice([81.0, 60.0, None]), 'zoom_x': r.choice([1.0, 2.0, None]),
                          'zoom_changing': r.random() < 0.1,
                          'image_top_direction_deg': r.choice([0.0, 90.0, 180.0, 270.0])}
        if mode == 'weird':
            meta['position'] = {'lat': float('nan'), 'lon': r.choice([None, 'x', float('nan')])}
    elif mode == 'alien':
        meta = {'format': 'other.format', 'rel_alt_m': 30.0}
    size = r.choice([(W, H), (3840, 2160), (1280, 720)])
    write_photo(folder, name, meta, size)
    if mode == 'broken':
        (Path(folder) / (os.path.splitext(name)[0] + '.json')).write_text('{broken')
    u = r.choice([r.uniform(0, size[0]) for _ in range(12)] + [float('nan')])
    v = r.choice([r.uniform(0, size[1]) for _ in range(12)] + [float('inf')])
    return name, msg_lat, msg_lon, u, v


@pytest.fixture(scope='module')
def main_controller_module(team_path):
    pytest.importorskip('rclpy', reason='ROS 2 (rclpy) is not sourced')
    pytest.importorskip('interfaces.msg', reason='the interfaces package is not built/sourced')
    pytest.importorskip('mavros_msgs.msg', reason='mavros_msgs is not installed')
    pytest.importorskip('cv_bridge', reason='cv_bridge is not installed')
    folder = os.path.dirname(team_path)
    if not os.path.isfile(os.path.join(folder, 'main_controller.py')):
        pytest.skip('main_controller.py is not next to target_geolocation.py')
    sys.path.insert(0, folder)
    try:
        import main_controller
    finally:
        sys.path.remove(folder)
    return main_controller


def stand_in(mc, folder, aim):
    """Exactly the state MainController.locate_detection uses, without creating a node."""
    c = types.SimpleNamespace()
    c.camera_feed_path = str(folder)
    c.aim_at_box = aim.aim_at_detection_box
    c.default_pitch, c.default_hfov = aim.default_camera_pitch_deg, aim.default_camera_hfov_deg
    c.ground_height, c.max_aim_offset = aim.target_ground_height_m, aim.max_aim_offset_m
    c._latlon_from_name = mc.MainController._latlon_from_name
    c.locate_detection = types.MethodType(mc.MainController.locate_detection, c)
    return c


@pytest.mark.parametrize('aim', [
    AimSettings(), AimSettings(target_ground_height_m=4.5), AimSettings(aim_at_detection_box=False),
    AimSettings(default_camera_pitch_deg=-85.0, default_camera_hfov_deg=70.0, max_aim_offset_m=40.0),
], ids=['defaults', 'ground 4.5 m', 'no box aiming', 'other pitch fov limit'])
def test_the_answer_equals_main_controllers_own_answer_for_random_photos(tmp_path, geo, main_controller_module, aim):
    from interfaces.msg import ImageResult
    from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
    r = rng(2026)
    reference = stand_in(main_controller_module, tmp_path, aim)
    locator = T.TargetLocator(geo, str(tmp_path), aim, allow_photo_fallback=True)
    compared = {'box': 0, 'photo': 0, 'none': 0}
    for index in range(400):
        name, msg_lat, msg_lon, u, v = random_case(r, tmp_path, index)
        msg = ImageResult()
        msg.image_name = name
        msg.latitude, msg.longitude = float(msg_lat), float(msg_lon)
        detection = Detection2D()
        detection.bbox.center.position.x, detection.bbox.center.position.y = float(u), float(v)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = '0', 0.9
        detection.results.append(hypothesis)
        msg.detections.detections.append(detection)

        theirs, why = reference.locate_detection(msg, detection)
        job, _ = T.extract_job(msg)
        mine = locator.locate(locator.open_photo(job), job.detections[0].u, job.detections[0].v)
        if theirs is None:
            assert not mine.ok, f'case {index} {name!r}: main_controller ignores it ({why}) but we found {mine}'
            compared['none'] += 1
        else:
            assert mine.ok, f'case {index} {name!r}: main_controller aims at {theirs} but we skip: {mine.skip_text}'
            assert (mine.lat, mine.lon, mine.method) == (theirs.lat, theirs.lon, theirs.method), index
            assert mine.note == theirs.note
            compared[theirs.method] += 1
    # the random cases really cover all three outcomes (so the comparison above means something)
    if aim.aim_at_detection_box:
        assert compared['box'] > 25 and compared['photo'] > 20 and compared['none'] > 10, compared
    else:
        assert compared['box'] == 0 and compared['photo'] > 150 and compared['none'] > 10, compared


def test_the_default_photo_folder_equals_main_controllers_own_choice(team_path, main_controller_module):
    reference = types.SimpleNamespace()
    ours = T.default_photo_dir(team_path)
    theirs = main_controller_module.MainController._resolve_camera_feed_path(reference)
    assert ours == theirs
