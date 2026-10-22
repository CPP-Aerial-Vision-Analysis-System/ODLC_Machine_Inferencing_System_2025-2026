"""
sim_demo/test/test_photos.py -- PURPOSE: unit tests of photos.py: the "<lat> , <lon>.jpg" file
name contract (same cases as the team's detection/test/test_latlon_filename.py), photo folder
watching (new files, half-written files, preexisting files), JPEG size reading, and the
workspace search that finds <ws>/src/video_cam/mapping_photos.
"""
import os
import struct
import time

import pytest

from sim_demo.photos import (PhotoWatcher, find_ros2_ws_src, gps_photo_name, is_photo_name,
                             jpeg_size, list_photos, parse_latlon_from_name, resolve_photo_dir)


@pytest.mark.parametrize('name,expected', [
    ('38.315339 , -76.548108.jpg', (38.315339, -76.548108)),
    ('-33.123456 , 151.654321.jpg', (-33.123456, 151.654321)),
    ('IMG_0042.jpg', (None, None)),                 # SD-card name: no GPS fix
    ('38.31,-76.54.jpg', (None, None)),             # wrong delimiter
    ('abc , def.jpg', (None, None)),                # not numbers
    ('/some/folder/38.000001 , -76.000002.jpg', (38.000001, -76.000002)),
])
def test_parse_latlon_like_geotag(name, expected):
    assert parse_latlon_from_name(name) == expected


def test_round_trip_with_video_cam_format():
    name = gps_photo_name(38.3153391, -76.5481079)
    assert name == '38.315339 , -76.548108.jpg'
    assert parse_latlon_from_name(name) == (38.315339, -76.548108)


def test_is_photo_name():
    assert is_photo_name('38.1 , -76.2.jpg')
    assert is_photo_name('x.PNG')
    assert not is_photo_name('38.1 , -76.2.tmp.jpg')     # half-written (video_cam atomic write)
    assert not is_photo_name('notes.txt')
    assert not is_photo_name('.hidden.jpg')


def _touch(path, data=b'x' * 2000, age=0.0):
    with open(path, 'wb') as f:
        f.write(data)
    if age:
        t = time.time() - age
        os.utime(path, (t, t))


def test_watcher_reports_finished_files_once(tmp_path):
    folder = str(tmp_path)
    _touch(os.path.join(folder, 'old.jpg'), age=100)
    w = PhotoWatcher(folder, min_age_s=0.3)
    first = w.poll()
    assert [e.name for e in first] == ['old.jpg'] and first[0].preexisting
    _touch(os.path.join(folder, '38.5 , -76.5.jpg'), age=1.0)
    _touch(os.path.join(folder, '38.6 , -76.6.tmp.jpg'), age=1.0)
    assert w.poll() == []                     # first sighting: size not yet confirmed stable
    ev = w.poll()
    assert [e.name for e in ev] == ['38.5 , -76.5.jpg']
    assert (ev[0].lat, ev[0].lon) == (38.5, -76.5) and not ev[0].preexisting
    assert w.poll() == []                     # reported only once


def test_watcher_waits_for_young_files(tmp_path):
    folder = str(tmp_path)
    w = PhotoWatcher(folder, min_age_s=5.0)
    w.poll()
    _touch(os.path.join(folder, 'a.jpg'))
    assert w.poll() == [] and w.poll() == []   # too young
    assert [e.name for e in w.poll(now=time.time() + 10)] == ['a.jpg']


def test_watcher_missing_folder_is_fine(tmp_path):
    w = PhotoWatcher(str(tmp_path / 'does_not_exist'))
    assert w.poll() == []
    assert list_photos(str(tmp_path / 'nope')) == []


def _write_fake_jpeg(path, w, h):
    # SOI, APP0 (short), SOF0 with height/width, EOI: enough for the header reader.
    sof = b'\xff\xc0' + struct.pack('>HBHHB', 11, 8, h, w, 1) + b'\x01\x11\x00'
    app0 = b'\xff\xe0' + struct.pack('>H', 16) + b'JFIF\x00' + b'\x01\x01\x00\x00\x01\x00\x01\x00\x00'
    with open(path, 'wb') as f:
        f.write(b'\xff\xd8' + app0 + sof + b'\xff\xd9')


def test_jpeg_size_header_reader(tmp_path):
    p = str(tmp_path / 'fake.jpg')
    _write_fake_jpeg(p, 2560, 1440)
    assert jpeg_size(p) == (2560, 1440)
    assert jpeg_size(str(tmp_path / 'missing.jpg')) is None


def test_find_ros2_ws_src(tmp_path):
    ws = tmp_path / 'ws'
    (ws / 'install' / 'video_cam' / 'lib').mkdir(parents=True)
    (ws / 'src').mkdir()
    assert find_ros2_ws_src(str(ws / 'install' / 'video_cam' / 'lib')) == str(ws / 'src')
    assert find_ros2_ws_src(str(tmp_path)) is None


def test_resolve_photo_dir_override_and_env(tmp_path, monkeypatch):
    assert resolve_photo_dir(str(tmp_path))[0] == str(tmp_path)
    monkeypatch.setenv('ASTRA_PHOTO_DIR', str(tmp_path / 'env'))
    path, how = resolve_photo_dir('')
    assert path == str(tmp_path / 'env') and 'ASTRA_PHOTO_DIR' in how
