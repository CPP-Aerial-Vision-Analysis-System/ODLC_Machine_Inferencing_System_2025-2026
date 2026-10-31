"""
sim_demo/test/test_pov_server.py -- PURPOSE: prove the camera feed for Mission Planner without ROS or Gazebo: ROS
pictures in every supported encoding become correct JPEGs (right colours, padded rows handled, big pictures shrunk,
corrupt ones refused), the web server answers the page, a single picture, the status line and a real multipart stream
with the right framing, a quiet camera does not kill the connection, too many viewers are turned away politely, a
taken port gives a message that says what to do, and the stream ends when the server stops.
It also proves the GIMBAL-ANGLE overlay: the angles are computed the way the SIYI camera reports them (checked against
hand-worked cases and against the SIYI emulator's own formulas), the text says plainly when data is missing or old,
the box is drawn where asked (top or bottom) without touching the rest of the picture, and with no overlay the
picture is exactly what it was before the overlay existed.
"""

import http.client
import io
import math
import random
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from sim_demo.pov_server import (BOUNDARY, GIMBAL_STALE_S, FrameStore, GimbalReadout, PovServer, draw_overlay,
                                 encode_frame, placeholder_jpeg, siyi_angles)


def pixels(jpeg):
    return np.asarray(Image.open(io.BytesIO(jpeg)).convert('RGB'))


def solid(width, height, rgb, encoding='rgb8', pad=0):
    """A solid-colour ROS image: (width, height, encoding, step, data)."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[encoding]
    row = bytes()
    if encoding in ('rgb8',):
        pixel = bytes(rgb)
    elif encoding == 'bgr8':
        pixel = bytes(rgb[::-1])
    elif encoding == 'rgba8':
        pixel = bytes(rgb) + b'\xff'
    elif encoding == 'bgra8':
        pixel = bytes(rgb[::-1]) + b'\xff'
    else:
        pixel = bytes([rgb[0]])
    row = pixel * width + b'\x00' * pad
    return width, height, encoding, width * channels + pad, row * height


# ------------------------------------------------------------------------------------------ pictures
@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8', 'rgba8', 'bgra8'])
def test_colours_come_out_right_whatever_the_encoding(encoding):
    jpeg = encode_frame(*solid(64, 48, (250, 10, 10), encoding), quality=95, max_width=0)
    assert jpeg[:2] == b'\xff\xd8' and jpeg[-2:] == b'\xff\xd9'
    centre = pixels(jpeg)[24, 32]
    assert centre[0] > 200 and centre[1] < 60 and centre[2] < 60                 # red stays red (not blue)


def test_a_grey_picture_stays_grey():
    jpeg = encode_frame(*solid(64, 48, (128, 0, 0), 'mono8'), max_width=0)
    r, g, b = pixels(jpeg)[10, 10]
    assert abs(int(r) - 128) < 8 and abs(int(r) - int(g)) < 3 and abs(int(r) - int(b)) < 3


def test_padded_rows_are_read_correctly():
    jpeg = encode_frame(*solid(50, 40, (10, 240, 10), 'rgb8', pad=14), max_width=0)
    image = pixels(jpeg)
    assert image.shape == (40, 50, 3) and image[20, 25, 1] > 200 and image[20, 49, 1] > 200


def test_a_wide_picture_is_shrunk_and_a_narrow_one_is_not():
    big = Image.open(io.BytesIO(encode_frame(*solid(1920, 1080, (1, 2, 3)), max_width=960)))
    assert big.size == (960, 540)
    small = Image.open(io.BytesIO(encode_frame(*solid(320, 200, (1, 2, 3)), max_width=960)))
    assert small.size == (320, 200)
    unlimited = Image.open(io.BytesIO(encode_frame(*solid(1920, 1080, (1, 2, 3)), max_width=0)))
    assert unlimited.size == (1920, 1080)


def test_lower_quality_means_a_smaller_picture():
    rng = np.random.default_rng(1)
    noise = rng.integers(0, 255, (120, 160, 3), dtype=np.uint8).tobytes()
    assert len(encode_frame(160, 120, 'rgb8', 480, noise, quality=20)) < len(encode_frame(160, 120, 'rgb8', 480, noise, quality=95))


@pytest.mark.parametrize('args', [
    (64, 48, '16UC1', 128, b'\x00' * 128 * 48),           # an encoding we do not support
    (64, 48, 'rgb8', 192, b'\x00' * 100),                  # data too short for the stated size
    (64, 48, 'rgb8', 100, b'\x00' * 100 * 48),             # a row shorter than the pixels in it
    (0, 48, 'rgb8', 0, b''), (64, 0, 'rgb8', 192, b''),    # no size
    (64.0, 48, 'rgb8', 192, b'\x00' * 192 * 48),           # not whole numbers
])
def test_a_corrupt_picture_is_refused_with_a_value_error(args):
    with pytest.raises(ValueError):
        encode_frame(*args)


def test_the_waiting_picture_is_a_real_jpeg():
    image = Image.open(io.BytesIO(placeholder_jpeg('waiting')))
    assert image.size == (640, 360) and image.format == 'JPEG'


# ------------------------------------------------------------------------------------------ gimbal angles
def quat_about(axis, angle_deg):
    """(x, y, z, w) quaternion of a turn by angle_deg about the world axis 'x', 'y' or 'z' (right-hand rule)."""
    half = math.radians(angle_deg) / 2.0
    s = math.sin(half)
    vector = {'x': (s, 0.0, 0.0), 'y': (0.0, s, 0.0), 'z': (0.0, 0.0, s)}[axis]
    return (*vector, math.cos(half))


def test_the_camera_straight_down_on_a_level_drone_reads_pitch_minus_90():
    yaw, pitch, roll = siyi_angles(0.0, 0.0, math.radians(-90))
    assert abs(pitch + 90) < 1e-6 and abs(yaw) < 1e-9 and abs(roll) < 1e-6


def test_the_stabiliser_keeps_the_camera_pointing_down_when_the_drone_tips_nose_down():
    # A turn about the drone's LEFT axis (+Y) by +10 deg puts the nose 10 deg down. The pitch motor then sits at
    # -80 deg and the camera still looks straight down: the reported pitch is -90, the joint says -80.
    _, pitch, _ = siyi_angles(0.0, 0.0, math.radians(-80), quat_about('y', 10))
    assert abs(pitch + 90) < 1e-6
    assert abs(siyi_angles(0.0, 0.0, math.radians(-80))[1] + 80) < 1e-6      # same joint, level drone: 80 deg down


def test_yaw_is_reported_against_the_nose_and_pitch_against_the_horizon():
    # drone facing north (ENU yaw 90), camera turned 30 deg LEFT and 45 deg down
    yaw, pitch, roll = siyi_angles(math.radians(30), 0.0, math.radians(-45), quat_about('z', 90))
    assert abs(yaw - 30) < 1e-9 and abs(pitch + 45) < 1e-6 and abs(roll) < 1e-6


def test_a_rolled_drone_shows_as_camera_roll_unless_the_roll_motor_cancels_it():
    _, pitch, roll = siyi_angles(0.0, 0.0, 0.0, quat_about('x', 12))           # right side 12 deg down, camera forward
    assert abs(roll - 12) < 1e-6 and abs(pitch) < 1e-6
    assert abs(siyi_angles(0.0, math.radians(-12), 0.0, quat_about('x', 12))[2]) < 1e-6


def test_the_numbers_stay_steady_when_the_camera_is_a_hair_past_straight_down():
    # the old Z-Y-X formula turned 0.4 deg past straight down into "pitch -89.6, roll 180" (and random rolls nearby)
    yaw, pitch, roll = siyi_angles(0.0, 0.0, math.radians(-90.4))
    assert abs(pitch + 90.4) < 1e-6 and abs(roll) < 1e-6
    rng = random.Random(5)
    for _ in range(200):                       # stabilised camera on a tilting drone, joints a little off
        tilt_roll, tilt_pitch = rng.uniform(-15, 15), rng.uniform(-15, 15)
        quat = _mul_quat(quat_about('y', tilt_pitch), quat_about('x', tilt_roll))
        _, pitch, roll = siyi_angles(0.0, math.radians(-tilt_roll + rng.uniform(-0.3, 0.3)),
                                     math.radians(-90 + tilt_pitch + rng.uniform(-0.3, 0.3)), quat)
        assert abs(pitch + 90) < 0.8 and abs(roll) < 0.8


def _mul_quat(a, b):
    """Quaternion product a * b, (x, y, z, w) order (turn b first, then a, both about world axes)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def test_the_angles_are_the_ones_the_siyi_emulator_reports():
    """Same numbers as siyi_emulator's GimbalModel.reported_attitude (what video_cam reads over SDK 0x0D) in the
    emulator's default, stabilising mode (GimbalModel() = stabilize True; with stabilize False it reports raw joints)."""
    emulator_src = Path(__file__).resolve().parents[2] / 'siyi_emulator'
    if str(emulator_src) not in sys.path:
        sys.path.insert(0, str(emulator_src))
    gimbal_control = pytest.importorskip('siyi_emulator.gimbal_control')
    rng = random.Random(7)
    for _ in range(300):
        joints = (math.radians(rng.uniform(-150, 150)), math.radians(rng.uniform(-30, 30)),
                  math.radians(rng.uniform(-130, 40)))
        quat = tuple(rng.uniform(-1, 1) for _ in range(4))
        model = gimbal_control.GimbalModel()
        model.set_drone_attitude(*quat)
        expected = model.reported_attitude(joints)
        got = siyi_angles(*joints, quat)
        assert max(abs(a - b) for a, b in zip(expected, got)) < 1e-6


class FakeClock:
    """A clock the test moves by hand."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_the_readout_says_plainly_when_there_is_no_data_yet_or_it_is_old():
    clock = FakeClock()
    readout = GimbalReadout(clock=clock)
    assert readout.lines() == ['gimbal: no angle data yet']
    assert readout.set_joints(['gimbal_yaw', 'gimbal_roll', 'gimbal_pitch'], [0.0, 0.0, -math.pi / 2])
    readout.set_attitude(0.0, 0.0, 0.0, 1.0)
    assert len(readout.lines()) == 2
    clock.now += GIMBAL_STALE_S + 1.0
    assert readout.lines() == [f'gimbal: no angle data for {GIMBAL_STALE_S + 1.0:.0f} s']


def test_the_readout_text_for_a_camera_looking_straight_down():
    clock = FakeClock()
    readout = GimbalReadout(clock=clock)
    # scoped joint names (model::joint) and an extra joint are fine; positions in radians
    readout.set_joints(['iris::rotor_0_joint', 'iris::gimbal_pitch', 'iris::gimbal_yaw', 'iris::gimbal_roll'],
                       [12.0, -math.pi / 2, -1e-9, 0.0])
    readout.set_attitude(0.0, 0.0, 0.0, 1.0)
    first, second = readout.lines()
    assert first == 'gimbal pitch -90.0 deg, yaw 0.0 deg, roll 0.0 deg'           # never "-0.0"
    assert second == 'joints (vs drone): pitch -90.0, yaw 0.0, roll 0.0 deg'


def test_the_readout_shows_the_stabiliser_at_work_on_a_tilted_drone():
    readout = GimbalReadout(clock=FakeClock())
    readout.set_joints(['gimbal_yaw', 'gimbal_roll', 'gimbal_pitch'], [0.0, 0.0, math.radians(-80)])
    readout.set_attitude(*quat_about('y', 10))                                     # nose 10 deg down
    first, second = readout.lines()
    assert first.startswith('gimbal pitch -90.0 deg') and 'pitch -80.0' in second


def test_without_the_drone_attitude_the_readout_assumes_a_level_drone_and_says_so():
    clock = FakeClock()
    readout = GimbalReadout(clock=clock)
    readout.set_joints(['gimbal_yaw', 'gimbal_roll', 'gimbal_pitch'], [0.0, 0.0, math.radians(-80)])
    first, _ = readout.lines()
    assert first.startswith('gimbal pitch -80.0 deg') and 'level drone assumed' in first
    readout.set_attitude(0.0, 0.0, 0.0, 1.0)
    clock.now += GIMBAL_STALE_S + 0.5                                              # attitude old, joints renewed
    readout.set_joints(['gimbal_yaw', 'gimbal_roll', 'gimbal_pitch'], [0.0, 0.0, math.radians(-80)])
    assert 'level drone assumed' in readout.lines()[0]


def test_incomplete_or_broken_angle_messages_are_ignored():
    readout = GimbalReadout(clock=FakeClock())
    assert not readout.set_joints(['gimbal_yaw', 'gimbal_pitch'], [0.0, 0.0])                   # roll missing
    assert not readout.set_joints(['gimbal_yaw', 'gimbal_roll', 'gimbal_pitch'], [0.0, float('nan'), 0.0])
    assert not readout.set_attitude(0.0, 0.0, 0.0, 0.0) and not readout.set_attitude(float('inf'), 0, 0, 1)
    assert readout.lines() == ['gimbal: no angle data yet']


# ------------------------------------------------------------------------------------------ the overlay box
def test_the_overlay_box_darkens_the_top_left_corner_and_leaves_the_rest_alone():
    plain = pixels(encode_frame(*solid(960, 540, (200, 200, 200)), quality=95))
    drawn = pixels(encode_frame(*solid(960, 540, (200, 200, 200)), quality=95,
                                overlay_lines=['gimbal pitch -90.0 deg, yaw 0.0 deg, roll 0.0 deg', 'joints ...']))
    assert drawn.shape == plain.shape == (540, 960, 3)
    corner = drawn[8:40, 8:60].astype(int)
    assert corner.min() < 120                     # the dark box (black at ~60 % over light grey) is there
    assert corner.max() > 200                     # ... with light text in it
    assert np.abs(drawn[450:, 600:].astype(int) - plain[450:, 600:].astype(int)).max() < 6     # rest unchanged


def test_the_overlay_can_go_to_the_bottom_left_corner():
    drawn = pixels(encode_frame(*solid(960, 540, (200, 200, 200)), quality=95, overlay_lines=['bottom text'],
                                overlay_position='bottom_left'))
    assert drawn[520:532, 8:40].astype(int).min() < 120 and drawn[8:30, 8:40].astype(int).min() > 180


def test_the_overlay_works_on_a_grey_picture_and_text_size_follows_the_picture():
    drawn = pixels(encode_frame(*solid(320, 180, (230, 0, 0), 'mono8'), quality=95, overlay_lines=['grey picture']))
    assert drawn[4:12, 4:30].astype(int).min() < 140
    small = draw_overlay(Image.new('RGB', (320, 180), (230, 230, 230)), ['abc'])
    big = draw_overlay(Image.new('RGB', (1920, 1080), (230, 230, 230)), ['abc'])
    dark_rows = [int((np.asarray(im)[:, :, 0] < 150).any(axis=1).sum()) for im in (small, big)]
    assert dark_rows[1] > 2 * dark_rows[0]        # the box (and text) is taller on a bigger picture


def test_text_is_measured_even_where_old_pillow_refuses_to_measure_the_builtin_font(monkeypatch):
    # Ubuntu 22.04's Pillow 9.0 raises "Only supported for TrueType fonts" in textbbox for its built-in font;
    # the overlay and the "waiting" picture must still work there (and on a PC without the DejaVu font).
    from PIL import ImageDraw, ImageFont

    import sim_demo.pov_server as pov

    def refuse(self, *args, **kwargs):
        raise ValueError('Only supported for TrueType fonts')

    monkeypatch.setattr(ImageDraw.ImageDraw, 'textbbox', refuse)
    font = ImageFont.load_default()
    if not hasattr(font, 'getsize'):
        pytest.skip('this Pillow has no getsize() (Pillow >= 10 measures every font with textbbox)')
    monkeypatch.setattr(pov, '_font', lambda size: font)
    drawn = np.asarray(draw_overlay(Image.new('RGB', (320, 180), (230, 230, 230)), ['gimbal pitch -90.0 deg']))
    assert drawn[6:12, 6:30].min() < 150                       # the box is there
    assert Image.open(io.BytesIO(placeholder_jpeg('waiting'))).size == (640, 360)


def test_without_an_overlay_the_picture_is_exactly_what_it_was_before():
    rng = np.random.default_rng(3)
    data = rng.integers(0, 255, (90, 160, 3), dtype=np.uint8).tobytes()
    assert encode_frame(160, 90, 'rgb8', 480, data) == encode_frame(160, 90, 'rgb8', 480, data, overlay_lines=None)
    assert encode_frame(160, 90, 'rgb8', 480, data) == encode_frame(160, 90, 'rgb8', 480, data, overlay_lines=[])


# ------------------------------------------------------------------------------------------ the store
def test_the_store_hands_the_newest_picture_to_a_waiting_reader():
    store = FrameStore()
    assert store.latest() == (0, None) and store.age_s() is None and store.count == 0
    threading.Timer(0.05, store.put, args=(b'one',)).start()
    started = time.monotonic()
    seq, jpeg = store.wait_newer(0, 3.0)
    assert (seq, jpeg) == (1, b'one') and time.monotonic() - started < 1.5
    assert store.count == 1 and 0 <= store.age_s() < 1.0


def test_waiting_for_a_picture_that_never_comes_returns_after_the_timeout():
    store = FrameStore()
    started = time.monotonic()
    assert store.wait_newer(0, 0.15) == (0, None)
    assert 0.1 <= time.monotonic() - started < 1.0


def test_only_the_newest_of_several_pictures_is_kept():
    store = FrameStore()
    for index in range(5):
        store.put(f'frame{index}'.encode())
    assert store.latest() == (5, b'frame4')


# ------------------------------------------------------------------------------------------ the server
@pytest.fixture()
def server():
    store = FrameStore()
    running = PovServer(store, '127.0.0.1', 0, keepalive_s=0.2, max_clients=2).start()
    yield running
    running.stop()


def get(server, path, timeout=5.0):
    connection = http.client.HTTPConnection('127.0.0.1', server.port, timeout=timeout)
    connection.request('GET', path)
    response = connection.getresponse()
    return connection, response


def read_parts(response, count):
    """Read `count` multipart parts from a stream: [(headers dict, body bytes)]."""
    parts = []
    for _ in range(count):
        assert response.readline().strip() == f'--{BOUNDARY}'.encode()
        headers = {}
        while True:
            line = response.readline().strip()
            if not line:
                break
            key, _, value = line.decode().partition(':')
            headers[key.strip().lower()] = value.strip()
        body = response.read(int(headers['content-length']))
        assert response.read(2) == b'\r\n'
        parts.append((headers, body))
    return parts


def jpeg(color):
    buffer = io.BytesIO()
    Image.new('RGB', (32, 24), color).save(buffer, 'JPEG')
    return buffer.getvalue()


def test_the_single_picture_is_the_waiting_one_first_and_then_the_newest(server):
    connection, response = get(server, '/pov.jpg')
    first = response.read()
    assert response.status == 200 and response.getheader('Content-Type') == 'image/jpeg' and first == server.placeholder()
    connection.close()
    server.store.put(jpeg((255, 0, 0)))
    connection, response = get(server, '/pov.jpg')
    assert response.read() == server.store.latest()[1]
    connection.close()


def test_the_page_tells_the_three_clicks_for_mission_planner(server):
    connection, response = get(server, '/')
    page = response.read().decode()
    connection.close()
    assert response.status == 200 and 'Set MJPEG Source' in page and f'{server.url}/pov.mjpg' in page and '/pov.mjpg' in page


def test_the_status_line_counts_frames_and_viewers(server):
    server.store.put(jpeg((0, 0, 0)))
    connection, response = get(server, '/health')
    text = response.read().decode()
    connection.close()
    assert text.startswith('ok frames=1 newest_age_s=0') and 'viewers=0' in text


def test_unknown_paths_are_404(server):
    connection, response = get(server, '/nope')
    assert response.status == 404
    connection.close()


def test_the_stream_is_multipart_with_one_part_per_picture(server):
    first, second = jpeg((255, 0, 0)), jpeg((0, 0, 255))
    server.store.put(first)
    connection, response = get(server, '/pov.mjpg')
    assert response.status == 200
    assert response.getheader('Content-Type') == f'multipart/x-mixed-replace; boundary={BOUNDARY}'
    [(headers, body)] = read_parts(response, 1)
    assert body == first and headers['content-type'] == 'image/jpeg' and int(headers['content-length']) == len(first)
    server.store.put(second)
    [(_, body)] = read_parts(response, 1)
    assert body == second and body[:2] == b'\xff\xd8' and body[-2:] == b'\xff\xd9'
    connection.close()


def test_a_stream_before_any_picture_shows_the_waiting_picture_and_then_the_real_one(server):
    connection, response = get(server, '/pov.mjpg')
    [(_, waiting)] = read_parts(response, 1)
    assert waiting == server.placeholder()
    real = jpeg((0, 255, 0))
    server.store.put(real)
    deadline = time.monotonic() + 3.0
    body = waiting
    while body != real and time.monotonic() < deadline:
        [(_, body)] = read_parts(response, 1)
    assert body == real
    connection.close()


def test_a_quiet_camera_repeats_the_newest_picture_so_the_connection_stays_alive(server):
    only = jpeg((9, 9, 9))
    server.store.put(only)
    connection, response = get(server, '/pov.mjpg')
    started = time.monotonic()
    parts = read_parts(response, 3)
    assert [body for _, body in parts] == [only] * 3 and time.monotonic() - started >= 0.3      # keepalive_s is 0.2
    connection.close()


def test_too_many_viewers_get_a_polite_503_and_the_slot_comes_back(server):
    open_streams = []
    for _ in range(2):
        open_streams.append(get(server, '/pov.mjpg'))
        read_parts(open_streams[-1][1], 1)
    assert server.clients == 2
    connection, response = get(server, '/pov.mjpg')
    assert response.status == 503
    connection.close()
    open_streams.pop()[0].close()
    deadline = time.monotonic() + 3.0
    while server.clients > 1 and time.monotonic() < deadline:
        server.store.put(jpeg((1, 1, 1)))                                   # a write makes the dead viewer's thread notice
        time.sleep(0.05)
    assert server.clients == 1
    for connection, _ in open_streams:
        connection.close()


@pytest.mark.skipif(sys.platform == 'win32', reason='Windows lets two programs share a port (SO_REUSEADDR); Linux, where this runs, refuses')
def test_a_taken_port_gives_a_message_that_says_what_to_do(server):
    other = PovServer(FrameStore(), '127.0.0.1', server.port)
    with pytest.raises(OSError) as caught:
        other.start()
    assert 'cannot listen' in str(caught.value) and 'port:=8082' in str(caught.value)


def test_stopping_ends_open_streams_and_can_be_repeated():
    store = FrameStore()
    running = PovServer(store, '127.0.0.1', 0, keepalive_s=0.1).start()
    store.put(jpeg((5, 5, 5)))
    connection, response = get(running, '/pov.mjpg')
    read_parts(response, 1)
    running.stop()
    running.stop()
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if not response.read(1):
                break
    except (ConnectionError, OSError, http.client.HTTPException):
        pass
    connection.close()


def test_it_listens_on_this_computer_only_by_default_and_shows_a_usable_address():
    assert PovServer(FrameStore()).host == '127.0.0.1'
    assert PovServer(FrameStore(), '0.0.0.0', 8081).url == 'http://127.0.0.1:8081'
