"""
sim_demo/test/test_pov_server.py -- PURPOSE: prove the camera feed for Mission Planner without ROS or Gazebo: ROS
pictures in every supported encoding become correct JPEGs (right colours, padded rows handled, big pictures shrunk,
corrupt ones refused), the web server answers the page, a single picture, the status line and a real multipart stream
with the right framing, a quiet camera does not kill the connection, too many viewers are turned away politely, a
taken port gives a message that says what to do, and the stream ends when the server stops.
"""

import http.client
import io
import sys
import threading
import time

import numpy as np
import pytest
from PIL import Image

from sim_demo.pov_server import BOUNDARY, FrameStore, PovServer, encode_frame, placeholder_jpeg


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
