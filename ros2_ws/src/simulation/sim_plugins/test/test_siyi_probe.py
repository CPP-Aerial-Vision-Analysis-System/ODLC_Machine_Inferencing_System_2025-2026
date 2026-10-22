"""
sim_plugins/test/test_siyi_probe.py -- PURPOSE: unit tests of the camera-slot network probe (siyi_probe.py):
the SIYI SDK frame and CRC match the team's own constant photo packet, frames round-trip, and the two probes
work against tiny fake servers on 127.0.0.1 (a UDP "camera" that answers 0x02, an HTTP media API) and fail
politely when nothing answers.
"""

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

from sim_plugins import siyi_probe as sp


def test_crc_matches_the_teams_photo_packet():
    # video_cam always sends this exact packet for "take a photo" (SYNTHESIS.md section 2.6):
    team_packet = bytes.fromhex('556601010000000c0034ce')
    assert sp.build_frame(0x0C, b'\x00') == team_packet
    assert sp.crc16_xmodem(team_packet[:-2]) == 0xCE34


def test_frame_round_trip_and_bad_crc():
    frame = sp.build_frame(0x02, b'', seq=7)
    assert sp.parse_frame(frame) == (0x02, b'')
    data_frame = sp.build_frame(0x02, b'6B0123456789', seq=1)
    assert sp.parse_frame(b'junk' + data_frame) == (0x02, b'6B0123456789')
    broken = bytearray(data_frame)
    broken[-1] ^= 0xFF
    assert sp.parse_frame(bytes(broken)) is None
    assert sp.parse_frame(b'\x55\x66\x01') is None


def _free_udp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    return sock


def test_probe_sdk_against_a_fake_camera():
    server = _free_udp_server()
    port = server.getsockname()[1]

    def answer():
        data, addr = server.recvfrom(1024)
        if sp.parse_frame(data) == (0x02, b''):
            server.sendto(sp.build_frame(0x02, b'73' + b'0' * 10, need_ack=False), addr)

    t = threading.Thread(target=answer, daemon=True)
    t.start()
    ok, text = sp.probe_sdk('127.0.0.1', port=port, timeout=3.0)
    server.close()
    assert ok and 'hardware id "73' in text


def test_probe_sdk_silence_is_reported():
    server = _free_udp_server()                     # bound, but never answers
    port = server.getsockname()[1]
    ok, text = sp.probe_sdk('127.0.0.1', port=port, timeout=0.3)
    server.close()
    assert not ok and 'no answer' in text and 'camera_ip_alias.sh' in text


class _Media(BaseHTTPRequestHandler):
    def do_GET(self):                               # noqa: N802 - http.server's name
        body = json.dumps({'code': 200, 'data': {'directories': [{'name': '101SIYI_IMG'}]}}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):                   # keep the test output clean
        pass


def test_probe_media_against_a_fake_web_api():
    server = HTTPServer(('127.0.0.1', 0), _Media)
    t = threading.Thread(target=server.handle_request, daemon=True)
    t.start()
    ok, text = sp.probe_media('127.0.0.1', port=server.server_address[1], timeout=3.0)
    server.server_close()
    assert ok and '101SIYI_IMG' in text


def test_probe_media_nothing_listening():
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()                                    # a port that is free right now: connection refused
    ok, text = sp.probe_media('127.0.0.1', port=port, timeout=1.0)
    assert not ok and 'does not answer' in text
