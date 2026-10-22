"""
test/test_media_server.py -- PURPOSE: unit tests of siyi_emulator.media_server over real HTTP on
127.0.0.1 (free port): the exact JSON shapes video_cam depends on (getdirectories, getmedialist with
count=1 / count=9999 and "total", getmediacount), file download, 404s, SIYI's double-slash base URL,
media_type validation, URLs built from the Host header, and keep-alive with requests.Session.
Run: python3 -m pytest test/   (no ROS needed)
"""

import json
import urllib.error
import urllib.request

import pytest

from siyi_emulator.media_server import (MediaServer, directories_response, media_list_response,
                                        parse_media_type)
from siyi_emulator.sd_card import SdCard

API = "/cgi-bin/media.cgi/api/v1"


@pytest.fixture()
def server(tmp_path):
    sd = SdCard(str(tmp_path), "101SIYI_IMG")
    srv = MediaServer(sd, "127.0.0.1", 0)
    srv.start()
    yield srv
    srv.stop()


def get(srv, path, host=None):
    req = urllib.request.Request(f"http://127.0.0.1:{srv.port}{path}")
    if host:
        req.add_header("Host", host)
    with urllib.request.urlopen(req, timeout=5) as resp:
        return resp.status, resp.headers, resp.read()


def get_json(srv, path, host=None):
    status, _, body = get(srv, path, host)
    assert status == 200
    return json.loads(body)


def test_getdirectories_shape(server):
    doc = get_json(server, f"{API}/getdirectories?media_type=0")
    assert doc["success"] is True and doc["code"] == 200
    assert doc["data"]["directories"] == [{"name": "101SIYI_IMG", "path": "101SIYI_IMG"}]
    # video_cam takes directories[-1]['path']
    assert doc["data"]["directories"][-1]["path"] == "101SIYI_IMG"


def test_getmedialist_shapes_and_total(server):
    for _ in range(3):
        server.sd.write_photo(b"\xff\xd8" + b"x" * 100)
    doc = get_json(server, f"{API}/getmedialist?media_type=0&path=101SIYI_IMG&start=0&count=9999",
                   host="192.168.144.25:82")
    items = doc["data"]["list"]
    assert doc["success"] is True and doc["data"]["total"] == 3
    assert [i["name"] for i in items] == ["IMG_0001.jpg", "IMG_0002.jpg", "IMG_0003.jpg"]
    assert items[0]["url"] == "http://192.168.144.25:82/101SIYI_IMG/IMG_0001.jpg"
    assert items[0]["size"] == 102
    # video_cam's get_media_count(): count=1 and read data.total
    one = get_json(server, f"{API}/getmedialist?media_type=0&path=101SIYI_IMG&start=0&count=1")
    assert len(one["data"]["list"]) == 1 and one["data"]["total"] == 3
    page = get_json(server, f"{API}/getmedialist?media_type=0&path=101SIYI_IMG&start=2&count=5")
    assert [i["name"] for i in page["data"]["list"]] == ["IMG_0003.jpg"]


def test_url_uses_host_header_and_downloads(server):
    photo = server.sd.write_photo(b"\xff\xd8JPEGDATA")
    doc = get_json(server, f"{API}/getmedialist?media_type=0&path=101SIYI_IMG")
    url = doc["data"]["list"][0]["url"]
    assert url == f"http://127.0.0.1:{server.port}/101SIYI_IMG/IMG_0001.jpg"
    with urllib.request.urlopen(url, timeout=5) as resp:
        assert resp.status == 200 and resp.headers["Content-Type"] == "image/jpeg"
        assert resp.read() == b"\xff\xd8JPEGDATA"
    assert photo.size == len(b"\xff\xd8JPEGDATA")


def test_errors_and_videos(server):
    doc = get_json(server, f"{API}/getmedialist?media_type=0&path=A:/DCIM/100MEDIA")
    assert doc["success"] is False and doc["message"] == "path not exist"
    doc = get_json(server, f"{API}/getdirectories?media_type=7")
    assert doc["success"] is False and doc["message"] == "Invalid media type"
    doc = get_json(server, f"{API}/getdirectories?media_type=1")
    assert doc["success"] is True and doc["data"]["directories"] == []
    doc = get_json(server, f"{API}/getmedialist?media_type=1&path=101SIYI_IMG")
    assert doc["data"] == {"list": [], "total": 0}
    with pytest.raises(urllib.error.HTTPError) as err:
        get(server, "/101SIYI_IMG/IMG_0099.jpg")
    assert err.value.code == 404
    with pytest.raises(urllib.error.HTTPError):
        get(server, "/../../etc/passwd")


def test_double_slash_base_url_and_count(server):
    server.sd.write_photo(b"x")
    doc = get_json(server, f"//cgi-bin/media.cgi/api/v1/getmediacount?media_type=0&path=101SIYI_IMG")
    assert doc["success"] is True and doc["data"]["count"] == 1


def test_index_page(server):
    server.sd.write_photo(b"x")
    status, headers, body = get(server, "/")
    assert status == 200 and b"IMG_0001.jpg" in body and "text/html" in headers["Content-Type"]


def test_keep_alive_with_requests_session(server):
    requests = pytest.importorskip("requests")
    server.sd.write_photo(b"\xff\xd8abc")
    with requests.Session() as session:
        session.headers.update({"User-Agent": "SIYI-ROS-Client/1.0", "Connection": "keep-alive"})
        for _ in range(5):   # same connection reused, like video_cam's pooled session
            r = session.get(f"http://127.0.0.1:{server.port}{API}/getmedialist",
                            params={"media_type": "0", "path": "101SIYI_IMG", "start": 0, "count": 9999}, timeout=5)
            assert r.status_code == 200 and r.json()["data"]["total"] == 1
        r = session.get(r.json()["data"]["list"][0]["url"], timeout=5)
        assert r.content == b"\xff\xd8abc"


def test_pure_builders(tmp_path):
    sd = SdCard(str(tmp_path))
    assert parse_media_type(None) == 0 and parse_media_type("1") == 1 and parse_media_type("x") is None
    assert directories_response(sd, None)["success"] is False
    assert media_list_response(sd, 0, "101SIYI_IMG", 0, 10, "http://h")["data"] == {"list": [], "total": 0}


def test_port_in_use_raises(server):
    other = MediaServer(server.sd, "127.0.0.1", server.port)
    with pytest.raises(OSError):
        other.start()
