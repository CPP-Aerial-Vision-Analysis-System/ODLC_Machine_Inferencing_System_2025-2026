"""
test/test_sd_card.py -- PURPOSE: unit tests of siyi_emulator.sd_card: IMG_NNNN.jpg naming, oldest-first
listing, one directory for the run, atomic writes with JSON sidecars, format (0x48) without reusing
names, clear-on-start, and path-traversal safety of resolve().
Run: python3 -m pytest test/   (no ROS needed)
"""

import json

import pytest

from siyi_emulator.sd_card import SdCard, default_sd_card_dir


def test_naming_listing_and_sidecar(tmp_path):
    sd = SdCard(str(tmp_path), "101SIYI_IMG")
    assert sd.directories() == [{"name": "101SIYI_IMG", "path": "101SIYI_IMG"}]
    assert sd.list_photos() == []
    assert sd.peek_next_name() == "IMG_0001.jpg"
    p1 = sd.write_photo(b"\xff\xd8 first", {"zoom": 1.0})
    p2 = sd.write_photo(b"\xff\xd8 second")
    assert (p1.name, p2.name) == ("IMG_0001.jpg", "IMG_0002.jpg")
    assert p1.rel_path == "101SIYI_IMG/IMG_0001.jpg"
    names = [p.name for p in sd.list_photos()]
    assert names == ["IMG_0001.jpg", "IMG_0002.jpg"]          # oldest first
    meta = json.loads((tmp_path / "101SIYI_IMG" / "IMG_0001.json").read_text())
    assert meta["file"] == "101SIYI_IMG/IMG_0001.jpg" and meta["index"] == 1 and meta["zoom"] == 1.0
    assert not (tmp_path / "101SIYI_IMG" / "IMG_0002.json").exists()
    assert sd.count() == 2 and sd.list_photos()[0].size == len(b"\xff\xd8 first")


def test_listing_order_is_numeric_not_alphabetic(tmp_path):
    sd = SdCard(str(tmp_path))
    sd._next_index = 9998
    for _ in range(3):
        sd.write_photo(b"x")
    assert [p.index for p in sd.list_photos()] == [9998, 9999, 10000]


def test_format_deletes_everything_but_keeps_numbering(tmp_path):
    sd = SdCard(str(tmp_path))
    for _ in range(3):
        sd.write_photo(b"x", {"a": 1})
    assert sd.format() == 6                                     # 3 jpg + 3 json
    assert sd.list_photos() == []
    assert sd.write_photo(b"y").name == "IMG_0004.jpg"          # never reuse a name video_cam has seen


def test_format_can_restart_numbering(tmp_path):
    sd = SdCard(str(tmp_path), restart_numbering_after_format=True)
    sd.write_photo(b"x")
    sd.format()
    assert sd.write_photo(b"y").name == "IMG_0001.jpg"


def test_clear_on_start_and_continue_numbering(tmp_path):
    sd = SdCard(str(tmp_path))
    sd.write_photo(b"x")
    sd.write_photo(b"x")
    kept = SdCard(str(tmp_path), clear_on_start=False)
    assert kept.count() == 2 and kept.peek_next_name() == "IMG_0003.jpg"
    fresh = SdCard(str(tmp_path), clear_on_start=True)
    assert fresh.count() == 0 and fresh.peek_next_name() == "IMG_0001.jpg"


def test_has_directory_ignores_slashes(tmp_path):
    sd = SdCard(str(tmp_path), "101SIYI_IMG")
    assert sd.has_directory("101SIYI_IMG") and sd.has_directory("/101SIYI_IMG/")
    assert not sd.has_directory("A:/DCIM/100MEDIA") and not sd.has_directory("")


def test_resolve_is_safe(tmp_path):
    sd = SdCard(str(tmp_path))
    photo = sd.write_photo(b"x")
    assert str(sd.resolve("101SIYI_IMG/IMG_0001.jpg")) == str((tmp_path / photo.rel_path).resolve())
    assert sd.resolve("/101SIYI_IMG/IMG_0001.jpg") is not None
    for bad in ("../../etc/passwd", "101SIYI_IMG/../../x", "", "101SIYI_IMG/IMG_9999.jpg", "101SIYI_IMG//x"):
        assert sd.resolve(bad) is None, bad


def test_bad_directory_names_rejected(tmp_path):
    for bad in ("../x", "a/b"):
        with pytest.raises(ValueError):
            SdCard(str(tmp_path), bad)


def test_default_dir_expands_home():
    assert default_sd_card_dir("").endswith("/.ros/siyi_emulator/sd_card") or "siyi_emulator" in default_sd_card_dir("")
    assert "~" not in default_sd_card_dir("~/x")
