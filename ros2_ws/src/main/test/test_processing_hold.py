"""
Unit tests for main/src/processing_hold.py (when main_controller's detector hold ends).

test_processing_hold.py -- PURPOSE: check the rules of the GUIDED "wait for the detector" hold
after the Task 2 review fix: listing a missing, unreadable or non-folder path never raises, an
unreadable folder means "keep waiting" until the deadline and then always "timed out" (the hold
can no longer last for ever), the counts and the all-done / timeout answers are the same as the
old code gave for a readable folder, and a broken (NaN) elapsed time ends the hold.

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/main/test/test_processing_hold.py -v
"""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import processing_hold as hold  # noqa: E402


def test_lists_only_photos(tmp_path):
    for name in ('a.jpg', 'B.JPEG', 'c.png', 'd.bmp', 'a.json', 'x.tmp'):
        (tmp_path / name).write_bytes(b'')
    names, why = hold.list_photo_names(str(tmp_path))
    assert names == {'a.jpg', 'B.JPEG', 'c.png', 'd.bmp'} and why == ''


def test_missing_folder_does_not_raise(tmp_path):
    names, why = hold.list_photo_names(str(tmp_path / 'gone'))
    assert names is None and 'FileNotFoundError' in why


def test_a_file_instead_of_a_folder_does_not_raise(tmp_path):
    f = tmp_path / 'file'
    f.write_text('x')
    names, why = hold.list_photo_names(str(f))
    assert names is None and 'NotADirectoryError' in why


@pytest.mark.skipif(not hasattr(os, 'geteuid') or os.geteuid() == 0,
                    reason='needs a non-root POSIX user (root may read any folder)')
def test_folder_without_permission_does_not_raise(tmp_path):
    locked = tmp_path / 'locked'
    locked.mkdir()
    (locked / 'a.jpg').write_bytes(b'')
    locked.chmod(0)
    try:
        names, why = hold.list_photo_names(str(locked))
    finally:
        locked.chmod(0o755)
    assert names is None and 'PermissionError' in why


def test_readable_folder_counts_like_before():
    d = hold.decide({'a.jpg', 'b.jpg', 'c.jpg'}, {'a.jpg', 'old.jpg'}, 10.0, 90.0)
    assert (d.action, d.processed, d.total, d.remaining) == (hold.WAIT, 1, 3, 2)
    assert hold.decide({'a.jpg'}, {'a.jpg'}, 10.0, 90.0).action == hold.ALL_DONE
    assert hold.decide(set(), set(), 0.0, 90.0).action == hold.ALL_DONE     # no photos at all
    d = hold.decide({'a.jpg', 'b.jpg'}, {'a.jpg'}, 90.0, 90.0)
    assert (d.action, d.remaining) == (hold.TIMED_OUT, 1)


def test_unreadable_folder_waits_then_always_times_out():
    # The Task 2 bug: with an unreadable folder the old code never reached its timeout.
    assert hold.decide(None, {'a.jpg'}, 89.9, 90.0).action == hold.WAIT
    d = hold.decide(None, {'a.jpg'}, 90.0, 90.0)
    assert d.action == hold.TIMED_OUT and not d.folder_readable
    assert hold.decide(None, set(), 5000.0, 90.0).action == hold.TIMED_OUT


def test_broken_elapsed_time_ends_the_hold():
    assert hold.decide({'a.jpg'}, set(), math.nan, 90.0).action == hold.TIMED_OUT
    assert hold.decide(None, set(), math.nan, 90.0).action == hold.TIMED_OUT
