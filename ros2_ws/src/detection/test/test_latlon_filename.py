"""Unit tests for the GPS-in-filename contract.

siyi_node._generate_gps_filename() writes captures as "<lat> , <lon>.jpg";
new_od._parse_latlon_from_name() reads them back via detection.geotag.
main_controller uses the result to place the drop waypoints, so if this
contract breaks, every target silently geolocates to (0.0, 0.0) and we
only find out in the field.

Needs nothing but Python -- no ROS, no build, no sourcing:
    python3 -m pytest ros2_ws/src/detection/test/test_latlon_filename.py -v
"""

import os
import sys

# Import detection.geotag straight from the source tree, whatever the cwd.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from detection.geotag import parse_latlon_from_name as parse  # noqa: E402


def test_parses_a_normal_capture_name():
    assert parse("38.315339 , -76.548108.jpg") == (38.315339, -76.548108)


def test_parses_a_negative_latitude():
    assert parse("-12.500000 , 34.750000.jpg") == (-12.5, 34.75)


def test_sd_card_name_has_no_coordinates():
    # Camera fell back to the SD name because there was no GPS fix.
    assert parse("IMG_0042.jpg") == (None, None)


def test_comma_without_spaces_is_not_the_delimiter():
    # The delimiter is " , " (space-comma-space), not ",".
    assert parse("38.31,-76.54.jpg") == (None, None)


def test_non_numeric_halves_are_rejected():
    assert parse("abc , def.jpg") == (None, None)


def test_round_trip_with_the_exact_format_siyi_node_writes():
    lat, lon = 38.315339, -76.548108
    filename = f"{lat:.6f} , {lon:.6f}.jpg"   # same f-string as siyi_node
    assert parse(filename) == (lat, lon)
