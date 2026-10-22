"""
test/test_astra_sitl_parm.py -- PURPOSE: check config/astra_sitl.parm the way ArduPilot will read it.

ArduPilot reads a defaults file with fgets() into a 100-byte buffer (AP_Param, char line[100],
fgets(line, sizeof(line) - 1)), so a line longer than 98 bytes is cut into pieces and every
piece is parsed as its own "NAME VALUE" line. This test re-creates that reader and makes sure
the file sets exactly the parameters we mean, that every line is short and commented, and that
the values the simulation depends on are there.

Run:  python3 -m pytest -q test/test_astra_sitl_parm.py
"""

import os
import re

import pytest

PARM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "astra_sitl.parm")
MAX_LINE = 90  # our own limit (ArduPilot's hard limit is 98 bytes incl. newline)
AP_MAX_NAME_SIZE = 16  # ArduPilot parameter names are at most 16 characters

# What the simulation needs (name: value). The file may set more, but never different values.
REQUIRED = {
    "SERVO11_FUNCTION": 0.0,  # person drop servo free for MAV_CMD_DO_SET_SERVO
    "SERVO13_FUNCTION": 0.0,  # tent drop servo free for MAV_CMD_DO_SET_SERVO
    "MNT1_TYPE": 0.0,  # ArduPilot does not drive the gimbal
    "AUTO_OPTIONS": 3.0,  # arm + take off in AUTO without RC sticks
    "FENCE_ENABLE": 0.0,  # no geofence
    "SIM_SPEEDUP": 1.0,  # real time (team code uses wall-clock sleeps)
}


def ardupilot_parse(path):
    """Read a defaults file like AP_Param::read_param_defaults_file and return [(name, value)]."""
    out = []
    with open(path, "rb") as f:
        data = f.read()
    pos = 0
    while pos < len(data):
        # fgets(line, 99): at most 98 bytes, stops after a newline
        end = data.find(b"\n", pos, pos + 98)
        chunk = data[pos : end + 1] if end != -1 else data[pos : pos + 98]
        pos += len(chunk)
        line = chunk.decode("ascii", "replace")
        if line.startswith("#"):
            continue
        tokens = [t for t in re.split(r"[, =\t\r\n]+", line) if t]
        if len(tokens) < 2:
            continue
        name, value = tokens[0], tokens[1]
        if len(name) > AP_MAX_NAME_SIZE:
            continue
        try:
            out.append((name, float(value)))
        except ValueError:
            # ArduPilot would use strtof() = 0.0 here; that is never what we want
            out.append((name, float("nan")))
    return out


def test_file_exists():
    assert os.path.isfile(PARM)


def test_every_line_is_short_ascii():
    with open(PARM, "rb") as f:
        for n, raw in enumerate(f, 1):
            assert len(raw.rstrip(b"\r\n")) < MAX_LINE, f"line {n} is too long ({len(raw)} bytes)"
            raw.decode("ascii")  # raises on non-ASCII


def test_every_parameter_line_has_a_reason():
    with open(PARM) as f:
        for n, line in enumerate(f, 1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            assert "#" in s, f'line {n} has no "# why" comment: {s}'
            reason = s.split("#", 1)[1].strip()
            assert len(reason) >= 8, f"line {n}: the comment is too short to explain why"


def test_parsed_parameters_are_exactly_the_intended_ones():
    parsed = ardupilot_parse(PARM)
    names = [n for n, _ in parsed]
    # every parsed name must look like an ArduPilot parameter (no comment pieces slipped in)
    for name, value in parsed:
        assert re.fullmatch(r"[A-Z][A-Z0-9_]*", name), f"stray token parsed as a parameter: {name}"
        assert value == value, f"{name} has a non-numeric value"
    assert len(names) == len(set(names)), "a parameter is set twice"
    values = dict(parsed)
    for name, value in REQUIRED.items():
        assert values.get(name) == pytest.approx(value), f"{name} must be {value}"


def test_parser_catches_long_lines(tmp_path):
    """Self-test of the reader: a 110-byte line is split after 98 bytes, like ArduPilot does,
    and the tail is read as a NEW parameter (this is the accident the line limit prevents)."""
    p = tmp_path / "long.parm"
    p.write_text("SERVO11_FUNCTION 0 " + "x" * 79 + "MNT1_TYPE 1\n")
    parsed = ardupilot_parse(str(p))
    assert [n for n, _ in parsed] == ["SERVO11_FUNCTION", "MNT1_TYPE"]
    assert dict(parsed)["MNT1_TYPE"] == 1.0
