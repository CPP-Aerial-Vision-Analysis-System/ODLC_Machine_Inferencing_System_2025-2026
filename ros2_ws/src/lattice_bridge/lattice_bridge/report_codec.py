"""
lattice_bridge/report_codec.py -- PURPOSE: the tiny radio message that carries "I found a target" from the
Jetson on the drone to the ground laptop. In flight the Jetson has no internet; its only link to the ground
is the MAVLink telemetry radio. A MAVLink STATUSTEXT (text message, shown in Mission Planner as well) can
carry at most 50 characters, so a target report is one short line of plain ASCII:

    TGT <p|t> <latitude> <longitude> <confidence 0-100> <seq>
    TGT p 36.214935 -96.006203 95 12

      TGT         the fixed word that marks our messages (everything else on the radio is ignored)
      p | t       person (the hiker) or tent
      latitude    degrees, exactly 6 decimals (about 11 cm), -90 .. 90
      longitude   degrees, exactly 6 decimals, -180 .. 180
      confidence  whole percent, 0 .. 100
      seq         message counter 0 .. 999999, then back to 0 (lets the ground notice lost or repeated messages)

One message is sent per accepted target change. The Jetson sends it as `/mavros/statustext/send` with
severity INFO (6); the ground bridge reads it from the STATUSTEXT stream.

SAFETY OF THE DECODER: the text arrives over a radio, so it is UNTRUSTED. decode_tgt() never raises, never
evaluates, executes or formats the text (no eval/exec/format/%), accepts exactly the canonical form above and
returns None for everything else: wrong word, extra or missing spaces, upper/lower-case mistakes, numbers with
the wrong number of decimals or with leading zeros, out-of-range values, (0, 0), non-ASCII characters, digits
from other alphabets, newlines, texts over 50 characters, non-text input. The single tolerated oddity is
trailing NUL characters, because a MAVLink text field is NUL-padded. test_report_codec.py fuzzes it.

Pure Python (re, math): no ROS, no I/O.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Optional

from . import constants as C
from .geo import is_real_number, require_lat_lon

# One strict pattern: ASCII digits only ([0-9], never \d, which also matches other alphabets), no leading
# zeros, exactly 6 decimals, fullmatch() so nothing may follow (not even a newline). Ranges are checked after.
_TGT = re.compile(
    r'TGT (?P<code>[pt])'
    r' (?P<lat>-?(?:0|[1-9][0-9]?)\.[0-9]{6})'
    r' (?P<lon>-?(?:0|[1-9][0-9]{0,2})\.[0-9]{6})'
    r' (?P<conf>0|[1-9][0-9]?|100)'
    r' (?P<seq>0|[1-9][0-9]{0,5})'
)


@dataclass(frozen=True)
class TgtReport:
    """A decoded TGT message. `confidence` is the fraction 0..1, `confidence_percent` the whole percent."""
    kind: str                  # 'person' or 'tent'
    lat_deg: float
    lon_deg: float
    confidence_percent: int
    seq: int

    @property
    def confidence(self) -> float:
        """The confidence as a fraction from 0.0 to 1.0."""
        return self.confidence_percent / 100.0


def next_seq(seq: int) -> int:
    """The message counter after `seq`: +1, wrapping from 999999 to 0."""
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= C.TGT_SEQ_MAX:
        raise ValueError(f'seq must be an integer from 0 to {C.TGT_SEQ_MAX}')
    return (seq + 1) % (C.TGT_SEQ_MAX + 1)


def _format_coordinate(value: float) -> str:
    """Six decimals, and never the text '-0.000000'."""
    text = f'{round(value, 6):.6f}'
    return '0.000000' if text == '-0.000000' else text


def encode_tgt(kind, lat_deg, lon_deg, confidence, seq) -> str:
    """Build the TGT message text. Raises ValueError for anything that cannot be sent faithfully.

    `kind`: 'person' / 'tent' (or the codes 'p' / 't'). `confidence`: the detector's fraction 0.0 .. 1.0
    (rounded to whole percent). `seq`: integer 0 .. 999999. The result is at most 43 characters.
    """
    code = None
    if isinstance(kind, str):                       # (an unhashable kind such as a list must not reach a dict)
        code = C.TGT_CODE_BY_KIND.get(kind, kind if kind in C.TGT_KIND_BY_CODE else None)
    if code is None:
        raise ValueError(f"kind must be 'person' or 'tent', got {str(kind)[:20]!r}")
    lat, lon = require_lat_lon(lat_deg, lon_deg, 'target position')
    if not is_real_number(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError(f'confidence must be a number from 0 to 1, got {str(confidence)[:20]!r}')
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= C.TGT_SEQ_MAX:
        raise ValueError(f'seq must be an integer from 0 to {C.TGT_SEQ_MAX}, got {str(seq)[:20]!r}')
    percent = int(math.floor(float(confidence) * 100.0 + 0.5))
    text = f'{C.TGT_PREFIX} {code} {_format_coordinate(lat)} {_format_coordinate(lon)} {percent} {seq}'
    if len(text) > C.STATUSTEXT_MAX_CHARS or not text.isascii():          # cannot happen; belt and braces
        raise ValueError('encoded message is too long for a STATUSTEXT')
    return text


def decode_tgt(text) -> Optional[TgtReport]:
    """Read a TGT message; return a TgtReport, or None when `text` is anything else. Never raises."""
    try:
        if isinstance(text, (bytes, bytearray)):
            text = bytes(text).rstrip(b'\x00').decode('ascii')      # a wire field is NUL-padded
        elif isinstance(text, str):
            text = text.rstrip('\x00')
        else:
            return None
        if not 0 < len(text) <= C.STATUSTEXT_MAX_CHARS or not text.isascii():
            return None
        match = _TGT.fullmatch(text)
        if match is None:
            return None
        lat_text, lon_text = match.group('lat'), match.group('lon')
        lat, lon = float(lat_text), float(lon_text)
        if (lat_text[0] == '-' and lat == 0.0) or (lon_text[0] == '-' and lon == 0.0):
            return None                                              # '-0.000000' is not canonical
        lat, lon = require_lat_lon(lat, lon, 'target position')      # range check and the (0, 0) check
        return TgtReport(C.TGT_KIND_BY_CODE[match.group('code')], lat, lon,
                         int(match.group('conf')), int(match.group('seq')))
    except (ValueError, UnicodeError, TypeError, OverflowError):
        return None
