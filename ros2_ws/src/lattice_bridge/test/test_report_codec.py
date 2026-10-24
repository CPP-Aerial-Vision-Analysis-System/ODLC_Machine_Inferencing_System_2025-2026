"""
lattice_bridge/test/test_report_codec.py -- PURPOSE: prove the TGT radio message. The encoder must make exactly
the documented text and never one over 50 characters; the decoder must accept exactly that text and nothing
else, never raise, never run or format what it reads (it is UNTRUSTED radio input), and survive tens of
thousands of random and mutated messages. Round trips must be exact.
"""

import ast
import math
import random
import string
from pathlib import Path

import pytest

from lattice_bridge import constants as C
from lattice_bridge import report_codec as R
from lattice_bridge.report_codec import TgtReport, decode_tgt, encode_tgt, next_seq
from support import rng

EXAMPLE = 'TGT p 36.214935 -96.006203 95 12'


# ------------------------------------------------------------------------------------ the documented examples
def test_the_documented_example_encodes_exactly():
    assert encode_tgt('person', 36.214935, -96.006203, 0.95, 12) == EXAMPLE


def test_the_documented_example_decodes_exactly():
    report = decode_tgt(EXAMPLE)
    assert report == TgtReport('person', 36.214935, -96.006203, 95, 12)
    assert report.confidence == 0.95 and report.confidence_percent == 95


def test_tent_message():
    assert encode_tgt('tent', 36.21502595, -96.00680007, 0.85, 0) == 'TGT t 36.215026 -96.006800 85 0'
    assert decode_tgt('TGT t 36.215026 -96.006800 85 0').kind == 'tent'


def test_kind_codes_are_accepted_by_the_encoder():
    assert encode_tgt('p', 36.2, -96.0, 0.5, 1) == encode_tgt('person', 36.2, -96.0, 0.5, 1)
    assert encode_tgt('t', 36.2, -96.0, 0.5, 1) == encode_tgt('tent', 36.2, -96.0, 0.5, 1)


def test_coordinates_always_have_six_decimals_and_negative_zero_never_appears():
    assert encode_tgt('person', 5.0, 7.0, 0.5, 1) == 'TGT p 5.000000 7.000000 50 1'
    assert encode_tgt('person', -1e-9, 7.0, 0.5, 1) == 'TGT p 0.000000 7.000000 50 1'
    assert encode_tgt('person', 5.0, -4e-7, 0.5, 1) == 'TGT p 5.000000 0.000000 50 1'


def test_confidence_is_rounded_to_whole_percent():
    for conf, percent in [(0.0, 0), (0.004, 0), (0.006, 1), (0.954, 95), (0.956, 96), (1.0, 100)]:
        assert encode_tgt('person', 36.2, -96.0, conf, 1).split()[4] == str(percent)


# ------------------------------------------------------------------------------------ length
def test_the_longest_possible_message_fits_in_a_statustext():
    worst = encode_tgt('person', -89.999999, -179.999999, 1.0, C.TGT_SEQ_MAX)
    assert worst == 'TGT p -89.999999 -179.999999 100 999999'
    assert len(worst) <= C.STATUSTEXT_MAX_CHARS
    widest = encode_tgt('person', -90.0, -180.0, 1.0, C.TGT_SEQ_MAX)
    assert len(widest) == 39 <= C.STATUSTEXT_MAX_CHARS and decode_tgt(widest) is not None


def test_encoded_messages_are_always_short_ascii_and_round_trip():
    r = rng(40)
    for _ in range(5000):
        kind = r.choice(['person', 'tent'])
        lat = r.uniform(-90, 90) or 1.0
        lon = r.uniform(-180, 180) or 1.0
        conf = r.random()
        seq = r.randint(0, C.TGT_SEQ_MAX)
        text = encode_tgt(kind, lat, lon, conf, seq)
        assert len(text) <= C.STATUSTEXT_MAX_CHARS and text.isascii() and text.isprintable()
        report = decode_tgt(text)
        assert report is not None
        assert report.kind == kind and report.seq == seq
        assert report.lat_deg == pytest.approx(lat, abs=5.1e-7) and report.lon_deg == pytest.approx(lon, abs=5.1e-7)
        assert abs(report.confidence_percent - conf * 100) <= 0.5 + 1e-9
        assert encode_tgt(report.kind, report.lat_deg, report.lon_deg, report.confidence, report.seq) == text


def test_boundary_values_round_trip():
    for lat, lon in [(90.0, 180.0), (-90.0, -180.0), (90.0, -180.0), (0.000001, 0.000001), (-0.000001, -0.000001),
                     (36.0, 0.0), (0.0, 36.0)]:
        for conf in (0.0, 1.0):
            for seq in (0, 1, C.TGT_SEQ_MAX):
                report = decode_tgt(encode_tgt('tent', lat, lon, conf, seq))
                assert report is not None and (report.lat_deg, report.lon_deg) == (lat, lon)


# ------------------------------------------------------------------------------------ encoder refuses bad input
@pytest.mark.parametrize('args', [
    ('dog', 36.2, -96.0, 0.9, 1), (None, 36.2, -96.0, 0.9, 1), (['person'], 36.2, -96.0, 0.9, 1),
    ('person', 0.0, 0.0, 0.9, 1),
    ('person', float('nan'), -96.0, 0.9, 1), ('person', 36.2, float('inf'), 0.9, 1), ('person', 91, -96.0, 0.9, 1),
    ('person', 36.2, -181, 0.9, 1), ('person', None, None, 0.9, 1),
    ('person', 36.2, -96.0, 1.1, 1), ('person', 36.2, -96.0, -0.1, 1), ('person', 36.2, -96.0, float('nan'), 1),
    ('person', 36.2, -96.0, '0.9', 1), ('person', 36.2, -96.0, True, 1),
    ('person', 36.2, -96.0, 0.9, -1), ('person', 36.2, -96.0, 0.9, C.TGT_SEQ_MAX + 1),
    ('person', 36.2, -96.0, 0.9, 1.0), ('person', 36.2, -96.0, 0.9, '1'), ('person', 36.2, -96.0, 0.9, True),
    ('person', 36.2, -96.0, 0.9, None),
])
def test_the_encoder_raises_value_error_for_anything_it_cannot_send_faithfully(args):
    with pytest.raises(ValueError):
        encode_tgt(*args)


def test_next_seq_wraps():
    assert next_seq(0) == 1 and next_seq(41) == 42
    assert next_seq(C.TGT_SEQ_MAX) == 0
    for bad in (-1, C.TGT_SEQ_MAX + 1, 1.0, '1', None, True):
        with pytest.raises(ValueError):
            next_seq(bad)


# ------------------------------------------------------------------------------------ the decoder is strict
@pytest.mark.parametrize('text', [
    'TGT p 36.214935 -96.006203 95 12 ',                  # trailing space
    ' TGT p 36.214935 -96.006203 95 12',                  # leading space
    'TGT p 36.214935 -96.006203 95 12\n',                 # trailing newline
    'TGT p 36.214935 -96.006203 95 12\r\n',
    'TGT  p 36.214935 -96.006203 95 12',                  # double space
    'TGT\tp 36.214935 -96.006203 95 12',                  # tab
    'tgt p 36.214935 -96.006203 95 12',                   # lower case word
    'Tgt p 36.214935 -96.006203 95 12',
    'TGT P 36.214935 -96.006203 95 12',                   # upper case kind
    'TGT x 36.214935 -96.006203 95 12',                   # unknown kind
    'TGT person 36.214935 -96.006203 95 12',
    'TGT p 36.21493 -96.006203 95 12',                    # 5 decimals
    'TGT p 36.2149350 -96.006203 95 12',                  # 7 decimals
    'TGT p 36 -96.006203 95 12',                          # no decimals
    'TGT p +36.214935 -96.006203 95 12',                  # explicit plus
    'TGT p 036.214935 -96.006203 95 12',                  # leading zero
    'TGT p 36.214935 -096.006203 95 12',
    'TGT p 36.214935 -96.006203 095 12',
    'TGT p 36.214935 -96.006203 95 012',
    'TGT p 36.214935 -96.006203 101 12',                  # confidence over 100
    'TGT p 36.214935 -96.006203 -5 12',
    'TGT p 36.214935 -96.006203 95.5 12',
    'TGT p 36.214935 -96.006203 95 -1',                   # negative seq
    'TGT p 36.214935 -96.006203 95 1000000',              # seq out of range
    'TGT p 36.214935 -96.006203 95 1.5',
    'TGT p 91.000000 -96.006203 95 12',                   # latitude out of range
    'TGT p -90.000001 -96.006203 95 12',
    'TGT p 36.214935 180.000001 95 12',                   # longitude out of range
    'TGT p 36.214935 -181.000000 95 12',
    'TGT p 0.000000 0.000000 95 12',                      # (0, 0) = no GPS fix
    'TGT p -0.000000 5.000000 95 12',                     # negative zero
    'TGT p 5.000000 -0.000000 95 12',
    'TGT p 36.214935 -96.006203 95',                      # missing field
    'TGT p 36.214935 -96.006203',
    'TGT p 36.214935',
    'TGT p',
    'TGT',
    '',
    'TGT p 36.214935 -96.006203 95 12 extra',
    'XTGT p 36.214935 -96.006203 95 12',
    'TGT p 36.214935 -96.006203 95 12;ls',
    'TGT p 36.214935 -96.006203 95 12 %s',
    'TGT p {0} {1} 95 12',
    'TGT p 36.214935 -96.006203 9E1 12',
    'TGT p 36.214935 -96.006203 0x5f 12',
    'TGT p nan -96.006203 95 12',
    'TGT p inf -96.006203 95 12',
    'TGT p 1e1.000000 -96.006203 95 12',
    'TGT p ٣٦.٢١٤٩٣٥ -96.006203 95 12',                  # Arabic-Indic digits
    'TGT p ３６.２１４９３５ -96.006203 95 12',              # full-width digits
    'TGT p 36.214935 -96.006203 95 12\x00x',              # NUL then more text
    'TGT\x00p 36.214935 -96.006203 95 12',
    'TGT p 36.214935 -96.006203 95 12​',             # zero-width space
    'TGT p 36.214935 -96.006203 95 12é',
    'A' * 51,
    'TGT p 36.214935 -96.006203 95 12' + ' ' * 40,        # padded past 50
])
def test_anything_but_the_canonical_form_is_rejected(text):
    assert decode_tgt(text) is None


@pytest.mark.parametrize('junk', [None, 5, 5.5, True, [], {}, (), set(), object(), [EXAMPLE], {'text': EXAMPLE},
                                  b'\xff\xfe', b'TGT p 36.214935 -96.006203 95 12\xc3\xa9', bytearray(b'\xff'),
                                  float('nan'), 10 ** 400, memoryview(b'TGT')])
def test_non_text_input_gives_none(junk):
    assert decode_tgt(junk) is None


def test_the_nul_padding_of_a_wire_field_is_tolerated_and_only_that():
    assert decode_tgt(EXAMPLE + '\x00' * 10) == decode_tgt(EXAMPLE)
    assert decode_tgt((EXAMPLE + '\x00' * 17).encode('ascii')) == decode_tgt(EXAMPLE)
    assert decode_tgt(bytearray(EXAMPLE, 'ascii')) == decode_tgt(EXAMPLE)
    assert decode_tgt(('\x00' + EXAMPLE)) is None


def test_bytes_that_are_not_ascii_are_rejected_not_raised():
    assert decode_tgt(b'TGT p 36.214935 -96.006203 95 12\x80') is None
    assert decode_tgt('TGT p 36.214935 -96.006203 95 12\udcff') is None             # a lone surrogate


def test_a_huge_text_is_rejected_quickly():
    assert decode_tgt('TGT ' + 'A' * 10_000_000) is None
    assert decode_tgt(('9' * 1_000_000)) is None


def test_exactly_fifty_characters_is_the_limit():
    ok = 'TGT p 36.214935 -96.006203 95 12'
    assert len(ok) < 50 and decode_tgt(ok) is not None
    assert decode_tgt(ok + '\x00' * 30) is not None                                    # padding does not count


# ------------------------------------------------------------------------------------ fuzzing
ALPHABET = string.printable + 'éß٣３​\x00\udcff'


def test_random_text_never_raises_and_almost_never_decodes():
    r = random.Random(41)
    decoded = 0
    for _ in range(20000):
        text = ''.join(r.choice(ALPHABET) for _ in range(r.randint(0, 60)))
        report = decode_tgt(text)
        if report is not None:
            decoded += 1
    assert decoded == 0


def test_random_bytes_never_raise():
    r = random.Random(42)
    for _ in range(10000):
        data = bytes(r.randrange(256) for _ in range(r.randint(0, 70)))
        decode_tgt(data)
        decode_tgt(bytearray(data))


def test_random_tgt_shaped_text_never_raises_and_only_decodes_in_range():
    r = random.Random(43)
    pieces = ['TGT', 'p', 't', 'x', '36.214935', '-96.006203', '91.000000', '-0.000000', '0', '95', '101', '12',
              '999999', '1000000', '', ' ', '-', '.', '1e5', 'nan', '٣']
    for _ in range(20000):
        text = ' '.join(r.choice(pieces) for _ in range(r.randint(1, 7)))
        report = decode_tgt(text)
        if report is not None:
            assert -90 <= report.lat_deg <= 90 and -180 <= report.lon_deg <= 180
            assert 0 <= report.confidence_percent <= 100 and 0 <= report.seq <= C.TGT_SEQ_MAX
            assert report.kind in ('person', 'tent')


def test_single_character_mutations_of_a_valid_message_stay_safe():
    """Flip, drop, insert or replace one character at every position: never raises; whatever decodes is in range."""
    bases = [EXAMPLE, 'TGT t -33.868820 151.209290 7 999999', 'TGT p -89.999999 -179.999999 100 0']
    for base in bases:
        for index in range(len(base) + 1):
            for replacement in ['', '0', '9', ' ', '-', '.', 'x', 'é', '\x00', '5']:
                variants = [base[:index] + replacement + base[index + 1:], base[:index] + replacement + base[index:]]
                for text in variants:
                    report = decode_tgt(text)
                    if report is not None:
                        assert -90 <= report.lat_deg <= 90 and -180 <= report.lon_deg <= 180
                        assert 0 <= report.confidence_percent <= 100 and 0 <= report.seq <= C.TGT_SEQ_MAX
                        assert text.rstrip('\x00') == encode_tgt(report.kind, report.lat_deg, report.lon_deg,
                                                                 report.confidence, report.seq)


def test_decoded_values_always_re_encode_to_the_same_text():
    """The canonical form is unique: if it decodes, encoding the result gives the identical text."""
    r = random.Random(45)
    seen = 0
    for _ in range(20000):
        kind = r.choice('pt')
        lat = f'{r.choice(["", "-"])}{r.randint(0, 99)}.{r.randint(0, 999999):06d}'
        lon = f'{r.choice(["", "-"])}{r.randint(0, 199)}.{r.randint(0, 999999):06d}'
        text = f'TGT {kind} {lat} {lon} {r.randint(0, 105)} {r.randint(0, 1100000)}'
        report = decode_tgt(text)
        if report is not None:
            seen += 1
            assert encode_tgt(report.kind, report.lat_deg, report.lon_deg, report.confidence, report.seq) == text
    assert seen > 100


# ------------------------------------------------------------------------------------ the text is never run
def test_the_codec_never_evaluates_or_formats_what_it_reads():
    """Static check: no eval/exec/compile/__import__/getattr/format/% on data in the codec module."""
    source = Path(R.__file__).read_text(encoding='utf-8')
    tree = ast.parse(source)
    banned_calls = {'eval', 'exec', 'compile', '__import__', 'getattr', 'setattr', 'open', 'input', 'format',
                    'globals', 'locals', 'vars'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            name = func.id if isinstance(func, ast.Name) else func.attr if isinstance(func, ast.Attribute) else ''
            if isinstance(func, ast.Attribute) and name == 'compile':
                continue                                          # re.compile of our own fixed pattern is fine
            assert name not in banned_calls, f'banned call {name}() at line {node.lineno}'
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mod) and isinstance(node.left, ast.Constant) \
                and isinstance(node.left.value, str):
            pytest.fail(f'% string formatting at line {node.lineno}')
    decode_function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'decode_tgt')
    assert not [n for n in ast.walk(decode_function) if isinstance(n, ast.JoinedStr)], 'f-string in decode_tgt'


def test_hostile_looking_text_is_just_text():
    for text in ['__import__("os").system("true")', '${jndi:ldap://x}', '%s%s%s%s', '{0.__class__}', '`id`',
                 "'; DROP TABLE targets; --", 'TGT p 36.214935 -96.006203 95 12; rm -rf /']:
        assert decode_tgt(text) is None


def test_codec_and_constants_agree():
    assert R.C.STATUSTEXT_MAX_CHARS == 50
    assert math.isclose(TgtReport('tent', 1.0, 2.0, 50, 0).confidence, 0.5)
