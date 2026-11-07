"""
sim_plugins/test/test_model_check.py -- PURPOSE: unit tests of the detector model check (model_check.py) without
PyTorch: a missing / empty / misnamed / broken weights file gives an error that says what, why and the fix; a
TensorRT .engine on a computer without TensorRT is refused (or warned when a .pt twin exists); the SHA-256
fingerprint is right; the tile estimate matches new_od's tiling; the class-name coverage warns when the model has
nothing the team code turns into a person or a tent; the copy of the team's class map matches the team file; and
the command line exits 0 / 3 as documented.
"""

import hashlib
import json
import re
from pathlib import Path

from sim_plugins import model_check as mc

REPO = Path(__file__).resolve().parents[5]
ZIP_HEAD = b'PK\x03\x04' + b'\0' * 60          # how a torch >= 1.6 .pt file starts (a zip archive)


def write(path: Path, data: bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


def no_modules(_name):
    return False


def all_modules(_name):
    return True


# ---- quick_check ------------------------------------------------------------------------------------------

def test_missing_file_says_what_why_and_fix(tmp_path):
    rep = mc.quick_check(str(tmp_path / 'nope.pt'))
    assert not rep.ok
    text = rep.errors[0]
    assert 'does not exist' in text and 'Likely cause' in text and 'Fix:' in text and '--model' in text


def test_no_file_given():
    rep = mc.quick_check('')
    assert not rep.ok and 'no model file' in rep.errors[0]


def test_empty_file(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'm.pt', b'')))
    assert not rep.ok and '0 bytes' in rep.errors[0]


def test_space_in_path_is_refused(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'my models' / 'm.pt', ZIP_HEAD)))
    assert any('contains a space' in e and 'Fix:' in e for e in rep.errors)


def test_unknown_extension(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'm.h5', b'abc')))
    assert not rep.ok and rep.format == 'unknown' and '.h5' in rep.errors[0] and 'Fix:' in rep.errors[0]


def test_good_pt_has_size_format_and_sha256(tmp_path):
    data = ZIP_HEAD + b'weights' * 100
    path = write(tmp_path / 'my_detector.pt', data)
    rep = mc.quick_check(str(path))
    assert rep.ok, rep.errors
    assert rep.format == 'pytorch' and rep.size_bytes == len(data) and rep.name == 'my_detector.pt'
    assert rep.sha256 == hashlib.sha256(data).hexdigest()
    d = rep.to_dict()
    assert d['ok'] is True and d['sha256'] == rep.sha256 and 'size_mb' in d
    assert 'my_detector.pt' in rep.render()


def test_no_hash_skips_the_fingerprint(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'm.pt', ZIP_HEAD)), compute_hash=False)
    assert rep.ok and rep.sha256 == '' and 'sha256' not in rep.render()


def test_old_pickle_pt_is_accepted(tmp_path):
    assert mc.quick_check(str(write(tmp_path / 'm.pt', b'\x80\x02' + b'\0' * 20))).ok


def test_html_saved_as_pt_is_refused(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'm.pt', b'<!DOCTYPE html><html>')))
    assert not rep.ok and 'web page' in rep.errors[0] and 'Fix:' in rep.errors[0]
    assert rep.render().startswith(' XX model m.pt')                # never " OK" for a model that cannot be used


def test_engine_next_to_pt_is_warned(tmp_path):
    pt = write(tmp_path / 'm.pt', ZIP_HEAD)
    write(tmp_path / 'm.engine', b'engine')
    rep = mc.quick_check(str(pt))
    assert rep.ok and any('loads the .engine' in w for w in rep.warnings)


def test_engine_without_tensorrt_and_no_twin_is_refused(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'm.engine', b'engine')), find_module=no_modules)
    assert not rep.ok and rep.format == 'tensorrt'
    assert 'no TensorRT' in rep.errors[0] and 'Jetson' in rep.errors[0] and 'Fix:' in rep.errors[0]


def test_engine_without_tensorrt_with_pt_twin_is_a_warning(tmp_path):
    write(tmp_path / 'm.pt', ZIP_HEAD)
    rep = mc.quick_check(str(write(tmp_path / 'm.engine', b'engine')), find_module=no_modules)
    assert rep.ok and any('falls back to m.pt' in w for w in rep.warnings)


def test_engine_with_tensorrt_is_noted(tmp_path):
    rep = mc.quick_check(str(write(tmp_path / 'm.engine', b'engine')), find_module=all_modules)
    assert rep.ok and any('TensorRT engine' in n for n in rep.notes)


def test_onnx_needs_onnxruntime(tmp_path):
    path = str(write(tmp_path / 'm.onnx', b'onnx'))
    assert not mc.quick_check(path, find_module=no_modules).ok
    rep = mc.quick_check(path, find_module=all_modules)
    assert rep.ok and any('never tested with ONNX' in w for w in rep.warnings)


# ---- estimates and classes ---------------------------------------------------------------------------------

def test_tiles_per_photo_matches_new_ods_tiling():
    # 640 px tiles, 15 % overlap (step 544): 2560 -> 5 columns, 1440 -> 3 rows, + SAHI's full-photo pass
    assert mc.tiles_per_photo(2560, 1440) == 5 * 3 + 1
    assert mc.tiles_per_photo(3840, 2160) == 7 * 4 + 1          # 4K: 7 columns, 4 rows
    assert mc.tiles_per_photo(640, 640) == 2


def test_class_coverage_of_a_stock_coco_model():
    cov = mc.class_coverage(['person', 'bicycle', 'umbrella', 'kite'])
    assert cov == {'person': ['person'], 'tent': ['umbrella', 'kite'], 'ignored_tent_like': []}
    warnings, notes = mc.coverage_findings(['person', 'umbrella'])
    assert warnings == [] and 'person <- person' in notes[0]


def test_a_model_without_targets_is_warned():
    warnings, _ = mc.coverage_findings(['car', 'truck'])
    assert any('PERSON' in w for w in warnings) and any('TENT' in w for w in warnings)


def test_a_class_called_tent_is_ignored_by_the_team_code():
    warnings, _ = mc.coverage_findings(['person', 'tent'])
    assert any('"umbrella"' in w and 'IGNORED' in w for w in warnings)


def test_team_class_map_copy_matches_the_team_file():
    src = (REPO / 'ros2_ws' / 'src' / 'detection' / 'detection' / 'detection_processor.py').read_text(encoding='utf-8')
    block = src.split('CLASS_MAP = {', 1)[1].split('\n}', 1)[0]
    team = dict(re.findall(r"'(\w+)':\s*\('(\w+)'", block))
    assert team == mc.TEAM_CLASS_MAP


def test_load_check_is_skipped_when_the_file_is_bad(tmp_path):
    rep = mc.quick_check(str(tmp_path / 'missing.pt'))
    before = list(rep.errors)
    assert mc.load_check(rep).errors == before and rep.load == {}


# ---- command line -----------------------------------------------------------------------------------------

def test_cli_exit_codes_and_json(tmp_path, capsys):
    good = write(tmp_path / 'm.pt', ZIP_HEAD)
    out = tmp_path / 'out' / 'report.json'
    assert mc.main([str(good), '--json', str(out)]) == 0
    data = json.loads(out.read_text(encoding='utf-8'))
    assert data['ok'] is True and data['format'] == 'pytorch' and len(data['sha256']) == 64
    assert mc.main([str(tmp_path / 'missing.pt')]) == mc.EXIT_UNUSABLE == 3
    err = capsys.readouterr().err
    assert '[model_check] ERROR' in err
