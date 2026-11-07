"""test_hil_pc.py -- PURPOSE: unit tests of the PC side's helper hil_pc.py (no ROS, no board; plain Python on Windows
or in WSL): the photo pace and mirror speed limits for a shared SSH login, the clock offset from SSH samples and its
interpolation, the photo publishing into a watched folder (sidecar first, times moved to the PC clock, never a
half-written file, nothing published twice), the tegrastats parser and the run summary with its "Differences from
the field" list.

Run:  python -m pytest simulation/jetson/pc/test/test_hil_pc.py -q
"""
import importlib.util
import json
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PC = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location('hil_pc', os.path.join(PC, 'hil_pc.py'))
hil_pc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hil_pc)


# ---- pace / bwlimit -------------------------------------------------------------------------------------------------

def test_pace_rules():
    p = hil_pc.photo_pace_mbit
    assert p('off', 'login', 'zr10', 200) == 0
    assert p('25', 'login', 'zr10', 200) == 25
    assert p('auto', 'key', 'zr10', 200) == 0                 # key mode: photos have their own connection
    assert p('auto', 'login', 'zr10', 100) == 60              # 60 % of 100
    assert p('auto', 'login', 'zr10', 300) == 80              # capped at 80
    assert p('auto', 'login', 'a8mini', 20) == 28.5           # never below 1.5 x 19 Mbit/s
    assert p('auto', 'login', 'zr10', 20) == 16.5             # never below 1.5 x 11 Mbit/s
    assert p('auto', 'login', 'zr10', None) == 80             # not measured: the cap
    with pytest.raises(ValueError):
        p('fast', 'login', 'zr10', None)


def test_bwlimit():
    assert hil_pc.mirror_bwlimit_kbps('key', 100) == 0
    assert hil_pc.mirror_bwlimit_kbps('login', None) == 0
    assert hil_pc.mirror_bwlimit_kbps('login', 100) == 5000   # 40 % of 100 Mbit/s = 40 Mbit/s = 5000 KB/s
    assert hil_pc.mirror_bwlimit_kbps('login', 1) == 256      # floor


def test_speed_file(tmp_path):
    f = tmp_path / 'speed.env'
    f.write_text('THROUGHPUT_MBIT=123.4\nRTT_MS=1.2\n')
    assert hil_pc.measured_mbit(str(f)) == 123.4
    assert hil_pc.measured_mbit(str(tmp_path / 'missing')) is None


# ---- clock ----------------------------------------------------------------------------------------------------------

def test_offset_from_samples_uses_shortest_round_trip():
    samples = [(100.0, 105.30, 100.20),       # rtt 200 ms -> offset 5.20
               (101.0, 106.005, 101.01),      # rtt 10 ms  -> offset 5.000 (the one that counts)
               (102.0, 107.4, 102.5)]
    off, rtt = hil_pc.offset_from_samples(samples)
    assert off == pytest.approx(5.0, abs=1e-6)
    assert rtt == pytest.approx(10.0, abs=1e-6)
    with pytest.raises(ValueError):
        hil_pc.offset_from_samples([(5.0, 1.0, 4.0)])


def test_parse_clock_line():
    assert hil_pc.parse_clock_line('offset_s=-0.25 rtt_ms=1.5') == (-0.25, 1.5)
    with pytest.raises(ValueError):
        hil_pc.parse_clock_line('Not logged in')


def test_record_and_interpolate(tmp_path):
    f = str(tmp_path / 'jetson' / 'clock.txt')
    hil_pc.record_clock(f, 'start', 2.0, 1.0, t_unix=1000.0)
    c = hil_pc.read_env(f)
    assert c['offset_s'] == '2.0000'
    assert hil_pc.offset_at(c, 1500.0) == 2.0                 # one measurement: that offset
    hil_pc.record_clock(f, 'end', 3.0, 2.0, t_unix=2000.0)
    c = hil_pc.read_env(f)
    assert c['offset_s'] == '2.5000' and c['rtt_ms'] == '1.00'
    # halfway in PC time (jetson time = pc + offset): pc 1500 -> jetson ~1502
    assert hil_pc.offset_at(c, 1502.0) == pytest.approx(2.5, abs=0.01)
    assert hil_pc.offset_at(c, 0.0) == 2.0                    # clamped before the start
    hil_pc.record_clock(f, 'end', 2.1, 2.0, t_unix=2000.0)    # within 0.2 s: no interpolation
    assert hil_pc.offset_at(hil_pc.read_env(f), 1502.0) == 2.0
    assert hil_pc.offset_at({}, 5.0) == 0.0


# ---- publish --------------------------------------------------------------------------------------------------------

def _photo(d, name, mtime, data=b'jpeg'):
    p = d / name
    p.write_bytes(data)
    os.utime(p, (mtime, mtime))
    return p


def test_publish_shifts_times_sidecar_first_and_once(tmp_path, monkeypatch):
    staging, dest = tmp_path / 'staging', tmp_path / 'photos'
    staging.mkdir()
    clock = tmp_path / 'clock.txt'
    hil_pc.record_clock(str(clock), 'start', 3.0, 1.0, t_unix=1000.0)   # Jetson 3 s ahead
    _photo(staging, '36.1 , -96.1.jpg', 2000.0)
    _photo(staging, '36.1 , -96.1.json', 2000.5, b'{}')
    _photo(staging, '.36.2 , -96.2.jpg.XyZ12', 2001.0)                    # rsync's temporary file: ignored
    order = []
    real_replace = os.replace
    monkeypatch.setattr(hil_pc.os, 'replace', lambda a, b: (order.append(os.path.basename(b)), real_replace(a, b)))
    state = str(tmp_path / 'state.json')
    n = hil_pc.publish_once(str(staging), str(dest), str(clock), state, now=5000.0)
    assert n == 2
    names = [o for o in order if not o.startswith('state')]
    assert names[:2] == ['36.1 , -96.1.json', '36.1 , -96.1.jpg']         # sidecar BEFORE the photo
    assert os.path.getmtime(dest / '36.1 , -96.1.jpg') == pytest.approx(1997.0, abs=0.01)   # on the PC clock
    assert sorted(os.listdir(dest)) == ['36.1 , -96.1.jpg', '36.1 , -96.1.json']           # no temp files left
    assert hil_pc.publish_once(str(staging), str(dest), str(clock), state, now=5001.0) == 0  # nothing twice
    _photo(staging, '36.1 , -96.1.jpg', 2010.0, b'jpeg-v2')                                  # changed: again
    assert hil_pc.publish_once(str(staging), str(dest), str(clock), state, now=5002.0) == 1


def test_photo_waits_for_its_sidecar(tmp_path):
    staging, dest = tmp_path / 's', tmp_path / 'd'
    staging.mkdir()
    _photo(staging, 'a , b.jpg', 100.0)
    state = str(tmp_path / 'st.json')
    assert hil_pc.publish_once(str(staging), str(dest), '', state, now=1000.0) == 0   # no sidecar yet
    assert hil_pc.publish_once(str(staging), str(dest), '', state, now=1003.0) == 0
    assert hil_pc.publish_once(str(staging), str(dest), '', state, now=1006.5) == 1   # gave up waiting (6 s)
    assert json.load(open(state))['published']['a , b.jpg'][0] == 4


def test_plan_publish_order():
    staging = {'p.jpg': (1, 1.0), 'p.json': (1, 1.0), 'q.json': (1, 1.0)}
    assert hil_pc.plan_publish(staging, {}, {}, now=0.0) == ['p.json', 'q.json', 'p.jpg']


# ---- summary --------------------------------------------------------------------------------------------------------

TEGRA = ('RAM 3456/7620MB (lfb 2x4MB) SWAP 0/3810MB CPU [12%@1510] GR3D_FREQ 40% cpu@48.5C soc2@47.1C tj@49C\n'
         'RAM 5000/7620MB CPU [50%@1510] GR3D_FREQ 80% cpu@55.0C tj@56.2C VDD_IN 5000mW/5000mW\n')


def test_parse_tegrastats():
    t = hil_pc.parse_tegrastats(TEGRA)
    assert t == {'samples': 2, 'gpu_avg': 60.0, 'gpu_peak': 80, 'temp_peak_c': 56.2, 'mem_free_min_mb': 2620}
    assert hil_pc.parse_tegrastats('') == {'samples': 0}


def test_summary_has_field_differences_power_and_clock_warning(tmp_path):
    run = tmp_path / 'run'
    jd = run / 'jetson'
    jd.mkdir(parents=True)
    (jd / 'status_start.json').write_text(json.dumps({'image': 'astra-hil:jetson', 'git_sha': 'abc', 'engine': 'ok',
                                                      'power_mode': 'MAXN_SUPER (2)', 'clocks': 'not locked'}))
    (jd / 'tegrastats.log').write_text(TEGRA)
    hil_pc.record_clock(str(jd / 'clock.txt'), 'start', 0.9, 1.0, t_unix=1.0)
    (jd / 'relay.status').write_text('MAV_DROPPED_BYTES=42\nMAX_GAP_S=0.8\nREDIALS=1\n')
    (jd / 'link_state.env').write_text('DROPS=1\nDOWN_S=20\n')
    opts = {'link': 'usb', 'auth': 'login', 'host': '192.168.55.1', 'user': 'astra-dev', 'photos': 'live',
            'pace': '80', 'baud': 'off', 'connected': ['legacy photo pipeline'],
            'extra': hil_pc.run_extras('usb', 'zr10', 'live', 'off', False, 'legacy')}
    text = hil_pc.build_summary(str(run), opts)
    assert '## Differences from the field' in text
    assert text.count('\n1. ') == 1 and f'{len(hil_pc.FIELD_DIFFERENCES)}. ' in text
    assert 'MAXN_SUPER (2)' in text and 'GPU load average 60.0' in text
    assert 'WARNING' in text and '0.90 s off' in text                     # above the 0.5 s threshold
    assert 'bytes the gate dropped (MAVROS commands never sent late): 42' in text and 'SSH link drops: 1' in text
    assert 'no 115200-baud limit' in text and 'legacy (2025) photo pipeline' in text
    assert 'backend was not reported' in text and 'not recorded (RUN_DIR/jetson/versions.env' in text
    assert 'photo pace: 80 Mbit/s' in text


def test_summary_of_a_virtual_run_with_a_drop_rehearsal(tmp_path):
    """What the first virtual-Jetson flight (2026-10-31 15:32) left: a 20 s blackhole seen by the relay, no SSH, no
    photo pace, no board facts; new_od's log lines carry colour codes."""
    jd = tmp_path / 'jetson'
    jd.mkdir()
    (jd / 'status_start.json').write_text(json.dumps({'image': 'sha256:bc79', 'engine': 'missing', 'l4t': ''}))
    (jd / 'relay.status').write_text('MAV=up\nREDIALS=4\nMAV_DROPPED_BYTES=5620\nMAX_GAP_S=2.429\nMAV_DROPS=1\n'
                                     'MAV_DOWN_S=20.014\nSDK_DROPPED=0\n')
    (jd / 'link_state.env').write_text('LINK=virtual\nDROPS=0\nDOWN_S=0\n')
    (tmp_path / 'team_stack.log').write_text('[new_od-5] \x1b[0m[INFO] CUDA GPU detected: NVIDIA GeForce RTX 4060 '
                                             'Laptop GPU\x1b[0m\n[new_od-5] [INFO] Device: cuda:0\x1b[0m\n'
                                             '[new_od-5] [INFO] PyTorch model loaded via SAHI\n')
    text = hil_pc.build_summary(str(tmp_path), {'link': 'virtual', 'auth': 'login', 'pace': ''})
    assert 'autopilot silent (seen by the relay in the container): 1 time(s), 20.014 s in total' in text
    assert 'SSH link drops: 0' in text and 'bytes the gate dropped (MAVROS commands never sent late): 5620' in text
    assert 'photo pace: not used (virtual link' in text
    assert 'L4T: not applicable (virtual Jetson' in text and 'power mode** at the start: not applicable' in text
    assert '**detector backend**: PyTorch on cuda:0 (NVIDIA GeForce RTX 4060 Laptop GPU)\n' in text


def test_summary_backend_and_versions(tmp_path):
    jd = tmp_path / 'jetson'
    jd.mkdir()
    (jd / 'status_start.json').write_text(json.dumps({'detector_backend': 'onnxruntime-cuda',
                                                      'versions': {'TENSORRT_PY': '10.3.0', 'MAVROS': '2.12.0'}}))
    text = hil_pc.build_summary(str(tmp_path), {'link': 'virtual'})
    assert '| TensorRT (Python) | 10.3.0 | 10.3.0 | yes |' in text and '| MAVROS | 2.12.0 | 2.12.0 | yes |' in text
    assert 'ran on "onnxruntime-cuda", not TensorRT' in text
    (jd / 'status_start.json').write_text(json.dumps({'detector_backend': 'TensorRT 10.3 (engine built here)'}))
    assert 'not TensorRT' not in hil_pc.build_summary(str(tmp_path), {'link': 'usb'})


def test_summary_versions_env_and_backend_from_team_log(tmp_path):
    """versions.env (container_versions.py output) gives the table; new_od's own log lines give the backend."""
    jd = tmp_path / 'jetson'
    jd.mkdir()
    (jd / 'status_start.json').write_text(json.dumps({'image': 'astra-hil:virtual'}))
    (jd / 'versions.env').write_text('OS=Ubuntu 22.04.4 LTS\nARCH=x86_64\nTORCH_CUDA=12.6\nTENSORRT_PY=10.3.0\n'
                                     'MAVROS=2.14.0\nGPU=NVIDIA GeForce RTX 4060 Laptop GPU\nTORCH=2.8.0+cu126\n'
                                     'MODEL_ENGINE=/models/yolo26m.engine 52 MiB\n')
    (tmp_path / 'team_stack.log').write_text('[new_od]: CUDA GPU detected: NVIDIA GeForce RTX 4060 Laptop GPU\n'
                                             '[new_od]: Device: cuda:0\n[new_od]: TensorRT model loaded and wrapped\n')
    text = hil_pc.build_summary(str(tmp_path), {'link': 'virtual'})
    assert '**detector backend**: TensorRT on cuda:0 (NVIDIA GeForce RTX 4060 Laptop GPU)' in text
    assert '| CUDA (PyTorch build) | 12.6 | 12.6 | yes |' in text
    assert '| MAVROS | 2.14.0 | 2.12.0 | no |' in text
    assert '| PyTorch | 2.8.0+cu126 | 2.8.0 | yes |' in text
    assert '| CPU architecture | x86_64 | aarch64 | no |' in text
    assert '| GPU | NVIDIA GeForce RTX 4060 Laptop GPU | Orin' in text and '| - |' in text
    assert '| numpy | unknown | 1.26.4 | ? |' in text
    assert 'not TensorRT' not in text and '/models/yolo26m.engine 52 MiB' in text


def test_detector_backend_from_log():
    f = hil_pc.detector_backend_from_log
    assert f('') == ''
    assert f('Device: cpu\nPyTorch model loaded via SAHI\n') == 'PyTorch on cpu'
    assert f('Device: cuda:0\nTensorRT model loaded (batched path, SAHI bypassed)') == 'TensorRT on cuda:0'
    assert f('TensorRT model loaded\nFailed to load model\nPyTorch fallback model loaded').startswith('PyTorch (fallback')
    assert f('Model ready -- processing can begin') == ''


def test_version_rows_matching():
    rows = {r[0]: r for r in hil_pc.version_rows({'TORCH_CUDA': '12.6', 'PYTHON': '3.10.12', 'ROS_DISTRO': 'humble',
                                                    'TORCH_CUDNN': '9.3.0', 'MAVROS': '2.14.0'})}
    assert rows['CUDA (PyTorch build)'][3] == 'yes' and rows['Python'][3] == 'yes' and rows['ROS 2'][3] == 'yes'
    assert rows['cuDNN (what PyTorch loads)'][3] == 'yes' and rows['MAVROS'][3] == 'no'
    assert rows['TensorRT (Python)'][1] == 'unknown' and rows['TensorRT (Python)'][3] == '?'


def test_summary_cli_writes_file(tmp_path, capsys):
    run = tmp_path / 'r'
    assert hil_pc.main(['summary', '--run-dir', str(run), '--link', 'virtual', '--photos', 'after',
                        '--connected', 'x']) == 0
    text = (run / 'jetson' / 'summary.md').read_text()
    assert 'virtual Jetson' in text and 'photos were copied after the flight' in text
    assert 'the clock offset was not measured' in text
    # differences.txt: the fixed list + this run's own, one "- " line each (what sim_demo run_summary reads)
    diff = [ln[2:] for ln in (run / 'jetson' / 'differences.txt').read_text().splitlines() if ln.startswith('- ')]
    assert diff[:len(hil_pc.FIELD_DIFFERENCES)] == hil_pc.FIELD_DIFFERENCES
    assert any('virtual Jetson' in d for d in diff) and any('backend was not reported' in d for d in diff)


def test_cli_typo_exit_2():
    assert hil_pc.main(['nope']) == 2
    with pytest.raises(SystemExit) as e:
        hil_pc.main(['pace', '--mode', 'fast'])
    assert e.value.code == 2
