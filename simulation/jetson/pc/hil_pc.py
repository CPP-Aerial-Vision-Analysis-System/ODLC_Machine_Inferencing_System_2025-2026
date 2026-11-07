#!/usr/bin/env python3
"""hil_pc.py -- small, testable helpers for the PC side of a Jetson-in-the-loop flight.

PURPOSE (plain words): the shell scripts of the PC side (link.sh, collect.sh, hil_steps.sh) need a few jobs that are
easier and safer in Python, and that must be unit-tested without the board:
  * pace        how fast the camera's photo downloads may go through a shared SSH login (so photos never block the
                autopilot link for long; docs/jetson/HIL_DESIGN.md 4.1)
  * bwlimit     the same idea for the live photo mirror (rsync --bwlimit, in KB/s)
  * clock       record the Jetson's clock offset at the start / end of a flight into RUN_DIR/jetson/clock.txt
  * publish     move mirrored photos into a watched folder: each file's time shifted from the Jetson's clock to the
                PC's clock, the .json sidecar BEFORE its photo, through a hidden temporary name and a rename, so the
                PC's photo watcher never sees a half-written file (HIL_DESIGN.md 7 item 4 and 11)
  * summary     write RUN_DIR/jetson/summary.md: link, image, power mode, clocks, link drops, photos, the detector
                backend that really ran (new_od's own log lines), the version table "this run's container vs the
                field Jetson" (from RUN_DIR/jetson/versions.env, made by container_versions.py), the "Differences
                from the field" list (HIL_DESIGN.md 9.7 and 12.1); plus RUN_DIR/jetson/differences.txt (the same
                list, one per line, which sim_demo run_summary prefers)
Every command prints errors as "[hil_pc] ERROR: what. Likely cause: why. Fix: how" and exits 2 for a typo and 1 for
a failed step. Tests: simulation/jetson/pc/test/test_hil_pc.py (plain Python, no ROS).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

PROG = '[hil_pc]'
IMAGE_EXT = ('.jpg', '.jpeg', '.png')
SIDECAR_EXT = '.json'

#: Mbit/s the legacy photo pipeline needs at about one photo per second (VERIFIED sizes, HIL_DESIGN.md 6 / 17 items
#: 5 and 9): zr10 2560x1440 ~1.32 MiB -> ~11 Mbit/s; a8mini 3840x2160 ~2.35 MiB -> ~19 Mbit/s.
CAMERA_NEED_MBIT = {'zr10': 11.0, 'a8mini': 19.0}
PACE_SHARE = 0.60          # login mode: photos may use 60 % of the measured link throughput ...
PACE_CAP_MBIT = 80.0       # ... at most 80 Mbit/s ...
PACE_FLOOR_FACTOR = 1.5    # ... and never less than 1.5 x what the camera profile needs
MIRROR_SHARE = 0.40        # login mode: the live photo mirror may use 40 % of the measured throughput
CLOCK_WARN_S = 0.5         # above this the summary and check warn (photo times are corrected anyway)
INTERPOLATE_ABOVE_S = 0.2  # start and end offsets further apart than this: interpolate by time

#: HIL_DESIGN.md 12.1: what still differs from the field in every Jetson-in-the-loop flight (the fixed list).
FIELD_DIFFERENCES = [
    'The autopilot is ArduCopter 4.6.3 SITL on the PC, not the Pixhawk (same firmware version; simulated sensors, '
    'physics and GPS; simulated time runs at real speed).',
    'The MAVLink link is TCP through SSH (USB network, Ethernet or Wi-Fi), not TELEM2 serial: MAVROS uses '
    'FCU_URL=tcp://127.0.0.1:5763 instead of /dev/ttyUSB0:115200 (launcher.sh runs in hw mode in both).',
    'Link glitches behave differently: a broken serial link loses bytes; through SSH, bytes already sent can arrive up '
    'to ~6 s late if the connection recovers by itself (the relay\'s gate limits this to what MAVROS sent in the first '
    '2.5 s of the glitch). A long drop ends the connection.',
    'The camera is the emulator on the PC, reached through the relay: real SIYI protocol, RTSP stream and full-size '
    'photos, but rendered by Gazebo and carried over the SSH link (photos paced in login mode) instead of the camera\'s '
    'own Ethernet port, so latency and download times differ from the real camera\'s.',
    'Camera pipeline and launch file: the new video pipeline is started by the video workflow\'s PROPOSED field launch '
    'file, which the team has not adopted yet; its detector container joins our container\'s private network instead '
    'of --network host. The legacy photo option uses the zr10 photo profile (2560x1440).',
    'ROS domain 77 with ROS_LOCALHOST_ONLY=1 instead of the field\'s domain 0 (the team code does not depend on it).',
    'The Jetson\'s clock may differ from the PC\'s: photo file times are corrected, sidecar contents and message stamps '
    'are not.',
    'The board\'s setup may differ from flight day: desktop running vs headless, power mode, other programs, the '
    'containers\' memory and CPU limits (recorded below).',
    'Files go to our own folder (~/astra_hil/photos, not the team\'s mapping_photos) in SIM mode, so the team\'s '
    'folders are never touched.',
    'ROS graph: sealed inside the container; on the drone the team\'s other tools on the Jetson host could see it.',
    'Mission wiring of the new detections: unknown until the team says how the yolo26 detections reach main_controller; '
    'see "What was connected" below.',
]


#: The field Jetson's versions, for the "Versions" table of every run summary (owner 12:32: "the Jetson's software,
#: version for version"). Sources: the owner-approved read-only live probe of the board on 2026-10-31 11:47
#: (D:\suas_scratch\coord\jetson_probe.md, recorded in simulation/jetson/test/samples/board_live_1147.env) and the
#: team's own pins (requirements_jetson.txt) for the Python packages the image installs. Each row: (label, KEY in
#: versions.env, field value, where the field value comes from).
FIELD_JETSON_VERSIONS = [
    ('OS', 'OS', 'Ubuntu 22.04.5', 'live probe'),
    ('CPU architecture', 'ARCH', 'aarch64', 'live probe'),
    ('L4T (board) / L4T in the image', 'L4T_IN_IMAGE', 'R36.4.7', 'live probe'),
    ('CUDA (PyTorch build)', 'TORCH_CUDA', '12.6', 'live probe (CUDA 12.6.11)'),
    ('cuDNN (what PyTorch loads)', 'TORCH_CUDNN', '9.3', 'board notes (JetPack 6.2.1)'),
    ('cuDNN (system package in the image)', 'CUDNN_PKG', '9.3', 'board notes (JetPack 6.2.1)'),
    ('TensorRT (Python)', 'TENSORRT_PY', '10.3.0', 'live probe'),
    ('TensorRT (system libnvinfer in the image)', 'TENSORRT_PKG', '10.3.0', 'live probe'),
    ('ROS 2', 'ROS_DISTRO', 'humble', 'live probe'),
    ('MAVROS', 'MAVROS', '2.12.0', 'live probe (native install)'),
    ('Python', 'PYTHON', '3.10.12', 'live probe'),
    ('PyTorch', 'TORCH', '2.8.0', "team pins (none installed natively on the board)"),
    ('torchvision', 'TORCHVISION', '0.23.0', 'team pins'),
    ('Ultralytics', 'ULTRALYTICS', '8.4.75', 'team pins'),
    ('numpy', 'NUMPY', '1.26.4', 'team pins (must stay < 2)'),
    ('OpenCV (Python)', 'OPENCV_PY', '4.10.0.84', 'team pins (opencv-python)'),
    ('GPU', 'GPU', 'Orin (Jetson Orin Nano 8 GB Super)', 'live probe'),
]


def version_rows(versions: Dict[str, str]) -> List[Tuple[str, str, str, str]]:
    """Rows (part, this run's container, field Jetson, same?) for the summary's version table.

    'same?' is 'yes' when the container's value starts with the field value's leading version numbers (so 12.6 matches
    12.6.11 and 2.8.0 matches 2.8.0+cu126), 'no' when both are known and differ, '?' when one side is unknown.
    The GPU and L4T rows are never "the same" on the virtual Jetson (an RTX GPU, no L4T): they show '-'.
    """
    rows = []
    for label, key, field, _src in FIELD_JETSON_VERSIONS:
        mine = (versions.get(key) or '').strip()
        if key in ('GPU', 'L4T_IN_IMAGE'):
            same = '-'
        elif not mine or not field:
            same = '?'
        else:
            want = re.match(r'[0-9]+(\.[0-9]+)*', field)
            got = re.match(r'[0-9]+(\.[0-9]+)*', mine)
            if want and got:
                w, g = want.group(0).split('.'), got.group(0).split('.')
                same = 'yes' if g[:len(w)] == w or w[:len(g)] == g else 'no'
            else:
                same = 'yes' if mine.lower().startswith(field.lower()[:12]) else 'no'
        rows.append((label, mine or 'unknown', field, same))
    return rows


def detector_backend_from_log(text: str) -> str:
    """What the team's legacy detector new_od really ran on, read from its own log lines in team_stack.log.

    new_od (detection/model_manager.py) logs "TensorRT model loaded ...", "PyTorch model loaded ..." or "PyTorch
    fallback model loaded", and "Device: cuda:0" / "Device: cpu" (new_od.py). '' when none of them is in the log
    (for example the new video pipeline, whose detector logs elsewhere).
    """
    text = re.sub(r'\x1b\[[0-9;]*m', '', text or '')        # launch output carries colour codes
    if 'PyTorch fallback model loaded' in text:
        engine = 'PyTorch (fallback: the TensorRT engine did not load)'
    elif 'TensorRT model loaded' in text:
        engine = 'TensorRT'
    elif 'PyTorch model loaded' in text:
        engine = 'PyTorch'
    else:
        return ''
    devices = re.findall(r'Device: ([A-Za-z0-9:_-]+)', text)
    gpus = re.findall(r'CUDA GPU detected: ([^\n\]]+)', text)
    where = devices[-1] if devices else ''
    if gpus and where.startswith('cuda'):
        where += f' ({gpus[-1].strip()})'
    return f'{engine} on {where}' if where else engine


def err(what: str, cause: str, fix: str, code: int = 1):
    """Print the standard error and exit."""
    print(f'{PROG} ERROR: {what}\n    Likely cause: {cause}\n    Fix: {fix}', file=sys.stderr)
    sys.exit(code)


def read_env(path: str) -> Dict[str, str]:
    """KEY=VALUE lines of a file ({} when missing). Quotes around a value are removed."""
    out: Dict[str, str] = {}
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                out[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        pass
    return out


def write_env(path: str, values: Dict[str, object]) -> None:
    """Write KEY=VALUE lines atomically."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = f'{path}.tmp{os.getpid()}'
    with open(tmp, 'w', encoding='utf-8') as f:
        for k, v in values.items():
            f.write(f'{k}={v}\n')
    os.replace(tmp, path)


def to_float(text, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


# =====================================================================================================================
#  pace / bwlimit
# =====================================================================================================================

def measured_mbit(speed_file: str) -> Optional[float]:
    """The throughput `jetson.sh check --speed` measured (THROUGHPUT_MBIT in its file), or None."""
    return to_float(read_env(speed_file).get('THROUGHPUT_MBIT')) if speed_file else None


def photo_pace_mbit(mode: str, auth: str, camera: str, measured: Optional[float]) -> float:
    """The pace for hil_relay.py pc-http --pace-mbit (0 = no pacing).

    off -> 0; a number -> that number; auto -> 0 in key mode (photos have their own connection there), and in login
    mode 60 % of the measured throughput, at most 80 Mbit/s, never below 1.5 x the camera profile's need. Without a
    measurement, login mode uses the cap (80 Mbit/s): it keeps a slow link from being flooded by one photo.
    """
    mode = (mode or 'auto').strip().lower()
    if mode == 'off':
        return 0.0
    if mode != 'auto':
        value = to_float(mode)
        if value is None or value < 0:
            raise ValueError(f'--photo-pace must be auto, off or a number of Mbit/s, not "{mode}"')
        return value
    if auth == 'key':
        return 0.0
    floor = PACE_FLOOR_FACTOR * CAMERA_NEED_MBIT.get(camera, CAMERA_NEED_MBIT['a8mini'])
    if measured is None or measured <= 0:
        return PACE_CAP_MBIT
    return round(max(floor, min(PACE_CAP_MBIT, PACE_SHARE * measured)), 1)


def mirror_bwlimit_kbps(auth: str, measured: Optional[float]) -> int:
    """rsync --bwlimit (KB/s, 0 = none) for the live photo mirror: 40 % of the measured throughput in login mode."""
    if auth == 'key' or measured is None or measured <= 0:
        return 0
    return max(256, int(MIRROR_SHARE * measured * 1000 / 8))


def link_verdict(measured: Optional[float], camera: str) -> str:
    """'ok', 'slow' (photos should come after the flight) or 'unknown'."""
    if measured is None:
        return 'unknown'
    return 'ok' if measured >= PACE_FLOOR_FACTOR * CAMERA_NEED_MBIT.get(camera, 19.0) * 2 else 'slow'


# =====================================================================================================================
#  clock
# =====================================================================================================================

def parse_clock_line(text: str) -> Tuple[float, float]:
    """'offset_s=0.123 rtt_ms=1.5' (what `jetson.sh clock` prints) -> (offset_s, rtt_ms)."""
    found = dict(re.findall(r'(offset_s|rtt_ms)=(-?[0-9.]+)', text or ''))
    if 'offset_s' not in found:
        raise ValueError(f'no offset_s=... in "{text.strip()[:120]}"')
    return float(found['offset_s']), float(found.get('rtt_ms', 'nan'))


def offset_from_samples(samples: Sequence[Tuple[float, float, float]]) -> Tuple[float, float]:
    """(offset_s, rtt_ms) from samples (t0 = PC time, tj = Jetson time, t1 = PC time) (HIL_DESIGN.md 11).

    offset = tj - (t0 + t1) / 2 of the sample with the shortest round trip; its uncertainty is half that round trip.
    """
    good = [(t1 - t0, tj - (t0 + t1) / 2) for t0, tj, t1 in samples if t1 >= t0]
    if not good:
        raise ValueError('no usable clock sample (PC time went backwards or nothing was measured)')
    rtt, off = min(good)
    return off, rtt * 1000.0


def record_clock(clock_file: str, which: str, offset_s: float, rtt_ms: float, t_unix: Optional[float] = None) -> dict:
    """Add the start or end measurement to clock.txt; offset_s = the mean of both when both exist."""
    data = read_env(clock_file)
    t_unix = time.time() if t_unix is None else t_unix
    data[f'offset_{which}_s'] = f'{offset_s:.4f}'
    data[f'rtt_{which}_ms'] = f'{rtt_ms:.2f}'
    data[f't_{which}_unix'] = f'{t_unix:.3f}'
    offs = [to_float(data.get(k)) for k in ('offset_start_s', 'offset_end_s') if data.get(k)]
    data['offset_s'] = f'{sum(offs) / len(offs):.4f}'
    rtts = [to_float(data.get(k)) for k in ('rtt_start_ms', 'rtt_end_ms') if data.get(k)]
    rtts = [r for r in rtts if r is not None and r == r]
    if rtts:
        data['rtt_ms'] = f'{min(rtts):.2f}'
    data['meaning'] = 'offset_s>0 means the Jetson clock is ahead of the PC; pc_time = jetson_time - offset'
    write_env(clock_file, data)
    return data


def offset_at(clock: Dict[str, str], jetson_time: float) -> float:
    """The offset to subtract from a Jetson time stamp (0 when nothing was measured).

    One measurement: that offset. Start and end more than 0.2 s apart: linear interpolation by time (clamped).
    """
    start, end = to_float(clock.get('offset_start_s')), to_float(clock.get('offset_end_s'))
    if start is None and end is None:
        return to_float(clock.get('offset_s'), 0.0) or 0.0
    if start is None or end is None or abs(end - start) <= INTERPOLATE_ABOVE_S:
        return start if start is not None else end
    t0, t1 = to_float(clock.get('t_start_unix')), to_float(clock.get('t_end_unix'))
    if t0 is None or t1 is None or t1 <= t0:
        return (start + end) / 2
    pc_guess = jetson_time - start
    frac = min(1.0, max(0.0, (pc_guess - t0) / (t1 - t0)))
    return start + (end - start) * frac


# =====================================================================================================================
#  publish (photo mirror into a watched folder)
# =====================================================================================================================

def _is_image(name: str) -> bool:
    low = name.lower()
    return low.endswith(IMAGE_EXT) and not low.startswith('.') and '.tmp.' not in low


def _is_sidecar(name: str) -> bool:
    return name.lower().endswith(SIDECAR_EXT) and not name.startswith('.')


def plan_publish(staging: Dict[str, Tuple[int, float]], published: Dict[str, list], first_seen: Dict[str, float],
                 now: float, wait_sidecar_s: float = 6.0) -> List[str]:
    """Which staged files to publish now, in order (each sidecar before its photo).

    staging   name -> (size, mtime) of the files the mirror holds now
    published name -> [size, mtime] of what was already moved (a changed file is published again)
    first_seen name -> when the photo first appeared (a photo waits up to wait_sidecar_s for its sidecar)
    """
    todo_side, todo_img = [], []
    for name, (size, mtime) in sorted(staging.items()):
        if published.get(name) == [size, mtime]:
            continue
        if _is_sidecar(name):
            todo_side.append(name)
        elif _is_image(name):
            stem = os.path.splitext(name)[0]
            side = stem + SIDECAR_EXT
            has_side = side in staging
            if has_side or now - first_seen.get(name, now) >= wait_sidecar_s:
                todo_img.append(name)
    # Every sidecar first (also those of photos not ready yet: tools that only look for images ignore .json files),
    # then the photos, so a photo never appears before its sidecar.
    return todo_side + todo_img


def publish_once(staging_dir: str, dest_dir: str, clock_file: str, state_file: str,
                 now: Optional[float] = None) -> int:
    """Move new/changed staged photos into dest_dir with corrected times. Returns how many files were published."""
    now = time.time() if now is None else now
    state = {}
    try:
        with open(state_file, encoding='utf-8') as f:
            state = json.load(f)
    except (OSError, ValueError):
        state = {}
    published = state.setdefault('published', {})
    first_seen = state.setdefault('first_seen', {})
    clock = read_env(clock_file)
    staging: Dict[str, Tuple[int, float]] = {}
    try:
        names = os.listdir(staging_dir)
    except OSError:
        names = []
    for name in names:
        if name.startswith('.'):
            continue                                   # rsync's temporary files
        path = os.path.join(staging_dir, name)
        try:
            st = os.stat(path)
        except OSError:
            continue
        if not os.path.isfile(path):
            continue
        staging[name] = (st.st_size, round(st.st_mtime, 6))
        first_seen.setdefault(name, now)
    os.makedirs(dest_dir, exist_ok=True)
    count = 0
    for name in plan_publish(staging, published, first_seen, now):
        size, mtime = staging[name]
        corrected = mtime - offset_at(clock, mtime)
        tmp = os.path.join(dest_dir, f'.{name}.tmp.{os.getpid()}')
        try:
            shutil.copyfile(os.path.join(staging_dir, name), tmp)
            os.utime(tmp, (corrected, corrected))
            os.replace(tmp, os.path.join(dest_dir, name))
        except OSError as e:
            print(f'{PROG} WARNING: could not publish {name}: {e}. Consequence: this photo is missing on the PC for now '
                  f'(the next round tries again).', file=sys.stderr)
            try:
                os.unlink(tmp)
            except OSError:
                pass
            continue
        published[name] = [size, mtime]
        count += 1
    os.makedirs(os.path.dirname(state_file) or '.', exist_ok=True)
    tmp_state = f'{state_file}.tmp{os.getpid()}'
    with open(tmp_state, 'w', encoding='utf-8') as f:
        json.dump(state, f)
    os.replace(tmp_state, state_file)
    return count


# =====================================================================================================================
#  summary
# =====================================================================================================================

def parse_tegrastats(text: str) -> dict:
    """GPU load average / peak, peak temperature and lowest free memory from tegrastats lines."""
    gpu, temps, free = [], [], []
    for line in text.splitlines():
        m = re.search(r'GR3D_FREQ (\d+)%', line)
        if m:
            gpu.append(int(m.group(1)))
        m = re.search(r'RAM (\d+)/(\d+)MB', line)
        if m:
            free.append(int(m.group(2)) - int(m.group(1)))
        temps += [float(t) for name, t in re.findall(r'([A-Za-z0-9_]+)@(-?[0-9.]+)C', line)
                  if not name.lower().startswith('pmic')]
    out = {'samples': len(gpu)}
    if gpu:
        out.update(gpu_avg=round(sum(gpu) / len(gpu), 1), gpu_peak=max(gpu))
    if temps:
        out['temp_peak_c'] = max(temps)
    if free:
        out['mem_free_min_mb'] = min(free)
    return out


def _load_json(path: str) -> dict:
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def build_summary(run_dir: str, opts: dict) -> str:
    """The Markdown text of RUN_DIR/jetson/summary.md."""
    jd = os.path.join(run_dir, 'jetson')
    start = _load_json(os.path.join(jd, 'status_start.json'))
    end = _load_json(os.path.join(jd, 'status_end.json'))
    clock = read_env(os.path.join(jd, 'clock.txt'))
    relay = read_env(os.path.join(jd, 'relay.status'))
    tap = read_env(os.path.join(jd, 'tap_status.env'))
    link = read_env(os.path.join(jd, 'link_state.env'))
    teg = {}
    try:
        with open(os.path.join(jd, 'tegrastats.log'), encoding='utf-8', errors='replace') as f:
            teg = parse_tegrastats(f.read())
    except OSError:
        pass
    photos_dir = os.path.join(run_dir, 'photos')
    try:
        n_photos = len([n for n in os.listdir(photos_dir) if _is_image(n)])
    except OSError:
        n_photos = 0
    versions = read_env(os.path.join(jd, 'versions.env'))
    # which detector backend really ran: jetson.sh status may report it; else new_od's own log lines say it
    backend = str(start.get('detector_backend') or '')
    if not backend:
        try:
            with open(os.path.join(run_dir, 'team_stack.log'), encoding='utf-8', errors='replace') as f:
                backend = detector_backend_from_log(f.read())
        except OSError:
            backend = ''

    def val(d: dict, k: str, unknown: str = 'unknown') -> str:
        v = d.get(k)
        return str(v) if v not in (None, '') else unknown

    virtual = opts.get('link') == 'virtual'
    board_only = 'not applicable (virtual Jetson: this PC)' if virtual else None
    pace = str(opts.get('pace') or '')
    pace_text = (f'{pace} Mbit/s (0 = none)' if pace else
                 ('not used (virtual link: no SSH to share)' if virtual else 'unknown'))
    lines = ['# Jetson in the loop: run summary', '',
             f'Run folder: `{run_dir}`  ', f'Written: {time.strftime("%Y-%m-%d %H:%M:%S")}', '',
             '## The link', '',
             f'- link: **{opts.get("link", "?")}** ({opts.get("auth", "?")} mode), host `{opts.get("host", "?")}`, '
             f'user `{opts.get("user", "?")}`',
             f'- photos: {opts.get("photos", "live")}, photo pace: {pace_text}, '
             f'MAVLink baud limit: {opts.get("baud", "off")}',
             # two views of a lost link: the SSH connection (link.sh's watcher; never for the virtual link) and the
             # autopilot stream as the relay in the container saw it (SITL silent; also a virtual drop rehearsal)
             f'- SSH link drops: {val(link, "DROPS", "0")} (total down {val(link, "DOWN_S", "0")} s); autopilot silent '
             f'(seen by the relay in the container): {val(relay, "MAV_DROPS", "0")} time(s), '
             f'{val(relay, "MAV_DOWN_S", "0")} s in total',
             f'- bytes the gate dropped (MAVROS commands never sent late): {val(relay, "MAV_DROPPED_BYTES", "unknown")}; '
             f'relay re-dials: {val(relay, "REDIALS", "unknown")}; largest MAVLink gap inside one connection: '
             f'{val(relay, "MAX_GAP_S", "unknown")} s; camera datagrams dropped while down: '
             f'{val(relay, "SDK_DROPPED", "unknown")}',
             f'- tap: {val(tap, "MSGS", "0")} messages copied, last rate {val(tap, "RATE_KBIT", "?")} kbit/s, '
             f'presence nodes: {val(tap, "NODES", "none")}', '',
             '## The board and the image', '',
             f'- image: `{val(start, "image")}`, git sha `{val(start, "git_sha")}`; engine: {val(start, "engine")}',
             f'- L4T: {board_only or val(start, "l4t")}',
             f'- **power mode** at the start: {board_only or val(start, "power_mode", "unknown (needs sudo)")}; at the '
             f'end: {board_only or val(end, "power_mode", "unknown (needs sudo)")}; clocks: '
             f'{board_only or (val(start, "clocks") + " -> " + val(end, "clocks"))}',
             '- tegrastats during the flight: ' + (
                 f'{teg["samples"]} samples, GPU load average {teg.get("gpu_avg", "?")} % / peak '
                 f'{teg.get("gpu_peak", "?")} %, peak temperature {teg.get("temp_peak_c", "?")} C, lowest free memory '
                 f'{teg.get("mem_free_min_mb", "?")} MB' if teg.get('samples') else 'no samples (virtual Jetson, or '
                                                                                     'tegrastats was not recorded)'),
             f'- memory available at the start: {val(start, "mem_avail_mb")} MB; disk free: '
             f'{val(start, "disk_free_gb")} GB',
             f'- **detector backend**: {backend or "unknown (not in jetson.sh status, and new_od logged no model line)"}',
             '']
    # The software versions in the container (owner 12:32: "the version table PC vs Jetson in each run summary").
    # versions.env comes from container_versions.py run inside the container at the start of the flight;
    # jetson.sh status --json may also carry a "versions" object (used only when versions.env is missing).
    if not versions and isinstance(start.get('versions'), dict):
        versions = {str(k): str(v) for k, v in start['versions'].items()}
    lines += ['## Versions: this run\'s container vs the field Jetson', '']
    if versions:
        lines += ['| Part | This run\'s container | Field Jetson | Same? |', '|---|---|---|---|']
        lines += [f'| {a} | {b} | {c} | {d} |' for a, b, c, d in version_rows(versions)]
        lines += ['', f'Image: flavor {versions.get("IMAGE_FLAVOR") or "?"}, git {versions.get("IMAGE_GIT_SHA") or "?"}, '
                      f'built {versions.get("IMAGE_BUILT") or "?"}; model files: {versions.get("MODEL_PT") or "no .pt"}; '
                      f'{versions.get("MODEL_ENGINE") or "no TensorRT engine"}. Field values: the read-only live probe '
                      'of the board (2026-10-31 11:47) and the team\'s pins (requirements_jetson.txt). MAVROS 2.14.0 vs '
                      '2.12.0 is a known difference (no apt snapshot has 2.12.0; same messages).']
    else:
        lines.append('- not recorded (RUN_DIR/jetson/versions.env is missing: the container could not be asked)')
    lines += ['', '## Clocks', '']
    off_s, off_e = to_float(clock.get('offset_start_s')), to_float(clock.get('offset_end_s'))
    if off_s is None and off_e is None:
        lines.append('- the clock offset was not measured (photo times were not corrected)')
    else:
        lines.append(f'- Jetson minus PC: start {clock.get("offset_start_s", "?")} s, end '
                     f'{clock.get("offset_end_s", "?")} s (round trip {clock.get("rtt_ms", "?")} ms); photo file times '
                     f'were shifted by this offset')
        worst = max(abs(x) for x in (off_s, off_e) if x is not None)
        if worst > CLOCK_WARN_S:
            lines.append(f'- **WARNING:** the Jetson clock is {worst:.2f} s off (more than {CLOCK_WARN_S} s): the team\'s '
                         'own log and sidecar times are off by that much. Set the clock (owner, in the login window): '
                         'sudo date -s @<PC time>, or connect the Jetson to the internet once (NTP).')
    lines += ['', '## Photos', '', f'- photos in the run folder: {n_photos}', '',
              '## What was connected', '']
    for item in opts.get('connected', []):
        lines.append(f'- {item}')
    if not opts.get('connected'):
        lines.append('- (nothing recorded)')
    lines += ['', '## Differences from the field', '']
    for i, item in enumerate(FIELD_DIFFERENCES, 1):
        lines.append(f'{i}. {item}')
    extra = run_differences(opts, backend)
    if extra:
        lines += ['', 'This run also differed in:', '']
        lines += [f'- {e}' for e in extra]
    return '\n'.join(lines) + '\n'


def run_differences(opts: dict, backend: str) -> List[str]:
    """This run's own additions to the fixed differences list (run_extras + the detector backend)."""
    extra = list(opts.get('extra', []))
    if backend and 'tensorrt' not in backend.lower():
        # owner 12:25 item 5: a run whose detector did not run on TensorRT says so as a difference
        extra.append(f'the detector ran on "{backend}", not TensorRT (the field runs TensorRT on the Jetson GPU)')
    elif not backend:
        extra.append('the detector backend was not reported, so it is not known whether TensorRT ran')
    return extra


def differences_text(run_dir: str, opts: dict) -> str:
    """RUN_DIR/jetson/differences.txt: every difference from the field, one per line (the fixed list, then this run's
    own). sim_demo run_summary prefers this file for its "What still differs from the field" section."""
    backend = ''
    try:
        backend = str(_load_json(os.path.join(run_dir, 'jetson', 'status_start.json')).get('detector_backend') or '')
        if not backend:
            with open(os.path.join(run_dir, 'team_stack.log'), encoding='utf-8', errors='replace') as f:
                backend = detector_backend_from_log(f.read())
    except OSError:
        pass
    items = FIELD_DIFFERENCES + run_differences(opts, backend)
    return ('# differences.txt -- written by simulation/jetson/pc/hil_pc.py summary: what this Jetson-in-the-loop run '
            'did not do like the drone at the field (one per line; docs/jetson/HIL_DESIGN.md 12.1)\n'
            + ''.join(f'- {i}\n' for i in items))


def run_extras(link: str, camera: str, photos: str, baud: str, team_agreed: bool, pipeline: str) -> List[str]:
    """The run-specific additions to the differences list."""
    out = []
    if link == 'virtual':
        out.append('virtual Jetson: the x86 build of the image on this PC (no Jetson GPU, no arm64, no USB link)')
    if link == 'ssh':
        out.append('the link was plain SSH (Ethernet / Wi-Fi / LAN), not the USB-C cable')
    if pipeline == 'legacy':
        out.append('legacy (2025) photo pipeline, not the field\'s new video pipeline')
        if camera == 'a8mini':
            out.append('camera profile a8mini (4K photos): a heavier load test than the ZR10\'s 2K photos')
    if photos == 'after':
        out.append('photos were copied after the flight (no live per-photo table until score_run --rescore)')
    if baud in ('off', ''):
        out.append('no 115200-baud limit on the MAVLink link (TELEM2 has one)')
    if team_agreed:
        out.append('--team-agreed was used (the team shared the board during the run)')
    return out


# =====================================================================================================================
#  command line
# =====================================================================================================================

def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(prog='hil_pc.py', description=__doc__.split('\n\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('pace')
    a.add_argument('--mode', default='auto')
    a.add_argument('--auth', default='login', choices=('login', 'key'))
    a.add_argument('--camera', default='zr10')
    a.add_argument('--speed-file', default='')
    a = sub.add_parser('bwlimit')
    a.add_argument('--auth', default='login', choices=('login', 'key'))
    a.add_argument('--speed-file', default='')
    a = sub.add_parser('clock')
    a.add_argument('--clock-file', required=True)
    a.add_argument('--which', required=True, choices=('start', 'end'))
    a.add_argument('--line', default='', help='what `jetson.sh clock` printed (offset_s=... rtt_ms=...)')
    a.add_argument('--samples', default='', help='or a file of "t0 tj t1" lines (PC, Jetson, PC times)')
    a = sub.add_parser('publish')
    a.add_argument('--staging', required=True)
    a.add_argument('--dest', required=True)
    a.add_argument('--clock-file', default='')
    a.add_argument('--state', required=True)
    a = sub.add_parser('summary')
    a.add_argument('--run-dir', required=True)
    for k in ('link', 'auth', 'host', 'user', 'photos', 'pace', 'baud', 'camera', 'pipeline'):
        a.add_argument(f'--{k}', default='')
    a.add_argument('--team-agreed', action='store_true')
    a.add_argument('--connected', action='append', default=[], help='one line of "What was connected" (repeatable)')
    try:
        args = p.parse_args(argv)
    except SystemExit as e:
        return 2 if e.code not in (0, None) else 0

    if args.cmd == 'pace':
        try:
            print(f'{photo_pace_mbit(args.mode, args.auth, args.camera, measured_mbit(args.speed_file)):g}')
        except ValueError as e:
            err(str(e), 'a typo in --photo-pace.', 'use --photo-pace auto, off, or a number such as 40', code=2)
        return 0
    if args.cmd == 'bwlimit':
        print(mirror_bwlimit_kbps(args.auth, measured_mbit(args.speed_file)))
        return 0
    if args.cmd == 'clock':
        try:
            if args.samples:
                rows = []
                with open(args.samples, encoding='utf-8') as f:
                    for line in f:
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                rows.append(tuple(float(x) for x in parts))
                            except ValueError:
                                continue
                off, rtt = offset_from_samples(rows)
            else:
                off, rtt = parse_clock_line(args.line)
        except (OSError, ValueError) as e:
            err(f'cannot read the clock measurement: {e}.', 'jetson.sh clock failed or printed something else.',
                'run  bash simulation/jetson/jetson.sh clock  by hand and read its message')
        data = record_clock(args.clock_file, args.which, off, rtt)
        print(f'offset_{args.which}_s={data[f"offset_{args.which}_s"]} rtt_ms={data.get("rtt_ms", "?")}')
        if abs(off) > CLOCK_WARN_S:
            print(f'{PROG} WARNING: the Jetson clock is {off:+.2f} s off the PC clock. Consequence: the team\'s own log '
                  f'and sidecar times are off by that much (photo file times are corrected). Fix (owner, in the login '
                  f'window): sudo date -s @$(date +%s)  typed with the PC time, or connect the Jetson to the internet',
                  file=sys.stderr)
        return 0
    if args.cmd == 'publish':
        n = publish_once(args.staging, args.dest, args.clock_file, args.state)
        print(n)
        return 0
    if args.cmd == 'summary':
        opts = vars(args).copy()
        opts['extra'] = run_extras(args.link, args.camera, args.photos, args.baud, args.team_agreed, args.pipeline)
        text = build_summary(args.run_dir, opts)
        out = os.path.join(args.run_dir, 'jetson', 'summary.md')
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w', encoding='utf-8') as f:
            f.write(text)
        with open(os.path.join(args.run_dir, 'jetson', 'differences.txt'), 'w', encoding='utf-8') as f:
            f.write(differences_text(args.run_dir, opts))
        print(text)
        return 0
    return 2


if __name__ == '__main__':
    sys.exit(main())
