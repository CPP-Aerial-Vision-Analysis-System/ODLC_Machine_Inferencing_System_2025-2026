#!/usr/bin/env python3
"""simulation/jetson/lib/jetson_facts.py -- judge the Jetson's facts: OK / WARN / FAIL, each with a fix.

PURPOSE: `jetson.sh check` runs lib/remote_facts.sh on the Jetson (read-only; it prints KEY=VALUE lines) and hands
the lines to this module. Here, on the PC, every fact is judged against what the Jetson-in-the-loop container
needs (docs/jetson/HIL_DESIGN.md section 9.2, rows 5-16): the board and its JetPack/L4T release, Docker without a
password, the NVIDIA runtime, "is the team using the board right now?" (9.6), disk with a reserve for the team,
memory, the power mode (reported, never changed), the clock offset, the registry port, and our image and TensorRT
engine. Each row prints one line in the simulator's message style; a FAIL line says what failed, the likely cause
and the exact fix. The exit code is the code of the FIRST failed row (HIL_DESIGN.md 15.4), 0 when nothing failed.
Warnings never change the exit code.

Pure Python standard library, so the tests run on Windows Python too (simulation/jetson/test/test_jetson_facts.py).

Usage:
  python3 jetson_facts.py judge --facts FILE [--for check|deploy|warmup|flight|start] [--link usb|ssh|virtual]
                                [--team-agreed] [--pc-git-sha SHA] [--pc-code-sha SHA] [--pipeline new|legacy]
                                [--rows all|busy]
                                [--disk-reserve-gb N] [--host HOST] [--jetson-cmd CMD]
  python3 jetson_facts.py status-json --facts FILE [--link ...]      (the facts part of `jetson.sh status --json`)
Exit codes: as HIL_DESIGN.md 15.4 (7 board, 8 docker, 9 runtime, 10 disk, 11 memory, 12 image, 16 team busy,
19 engine), 2 = typed wrong or unreadable facts file.
"""

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# Exit codes (HIL_DESIGN.md 15.4)
E_BOARD, E_DOCKER, E_RUNTIME, E_DISK, E_MEMORY, E_IMAGE, E_BUSY, E_ENGINE = 7, 8, 9, 10, 11, 12, 16, 19

MEM_WARN_MB = 4096        # CPU and GPU share the Orin Nano's 7.6 GB
MEM_FAIL_MB = 2560
GPU_BUSY_PCT = 20         # GR3D_FREQ average above this = the GPU is in use
CONTAINER_BUSY_PCT = 10.0 # another container above this CPU % = busy
CLOCK_WARN_S = 0.5        # HIL_DESIGN.md 11: warning above 0.5 s
NEED_GB = {'check': 0, 'deploy': 15, 'warmup': 2, 'flight': 1, 'start': 1}


@dataclass
class Row:
    """One judged line of the check."""
    num: str
    title: str
    level: str                 # OK | WARN | FAIL | INFO | SKIP
    text: str                  # what was found
    cause: str = ''            # FAIL/WARN: likely cause
    fix: str = ''              # FAIL/WARN: the exact fix
    code: int = 0              # FAIL: the exit code
    extra: List[str] = field(default_factory=list)


def parse_facts(text: str) -> Dict[str, str]:
    """KEY=VALUE lines -> dict (later lines win; lines without '=' are ignored)."""
    out: Dict[str, str] = {}
    for line in text.splitlines():
        line = line.rstrip('\r')
        if not line or line.startswith('#') or '=' not in line:
            continue
        k, v = line.split('=', 1)
        if re.fullmatch(r'[A-Za-z0-9_]+', k):
            out[k] = v.strip()
    return out


def _num(v: Optional[str], default: Optional[float] = None) -> Optional[float]:
    try:
        return float(str(v).strip().rstrip('%'))
    except (TypeError, ValueError):
        return default


def _l4t_ok(l4t: str) -> bool:
    return bool(re.fullmatch(r'R36\.4(\.\d+)?', l4t or ''))


def busy_evidence(f: Dict[str, str]) -> List[str]:
    """What shows the team is using the board right now (empty list = not busy)."""
    found = []
    procs = [p for p in (f.get('TEAM_PROCS') or '').split(';') if p.strip()]
    if procs:
        shown = ', '.join(procs[:4]) + (f' (+{len(procs) - 4} more)' if len(procs) > 4 else '')
        found.append(f'team programs running: {shown}')
    gpu = _num(f.get('GPU_LOAD_PCT'))
    if gpu is not None and gpu > GPU_BUSY_PCT:
        found.append(f'GPU {gpu:.0f} %')
    for item in [c for c in (f.get('OTHER_CONTAINERS') or '').split(';') if c.strip()]:
        parts = item.rsplit(' ', 1)
        cpu = _num(parts[1]) if len(parts) == 2 else None
        if cpu is not None and cpu > CONTAINER_BUSY_PCT:
            found.append(f'container {parts[0]} at {cpu:.0f} % CPU')
    return found


class Judge:
    """Turns facts into rows. `purpose` = check|deploy|warmup|flight|start."""

    def __init__(self, facts: Dict[str, str], purpose: str = 'check', link: str = 'usb', team_agreed: bool = False,
                 pc_git_sha: str = '', pipeline: str = 'new', reserve_gb: Optional[float] = None,
                 host: str = '', jcmd: str = 'bash simulation/jetson/jetson.sh', pc_code_sha: str = ''):
        self.f = facts
        self.purpose = purpose
        self.link = link
        self.virtual = link == 'virtual'
        self.team_agreed = team_agreed
        self.pc_sha = pc_git_sha
        # the fingerprint of the image's files on this PC (jetson.sh code_sha); preferred over the git commit, which
        # says nothing about uncommitted changes and is "unknown" in the Linux copy (no .git there)
        self.pc_code = '' if pc_code_sha in ('', 'unknown') else pc_code_sha
        self.pipeline = pipeline
        self.reserve_gb = reserve_gb
        self.host = host or ('this PC' if self.virtual else 'the Jetson')
        self.j = jcmd

    # ---- rows -----------------------------------------------------------------------------------------------------
    def board(self) -> Row:
        f = self.f
        if self.virtual:
            return Row('5', 'board + JetPack', 'SKIP', 'virtual Jetson: this PC stands in for the board')
        arch, l4t, model = f.get('UNAME_M', ''), f.get('L4T', ''), f.get('MODEL', '') or 'unknown board'
        osv = f"{f.get('OS_ID', '?')} {f.get('OS_VERSION_ID', '?')}"
        text = f'{model}, L4T {l4t or "unknown"}, {osv} ({arch or "?"})'
        if arch != 'aarch64' or not _l4t_ok(l4t) or f.get('OS_VERSION_ID') not in ('22.04', None):
            return Row('5', 'board + JetPack', 'FAIL', text,
                       cause='the container image is made for a Jetson with JetPack 6.2.x = L4T R36.4 on Ubuntu 22.04 '
                             f'(aarch64); this machine runs {l4t or "no L4T"} on {arch or "?"}',
                       fix='use the Jetson Orin Nano with JetPack 6.2.x (re-flash it with NVIDIA SDK Manager), or rebuild '
                           'the image on another base tag (simulation/jetson/image/Dockerfile.l4t-ros, ARG L4T_TAG)',
                       code=E_BOARD)
        return Row('5', 'board + JetPack', 'OK', text)

    def docker(self) -> Row:
        f = self.f
        acc = f.get('DOCKER_ACCESS', 'none')
        if f.get('DOCKER_PRESENT') != 'yes':
            return Row('6', 'Docker', 'FAIL', 'docker is not installed',
                       cause='Docker is part of JetPack 6.2 but is missing on this board',
                       fix='the owner/team installs it (in the login window): sudo apt-get install docker.io nvidia-container',
                       code=E_DOCKER)
        ver = f.get('DOCKER_VERSION', '')
        if acc == 'direct':
            return Row('6', 'Docker without a password', 'OK', f'docker {ver} (user {f.get("USER", "?")} may use it)')
        if acc == 'sudo-n':
            return Row('6', 'Docker without a password', 'OK', f'docker {ver} through passwordless sudo (sudo -n docker)')
        user = f.get('USER', '$USER')
        return Row('6', 'Docker without a password', 'FAIL',
                   f'docker is installed, but user {user} cannot use it without a password'
                   + (f' ({f["DOCKER_ERROR"]})' if f.get('DOCKER_ERROR') else ''),
                   cause=f'{user} is not in the "docker" group (the owner\'s guide noted this), and sudo needs a password',
                   fix=f'the owner/team decides (the docker group gives root rights on the board) and types in the login '
                       f'window: sudo usermod -aG docker {user}   then closes the login window and runs '
                       f'{self.j} login again',
                   code=E_DOCKER)

    def runtime(self) -> Row:
        f = self.f
        if f.get('DOCKER_ACCESS', 'none') == 'none':
            return Row('7', 'NVIDIA container runtime', 'SKIP', 'needs Docker access first (row 6)')
        rts = (f.get('DOCKER_RUNTIMES') or '').split()
        ctk = f.get('NVIDIA_CTK_VERSION', '')
        if 'nvidia' in rts:
            return Row('7', 'NVIDIA container runtime', 'OK',
                       f'runtimes: {" ".join(rts)} (default {f.get("DOCKER_DEFAULT_RUNTIME", "?")}; our containers ask for '
                       f'{"--gpus all" if self.virtual else "--runtime nvidia"})' + (f'; {ctk}' if ctk else ''))
        fix = ('install the NVIDIA Container Toolkit in WSL (docs/simulation/DOCKER.md)' if self.virtual else
               'the owner types in the login window: sudo apt-get install nvidia-container  then  sudo systemctl restart docker')
        return Row('7', 'NVIDIA container runtime', 'FAIL', f'runtimes: {" ".join(rts) or "none listed"}',
                   cause='Docker does not know the "nvidia" runtime, so the container cannot use the GPU',
                   fix=fix, code=E_RUNTIME)

    def team_busy(self) -> Row:
        f = self.f
        if self.virtual:
            return Row('8', 'team busy?', 'SKIP', 'virtual Jetson: no team on this PC')
        mem = _num(f.get('MEM_AVAIL_MB'))
        found = busy_evidence(f)
        who = f.get('WHO', '')
        if not found:
            return Row('8', 'team busy?', 'OK',
                       'no team programs, GPU idle' + (f'; logged in: {who}' if who else ''))
        text = '; '.join(found)
        if self.team_agreed:
            return Row('8', 'team busy?', 'WARN', text,
                       cause='--team-agreed was given: the owner and the team agreed to share the board',
                       fix='none (recorded in the run summary)')
        return Row('8', 'team busy?', 'FAIL', f'the team is using the Jetson right now (found: {text})',
                   cause='the team is running their stack or a test',
                   fix='ask the team; when they agree and have stopped, run the command again. Only if the owner and the '
                       'team agreed to share the board: add --team-agreed',
                   code=E_BUSY)

    def disk(self) -> Row:
        f = self.f
        free = _num(f.get('DISK_DOCKER_FREE_GB'))
        total = _num(f.get('DISK_DOCKER_TOTAL_GB'))
        if free is None or total is None:
            return Row('9', 'disk', 'WARN', 'free disk space unknown', cause='df gave no answer for Docker\'s folder',
                       fix='look on the board: df -h /var/lib/docker')
        reserve = self.reserve_gb if self.reserve_gb is not None else max(100.0, 0.25 * total)
        need = NEED_GB.get(self.purpose, 0)
        text = (f'{free:.0f} GB free of {total:.0f} GB; this needs ~{need} GB; reserve kept free for the team: '
                f'{reserve:.0f} GB')
        if free - need < reserve:
            return Row('9', 'disk (with a reserve for the team)', 'FAIL', text,
                       cause='using this space would leave less than the reserve for the team',
                       fix=f'delete OUR old images and runs: {self.j} image prune  (only images labelled '
                           'org.astra.owner=hil; never docker system prune), or lower the reserve: JETSON_DISK_RESERVE_GB=N',
                       code=E_DISK)
        return Row('9', 'disk (with a reserve for the team)', 'OK', text)

    def memory(self) -> Row:
        mem = _num(self.f.get('MEM_AVAIL_MB'))
        tot = _num(self.f.get('MEM_TOTAL_MB'))
        if mem is None:
            return Row('10', 'memory', 'WARN', 'free memory unknown', cause='/proc/meminfo not readable', fix='free -m on the board')
        text = f'{mem:.0f} MB available of {tot or 0:.0f} MB (CPU and GPU share it)'
        if mem < MEM_FAIL_MB:
            return Row('10', 'memory', 'FAIL', text, cause='too little memory left for the stack and the detector',
                       fix='close programs on the board, or run it without the desktop for the test (the owner types: '
                           'sudo systemctl isolate multi-user.target; back with graphical.target)', code=E_MEMORY)
        if mem < MEM_WARN_MB:
            return Row('10', 'memory', 'WARN', text, cause=f'below {MEM_WARN_MB} MB the detector may run out of memory',
                       fix='close the desktop apps, or run headless for the test')
        return Row('10', 'memory', 'OK', text)

    def cuda(self) -> Row:
        f = self.f
        if self.virtual:
            return Row('11', 'CUDA / TensorRT', 'SKIP', 'virtual Jetson: the image brings CUDA for the PC GPU')
        text = (f"host L4T {f.get('L4T', '?')}, CUDA {f.get('CUDA_VERSION') or '?'}, TensorRT "
                f"{f.get('TENSORRT_VERSION') or '?'} (the container brings its own CUDA/cuDNN/TensorRT; the host gives the "
                "GPU driver)")
        if not _l4t_ok(f.get('L4T', '')):
            return Row('11', 'CUDA / TensorRT', 'FAIL', text, cause='the image needs a host with L4T R36.4.x',
                       fix='see row 5', code=E_BOARD)
        return Row('11', 'CUDA / TensorRT', 'OK', text)

    def power(self) -> Row:
        f = self.f
        if self.virtual:
            return Row('13', 'power mode and clocks', 'SKIP', 'virtual Jetson')
        mode = f.get('NVPMODEL_MODE') or 'unknown (nvpmodel -q needs sudo here)'
        mid = f.get('NVPMODEL_ID', '')
        lst = f.get('NVPMODEL_LIST') or 'not readable'
        clocks = f.get('CLOCKS', 'unknown')
        r = Row('13', 'power mode and clocks (report only)', 'INFO',
                f"mode {mode}{' (id ' + mid + ')' if mid else ''}; clocks {clocks}; the board's modes: {lst}; tegrastats "
                f"{'available' if f.get('TEGRASTATS_PRESENT') == 'yes' else 'not found'}")
        r.extra = ["This is the board's current mode. The owner picks the field mode. To see the modes: "
                   "grep POWER_MODEL /etc/nvpmodel.conf. To change it (owner, in the login window): "
                   "sudo nvpmodel -m <ID from that list>."]
        return r

    def clock(self) -> Row:
        f = self.f
        if self.virtual:
            return Row('14', 'clock offset', 'SKIP', 'virtual Jetson: same clock as the PC')
        off = _num(f.get('CLOCK_OFFSET_S'))
        rtt = _num(f.get('CLOCK_RTT_MS'))
        if off is None:
            return Row('14', 'clock offset', 'INFO', 'not measured (--quick, or no login)')
        text = f'Jetson clock {"ahead of" if off > 0 else "behind"} the PC by {abs(off):.3f} s (round trip {rtt or 0:.1f} ms)'
        if abs(off) > CLOCK_WARN_S:
            pc = f.get('PC_TIME_UNIX', '')
            return Row('14', 'clock offset', 'WARN', text,
                       cause='the Jetson has no internet time over USB; photo times are corrected by collect.sh, but the '
                             "team's log and sidecar times are off by this much",
                       fix=('the owner types in the login window: sudo date -s @' + (pc.split('.')[0] if pc else '<PC time>')
                            + '   (or connect the Jetson to the internet once)'))
        return Row('14', 'clock offset', 'OK', text)

    def port(self) -> Row:
        if self.virtual:
            return Row('15', 'registry port on the Jetson', 'SKIP', 'virtual Jetson')
        p = self.f.get('PORT_29517', '')
        if p == 'busy':
            lvl = 'FAIL' if self.purpose == 'deploy' else 'WARN'
            return Row('15', 'registry port on the Jetson', lvl, '127.0.0.1:29517 is in use on the Jetson',
                       cause='another program listens on the port deploy needs for a moment',
                       fix=f'{self.j} deploy --registry-port N  (any free port), or --via save', code=1 if lvl == 'FAIL' else 0)
        return Row('15', 'registry port on the Jetson', 'OK', '127.0.0.1:29517 free' if p == 'free' else 'not checked (no ss)')

    def image(self) -> Row:
        f = self.f
        img = 'astra-hil:virtual' if self.virtual else 'astra-hil:jetson'
        need = self.purpose in ('flight', 'start', 'warmup')
        if f.get('DOCKER_ACCESS', 'none') == 'none':
            return Row('16', 'image', 'SKIP', 'needs Docker access first (row 6)')
        iid = f.get('HIL_IMAGE_ID', '')
        if not iid:
            lvl = 'FAIL' if need else 'INFO'
            return Row('16', 'image', lvl, f'{img} is not on {self.host} yet',
                       cause='it was never deployed' if lvl == 'FAIL' else '',
                       fix=(f'{self.j} image build --flavor virtual' if self.virtual else
                            f'{self.j} image build   (on the PC), then {self.j} deploy'),
                       code=E_IMAGE if lvl == 'FAIL' else 0)
        sha, code = f.get('HIL_IMAGE_SHA', ''), f.get('HIL_IMAGE_CODE', '')
        text = (f'{img} built {f.get("HIL_IMAGE_BUILT", "?")} from code {code or "?"} (git {sha or "?"}'
                f'{", TensorRT" if f.get("HIL_IMAGE_TRT") == "yes" else ""}); container: {f.get("HIL_CONTAINER") or "none"}')
        if code and self.pc_code:
            stale, have = code != self.pc_code, f'code {self.pc_code}'
        else:   # an image from before the code fingerprint existed: fall back to the git commit (when both known)
            stale = bool(self.pc_sha and self.pc_sha != 'unknown' and sha and sha != self.pc_sha)
            have = f'git {self.pc_sha}'
        if stale:
            lvl = 'FAIL' if self.purpose in ('flight', 'start') else 'WARN'
            return Row('16', 'image', lvl, text + f' (the PC has {have})',
                       cause=('the team packages or the image files changed on the PC since the image was built'
                              if self.virtual else 'the image on the board is older (or newer) than the code on the PC'),
                       fix=(f'{self.j} image build --flavor virtual' if self.virtual else
                            f'{self.j} image build, then {self.j} deploy'), code=E_IMAGE if lvl == 'FAIL' else 0)
        return Row('16', 'image', 'OK', text)

    def engine(self) -> Row:
        f = self.f
        if self.virtual and f.get('HIL_IMAGE_TRT') != 'yes':
            return Row('16b', 'TensorRT engine', 'SKIP', 'virtual Jetson without TensorRT (plain ROS base): new_od runs '
                                                        'the .pt model with PyTorch on the PC GPU')
        need = self.purpose in ('flight', 'start')
        present = f.get('ENGINE_PRESENT') == 'yes'
        # the virtual Jetson's engine is built with L4T=pc and must be for THIS PC's GPU (one engine per machine)
        meta_l4t, host_l4t = f.get('ENGINE_L4T', ''), ('pc' if self.virtual else f.get('L4T', ''))
        gpu_ok = not (self.virtual and f.get('GPU_NAME') and f.get('ENGINE_GPU') and f['GPU_NAME'] != f['ENGINE_GPU'])
        trt_ok = not f.get('DEPLOYED_TRT') or not f.get('ENGINE_TRT') or f.get('DEPLOYED_TRT') == f.get('ENGINE_TRT')
        model_ok = (not f.get('DEPLOYED_MODEL_SHA') or not f.get('ENGINE_MODEL_SHA')
                    or f.get('DEPLOYED_MODEL_SHA') == f.get('ENGINE_MODEL_SHA'))
        if present and meta_l4t == host_l4t and trt_ok and model_ok and gpu_ok:
            return Row('16b', 'TensorRT engine', 'OK',
                       f"yolo26m.engine for L4T {meta_l4t}, TensorRT {f.get('ENGINE_TRT', '?')}, GPU {f.get('ENGINE_GPU', '?')}")
        why = ('missing' if not present else
               f'built for L4T {meta_l4t or "?"} / TensorRT {f.get("ENGINE_TRT", "?")}' if not (meta_l4t == host_l4t and trt_ok)
               else f'built for another GPU ({f.get("ENGINE_GPU", "?")})' if not gpu_ok
               else 'built for another model file')
        # The legacy photo pipeline's new_od needs the engine; the new video pipeline's need is not known yet (6.3).
        hard = need and self.pipeline == 'legacy'
        return Row('16b', 'TensorRT engine', 'FAIL' if hard else ('WARN' if need else 'INFO'), f'yolo26m.engine {why}',
                   cause='an engine belongs to one board, one TensorRT version and one model file' if (hard or need) else '',
                   fix=(f'{self.j} warmup --link virtual   (a few minutes on this PC\'s GPU; deploy --link virtual does it too)'
                        if self.virtual else
                        f'{self.j} warmup   (once; uses the GPU for several minutes; only when the team is not using the board)'),
                   code=E_ENGINE if hard else 0)

    def rows(self, which: str = 'all') -> List[Row]:
        if which == 'busy':
            return [self.team_busy(), self.memory()]
        out = [self.board(), self.docker(), self.runtime(), self.team_busy(), self.disk(), self.memory(), self.cuda(),
               self.power(), self.clock(), self.port(), self.image()]
        if self.purpose in ('check', 'flight', 'start'):
            out.append(self.engine())
        return out


def first_fail_code(rows: List[Row]) -> int:
    for r in rows:
        if r.level == 'FAIL':
            return r.code or 1
    return 0


def format_row(r: Row, color: bool) -> str:
    """One row in the style of simulation/scripts/_common.sh (ok / warn / die)."""
    def c(code: str, s: str) -> str:
        return f'\033[{code}m{s}\033[0m' if color else s
    head = f'[{r.num:>3}] {r.title}'
    if r.level == 'OK':
        lines = [c('1;32', f' OK {head}: {r.text}')]
    elif r.level in ('INFO', 'SKIP'):
        lines = [f' -- {head}: {r.text}' + (' (skipped)' if r.level == 'SKIP' else '')]
    elif r.level == 'WARN':
        lines = [c('1;33', f' !! [jetson] WARNING: {head}: {r.text}.'
                          + (f' Consequence/likely cause: {r.cause}.' if r.cause else '')
                          + (f' Fix: {r.fix}' if r.fix else ''))]
    else:
        lines = [c('1;31', f' XX [jetson] ERROR: {head}: {r.text}. Likely cause: {r.cause}. Fix: {r.fix}')]
    lines += [f'        {e}' for e in r.extra]
    return '\n'.join(lines)


def status_json(f: Dict[str, str], link: str) -> dict:
    j = Judge(f, 'check', link)
    eng = j.engine()
    return {
        'l4t': f.get('L4T', ''), 'power_mode': f.get('NVPMODEL_MODE', '') or 'unknown',
        'clocks': f.get('CLOCKS', 'unknown'),
        'container': f.get('HIL_CONTAINER') or 'missing',
        'image': f.get('HIL_IMAGE_ID', ''), 'git_sha': f.get('HIL_IMAGE_SHA', ''),
        'code_sha': f.get('HIL_IMAGE_CODE', ''), 'tensorrt': f.get('HIL_IMAGE_TRT', ''),
        'engine': 'ok' if eng.level == 'OK' else ('missing' if f.get('ENGINE_PRESENT') != 'yes' else 'mismatch'),
        'team_busy': bool(busy_evidence(f)) and link != 'virtual',
        'mem_avail_mb': int(_num(f.get('MEM_AVAIL_MB'), 0) or 0),
        'disk_free_gb': int(_num(f.get('DISK_DOCKER_FREE_GB'), 0) or 0),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd')
    for name in ('judge', 'status-json'):
        p = sub.add_parser(name)
        p.add_argument('--facts', required=True)
        p.add_argument('--link', default='usb', choices=['usb', 'ssh', 'virtual'])
        p.add_argument('--for', dest='purpose', default='check', choices=list(NEED_GB))
        p.add_argument('--team-agreed', action='store_true')
        p.add_argument('--pc-git-sha', default='')
        p.add_argument('--pc-code-sha', default='')
        p.add_argument('--pipeline', default='new', choices=['new', 'legacy'])
        p.add_argument('--rows', default='all', choices=['all', 'busy'])
        p.add_argument('--disk-reserve-gb', type=float, default=None)
        p.add_argument('--host', default='')
        p.add_argument('--jetson-cmd', default='bash simulation/jetson/jetson.sh')
        p.add_argument('--color', default='auto', choices=['auto', 'always', 'never'])
    try:
        a = ap.parse_args(argv)
    except SystemExit as e:
        return 2 if e.code else 0
    if not a.cmd:
        ap.print_help()
        return 2
    try:
        with open(a.facts, encoding='utf-8', errors='replace') as fh:
            facts = parse_facts(fh.read())
    except OSError as e:
        print(f'[jetson] ERROR: cannot read the facts file {a.facts} ({e}). Likely cause: the facts were not collected. '
              f'Fix: run the check again.', file=sys.stderr)
        return 2
    if a.disk_reserve_gb is None and os.environ.get('JETSON_DISK_RESERVE_GB'):
        a.disk_reserve_gb = _num(os.environ['JETSON_DISK_RESERVE_GB'])
    if a.cmd == 'status-json':
        print(json.dumps(status_json(facts, a.link)))
        return 0
    judge = Judge(facts, a.purpose, a.link, a.team_agreed, a.pc_git_sha, a.pipeline, a.disk_reserve_gb, a.host,
                  a.jetson_cmd, a.pc_code_sha)
    rows = judge.rows(a.rows)
    color = a.color == 'always' or (a.color == 'auto' and sys.stdout.isatty())
    for r in rows:
        print(format_row(r, color))
    return first_fail_code(rows)


if __name__ == '__main__':
    sys.exit(main())
