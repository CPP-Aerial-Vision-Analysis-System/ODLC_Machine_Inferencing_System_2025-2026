"""
sim_plugins/model_check.py -- PURPOSE: check a DETECTOR MODEL FILE (the YOLO weights the team's new_od loads)
BEFORE a flight, so a wrong file stops the demo in the first seconds with a clear message instead of after five
minutes of flying with a detector that never loaded. It also writes down exactly WHICH model flew (name, size,
SHA-256 fingerprint) for the run folder (run_meta.py puts it in <run>/run_meta/detector.json).

What it checks (quick_check: no ROS, no PyTorch, a fraction of a second):
  * the file exists, is not empty and has a path without spaces (./launcher.sh cannot pass a path with a space);
  * the format from the extension: .pt (PyTorch, works everywhere), .engine (TensorRT: only on the computer it
    was built for, normally the Jetson Orin Nano, and only where the tensorrt module exists), .onnx (untested
    in new_od; needs onnxruntime); anything else is refused;
  * a .pt really looks like PyTorch weights (a zip archive or an old pickle), not a web page saved by mistake;
  * a .engine with the SAME name next to a .pt: new_od's model_format "auto" loads the .engine FIRST (warned);
  * the SHA-256 fingerprint and the size (so two runs can be compared: "was it really the same model?").
What load_check adds (PyTorch + Ultralytics, ~5-15 s):
  * the model really loads (Ultralytics YOLO, like new_od), on the GPU when CUDA works, else on the CPU (warned,
    with the MEASURED time per 640 px tile and the expected time per photo);
  * its class names: the team code (detection/detection_processor.py CLASS_MAP) only turns person / mannequin /
    doll into a PERSON and umbrella / kite into a TENT. A model without any of those finds nothing the drone
    can use (warned), and a class literally called "tent" is ignored by the team code (warned, with the fix).

Command line (inside Ubuntu, workspace sourced):
    python3 -m sim_plugins.model_check ~/.astra_sim/models/my_detector.pt            quick check
    python3 -m sim_plugins.model_check ~/.astra_sim/models/my_detector.pt --load     ... and load it once
    ros2 run sim_plugins model_check FILE [--load] [--device auto|cpu|cuda:0] [--no-hash] [--json OUT]
Exit codes: 0 = the model can be used (warnings allowed), 3 = it cannot (the reasons are printed, each with a
fix), 2 = bad usage. (3, not 1, so a script can tell "bad model" from "Python could not even start the check".)
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

#: The formats new_od can load (detection/model_manager.py), by file extension.
FORMATS = {'.pt': 'pytorch', '.engine': 'tensorrt', '.onnx': 'onnx'}

#: A COPY of the team's class map (detection/detection/detection_processor.py CLASS_MAP: YOLO class name ->
#: what main_controller gets). test_model_check.py compares it with the team file, so a change there shows up.
TEAM_CLASS_MAP = {'person': 'person', 'mannequin': 'person', 'doll': 'person', 'umbrella': 'tent', 'kite': 'tent'}

#: new_od's tiling (detection/new_od.py DEFAULT_SLICE / DEFAULT_OVERLAP): 640 px tiles with 15 % overlap.
TILE_PX = 640
TILE_OVERLAP = 0.15

#: Photo sizes of the two camera profiles (zr10 = the simulator's default, a8mini = 4K).
PHOTO_SIZES = {'zr10': (2560, 1440), 'a8mini': (3840, 2160)}

#: The team code takes one photo per second; a detector slower than this falls behind the camera.
PHOTO_PERIOD_S = 1.0


@dataclass
class ModelReport:
    """Everything known about one model file.

    path        the file as given (expanded)
    name        the file name
    format      pytorch | tensorrt | onnx | unknown
    size_bytes  file size (0 when missing)
    sha256      the SHA-256 fingerprint ('' when not computed)
    errors      reasons it cannot be used (each says what, why and how to fix it)
    warnings    things that work but need a look (each says the consequence)
    notes       information
    load        load_check's results ({} when it did not run)
    """
    path: str
    name: str = ''
    format: str = 'unknown'
    size_bytes: int = 0
    sha256: str = ''
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    load: Dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """True when nothing stops the model from being used."""
        return not self.errors

    def to_dict(self) -> Dict:
        """A JSON-ready dict (run_meta.py writes it into the run folder)."""
        out = asdict(self)
        out['ok'] = self.ok
        out['size_mb'] = round(self.size_bytes / 1e6, 1)
        return out

    def render(self) -> str:
        """Printable lines in the style of the other tools ( OK / !! / XX / .. )."""
        lines = []
        if self.size_bytes:
            # ' OK' only for a model that can be used; a bad one gets the same facts with ' XX' (reasons below)
            mark = ' OK' if self.ok else ' XX'
            digest = f', sha256 {self.sha256[:16]}...' if self.sha256 else ''
            lines.append(f'{mark} model {self.name}: {self.format}, {self.size_bytes / 1e6:.1f} MB{digest}')
        lines += [f' .. {n}' for n in self.notes]
        lines += [f' !! {w}' for w in self.warnings]
        lines += [f' XX {e}' for e in self.errors]
        return '\n'.join(lines)


def expand(path: str) -> str:
    """~ and $VARS expanded (no other change)."""
    return os.path.expandvars(os.path.expanduser(str(path or '').strip()))


def sha256_of(path: str, chunk: int = 1 << 20) -> str:
    """The SHA-256 fingerprint of a file, read in 1 MB pieces (a 40 MB model takes well under a second)."""
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(chunk), b''):
            digest.update(block)
    return digest.hexdigest()


def module_available(name: str) -> bool:
    """True when a Python module could be imported here (without importing it: no slow PyTorch start)."""
    try:
        import importlib.util
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def _first_bytes(path: str, n: int = 8) -> bytes:
    try:
        with open(path, 'rb') as f:
            return f.read(n)
    except OSError:
        return b''


def looks_like_pytorch(head: bytes) -> bool:
    """True for the start of a PyTorch weights file: a zip archive (torch >= 1.6) or a pickle (older)."""
    return head.startswith(b'PK\x03\x04') or head.startswith(b'\x80')


def quick_check(path: str, find_module: Callable[[str], bool] = module_available,
                compute_hash: bool = True) -> ModelReport:
    """The fast checks (see the module docstring). Never raises: every problem goes into the report."""
    path = expand(path)
    rep = ModelReport(path=path, name=Path(path).name if path else '')
    fix_default = ('Fix: give an existing weights file with --model FILE (for example the default '
                   '~/.astra_sim/models/yolo26m.pt, downloaded by simulation/setup/03_build.sh), or fly with '
                   '--detector truth.')
    if not path:
        rep.errors.append('no model file was given. ' + fix_default)
        return rep
    if ' ' in path:
        rep.errors.append(f'the model path "{path}" contains a space. Why: ./launcher.sh passes the path on a '
                          'command line, where a space splits it in two. Fix: move the file to a folder without '
                          'spaces, e.g. cp "<file>" ~/.astra_sim/models/')
    p = Path(path)
    if not p.is_file():
        rep.errors.append(f'the model file {path} does not exist. Likely cause: a typo in the path, or the file is '
                          'still on Windows (copy it into Ubuntu first). ' + fix_default)
        return rep
    rep.size_bytes = p.stat().st_size
    if rep.size_bytes == 0:
        rep.errors.append(f'the model file {path} is empty (0 bytes). Likely cause: a download that failed. '
                          + fix_default)
        return rep
    rep.format = FORMATS.get(p.suffix.lower(), 'unknown')
    if compute_hash:
        try:
            rep.sha256 = sha256_of(path)
        except OSError as exc:
            rep.errors.append(f'cannot read {path}: {exc}. Fix: check the file permissions (chmod a+r FILE).')
            return rep
    head = _first_bytes(path)
    if rep.format == 'unknown':
        rep.errors.append(f'{p.name}: the extension "{p.suffix or "(none)"}" is not a model format new_od loads. '
                          'Why: new_od (detection/model_manager.py) loads .pt (PyTorch) and .engine (TensorRT). '
                          'Fix: export your model to .pt (Ultralytics: model.save("my.pt")) and give that file.')
    elif rep.format == 'pytorch':
        if not looks_like_pytorch(head):
            what = 'a web page (HTML)' if head.lstrip().startswith(b'<') else f'bytes {head[:4]!r}'
            rep.errors.append(f'{p.name} does not look like PyTorch weights (it starts with {what}). Likely cause: '
                              'a broken or wrong download, or a file renamed to .pt. Fix: download / export the '
                              'weights again and check: python3 -c "from ultralytics import YOLO; YOLO(\'FILE\')"')
        sibling = p.with_suffix('.engine')
        if sibling.is_file():
            rep.warnings.append(f'{sibling.name} sits next to {p.name}: new_od (model_format auto) loads the .engine '
                                'FIRST. Consequence: on this PC it fails and falls back to the .pt after an error, on '
                                'the Jetson the .engine flies instead of your .pt. Fix: move the .engine away, or give '
                                'the .pt from a folder of its own.')
    elif rep.format == 'tensorrt':
        twin = p.with_suffix('.pt')
        if not find_module('tensorrt'):
            if twin.is_file():
                rep.warnings.append(f'{p.name} is a TensorRT engine and this computer has no TensorRT. Consequence: '
                                    f'new_od logs a load error, then falls back to {twin.name} next to it (so the '
                                    '.pt flies, not the engine). Fix: give the .pt here; test the .engine on the '
                                    'Jetson Orin Nano (or in the Jetson container).')
            else:
                rep.errors.append(f'{p.name} is a TensorRT engine, but this computer has no TensorRT (no "tensorrt" '
                                  'Python module), and there is no .pt with the same name next to it. Why: an '
                                  '.engine only runs where TensorRT is installed and only on the GPU / TensorRT '
                                  'version it was built for (normally the Jetson Orin Nano). Fix: on this PC give '
                                  'the .pt weights (--model my_detector.pt); fly the .engine on the Jetson.')
        else:
            rep.notes.append(f'{p.name} is a TensorRT engine: it only loads on the GPU and TensorRT version it was '
                             'built with (load it once with --load to be sure)')
        rep.notes.append('the team\'s batched path (detector.batched: true) needs a TensorRT engine; the default '
                         'SAHI path works with .pt and .engine')
    elif rep.format == 'onnx':
        if not find_module('onnxruntime'):
            rep.errors.append(f'{p.name} is an ONNX model, and this computer has no onnxruntime. Why: Ultralytics '
                              'needs onnxruntime to run .onnx files. Fix: give the .pt weights instead (preferred: '
                              'new_od is only tested with .pt and .engine), or: pip install onnxruntime-gpu')
        else:
            rep.warnings.append(f'{p.name} is an ONNX model: new_od was never tested with ONNX. Consequence: it may '
                                'load slower or not at all. Prefer the .pt weights.')
    return rep


# ---------------------------------------------------------------------------------------------
# The load test (needs PyTorch + Ultralytics: inside Ubuntu with the team's Python stack)
# ---------------------------------------------------------------------------------------------

def tiles_per_photo(width: int, height: int, tile: int = TILE_PX, overlap: float = TILE_OVERLAP) -> int:
    """How many tiles new_od cuts a photo into (SAHI: tile size, overlap ratio), plus SAHI's one full-photo pass."""
    step = tile * (1.0 - overlap)

    def count(size: int) -> int:
        return 1 if size <= tile else int(math.ceil((size - tile) / step)) + 1
    return count(width) * count(height) + 1


def class_coverage(names: Sequence[str]) -> Dict[str, List[str]]:
    """Which of the model's class names the team code turns into a person / a tent, and which literal
    "tent"-like names it would IGNORE (they are not in the team's class map)."""
    low = [str(n).strip().lower() for n in names]
    return {'person': [n for n in low if TEAM_CLASS_MAP.get(n) == 'person'],
            'tent': [n for n in low if TEAM_CLASS_MAP.get(n) == 'tent'],
            'ignored_tent_like': [n for n in low if 'tent' in n and n not in TEAM_CLASS_MAP]}


def coverage_findings(names: Sequence[str]) -> tuple:
    """(warnings, notes) about the model's classes for the team code (see class_coverage)."""
    cov = class_coverage(names)
    warnings, notes = [], []
    if not cov['person']:
        warnings.append('the model has no class the team code turns into a PERSON (person, mannequin, doll). '
                        'Consequence: the hiker is never found and nothing is dropped on it.')
    if not cov['tent']:
        warnings.append('the model has no class the team code turns into a TENT (umbrella, kite). Consequence: the '
                        'tent is never found and nothing is dropped on it.')
    if cov['ignored_tent_like']:
        warnings.append(f'the model has the class(es) {", ".join(cov["ignored_tent_like"])}, but the team code\'s '
                        'CLASS_MAP (detection/detection_processor.py) does not know them, so those boxes are '
                        'IGNORED. Fix: name the class "umbrella" when you train / export, or add the name to '
                        'CLASS_MAP in the team code (a team-code change: see docs/simulation/SWAPPING.md).')
    if cov['person'] or cov['tent']:
        notes.append(f'classes the team code uses: person <- {", ".join(cov["person"]) or "-"}; '
                     f'tent <- {", ".join(cov["tent"]) or "-"}')
    return warnings, notes


def load_check(rep: ModelReport, device: str = 'auto', camera: str = 'zr10') -> ModelReport:
    """Load the model once like new_od (Ultralytics YOLO) and time one 640 px tile. Adds to rep and returns it.

    Never raises: a load failure becomes an error with the reason; a missing GPU becomes a warning with the
    measured CPU time per tile and the expected time per photo.
    """
    if rep.errors or rep.format == 'unknown':
        return rep
    t0 = time.monotonic()
    try:
        import numpy as np
        import torch
        from ultralytics import YOLO
    except Exception as exc:  # noqa: BLE001 - a broken ML stack must become a message, never a crash
        rep.errors.append(f'cannot load the model: PyTorch / Ultralytics are not importable here ({exc}). Why: the '
                          'team\'s Python stack is missing or broken. Fix: sudo bash '
                          'simulation/setup/01_install_humble.sh (installs it; numpy must stay < 2).')
        return rep
    cuda = bool(torch.cuda.is_available())
    dev = ('cuda:0' if cuda else 'cpu') if device == 'auto' else device
    info: Dict = {'device': dev, 'cuda_available': cuda,
                  'gpu': torch.cuda.get_device_name(0) if cuda else '', 'torch': torch.__version__}
    try:
        model = YOLO(rep.path, task='detect')
        names_obj = getattr(model, 'names', {}) or {}
        names = list(names_obj.values()) if isinstance(names_obj, dict) else list(names_obj)
        info['load_s'] = round(time.monotonic() - t0, 2)
        tile = np.zeros((TILE_PX, TILE_PX, 3), dtype=np.uint8)
        model.predict(tile, device=dev, imgsz=TILE_PX, verbose=False)        # warm-up (first call is slow)
        t1 = time.monotonic()
        runs = 3
        for _ in range(runs):
            model.predict(tile, device=dev, imgsz=TILE_PX, verbose=False)
        ms = (time.monotonic() - t1) / runs * 1000.0
    except Exception as exc:  # noqa: BLE001 - every load problem becomes a message
        rep.errors.append(f'the model {rep.name} did not load: {type(exc).__name__}: {str(exc)[:300]}. Likely '
                          'cause: not a YOLO detection model, made with a much newer/older Ultralytics, or a '
                          'TensorRT engine built for another GPU. Fix: export it again with Ultralytics '
                          '8.4.x (the team pins 8.4.75) as .pt, then run this check again.')
        rep.load = info
        return rep
    w, h = PHOTO_SIZES.get(camera, PHOTO_SIZES['zr10'])
    tiles = tiles_per_photo(w, h)
    per_photo = ms * tiles / 1000.0
    info.update({'classes': names, 'num_classes': len(names), 'ms_per_tile': round(ms, 1),
                 'tiles_per_photo': tiles, 'photo_size': f'{w}x{h}', 's_per_photo': round(per_photo, 2)})
    rep.load = info
    on_gpu = dev.startswith('cuda')
    gpu_text = f' ({info["gpu"]})' if info['gpu'] and on_gpu else ''
    rep.notes.append(f'loaded in {info["load_s"]:.1f} s on {dev}{gpu_text}: '
                     f'{len(names)} class(es), {ms:.0f} ms per 640 px tile, ~{per_photo:.1f} s per {w}x{h} photo '
                     f'({tiles} tiles)')
    if not on_gpu:
        why = 'no CUDA GPU is usable here' if not cuda else f'--device {dev} was asked for'
        fix = ('Fix in WSL: update the Windows NVIDIA driver, then nvidia-smi inside Ubuntu must show the card.'
               if not cuda else f'Fix: leave --device at auto ({info["gpu"]} is usable).')
        rep.warnings.append(f'the detector runs on the CPU ({why}): measured {ms:.0f} ms per tile = ~{per_photo:.1f} s '
                            f'per photo, typically 10-30x slower than on a GPU (that factor is expected, not measured '
                            'here). Consequence: the camera takes 1 photo per second, so new_od falls behind; '
                            'main_controller waits at most 90 s at the last waypoint and then flies with what '
                            f'was checked. {fix}')
    elif per_photo > 3 * PHOTO_PERIOD_S:
        rep.warnings.append(f'~{per_photo:.1f} s per photo even on the GPU: new_od falls behind the 1 photo per '
                            'second camera (main_controller waits at most 90 s at the last waypoint).')
    warns, notes = coverage_findings(names)
    rep.warnings += warns
    rep.notes += notes
    return rep


#: Exit code of the command line when the model cannot be used (run_meta detector uses the same).
EXIT_UNUSABLE = 3


def check(path: str, load: bool = False, device: str = 'auto', camera: str = 'zr10',
          compute_hash: bool = True) -> ModelReport:
    """quick_check, then (when asked and nothing is wrong yet) load_check."""
    rep = quick_check(path, compute_hash=compute_hash)
    if load:
        load_check(rep, device=device, camera=camera)
    return rep


# ---------------------------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    """The argument parser (see the module docstring)."""
    parser = argparse.ArgumentParser(prog='model_check', description=__doc__.split('\n\n')[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('model', help='the weights file (.pt, .engine, .onnx)')
    parser.add_argument('--load', action='store_true', help='also load it once (PyTorch + Ultralytics, ~5-15 s)')
    parser.add_argument('--device', default='auto', help='auto (GPU when CUDA works) | cpu | cuda:0')
    parser.add_argument('--camera', default='zr10', choices=sorted(PHOTO_SIZES), help='photo size for the estimate')
    parser.add_argument('--no-hash', action='store_true', help='skip the SHA-256 fingerprint (a quick look only)')
    parser.add_argument('--json', default='', help='also write the report as JSON to this file')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Command-line entry point (see the module docstring)."""
    args = build_parser().parse_args(list(sys.argv[1:] if argv is None else argv))
    rep = check(args.model, load=args.load, device=args.device, camera=args.camera, compute_hash=not args.no_hash)
    print(rep.render(), flush=True)
    if args.json:
        try:
            Path(args.json).parent.mkdir(parents=True, exist_ok=True)
            Path(args.json).write_text(json.dumps(rep.to_dict(), indent=2) + '\n', encoding='utf-8')
        except OSError as exc:
            print(f' !! [model_check] could not write {args.json}: {exc}', file=sys.stderr)
    if not rep.ok:
        print(f'[model_check] ERROR: the model {rep.path or "(none)"} cannot be used (reasons above, each with '
              'its fix).', file=sys.stderr)
    return 0 if rep.ok else EXIT_UNUSABLE


if __name__ == '__main__':
    sys.exit(main())
