#!/usr/bin/env python3
"""/opt/astra/selftest.py -- does this image have what the team stack needs? (simulation/jetson/image/selftest.py)

PURPOSE: `jetson.sh deploy` (and `check --deep`) runs this inside the freshly deployed image, on the Jetson or on the
PC (virtual Jetson), BEFORE any flight. It prints one line per item (OK / WARN / FAIL) and a few KEY=VALUE lines the
kit records in ~/astra_hil/deployed.env (TRT=, TORCH=, CUDA=): the GPU seen by PyTorch, TensorRT for Python (the
legacy detector new_od exports and loads .engine files with it), numpy below 2 (the team's pins), the OpenCV
versions found (two side by side is a known risk on L4T, HIL_DESIGN.md risk 17), every team ROS package plus MAVROS,
the photo folder the team code resolves, and the model files on /models. Exit code 1 when a FAIL line was printed.
Options: --no-gpu (a build check on a machine without a GPU: the GPU rows become WARN).
"""
import os
import subprocess
import sys

FAILS = []
NO_GPU = '--no-gpu' in sys.argv
FLAVOR = os.environ.get('ASTRA_FLAVOR', 'virtual')
PACKAGES = ['interfaces', 'video_cam', 'main', 'detection', 'mapping', 'waypoint_mavros', 'wp_sender', 'gps_ros2',
            'sim_bringup', 'mavros', 'mavros_msgs']
OPTIONAL_PACKAGES = ['zr10_ros2', 'zr10_stream']


def line(level, what, text):
    print(f' {level:4} {what}: {text}', flush=True)
    if level == 'FAIL':
        FAILS.append(what)


def gpu_level():
    return 'WARN' if NO_GPU else 'FAIL'


def main():
    env = {}
    for path in ('/opt/astra/image.env', '/opt/astra/paths.env'):
        try:
            with open(path, encoding='utf-8') as fh:
                env.update(dict(l.strip().split('=', 1) for l in fh if '=' in l))
        except OSError:
            line('FAIL', 'image files', f'{path} is missing (broken image: rebuild it)')
    line('OK', 'image', f"flavor {FLAVOR}, git {env.get('GIT_SHA', '?')}, built {env.get('BUILT', '?')}, "
                        f"arch {os.uname().machine}")
    # PyTorch + GPU
    try:
        import torch
        cuda = torch.cuda.is_available()
        dev = torch.cuda.get_device_name(0) if cuda else 'no GPU visible'
        print(f'TORCH={torch.__version__}')
        print(f'CUDA={getattr(torch.version, "cuda", "") or ""}')
        line('OK' if cuda else gpu_level(), 'PyTorch + GPU',
             f'torch {torch.__version__} (CUDA {torch.version.cuda}), {dev}'
             + ('' if cuda else '. Likely cause: the container was started without the GPU (--runtime nvidia on '
                                'the Jetson, --gpus all on the PC). Fix: jetson.sh check (row 7)'))
    except Exception as e:  # noqa: BLE001
        line('FAIL', 'PyTorch', f'import failed: {e}. Likely cause: a broken or mismatched torch wheel '
                                f'(HIL_DESIGN.md risk 18). Fix: rebuild the image')
    # TensorRT for Python
    try:
        import tensorrt
        print(f'TRT={tensorrt.__version__}')
        line('OK', 'TensorRT (Python)', tensorrt.__version__)
    except Exception as e:  # noqa: BLE001
        print('TRT=')
        line('FAIL' if FLAVOR == 'jetson' else 'WARN', 'TensorRT (Python)',
             f'not importable ({e})' + ('' if FLAVOR == 'jetson' else ' (the virtual Jetson runs the .pt model)'))
    # ONNX: Ultralytics writes an ONNX file before every TensorRT export (warmup, or new_od's first start) and needs
    # onnx >= 1.12 that imports next to numpy 1.x (apt's python3-onnx 1.7.0 does not: np.object is gone)
    has_trt = os.environ.get('ASTRA_TENSORRT', 'yes' if FLAVOR == 'jetson' else 'no') == 'yes'
    try:
        import onnx
        parts = [int(x) for x in onnx.__version__.split('.')[:2]]
        good = parts >= [1, 12]
        line('OK' if good else ('FAIL' if has_trt else 'WARN'), 'ONNX (for the TensorRT export)',
             onnx.__version__ + ('' if good else ': Ultralytics needs onnx >= 1.12. Fix: rebuild the image'))
    except Exception as e:  # noqa: BLE001
        line('FAIL' if has_trt else 'WARN', 'ONNX (for the TensorRT export)',
             f'not importable ({e})' + ('. Likely cause: an old apt python3-onnx next to numpy 1.26. Fix: rebuild the '
                                        'image (layer 3 installs onnx 1.14.0)' if has_trt else
                                        ' (this image has no TensorRT, so no engine is ever exported)'))
    # numpy
    try:
        import numpy
        major = int(numpy.__version__.split('.')[0])
        line('OK' if major < 2 else 'FAIL', 'numpy', f'{numpy.__version__}' + ('' if major < 2 else
             ': the team pins numpy < 2 (torch / cv_bridge break). Fix: rebuild the image'))
    except Exception as e:  # noqa: BLE001
        line('FAIL', 'numpy', str(e))
    # OpenCV (Python and the apt libraries)
    try:
        import cv2
        cvv = cv2.__version__
    except Exception as e:  # noqa: BLE001
        cvv = f'not importable ({e})'
    try:
        libs = subprocess.run(['dpkg-query', '-W', '-f', '${Package}=${Version} ', 'libopencv-core*', 'nvidia-opencv*'],
                              capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception:  # noqa: BLE001
        libs = ''
    line('OK', 'OpenCV', f'python cv2 {cvv}; packages: {libs or "none found"}')
    # ROS packages
    try:
        from ament_index_python.packages import get_package_prefix
        missing = []
        for p in PACKAGES:
            try:
                get_package_prefix(p)
            except Exception:  # noqa: BLE001
                missing.append(p)
        line('FAIL' if missing else 'OK', 'ROS packages',
             ('missing: ' + ', '.join(missing) + '. Fix: rebuild the image') if missing else ', '.join(PACKAGES))
        extra = []
        for p in OPTIONAL_PACKAGES:
            try:
                get_package_prefix(p)
                extra.append(p)
            except Exception:  # noqa: BLE001
                pass
        line('OK' if extra else 'WARN', 'new video pipeline packages',
             ', '.join(extra) if extra else 'not in this image (the legacy photo pipeline runs; rebuild once the video '
                                            'workflow delivered zr10_ros2 / zr10_stream)')
    except Exception as e:  # noqa: BLE001
        line('FAIL', 'ROS packages', f'ROS is not sourced ({e})')
    # photo folder
    try:
        from sim_bringup.sim_paths import team_photo_dir
        got = team_photo_dir()
        want = env.get('PHOTO_DIR', '')
        line('OK' if got == want else 'FAIL', 'photo folder',
             got + ('' if got == want else f' (paths.env says {want}: the mounts would miss it; rebuild the image)'))
    except Exception as e:  # noqa: BLE001
        line('WARN', 'photo folder', f'could not ask sim_paths ({e})')
    # model files
    pt = '/models/yolo26m.pt'
    line('OK' if os.path.isfile(pt) else 'WARN', 'model', pt + ('' if os.path.isfile(pt) else
         ' is missing (jetson.sh deploy copies it; --model FILE)'))
    eng, meta = '/models/yolo26m.engine', '/models/yolo26m.engine.meta'
    if os.path.isfile(eng) and os.path.isfile(meta):
        with open(meta, encoding='utf-8') as fh:
            line('OK', 'TensorRT engine', ' '.join(l.strip() for l in fh if '=' in l))
    elif FLAVOR == 'jetson':
        line('WARN', 'TensorRT engine', 'not built yet. Next: jetson.sh warmup (once, with the owner\'s OK)')
    print('SELFTEST=' + ('failed: ' + ', '.join(FAILS) if FAILS else 'ok'))
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
