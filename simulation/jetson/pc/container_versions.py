#!/usr/bin/env python3
"""container_versions.py -- print the software versions INSIDE the team-stack container, as KEY=VALUE lines.

PURPOSE (plain words): the owner wants every Jetson-in-the-loop run summary to show "the Jetson's software, version
for version" (owner entry 12:32): the operating system, CUDA, cuDNN, TensorRT, ROS 2, MAVROS, PyTorch, Ultralytics,
numpy, OpenCV and the GPU that the container really had during THIS flight. demo.sh --jetson (hil_steps.sh) runs this
file inside the running container, on the real Jetson or on the virtual Jetson, by sending it on standard input:

    bash simulation/jetson/jetson.sh exec [TARGET] -- python3 - < simulation/jetson/pc/container_versions.py

so nothing has to be installed in the image for it. It only READS (no file is written, nothing is started) and never
fails: a part that cannot be found prints an empty value. The PC writes the output to RUN_DIR/jetson/versions.env and
hil_pc.py summary turns it into the "Versions" table next to the field Jetson's versions.
Uses only the Python standard library at the top level; every optional import is guarded.
"""
import os
import platform
import re
import subprocess


def out(key, value):
    """One KEY=VALUE line (newlines and '=' signs in the value are made harmless)."""
    text = str(value if value is not None else '').replace('\n', ' ').strip()
    print(f'{key}={text}', flush=True)


def read_file(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            return fh.read()
    except OSError:
        return ''


def dpkg_version(pattern):
    """The version of the first installed Debian package matching pattern ('' if none)."""
    try:
        res = subprocess.run(['dpkg-query', '-W', '-f', '${Package} ${Version}\n', pattern],
                             capture_output=True, text=True, timeout=20)
    except Exception:  # noqa: BLE001  (no dpkg: not Debian/Ubuntu)
        return ''
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[1]:
            return parts[1]
    return ''


def module_version(name):
    """__version__ of a Python module ('' when it cannot be imported)."""
    try:
        mod = __import__(name)
        return getattr(mod, '__version__', '') or ''
    except Exception:  # noqa: BLE001
        return ''


def main():
    osr = read_file('/etc/os-release')
    m = re.search(r'^PRETTY_NAME="?([^"\n]*)', osr, re.M)
    out('OS', m.group(1) if m else '')
    out('ARCH', platform.machine())
    out('PYTHON', platform.python_version())
    image = dict(line.strip().split('=', 1) for line in read_file('/opt/astra/image.env').splitlines() if '=' in line)
    out('IMAGE_FLAVOR', image.get('FLAVOR', ''))
    out('IMAGE_GIT_SHA', image.get('GIT_SHA', ''))
    out('IMAGE_BUILT', image.get('BUILT', ''))
    tegra = read_file('/etc/nv_tegra_release').splitlines()
    out('L4T_IN_IMAGE', tegra[0].strip() if tegra else '')
    out('ROS_DISTRO', os.environ.get('ROS_DISTRO', ''))
    mavros_xml = read_file('/opt/ros/humble/share/mavros/package.xml')
    m = re.search(r'<version>([^<]+)</version>', mavros_xml)
    out('MAVROS', m.group(1) if m else '')
    # CUDA / cuDNN as PyTorch sees them, plus the toolkit packages in the image
    torch_v = cuda = cudnn = gpu = ''
    try:
        import torch
        torch_v = torch.__version__
        cuda = getattr(torch.version, 'cuda', '') or ''
        try:
            raw = int(torch.backends.cudnn.version() or 0)
            # PyTorch prints cuDNN as one number: 90300 = 9.3.0 (cuDNN 9: M*10000 + m*100 + p; cuDNN 8: M*1000 ...)
            if raw >= 10000:
                cudnn = f'{raw // 10000}.{(raw % 10000) // 100}.{raw % 100}'
            elif raw > 0:
                cudnn = f'{raw // 1000}.{(raw % 1000) // 100}.{raw % 100}'
        except Exception:  # noqa: BLE001
            cudnn = ''
        gpu = torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'no GPU visible to PyTorch'
    except Exception as e:  # noqa: BLE001
        gpu = f'PyTorch not importable ({type(e).__name__})'
    out('TORCH', torch_v)
    out('TORCH_CUDA', cuda)
    out('TORCH_CUDNN', cudnn)
    out('GPU', gpu)
    out('CUDA_RUNTIME_PKG', dpkg_version('cuda-cudart-12-*') or dpkg_version('cuda-cudart-*'))
    out('CUDNN_PKG', dpkg_version('libcudnn9-cuda-12') or dpkg_version('libcudnn9*'))
    out('TENSORRT_PKG', dpkg_version('libnvinfer10') or dpkg_version('libnvinfer*'))
    out('TENSORRT_PY', module_version('tensorrt'))
    out('TORCHVISION', module_version('torchvision'))
    out('ULTRALYTICS', module_version('ultralytics'))
    out('NUMPY', module_version('numpy'))
    out('OPENCV_PY', module_version('cv2'))
    out('ONNXRUNTIME', module_version('onnxruntime'))
    # the model files the detector can use (an engine next to the .pt means TensorRT at start)
    for key, path in (('MODEL_PT', '/models/yolo26m.pt'), ('MODEL_ENGINE', '/models/yolo26m.engine')):
        try:
            out(key, f'{path} {os.path.getsize(path) // (1024 * 1024)} MiB')
        except OSError:
            out(key, '')
    meta = read_file('/models/yolo26m.engine.meta').split()
    out('ENGINE_META', ' '.join(meta))


if __name__ == '__main__':
    main()
