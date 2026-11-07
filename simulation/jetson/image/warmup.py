#!/usr/bin/env python3
"""/opt/astra/warmup.py -- build the TensorRT engine for THIS Jetson, once (simulation/jetson/image/warmup.py).

PURPOSE: the legacy detector new_od runs a TensorRT ".engine" on the Jetson GPU. An engine belongs to one GPU, one
TensorRT version and one model file, so it must be built ON the board that runs it (never copied, never baked into
the image). `jetson.sh warmup` runs this once per board / JetPack / model, with the owner's OK, while the team is not
using the board (docs/jetson/HIL_DESIGN.md 9.5). The virtual Jetson (the TensorRT 10.3 image on the PC) runs it too,
with L4T=pc, so it builds its own engine for the PC's GPU from the same model file (`jetson.sh deploy --link virtual`). It uses the TEAM'S OWN code path (detection.model_manager.
resolve_model_path) with new_od's default settings (slice 640, workspace 1 GB, batched inference on with batch size 2,
so a dynamic-batch engine), so the result is exactly the engine new_od would build at its first start. It writes
/models/yolo26m.engine and /models/yolo26m.engine.meta (TRT=, L4T=, GPU=, MODEL_SHA=, BUILT=). The entrypoint runs it
under nice -n 19 ionice -c 3 inside the container (the GPU part itself cannot be made low priority).
Exit codes: 0 built (or already up to date), 1 the build failed, 2 no model file.
"""
import hashlib
import os
import sys
import time

MODEL = '/home/astra-dev/astra/ros2_ws/src/yolo26m.pt'    # new_od's default model_path (a link into /models)
ENGINE = '/models/yolo26m.engine'
META = '/models/yolo26m.engine.meta'


def sha16(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()[:16]


def main():
    if not os.path.isfile('/models/yolo26m.pt'):
        print('[warmup] ERROR: /models/yolo26m.pt is missing. Likely cause: deploy did not copy a model. '
              'Fix: jetson.sh deploy --model FILE', file=sys.stderr)
        return 2
    import rclpy.logging
    import tensorrt
    import torch
    from detection.model_manager import resolve_model_path, MODEL_FORMAT_AUTO, MODEL_FORMAT_TENSORRT
    logger = rclpy.logging.get_logger('warmup')
    msha = sha16('/models/yolo26m.pt')
    l4t = os.environ.get('L4T', '')
    gpu = torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none'
    if os.path.isfile(ENGINE) and os.path.isfile(META):
        meta = dict(l.strip().split('=', 1) for l in open(META, encoding='utf-8') if '=' in l)
        if (meta.get('TRT') == tensorrt.__version__ and meta.get('MODEL_SHA') == msha and meta.get('L4T') == l4t
                and meta.get('GPU', gpu) == gpu):
            print(f'[warmup] the engine is already up to date ({ENGINE}: TensorRT {tensorrt.__version__}, L4T {l4t}, '
                  f'GPU {gpu}).')
            return 0
        print('[warmup] the engine is for another TensorRT / L4T / GPU / model: building it again')
        os.replace(ENGINE, ENGINE + '.old')
    t0 = time.time()
    print(f'[warmup] building the TensorRT engine (TensorRT {tensorrt.__version__}, GPU {gpu}); several minutes...',
          flush=True)
    path, fmt = resolve_model_path(MODEL, MODEL_FORMAT_AUTO, True, 640, 640, 1, 'cuda:0', logger, max_batch=2)
    if fmt != MODEL_FORMAT_TENSORRT or not os.path.isfile(ENGINE):
        print(f'[warmup] ERROR: no engine was made (the team code returned {path}, {fmt}). Likely cause: the TensorRT '
              'export failed (see the lines above), or not enough memory. Fix: read the lines above; the team notes '
              'in ros2_ws/src/fixtensorRT.md', file=sys.stderr)
        return 1
    with open(META, 'w', encoding='utf-8') as fh:
        fh.write(f'TRT={tensorrt.__version__}\nL4T={l4t}\nGPU={gpu}\nMODEL_SHA={msha}\nMAX_BATCH=2\nSLICE=640\n'
                 f'BUILT={time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}\n')
    print(f'[warmup] done in {time.time() - t0:.0f} s: {ENGINE} + {META}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
