import os
import sys
import gc
import subprocess
from contextlib import contextmanager

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


@contextmanager
def suppress_native_output():
    """Silence C-level stdout/stderr (e.g. TensorRT logs)."""
    sys.stdout.flush()
    sys.stderr.flush()
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_stdout = os.dup(1)
    saved_stderr = os.dup(2)
    try:
        os.dup2(devnull_fd, 1)
        os.dup2(devnull_fd, 2)
        yield
    finally:
        os.dup2(saved_stdout, 1)
        os.dup2(saved_stderr, 2)
        os.close(saved_stdout)
        os.close(saved_stderr)
        os.close(devnull_fd)

def detect_device(logger) -> str:
    """Auto-detect compute device: CUDA or CPU."""
    if not TORCH_AVAILABLE:
        logger.warn("PyTorch not available, falling back to CPU")
        return "cpu"

    if torch.cuda.is_available():
        logger.info(f"CUDA GPU detected: {torch.cuda.get_device_name(0)}")
        with suppress(OSError, IOError):
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read()
                if 'jetson' in model.lower():
                    logger.info(f"Platform: NVIDIA Jetson ({model.strip()})")
        return "cuda:0"

    logger.warn("No GPU detected, using CPU")
    with suppress(OSError, IOError, FileNotFoundError):
        with open('/proc/device-tree/model', 'r') as f:
            if 'jetson' in f.read().lower():
                logger.error("Jetson detected but GPU not available! Check CUDA install.")
    return "cpu"

def check_jetson_power_mode(logger) -> None:
    # Warn if Jetson is not in MAXN power mode or jetson_clocks is not active. Free 20-40% on Orin: MAXN unlocks all CPU/GPU clocks, jetson_clocks pins them to max. Without this, the platform throttles aggressively.
    is_jetson = False
    try:
        with open('/proc/device-tree/model', 'r') as f:
            is_jetson = 'jetson' in f.read().lower()
    except (OSError, IOError, FileNotFoundError):
        return  # not a Jetson, nothing to check

    if not is_jetson:
        return

    # Power mode (nvpmodel)
    try:
        out = subprocess.run(
            ['nvpmodel', '-q'], capture_output=True, text=True, timeout=2
        )
        text = (out.stdout or '') + (out.stderr or '')
        if 'MAXN' not in text.upper():
            logger.warn(
                "Jetson not in MAXN power mode. For best performance: "
                "`sudo nvpmodel -m 0`"
            )
        else:
            logger.info("Jetson power mode: MAXN")
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        logger.debug("nvpmodel not available; skipping power-mode check")

    # jetson_clocks lock
    try:
        out = subprocess.run(
            ['jetson_clocks', '--show'], capture_output=True, text=True, timeout=2
        )
        # If clocks aren't locked, governor is "schedutil" or similar.
        # The presence of `cur=` matching `max=` is the giveaway, but the
        # easiest practical check is just to remind the user.
        if out.returncode == 0:
            logger.info(
                "jetson_clocks accessible. If clocks are not locked, run: "
                "`sudo jetson_clocks` to pin to max."
            )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        logger.debug("jetson_clocks not available; skipping clocks check")

def cleanup_gpu():
    if TORCH_AVAILABLE:
        with suppress(Exception):
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
