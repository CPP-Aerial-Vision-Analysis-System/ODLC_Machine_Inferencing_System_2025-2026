import os
import sys
import gc
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
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read()
                if 'jetson' in model.lower():
                    logger.info(f"Platform: NVIDIA Jetson ({model.strip()})")
        except (OSError, IOError, FileNotFoundError):
            pass
        return "cuda:0"

    logger.warn("No GPU detected, using CPU")
    try:
        with open('/proc/device-tree/model', 'r') as f:
            if 'jetson' in f.read().lower():
                logger.error("Jetson detected but GPU not available! Check CUDA install.")
    except (OSError, IOError, FileNotFoundError):
        pass
    return "cpu"


def optimize_gpu_memory(device, slice_height, slice_width, overlap_h, overlap_w, logger):
    """Tune GPU memory for SAHI on Jetson. May adjust slice/overlap params."""
    if not device.startswith('cuda') or not TORCH_AVAILABLE:
        return slice_height, slice_width, overlap_h, overlap_w

    try:
        total_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3

        if total_memory < 4:  # Jetson Nano
            recommended_slice = 640
            recommended_overlap = 0.10
            max_fraction = 0.7
            logger.warn("Jetson Nano detected: large slices + low overlap for stability")
        elif total_memory < 8:  # Jetson Xavier NX / TX2
            recommended_slice = 512
            recommended_overlap = 0.15
            max_fraction = 0.75
            logger.info("Jetson Xavier NX/TX2 detected: balanced settings")
        else:  # Jetson AGX / Orin
            recommended_slice = 416
            recommended_overlap = 0.20
            max_fraction = 0.8
            logger.info("Jetson AGX/Orin detected: moderate slices")

        # Estimate slice counts for a worst-case 4K image
        current_slices = _estimate_slice_count(
            3840, 2160, slice_width, slice_height, overlap_w, overlap_h
        )
        recommended_slices = _estimate_slice_count(
            3840, 2160, recommended_slice, recommended_slice,
            recommended_overlap, recommended_overlap
        )

        if current_slices > 100:
            logger.error(
                f"Current settings generate ~{current_slices} slices for 4K — "
                f"will cause OOM. Forcing safe settings (~{recommended_slices} slices)."
            )
            slice_height = recommended_slice
            slice_width = recommended_slice
            overlap_h = recommended_overlap
            overlap_w = recommended_overlap
        elif current_slices > 60:
            logger.warn(
                f"Current settings generate ~{current_slices} slices. "
                f"Consider slice_size={recommended_slice}, overlap={recommended_overlap:.2f}"
            )

        torch.cuda.set_per_process_memory_fraction(max_fraction, 0)

        split_size = 256 if recommended_slice >= 512 else 128
        os.environ['PYTORCH_CUDA_ALLOC_CONF'] = (
            f'max_split_size_mb:{split_size},expandable_segments:True'
        )

        logger.info(
            f"GPU: {total_memory:.1f}GB total, {max_fraction*100:.0f}% max, "
            f"split={split_size}MB, slices={slice_height}x{slice_width}, "
            f"overlap={overlap_h:.0%}"
        )

    except Exception as e:
        logger.warn(f"GPU memory optimization failed: {e}")

    return slice_height, slice_width, overlap_h, overlap_w


def cleanup_gpu():
    if TORCH_AVAILABLE:
        try:
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:
            pass


def _estimate_slice_count(img_w, img_h, slice_w, slice_h, overlap_w, overlap_h):
    import math
    stride_w = int(slice_w * (1 - overlap_w))
    stride_h = int(slice_h * (1 - overlap_h))
    if stride_w <= 0 or stride_h <= 0:
        return 999999
    slices_w = max(1, math.ceil((img_w - slice_w) / stride_w) + 1)
    slices_h = max(1, math.ceil((img_h - slice_h) / stride_h) + 1)
    return slices_w * slices_h