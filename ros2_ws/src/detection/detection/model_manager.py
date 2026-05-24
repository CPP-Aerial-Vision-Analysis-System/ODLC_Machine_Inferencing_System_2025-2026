import os
import rclpy.logging
from typing import Optional, Tuple

from detection.gpu_utils import suppress_native_output, cleanup_gpu, TORCH_AVAILABLE
from video_cam.storage_manager import get_ros2_ws_directory
# Optional deps
try:
    from sahi import AutoDetectionModel
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import tensorrt as trt
    TENSORRT_AVAILABLE = True
    _trt_logger = trt.Logger(trt.Logger.WARNING)
except ImportError:
    TENSORRT_AVAILABLE = False

if TORCH_AVAILABLE:
    import torch

MODEL_FORMAT_PYTORCH = 'pytorch'
MODEL_FORMAT_TENSORRT = 'tensorrt'
MODEL_FORMAT_AUTO = 'auto'
_MAX_SEARCH_DEPTH = 10


def resolve_model_path(model_path, model_format, auto_convert, slice_height,
                       slice_width, tensorrt_workspace, device, logger=None):
    """Resolve model path and determine final format. Returns (path, format) or (None, None)."""
    logger = logger or rclpy.logging.get_logger('model_manager')
    ros2_ws = get_ros2_ws_directory()

    # Resolve relative paths
    if not os.path.isabs(model_path):
        candidates = [
            os.path.join(ros2_ws, model_path),
            os.path.join(os.path.dirname(__file__), model_path),
        ]
        for c in candidates:
            if os.path.exists(c):
                model_path = c
                break
        else:
            if not os.path.exists(model_path):
                logger.warn(f"Model file not found at {model_path}; "
                "Will download from Ultralytics if needed"
                )

    detected_fmt = _detect_format(model_path)

    if model_format == MODEL_FORMAT_AUTO:
        base = os.path.splitext(model_path)[0]
        engine_path = f"{base}.engine"
        pt_path = f"{base}.pt"

        # Prefer existing engine
        if os.path.exists(engine_path):
            logger.info(f"Found TensorRT engine: {engine_path}")
            return engine_path, MODEL_FORMAT_TENSORRT

        # No engine yet — try to build from .pt (whether the user asked for
        # .engine or .pt; the goal is the engine).
        if os.path.exists(pt_path) and auto_convert and TENSORRT_AVAILABLE:
            if _convert_pt_to_trt(pt_path, engine_path, slice_height, slice_width, tensorrt_workspace, device, logger):
                return engine_path, MODEL_FORMAT_TENSORRT
            logger.warn("TensorRT conversion failed, falling back to PyTorch")
            return pt_path, MODEL_FORMAT_PYTORCH

        # No engine, no .pt to convert — let SAHI/Ultralytics try whatever
        # was requested (e.g. download .pt).
        if model_path.endswith('.engine') and not os.path.exists(model_path):
            logger.warn(
                f"Engine {engine_path} missing and {pt_path} not found; "
                "falling back to PyTorch by name"
            )
            return pt_path, MODEL_FORMAT_PYTORCH
        return model_path, detected_fmt

    if model_format == MODEL_FORMAT_TENSORRT:
        if model_path.endswith('.engine') and os.path.exists(model_path):
            return model_path, MODEL_FORMAT_TENSORRT
        base = os.path.splitext(model_path)[0]
        engine_path = f"{base}.engine"
        if os.path.exists(engine_path):
            return engine_path, MODEL_FORMAT_TENSORRT
        if model_path.endswith('.pt') and os.path.exists(model_path) and _convert_pt_to_trt(model_path, engine_path, slice_height, slice_width, tensorrt_workspace, device, logger):
                return engine_path, MODEL_FORMAT_TENSORRT
        logger.error("TensorRT model required but not available")
        return None, None

    # PyTorch explicit
    return model_path, MODEL_FORMAT_PYTORCH

def load_sahi_model(resolved_path, final_format, confidence_threshold, device, logger=None):
    """Load a SAHI-wrapped detection model. Returns model or None."""
    logger = logger or rclpy.logging.get_logger('model_manager')
    if not SAHI_AVAILABLE:
        logger.error("SAHI not installed: pip install sahi")
        return None
    if not YOLO_AVAILABLE:
        logger.error("Ultralytics not installed: pip install ultralytics(make sure to get the correct versions if on Jetson)")
        return None

    # Pre-load GPU optimisation
    if device.startswith('cuda') and TORCH_AVAILABLE:
        try:
            import gc
            gc.collect()
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
            torch.backends.cudnn.benchmark = True
            torch.backends.cudnn.enabled = True
            # FP32 matmul → TF32 on Ampere (Orin). Free precision/perf trade.
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
            try:
                torch.set_float32_matmul_precision('high')
            except AttributeError:
                pass  # older torch versions
        except Exception as e:
            logger.debug(f"GPU pre-load setup: {e}")

    try:
        if final_format == MODEL_FORMAT_TENSORRT:
            logger.info(f"Loading TensorRT engine: {resolved_path}")
            with suppress_native_output():
                yolo_model = YOLO(resolved_path, task='detect')
            detection_model = AutoDetectionModel.from_pretrained(
                model_type='yolov8',
                model_path=resolved_path,
                model=yolo_model,
                confidence_threshold=confidence_threshold,
                device=device,
            )
            logger.info("TensorRT model loaded and wrapped for SAHI")
        else:
            detection_model = AutoDetectionModel.from_pretrained(
                model_type='yolov8',
                model_path=resolved_path,
                confidence_threshold=confidence_threshold,
                device=device,
            )
            logger.info("PyTorch model loaded via SAHI")
        return detection_model

    except Exception as e:
        logger.error(f"Failed to load model ({final_format}): {e}")

        # TensorRT fallback to PyTorch
        if final_format == MODEL_FORMAT_TENSORRT:
            pt_path = f'{os.path.splitext(resolved_path)[0]}.pt'
            if os.path.exists(pt_path):
                logger.warn(f"Falling back to PyTorch: {pt_path}")
                try:
                    m = AutoDetectionModel.from_pretrained(
                        model_type='yolov8',
                        model_path=pt_path,
                        confidence_threshold=confidence_threshold,
                        device=device,
                    )
                    logger.info("PyTorch fallback model loaded")
                    return m
                except Exception as e2:
                    logger.error(f"PyTorch fallback also failed: {e2}")
        return None

def warmup_model(model, slice_h, slice_w, overlap_h, overlap_w, logger=None):
    """Run a dummy inference to warm up the model."""
    logger = logger or rclpy.logging.get_logger('model_manager')
    if model is None:
        return
    logger.info("Warming up model...")
    try:
        import numpy as np
        from sahi.predict import get_sliced_prediction
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        with suppress_native_output():
            get_sliced_prediction(
                dummy, model,
                slice_height=slice_h, slice_width=slice_w,
                overlap_height_ratio=overlap_h, overlap_width_ratio=overlap_w,
                verbose=0,
            )
        logger.info("Model warmup complete")
    except Exception as e:
        logger.warn(f"Model warmup failed: {e}")

def _detect_format(path: str) -> str:
    if path.endswith('.engine'):
        return MODEL_FORMAT_TENSORRT
    return MODEL_FORMAT_PYTORCH

def _convert_pt_to_trt(pt_path, engine_path, slice_h, slice_w, workspace_gb, device, logger):
    """Convert .pt model to TensorRT .engine."""
    if not YOLO_AVAILABLE or not TENSORRT_AVAILABLE:
        logger.error("YOLO and TensorRT are both required for conversion")
        return False

    try:
        logger.info(f"Converting {pt_path} → TensorRT (may take several minutes)...")
        cleanup_gpu()

        model = YOLO(pt_path)
        logger.info(
            f"TensorRT export: imgsz=({slice_h},{slice_w}), workspace={workspace_gb}GB"
        )

        with suppress_native_output():
            model.export(
                format='engine',
                device=0 if device.startswith('cuda') else 'cpu',
                imgsz=(slice_h, slice_w),
                half=True,
                workspace=workspace_gb,
                simplify=True,
                verbose=False,
            )

        base = os.path.splitext(pt_path)[0]
        exported = f"{base}.engine"
        if os.path.exists(exported):
            if exported != engine_path:
                import shutil
                shutil.move(exported, engine_path)
            size_mb = os.path.getsize(engine_path) / (1024 ** 2)
            logger.info(f"TensorRT conversion done: {engine_path} ({size_mb:.1f}MB)")
            return True
        logger.error(f"Engine file not found at {exported}")
        return False
    except Exception as e:
        logger.error(f"TensorRT conversion failed: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return False

