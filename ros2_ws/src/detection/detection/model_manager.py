import os
from typing import Optional, Tuple

from detection.gpu_utils import suppress_native_output, cleanup_gpu, TORCH_AVAILABLE
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


def get_ros2_ws_directory() -> str:
    current_file = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file)
    
    # Navigate up to find ros2_ws (look for install/ or src/ directories)
    search_dir = current_dir
    ros2_ws_dir = None
    
    for _ in range(10):  # Limit search depth
        if os.path.exists(os.path.join(search_dir, "install")) or os.path.exists(os.path.join(search_dir, "src")):
            if os.path.exists(os.path.join(search_dir, "install")) and os.path.exists(os.path.join(search_dir, "src")):
                ros2_ws_dir = search_dir
                break
            parent = os.path.dirname(search_dir)
            if os.path.exists(os.path.join(parent, "install")) and os.path.exists(os.path.join(parent, "src")):
                ros2_ws_dir = parent
                break
        search_dir = os.path.dirname(search_dir)
        if search_dir == "/":
            break

    
    if ros2_ws_dir and os.path.exists(os.path.join(ros2_ws_dir, "src")):
        ros2_ws_dir = os.path.join(ros2_ws_dir, "src")
        
    # Fallback: construct path directly
    if ros2_ws_dir is None:
        ros2_ws_dir = "/astra/ros2_ws/src"
    
    video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
    os.makedirs(video_cam_dir, exist_ok=True)
    
    return ros2_ws_dir

def resolve_model_path(model_path, model_format, auto_convert, slice_height,
                       slice_width, tensorrt_workspace, device, logger):
    """Resolve model path and determine final format. Returns (path, format) or (None, None)."""
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
                logger.warn(
                    f"Model file not found at {model_path}; "
                    "will download from Ultralytics if needed"
                )

    detected_fmt = _detect_format(model_path)

    if model_format == MODEL_FORMAT_AUTO:
        base = os.path.splitext(model_path)[0]
        engine_path = f"{base}.engine"
        if os.path.exists(engine_path):
            logger.info(f"Found TensorRT engine: {engine_path}")
            return engine_path, MODEL_FORMAT_TENSORRT
        if (
            model_path.endswith('.pt')
            and auto_convert
            and TENSORRT_AVAILABLE
        ):
            if _convert_pt_to_trt(model_path, engine_path, slice_height, slice_width, tensorrt_workspace, device, logger):
                return engine_path, MODEL_FORMAT_TENSORRT
            logger.warn("TensorRT conversion failed, using PyTorch")
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

def load_sahi_model(resolved_path, final_format, confidence_threshold, device, logger):
    """Load a SAHI-wrapped detection model. Returns model or None."""
    if not SAHI_AVAILABLE:
        logger.error("SAHI not installed: pip install sahi")
        return None
    if not YOLO_AVAILABLE:
        logger.error("Ultralytics not installed: pip install ultralytics(make sure to get the correct versions if on Jetson)")
        return None

    # Pre-load GPU optimisation
    if device.startswith('cuda') and TORCH_AVAILABLE:
        try:
            _setup_gpu_optimizations()
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

def _setup_gpu_optimizations():
    """Pre-configure GPU memory and CUDA settings for optimal performance."""
    import gc
    gc.collect()
    torch.cuda.empty_cache()
    torch.cuda.synchronize()
    torch.backends.cudnn.benchmark = True
    torch.backends.cudnn.enabled = True

def warmup_model(model, slice_h, slice_w, overlap_h, overlap_w, logger):
    """Run a dummy inference to warm up the model."""
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
