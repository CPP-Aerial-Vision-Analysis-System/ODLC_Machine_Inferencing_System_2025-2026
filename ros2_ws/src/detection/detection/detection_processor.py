import cv2
from typing import List, Dict

from detection.gpu_utils import TORCH_AVAILABLE, cleanup_gpu
from detection.model_manager import MODEL_FORMAT_TENSORRT

# Optional deps
try:
    from sahi.predict import get_sliced_prediction
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False

# ── Classification mappings ──────────────────────────────────────────────────
#
# KEEP THESE TIGHT. False positives here mean the drone flies to the wrong
# location. Only add classes you have empirically validated from aerial views.

CLASS_MAP = {
    'person': ('person', 'person', True),
    'mannequin': ('person', 'person', True),
    'doll': ('person', 'person-like (doll)', True),

    'umbrella': ('tent', 'tent-like (umbrella)', True),
    'kite': ('tent', 'tent-like (kite)', True),
}


def run_sahi_detection(frame, model, slice_height, slice_width, overlap_h, overlap_w,
                      confidence_threshold, min_area, max_area, min_aspect, max_aspect,
                      model_format, device, enable_gpu_cleanup, images_processed, logger):
    """Run SAHI sliced prediction and return categorised detections."""
    if not SAHI_AVAILABLE or model is None:
        return []

    # SAHI requires RGB. cv2.cvtColor uses SIMD and runs ~10-20ms on a 4K
    # frame — the cheapest contiguous BGR→RGB path on Jetson.
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    result = get_sliced_prediction(
        frame_rgb,
        model,
        slice_height=slice_height,
        slice_width=slice_width,
        overlap_height_ratio=overlap_h,
        overlap_width_ratio=overlap_w,
        # Skip the redundant full-image inference pass — for high-res aerial
        # imagery the small targets don't survive the resize to 640×640
        # anyway, so the slice pass already covers them.
        perform_standard_pred=False,
        # Use the slice size we passed; don't let SAHI override it.
        auto_slice_resolution=False,
        postprocess_type="NMS",
        postprocess_match_metric="IOS",
        postprocess_match_threshold=0.5,
        postprocess_class_agnostic=False,
        verbose=0,
    )

    method = (
        'sahi+yolo+tensorrt' if model_format == MODEL_FORMAT_TENSORRT
        else 'sahi+yolo'
    )

    detections: List[Dict] = []
    for pred in result.object_prediction_list:
        bbox = pred.bbox
        x1, y1, x2, y2 = int(bbox.minx), int(bbox.miny), int(bbox.maxx), int(bbox.maxy)
        class_name = pred.category.name
        confidence = float(pred.score.value)

        det = _categorize(
            class_name, confidence, [x1, y1, x2, y2],
            confidence_threshold, min_area, max_area, min_aspect, max_aspect,
            method,
        )
        if det is not None:
            detections.append(det)

    # Periodic GPU cache cleanup (every 10 images)
    if enable_gpu_cleanup and device.startswith('cuda') and TORCH_AVAILABLE and images_processed % 10 == 0:
        cleanup_gpu()

    return detections


def _categorize(class_name, confidence, bbox, conf_thresh, min_area, max_area,
                min_aspect, max_aspect, method):
    """Classify a YOLO detection into person / tent / object. Returns None to discard."""
    x1, y1, x2, y2 = bbox
    w = x2 - x1
    h = y2 - y1
    area = w * h
    aspect = w / h if h > 0 else 0.0

    if (area < min_area or area > max_area or aspect < min_aspect 
        or confidence < conf_thresh 
        or aspect > max_aspect):
        return None

    cat, desc, target = CLASS_MAP.get(
        class_name,
        ('object', f'detected: {class_name}', False)
    )

    return {
        'class': cat,
        'yolo_class': class_name,
        'confidence': confidence,
        'bbox': bbox,
        'description': desc,
        'method': method,
        'area': area,
        'is_target': target,
    }
