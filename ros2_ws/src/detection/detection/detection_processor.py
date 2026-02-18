import cv2
import numpy as np
from typing import List, Dict, Optional

from detection.gpu_utils import TORCH_AVAILABLE, cleanup_gpu
from detection.model_manager import MODEL_FORMAT_TENSORRT

if TORCH_AVAILABLE:
    import torch

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

# Direct YOLO classes that map to "person"
PERSON_CLASSES = frozenset({'person'})

# Classes that legitimately look like people from altitude (mannequins)
PERSON_LIKE_CLASSES = frozenset({'doll', 'teddy bear'})

# Classes that look like tents/tarps/shelters from aerial view.
# NOTE: 'car', 'truck', 'bus', 'airplane', 'bench', 'surfboard', 'frisbee'
# were previously included but are NOT tents. Including them causes dangerous
# false positives where the drone flies to a parked car thinking it's a tent.
TENT_LIKE_CLASSES = frozenset({
    'umbrella',   # umbrella canopy from above ≈ tent top
    'kite',       # flat fabric object from above
})


def run_sahi_detection(frame, model, slice_height, slice_width, overlap_h, overlap_w,
                      confidence_threshold, min_area, max_area, min_aspect, max_aspect,
                      model_format, device, enable_gpu_cleanup, images_processed, logger):
    """Run SAHI sliced prediction and return categorised detections."""
    if not SAHI_AVAILABLE or model is None:
        return []

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    result = get_sliced_prediction(
        frame_rgb,
        model,
        slice_height=slice_height,
        slice_width=slice_width,
        overlap_height_ratio=overlap_h,
        overlap_width_ratio=overlap_w,
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
    if enable_gpu_cleanup and device.startswith('cuda') and TORCH_AVAILABLE:
        if images_processed % 10 == 0:
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

    if area < min_area or area > max_area:
        return None
    if aspect < min_aspect or aspect > max_aspect:
        return None
    if confidence < conf_thresh:
        return None

    if class_name in PERSON_CLASSES:
        cat = 'person'
        desc = 'person'
        target = True
    elif class_name in PERSON_LIKE_CLASSES:
        cat = 'person'
        desc = f'person-like ({class_name})'
        target = True
    elif class_name in TENT_LIKE_CLASSES:
        cat = 'tent'
        desc = f'tent-like ({class_name})'
        target = True
    else:
        cat = 'object'
        desc = f'detected: {class_name}'
        target = False

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
