"""Manual slicer + batched inference path. Drop-in alternative to run_sahi_detection.

Mirrors SAHI's behavior:
- Step = slice * (1 - overlap), last row/col clamped to frame edge for full coverage.
- Per-slice NMS comes from ultralytics' predict().
- Cross-slice merge uses Intersection-over-Smaller (IoS) at 0.5, class-aware
  — matches SAHI's postprocess_type="NMS", match_metric="IOS",
  match_threshold=0.5, class_agnostic=False.

Returns the same list-of-dict shape as run_sahi_detection so the rest of the
pipeline is unchanged.
"""

from typing import List, Dict, Tuple

import cv2
import numpy as np

from detection.gpu_utils import TORCH_AVAILABLE, cleanup_gpu
from detection.model_manager import MODEL_FORMAT_TENSORRT
from detection.detection_processor import _categorize


def _generate_slices(frame_h: int, frame_w: int,
                     slice_h: int, slice_w: int,
                     overlap_h: float, overlap_w: float
                     ) -> List[Tuple[int, int, int, int]]:
    """Tile the frame into (x, y, w, h) rects covering everything."""
    step_h = max(1, int(slice_h * (1.0 - overlap_h)))
    step_w = max(1, int(slice_w * (1.0 - overlap_w)))

    if frame_h <= slice_h:
        ys = [0]
    else:
        ys = list(range(0, frame_h - slice_h, step_h))
        if not ys or ys[-1] != frame_h - slice_h:
            ys.append(frame_h - slice_h)

    if frame_w <= slice_w:
        xs = [0]
    else:
        xs = list(range(0, frame_w - slice_w, step_w))
        if not xs or xs[-1] != frame_w - slice_w:
            xs.append(frame_w - slice_w)

    ys = sorted(set(ys))
    xs = sorted(set(xs))

    out = []
    for y in ys:
        for x in xs:
            w = min(slice_w, frame_w - x)
            h = min(slice_h, frame_h - y)
            out.append((x, y, w, h))
    return out


def _extract_yolo(model):
    """Return the underlying ultralytics YOLO from a SAHI wrapper (or pass through)."""
    inner = getattr(model, 'model', None)
    if inner is not None and hasattr(inner, 'predict'):
        return inner
    if hasattr(model, 'load_model'):
        try:
            model.load_model()
        except Exception:
            pass
        inner = getattr(model, 'model', None)
        if inner is not None and hasattr(inner, 'predict'):
            return inner
    return model


def _ios_nms(raw: List[Dict], threshold: float = 0.5) -> List[Dict]:
    """Greedy class-aware NMS using Intersection-over-Smaller."""
    if not raw:
        return []
    dets = sorted(raw, key=lambda d: d['conf'], reverse=True)
    suppressed = [False] * len(dets)
    keep: List[Dict] = []
    for i in range(len(dets)):
        if suppressed[i]:
            continue
        keep.append(dets[i])
        xi1, yi1, xi2, yi2 = dets[i]['bbox']
        ai = max(0, (xi2 - xi1)) * max(0, (yi2 - yi1))
        ci = dets[i]['class_name']
        for j in range(i + 1, len(dets)):
            if suppressed[j] or dets[j]['class_name'] != ci:
                continue
            xj1, yj1, xj2, yj2 = dets[j]['bbox']
            aj = max(0, (xj2 - xj1)) * max(0, (yj2 - yj1))
            iw = max(0, min(xi2, xj2) - max(xi1, xj1))
            ih = max(0, min(yi2, yj2) - max(yi1, yj1))
            inter = iw * ih
            smaller = min(ai, aj)
            if smaller > 0 and (inter / smaller) >= threshold:
                suppressed[j] = True
    return keep


def run_batched_detection(frame, model, slice_height, slice_width, overlap_h, overlap_w,
                          confidence_threshold, min_area, max_area, min_aspect, max_aspect,
                          model_format, device, enable_gpu_cleanup, images_processed,
                          logger, batch_size: int = 8):
    """Manual-slice + batched inference. Same signature/return as run_sahi_detection."""
    if model is None:
        return []

    yolo = _extract_yolo(model)

    # Match SAHI: feed RGB to the model.
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w = frame_rgb.shape[:2]

    slices = _generate_slices(h, w, slice_height, slice_width, overlap_h, overlap_w)
    if not slices:
        return []

    # Pad partial edge crops to (slice_h, slice_w) so a batch tensor is uniform.
    slice_imgs: List[np.ndarray] = []
    for (x, y, sw, sh) in slices:
        crop = frame_rgb[y:y + sh, x:x + sw]
        if sh != slice_height or sw != slice_width:
            padded = np.zeros((slice_height, slice_width, 3), dtype=crop.dtype)
            padded[:sh, :sw] = crop
            crop = padded
        slice_imgs.append(crop)

    method = ('manual-batched+yolo+tensorrt'
              if model_format == MODEL_FORMAT_TENSORRT
              else 'manual-batched+yolo')

    # Ultralytics doesn't accept 'auto'; pass None to let it choose.
    yolo_device = None if (device in (None, '', 'auto')) else device

    raw: List[Dict] = []

    for start in range(0, len(slice_imgs), batch_size):
        batch = slice_imgs[start:start + batch_size]
        meta = slices[start:start + batch_size]
        try:
            results = yolo.predict(
                batch,
                imgsz=slice_height,
                conf=confidence_threshold,
                verbose=False,
                device=yolo_device,
            )
        except Exception as e:
            logger.error(f"Batched predict failed (slices {start}..{start + len(batch)}): {e}")
            continue

        for res, (sx, sy, sw, sh) in zip(results, meta):
            boxes = getattr(res, 'boxes', None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            cls_ids = boxes.cls.cpu().numpy().astype(int)
            names = getattr(res, 'names', None) or getattr(yolo, 'names', {})

            for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, cls_ids):
                # Drop boxes entirely in the padded region.
                if x1 >= sw or y1 >= sh:
                    continue
                # Clip to real slice content before translating to global coords.
                x2 = min(float(x2), float(sw))
                y2 = min(float(y2), float(sh))
                gx1 = int(x1 + sx)
                gy1 = int(y1 + sy)
                gx2 = int(x2 + sx)
                gy2 = int(y2 + sy)

                if isinstance(names, dict):
                    class_name = names.get(int(cls_id), str(cls_id))
                else:
                    class_name = (names[int(cls_id)]
                                  if int(cls_id) < len(names) else str(cls_id))

                raw.append({
                    'bbox': [gx1, gy1, gx2, gy2],
                    'conf': float(conf),
                    'class_name': class_name,
                })

    # Cross-slice merge BEFORE area/aspect filtering to match SAHI's order
    # (SAHI returns merged ObjectPredictions, then we categorize/filter).
    merged = _ios_nms(raw, threshold=0.5)

    detections: List[Dict] = []
    for r in merged:
        det = _categorize(
            r['class_name'], r['conf'], r['bbox'],
            confidence_threshold, min_area, max_area, min_aspect, max_aspect,
            method,
        )
        if det is not None:
            detections.append(det)

    if (enable_gpu_cleanup and device.startswith('cuda')
            and TORCH_AVAILABLE and images_processed % 10 == 0):
        cleanup_gpu()

    return detections
