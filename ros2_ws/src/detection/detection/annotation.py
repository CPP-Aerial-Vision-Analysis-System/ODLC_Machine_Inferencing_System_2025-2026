import os
import cv2
import numpy as np
from typing import List, Dict, Optional

from detection.model_manager import MODEL_FORMAT_TENSORRT

# BGR colours
COLOR_TENT = (0, 200, 255)     # orange-yellow
COLOR_PERSON = (0, 200, 0)     # green
COLOR_OBJECT = (255, 100, 0)   # blue
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

_FONT = cv2.FONT_HERSHEY_DUPLEX
_FONT_SCALE = 0.45
_FONT_THICK = 1

# will need to comment this out as well, once i figure out how to send confidence by itself
def annotate_frame(frame, detections, processing_time, slice_height, slice_width, model_format):
    """Draw bounding boxes, labels, and header onto frame. Returns annotated copy."""
    annotated = frame.copy()
    height, width = frame.shape[:2]

    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        cls = det['class']
        conf = det['confidence']

        if cls == 'person':
            color = COLOR_PERSON
            label = f"PERSON ({conf:.0%})"
            align_right = True
        elif cls == 'tent':
            color = COLOR_TENT
            label = f"TENT ({conf:.0%})"
            align_right = False
        else:
            # Non-target: thin blue box, no label
            cv2.rectangle(annotated, (x1, y1), (x2, y2), COLOR_OBJECT, 1)
            continue

        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 1)

        (tw, th), _ = cv2.getTextSize(label, _FONT, _FONT_SCALE, _FONT_THICK)
        pad = 4
        lh = th + pad * 2

        # Vertical position
        if y1 - lh >= 0:
            ly1, ly2 = y1 - lh, y1
        else:
            ly1, ly2 = y1, y1 + lh

        # Horizontal position
        if align_right:
            lx1 = max(0, x2 - tw - pad * 2)
            lx2 = x2
        else:
            lx1 = x1
            lx2 = x1 + tw + pad * 2

        cv2.rectangle(annotated, (lx1, ly1), (lx2, ly2), color, -1)
        tx, ty = lx1 + pad, ly2 - pad
        cv2.putText(annotated, label, (tx, ty), _FONT, _FONT_SCALE, COLOR_BLACK, _FONT_THICK + 1, cv2.LINE_AA)
        cv2.putText(annotated, label, (tx, ty), _FONT, _FONT_SCALE, COLOR_WHITE, _FONT_THICK, cv2.LINE_AA)

    # Header
    n_people = sum(1 for d in detections if d['class'] == 'person')
    n_tents = sum(1 for d in detections if d['class'] == 'tent')
    n_other = len(detections) - n_people - n_tents
    engine = 'TensorRT' if model_format == MODEL_FORMAT_TENSORRT else 'PyTorch'

    header = f"SAHI+YOLO ({engine}) | {n_people} people, {n_tents} tents, {n_other} other"
    subline = f"Time: {processing_time:.1f}s | Slices: {slice_height}x{slice_width}"

    overlay = annotated.copy()
    cv2.rectangle(overlay, (0, 0), (width, 50), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated)

    hfs = 0.6
    cv2.putText(annotated, header, (10, 20), _FONT, hfs, COLOR_WHITE, 1, cv2.LINE_AA)
    cv2.putText(annotated, subline, (10, 42), _FONT, hfs * 0.8, (200, 200, 200), 1, cv2.LINE_AA)

    return annotated


# def save_top_matches_crop(frame, detections, image_path, output_dir):
#     """Crop best person and tent, combine side-by-side, save as TM_<name>."""
#     persons = [d for d in detections if d['class'] == 'person']
#     tents = [d for d in detections if d['class'] == 'tent']
#     best_person = max(persons, key=lambda x: x['confidence']) if persons else None
#     best_tent = max(tents, key=lambda x: x['confidence']) if tents else None

#     if not best_person and not best_tent:
#         return None

#     h, w = frame.shape[:2]
#     crops, labels = [], []

#     for best, tag in [(best_person, 'PERSON'), (best_tent, 'TENT')]:
#         if best is None:
#             continue
#         x1, y1, x2, y2 = best['bbox']
#         x1, y1 = max(0, x1), max(0, y1)
#         x2, y2 = min(w, x2), min(h, y2)
#         if x2 > x1 and y2 > y1:
#             crops.append(frame[y1:y2, x1:x2].copy())
#             labels.append(f"{tag} {best['confidence']:.0%}")

#     if not crops:
#         return None

#     target_h = 200
#     resized = []
#     for crop, label in zip(crops, labels):
#         ch, cw = crop.shape[:2]
#         if ch == 0:
#             continue
#         scale = target_h / ch
#         nw = int(cw * scale)
#         r = cv2.resize(crop, (nw, target_h), interpolation=cv2.INTER_AREA)

#         (tw_, th_), _ = cv2.getTextSize(label, _FONT, 0.5, 1)
#         lbl_h = th_ + 10
#         padded = np.zeros((target_h + lbl_h, nw, 3), dtype=np.uint8)
#         padded[:target_h, :] = r
#         cv2.rectangle(padded, (0, target_h), (nw, target_h + lbl_h), (40, 40, 40), -1)
#         tx_ = (nw - tw_) // 2
#         cv2.putText(padded, label, (tx_, target_h + th_ + 3), _FONT, 0.5, COLOR_WHITE, 1, cv2.LINE_AA)
#         resized.append(padded)

#     if not resized:
#         return None

#     sep = 5
#     total_w = sum(c.shape[1] for c in resized) + sep * (len(resized) - 1)
#     combined = np.zeros((resized[0].shape[0], total_w, 3), dtype=np.uint8)
#     xoff = 0
#     for i, c in enumerate(resized):
#         if i > 0:
#             combined[:, xoff:xoff + sep] = (80, 80, 80)
#             xoff += sep
#         combined[:, xoff:xoff + c.shape[1]] = c
#         xoff += c.shape[1]

#     name, ext = os.path.splitext(os.path.basename(image_path))
#     out_path = os.path.join(output_dir, f"TM_{name}{ext}")
#     cv2.imwrite(out_path, combined)
#     return out_path
