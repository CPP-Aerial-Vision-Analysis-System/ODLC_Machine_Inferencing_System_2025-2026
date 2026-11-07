"""Compare C++ test images with the original Python implementation, pixel for pixel.

Usage: python3 compare_python.py /path/to/annotation.py /path/to/test-output
Only test tooling uses Python; the annotation library has no Python dependency.
"""
import importlib.util
from pathlib import Path
import sys
import types

import cv2
import numpy as np


def main():
    original, output = Path(sys.argv[1]), Path(sys.argv[2])
    # annotation.py only needs this constant. Avoid importing the inference stack.
    module = types.ModuleType("detection.model_manager")
    module.MODEL_FORMAT_TENSORRT = "tensorrt"
    sys.modules["detection.model_manager"] = module
    spec = importlib.util.spec_from_file_location("original_annotation", original)
    annotation = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(annotation)

    source = cv2.imread(str(output / "source.png"))
    comparisons = 0
    for case in sorted(output.glob("case_*")):
        print(f"Comparing {case.name}", flush=True)
        lines = (case / "detections.txt").read_text().splitlines()
        detections = []
        for line in lines[1:]:
            cls, confidence, *bbox = line.split()
            detections.append(dict(
                **{"class": cls}, confidence=float(confidence), bbox=list(map(int, bbox))))
        annotated = annotation.annotate_frame(source, detections, 1.25, 640, 512, lines[0])
        expected_dir = case / "python"
        expected_dir.mkdir(exist_ok=True)
        cv2.imwrite(str(expected_dir / "annotated.png"), annotated)
        actual = cv2.imread(str(case / "annotated.png"))
        assert np.array_equal(actual, annotated), f"{case.name}: annotation pixels differ"
        comparisons += 1
        crop = annotation.save_top_matches_crop(source, detections, "input/photo.png", str(expected_dir))
        assert (crop is not None) == (case / "TM_photo.png").exists(), case.name
        if crop:
            assert np.array_equal(cv2.imread(crop), cv2.imread(str(case / "TM_photo.png"))), (
                f"{case.name}: crop pixels differ")
            comparisons += 1
    print(f"PASS: {comparisons} C++ outputs match original Python pixel for pixel")


if __name__ == "__main__":
    main()
