## SAHI + YOLO Object Detection — Design & Implementation Notes

This document explains how `object_detection_sahi.py` works, the problem it solved, important functions and components, Jetson-specific optimizations used to achieve low latency, and guidance for anyone reimplementing or extending the algorithm.

File: `ros2_ws/src/video_cam/video_cam/object_detection_sahi.py`

---

## Purpose and high-level summary

This node implements a small-object-aware object detection pipeline for aerial imagery by combining SAHI (Slicing Aided Hyper Inference) with an Ultralytics YOLO model (referred to in the code as `yolo11s.pt`). The main goals:

- Detect small, sparse targets (people and tents) in large aerial images.
- Improve detection recall for small objects by slicing images and merging predictions.
- Run on embedded devices (NVIDIA Jetson) with minimal latency using GPU acceleration when available.
- Provide ROS2 integration: publishes annotated images and detection metadata.

Key idea: SAHI slices the input image into overlapping windows, runs the detector on each slice, then merges results using NMS/postprocessing. This avoids missing small objects that would otherwise be too small at full-resolution inference.

---

## What problem was faced before this approach

- Small objects in high-resolution aerial images are frequently missed by standard full-image detection because the objects occupy too few pixels. A single forward pass often lacks the resolution to detect small tents or people.
- Processing the entire image at very high resolution is expensive or impossible on embedded hardware.
- Naively slicing without careful overlap or merging leads to duplicate detections or missed boundary objects.

This implementation uses SAHI to slice with overlap (default 512×512 with 30% overlap), runs YOLO per slice, and merges results using NMS and a postprocess match metric (IOS). The node was tuned to favor recall for small objects while keeping latency low on Jetson by using GPU acceleration and memory-aware model loading.

---

## Quick architecture / data flow

1. ROS2 node `SAHIObjectDetectionNode` periodically looks for new images in a monitored folder (`camera_feed`).
2. For each new image, `process_image` loads the frame and calls `detect_objects_sahi`.
3. `detect_objects_sahi` converts BGR→RGB and calls SAHI's `get_sliced_prediction` with configured slice size and overlap.
4. The raw SAHI object predictions are converted into a normalized detection dict via `_categorize_detection`.
5. Detections are filtered and de-duplicated with `_filter_detections` and NMS (`_apply_nms`).
6. `annotate_frame` draws boxes and textual info on the image.
7. Annotated image is published to `/sahi_detection_results` and metadata to `/sahi_detection_info`, and saved to `detection_results_sahi`.

---

## Important functions and components (what to read first)

- SAHIObjectDetectionNode.__init__
  - Declares ROS2 parameters (model_path, thresholds, slice size/overlap, device).
  - Initializes publishers, filesystem paths, device auto-detection, and the SAHI model (`initialize_sahi_model`).

- _get_device(self)
  - Auto-detects compute device priority: CUDA (NVIDIA) → MPS (Apple) → CPU.
  - Contains Jetson-specific checks and helpful logs about missing CUDA-enabled PyTorch.

- initialize_sahi_model(self)
  - Loads the SAHI AutoDetectionModel from the provided `model_path` and places it on the chosen device.
  - Performs some GPU memory checks and cache clearing to reduce OOM on constrained devices.

- check_for_new_images(self)
  - Scans the monitored folder for images and calls `process_image` for unprocessed files.

- process_image(self, image_path)
  - Loads the image, times the detection, collects detections, annotates, publishes, and updates statistics.

- detect_objects_sahi(self, frame)
  - Core SAHI invocation: calls `get_sliced_prediction` with slice size, overlap, and post-processing options.
  - Converts SAHI predictions to the node's detection dict format.

- _categorize_detection(self, class_name, confidence, bbox, frame)
  - Heuristic mapping of YOLO classes into target classes (`person`, `tent`).
  - Contains per-class confidence thresholds and simple geometric checks for tent-like objects.

- _filter_detections / _apply_nms / _calculate_iou
  - Sorts by confidence, applies class-wise NMS using IoU thresholding, removes overlaps and duplicates.

- annotate_frame(self, frame, detections, processing_time)
  - Draws bounding boxes, labels, header, and timing info on the image; returns annotated BGR frame.

- publish_results(self, annotated_frame, detections, image_path)
  - Publishes ROS Image and String messages; writes annotated frames to disk.

- main()
  - ROS2 entrypoint that validates dependencies and spins the node.

Tip: read `detect_objects_sahi` first to understand the inference core, then `initialize_sahi_model` for device/model-loading concerns.

---

## Configuration options (ROS2 params)

- `model_path` — path to the YOLO model file (default `yolo11s.pt`).
- `confidence_threshold` — threshold passed when building AutoDetectionModel.
- `slice_height`, `slice_width` — SAHI slice dimensions (default 512).
- `overlap_height_ratio`, `overlap_width_ratio` — overlap ratios between slices (default 0.3).
- `check_interval` — how often the node checks the `camera_feed` folder (seconds).
- `device` — `'auto'`, `'cuda:0'`, `'mps'`, or `'cpu'`.

You can override these when launching the node via ROS2 `--ros-args -p <param>:=<value>` or by changing defaults in the code.

---

## Jetson (NVIDIA) specific notes and minimal-latency tips

The node was used on NVIDIA Jetson devices and tuned to minimize latency. Key suggestions and rationale:

- Use a PyTorch build that supports CUDA for Jetson (not the CPU-only wheel). On Jetson, PyTorch must be the special JetPack build or wheel compatible with the board's CUDA / cuDNN versions. See Jetson forums and NVIDIA docs for specific wheel URLs.
- Device auto-detection checks `/proc/device-tree/model` to flag Jetson platforms and issues warnings when GPU is present but inaccessible.
- Before loading large models, call `torch.cuda.empty_cache()` to release fragmentation and call `torch.cuda.get_device_properties(0)` to check available memory. If memory is low, the node warns and suggests a smaller model.
- Use smaller, faster models when latency is critical (e.g. YOLOv8n/yolov8s or pruned/quantized variants). The current code is set to use a small/specialized model (`yolo11s.pt`), but you can replace it with even smaller export formats.
- Consider converting the model to TensorRT / ONNX for faster inference on Jetson:
  - Export from Ultralytics to ONNX: `yolo export model=yolo11s.pt format=onnx` (or use the `ultralytics` API).
  - Convert ONNX → TensorRT engine (use trtexec or TensorRT Python API). Use FP16/INT8 where possible.
  - Run the TensorRT engine to get lower latency and higher throughput.
- Use mixed precision (FP16) on Jetson when supported: convert the model and run inference with half precision. This reduces memory and compute.
- Reduce slice overlap or slice size if latency is too high (trade-off: less overlap → borderline objects may be missed). For many cases, smaller slices reduce per-slice compute but increase slice count.
- Warm up the model after loading by running a few dummy inferences to stabilize cuDNN kernels and caches.
- Set `torch.backends.cudnn.benchmark = True` if input sizes are consistent to gain speed on convolution benchmarks.

Practical checklist for Jetson:

1. Confirm JetPack/CUDA/cuDNN versions.
2. Install a Jetson-compatible PyTorch with CUDA support.
3. Use a small YOLO model or convert to TensorRT with FP16.
4. Monitor GPU memory and reduce slice size/overlap if OOMs occur.

---

## Re-implementation / portability guidance

If you need to reimplement this pipeline in another framework or operator:

- Keep the same high-level steps: slice → detect per-slice → merge/postprocess → annotate/publish.
- Make slicing parameters configurable and experiment with slice size vs. overlap to balance latency and recall for your dataset.
- Keep class-mapping logic (`_categorize_detection`) as a pluggable mapping so you can adapt to different detectors/labels.
- Implement class-wise NMS and support both IoU and IOS as merge metrics; SAHI provides useful utilities, but you can reimplement merging if you want lower dependency footprint.
- Ensure device detection and graceful fallback are present; provide clear logs when GPU is expected but inaccessible.

---

## Troubleshooting / common errors & fixes

- SAHI ImportError: `SAHI not available` — run `pip install sahi` in the environment used by ROS2. Prefer a virtualenv or the same Python used by ROS.
- Ultralytics/YOLO ImportError: `Ultralytics YOLO not available` — `pip install ultralytics`.
- PyTorch present but no CUDA: node will log a Jetson-specific error. Fix by installing a CUDA-enabled PyTorch for the Jetson platform (special wheel or instructions on NVIDIA forums).
- OOM on model load: try a smaller model, reduce slice size, or convert to TensorRT FP16.
- Detections missing: increase slice size or overlap, lower confidence thresholds in `_categorize_detection`, or improve model training data for small objects.

---

## Testing and verification suggestions

- Unit test the small helper functions (`_calculate_iou`, `_apply_nms`, `_filter_detections`) with deterministic boxes to ensure NMS behavior.
- Run offline benchmarks with a representative set of images and measure `avg_processing_time` recorded by the node.
- On Jetson, measure latency and memory usage using `tegrastats` and tune slice size, overlap, or model format accordingly.

---

## Last notes and where to look in the code

- The primary file to study is `object_detection_sahi.py`. For inference details look at `detect_objects_sahi` and `initialize_sahi_model`.
- For ROS2 integration, check the parameter declarations in `__init__` and publishers in `publish_results`.
- The code contains inline TODOs (e.g., MobileNet validation) for future improvements.

If you want, I can also:

- Produce a short README with example ROS2 launch/invocation commands that override parameters.
- Add a small unit test file exercising the NMS and IoU helpers.
- Add optional code to export the model to ONNX/TensorRT and a minimal runner for Jetson.

---

Document created to assist future maintainers reimplementing, optimizing, and troubleshooting the SAHI+YOLO small-object detection pipeline.
