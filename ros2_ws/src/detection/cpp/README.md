# Annotation: first C++ migration step

This standalone C++17/OpenCV library ports `detection/annotation.py`. It draws
existing predictions; it does not load a model or run inference. The Python ROS
node still calls the original Python module until the later node migration.
No Python interpreter, SAHI, PyTorch, CUDA, or ROS installation is needed to use
this C++ library. Python is used only by the optional equivalence test.

## Files

- `include/detection/annotation.hpp`: public data type and function declarations.
- `src/annotation.cpp`: drawing, crop composition, and image saving.
- `CMakeLists.txt`: standalone library and CTest target.
- `test/annotation_test.cpp`: behavioral and error tests; writes comparison images.
- `test/compare_python.py`: compares those images against the original Python code.

## Build and test

On Ubuntu with CMake, a C++ compiler, and OpenCV 4 development packages installed,
run these commands from the repository root:

```bash
cmake -S ros2_ws/src/detection/cpp -B /tmp/odlc-annotation-build
cmake --build /tmp/odlc-annotation-build -j2
ctest --test-dir /tmp/odlc-annotation-build --output-on-failure
```

For an optional pixel-for-pixel comparison, also install Python OpenCV and NumPy:

```bash
python3 ros2_ws/src/detection/cpp/test/compare_python.py \
  ros2_ws/src/detection/detection/annotation.py \
  /tmp/odlc-annotation-build/test-output
```

`test/Dockerfile` builds a disposable test environment from this developer
machine's existing `yolo26-ros:latest` image. It is not a deployment image or a
required dependency of the library.

## Calling the C++ API

```cpp
#include "detection/annotation.hpp"
#include <opencv2/imgcodecs.hpp>

cv::Mat frame = cv::imread("photo.png");
std::vector<detection::AnnotationDetection> detections{
    {"person", 0.87, {120, 80, 190, 230}},
    {"tent", 0.92, {300, 150, 550, 350}},
};
cv::Mat annotated = detection::annotate_frame(
    frame, detections, 1.2, 640, 640, "tensorrt");
std::filesystem::create_directories("results");
auto crop_path = detection::save_top_matches_crop(
    frame, detections, "photo.png", "results");
// crop_path is std::nullopt if there is no valid target crop.
```

Link your executable to `detection_annotation`. Both functions expect a nonempty
`CV_8UC3` BGR image, exactly like one returned by `cv::imread` with default flags.
Bounding boxes are `[x1, y1, x2, y2]` in image pixels, with exclusive right/bottom
edges for cropping. Confidence must be finite and between zero and one.

## Section-by-section explanation

### 1. Public interface and detection data

Python's dictionaries become `AnnotationDetection`: `class_name` replaces
`det['class']`, `confidence` holds the score, and `std::array<int, 4>` holds the
bounding box. `std::vector<AnnotationDetection>` replaces the list. This catches
missing fields and incorrect types during compilation. The annotation-only type
does not require a model manager or inferencing dependencies.

The header declares the two functions so other C++ files can call them. Their
definitions live in the `.cpp` file. Namespace `detection` prevents name clashes.

### 2. Colors, fonts, input checks, and formatting

Python BGR tuples become `cv::Scalar`. Font, scale, thickness, colors, and
antialiasing match the original. Internal helpers are hidden in an anonymous
namespace. `std::ostringstream` replaces f-string number formatting; the classic
locale prevents a user's locale from replacing a decimal point with a comma.

The port rejects empty/non-BGR images, invalid confidence, invalid processing
time, and nonpositive slice dimensions with exceptions. These explicit input
contracts are additions to the original implementation.

### 3. Bounding boxes and labels

`frame.copy()` becomes `frame.clone()`: plain C++ `cv::Mat` assignment shares the
image memory and would allow the drawing code to modify the caller's image.

Each detection draws the same green person box, orange-yellow tent box, or blue
unlabeled other-object box. `cv::getTextSize` calculates label dimensions. The
same padding and coordinates place text above a box when possible or below its
top edge otherwise. Person labels align right and tent labels align left. Text
is drawn twice, black then white, with the original thicknesses.

### 4. Summary header

`std::count_if` replaces the Python counting expressions. The header still shows
people/tents/other counts, backend, processing seconds, and slice size. An overlay
copy, filled rectangle, and `cv::addWeighted` preserve the translucent background.

The backend string is descriptive input, not hardware detection. As in Python,
`"tensorrt"` produces `TensorRT`; other values produce `PyTorch`. This convention
will need extending when an ONNX backend is added in the model-manager migration.

### 5. Highest-confidence target selection

A single pass keeps pointers to the best person and tent, replacing filtered
lists and `max(..., key=...)`. Pointers refer to the caller's vector only during
this function call; nothing owns or deletes those detections. Equal confidence
keeps the first detection, matching Python's `max` behavior. No heap allocation
or copy of a detection is needed.

Selection occurs before validating crop coordinates, matching the original:
an invalid highest-confidence box does not cause selection of a lower-scoring box.

### 6. Cropping, resizing, and captions

Coordinates are clipped to the image. Invalid or fully out-of-frame crops are
skipped. NumPy slicing becomes a `cv::Rect` region of interest. The crop is only
read, so it does not need its own deep copy. `cv::resize` produces an independent
image at height 200 using `INTER_AREA`, preserving the aspect ratio.

`cv::Mat::zeros` replaces `np.zeros`, and `copyTo` replaces NumPy assignment.
Caption positions and dark backgrounds match Python. A helper reproduces Python
floor division when a caption is wider than its crop; C++ division otherwise
truncates negative values toward zero and can shift the text by one pixel.

For extremely tall, thin crops, the resized width is clamped to at least one
pixel. Python can round the width down to zero and fail in `cv2.resize`.

### 7. Combining and saving

Person and tent crops are placed in that order, with the original five-pixel gray
separator. `std::filesystem::path` replaces `os.path` operations. Prefixing the
input filename with `TM_` preserves the name and extension. The caller creates
the output directory, as in the original pipeline.

Python's `None` becomes `std::nullopt`; a successful save returns the output path.
The port checks `cv::imwrite` and throws if saving fails. Python ignored its
boolean return value and could report a path without actually saving the image.

### 8. Build and verification

The standalone CMake target links only OpenCV core, imgproc, and imgcodecs.
Tests cover no targets, other objects, each target separately, multiple targets,
confidence ties, clipping, invalid boxes, caption alignment, both backend headers,
source-image preservation, a very narrow crop, and invalid input/write errors.
The optional Python test compares decoded pixels rather than PNG file bytes,
since different encoders can store identical pixels differently.

Verified in the local Docker test environment with GCC 11.4 and OpenCV 4.5.4:
the C++ library compiles, CTest passes, and 15 outputs across nine scenarios match
the original Python pixel for pixel. Single-pixel and extremely narrow crops are
tested separately in C++; the original Python/OpenCV 4.5.4 path fails for these
edge cases. This is library verification, not a ROS integration or GPU test.

## Scope of this step

The original `annotation.py` remains available so `new_od.py` continues to import
it. This library becomes the active annotation implementation when the ROS node
is ported to C++. No changes to model behavior or the `ultralytics_ros` submodule
are required for this step.
