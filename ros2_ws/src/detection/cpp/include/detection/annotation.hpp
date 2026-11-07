#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace detection {

// Pixel coordinates use the same [x1, y1, x2, y2] convention as annotation.py.
// The crop's right and bottom coordinates are exclusive.
struct AnnotationDetection {
  std::string class_name;
  double confidence;
  std::array<int, 4> bbox;
};

// Draw on a deep copy; frame must be a nonempty, 8-bit, three-channel BGR image.
// Preserve the Python backend header convention: "tensorrt" or otherwise PyTorch.
cv::Mat annotate_frame(
    const cv::Mat& frame,
    const std::vector<AnnotationDetection>& detections,
    double processing_time, int slice_height, int slice_width,
    const std::string& model_format);

// Returns nullopt when no valid person/tent crop exists. The output directory
// must already exist. Throws on invalid input or a failed image write.
std::optional<std::filesystem::path> save_top_matches_crop(
    const cv::Mat& frame,
    const std::vector<AnnotationDetection>& detections,
    const std::filesystem::path& image_path,
    const std::filesystem::path& output_dir);

}  // namespace detection
