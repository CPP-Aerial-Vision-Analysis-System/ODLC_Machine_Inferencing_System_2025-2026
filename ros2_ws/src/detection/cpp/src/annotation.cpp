#include "detection/annotation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace detection {
namespace {

// OpenCV uses BGR, matching the Python implementation.
const cv::Scalar kTent{0, 200, 255};
const cv::Scalar kPerson{0, 200, 0};
const cv::Scalar kObject{255, 100, 0};
const cv::Scalar kWhite{255, 255, 255};
const cv::Scalar kBlack{0, 0, 0};
constexpr int kFont = cv::FONT_HERSHEY_DUPLEX;
constexpr double kFontScale = 0.45;
constexpr int kFontThickness = 1;

void validate_frame(const cv::Mat& frame) {
  if (frame.empty() || frame.type() != CV_8UC3) {
    throw std::invalid_argument("Expected a nonempty CV_8UC3 BGR image");
  }
}

void validate_detections(const std::vector<AnnotationDetection>& detections) {
  for (const auto& det : detections) {
    if (!std::isfinite(det.confidence) || det.confidence < 0 || det.confidence > 1) {
      throw std::invalid_argument("Detection confidence must be finite and in [0, 1]");
    }
  }
}

std::string fixed(double value, int precision) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string percent(double confidence) {
  return fixed(confidence * 100.0, 0) + "%";
}

// Python's // floors negative numbers; C++ integer division truncates to zero.
int floor_half(int value) {
  return value / 2 - (value < 0 && value % 2 != 0 ? 1 : 0);
}

}  // namespace

// Section 1: boxes and labels on a copy of the source image.
cv::Mat annotate_frame(
    const cv::Mat& frame,
    const std::vector<AnnotationDetection>& detections,
    double processing_time, int slice_height, int slice_width,
    const std::string& model_format) {
  validate_frame(frame);
  validate_detections(detections);
  if (!std::isfinite(processing_time) || processing_time < 0 ||
      slice_height <= 0 || slice_width <= 0) {
    throw std::invalid_argument("Expected nonnegative time and positive slice dimensions");
  }
  cv::Mat annotated = frame.clone();

  for (const auto& det : detections) {
    const auto [x1, y1, x2, y2] = det.bbox;
    const bool person = det.class_name == "person";
    const bool tent = det.class_name == "tent";
    if (!person && !tent) {
      cv::rectangle(annotated, {x1, y1}, {x2, y2}, kObject, 1);
      continue;
    }
    const auto& color = person ? kPerson : kTent;
    const std::string label = std::string(person ? "PERSON (" : "TENT (") +
                              percent(det.confidence) + ")";
    cv::rectangle(annotated, {x1, y1}, {x2, y2}, color, 1);

    int baseline = 0;
    const auto text = cv::getTextSize(label, kFont, kFontScale, kFontThickness, &baseline);
    constexpr int pad = 4;
    const int label_height = text.height + pad * 2;
    const int ly1 = y1 - label_height >= 0 ? y1 - label_height : y1;
    const int ly2 = y1 - label_height >= 0 ? y1 : y1 + label_height;
    const int lx1 = person ? std::max(0, x2 - text.width - pad * 2) : x1;
    const int lx2 = person ? x2 : x1 + text.width + pad * 2;
    cv::rectangle(annotated, {lx1, ly1}, {lx2, ly2}, color, cv::FILLED);
    const cv::Point origin{lx1 + pad, ly2 - pad};
    cv::putText(annotated, label, origin, kFont, kFontScale, kBlack,
                kFontThickness + 1, cv::LINE_AA);
    cv::putText(annotated, label, origin, kFont, kFontScale, kWhite,
                kFontThickness, cv::LINE_AA);
  }

  // Section 2: detection counts and the translucent status header.
  const auto people = std::count_if(detections.begin(), detections.end(),
      [](const auto& d) { return d.class_name == "person"; });
  const auto tents = std::count_if(detections.begin(), detections.end(),
      [](const auto& d) { return d.class_name == "tent"; });
  const auto other = detections.size() - people - tents;
  const std::string engine = model_format == "tensorrt" ? "TensorRT" : "PyTorch";
  const std::string header = "SAHI+YOLO (" + engine + ") | " + std::to_string(people) +
      " people, " + std::to_string(tents) + " tents, " + std::to_string(other) + " other";
  const std::string subline = "Time: " + fixed(processing_time, 1) + "s | Slices: " +
      std::to_string(slice_height) + "x" + std::to_string(slice_width);
  cv::Mat overlay = annotated.clone();
  cv::rectangle(overlay, {0, 0}, {frame.cols, 50}, kBlack, cv::FILLED);
  cv::addWeighted(overlay, 0.6, annotated, 0.4, 0, annotated);
  cv::putText(annotated, header, {10, 20}, kFont, 0.6, kWhite, 1, cv::LINE_AA);
  cv::putText(annotated, subline, {10, 42}, kFont, 0.6 * 0.8,
              {200, 200, 200}, 1, cv::LINE_AA);
  return annotated;
}

std::optional<std::filesystem::path> save_top_matches_crop(
    const cv::Mat& frame,
    const std::vector<AnnotationDetection>& detections,
    const std::filesystem::path& image_path,
    const std::filesystem::path& output_dir) {
  validate_frame(frame);
  validate_detections(detections);

  // Section 3: select one winner per target class, retaining the first on ties.
  const AnnotationDetection* best_person = nullptr;
  const AnnotationDetection* best_tent = nullptr;
  for (const auto& det : detections) {
    if (det.class_name == "person" &&
        (!best_person || det.confidence > best_person->confidence)) best_person = &det;
    if (det.class_name == "tent" &&
        (!best_tent || det.confidence > best_tent->confidence)) best_tent = &det;
  }

  // Section 4: clip, resize, and add a caption underneath each valid crop.
  std::vector<cv::Mat> resized;
  const std::array<std::pair<const AnnotationDetection*, const char*>, 2> matches{{
      {best_person, "PERSON"}, {best_tent, "TENT"}}};
  for (const auto& [best, tag] : matches) {
    if (!best) continue;
    const int x1 = std::clamp(best->bbox[0], 0, frame.cols);
    const int y1 = std::clamp(best->bbox[1], 0, frame.rows);
    const int x2 = std::clamp(best->bbox[2], 0, frame.cols);
    const int y2 = std::clamp(best->bbox[3], 0, frame.rows);
    if (x2 <= x1 || y2 <= y1) continue;
    const cv::Mat crop = frame(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    constexpr int target_height = 200;
    const double scaled_width = crop.cols * (double(target_height) / crop.rows);
    if (scaled_width >= std::numeric_limits<int>::max()) {
      throw std::overflow_error("Crop is too wide to resize");
    }
    // Unlike Python, keep extremely narrow crops at least one pixel wide.
    const int width = std::max(1, static_cast<int>(scaled_width));
    cv::Mat scaled;
    cv::resize(crop, scaled, {width, target_height}, 0, 0, cv::INTER_AREA);
    const std::string label = std::string(tag) + " " + percent(best->confidence);
    int baseline = 0;
    const auto text = cv::getTextSize(label, kFont, 0.5, 1, &baseline);
    const int label_height = text.height + 10;
    cv::Mat padded = cv::Mat::zeros(target_height + label_height, width, CV_8UC3);
    scaled.copyTo(padded(cv::Rect(0, 0, width, target_height)));
    cv::rectangle(padded, {0, target_height}, {width, target_height + label_height},
                  {40, 40, 40}, cv::FILLED);
    cv::putText(padded, label, {floor_half(width - text.width), target_height + text.height + 3},
                kFont, 0.5, kWhite, 1, cv::LINE_AA);
    resized.push_back(std::move(padded));
  }
  if (resized.empty()) return std::nullopt;

  // Section 5: assemble person then tent, separated by five gray pixels, and save.
  constexpr int separator = 5;
  std::int64_t total_width = separator * (resized.size() - 1);
  for (const auto& crop : resized) total_width += crop.cols;
  if (total_width > std::numeric_limits<int>::max()) {
    throw std::overflow_error("Combined crops are too wide");
  }
  cv::Mat combined = cv::Mat::zeros(resized.front().rows, static_cast<int>(total_width), CV_8UC3);
  int offset = 0;
  for (std::size_t i = 0; i < resized.size(); ++i) {
    if (i > 0) {
      combined.colRange(offset, offset + separator).setTo(cv::Scalar(80, 80, 80));
      offset += separator;
    }
    resized[i].copyTo(combined(cv::Rect(offset, 0, resized[i].cols, resized[i].rows)));
    offset += resized[i].cols;
  }
  const auto output_path = output_dir / ("TM_" + image_path.filename().string());
  if (!cv::imwrite(output_path.string(), combined)) {
    throw std::runtime_error("Failed to save crop image: " + output_path.string());
  }
  return output_path;
}

}  // namespace detection
