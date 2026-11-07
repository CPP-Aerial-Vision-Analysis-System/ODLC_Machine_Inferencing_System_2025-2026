#include "detection/annotation.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;
using detection::AnnotationDetection;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <class Function>
void expect_failure(Function function, const std::string& message) {
  bool failed = false;
  try { function(); } catch (const std::exception&) { failed = true; }
  require(failed, message);
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Expected output directory argument");
    const fs::path output = argv[1];
    fs::create_directories(output);
    cv::Mat frame(600, 1200, CV_8UC3);
    for (int y = 0; y < frame.rows; ++y) {
      for (int x = 0; x < frame.cols; ++x) {
        frame.at<cv::Vec3b>(y, x) = cv::Vec3b(x % 256, y % 256, (x + y) % 256);
      }
    }
    const cv::Mat original = frame.clone();
    require(cv::imwrite((output / "source.png").string(), frame), "Save source");

    const std::vector<std::vector<AnnotationDetection>> cases{
      {},
      {{"object", 0.5, {300, 100, 400, 200}}},
      {{"person", 0.87, {200, 100, 260, 300}}},
      {{"tent", 0.65, {400, 200, 650, 400}}},
      {{"person", 0.7, {200, 100, 260, 300}},
       {"person", 0.95, {300, 150, 400, 400}},
       {"tent", 0.85, {500, 200, 700, 400}},
       {"object", 0.6, {800, 200, 900, 300}}},
      {{"person", 0.125, {-20, -10, 50, 90}},
       {"tent", 0.375, {1100, 500, 1300, 700}}},
      {{"person", 0.9, {1400, 700, 1500, 800}},
       {"tent", 0.8, {500, 300, 500, 400}}},
      {{"person", 0.5, {50, 2, 52, 4}},
       {"person", 0.5, {600, 200, 800, 400}},
       {"tent", 1.0, {300, 100, 302, 104}}},
      {{"person", 0.0, {100, 100, 110, 200}},
       {"tent", 1.0, {500, 100, 510, 200}}}
    };

    for (std::size_t i = 0; i < cases.size(); ++i) {
      const auto dir = output / ("case_" + std::to_string(i));
      fs::create_directories(dir);
      const std::string format = i % 2 == 0 ? "tensorrt" : "pytorch";
      auto annotated = detection::annotate_frame(frame, cases[i], 1.25, 640, 512, format);
      require(annotated.size() == frame.size(), "Annotated dimensions changed");
      require(annotated.data != frame.data, "Annotated image aliases source");
      require(cv::norm(frame, original, cv::NORM_INF) == 0, "Annotation mutated source");
      require(cv::imwrite((dir / "annotated.png").string(), annotated), "Save annotation");
      const auto crop = detection::save_top_matches_crop(frame, cases[i], "input/photo.png", dir);
      const bool expect_crop = i != 0 && i != 1 && i != 6;
      require(crop.has_value() == expect_crop, "Unexpected crop presence");
      if (crop) {
        require(crop->filename() == "TM_photo.png", "Wrong crop filename");
        require(!cv::imread(crop->string()).empty(), "Crop unreadable");
      }
      require(cv::norm(frame, original, cv::NORM_INF) == 0, "Cropping mutated source");
      std::ofstream fixture(dir / "detections.txt");
      fixture << format << '\n';
      for (const auto& d : cases[i]) {
        fixture << d.class_name << ' ' << d.confidence;
        for (int v : d.bbox) fixture << ' ' << v;
        fixture << '\n';
      }
    }

    // This previously caused cv2.resize to fail with a zero-width destination.
    const auto narrow = detection::save_top_matches_crop(
        frame, {{"person", 0.8, {100, 0, 101, 600}}}, "narrow.png", output);
    require(narrow && cv::imread(narrow->string()).cols == 1, "Narrow crop must be one pixel wide");
    const auto single = detection::save_top_matches_crop(
        frame, {{"person", 0.5, {50, 2, 51, 3}}}, "single.png", output);
    require(single && cv::imread(single->string()).cols == 200, "Single-pixel crop failed");
    expect_failure([&] { detection::annotate_frame({}, {}, 1, 640, 640, "pytorch"); },
                   "Empty image accepted");
    expect_failure([&] { detection::annotate_frame(cv::Mat(10, 10, CV_8UC1), {}, 1, 640, 640, "pytorch"); },
                   "Grayscale image accepted");
    expect_failure([&] { detection::annotate_frame(frame,
        {{"person", std::numeric_limits<double>::quiet_NaN(), {1, 2, 3, 4}}}, 1, 640, 640, "pytorch"); },
                   "NaN confidence accepted");
    expect_failure([&] { detection::annotate_frame(frame, {}, -1, 640, 640, "pytorch"); },
                   "Negative time accepted");
    expect_failure([&] { detection::annotate_frame(frame, {}, 1, 0, 640, "pytorch"); },
                   "Zero slice size accepted");
    expect_failure([&] { detection::save_top_matches_crop(frame,
        {{"person", 0.8, {10, 10, 30, 40}}}, "photo.png", output / "missing" / "directory"); },
                   "Failed image write was ignored");
    std::cout << "PASS: 9 rendering scenarios, source preservation, crop selection, "
                 "narrow crop, and input/write error checks\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
