// mapping: incremental aerial image stitcher (C++ port of mapping/mapping.py)
//
// Loads the captured survey photos from video_cam/mapping_photos, estimates
// frame-to-frame affine transforms (ORB or SIFT + RANSAC), accumulates a global
// homography, and distance-transform-blends each frame into a growing panorama,
// which is cropped and saved as final_panorama.jpg.
//
// The Python original referenced several attributes it never set
// (camera_feed_path, input_dir, output_dir, save_dir), so it crashed on
// startup; this port resolves those paths from the ros2_ws layout.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/status_text.hpp"

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

namespace
{
// Resolve <ros2_ws>/src by walking up from the current directory (install+src),
// falling back to $HOME.
fs::path ros2_ws_src()
{
  fs::path search = fs::current_path();
  for (int i = 0; i < 10; ++i) {
    if (fs::is_directory(search / "install") && fs::is_directory(search / "src")) {
      return search / "src";
    }
    if (search.parent_path() == search) {
      break;
    }
    search = search.parent_path();
  }
  const char * home = std::getenv("HOME");
  return fs::path(home ? home : "/");
}

// Result of a frame-to-frame affine estimate.
struct AffineResult
{
  bool ok{false};
  cv::Mat H;  // 3x3
  int inliers{0};
  std::string info;
};

cv::Mat enhance(const cv::Mat & img)
{
  cv::Mat gray;
  if (img.channels() == 3) {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = img;
  }
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  cv::Mat out;
  clahe->apply(gray, out);
  return out;
}

// Estimate affine transform mapping img_new -> img_ref coordinates.
AffineResult estimate_affine(
  const cv::Mat & img_ref, const cv::Mat & img_new, int min_matches, double ratio, bool use_sift,
  double ransac_thresh)
{
  AffineResult res;
  cv::Mat g_ref = enhance(img_ref);
  cv::Mat g_new = enhance(img_new);

  cv::Ptr<cv::Feature2D> det;
  int norm_type;
  if (use_sift) {
    det = cv::SIFT::create(5000);
    norm_type = cv::NORM_L2;
  } else {
    det = cv::ORB::create(8000, 1.2f, 8, 15, 0, 2, cv::ORB::HARRIS_SCORE, 31, 10);
    norm_type = cv::NORM_HAMMING;
  }

  std::vector<cv::KeyPoint> k_ref, k_new;
  cv::Mat d_ref, d_new;
  det->detectAndCompute(g_ref, cv::noArray(), k_ref, d_ref);
  det->detectAndCompute(g_new, cv::noArray(), k_new, d_new);

  if (d_ref.empty() || d_new.empty() ||
    static_cast<int>(k_ref.size()) < min_matches ||
    static_cast<int>(k_new.size()) < min_matches)
  {
    res.info = "Too few keypoints";
    return res;
  }

  cv::BFMatcher bf(norm_type, false);
  std::vector<std::vector<cv::DMatch>> knn;
  bf.knnMatch(d_new, d_ref, knn, 2);

  std::vector<cv::DMatch> good;
  for (const auto & m : knn) {
    if (m.size() == 2 && m[0].distance < ratio * m[1].distance) {
      good.push_back(m[0]);
    }
  }

  const int MIN_INLIERS = 10;
  const double MIN_INLIER_RATIO = 0.25;

  if (static_cast<int>(good.size()) < min_matches) {
    res.info = "Too few good matches: " + std::to_string(good.size());
    return res;
  }

  std::vector<cv::Point2f> src, dst;
  src.reserve(good.size());
  dst.reserve(good.size());
  for (const auto & m : good) {
    src.push_back(k_new[m.queryIdx].pt);
    dst.push_back(k_ref[m.trainIdx].pt);
  }

  cv::Mat inlier_mask;
  cv::Mat M = cv::estimateAffinePartial2D(
    src, dst, inlier_mask, cv::RANSAC, ransac_thresh, 5000, 0.99, 10);

  if (M.empty() || inlier_mask.empty()) {
    res.info = "Affine estimation failed";
    return res;
  }

  const int inliers = cv::countNonZero(inlier_mask);
  const double ratio_i = static_cast<double>(inliers) / static_cast<double>(good.size());
  if (inliers < MIN_INLIERS || ratio_i < MIN_INLIER_RATIO) {
    res.inliers = inliers;
    res.info = "Weak inliers: " + std::to_string(inliers);
    return res;
  }

  const double sx = std::hypot(M.at<double>(0, 0), M.at<double>(0, 1));
  const double sy = std::hypot(M.at<double>(1, 0), M.at<double>(1, 1));
  if (!(0.65 < sx && sx < 1.55 && 0.65 < sy && sy < 1.55)) {
    res.inliers = inliers;
    res.info = "Bad scale";
    return res;
  }

  cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
  M.copyTo(H(cv::Rect(0, 0, 3, 2)));
  res.ok = true;
  res.H = H;
  res.inliers = inliers;
  res.info = "OK inliers=" + std::to_string(inliers);
  return res;
}

struct BlendResult
{
  cv::Mat panorama;
  cv::Mat mask;
  int tx{0};
  int ty{0};
  bool overlap_ok{false};
  std::string info;
};

// Warp `frame` into panorama coords via H_global and distance-transform blend.
BlendResult warp_and_blend(
  const cv::Mat & panorama, const cv::Mat & pano_mask, const cv::Mat & frame,
  const cv::Mat & H_global)
{
  BlendResult out;
  const int h_p = panorama.rows, w_p = panorama.cols;
  const int h_f = frame.rows, w_f = frame.cols;

  std::vector<cv::Point2f> corners_f = {
    {0, 0}, {0, static_cast<float>(h_f)}, {static_cast<float>(w_f), static_cast<float>(h_f)},
    {static_cast<float>(w_f), 0}};
  std::vector<cv::Point2f> corners_w;
  cv::perspectiveTransform(corners_f, corners_w, H_global);

  std::vector<cv::Point2f> all_c = {
    {0, 0}, {0, static_cast<float>(h_p)}, {static_cast<float>(w_p), static_cast<float>(h_p)},
    {static_cast<float>(w_p), 0}};
  all_c.insert(all_c.end(), corners_w.begin(), corners_w.end());

  float xmin = all_c[0].x, ymin = all_c[0].y, xmax = all_c[0].x, ymax = all_c[0].y;
  for (const auto & p : all_c) {
    xmin = std::min(xmin, p.x);
    ymin = std::min(ymin, p.y);
    xmax = std::max(xmax, p.x);
    ymax = std::max(ymax, p.y);
  }

  const int tx = std::max(0, static_cast<int>(-(xmin - 0.5f)));
  const int ty = std::max(0, static_cast<int>(-(ymin - 0.5f)));
  const int out_w = static_cast<int>(xmax + 0.5f) - static_cast<int>(xmin - 0.5f);
  const int out_h = static_cast<int>(ymax + 0.5f) - static_cast<int>(ymin - 0.5f);
  out.tx = tx;
  out.ty = ty;

  const int max_dim = std::max({w_p, h_p, w_f, h_f}) * 8;
  if (out_w <= 0 || out_h <= 0 || out_w > max_dim || out_h > max_dim) {
    out.panorama = panorama;
    out.mask = pano_mask;
    out.tx = 0;
    out.ty = 0;
    out.info = "Canvas too large";
    return out;
  }

  cv::Mat T = cv::Mat::eye(3, 3, CV_64F);
  T.at<double>(0, 2) = tx;
  T.at<double>(1, 2) = ty;
  cv::Mat TH = T * H_global;

  cv::Mat warped_f;
  cv::warpPerspective(frame, warped_f, TH, cv::Size(out_w, out_h));
  cv::Mat frame_fill(h_f, w_f, CV_8U, cv::Scalar(255));
  cv::Mat warped_fm;
  cv::warpPerspective(frame_fill, warped_fm, TH, cv::Size(out_w, out_h));

  cv::Mat new_pano = cv::Mat::zeros(out_h, out_w, CV_8UC3);
  panorama.copyTo(new_pano(cv::Rect(tx, ty, w_p, h_p)));
  cv::Mat new_mask = cv::Mat::zeros(out_h, out_w, CV_8U);
  pano_mask.copyTo(new_mask(cv::Rect(tx, ty, w_p, h_p)));

  cv::Mat overlap;
  cv::bitwise_and(new_mask, warped_fm, overlap);
  const int overlap_px = cv::countNonZero(overlap);
  const int frame_px = cv::countNonZero(warped_fm);
  const double overlap_frac = frame_px > 0 ? static_cast<double>(overlap_px) / frame_px : 0.0;
  if (overlap_frac < 0.03) {
    out.panorama = panorama;
    out.mask = pano_mask;
    out.info = "No overlap";
    return out;
  }

  cv::Mat dist1, dist2;
  cv::distanceTransform(new_mask, dist1, cv::DIST_L2, 5);
  cv::distanceTransform(warped_fm, dist2, cv::DIST_L2, 5);
  cv::Mat d_sum = dist1 + dist2 + 1e-6;
  cv::Mat w1, w2;
  cv::divide(dist1, d_sum, w1);
  cv::divide(dist2, d_sum, w2);
  cv::GaussianBlur(w1, w1, cv::Size(31, 31), 10);
  cv::GaussianBlur(w2, w2, cv::Size(31, 31), 10);
  cv::Mat ws = w1 + w2 + 1e-6;
  cv::divide(w1, ws, w1);
  cv::divide(w2, ws, w2);

  cv::Mat result = new_pano.clone();
  for (int y = 0; y < out_h; ++y) {
    const uchar * mp = new_mask.ptr<uchar>(y);
    const uchar * mf = warped_fm.ptr<uchar>(y);
    const cv::Vec3b * npx = new_pano.ptr<cv::Vec3b>(y);
    const cv::Vec3b * fpx = warped_f.ptr<cv::Vec3b>(y);
    const float * w1p = w1.ptr<float>(y);
    const float * w2p = w2.ptr<float>(y);
    cv::Vec3b * rp = result.ptr<cv::Vec3b>(y);
    for (int x = 0; x < out_w; ++x) {
      const bool in_pano = mp[x] > 0;
      const bool in_frame = mf[x] > 0;
      if (in_frame && !in_pano) {
        rp[x] = fpx[x];
      } else if (in_pano && in_frame) {
        for (int c = 0; c < 3; ++c) {
          rp[x][c] = cv::saturate_cast<uchar>(npx[x][c] * w1p[x] + fpx[x][c] * w2p[x]);
        }
      }
    }
  }

  cv::Mat new_mask_out;
  cv::bitwise_or(new_mask, warped_fm, new_mask_out);

  out.panorama = result;
  out.mask = new_mask_out;
  out.overlap_ok = true;
  out.info = "Blended overlap";
  return out;
}

int numeric_key(const std::string & path)
{
  std::string digits;
  for (char ch : fs::path(path).filename().string()) {
    if (std::isdigit(static_cast<unsigned char>(ch))) {
      digits.push_back(ch);
    }
  }
  return digits.empty() ? 0 : std::stoi(digits);
}
}  // namespace

class MappingNode : public rclcpp::Node
{
public:
  MappingNode()
  : Node("mapping_node")
  {
    use_sift_ = declare_parameter<bool>("use_sift", false);
    downscale_ = declare_parameter<double>("downscale_factor", 0.5);
    max_frames_ = declare_parameter<int>("max_frames", 150);
    min_matches_ = declare_parameter<int>("min_matches", 6);
    ratio_test_ = declare_parameter<double>("ratio_test", 0.8);
    ransac_thresh_ = declare_parameter<double>("ransac_thresh", 3.0);

    const fs::path ws_src = ros2_ws_src();
    camera_feed_path_ = (ws_src / "video_cam" / "mapping_photos").string();
    save_dir_ = (ws_src / "mapping" / "panoramas").string();
    std::error_code ec;
    fs::create_directories(save_dir_, ec);

    status_pub_ = create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);

    RCLCPP_INFO(
      get_logger(), "Mapping node initialized: detector=%s downscale=%.2f input=%s output=%s",
      use_sift_ ? "SIFT" : "ORB", downscale_, camera_feed_path_.c_str(), save_dir_.c_str());

    load_image_list();
    run_mapping();
    finish();
  }

private:
  void load_image_list()
  {
    static const std::vector<std::string> exts = {
      ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"};
    std::error_code ec;
    if (!fs::is_directory(camera_feed_path_, ec)) {
      RCLCPP_ERROR(get_logger(), "Input directory missing: %s", camera_feed_path_.c_str());
      return;
    }
    for (const auto & entry : fs::directory_iterator(camera_feed_path_, ec)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      std::string ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (std::find(exts.begin(), exts.end(), ext) != exts.end()) {
        image_files_.push_back(entry.path().string());
      }
    }
    std::sort(image_files_.begin(), image_files_.end(), [](const std::string & a, const std::string & b) {
      return numeric_key(a) < numeric_key(b);
    });
  }

  void run_mapping()
  {
    const int WINDOW = 10;
    // keyframes: (small frame, H at time of stitching)
    std::vector<std::pair<cv::Mat, cv::Mat>> keyframes;

    for (const auto & image_path : image_files_) {
      if (frames_processed_ >= max_frames_) {
        break;
      }
      cv::Mat frame = cv::imread(image_path);
      if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "Cannot read: %s", image_path.c_str());
        continue;
      }
      cv::Mat frame_small;
      cv::resize(frame, frame_small, cv::Size(), downscale_, downscale_);
      ++frames_processed_;
      const std::string fname = fs::path(image_path).filename().string();

      if (panorama_.empty()) {
        panorama_ = frame_small.clone();
        pano_mask_ = cv::Mat(frame_small.size(), CV_8U, cv::Scalar(255));
        H_global_ = cv::Mat::eye(3, 3, CV_64F);
        keyframes.emplace_back(frame_small.clone(), H_global_.clone());
        frames_stitched_ = 1;
        RCLCPP_INFO(get_logger(), "Bootstrap: %s", fname.c_str());
        continue;
      }

      // Try matching against recent keyframes, newest first.
      cv::Mat matched_H, matched_ref_H;
      bool matched = false;
      const int start = std::max(0, static_cast<int>(keyframes.size()) - WINDOW);
      for (int i = static_cast<int>(keyframes.size()) - 1; i >= start; --i) {
        AffineResult r = estimate_affine(
          keyframes[i].first, frame_small, min_matches_, ratio_test_, use_sift_, ransac_thresh_);
        if (r.ok) {
          matched_H = r.H;
          matched_ref_H = keyframes[i].second;
          matched = true;
          RCLCPP_INFO(get_logger(), "%s: %s", fname.c_str(), r.info.c_str());
          break;
        }
      }

      if (!matched) {
        RCLCPP_WARN(get_logger(), "Skipped %s: no candidate matched", fname.c_str());
        keyframes.emplace_back(frame_small.clone(), keyframes.back().second);
        continue;
      }

      cv::Mat H_new_global = matched_ref_H * matched_H;
      BlendResult b = warp_and_blend(panorama_, pano_mask_, frame_small, H_new_global);
      if (b.info.rfind("No overlap", 0) == 0) {
        RCLCPP_WARN(get_logger(), "%s: %s", fname.c_str(), b.info.c_str());
        continue;
      }

      cv::Mat T_shift = cv::Mat::eye(3, 3, CV_64F);
      T_shift.at<double>(0, 2) = b.tx;
      T_shift.at<double>(1, 2) = b.ty;
      panorama_ = b.panorama;
      pano_mask_ = b.mask;
      H_global_ = T_shift * H_new_global;

      for (auto & kf : keyframes) {
        kf.second = T_shift * kf.second;
      }
      keyframes.emplace_back(frame_small.clone(), H_global_.clone());
      ++frames_stitched_;
      RCLCPP_INFO(
        get_logger(), "  canvas: %dx%d  %s", panorama_.cols, panorama_.rows, b.info.c_str());
    }
  }

  void finish()
  {
    if (panorama_.empty()) {
      RCLCPP_ERROR(get_logger(), "No panorama created!");
      send_back("Mapping failed: no panorama created");
      return;
    }
    cv::Mat gray, mask;
    cv::cvtColor(panorama_, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY);
    if (cv::countNonZero(mask) > 0) {
      cv::Mat coords;
      cv::findNonZero(mask, coords);
      cv::Rect bbox = cv::boundingRect(coords);
      panorama_ = panorama_(bbox);
    } else {
      RCLCPP_WARN(get_logger(), "Panorama looks empty (all black); skipping crop.");
    }

    const std::string final_path = (fs::path(save_dir_) / "final_panorama.jpg").string();
    cv::imwrite(final_path, panorama_);

    RCLCPP_INFO(get_logger(), "FINAL RESULTS:");
    RCLCPP_INFO(get_logger(), "  Frames processed: %d", frames_processed_);
    RCLCPP_INFO(get_logger(), "  Frames stitched: %d", frames_stitched_);
    RCLCPP_INFO(get_logger(), "  Final size: %dx%d", panorama_.cols, panorama_.rows);
    RCLCPP_INFO(get_logger(), "  Saved to: %s", final_path.c_str());
    send_back("Mapping complete. Panorama saved to " + final_path);
  }

  void send_back(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.severity = 5;  // NOTICE
    msg.text = text;
    status_pub_->publish(msg);
  }

  bool use_sift_{false};
  double downscale_{0.5};
  int max_frames_{150};
  int min_matches_{6};
  double ratio_test_{0.8};
  double ransac_thresh_{3.0};

  std::string camera_feed_path_;
  std::string save_dir_;
  std::vector<std::string> image_files_;

  cv::Mat panorama_;
  cv::Mat pano_mask_;
  cv::Mat H_global_;
  int frames_processed_{0};
  int frames_stitched_{0};

  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
