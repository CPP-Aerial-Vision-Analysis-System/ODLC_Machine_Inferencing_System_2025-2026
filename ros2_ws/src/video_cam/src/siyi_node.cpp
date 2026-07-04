// video_cam: SIYI A8 Mini camera driver (C++ port of video_cam/siyi_node.py)
//
// Drives a SIYI A8 Mini over its UDP SDK (shutter, gimbal, zoom, focus) and HTTP
// media API (list + download), saving captures to video_cam/mapping_photos under
// a "<lat> , <lon>.jpg" name that detection/new_od parses back into GPS. Mirrors
// the Python node: /camera/trigger fires a capture, altitude gates the camera,
// and /camera/command runs gimbal/zoom/etc. commands.
//
// Ports camera_interface.py, storage_manager.py, pipeline_orchestrator.py and
// siyi_node.py into one node. The Python storage_manager/logging_utils/config
// modules are retained (as Python) because the detection package imports them.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/statvfs.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/status_text.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;
namespace fs = std::filesystem;

// ============================ config constants ==============================
namespace cfg
{
constexpr const char * CAMERA_IP = "192.168.144.25";
constexpr int CONTROL_PORT = 37260;
constexpr int MEDIA_PORT = 82;
constexpr double HTTP_TIMEOUT_SECONDS = 10.0;
constexpr double CAPTURE_TIMEOUT_SECONDS = 15.0;
constexpr double SDK_SOCKET_TIMEOUT_SECONDS = 2.0;
constexpr double SD_POLL_INTERVAL = 0.5;
constexpr double NODE_LOOP_PERIOD = 1.0;
constexpr double MIN_FREE_SPACE_MB = 50.0;
constexpr double REQUIRED_DOWNLOAD_SPACE_MB = 10.0;
constexpr int MIN_FILE_SIZE_BYTES = 1000;
constexpr const char * MAPPING_SUBDIR = "mapping_photos";
constexpr const char * ATOMIC_WRITE_SUFFIX = ".tmp";
constexpr double DEFAULT_MIN_ALTITUDE_AGL = -13.716;

// SDK command IDs
constexpr uint8_t CMD_AUTO_FOCUS = 0x04;
constexpr uint8_t CMD_MANUAL_ZOOM_AF = 0x05;
constexpr uint8_t CMD_MANUAL_FOCUS = 0x06;
constexpr uint8_t CMD_GIMBAL_ROTATE = 0x07;
constexpr uint8_t CMD_GIMBAL_CENTER = 0x08;
constexpr uint8_t CMD_FUNCTION_FEEDBACK = 0x0B;
constexpr uint8_t CMD_CAPTURE_RECORD = 0x0C;
constexpr uint8_t CMD_GIMBAL_ATTITUDE = 0x0D;
constexpr uint8_t CMD_SET_GIMBAL_ANGLES = 0x0E;
constexpr uint8_t CMD_ABSOLUTE_ZOOM_AF = 0x0F;
constexpr uint8_t CMD_SUPPORTED_ZOOM_RANGE = 0x16;
constexpr uint8_t CMD_CURRENT_ZOOM = 0x18;
constexpr uint8_t CMD_GIMBAL_MODE = 0x19;
constexpr uint8_t CMD_SINGLE_AXIS_CONTROL = 0x41;
constexpr uint8_t CMD_FORMAT_SD_CARD = 0x48;

// 4K/2.7K/1080P capture command bytes (full SDK frames, per config.py).
inline std::vector<uint8_t> capture_command(const std::string & res)
{
  if (res == "2.7K") {
    return {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x01, 0x35, 0xce};
  }
  if (res == "1080P") {
    return {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x02, 0x36, 0xce};
  }
  return {0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x34, 0xce};  // 4K
}

inline bool valid_resolution(const std::string & r)
{
  return r == "4K" || r == "2.7K" || r == "1080P";
}
}  // namespace cfg

// ============================ small helpers =================================
namespace
{
// CRC-16/CCITT (XMODEM), poly 0x1021, init 0x0000 — matches binascii.crc_hqx.
uint16_t crc16_ccitt(const uint8_t * data, size_t len)
{
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void put_u16le(std::vector<uint8_t> & v, uint16_t x)
{
  v.push_back(x & 0xFF);
  v.push_back((x >> 8) & 0xFF);
}

int16_t rd_i16le(const uint8_t * p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }
uint16_t rd_u16le(const uint8_t * p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

// Resolve <ros2_ws>/src by walking up from cwd; fall back to $HOME.
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
}  // namespace

// ============================ CameraInterface ===============================
struct SdkResponse
{
  uint8_t cmd_id{0};
  std::vector<uint8_t> data;
};

struct FileInfo
{
  std::string name;
  std::string url;
  long size{0};
};

class CameraInterface
{
public:
  CameraInterface(std::string ip, int ctrl_port, int media_port, double http_timeout, rclcpp::Logger logger)
  : camera_ip_(std::move(ip)), ctrl_port_(ctrl_port), media_port_(media_port),
    http_timeout_(http_timeout), logger_(logger)
  {
    base_url_ = "http://" + camera_ip_ + ":" + std::to_string(media_port_) +
      "/cgi-bin/media.cgi/api/v1";

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    set_socket_timeout(cfg::SDK_SOCKET_TIMEOUT_SECONDS);

    std::memset(&cam_addr_, 0, sizeof(cam_addr_));
    cam_addr_.sin_family = AF_INET;
    cam_addr_.sin_port = htons(static_cast<uint16_t>(ctrl_port_));
    inet_pton(AF_INET, camera_ip_.c_str(), &cam_addr_.sin_addr);

    curl_ = curl_easy_init();
  }

  ~CameraInterface() { close(); }

  void close()
  {
    if (sock_ >= 0) {
      ::close(sock_);
      sock_ = -1;
    }
    if (curl_) {
      curl_easy_cleanup(curl_);
      curl_ = nullptr;
    }
  }

  // ---- SDK (UDP) commands ----
  bool send_capture_command(const std::string & resolution)
  {
    std::string res = resolution;
    if (res != "4K") {
      RCLCPP_WARN(logger_, "Resolution %s not verified - using 4K for safety", res.c_str());
      res = "4K";
    }
    auto cmd = cfg::capture_command(res);
    sendto(sock_, cmd.data(), cmd.size(), 0, reinterpret_cast<sockaddr *>(&cam_addr_), sizeof(cam_addr_));
    auto fb = receive_packet({cfg::CMD_CAPTURE_RECORD, cfg::CMD_FUNCTION_FEEDBACK}, 0.8);
    if (!fb) {
      RCLCPP_WARN(logger_, "No capture feedback received (timeout), assuming success");
      return true;
    }
    if (fb->cmd_id == cfg::CMD_FUNCTION_FEEDBACK && !fb->data.empty()) {
      uint8_t info_type = fb->data[0];
      if (info_type == 1 || info_type == 4) {
        RCLCPP_ERROR(logger_, "Camera feedback indicates capture failure (%u)", info_type);
        return false;
      }
    }
    return true;
  }

  bool auto_focus(int x, int y)
  {
    x = std::clamp(x, 0, 65535);
    y = std::clamp(y, 0, 65535);
    std::vector<uint8_t> p;
    p.push_back(1);
    put_u16le(p, static_cast<uint16_t>(x));
    put_u16le(p, static_cast<uint16_t>(y));
    return status_ok(send_command(cfg::CMD_AUTO_FOCUS, p));
  }

  bool manual_zoom(int direction, double & out_zoom)
  {
    std::vector<uint8_t> p{static_cast<uint8_t>(static_cast<int8_t>(direction))};
    auto r = send_command(cfg::CMD_MANUAL_ZOOM_AF, p);
    if (!r || r->data.size() < 2) {
      return false;
    }
    out_zoom = rd_u16le(r->data.data()) / 10.0;
    return true;
  }

  bool manual_focus(int direction)
  {
    std::vector<uint8_t> p{static_cast<uint8_t>(static_cast<int8_t>(direction))};
    return status_ok(send_command(cfg::CMD_MANUAL_FOCUS, p));
  }

  bool absolute_zoom_autofocus(double zoom_multiple)
  {
    int zi = static_cast<int>(zoom_multiple);
    int zf = static_cast<int>(std::lround((zoom_multiple - zi) * 10));
    if (zf == 10) { zi += 1; zf = 0; }
    zi = std::clamp(zi, 0, 255);
    zf = std::clamp(zf, 0, 9);
    std::vector<uint8_t> p{static_cast<uint8_t>(zi), static_cast<uint8_t>(zf)};
    return status_ok(send_command(cfg::CMD_ABSOLUTE_ZOOM_AF, p));
  }

  bool get_supported_zoom_range(double & out)
  {
    auto r = send_command(cfg::CMD_SUPPORTED_ZOOM_RANGE, {});
    if (!r || r->data.size() < 2) {
      return false;
    }
    out = r->data[0] + r->data[1] / 10.0;
    return true;
  }

  bool get_current_zoom_magnification(double & out)
  {
    auto r = send_command(cfg::CMD_CURRENT_ZOOM, {});
    if (!r || r->data.size() < 2) {
      return false;
    }
    out = r->data[0] + r->data[1] / 10.0;
    return true;
  }

  bool rotate_gimbal(int yaw_speed, int pitch_speed)
  {
    yaw_speed = std::clamp(yaw_speed, -100, 100);
    pitch_speed = std::clamp(pitch_speed, -100, 100);
    std::vector<uint8_t> p{
      static_cast<uint8_t>(static_cast<int8_t>(yaw_speed)),
      static_cast<uint8_t>(static_cast<int8_t>(pitch_speed))};
    return status_ok(send_command(cfg::CMD_GIMBAL_ROTATE, p));
  }

  bool stop_gimbal_rotation() { return rotate_gimbal(0, 0); }

  bool center_gimbal() { return status_ok(send_command(cfg::CMD_GIMBAL_CENTER, {1})); }

  bool request_gimbal_attitude()
  {
    auto r = send_command(cfg::CMD_GIMBAL_ATTITUDE, {});
    return r && r->data.size() >= 12;
  }

  bool set_gimbal_angles(double yaw_deg, double pitch_deg)
  {
    int16_t yaw_raw = static_cast<int16_t>(std::lround(yaw_deg * 10.0));
    int16_t pitch_raw = static_cast<int16_t>(std::lround(pitch_deg * 10.0));
    std::vector<uint8_t> p;
    put_u16le(p, static_cast<uint16_t>(yaw_raw));
    put_u16le(p, static_cast<uint16_t>(pitch_raw));
    auto r = send_command(cfg::CMD_SET_GIMBAL_ANGLES, p);
    return r && r->data.size() >= 6;
  }

  bool set_single_axis_angle(const std::string & axis, double angle_deg)
  {
    uint8_t axis_flag;
    if (to_lower(axis) == "yaw") {
      axis_flag = 0;
    } else if (to_lower(axis) == "pitch") {
      axis_flag = 1;
    } else {
      throw std::invalid_argument("axis must be 'yaw' or 'pitch'");
    }
    int16_t angle_raw = static_cast<int16_t>(std::lround(angle_deg * 10.0));
    std::vector<uint8_t> p;
    put_u16le(p, static_cast<uint16_t>(angle_raw));
    p.push_back(axis_flag);
    auto r = send_command(
      cfg::CMD_SINGLE_AXIS_CONTROL, p, true, {cfg::CMD_SINGLE_AXIS_CONTROL, cfg::CMD_SET_GIMBAL_ANGLES});
    return r && r->data.size() >= 6;
  }

  bool get_gimbal_mode(std::string & out)
  {
    auto r = send_command(cfg::CMD_GIMBAL_MODE, {});
    if (!r || r->data.empty()) {
      return false;
    }
    switch (r->data[0]) {
      case 0: out = "lock"; break;
      case 1: out = "follow"; break;
      case 2: out = "fpv"; break;
      default: out = "unknown(" + std::to_string(r->data[0]) + ")";
    }
    return true;
  }

  bool set_gimbal_motion_mode(const std::string & mode)
  {
    uint8_t func_type;
    const std::string m = to_lower(mode);
    if (m == "lock") {
      func_type = 3;
    } else if (m == "follow") {
      func_type = 4;
    } else if (m == "fpv") {
      func_type = 5;
    } else {
      throw std::invalid_argument("mode must be one of: lock, follow, fpv");
    }
    send_command(cfg::CMD_CAPTURE_RECORD, {func_type}, false);
    auto fb = receive_packet({cfg::CMD_FUNCTION_FEEDBACK}, 0.4);
    if (fb && !fb->data.empty()) {
      uint8_t info_type = fb->data[0];
      if (info_type == 1 || info_type == 4) {
        return false;
      }
    }
    return true;
  }

  bool format_sd_card()
  {
    send_command(cfg::CMD_FORMAT_SD_CARD, {}, false);
    return true;
  }

  bool ping()
  {
    auto r = send_command(cfg::CMD_GIMBAL_ATTITUDE, {}, true, {cfg::CMD_GIMBAL_ATTITUDE}, 1.0);
    return static_cast<bool>(r);
  }

  // ---- HTTP media API ----
  void initialize_sd_card()
  {
    RCLCPP_INFO(logger_, "Initializing SD card...");
    auto dirs = get_directories();
    if (!dirs.empty()) {
      current_photo_dir_ = dirs.back();
      RCLCPP_INFO(logger_, "Photo directory: %s", current_photo_dir_.c_str());
      if (last_photo_count_ == 0) {
        int count = get_media_count(current_photo_dir_);
        if (count >= 0) {
          last_photo_count_ = count;
        }
        load_existing_sd_files();
      }
    } else {
      current_photo_dir_ = "A:/DCIM/100MEDIA";
    }
  }

  bool get_new_file(FileInfo & out)
  {
    if (current_photo_dir_.empty()) {
      return false;
    }
    auto files = get_media_list(current_photo_dir_);
    const int current_count = static_cast<int>(files.size());
    if (current_count > last_photo_count_) {
      std::lock_guard<std::mutex> lk(download_mutex_);
      for (auto it = files.rbegin(); it != files.rend(); ++it) {
        if (!it->name.empty() && downloaded_files_.count(it->name) == 0) {
          last_photo_count_ = current_count;
          out = *it;
          return true;
        }
      }
    }
    return false;
  }

  std::vector<uint8_t> download_image(std::string file_url)
  {
    // Point the URL at the configured camera IP if it hardcodes the default.
    const std::string def = "192.168.144.25";
    size_t pos;
    while ((pos = file_url.find(def)) != std::string::npos) {
      file_url.replace(pos, def.size(), camera_ip_);
    }
    long status = 0;
    std::string body = http_get_raw(file_url, status);
    if (status != 200) {
      RCLCPP_ERROR(logger_, "Download failed: HTTP %ld", status);
      return {};
    }
    return std::vector<uint8_t>(body.begin(), body.end());
  }

  cv::Mat decode_image(const std::vector<uint8_t> & bytes)
  {
    if (bytes.empty()) {
      return {};
    }
    return cv::imdecode(bytes, cv::IMREAD_COLOR);
  }

  int get_downloaded_count()
  {
    std::lock_guard<std::mutex> lk(download_mutex_);
    return static_cast<int>(downloaded_files_.size());
  }

  const std::string & current_photo_dir() const { return current_photo_dir_; }

private:
  void set_socket_timeout(double seconds)
  {
    timeval tv{};
    tv.tv_sec = static_cast<long>(seconds);
    tv.tv_usec = static_cast<long>((seconds - tv.tv_sec) * 1e6);
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  std::vector<uint8_t> build_packet(uint8_t cmd_id, const std::vector<uint8_t> & data, bool need_ack)
  {
    std::vector<uint8_t> pkt{0x55, 0x66};
    pkt.push_back(need_ack ? 0x01 : 0x00);
    put_u16le(pkt, static_cast<uint16_t>(data.size()));
    put_u16le(pkt, static_cast<uint16_t>(seq_));
    seq_ = (seq_ + 1) & 0xFFFF;
    pkt.push_back(cmd_id);
    pkt.insert(pkt.end(), data.begin(), data.end());
    uint16_t crc = crc16_ccitt(pkt.data(), pkt.size());
    put_u16le(pkt, crc);
    return pkt;
  }

  std::optional<SdkResponse> parse_packet(const uint8_t * p, size_t len)
  {
    if (len < 10 || p[0] != 0x55 || p[1] != 0x66) {
      return std::nullopt;
    }
    uint16_t data_len = rd_u16le(p + 3);
    uint8_t cmd_id = p[7];
    size_t frame_len = 8 + data_len + 2;
    if (len < frame_len) {
      return std::nullopt;
    }
    uint16_t crc_recv = rd_u16le(p + 8 + data_len);
    uint16_t crc_calc = crc16_ccitt(p, 8 + data_len);
    if (crc_recv != crc_calc) {
      RCLCPP_WARN(logger_, "SDK CRC mismatch: recv=%04x calc=%04x", crc_recv, crc_calc);
      return std::nullopt;
    }
    SdkResponse r;
    r.cmd_id = cmd_id;
    r.data.assign(p + 8, p + 8 + data_len);
    return r;
  }

  std::optional<SdkResponse> receive_packet(
    const std::set<uint8_t> & expected, double timeout, int max_frames = 8)
  {
    set_socket_timeout(timeout);
    uint8_t buf[2048];
    for (int i = 0; i < max_frames; ++i) {
      ssize_t n = recvfrom(sock_, buf, sizeof(buf), 0, nullptr, nullptr);
      if (n <= 0) {
        break;
      }
      auto parsed = parse_packet(buf, static_cast<size_t>(n));
      if (!parsed) {
        continue;
      }
      if (!expected.empty() && expected.count(parsed->cmd_id) == 0) {
        continue;
      }
      set_socket_timeout(cfg::SDK_SOCKET_TIMEOUT_SECONDS);
      return parsed;
    }
    set_socket_timeout(cfg::SDK_SOCKET_TIMEOUT_SECONDS);
    return std::nullopt;
  }

  std::optional<SdkResponse> send_command(
    uint8_t cmd_id, const std::vector<uint8_t> & data, bool expect_ack = true,
    std::set<uint8_t> expected = {}, double timeout = cfg::SDK_SOCKET_TIMEOUT_SECONDS)
  {
    auto pkt = build_packet(cmd_id, data, expect_ack);
    sendto(sock_, pkt.data(), pkt.size(), 0, reinterpret_cast<sockaddr *>(&cam_addr_), sizeof(cam_addr_));
    if (!expect_ack) {
      return std::nullopt;
    }
    if (expected.empty()) {
      expected = {cmd_id};
    }
    return receive_packet(expected, timeout);
  }

  static bool status_ok(const std::optional<SdkResponse> & r, uint8_t success_value = 1)
  {
    if (!r) {
      return false;
    }
    if (r->data.empty()) {
      return true;
    }
    return r->data[0] == success_value;
  }

  // ---- HTTP helpers (libcurl) ----
  static size_t write_cb(char * ptr, size_t size, size_t nmemb, void * userdata)
  {
    auto * out = static_cast<std::string *>(userdata);
    out->append(ptr, size * nmemb);
    return size * nmemb;
  }

  std::string http_get_raw(const std::string & url, long & status_code)
  {
    std::string body;
    status_code = 0;
    if (!curl_) {
      return body;
    }
    curl_easy_reset(curl_);
    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &body);
    curl_easy_setopt(curl_, CURLOPT_TIMEOUT, static_cast<long>(http_timeout_));
    curl_easy_setopt(curl_, CURLOPT_USERAGENT, "SIYI-ROS-Client/1.0");
    if (curl_easy_perform(curl_) == CURLE_OK) {
      curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &status_code);
    }
    return body;
  }

  std::string url_escape(const std::string & s)
  {
    char * esc = curl_easy_escape(curl_, s.c_str(), static_cast<int>(s.size()));
    std::string out = esc ? esc : s;
    if (esc) {
      curl_free(esc);
    }
    return out;
  }

  json http_get_json(const std::string & url)
  {
    long status = 0;
    std::string body = http_get_raw(url, status);
    if (status != 200 || body.empty()) {
      return json::object();
    }
    try {
      return json::parse(body);
    } catch (const std::exception &) {
      return json::object();
    }
  }

  std::vector<std::string> get_directories()
  {
    std::vector<std::string> out;
    json data = http_get_json(base_url_ + "/getdirectories?media_type=0");
    if (data.value("success", false)) {
      for (const auto & d : data["data"].value("directories", json::array())) {
        if (d.contains("path")) {
          out.push_back(d["path"].get<std::string>());
        }
      }
    }
    return out;
  }

  std::vector<FileInfo> get_media_list(const std::string & dir_path, int start = 0, int count = 9999)
  {
    std::vector<FileInfo> out;
    std::string url = base_url_ + "/getmedialist?media_type=0&path=" + url_escape(dir_path) +
      "&start=" + std::to_string(start) + "&count=" + std::to_string(count);
    json data = http_get_json(url);
    if (data.value("success", false)) {
      for (const auto & f : data["data"].value("list", json::array())) {
        FileInfo fi;
        fi.name = f.value("name", "");
        fi.url = f.value("url", "");
        fi.size = f.value("size", 0L);
        out.push_back(fi);
      }
    }
    return out;
  }

  int get_media_count(const std::string & dir_path)
  {
    std::string url = base_url_ + "/getmedialist?media_type=0&path=" + url_escape(dir_path) +
      "&start=0&count=1";
    json data = http_get_json(url);
    if (data.value("success", false)) {
      return data["data"].value("total", 0);
    }
    return -1;
  }

  void load_existing_sd_files()
  {
    auto files = get_media_list(current_photo_dir_);
    std::lock_guard<std::mutex> lk(download_mutex_);
    for (const auto & f : files) {
      if (!f.name.empty()) {
        downloaded_files_.insert(f.name);
      }
    }
    RCLCPP_INFO(logger_, "Marked %zu existing files as seen", files.size());
  }

  std::string camera_ip_;
  int ctrl_port_;
  int media_port_;
  double http_timeout_;
  rclcpp::Logger logger_;
  std::string base_url_;
  int sock_{-1};
  sockaddr_in cam_addr_{};
  CURL * curl_{nullptr};
  int seq_{0};

  std::string current_photo_dir_;
  int last_photo_count_{0};
  std::set<std::string> downloaded_files_;
  std::mutex download_mutex_;
};

// ============================ StorageManager ================================
class StorageManager
{
public:
  StorageManager(const std::string & workspace_root, rclcpp::Logger logger)
  : logger_(logger)
  {
    std::error_code ec;
    fs::create_directories(workspace_root, ec);
    mapping_dir_ = (fs::path(workspace_root) / cfg::MAPPING_SUBDIR).string();
    fs::create_directories(mapping_dir_, ec);
  }

  const std::string & get_mapping_dir() const { return mapping_dir_; }

  double get_free_space_mb() const
  {
    struct statvfs st;
    if (statvfs(mapping_dir_.c_str(), &st) != 0) {
      return 0.0;
    }
    return static_cast<double>(st.f_bavail) * st.f_frsize / (1024.0 * 1024.0);
  }

  bool check_disk_space(double required_mb) const
  {
    double free_mb = get_free_space_mb();
    if (free_mb < required_mb) {
      RCLCPP_ERROR(logger_, "Disk space critical: %.1fMB free (need %.1fMB)", free_mb, required_mb);
      return false;
    }
    return true;
  }

  bool verify_image(const cv::Mat & img) const
  {
    if (img.empty()) {
      return false;
    }
    if (img.rows < 100 || img.cols < 100) {
      RCLCPP_ERROR(logger_, "Image too small: %dx%d", img.cols, img.rows);
      return false;
    }
    return true;
  }

  // Returns the saved absolute path, or empty on failure.
  std::string save_image(const std::string & filename, const cv::Mat & img)
  {
    const std::string filepath = (fs::path(mapping_dir_) / filename).string();
    if (!atomic_write(filepath, img)) {
      RCLCPP_ERROR(logger_, "Failed to write: %s", filepath.c_str());
      return "";
    }
    std::error_code ec;
    if (!fs::exists(filepath) || fs::file_size(filepath, ec) < cfg::MIN_FILE_SIZE_BYTES) {
      RCLCPP_ERROR(logger_, "File verification failed: %s", filepath.c_str());
      return "";
    }
    RCLCPP_INFO(logger_, "Saved: %s", filepath.c_str());
    return filepath;
  }

private:
  bool atomic_write(const std::string & filepath, const cv::Mat & img)
  {
    fs::path p(filepath);
    const std::string tmp = (p.parent_path() /
      (p.stem().string() + cfg::ATOMIC_WRITE_SUFFIX + p.extension().string())).string();
    std::error_code ec;
    if (!cv::imwrite(tmp, img)) {
      return false;
    }
    if (!fs::exists(tmp) || fs::file_size(tmp, ec) < cfg::MIN_FILE_SIZE_BYTES) {
      fs::remove(tmp, ec);
      return false;
    }
    fs::rename(tmp, filepath, ec);
    if (ec) {
      fs::remove(tmp, ec);
      return false;
    }
    return true;
  }

  rclcpp::Logger logger_;
  std::string mapping_dir_;
};

// ============================ PipelineOrchestrator ==========================
class PipelineOrchestrator
{
public:
  PipelineOrchestrator(CameraInterface * camera, StorageManager * storage, bool rotate_180, rclcpp::Logger logger)
  : camera_(camera), storage_(storage), rotate_180_(rotate_180), logger_(logger) {}

  bool is_busy() const { return busy_.load(); }
  void set_resolution(const std::string & r)
  {
    resolution_ = r;
    RCLCPP_INFO(logger_, "Resolution set to: %s", r.c_str());
  }
  void initialize_sd_card() { camera_->initialize_sd_card(); }
  int photo_count() const { return photo_count_; }
  const std::string & resolution() const { return resolution_; }

  // Phases 1+2: shutter + SD index. Returns false on failure.
  bool capture_and_index(FileInfo & out)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(logger_, "capture_and_index: another capture mid-shutter, dropping trigger");
      return false;
    }
    struct Guard { std::atomic<bool> & b; ~Guard() { b.store(false); } } guard{busy_};

    if (!camera_->send_capture_command(resolution_)) {
      RCLCPP_ERROR(logger_, "Phase 1 failed: capture command rejected");
      return false;
    }
    ++photo_count_;
    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

    const double start = now_s();
    while (now_s() - start < cfg::CAPTURE_TIMEOUT_SECONDS) {
      if (camera_->get_new_file(out)) {
        RCLCPP_INFO(logger_, "Found new image: %s", out.name.c_str());
        return true;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(cfg::SD_POLL_INTERVAL));
    }
    RCLCPP_ERROR(logger_, "Phase 2 failed: timeout, new image not found on SD card");
    return false;
  }

  // Phase 3: download + decode + save. Returns saved path (and image) or empty.
  bool download_and_save(const FileInfo & file_info, const std::string & filename_override, std::string & saved_path, cv::Mat & img_out)
  {
    if (file_info.name.empty() || file_info.url.empty()) {
      return false;
    }
    const std::string save_name = filename_override.empty() ? file_info.name : filename_override;

    double required_mb = std::max(
      cfg::REQUIRED_DOWNLOAD_SPACE_MB, (file_info.size * 2.0) / (1024.0 * 1024.0));
    if (!storage_->check_disk_space(required_mb)) {
      RCLCPP_ERROR(logger_, "Insufficient disk space (need %.1fMB)", required_mb);
      return false;
    }

    auto bytes = camera_->download_image(file_info.url);
    if (bytes.empty()) {
      return false;
    }
    cv::Mat img = camera_->decode_image(bytes);
    if (img.empty()) {
      return false;
    }
    if (rotate_180_) {
      cv::rotate(img, img, cv::ROTATE_180);
    }
    if (!storage_->verify_image(img)) {
      return false;
    }
    saved_path = storage_->save_image(save_name, img);
    if (saved_path.empty()) {
      return false;
    }
    img_out = img;
    return true;
  }

private:
  static double now_s()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  CameraInterface * camera_;
  StorageManager * storage_;
  bool rotate_180_;
  rclcpp::Logger logger_;
  std::atomic<bool> busy_{false};
  int photo_count_{0};
  std::string resolution_{"4K"};
};

// ============================ SIYINode ======================================
class SIYINode : public rclcpp::Node
{
public:
  SIYINode()
  : Node("siyi")
  {
    use_real_camera_ = declare_parameter<bool>("use_real_camera", true);
    altitude_threshold_ = declare_parameter<double>("min_altitude_agl", cfg::DEFAULT_MIN_ALTITUDE_AGL);
    camera_ip_ = declare_parameter<std::string>("camera_ip", cfg::CAMERA_IP);
    ctrl_port_ = declare_parameter<int>("ctrl_port", cfg::CONTROL_PORT);
    media_port_ = declare_parameter<int>("media_port", cfg::MEDIA_PORT);
    http_timeout_ = declare_parameter<double>("http_timeout_sec", cfg::HTTP_TIMEOUT_SECONDS);
    resolution_ = declare_parameter<std::string>("resolution", "4K");
    rotate_180_ = declare_parameter<bool>("rotate_180", true);
    if (!cfg::valid_resolution(resolution_)) {
      RCLCPP_WARN(get_logger(), "Invalid resolution '%s', falling back to '4K'", resolution_.c_str());
      resolution_ = "4K";
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    status_pub_ = create_publisher<mavros_msgs::msg::StatusText>("/mavros/statustext/send", 10);
    camera_status_pub_ = create_publisher<std_msgs::msg::String>("/camera/status", 10);
    disk_status_pub_ = create_publisher<std_msgs::msg::Float64>("/camera/disk_free_mb", 10);

    trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/camera/trigger", 10, [this](std_msgs::msg::Bool::SharedPtr m) { camera_trigger_callback(m); });
    altitude_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/rel_alt", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64::SharedPtr m) { altitude_callback(m); });
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr m) { latest_gps_ = m; });
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/camera/command", 10, [this](std_msgs::msg::String::SharedPtr m) { camera_command_callback(m); });

    const std::string workspace_root = (ros2_ws_src() / "video_cam").string();
    std::error_code ec;
    fs::create_directories(workspace_root, ec);
    storage_ = std::make_unique<StorageManager>(workspace_root, get_logger());

    if (use_real_camera_) {
      camera_ = std::make_unique<CameraInterface>(
        camera_ip_, ctrl_port_, media_port_, http_timeout_, get_logger());
      pipeline_ = std::make_unique<PipelineOrchestrator>(
        camera_.get(), storage_.get(), rotate_180_, get_logger());
      pipeline_->set_resolution(resolution_);
      pipeline_->initialize_sd_card();
    } else {
      RCLCPP_INFO(get_logger(), "Simulation mode not supported in C++ port; running without camera.");
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(cfg::NODE_LOOP_PERIOD), [this]() { pipeline_loop(); });

    RCLCPP_INFO(
      get_logger(), "SIYI pipeline initialized (%s)", use_real_camera_ ? "REAL CAMERA" : "NO CAMERA");
  }

  void shutdown()
  {
    if (camera_) {
      camera_->close();
    }
  }

private:
  void pipeline_loop()
  {
    if (!camera_enabled_) {
      return;
    }
    if (capture_requested_.exchange(false)) {
      handle_capture_request();
    }
    publish_disk_status();
  }

  void handle_capture_request()
  {
    if (!use_real_camera_ || !pipeline_) {
      return;
    }
    if (pipeline_->is_busy()) {
      RCLCPP_WARN(get_logger(), "Previous capture still in shutter/index phase, skipping request");
      return;
    }
    // Non-blocking: run the (blocking) capture pipeline on a detached thread.
    std::thread([this]() { execute_real_camera_capture(); }).detach();
  }

  void execute_real_camera_capture()
  {
    try {
      const std::string gps_filename = generate_gps_filename();
      if (gps_filename.empty()) {
        RCLCPP_WARN(get_logger(), "No valid GPS fix; using the SD name");
        send_status("WARN: No GPS fix - image will not have lat/lon name");
      }

      FileInfo file_info;
      {
        std::lock_guard<std::mutex> lk(camera_control_mutex_);
        if (!pipeline_->capture_and_index(file_info)) {
          send_status("FAILED: Capture shutter/index error");
          publish_camera_status("FAILURE: Capture shutter/index error");
          return;
        }
      }

      std::string saved_path;
      cv::Mat img;
      if (!pipeline_->download_and_save(file_info, gps_filename, saved_path, img)) {
        send_status("FAILED: Capture download error");
        publish_camera_status("FAILURE: Capture download error");
        return;
      }

      send_status(
        "SUCCESS: Captured " + pipeline_->resolution() + " image #" +
        std::to_string(pipeline_->photo_count()));
      publish_captured_image(img);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Pipeline error: %s", e.what());
      send_status(std::string("FAILED: ") + e.what());
    }
  }

  // ---- camera command topic ----
  void camera_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string command, parameter;
    try {
      json data = json::parse(msg->data);
      command = data.value("command", "");
      parameter = data.value("parameter", "");
    } catch (const std::exception &) {
      size_t sp = msg->data.find(' ');
      if (sp == std::string::npos) {
        command = msg->data;
      } else {
        command = msg->data.substr(0, sp);
        parameter = msg->data.substr(sp + 1);
      }
    }

    try {
      std::lock_guard<std::mutex> lk(camera_control_mutex_);
      bool ok = execute_camera_command(command, parameter);
      if (ok) {
        publish_camera_status("CMD SUCCESS: " + command);
      } else {
        publish_camera_status("CMD FAILED: " + command);
        RCLCPP_ERROR(get_logger(), "Camera command failed: %s", command.c_str());
      }
    } catch (const std::exception & e) {
      publish_camera_status("CMD ERROR: " + msg->data);
      RCLCPP_ERROR(get_logger(), "Camera command error for %s: %s", msg->data.c_str(), e.what());
    }
  }

  static std::vector<std::string> split_csv(const std::string & p, size_t expected)
  {
    std::vector<std::string> parts;
    std::stringstream ss(p);
    std::string item;
    while (std::getline(ss, item, ',')) {
      // trim
      size_t a = item.find_first_not_of(" \t");
      size_t b = item.find_last_not_of(" \t");
      if (a != std::string::npos) {
        parts.push_back(item.substr(a, b - a + 1));
      }
    }
    if (parts.size() != expected) {
      throw std::invalid_argument("Expected " + std::to_string(expected) + " CSV values");
    }
    return parts;
  }

  bool execute_camera_command(const std::string & command, const std::string & parameter)
  {
    if (!use_real_camera_ || !camera_) {
      throw std::runtime_error("Camera command requires use_real_camera=true");
    }
    std::string cmd = to_lower(command);
    std::replace(cmd.begin(), cmd.end(), '-', '_');
    std::string param = parameter;

    if (cmd == "capture") {
      std::string res = "4K";
      if (!param.empty()) {
        res = param;
        std::transform(res.begin(), res.end(), res.begin(), ::toupper);
        if (!cfg::valid_resolution(res)) {
          throw std::invalid_argument("Invalid resolution '" + res + "'");
        }
      }
      if (pipeline_) {
        if (pipeline_->is_busy()) {
          return false;
        }
        pipeline_->set_resolution(res);
      }
      capture_requested_.store(true);
      return true;
    }

    if (pipeline_ && pipeline_->is_busy()) {
      return false;
    }

    if (cmd == "autofocus") {
      int x = 0, y = 0;
      if (!param.empty()) {
        auto p = split_csv(param, 2);
        x = std::stoi(p[0]);
        y = std::stoi(p[1]);
      }
      return camera_->auto_focus(x, y);
    }
    if (cmd == "zoom_manual") {
      int dir;
      if (param == "in") dir = 1;
      else if (param == "out") dir = -1;
      else if (param == "stop") dir = 0;
      else throw std::invalid_argument("zoom_manual parameter must be one of: in, out, stop");
      double z;
      return camera_->manual_zoom(dir, z);
    }
    if (cmd == "zoom_absolute" || cmd == "zoom_auto") {
      if (param.empty()) {
        throw std::invalid_argument(cmd + " requires a zoom multiple");
      }
      return camera_->absolute_zoom_autofocus(std::stod(param));
    }
    if (cmd == "zoom_range") {
      double v;
      return camera_->get_supported_zoom_range(v);
    }
    if (cmd == "zoom_current") {
      double v;
      return camera_->get_current_zoom_magnification(v);
    }
    if (cmd == "focus_manual") {
      int dir;
      if (param == "far") dir = 1;
      else if (param == "near") dir = -1;
      else if (param == "stop") dir = 0;
      else throw std::invalid_argument("focus_manual parameter must be one of: far, near, stop");
      return camera_->manual_focus(dir);
    }
    if (cmd == "gimbal_rotate") {
      auto p = split_csv(param, 2);
      return camera_->rotate_gimbal(std::stoi(p[0]), std::stoi(p[1]));
    }
    if (cmd == "gimbal_stop") {
      return camera_->stop_gimbal_rotation();
    }
    if (cmd == "gimbal_center") {
      return camera_->center_gimbal();
    }
    if (cmd == "gimbal_attitude") {
      return camera_->request_gimbal_attitude();
    }
    if (cmd == "gimbal_set_angles") {
      auto p = split_csv(param, 2);
      return camera_->set_gimbal_angles(std::stod(p[0]), std::stod(p[1]));
    }
    if (cmd == "gimbal_set_axis") {
      auto p = split_csv(param, 2);
      return camera_->set_single_axis_angle(p[0], std::stod(p[1]));
    }
    if (cmd == "gimbal_mode_get") {
      std::string m;
      return camera_->get_gimbal_mode(m);
    }
    if (cmd == "gimbal_mode_set") {
      if (param.empty()) {
        throw std::invalid_argument("gimbal_mode_set requires one of: lock, follow, fpv");
      }
      return camera_->set_gimbal_motion_mode(param);
    }
    if (cmd == "sd_format") {
      if (to_lower(param) != "yes") {
        throw std::invalid_argument("sd_format is destructive; pass parameter yes to continue");
      }
      return camera_->format_sd_card();
    }
    throw std::invalid_argument("Unsupported command: " + command);
  }

  std::string generate_gps_filename()
  {
    if (!latest_gps_) {
      return "";
    }
    double lat = latest_gps_->latitude;
    double lon = latest_gps_->longitude;
    if (lat == 0.0 && lon == 0.0) {
      RCLCPP_WARN(get_logger(), "Cached GPS fix is (0.0, 0.0) - treating as no fix");
      return "";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6f , %.6f.jpg", lat, lon);
    return buf;
  }

  void publish_captured_image(const cv::Mat & img)
  {
    if (img.empty()) {
      RCLCPP_WARN(get_logger(), "Nothing to publish on /image_raw");
      return;
    }
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = get_clock()->now();
    msg->header.frame_id = "camera_link";
    image_pub_->publish(*msg);
  }

  void publish_disk_status()
  {
    std_msgs::msg::Float64 msg;
    msg.data = storage_->get_free_space_mb();
    disk_status_pub_->publish(msg);
  }

  void camera_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_INFO(get_logger(), "Capture trigger received!");
      capture_requested_.store(true);
    }
  }

  void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double current_alt = msg->data;
    const bool was_enabled = camera_enabled_;
    if (current_alt >= altitude_threshold_) {
      camera_enabled_ = true;
      if (!was_enabled) {
        RCLCPP_INFO(get_logger(), "Altitude %.2fm >= threshold - Camera ENABLED", current_alt);
        send_status("Altitude threshold reached - Camera enabled");
      }
    } else {
      camera_enabled_ = false;
      if (was_enabled) {
        RCLCPP_INFO(get_logger(), "Altitude %.2fm < threshold - Camera DISABLED", current_alt);
        send_status("Below altitude threshold - Camera disabled");
      }
    }
  }

  void send_status(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.severity = 6;
    msg.text = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Status: %s", text.c_str());
  }

  void publish_camera_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    camera_status_pub_->publish(msg);
  }

  // params
  bool use_real_camera_{true};
  double altitude_threshold_{cfg::DEFAULT_MIN_ALTITUDE_AGL};
  std::string camera_ip_;
  int ctrl_port_{cfg::CONTROL_PORT};
  int media_port_{cfg::MEDIA_PORT};
  double http_timeout_{cfg::HTTP_TIMEOUT_SECONDS};
  std::string resolution_{"4K"};
  bool rotate_180_{true};

  // state
  std::atomic<bool> camera_enabled_{true};
  std::atomic<bool> capture_requested_{false};
  std::mutex camera_control_mutex_;
  sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_;

  std::unique_ptr<CameraInterface> camera_;
  std::unique_ptr<StorageManager> storage_;
  std::unique_ptr<PipelineOrchestrator> pipeline_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr disk_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  curl_global_init(CURL_GLOBAL_DEFAULT);
  auto node = std::make_shared<SIYINode>();
  rclcpp::spin(node);
  node->shutdown();
  rclcpp::shutdown();
  curl_global_cleanup();
  return 0;
}
