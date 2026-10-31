#include "zr10_ros2/zr10.h"
#include "zr10_ros2/media.h"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <curl/curl.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class ZR10Node final : public rclcpp::Node {
public:
  ZR10Node() : Node("zr10_camera"), camera_(
      declare_parameter<std::string>("camera_ip", "192.168.144.25"),
      declare_parameter<int>("camera_port", 37260)) {
    dry_run_ = declare_parameter<bool>("dry_run", true);
    RCLCPP_WARN(get_logger(), "ZR10 dry_run=%s; %s", dry_run_ ? "true" : "false",
      dry_run_ ? "camera commands will NOT be sent" : "LIVE camera commands enabled");
    // Single-threaded executor serializes ROS callbacks to the UDP driver.
    const auto service = [this](const std::string &name, std::function<bool()> command) {
      services_.push_back(create_service<std_srvs::srv::Trigger>(
        name, [this, name, command](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          try {
            response->success = dry_run_ ? true : command();
            response->message = dry_run_ ? "DRY RUN: no camera command sent" :
              (response->success ? "UDP packet sent (not camera ACK)" : "UDP send failed");
          } catch (const std::exception &e) {
            response->success = false;
            response->message = e.what();
          }
          RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), response->message.c_str());
        }));
    };
    service("take_photo", [this] { return camera_.takePhoto(); });
    service("autofocus", [this] { return camera_.autoFocus(); });
    service("toggle_recording", [this] { return camera_.toggleRecording(); });
    service("stop_zoom", [this] { const bool ok = dry_run_ || camera_.stopZoom(); if (ok) zoom_active_ = false; return ok; });
    service("stop_gimbal", [this] { const bool ok = dry_run_ || camera_.stopGimbal(); if (ok) gimbal_active_ = false; return ok; });
    service("center_gimbal", [this] { gimbal_active_ = false; return camera_.centerGimbal(); });

    zoom_sub_ = create_subscription<std_msgs::msg::Int8>(
      "zoom_direction", rclcpp::QoS(10), [this](std_msgs::msg::Int8::SharedPtr msg) {
        bool ok = false;
        switch (msg->data) {
          case 1: ok = dry_run_ || camera_.zoomIn(); break;
          case -1: ok = dry_run_ || camera_.zoomOut(); break;
          case 0: ok = dry_run_ || camera_.stopZoom(); break;
          default: RCLCPP_WARN(get_logger(), "zoom_direction must be -1, 0, or 1"); return;
        }
        if (ok) {
          zoom_active_ = msg->data != 0;
          zoom_last_ = std::chrono::steady_clock::now();
        } else RCLCPP_ERROR(get_logger(), "Failed to send zoom command");
      });

    gimbal_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "gimbal_speed", rclcpp::QoS(10), [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        // x = yaw, y = pitch; values are speed commands, not angles.
        if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
            msg->x < -100 || msg->x > 100 || msg->y < -100 || msg->y > 100) {
          RCLCPP_WARN(get_logger(), "Gimbal speeds must be finite and in [-100,100]");
          return;
        }
        const auto yaw = static_cast<int8_t>(std::lround(msg->x));
        const auto pitch = static_cast<int8_t>(std::lround(msg->y));
        if (dry_run_ || camera_.moveGimbal(yaw, pitch)) {
          gimbal_active_ = yaw != 0 || pitch != 0;
          gimbal_last_ = std::chrono::steady_clock::now();
        } else RCLCPP_ERROR(get_logger(), "Failed to send gimbal command");
      });

    const double timeout = declare_parameter<double>("motion_timeout_sec", 0.5);
    if (!(timeout > 0.0 && std::isfinite(timeout)))
      throw std::invalid_argument("motion_timeout_sec must be positive and finite");
    motion_timeout_ = std::chrono::duration<double>(timeout);
    watchdog_ = create_wall_timer(50ms, [this] {
      const auto now = std::chrono::steady_clock::now();
      if (zoom_active_ && now - zoom_last_ > motion_timeout_) {
        if (dry_run_ || camera_.stopZoom()) {
          zoom_active_ = false;
        } else {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "Zoom watchdog stop send failed; will retry");
        }
      }
      if (gimbal_active_ && now - gimbal_last_ > motion_timeout_) {
        if (dry_run_ || camera_.stopGimbal()) {
          gimbal_active_ = false;
        } else {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "Gimbal watchdog stop send failed; will retry");
        }
      }
    });

    if (declare_parameter<bool>("sync_photos", false) && !dry_run_) {
      MediaOptions options;
      options.base = declare_parameter<std::string>("media_base", "http://192.168.144.25:82/cgi-bin/media.cgi");
      options.directory = declare_parameter<std::string>("photo_directory", "photos");
      options.interval = declare_parameter<double>("sync_interval_sec", 2.0);
      options.timeout = declare_parameter<int>("media_timeout_sec", 15);
      if (options.interval <= 0 || options.timeout <= 0)
        throw std::invalid_argument("Media interval and timeout must be positive");
      if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK)
        throw std::runtime_error("curl_global_init failed");
      curl_initialized_ = true;
      media_thread_ = std::thread([this, options] { syncPhotos(options, media_stop_); });
    }
    RCLCPP_INFO(get_logger(), "ZR10 node started; UDP sends are not confirmed camera operations");
  }

  ~ZR10Node() override {
    media_stop_.store(true);
    if (media_thread_.joinable()) media_thread_.join();
    if (curl_initialized_) curl_global_cleanup();
    // Best-effort safety stop on orderly shutdown.
    if (!dry_run_) {
      camera_.stopZoom();
      camera_.stopGimbal();
    }
  }

private:
  ZR10 camera_;
  bool dry_run_{true};
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr zoom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::chrono::duration<double> motion_timeout_{0.5};
  std::chrono::steady_clock::time_point zoom_last_{}, gimbal_last_{};
  bool zoom_active_{false}, gimbal_active_{false};
  std::atomic<bool> media_stop_{false};
  std::thread media_thread_;
  bool curl_initialized_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ZR10Node>();
    rclcpp::spin(node); // Single-threaded executor: serialize camera callbacks.
    node.reset();
  } catch (const std::exception &e) {
    fprintf(stderr, "ZR10 node error: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
