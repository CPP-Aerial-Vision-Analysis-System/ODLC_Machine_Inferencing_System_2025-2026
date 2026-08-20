// siyi_wipe_sd.cpp
//
// Erases every image on the SIYI camera's SD card.
//
// The SIYI media HTTP API (getdirectories / getmedialist) is read-only -- it has
// no per-file delete. The only way to clear the card is SDK command 0x48,
// FORMAT SD CARD, sent as a UDP datagram to the camera's control port. That
// formats the WHOLE card: images, videos, everything. It cannot be undone.
//
// Two ways to run it:
//
//   1. One-shot from a shell (bench use):
//        ros2 run maincpp siyi_wipe_sd --now --confirm
//
//   2. As a node that Mission Planner can trigger (default):
//        ros2 run maincpp siyi_wipe_sd
//      Then from Mission Planner: Actions -> Set Servo, channel 12 -> 1900
//      (or a DO_SET_SERVO mission item on that channel). The node watches
//      /mavros/rc/out for a rising edge past the threshold and fires once.
//
// Safety interlocks in watch mode:
//   * refuses to fire while the vehicle is ARMED (require_disarmed, default true)
//   * rising-edge only -- holding the channel high will not re-trigger
//   * cooldown_sec between triggers
//   * every action is echoed to /mavros/statustext/send so it shows up in the
//     Mission Planner message pane
//
// Protocol (mirrors video_cam/camera_interface.py):
//   packet = 0x55 0x66 | ctrl(1) | data_len(2 LE) | seq(2 LE) | cmd_id(1) | data | crc16(2 LE)
//   crc = CRC-16/CCITT, poly 0x1021, init 0x0000  (Python binascii.crc_hqx)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/status_text.hpp"

namespace
{

constexpr uint8_t kStx0 = 0x55;
constexpr uint8_t kStx1 = 0x66;
constexpr uint8_t kCmdFormatSdCard = 0x48;

// CRC-16/CCITT (XMODEM): poly 0x1021, init 0x0000, no reflection, no final xor.
uint16_t crc16_ccitt(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0x0000;
  for (uint8_t byte : data) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<uint8_t> build_sdk_packet(uint8_t cmd_id,
                                      const std::vector<uint8_t> & data,
                                      bool need_ack,
                                      uint16_t seq)
{
  std::vector<uint8_t> pkt;
  pkt.push_back(kStx0);
  pkt.push_back(kStx1);
  pkt.push_back(need_ack ? 0x01 : 0x00);

  const uint16_t data_len = static_cast<uint16_t>(data.size());
  pkt.push_back(static_cast<uint8_t>(data_len & 0xFF));         // little endian
  pkt.push_back(static_cast<uint8_t>((data_len >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(seq & 0xFF));
  pkt.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
  pkt.push_back(cmd_id);
  pkt.insert(pkt.end(), data.begin(), data.end());

  const uint16_t crc = crc16_ccitt(pkt);
  pkt.push_back(static_cast<uint8_t>(crc & 0xFF));
  pkt.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return pkt;
}

// Sends the format command. Returns false only on a local socket error --
// the A8 mini does not ACK 0x48 (see SDK appendix), so a successful send is
// the strongest confirmation available.
bool send_format_command(const std::string & camera_ip, uint16_t control_port,
                         uint16_t seq, std::string & error_out)
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    error_out = std::string("socket() failed: ") + std::strerror(errno);
    return false;
  }

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(control_port);
  if (::inet_pton(AF_INET, camera_ip.c_str(), &dest.sin_addr) != 1) {
    error_out = "invalid camera_ip: " + camera_ip;
    ::close(fd);
    return false;
  }

  const std::vector<uint8_t> pkt =
    build_sdk_packet(kCmdFormatSdCard, {}, /*need_ack=*/false, seq);

  const ssize_t sent = ::sendto(fd, pkt.data(), pkt.size(), 0,
                                reinterpret_cast<sockaddr *>(&dest), sizeof(dest));
  const bool ok = (sent == static_cast<ssize_t>(pkt.size()));
  if (!ok) {
    error_out = std::string("sendto() failed: ") + std::strerror(errno);
  }
  ::close(fd);
  return ok;
}

}  // namespace

class SiyiWipeSd : public rclcpp::Node
{
public:
  SiyiWipeSd()
  : Node("siyi_wipe_sd")
  {
    camera_ip_ = declare_parameter<std::string>("camera_ip", "192.168.144.25");
    control_port_ = declare_parameter<int>("control_port", 37260);
    trigger_channel_ = declare_parameter<int>("trigger_channel", 12);
    trigger_pwm_ = declare_parameter<int>("trigger_pwm", 1800);
    require_disarmed_ = declare_parameter<bool>("require_disarmed", true);
    cooldown_sec_ = declare_parameter<double>("cooldown_sec", 30.0);

    status_pub_ = create_publisher<mavros_msgs::msg::StatusText>(
      "/mavros/statustext/send", 10);

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) { armed_ = msg->armed; });

    rc_sub_ = create_subscription<mavros_msgs::msg::RCOut>(
      "/mavros/rc/out", 10,
      std::bind(&SiyiWipeSd::rc_out_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Watching servo ch %d for >= %d us -> FORMAT SD on %s:%d "
                "(require_disarmed=%s, cooldown=%.0fs)",
                trigger_channel_, trigger_pwm_, camera_ip_.c_str(), control_port_,
                require_disarmed_ ? "true" : "false", cooldown_sec_);
  }

private:
  void send_status(const std::string & text)
  {
    mavros_msgs::msg::StatusText msg;
    msg.header.stamp = now();
    msg.severity = mavros_msgs::msg::StatusText::NOTICE;
    msg.text = text.substr(0, 50);   // STATUSTEXT payload is 50 chars
    status_pub_->publish(msg);
  }

  void rc_out_cb(const mavros_msgs::msg::RCOut::SharedPtr msg)
  {
    const size_t idx = static_cast<size_t>(trigger_channel_) - 1;  // ch is 1-based
    if (trigger_channel_ < 1 || idx >= msg->channels.size()) {
      return;
    }

    const bool high = msg->channels[idx] >= static_cast<uint16_t>(trigger_pwm_);
    const bool rising = high && !was_high_;
    was_high_ = high;

    if (!rising) {
      return;
    }

    if (require_disarmed_ && armed_) {
      RCLCPP_WARN(get_logger(), "Trigger ignored: vehicle is ARMED.");
      send_status("SD wipe BLOCKED: vehicle armed");
      return;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (fired_once_) {
      const double since =
        std::chrono::duration<double>(now_tp - last_fire_).count();
      if (since < cooldown_sec_) {
        RCLCPP_WARN(get_logger(), "Trigger ignored: cooldown (%.1fs left).",
                    cooldown_sec_ - since);
        return;
      }
    }

    RCLCPP_WARN(get_logger(), "FORMATTING SD card on %s -- this erases everything.",
                camera_ip_.c_str());
    send_status("Formatting SIYI SD card...");

    std::string err;
    if (send_format_command(camera_ip_, static_cast<uint16_t>(control_port_),
                            seq_++, err)) {
      last_fire_ = now_tp;
      fired_once_ = true;
      RCLCPP_INFO(get_logger(), "Format command sent (no ACK expected).");
      send_status("SIYI SD format cmd sent");
    } else {
      RCLCPP_ERROR(get_logger(), "Format command FAILED: %s", err.c_str());
      send_status("SIYI SD format FAILED");
    }
  }

  std::string camera_ip_;
  int control_port_{37260};
  int trigger_channel_{12};
  int trigger_pwm_{1800};
  bool require_disarmed_{true};
  double cooldown_sec_{30.0};

  bool armed_{false};
  bool was_high_{false};
  bool fired_once_{false};
  std::chrono::steady_clock::time_point last_fire_{};
  uint16_t seq_{0};

  rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr rc_sub_;
};

int main(int argc, char ** argv)
{
  bool now_mode = false;
  bool confirmed = false;
  std::string cli_ip = "192.168.144.25";
  int cli_port = 37260;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--now") {
      now_mode = true;
    } else if (a == "--confirm") {
      confirmed = true;
    } else if (a == "--ip" && i + 1 < argc) {
      cli_ip = argv[++i];
    } else if (a == "--port" && i + 1 < argc) {
      cli_port = std::stoi(argv[++i]);
    } else if (a == "--help" || a == "-h") {
      std::fprintf(stderr,
        "siyi_wipe_sd -- erase all media on the SIYI camera SD card\n\n"
        "  (no args)            run as a node; Mission Planner triggers via servo\n"
        "  --now --confirm      format immediately and exit\n"
        "  --ip <addr>          camera IP (default 192.168.144.25)\n"
        "  --port <n>           control port (default 37260)\n\n"
        "WARNING: formats the entire SD card. Images AND videos. Irreversible.\n");
      return 0;
    }
  }

  if (now_mode) {
    if (!confirmed) {
      std::fprintf(stderr,
        "Refusing to format: --now also requires --confirm.\n"
        "This ERASES THE ENTIRE SD CARD (images and videos) and cannot be undone.\n");
      return 2;
    }
    std::string err;
    if (!send_format_command(cli_ip, static_cast<uint16_t>(cli_port), 0, err)) {
      std::fprintf(stderr, "Format command failed: %s\n", err.c_str());
      return 1;
    }
    std::fprintf(stderr,
      "Format command sent to %s:%d.\n"
      "The camera does not ACK 0x48, so verify by checking the media list.\n",
      cli_ip.c_str(), cli_port);
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SiyiWipeSd>());
  rclcpp::shutdown();
  return 0;
}
