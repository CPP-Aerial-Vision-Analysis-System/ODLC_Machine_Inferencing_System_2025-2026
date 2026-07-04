// comms: ground sender (C++ port of comms/ground_sender.py)
//
// Interactive ground console: opens the RFD radio serial link, waits for a
// MAVLink heartbeat, then lets the operator type "reboot" / "shutdown" to send a
// COMMAND_LONG to the companion computer (sys 200 / comp 191) and reports the
// COMMAND_ACK.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <mavlink/v2.0/ardupilotmega/mavlink.h>

namespace fs = std::filesystem;

namespace
{
constexpr int CMD_REBOOT = 31004;
constexpr int CMD_SHUTDOWN = 31005;
constexpr double ACK_TIMEOUT = 3.0;

constexpr uint8_t TARGET_SYSTEM = 200;
constexpr uint8_t TARGET_COMPONENT = 191;

// This ground console identifies itself as GCS 255/0.
constexpr uint8_t GCS_SYSTEM = 255;
constexpr uint8_t GCS_COMPONENT = 0;

const std::map<std::string, int> CMD_MAP = {
  {"reboot", CMD_REBOOT},
  {"shutdown", CMD_SHUTDOWN},
};

const char * mav_result_name(uint8_t result)
{
  switch (result) {
    case MAV_RESULT_ACCEPTED: return "ACCEPTED";
    case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
    case MAV_RESULT_DENIED: return "DENIED";
    case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
    case MAV_RESULT_FAILED: return "FAILED";
    case MAV_RESULT_IN_PROGRESS: return "IN_PROGRESS";
    case MAV_RESULT_CANCELLED: return "CANCELLED";
    default: return "UNKNOWN";
  }
}

double now_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

std::string autodetect_serial_port()
{
  if (const char * env = std::getenv("RFD_PORT")) {
    return env;
  }
  auto first_glob = [](const fs::path & dir, const std::string & prefix) -> std::string {
    std::error_code ec;
    if (!fs::exists(dir, ec)) {
      return "";
    }
    std::vector<std::string> matches;
    for (const auto & e : fs::directory_iterator(dir, ec)) {
      const std::string name = e.path().filename().string();
      if (prefix.empty() || name.rfind(prefix, 0) == 0) {
        matches.push_back(e.path().string());
      }
    }
    std::sort(matches.begin(), matches.end());
    return matches.empty() ? "" : matches.front();
  };

  std::string p = first_glob("/dev/serial/by-id", "");
  if (!p.empty()) {
    return p;
  }
  p = first_glob("/dev", "ttyUSB");
  if (!p.empty()) {
    return p;
  }
  p = first_glob("/dev", "ttyACM");
  if (!p.empty()) {
    return p;
  }
  return "";
}

int open_serial(const std::string & port, int baud)
{
  const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    return -1;
  }
  speed_t speed = B57600;
  if (baud == 115200) {
    speed = B115200;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

// Read available bytes and feed the parser; returns true when a message of the
// wanted id arrives before the deadline (or any message if want_id < 0).
bool wait_for_message(int fd, int want_id, double timeout, mavlink_message_t & out)
{
  const double deadline = now_seconds() + timeout;
  mavlink_status_t status;
  uint8_t buf[512];
  while (now_seconds() < deadline) {
    const ssize_t n = read(fd, buf, sizeof(buf));
    for (ssize_t i = 0; i < n; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &out, &status)) {
        if (want_id < 0 || out.msgid == static_cast<uint32_t>(want_id)) {
          return true;
        }
      }
    }
    if (n <= 0) {
      usleep(5000);
    }
  }
  return false;
}

void send_command_long(int fd, int cmd_id)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
    GCS_SYSTEM, GCS_COMPONENT, &msg, TARGET_SYSTEM, TARGET_COMPONENT,
    static_cast<uint16_t>(cmd_id), 0, 1, 0, 0, 0, 0, 0, 0);
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ssize_t written = write(fd, buf, len);
  (void)written;
}

void wait_for_ack(int fd, int cmd_id)
{
  mavlink_message_t msg;
  if (wait_for_message(fd, MAVLINK_MSG_ID_COMMAND_ACK, ACK_TIMEOUT, msg)) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    if (ack.command == static_cast<uint16_t>(cmd_id)) {
      std::printf(
        "[ACK] cmd=%d result=%s (%u) from sys=%d comp=%d\n", cmd_id, mav_result_name(ack.result),
        ack.result, msg.sysid, msg.compid);
      return;
    }
  }
  std::printf(
    "[ACK] No ACK received for cmd=%d within %.0fs (radio link issue, or no ACKs)\n", cmd_id,
    ACK_TIMEOUT);
}

std::string strip_lower(const std::string & s)
{
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) {
    ++a;
  }
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) {
    --b;
  }
  std::string out = s.substr(a, b - a);
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}
}  // namespace

int main()
{
  const std::string port = autodetect_serial_port();
  if (port.empty()) {
    std::printf("ERROR: No serial port found. Plug in the RFD radio or set RFD_PORT.\n");
    return 1;
  }
  int baud = 57600;
  if (const char * b = std::getenv("RFD_BAUD")) {
    baud = std::atoi(b);
  }

  std::printf("Connecting to RFD on %s\n", port.c_str());
  const int fd = open_serial(port, baud);
  if (fd < 0) {
    std::printf("ERROR: Could not open serial port %s\n", port.c_str());
    return 1;
  }

  std::printf("Waiting for heartbeat (timeout 30s)\n");
  mavlink_message_t hb;
  if (!wait_for_message(fd, MAVLINK_MSG_ID_HEARTBEAT, 30.0, hb)) {
    std::printf("ERROR: No heartbeat received. Check mavlink-router and serial link.\n");
    close(fd);
    return 1;
  }
  std::printf(
    "MAVLink Heartbeat detected (from system=%d component=%d). Link is active.\n", hb.sysid,
    hb.compid);

  std::printf("Ground console ready.\n\n");
  std::printf("Available commands: reboot, shutdown\n");
  std::printf("Type 'exit' to quit.\n\n");

  std::string line;
  while (true) {
    std::printf("> ");
    std::fflush(stdout);
    if (!std::getline(std::cin, line)) {
      std::printf("\nExiting.\n");
      break;
    }
    const std::string s = strip_lower(line);
    if (s == "exit" || s == "quit") {
      std::printf("Exiting.\n");
      break;
    }
    if (s == "help") {
      std::printf("Available commands: reboot, shutdown\n");
      continue;
    }
    auto it = CMD_MAP.find(s);
    if (it == CMD_MAP.end()) {
      std::printf("Unknown command '%s'. Type 'help' for available commands.\n", s.c_str());
      continue;
    }
    const int cmd_id = it->second;
    send_command_long(fd, cmd_id);
    wait_for_ack(fd, cmd_id);
    std::string upper = s;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    std::printf("Sent COMMAND_LONG: %s (cmd_id=%d)\n", upper.c_str(), cmd_id);
  }

  close(fd);
  return 0;
}
