// comms: mavlink router launcher (C++ port of comms/mavlink_router.py)
//
// Auto-detects the Pixhawk serial device and execs mavlink-routerd, fanning the
// link out to the local UDP endpoints used by MAVROS (14550) and the command
// listener (14601), plus spares.

#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v ? std::string(v) : fallback;
}

std::string first_in(const fs::path & dir, const std::string & prefix)
{
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
}

std::string find_pixhawk_serial()
{
  if (const char * manual = std::getenv("PIXHAWK_SERIAL")) {
    return manual;
  }
  std::string p = first_in("/dev/serial/by-id", "");
  if (!p.empty()) {
    return p;
  }
  p = first_in("/dev", "ttyUSB");
  if (!p.empty()) {
    return p;
  }
  p = first_in("/dev", "ttyACM");
  if (!p.empty()) {
    return p;
  }
  if (fs::exists("/dev/ttyAMA0")) {
    return "/dev/ttyAMA0";
  }
  if (fs::exists("/dev/ttyS0")) {
    return "/dev/ttyS0";
  }
  return "";
}
}  // namespace

int main()
{
  const std::string router_bin = env_or("MAVLINK_ROUTER_BIN", "mavlink-routerd");
  const std::string pixhawk_baud = env_or("PIXHAWK_BAUD", "115200");

  const std::string pixhawk_serial = find_pixhawk_serial();
  if (pixhawk_serial.empty()) {
    std::printf("[ERROR] Could not find Pixhawk serial device.\n");
    std::printf("Plug in the TTL/USB converter or set PIXHAWK_SERIAL=/dev/ttyUSB0\n");
    return 1;
  }
  if (!fs::exists(pixhawk_serial)) {
    std::printf("[ERROR] Selected serial device does not exist: %s\n", pixhawk_serial.c_str());
    return 1;
  }

  const std::string serial_arg = pixhawk_serial + ":" + pixhawk_baud;

  std::printf("Starting Project Astra MAVLink Router\n");
  std::printf("------------------------------------\n");
  std::printf("Pixhawk serial:        %s\n", pixhawk_serial.c_str());
  std::printf("Baud rate:             %s\n", pixhawk_baud.c_str());
  std::printf("MAVROS endpoint:       127.0.0.1:14550\n");
  std::printf("Command listener port: 127.0.0.1:14601\n");
  std::fflush(stdout);

  std::vector<std::string> args = {
    router_bin,
    "-e", "127.0.0.1:14550",  // MAVROS
    "-e", "127.0.0.1:14601",  // Command listener
    "-e", "127.0.0.1:14602",  // Spare
    "-e", "127.0.0.1:14603",  // Spare
    "-e", "127.0.0.1:14604",  // Spare
    "-e", "127.0.0.1:14605",  // Spare
    "-e", "127.0.0.1:14606",  // Spare
    serial_arg,
  };

  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & a : args) {
    argv.push_back(const_cast<char *>(a.c_str()));
  }
  argv.push_back(nullptr);

  execvp(router_bin.c_str(), argv.data());

  // Only reached if exec failed.
  std::perror("execvp mavlink-routerd");
  std::printf("[ERROR] Install MAVLink Router: sudo apt install mavlink-router\n");
  return 1;
}
