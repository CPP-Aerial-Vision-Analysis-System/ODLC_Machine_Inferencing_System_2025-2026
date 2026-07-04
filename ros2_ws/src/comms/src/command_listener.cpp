// comms: command listener (C++ port of comms/command_listener.py)
//
// Binds a UDP MAVLink endpoint (matches mavlink-router's 14601 route), announces
// itself with a 1 Hz HEARTBEAT, and reacts to COMMAND_LONG messages addressed to
// this companion computer (sys 200 / comp 191): REBOOT / SHUTDOWN are recorded in
// the shared mission-state file and acknowledged with COMMAND_ACK.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string>

#include <mavlink/v2.0/ardupilotmega/mavlink.h>

#include "comms/mission_state.hpp"

namespace
{
constexpr uint16_t LISTEN_PORT = 14601;
constexpr int CMD_REBOOT = 31004;
constexpr int CMD_SHUTDOWN = 31005;

constexpr uint8_t SRC_SYSTEM = 200;
constexpr uint8_t SRC_COMPONENT = 191;

double now_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch())
    .count();
}

void send_heartbeat(int sock, const sockaddr_in & dst, bool have_dst)
{
  if (!have_dst) {
    return;
  }
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(
    SRC_SYSTEM, SRC_COMPONENT, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendto(sock, buf, len, 0, reinterpret_cast<const sockaddr *>(&dst), sizeof(dst));
}

void send_command_ack(int sock, const sockaddr_in & dst, uint16_t command)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_ack_pack(
    SRC_SYSTEM, SRC_COMPONENT, &msg, command, MAV_RESULT_ACCEPTED, 0, 0, 0, 0);
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  sendto(sock, buf, len, 0, reinterpret_cast<const sockaddr *>(&dst), sizeof(dst));
}
}  // namespace

int main()
{
  std::printf("\nListening for MAVLink on udpin:0.0.0.0:%u ...\n", LISTEN_PORT);
  std::printf("State file: %s\n\n", comms::state_file_path().c_str());
  std::printf("Commands: REBOOT=%d, SHUTDOWN=%d,\n", CMD_REBOOT, CMD_SHUTDOWN);
  std::printf("Waiting for COMMAND_LONG\n\n");

  // Initialise the state file if missing.
  {
    std::ifstream probe(comms::state_file_path());
    if (!probe) {
      comms::update_state("logging_enabled", false);
      std::printf("[STATE] Initialized %s\n\n", comms::state_file_path().c_str());
    }
  }

  const int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::perror("socket");
    return 1;
  }

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(LISTEN_PORT);
  if (bind(sock, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
    std::perror("bind");
    close(sock);
    return 1;
  }

  // 1 s receive timeout so the heartbeat loop keeps ticking.
  timeval tv{};
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  sockaddr_in remote{};
  bool have_remote = false;
  double last_heartbeat_time = 0.0;

  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t rx[2048];

  while (true) {
    const double current_time = now_seconds();
    if (current_time - last_heartbeat_time >= 1.0) {
      send_heartbeat(sock, remote, have_remote);
      last_heartbeat_time = current_time;
    }

    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    const ssize_t n =
      recvfrom(sock, rx, sizeof(rx), 0, reinterpret_cast<sockaddr *>(&src), &src_len);
    if (n <= 0) {
      continue;  // timeout or error -> loop and heartbeat again
    }

    remote = src;
    have_remote = true;

    for (ssize_t i = 0; i < n; ++i) {
      if (!mavlink_parse_char(MAVLINK_COMM_0, rx[i], &msg, &status)) {
        continue;
      }
      if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
        continue;
      }
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(&msg, &cmd);

      if (cmd.target_system != SRC_SYSTEM || cmd.target_component != SRC_COMPONENT) {
        continue;
      }
      const int command = static_cast<int>(cmd.command);
      if (command != CMD_REBOOT && command != CMD_SHUTDOWN) {
        continue;
      }

      const int src_sys = msg.sysid;
      const int src_comp = msg.compid;
      std::printf("COMMAND_LONG cmd=%d from sys=%d comp=%d\n", command, src_sys, src_comp);

      const char * action = (command == CMD_REBOOT) ? "reboot" : "shutdown";
      comms::update_state("pending_action", action);
      comms::update_state("last_sender_sysid", static_cast<long long>(src_sys));
      comms::update_state("last_sender_compid", static_cast<long long>(src_comp));
      comms::update_state("timestamp", now_seconds());
      std::printf("%s (state applied)\n\n", command == CMD_REBOOT ? "Reboot" : "Shutdown");

      send_command_ack(sock, remote, cmd.command);
    }
  }

  close(sock);
  return 0;
}
