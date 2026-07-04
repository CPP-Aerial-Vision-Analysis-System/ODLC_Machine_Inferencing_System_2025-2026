// comms: system controller (C++ port of comms/system_controller.py)
//
// Polls the shared mission-state file; when command_listener records a pending
// "shutdown" action, it clears the flag and powers the Jetson off.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "comms/mission_state.hpp"

namespace
{
constexpr double STATE_POLL_HZ = 2.0;
}

int main()
{
  const double state_period_s = 1.0 / STATE_POLL_HZ;

  std::printf("State file: %s\n\n", comms::state_file_path().c_str());
  std::printf("System Controller ready... (shutdown)\n\n");

  while (true) {
    const comms::State state = comms::load_state();
    const std::string action = comms::get_string(state, "pending_action");

    if (action == "shutdown") {
      comms::update_state_null("pending_action");
      comms::update_state("last_command", std::string("shutdown"));
      std::printf("Shutting down...\n");
      std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
      const std::string password = "UAV_Lab";
      const std::string cmd = "echo " + password + " | sudo -S shutdown -h now";
      if (std::system(cmd.c_str()) != 0) {
        std::printf("[ERROR] shutdown command failed\n");
      }
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(state_period_s));
  }

  return 0;
}
