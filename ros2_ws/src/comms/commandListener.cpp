#include <iostream>
#include <string>
#include <array>
#include "missionStateUtils.hpp"

std::string LISTEN_URI = "udpin:0.0.0.0:14601";

const int CMD_REBOOT = 31004;
const int CMD_SHUTDOWN = 31005;

std::array <int, 2> VALID_COMMANDS = {CMD_REBOOT, CMD_SHUTDOWN};

int main () {
    std::cout << "\n Listening for MAVLink on " << LISTEN_URI << " ...";
    std::cout << " State File: " << STATE_FILE;
    std::cout << "\n Commands: REBOOT=" << CMD_REBOOT << ", SHUTDOWN=" << CMD_SHUTODWN << ".";
    std::cout << "Waiting for COMMAND_LONG. \n";

    if (std::filesystem::exists(STATE_FILE)) std::cout << "State file does exist" << STATE_FILE << "\n";
    else std::cout << "State file does not exist" << STATE_FILE << "\n";
    return 0;
}