#!/usr/bin/env python3

import asyncio
import os
from mission_state_utils import load_state, update_state, STATE_FILE

STATE_POLL_HZ = 2.0


async def main():
    state_period_s = 1.0 / STATE_POLL_HZ

    print(f"State file: {STATE_FILE}")
    print()
    print("System Controller ready... (shutdown)")
    print()

    while True:
        state = load_state()
        action = state.get("pending_action")

        # if action == "reboot":
        #     update_state("pending_action", None)
        #     update_state("last_command", "reboot")
        #     print("Rebooting...")
        #     await asyncio.sleep(0.5)
        #     os.system("reboot")

        if action == "shutdown":
            update_state("pending_action", None)
            update_state("last_command", "shutdown")
            print("Shutting down...")
            await asyncio.sleep(0.5)
            password = "UAV_Lab"
            os.system(f"echo {password} | sudo -S shutdown -h now") #shutdown the jetson

        await asyncio.sleep(state_period_s)


if __name__ == "__main__":
    asyncio.run(main())