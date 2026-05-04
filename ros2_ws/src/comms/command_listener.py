#!/usr/bin/env python3
import json
import time
from pathlib import Path


from pymavlink import mavutil

import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))

from mission_state_utils import update_state, STATE_FILE

LISTEN_URI = "udpin:0.0.0.0:14601"

CMD_REBOOT = 31004
CMD_SHUTDOWN = 31005


VALID_COMMANDS = (
    CMD_REBOOT,
    CMD_SHUTDOWN,
)


def main():
   #Logs
   print()
   print(f"Listening for MAVLink on {LISTEN_URI} ...")
   print(f"State file: {STATE_FILE}")
   print()
   print(
    f"Commands: REBOOT={CMD_REBOOT}, SHUTDOWN={CMD_SHUTDOWN}, "
    )
   print("Waiting for COMMAND_LONG\n")


   #Makes sure file exists
   if not STATE_FILE.exists():
    update_state("logging_enabled", False)
    print(f"[STATE] Initialized {STATE_FILE}\n")


   m = mavutil.mavlink_connection(LISTEN_URI, source_system=200, source_component=191)

   print("Sending heartbeat to announce Jetson")
   m.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
   )

   print("Waiting for heartbeat to confirm MAVLink link")
   m.wait_heartbeat(timeout=30)
   print(f"Heartbeat received. Listening for COMMAND_LONG messages...\n")
  
   last_heartbeat_time = 0

   while True:
        # Send heartbeat at 1Hz to maintain route in mavlink-router
        current_time = time.time()
        if current_time - last_heartbeat_time >= 1.0:
            m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            last_heartbeat_time = current_time
    
        msg = m.recv_match(type="COMMAND_LONG", blocking=True, timeout=1.0)

        if msg is None:
            continue
    
        # Only process commands intended for this system and component
        if msg.target_system != 200 or msg.target_component != 191:
            continue

        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()

        cmd = int(msg.command)

        if cmd not in VALID_COMMANDS:
            continue
            
       
        print(f"COMMAND_LONG cmd={cmd} from sys={src_sys} comp={src_comp}")

        #Updates
       
        if cmd == CMD_REBOOT:
            update_state("pending_action", "reboot")
            update_state("last_sender_sysid", src_sys)
            update_state("last_sender_compid", src_comp)
            update_state("timestamp", time.time())
            print("Reboot (state applied)\n")

        elif cmd == CMD_SHUTDOWN:
            update_state("pending_action", "shutdown")
            update_state("last_sender_sysid", src_sys)
            update_state("last_sender_compid", src_comp)
            update_state("timestamp", time.time())
            print("Shutdown (state applied)\n")
        
        else:
            print(f"Unrecognised command id={cmd}, ignoring.\n")

        m.mav.command_ack_send(msg.command, 0)

if __name__ == "__main__":
   main()