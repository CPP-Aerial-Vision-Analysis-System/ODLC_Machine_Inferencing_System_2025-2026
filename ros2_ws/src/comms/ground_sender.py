#!/usr/bin/env python3
from pymavlink import mavutil
import time

import os
from pathlib import Path

def autodetect_serial_port():
    """
    Finds a likely USB serial device for the RFD radio.
    Priority:
    1. User-provided RFD_PORT env variable
    2. Stable /dev/serial/by-id path
    3. Common /dev/ttyUSB* ports
    4. Common /dev/ttyACM* ports
    """

    env_port = os.environ.get("RFD_PORT")
    if env_port:
        return env_port

    by_id_dir = Path("/dev/serial/by-id")
    if by_id_dir.exists():
        ports = sorted(by_id_dir.glob("*"))
        if ports:
            return str(ports[0])

    usb_ports = sorted(Path("/dev").glob("ttyUSB*"))
    if usb_ports:
        return str(usb_ports[0])

    acm_ports = sorted(Path("/dev").glob("ttyACM*"))
    if acm_ports:
        return str(acm_ports[0])

    raise RuntimeError(
        "No serial port found. Plug in the RFD radio or set RFD_PORT manually."
    )

RFD_PORT = autodetect_serial_port()
RFD_BAUD = int(os.environ.get("RFD_BAUD", 57600))

CMD_REBOOT = 31004
CMD_SHUTDOWN = 31005

ACK_TIMEOUT = 3

#Target system/component to target mavlink listener directly (1,1)
TARGET_SYSTEM    = 200
TARGET_COMPONENT = 191

CMD_MAP = {
    "reboot": CMD_REBOOT,
    "shutdown": CMD_SHUTDOWN,
}

MAV_RESULT = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}

def send_command_long(m, cmd_id: int) -> None:
    m.mav.command_long_send(
        TARGET_SYSTEM,    #target_system
        TARGET_COMPONENT, #target_component
        cmd_id,           #command
        0,                #confirmation (0 = first transmission)
        1,                #param1 — use as a simple flag if needed
        0, 0, 0, 0, 0, 0  #param2-7 unused
    )

def wait_for_ack(m, cmd_id: int, timeout: float = ACK_TIMEOUT) -> None:

    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack is None:
            continue
        if int(ack.command) != cmd_id:
            continue
        result_code = int(ack.result)
        result_name = MAV_RESULT.get(result_code, f"UNKNOWN({result_code})")
        src_sys  = ack.get_srcSystem()
        src_comp = ack.get_srcComponent()
        print(f"[ACK] cmd={cmd_id} result={result_name} ({result_code}) "
            f"from sys={src_sys} comp={src_comp}")
        return

    print(f"[ACK] No ACK received for cmd={cmd_id} within {timeout}s "
        f"(radio link issue, or listener does not send ACKs)")



    
def main():
    # print(f"Connecting to RFD on {RFD_PORT} @ {RFD_BAUD}...")
    # m = mavutil.mavlink_connection(RFD_PORT, baud=RFD_BAUD)

    print(f"Connecting to RFD on {RFD_PORT}")
    m = mavutil.mavlink_connection(RFD_PORT, baud=RFD_BAUD)

    print("Waiting for heartbeat (timeout 30s)")
    hb = m.wait_heartbeat(timeout=30)

    if not hb:
        print("ERROR: No heartbeat received. mavlink-router and upstream serial link.")
        return

    print(f"MAVLink Heartbeat detected (from system={hb.get_srcSystem()} component={hb.get_srcComponent()}). Link is active.")

    print("Waiting for telemetry...")
    msg = m.recv_match(type='SYS_STATUS', blocking=True, timeout=5)

    if msg is None:
        print("WARNING: MAVLink Heartbeat received but no telemetry stream detected")
    else:
        print("Telemetry stream confirmed")

    # We DO NOT update global targets based on the heartbeat.
    # The heartbeat is likely from the Pixhawk (sys 1), but we want to send 
    # commands to the companion computer (TARGET_SYSTEM=200, TARGET_COMPONENT=191).
    print("Ground console ready.\n")
    print()
    print("Available commands:", ", ".join(CMD_MAP.keys()))
    print("Type 'exit' to quit.\n")

    while True:
        try:
            s = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if s in ("exit", "quit"):
            print("Exiting.")
            break

        if s == "help":
            print("Available commands:", ", ".join(CMD_MAP.keys()))
            continue

        if s not in CMD_MAP:
            print(f"Unknown command '{s}'. Type 'help' for available commands.")
            continue

        cmd_id = CMD_MAP[s]
        send_command_long(m, cmd_id)
        wait_for_ack(m, cmd_id)
        print(f"Sent COMMAND_LONG: {s.upper()} (cmd_id={cmd_id})")


if __name__ == "__main__":
    main()