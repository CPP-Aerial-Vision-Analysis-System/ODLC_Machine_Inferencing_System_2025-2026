#!/usr/bin/env python3

from pymavlink import mavutil
import time
import os


MAVLINK_URL = os.environ.get("GROUND_MAVLINK_URL", "udpin:0.0.0.0:14601")

CMD_REBOOT = 31004
CMD_SHUTDOWN = 31005

ACK_TIMEOUT = 3

# Target the Jetson command listener directly.
TARGET_SYSTEM = 200
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
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        cmd_id,
        0,
        1,
        0, 0, 0, 0, 0, 0
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
        src_sys = ack.get_srcSystem()
        src_comp = ack.get_srcComponent()

        print(
            f"[ACK] cmd={cmd_id} result={result_name} ({result_code}) "
            f"from sys={src_sys} comp={src_comp}"
        )
        return

    print(
        f"[ACK] No ACK received for cmd={cmd_id} within {timeout}s "
        f"(radio link issue, routing issue, or listener does not send ACKs)"
    )


def main():
    print(f"Connecting to MAVLink bridge on {MAVLINK_URL}")
    print()
    print("Make sure the PowerShell MAVLink bridge is already running.")
    print("This sender should NOT connect directly to the RFD900 COM port.")
    print()

    m = mavutil.mavlink_connection(
        MAVLINK_URL,
        source_system=250,
        source_component=190
    )

    print("Waiting for heartbeat, timeout 30s...")
    hb = m.wait_heartbeat(timeout=30)

    if not hb:
        print("ERROR: No heartbeat received.")
        print("Check that:")
        print("  1. The MAVLink bridge is running")
        print("  2. The bridge has UDP output 127.0.0.1:14601 enabled")
        print("  3. The RFD900 link is active")
        print("  4. The Pixhawk or Jetson is sending MAVLink heartbeats")
        return

    print(
        f"MAVLink heartbeat detected from "
        f"system={hb.get_srcSystem()} component={hb.get_srcComponent()}"
    )

    print("Waiting for telemetry...")
    msg = m.recv_match(type="SYS_STATUS", blocking=True, timeout=5)

    if msg is None:
        print("WARNING: Heartbeat received but no SYS_STATUS telemetry detected")
    else:
        print("Telemetry stream confirmed")

    print()
    print("Ground console ready.")
    print("Available commands:", ", ".join(CMD_MAP.keys()))
    print("Type 'exit' to quit.")
    print()

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

        print(f"Sending COMMAND_LONG: {s.upper()} cmd_id={cmd_id}")
        send_command_long(m, cmd_id)
        wait_for_ack(m, cmd_id)


if __name__ == "__main__":
    main()
