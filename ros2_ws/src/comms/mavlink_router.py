#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys
from pathlib import Path


MAVLINK_ROUTER_BIN = os.environ.get("MAVLINK_ROUTER_BIN", "mavlink-routerd")
PIXHAWK_BAUD = os.environ.get("PIXHAWK_BAUD", "57600")


def find_pixhawk_serial():
    """
    Auto-detect the Pixhawk TTL/USB serial device.

    Priority:
    1. PIXHAWK_SERIAL environment variable
    2. /dev/serial/by-id/*
    3. /dev/ttyUSB*
    4. /dev/ttyACM*
    5. /dev/ttyAMA0
    6. /dev/ttyS0
    """

    manual_port = os.environ.get("PIXHAWK_SERIAL")
    if manual_port:
        return manual_port

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

    if Path("/dev/ttyAMA0").exists():
        return "/dev/ttyAMA0"

    if Path("/dev/ttyS0").exists():
        return "/dev/ttyS0"

    return None


def main():
    if shutil.which(MAVLINK_ROUTER_BIN) is None:
        print(f"[ERROR] {MAVLINK_ROUTER_BIN} not found.")
        print("Install MAVLink Router first:")
        print("  sudo apt install mavlink-router")
        print("or build/install it from source.")
        sys.exit(1)

    pixhawk_serial = find_pixhawk_serial()

    if pixhawk_serial is None:
        print("[ERROR] Could not find Pixhawk serial device.")
        print("Plug in the TTL/USB converter or manually set:")
        print("  PIXHAWK_SERIAL=/dev/ttyUSB0 python3 mavlink_router.py")
        sys.exit(1)

    if not Path(pixhawk_serial).exists():
        print(f"[ERROR] Selected serial device does not exist: {pixhawk_serial}")
        sys.exit(1)

    cmd = [
        MAVLINK_ROUTER_BIN,
        "-e", f"127.0.0.1:14550", #MAvros
        "-e", f"127.0.0.1:14601", #Command listener
        "-e", f"127.0.0.1:14602", #Spare
        "-e", f"127.0.0.1:14603", #Spare
        "-e", f"127.0.0.1:14604", #Spare
        "-e", f"127.0.0.1:14605", #Spare
        "-e", f"127.0.0.1:14606", #Spare
        f"{pixhawk_serial}:{PIXHAWK_BAUD}",
    ]

    print("Starting Project Astra MAVLink Router")
    print("------------------------------------")
    print(f"Pixhawk serial:        {pixhawk_serial}")
    print(f"Baud rate:             {PIXHAWK_BAUD}")
    print(f"MAVROS endpoint:       127.0.0.1:14550")
    print(f"Command listener port: 127.0.0.1:14601")

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nMAVLink router stopped.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] mavlink-routerd exited with code {e.returncode}")
        sys.exit(e.returncode)


if __name__ == "__main__":
    main()