#!/usr/bin/env python3
"""
ASTRA status tap.

Prints every message the mission stack sends to the ground station
(send_ack -> /mavros/statustext/send), plus what the flight controller
sends back (/mavros/statustext/recv) and the camera's own status line.

Meant to be run over SSH:
    ./scripts/watch_status.sh

Options:
    --send-only        only our own send_ack traffic
    --topic NAME       add another std_msgs/String topic (repeatable)
    --no-color         plain output (for piping to a file)
"""

import argparse
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import StatusText
from std_msgs.msg import String

# MAV_SEVERITY
SEVERITY = {
    0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERROR",
    4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG",
}

COLORS = {
    "TX": "\033[36m",      # cyan  - us -> GCS
    "RX": "\033[35m",      # magenta - FC -> us
    "CAM": "\033[32m",     # green
    "OTHER": "\033[37m",
}
RESET = "\033[0m"
DIM = "\033[2m"


class StatusTap(Node):
    def __init__(self, args):
        super().__init__("status_tap")
        self.color = not args.no_color and sys.stdout.isatty()

        self.create_subscription(
            StatusText, "/mavros/statustext/send",
            lambda m: self.on_statustext("TX", m), 10)

        if not args.send_only:
            # mavros publishes recv best-effort; match it.
            self.create_subscription(
                StatusText, "/mavros/statustext/recv",
                lambda m: self.on_statustext("RX", m), qos_profile_sensor_data)
            self.create_subscription(
                String, "/camera/status",
                lambda m: self.emit("CAM", "", m.data), 10)

        for topic in args.topic or []:
            self.create_subscription(
                String, topic,
                lambda m, t=topic: self.emit("OTHER", t, m.data), 10)

        self.emit("OTHER", "", "status tap running - Ctrl-C to stop")

    def on_statustext(self, tag, msg):
        self.emit(tag, SEVERITY.get(msg.severity, str(msg.severity)), msg.text)

    def emit(self, tag, note, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        if self.color:
            head = f"{DIM}{stamp}{RESET} {COLORS.get(tag, '')}{tag:<5}{RESET}"
            note = f"{DIM}{note}{RESET} " if note else ""
        else:
            head = f"{stamp} {tag:<5}"
            note = f"{note} " if note else ""
        print(f"{head} {note}{text}", flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--send-only", action="store_true")
    p.add_argument("--topic", action="append")
    p.add_argument("--no-color", action="store_true")
    args = p.parse_args()

    rclpy.init()
    node = StatusTap(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
