#!/usr/bin/env python3
"""
Standalone Pixhawk servo control (no ROS / MAVROS needed).

Talks to the flight controller directly over the serial link using pymavlink
and drives one or more PWM output channels with MAV_CMD_DO_SET_SERVO.

Channel map (Cube/Pixhawk): AUX1=9, AUX2=10, AUX3=11, AUX4=12, AUX5=13, AUX6=14

By default the sweep test opens/closes output 2 and repeats that cycle 3 times.

Examples
--------
  # Drive output 2 to 1900us (OPEN)
  python3 servo_control.py --pwm 1900

  # Drive it back to 1400us (CLOSE)
  python3 servo_control.py --pwm 1400

  # Run the sweep cycle on output 2, repeated 3 times
  python3 servo_control.py --sweep

  # A different channel
  python3 servo_control.py --channels 14 --pwm 1900

  # Interactive: type PWM values, 'q' to quit
  python3 servo_control.py --interactive

  # One-time fix so output 2 is a real PWM output, then reboot FCU
  python3 servo_control.py --setup

Notes
-----
* Nothing else may own the serial port. Stop launcher.sh / MAVROS first,
  otherwise you'll get "device busy".
* The servo rail (AUX) needs external 5V or the servos won't move even with
  a correct PWM signal.
"""

import argparse
import sys
import time

from pymavlink import mavutil

# Default single-command channels. The sweep test uses DEFAULT_SWEEP_GROUPS.
DEFAULT_CHANNELS = [2]
DEFAULT_SWEEP_GROUPS = [
    [2],   # output 2
]

# ArduPilot GPIO pin numbers for the AUX outputs (used by BTN_PINx / RELAYx etc.)
#   AUX1=50, AUX2=51, AUX3=52, AUX4=53, AUX5=54, AUX6=55
# The channels we manage and the button param (if any) that can steal each pin.
# AUX6 (pin 55) has no default BTN_PIN mapping, so btn_param is None.
MANAGED = [
    {'ch': 2, 'func_param': 'SERVO2_FUNCTION', 'btn_param': None},  # MAIN OUT 2
]


def connect(device, baud):
    print(f"[*] Connecting to {device} @ {baud} ...")
    master = mavutil.mavlink_connection(device, baud=baud)
    print("[*] Waiting for heartbeat ...")
    hb = master.wait_heartbeat(timeout=15)
    if hb is None:
        print("[!] No heartbeat received. Is the FCU connected and the port free?")
        sys.exit(1)
    # If we latched a broadcast/0 sysid, prefer the real autopilot (1,1).
    if not master.target_system:
        master.target_system = 1
        master.target_component = 1
    print(f"[+] Heartbeat. Using system {master.target_system}, "
          f"component {master.target_component}")
    return master


def set_servo(master, channel, pwm):
    """Send MAV_CMD_DO_SET_SERVO and report the autopilot's ACK."""
    print(f"[*] DO_SET_SERVO  channel={channel}  pwm={pwm}us")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,                # confirmation
        float(channel),   # param1: servo output number
        float(pwm),       # param2: PWM value (us)
        0, 0, 0, 0, 0)

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack is None:
        print(f"    [!] ch{channel}: no COMMAND_ACK (timed out).")
        return False
    accepted = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    result_name = mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
    print(f"    [{'+' if accepted else '!'}] ch{channel} ACK: {result_name}")
    return accepted


def set_servos(master, channels, pwm):
    for ch in channels:
        set_servo(master, ch, pwm)


def sweep(master, groups, low, high, cycles, dwell):
    for i in range(cycles):
        print(f"--- cycle {i + 1}/{cycles} ---")
        for group in groups:
            print(f"--- channels {group}: open ---")
            set_servos(master, group, high)
            time.sleep(dwell)
            print(f"--- channels {group}: close ---")
            set_servos(master, group, low)
            time.sleep(dwell)


def interactive(master, channels):
    print(f"Interactive mode on channels {channels}. "
          f"Type a PWM (e.g. 1900) and Enter. 'q' to quit.")
    while True:
        try:
            raw = input(f"ch{channels} pwm> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if raw.lower() in ('q', 'quit', 'exit'):
            break
        if not raw:
            continue
        try:
            pwm = int(raw)
        except ValueError:
            print("    enter an integer PWM (typ. 1000-2000) or 'q'")
            continue
        set_servos(master, channels, pwm)


def set_param(master, name, value, ptype=None):
    if ptype is None:
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    print(f"[*] Set param {name} = {value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('ascii'),
        float(value),
        ptype)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        print(f"    {msg.param_id} -> {msg.param_value}")


def setup(master):
    """Make output 2 a normal PWM output DO_SET_SERVO can drive."""
    print("[*] Configuring output 2 as a PWM output ...")
    for m in MANAGED:
        if m['btn_param']:
            set_param(master, m['btn_param'], -1)  # release the pin from the Button library
        set_param(master, m['func_param'], 0)      # 0 = Disabled = controllable by DO_SET_SERVO
    print("[*] Rebooting flight controller to apply ...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0)
    print("[+] Reboot command sent. Wait ~10s, then rerun with --sweep to test.")


def main():
    ap = argparse.ArgumentParser(description="Standalone Pixhawk servo control")
    ap.add_argument('--device', default='/dev/ttyUSB0',
                    help='serial device or connection string (default /dev/ttyUSB0). '
                         'Use e.g. udp:127.0.0.1:14550 for a UDP link.')
    ap.add_argument('--baud', type=int, default=115200, help='baud (default 115200)')
    ap.add_argument('--channels', default=None,
                    help='comma-separated servo channels (default "2")')
    ap.add_argument('--channel', type=int, default=None,
                    help='single channel shortcut (overrides --channels)')
    ap.add_argument('--pwm', type=int, help='PWM in microseconds to send once')
    ap.add_argument('--sweep', action='store_true',
                    help='cycle output 2 open/close, repeated by --cycles')
    ap.add_argument('--low', type=int, default=1400, help='sweep low PWM (default 1400)')
    ap.add_argument('--high', type=int, default=1900, help='sweep high PWM (default 1900)')
    ap.add_argument('--cycles', type=int, default=3, help='sweep cycles (default 3)')
    ap.add_argument('--dwell', type=float, default=1.0, help='seconds between moves')
    ap.add_argument('--interactive', action='store_true', help='type PWM values live')
    ap.add_argument('--setup', action='store_true',
                    help='set SERVO2_FUNCTION=0, reboot FCU')
    args = ap.parse_args()

    if args.channel is not None:
        channels = [args.channel]
    elif args.channels:
        channels = [int(c) for c in args.channels.split(',') if c.strip()]
    else:
        channels = list(DEFAULT_CHANNELS)

    master = connect(args.device, args.baud)

    if args.setup:
        setup(master)
        return
    if args.sweep:
        if args.channel is not None or args.channels:
            groups = [channels]
        else:
            groups = DEFAULT_SWEEP_GROUPS
        sweep(master, groups, args.low, args.high, args.cycles, args.dwell)
    elif args.interactive:
        interactive(master, channels)
    elif args.pwm is not None:
        set_servos(master, channels, args.pwm)
    else:
        print("Nothing to do. Pass --pwm, --sweep, --interactive, or --setup.")
        print("Run with -h for help.")


if __name__ == '__main__':
    main()
