#!/usr/bin/env python3
"""
Move two servos on outputs 7 and 8 up and down (standalone, no ROS / MAVROS).

Talks to the flight controller directly over the serial link with pymavlink and
drives PWM outputs 7 and 8 using MAV_CMD_DO_SET_SERVO. Use this to bench-test the
servo PDB wiring: the two servos should sweep down -> up repeatedly.

Examples
--------
  # Sweep both servos down/up 5 times (default)
  python3 servo_7_8_sweep.py

  # Slower sweep, 10 cycles, custom end points
  python3 servo_7_8_sweep.py --cycles 10 --dwell 1.5 --down 1100 --up 1900

  # Drive both to a single position once
  python3 servo_7_8_sweep.py --pwm 1500

Notes
-----
* Nothing else may own the serial port. Stop launcher.sh / MAVROS first,
  otherwise you'll get "device busy".
* The servo rail needs external 5V (from the PDB) or the servos won't move even
  with a correct PWM signal.
* If 7/8 don't respond, the outputs may be assigned to a flight function. Set
  SERVO7_FUNCTION = 0 and SERVO8_FUNCTION = 0 (Disabled) so DO_SET_SERVO can
  drive them, then reboot the FCU.
"""

import argparse
import sys
import time

from pymavlink import mavutil

CHANNELS = [7, 8]


def connect(device, baud):
    print(f"[*] Connecting to {device} @ {baud} ...")
    master = mavutil.mavlink_connection(device, baud=baud)
    print("[*] Waiting for heartbeat ...")
    hb = master.wait_heartbeat(timeout=15)
    if hb is None:
        print("[!] No heartbeat received. Is the FCU connected and the port free?")
        sys.exit(1)
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


def sweep(master, channels, down, up, cycles, dwell):
    for i in range(cycles):
        print(f"--- cycle {i + 1}/{cycles} ---")
        print(f"--- channels {channels}: up ---")
        set_servos(master, channels, up)
        time.sleep(dwell)
        print(f"--- channels {channels}: down ---")
        set_servos(master, channels, down)
        time.sleep(dwell)


def set_param(master, name, value):
    print(f"[*] Set param {name} = {value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('ascii'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        print(f"    {msg.param_id} -> {msg.param_value}")
    else:
        print(f"    [!] no PARAM_VALUE ack for {name} (timed out).")


def setup(master, channels):
    """Set SERVOx_FUNCTION = 0 (Disabled) so DO_SET_SERVO can drive the outputs."""
    print(f"[*] Freeing outputs {channels} for DO_SET_SERVO ...")
    for ch in channels:
        set_param(master, f'SERVO{ch}_FUNCTION', 0)
    print("[*] Rebooting flight controller to apply ...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0)
    print("[+] Reboot sent. Wait ~10s, then rerun (without --setup) to sweep.")


def main():
    ap = argparse.ArgumentParser(
        description="Move servos on outputs 7 and 8 up and down")
    ap.add_argument('--device', default='/dev/ttyUSB0',
                    help='serial device or connection string (default /dev/ttyUSB0). '
                         'Use e.g. udp:127.0.0.1:14550 for a UDP link.')
    ap.add_argument('--baud', type=int, default=115200, help='baud (default 115200)')
    ap.add_argument('--pwm', type=int,
                    help='drive both servos to this PWM once, then exit')
    ap.add_argument('--down', type=int, default=1100, help='down PWM (default 1100)')
    ap.add_argument('--up', type=int, default=1900, help='up PWM (default 1900)')
    ap.add_argument('--cycles', type=int, default=5, help='sweep cycles (default 5)')
    ap.add_argument('--dwell', type=float, default=1.0, help='seconds between moves')
    ap.add_argument('--setup', action='store_true',
                    help='set SERVO7/8_FUNCTION=0 (Disabled) and reboot FCU')
    args = ap.parse_args()

    master = connect(args.device, args.baud)

    if args.setup:
        setup(master, CHANNELS)
    elif args.pwm is not None:
        set_servos(master, CHANNELS, args.pwm)
    else:
        sweep(master, CHANNELS, args.down, args.up, args.cycles, args.dwell)


if __name__ == '__main__':
    main()
