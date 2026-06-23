#!/usr/bin/env python3
"""Confirm AUX6 (ch14) emits commanded PWM at the pin."""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
m.wait_heartbeat(timeout=15)
if not m.target_system:
    m.target_system, m.target_component = 1, 1
print(f"[+] connected sys {m.target_system}")

m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        36, int(1e6 / 20), 0, 0, 0, 0, 0)

def latest(window=1.2):
    last = None
    t = time.time()
    while time.time() - t < window:
        msg = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=0.3)
        if msg:
            last = msg
    return last

def cmd(ch, pwm):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            float(ch), float(pwm), 0, 0, 0, 0, 0)

print("\n--- drained live output ---")
for pwm in (1900, 1400, 1750, 1100):
    cmd(9, pwm)
    cmd(14, pwm)
    o = latest()
    if o:
        print(f"  commanded {pwm} -> ch9={o.servo9_raw}  ch14={o.servo14_raw}")
    else:
        print(f"  commanded {pwm} -> (no data)")
