#!/usr/bin/env python3
"""Clean read: drain SERVO_OUTPUT_RAW so values are current, watch ch9 vs ch12."""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
m.wait_heartbeat(timeout=15)
if not m.target_system:
    m.target_system, m.target_component = 1, 1
print(f"[+] connected sys {m.target_system}")

m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        36, int(1e6 / 20), 0, 0, 0, 0, 0)  # SERVO_OUTPUT_RAW @20Hz

def latest_outputs(window=1.0):
    """Drain for `window` seconds, return the most recent frame."""
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
    cmd(12, pwm)
    o = latest_outputs(1.2)
    if o:
        print(f"  commanded {pwm} -> ch9={o.servo9_raw}  ch11={o.servo11_raw}  "
              f"ch12={o.servo12_raw}  ch13={o.servo13_raw}")
    else:
        print(f"  commanded {pwm} -> (no SERVO_OUTPUT_RAW)")
