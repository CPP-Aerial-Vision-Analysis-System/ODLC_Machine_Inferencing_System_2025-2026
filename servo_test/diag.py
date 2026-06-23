#!/usr/bin/env python3
"""Diagnose why AUX4 (ch12) servo output isn't moving vs working AUX1 (ch9)."""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
m.wait_heartbeat(timeout=15)
if not m.target_system:
    m.target_system, m.target_component = 1, 1
print(f"[+] connected sys {m.target_system}")

# --- 1. compare servo params for ch9 (works) vs ch12 (doesn't) ---
pnames = []
for ch in (9, 12):
    for suf in ('FUNCTION', 'MIN', 'MAX', 'TRIM', 'REVERSED'):
        pnames.append(f'SERVO{ch}_{suf}')
want = set(pnames)
for n in pnames:
    m.mav.param_request_read_send(1, 1, n.encode('ascii'), -1)
got = {}
t = time.time()
while time.time() - t < 6 and len(got) < len(want):
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id in want:
        got[msg.param_id] = msg.param_value
print("\n--- servo params ---")
for n in pnames:
    print(f"  {n:20s} = {got.get(n, '(none)')}")

# --- 2. stream SERVO_OUTPUT_RAW (msg 36) and watch ch9 vs ch12 react ---
def set_interval(msgid, hz):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msgid, int(1e6 / hz), 0, 0, 0, 0, 0)

set_interval(36, 5)  # SERVO_OUTPUT_RAW at 5 Hz

def read_outputs():
    out = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
    if not out:
        return None, None
    return out.servo9_raw, out.servo12_raw

def cmd(ch, pwm):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            float(ch), float(pwm), 0, 0, 0, 0, 0)

print("\n--- live output (servo9_raw / servo12_raw) ---")
for pwm in (1900, 1400, 1900):
    cmd(9, pwm)
    cmd(12, pwm)
    time.sleep(0.8)
    s9, s12 = read_outputs()
    print(f"  commanded {pwm} -> ch9={s9}  ch12={s12}")
