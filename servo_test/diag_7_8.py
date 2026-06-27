#!/usr/bin/env python3
"""Diagnose servo PDB wiring on outputs 7 and 8.

Checks two things the FCU can see:
  1. SERVO7/8_FUNCTION params (must be 0 = Disabled for DO_SET_SERVO to work).
  2. SERVO_OUTPUT_RAW feedback -- confirms the FCU is actually emitting the PWM
     you commanded on ch7/ch8.

What this CAN'T see: whether the PDB is delivering 5V to the servo rail. If the
output reads back correctly here but the servos don't physically move, the signal
is fine and the problem is power (PDB rail / ground) -- check that with a
multimeter (see the README note at the bottom).
"""
import time
from pymavlink import mavutil

DEVICE = '/dev/ttyUSB0'
BAUD = 115200
CHANNELS = [7, 8]

m = mavutil.mavlink_connection(DEVICE, baud=BAUD)
print("[*] waiting for heartbeat ...")
m.wait_heartbeat(timeout=15)
if not m.target_system:
    m.target_system, m.target_component = 1, 1
print(f"[+] connected sys {m.target_system}")

# --- 1. read servo params for ch7/ch8 ---
pnames = []
for ch in CHANNELS:
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
print("\n--- servo params (FUNCTION should be 0 = Disabled) ---")
for n in pnames:
    print(f"  {n:20s} = {got.get(n, '(none)')}")

# --- 2. stream SERVO_OUTPUT_RAW (msg 36) and watch ch7/ch8 react ---
def set_interval(msgid, hz):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msgid, int(1e6 / hz), 0, 0, 0, 0, 0)

set_interval(36, 5)  # SERVO_OUTPUT_RAW at 5 Hz

def read_outputs():
    out = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
    if not out:
        return None, None
    return out.servo7_raw, out.servo8_raw

def cmd(ch, pwm):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                            float(ch), float(pwm), 0, 0, 0, 0, 0)

print("\n--- live output (servo7_raw / servo8_raw) ---")
print("    each commanded value should appear in the readback below.")
for pwm in (1900, 1100, 1900):
    for ch in CHANNELS:
        cmd(ch, pwm)
    time.sleep(0.8)
    s7, s8 = read_outputs()
    print(f"  commanded {pwm} -> ch7={s7}  ch8={s8}")

print("\n[done] If readback tracks the commanded value but servos don't move,")
print("       the signal path is OK -- check PDB 5V rail + ground with a meter.")
