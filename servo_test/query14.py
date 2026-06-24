#!/usr/bin/env python3
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
m.wait_heartbeat(timeout=15)
if not m.target_system:
    m.target_system, m.target_component = 1, 1

names = ['SERVO14_FUNCTION', 'SERVO14_MIN', 'SERVO14_MAX', 'SERVO14_TRIM',
         'SERVO14_REVERSED',
         'BTN_PIN1', 'BTN_PIN2', 'BTN_PIN3', 'BTN_PIN4', 'BTN_PIN5', 'BTN_PIN6']
want = set(names)
for n in names:
    m.mav.param_request_read_send(1, 1, n.encode('ascii'), -1)
got = {}
t = time.time()
while time.time() - t < 6 and len(got) < len(want):
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id in want:
        got[msg.param_id] = msg.param_value
for n in names:
    print(n, '=', got[n] if n in got else '(none)')
print("NOTE: AUX6 GPIO pin = 55; if any BTN_PIN==55 it must be cleared.")
