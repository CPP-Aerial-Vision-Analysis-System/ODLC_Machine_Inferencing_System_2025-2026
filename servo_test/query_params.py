#!/usr/bin/env python3
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
m.wait_heartbeat(timeout=15)
names = ['SERVO9_FUNCTION', 'SERVO12_FUNCTION', 'BTN_ENABLE',
         'BTN_PIN1', 'BTN_PIN2', 'BTN_PIN3', 'BTN_PIN4']
want = set(names)
for n in names:
    m.mav.param_request_read_send(1, 1, n.encode('ascii'), -1)

got = {}
t = time.time()
while time.time() - t < 5 and len(got) < len(want):
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg is None:
        continue
    if msg.param_id in want:
        got[msg.param_id] = msg.param_value
for n in names:
    val = got[n] if n in got else '(no response)'
    print(n, '=', val)
