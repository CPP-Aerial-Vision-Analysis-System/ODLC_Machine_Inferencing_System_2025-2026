#!/usr/bin/env python3
"""test_ros_tap_live.py -- PURPOSE: prove the ROS tap end to end with real ROS 2 (inside WSL, no board, no simulator):
a "team" publisher on ROS domain 77 (as in the Jetson container), ros_tap.py send on that domain, ros_tap.py receive on
another domain (88, standing in for the PC; never domain 0, so a simulator running on this PC is never disturbed),
connected through a socket file exactly like the virtual Jetson. It checks that:
  1. messages published on domain 77 arrive on domain 88 with the same type (raw bytes, no change),
  2. a "latched" (transient-local) message reaches a listener that connects later,
  3. the presence node /jetson/main_controller appears on domain 88,
  4. nothing flows the other way (a message published on domain 88 never reaches domain 77),
  5. after the receiver is killed and started again (a new client), messages flow again.
Skipped (exit 0 with a note) when rclpy is not available. Runs as a pytest test too.

Run (inside WSL):  source simulation/setup/sim_env.sh; /usr/bin/python3 simulation/jetson/pc/test/test_ros_tap_live.py
"""
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TAP = os.path.join(os.path.dirname(HERE), 'ros_tap.py')

PUBLISHER = r'''
import sys, time, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node('main_controller')
pub = n.create_publisher(String, '/camera/status', 10)
lat = n.create_publisher(String, '/latched', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                       reliability=ReliabilityPolicy.RELIABLE))
heard = []
n.create_subscription(String, '/from_pc', lambda m: heard.append(m.data), 10)
lat.publish(String(data='mission-list'))
i = 0
end = time.time() + float(sys.argv[1])
try:
    while time.time() < end:
        i += 1
        pub.publish(String(data=f'status {i}'))
        rclpy.spin_once(n, timeout_sec=0.2)
except Exception:          # (SIGTERM from the test: rclpy shuts down; still report what was heard)
    pass
finally:
    print('HEARD_FROM_PC', len(heard), flush=True)
'''

LISTENER = r'''
import sys, time, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node('pc_listener')
got, lat = [], []
n.create_subscription(String, '/camera/status', lambda m: got.append(m.data), 10)
n.create_subscription(String, '/latched', lambda m: lat.append(m.data),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
pub = n.create_publisher(String, '/from_pc', 10)
end = time.time() + float(sys.argv[1])
presence = False
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
    pub.publish(String(data='must never reach the Jetson'))
    if ('main_controller', '/jetson') in n.get_node_names_and_namespaces():
        presence = True
    if got and lat and presence:
        break
print('GOT', len(got), 'LATCHED', lat[:1], 'PRESENCE', presence, flush=True)
'''


def _env(domain):
    e = dict(os.environ)
    e['ROS_DOMAIN_ID'] = str(domain)
    e['ROS_LOCALHOST_ONLY'] = '1'
    return e


def run_live() -> int:
    try:
        import rclpy  # noqa: F401
    except ImportError:
        print('SKIP: rclpy is not available (source /opt/ros/humble/setup.bash and use /usr/bin/python3)')
        return 0
    tmp = tempfile.mkdtemp(prefix='tap_live_')
    topics = os.path.join(tmp, 'topics.txt')
    with open(topics, 'w') as f:
        f.write('[common]\n/camera/status std_msgs/msg/String\n/latched std_msgs/msg/String\n'
                '/from_pc std_msgs/msg/String\n[nodes]\nmain_controller\n')
    sock = os.path.join(tmp, 'tap.sock')
    py = sys.executable
    procs = []
    failures = []

    def start(args, domain, log):
        p = subprocess.Popen(args, env=_env(domain), stdout=open(os.path.join(tmp, log), 'w'), stderr=subprocess.STDOUT)
        procs.append(p)
        return p

    try:
        team = subprocess.Popen([py, '-c', PUBLISHER, '45'], env=_env(77), stdout=subprocess.PIPE, text=True)
        procs.append(team)
        start([py, TAP, 'send', '--listen', f'unix:{sock}', '--topics', topics, '--parts', 'common',
               '--log', os.path.join(tmp, 'send.log')], 77, 'send.out')
        recv = start([py, TAP, 'receive', '--connect', f'unix:{sock}', '--status-file', os.path.join(tmp, 'tap.env'),
                      '--log', os.path.join(tmp, 'recv.log')], 88, 'recv.out')
        out = subprocess.run([py, '-c', LISTENER, '30'], env=_env(88), capture_output=True, text=True, timeout=60).stdout
        print('listener 1:', out.strip())
        if 'GOT 0' in out or 'GOT' not in out:
            failures.append('1: no /camera/status message reached domain 88')
        if "LATCHED ['mission-list']" not in out:
            failures.append('2: the latched message did not reach a late listener')
        if 'PRESENCE True' not in out:
            failures.append('3: no presence node /jetson/main_controller')
        # 5. a new client (receiver restarted)
        recv.terminate()
        recv.wait(10)
        start([py, TAP, 'receive', '--connect', f'unix:{sock}', '--log', os.path.join(tmp, 'recv2.log')], 88, 'recv2.out')
        out2 = subprocess.run([py, '-c', LISTENER, '25'], env=_env(88), capture_output=True, text=True, timeout=60).stdout
        print('listener 2 (after the receiver restarted):', out2.strip())
        if 'GOT 0' in out2 or 'GOT' not in out2:
            failures.append('5: no messages after the receiver reconnected')
        team.terminate()
        team_out = team.communicate(timeout=10)[0]
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
    # 4. one way only: the team publisher must never hear the PC's /from_pc (even though /from_pc is in the list)
    heard = [ln for ln in (team_out or '').splitlines() if ln.startswith('HEARD_FROM_PC')]
    print('team publisher:', heard[0] if heard else '(no report)')
    if not heard:
        failures.append('4: the team publisher did not report what it heard')
    elif heard[0] != 'HEARD_FROM_PC 0':
        failures.append(f'4: the Jetson side heard the PC ({heard[0]})')
    with open(os.path.join(tmp, 'send.log')) as f:
        print('send.log (last lines):', ''.join(f.readlines()[-4:]).strip())
    if failures:
        print('FAILED:', '; '.join(failures), f'(logs in {tmp})')
        return 1
    print(f'PASSED: domain 77 -> 88 copy, latched, presence, one way, reconnect (logs in {tmp})')
    return 0


def test_tap_live():
    import pytest
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pytest.skip('rclpy not available')
    assert run_live() == 0


if __name__ == '__main__':
    sys.exit(run_live())
