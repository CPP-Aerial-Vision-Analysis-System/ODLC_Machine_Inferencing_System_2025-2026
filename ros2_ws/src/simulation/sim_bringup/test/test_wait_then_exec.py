"""
sim_bringup/test/test_wait_then_exec.py -- PURPOSE: unit tests for wait_then_exec, the "start this program only when
that port is open" helper that keeps Gazebo -> SITL -> MAVProxy in the right order. Uses fake /proc/net tables
and a real subprocess run (the helper replaces itself with `echo`).
"""

import subprocess
import sys

from sim_bringup import wait_then_exec as w

# Two sample lines in the kernel's format: UDP 127.0.0.1:9002 bound (0x232A), TCP 0.0.0.0:5760 listening (0x1680)
UDP_TABLE = ('  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref pointer drops\n'
             '  7: 0100007F:232A 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 1 2 0 0\n')
TCP_TABLE = ('  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode\n'
             '   0: 00000000:1680 00000000:0000 0A 00000000:00000000 00:00000000 00000000  1000        0 3\n'
             '   1: 0100007F:1F90 0100007F:9C40 01 00000000:00000000 00:00000000 00000000  1000        0 4\n')


def test_parse_tables():
    assert w._local_ports(UDP_TABLE, tcp=False) == {9002}
    # only LISTEN sockets count for TCP: port 8080 (0x1F90) is an established connection
    assert w._local_ports(TCP_TABLE, tcp=True) == {5760}


def test_missing_conditions_with_fake_proc(tmp_path):
    (tmp_path / 'udp').write_text(UDP_TABLE)
    (tmp_path / 'tcp').write_text(TCP_TABLE)
    assert w.missing_conditions([9002], [5760], [], proc_root=str(tmp_path)) == []
    missing = w.missing_conditions([9042], [5800], [str(tmp_path / 'nope')], proc_root=str(tmp_path))
    assert missing == ['UDP port 9042', 'TCP port 5800', f'file {tmp_path / "nope"}']


def test_bound_ports_reads_the_real_system():
    # must not crash on the real machine and must return a set of ints
    assert all(isinstance(p, int) for p in w.bound_ports('udp'))


def test_runs_the_program_when_ready(tmp_path):
    marker = tmp_path / 'ready'
    marker.write_text('x')
    res = subprocess.run([sys.executable, '-m', 'sim_bringup.wait_then_exec', '--file', str(marker),
                          '--label', 't', '--', 'echo', 'started-ok'], capture_output=True, text=True, timeout=30)
    assert res.returncode == 0 and 'started-ok' in res.stdout and '[t] ready' in res.stdout


def test_gives_up_after_timeout(tmp_path):
    res = subprocess.run([sys.executable, '-m', 'sim_bringup.wait_then_exec', '--file', str(tmp_path / 'never'),
                          '--timeout', '0.5', '--', 'echo', 'should-not-run'], capture_output=True, text=True, timeout=30)
    assert res.returncode == 1 and 'should-not-run' not in res.stdout and 'GAVE UP' in res.stdout
