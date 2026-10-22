"""
sim_bringup/wait_then_exec.py -- PURPOSE: wait until a network port is open (or a file exists), then start a program.

Why it exists: the simulator's programs must start in a fixed order, or they hang:
  1. Gazebo must be running WITH the drone loaded before ArduCopter SITL starts (the drone's
     ArduPilotPlugin opens UDP port 9002+10*I; SITL started earlier gets stuck on "Duplicate input frame").
  2. SITL must be listening on TCP 5760+10*I before MAVProxy connects (MAVProxy exits otherwise).
sim.launch.py therefore starts SITL as  `wait_then_exec --udp-port 9002 -- arducopter ...`  and MAVProxy as
`wait_then_exec --tcp-port 5760 -- mavproxy.py ...`.

How it checks: it reads the kernel's socket tables (/proc/net/udp, /proc/net/tcp and their IPv6 twins)
instead of connecting, because a test connection to SITL's port 5760 would itself count as "the first
client" and wake SITL up too early. When everything is ready it REPLACES itself with the program
(os.execvp), so the launch system's Ctrl+C reaches the program directly.

Usage:
  ros2 run sim_bringup wait_then_exec [--udp-port P] [--tcp-port P] [--file PATH]
                                      [--timeout SECONDS] [--delay SECONDS] [--label NAME] -- PROGRAM ARGS...
Exit codes: never returns on success (the program runs in its place); 1 = timeout, 2 = bad usage.
"""

import argparse
import os
import sys
import time
from typing import Iterable, List, Set

#: TCP state "LISTEN" in /proc/net/tcp (hex). UDP sockets have no listen state: bound is enough.
_TCP_LISTEN = '0A'


def _local_ports(table_text: str, tcp: bool) -> Set[int]:
    """Parse one /proc/net/{tcp,udp}[6] table and return the set of bound/listening local ports.

    Each data line looks like:  `  12: 0100007F:2328 00000000:0000 07 ...`  (local addr:port in hex, state).
    For TCP only LISTEN sockets count; for UDP every bound socket counts.
    """
    ports: Set[int] = set()
    for line in table_text.splitlines()[1:]:          # first line is the column header
        cols = line.split()
        if len(cols) < 4 or ':' not in cols[1]:
            continue
        if tcp and cols[3] != _TCP_LISTEN:
            continue
        try:
            ports.add(int(cols[1].rsplit(':', 1)[1], 16))
        except ValueError:
            continue
    return ports


def bound_ports(proto: str, proc_root: str = '/proc/net') -> Set[int]:
    """Return every local port that is bound (udp) or listening (tcp), IPv4 and IPv6 together.

    proto: 'udp' or 'tcp'. proc_root can point elsewhere for tests.
    """
    if proto not in ('udp', 'tcp'):
        raise ValueError("proto must be 'udp' or 'tcp'")
    ports: Set[int] = set()
    for name in (proto, proto + '6'):
        try:
            with open(os.path.join(proc_root, name), 'r', encoding='ascii', errors='replace') as f:
                ports |= _local_ports(f.read(), tcp=(proto == 'tcp'))
        except OSError:
            continue                                  # IPv6 table missing: fine
    return ports


def missing_conditions(udp_ports: Iterable[int], tcp_ports: Iterable[int], files: Iterable[str],
                       proc_root: str = '/proc/net') -> List[str]:
    """List the conditions that are NOT met yet, in words (empty list = everything ready)."""
    missing: List[str] = []
    udp_ports, tcp_ports = list(udp_ports), list(tcp_ports)
    if udp_ports:
        have = bound_ports('udp', proc_root)
        missing += [f'UDP port {p}' for p in udp_ports if p not in have]
    if tcp_ports:
        have = bound_ports('tcp', proc_root)
        missing += [f'TCP port {p}' for p in tcp_ports if p not in have]
    missing += [f'file {f}' for f in files if not os.path.exists(f)]
    return missing


def main(argv=None) -> int:
    """Command-line entry point (see the module docstring)."""
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--' in argv:
        split = argv.index('--')
        own, command = argv[:split], argv[split + 1:]
    else:
        own, command = argv, []
    parser = argparse.ArgumentParser(prog='wait_then_exec', description=__doc__.split('\n\n')[0])
    parser.add_argument('--udp-port', type=int, action='append', default=[], help='wait until this UDP port is bound')
    parser.add_argument('--tcp-port', type=int, action='append', default=[], help='wait until this TCP port listens')
    parser.add_argument('--file', action='append', default=[], help='wait until this file exists')
    parser.add_argument('--timeout', type=float, default=180.0, help='give up after this many seconds (default 180)')
    parser.add_argument('--delay', type=float, default=0.0, help='extra seconds to wait once everything is ready')
    parser.add_argument('--label', default='wait_then_exec', help='name used in the messages')
    args = parser.parse_args(own)
    if not command:
        parser.error('give the program to start after "--"')
        return 2

    tag = f'[{args.label}]'
    start = time.monotonic()
    last_report = 0.0
    while True:
        missing = missing_conditions(args.udp_port, args.tcp_port, args.file)
        elapsed = time.monotonic() - start
        if not missing:
            break
        if elapsed >= args.timeout:
            print(f'{tag} GAVE UP after {elapsed:.0f} s: still waiting for {", ".join(missing)}. '
                  f'Not starting {command[0]}.', flush=True)
            return 1
        if elapsed - last_report >= 10.0 or last_report == 0.0:
            print(f'{tag} waiting for {", ".join(missing)} before starting {os.path.basename(command[0])} '
                  f'({elapsed:.0f} s)', flush=True)
            last_report = elapsed if elapsed > 0 else 0.001
        time.sleep(0.2)
    if args.delay > 0:
        time.sleep(args.delay)
    print(f'{tag} ready after {time.monotonic() - start:.1f} s, starting: {" ".join(command)}', flush=True)
    try:
        os.execvp(command[0], command)
    except OSError as exc:
        print(f'{tag} could not start {command[0]}: {exc}', flush=True)
        return 1
    return 0  # not reached


if __name__ == '__main__':
    sys.exit(main())
