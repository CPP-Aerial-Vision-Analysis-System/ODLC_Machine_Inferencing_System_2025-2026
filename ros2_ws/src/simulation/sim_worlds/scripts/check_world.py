#!/usr/bin/env python3
"""
sim_worlds/scripts/check_world.py -- PURPOSE: prove that a world is healthy, headless, in about 30 seconds.

It loads the world with `gz sim -s -r` in its own partition (sim_worlds/gzrun.py) and checks:
  1. the log has no [Err] lines (Gazebo errors) and the server keeps running,
  2. every world system of the simulation contract was loaded (Physics, UserCommands, SceneBroadcaster,
     Sensors, Imu, NavSat),
  3. the real-time factor (how fast simulated time runs compared with the wall clock),
  4. GPS: temporary NavSat sensors at the world origin and on every target must report the expected
     latitude/longitude: the origin must equal config/<world>.yaml `home` (= SITL --home), and each target its
     truth-file lat_wgs84/lon_wgs84 (the ArduPilot-convention lat/lon is printed too: ~1 m apart at most).

usage:  check_world.py runway | suas_field | path/to/world.sdf  [--seconds 12] [--log DIR] [--json FILE]
exit code 0 = all checks passed.
"""

import argparse
import json
import math
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(os.path.dirname(HERE), 'sim_worlds')):
    sys.path.insert(0, os.path.dirname(HERE))              # running from the source tree

from sim_worlds import geo, truth as truthmod            # noqa: E402
from sim_worlds.gzrun import HeadlessGazebo, inject, navsat_model_sdf   # noqa: E402

CONTRACT_SYSTEMS = ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors', 'Imu', 'NavSat')
NAVSAT_TOLERANCE_M = 0.5


def home_from_sdf(text):
    """geo.Home from a world's <spherical_coordinates> (None when missing)."""
    def tag(name):
        m = re.search(rf'<{name}>\s*([-0-9.eE+]+)\s*</{name}>', text)
        return float(m.group(1)) if m else None
    lat, lon, ele = tag('latitude_deg'), tag('longitude_deg'), tag('elevation')
    head = tag('heading_deg') or 0.0
    return None if lat is None or lon is None else geo.Home(lat, lon, ele or 0.0, head)


def latlon_error_m(lat, lon, lat0, lon0):
    """Horizontal distance in metres between two nearby WGS84 points (local flat approximation)."""
    north = (lat - lat0) * 111132.95
    east = (lon - lon0) * 111412.84 * math.cos(math.radians(lat0))
    return math.hypot(north, east)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', help='world name (runway, suas_field) or a .sdf path')
    ap.add_argument('--seconds', type=float, default=12.0, help='simulated seconds to run (default 12)')
    ap.add_argument('--timeout', type=float, default=180.0, help='wall-clock limit in seconds (default 180)')
    ap.add_argument('--log', default=None, help='folder for the gz server log (default: temp folder)')
    ap.add_argument('--json', default=None, help='also write the results as JSON to this file')
    args = ap.parse_args(argv)

    cfg = truth = None
    if args.world.endswith('.sdf'):
        world_path = args.world
    else:
        from sim_worlds.world_config import load_world_config
        cfg = load_world_config(args.world)
        world_path = cfg.world_file
        if os.path.isfile(cfg.truth):
            truth = truthmod.load_truth(cfg.truth)
    with open(world_path, encoding='utf-8') as f:
        text = f.read()
    sdf_home = home_from_sdf(text)
    probes = {'origin': (0.0, 0.0)}
    if truth:
        for t in truth['targets']:
            probes[t['id']] = (t['x'], t['y'])
    world_text = inject(text, [navsat_model_sdf(n, x, y, 0.5) for n, (x, y) in probes.items()])
    log_dir = args.log or os.path.join('/tmp', 'sim_worlds_check')
    os.makedirs(log_dir, exist_ok=True)
    results = {'world': args.world, 'world_file': world_path, 'checks': {}, 'navsat': {}, 'problems': []}
    readings, stats, lock = {}, {'rtf': [], 'sim': 0.0}, threading.Lock()
    t0 = time.time()
    with HeadlessGazebo(world_path, world_text, os.path.join(log_dir, f'check_{os.path.basename(world_path)}.log'),
                        verbosity=4) as gz:
        from gz.transport13 import Node                       # noqa: E402 (needs the partition)
        from gz.msgs10.navsat_pb2 import NavSat              # noqa: E402
        from gz.msgs10.world_stats_pb2 import WorldStatistics  # noqa: E402

        def nav_cb_for(name):
            def cb(msg):
                with lock:
                    readings[name] = (msg.latitude_deg, msg.longitude_deg, msg.altitude)
            return cb

        def stats_cb(msg):
            with lock:
                stats['rtf'].append(msg.real_time_factor)
                stats['sim'] = msg.sim_time.sec + 1e-9 * msg.sim_time.nsec

        node = Node()
        for n in probes:
            node.subscribe(NavSat, f'/check/{n}/navsat', nav_cb_for(n))
        node.subscribe(WorldStatistics, f'/world/{gz.world_name}/stats', stats_cb)
        try:
            while time.time() - t0 < args.timeout and gz.alive():
                with lock:
                    if stats['sim'] >= args.seconds and len(readings) == len(probes):
                        break
                time.sleep(0.25)
        except KeyboardInterrupt:
            results['problems'].append('interrupted')
        alive = gz.alive()
        log = gz.log_text()

    # --- evaluate ---------------------------------------------------------------------------------------------
    errors = [ln.strip() for ln in log.splitlines() if '[Err]' in ln]
    loaded = sorted(set(re.findall(r'Loaded system \[gz::sim::systems::(\w+)\]', log)))
    missing = [s for s in CONTRACT_SYSTEMS if s not in loaded]
    rtf = sorted(stats['rtf'][len(stats['rtf']) // 3:]) or [0.0]
    results['checks'] = {'server_alive_until_end': alive, 'errors_in_log': len(errors), 'error_lines': errors[:20],
                         'systems_loaded': loaded, 'contract_systems_missing': missing,
                         'sim_seconds': round(stats['sim'], 2), 'wall_seconds': round(time.time() - t0, 1),
                         'real_time_factor_median': round(rtf[len(rtf) // 2], 3),
                         'real_time_factor_min': round(rtf[0], 3)}
    if not alive:
        results['problems'].append('gz sim stopped before the end of the check')
    if errors:
        results['problems'].append(f'{len(errors)} [Err] line(s) in the log')
    if missing:
        results['problems'].append(f'contract systems not loaded: {missing}')
    if stats['sim'] < args.seconds * 0.9:
        results['problems'].append(f'simulation only reached {stats["sim"]:.1f} s')
    home = cfg.home if cfg else sdf_home
    if cfg and sdf_home and (abs(cfg.home.lat - sdf_home.lat) > 1e-7 or abs(cfg.home.lon - sdf_home.lon) > 1e-7
                             or abs(cfg.home.alt - sdf_home.alt) > 0.05):
        results['problems'].append(f'config home {cfg.home.sitl_string()} != world <spherical_coordinates> '
                                   f'{sdf_home.sitl_string()}')
    for name, (x, y) in probes.items():
        if name not in readings:
            results['problems'].append(f'no NavSat reading from probe {name}')
            continue
        lat, lon, alt = readings[name]
        exp84 = geo.enu_to_wgs84(x, y, 0.5, home)
        expap = geo.enu_to_latlon(x, y, home)
        e84 = latlon_error_m(lat, lon, exp84[0], exp84[1])
        eap = latlon_error_m(lat, lon, expap[0], expap[1])
        results['navsat'][name] = {'xy': [round(x, 2), round(y, 2)], 'navsat_lat': round(lat, 8),
                                   'navsat_lon': round(lon, 8), 'navsat_alt': round(alt, 2),
                                   'expected_wgs84': [round(exp84[0], 8), round(exp84[1], 8), round(exp84[2], 2)],
                                   'error_vs_wgs84_m': round(e84, 3),
                                   'ardupilot_convention': [round(expap[0], 8), round(expap[1], 8)],
                                   'difference_to_ardupilot_convention_m': round(eap, 3)}
        if e84 > NAVSAT_TOLERANCE_M:
            results['problems'].append(f'NavSat at {name} is {e84:.2f} m from the expected WGS84 position')
        if name == 'origin' and abs(alt - (home.alt + 0.5)) > 0.5:
            results['problems'].append(f'NavSat altitude at the origin {alt:.2f} != home alt + 0.5 = {home.alt + 0.5:.2f}')

    # --- report -------------------------------------------------------------------------------------------------
    c = results['checks']
    print(f'world {args.world}: sim {c["sim_seconds"]} s in {c["wall_seconds"]} s wall, real-time factor median '
          f'{c["real_time_factor_median"]} (min {c["real_time_factor_min"]})')
    print(f'  systems loaded: {", ".join(loaded) or "none found in the log"}')
    print(f'  [Err] lines: {len(errors)}' + (''.join(f'\n    {e}' for e in errors[:10])))
    for name, r in results['navsat'].items():
        print(f'  NavSat {name:>12} at ({r["xy"][0]:8.2f}, {r["xy"][1]:8.2f}): lat {r["navsat_lat"]:.7f} lon '
              f'{r["navsat_lon"]:.7f} alt {r["navsat_alt"]:.2f}  |  error vs WGS84 {r["error_vs_wgs84_m"]:.3f} m, '
              f'vs ArduPilot convention {r["difference_to_ardupilot_convention_m"]:.3f} m')
    print('RESULT: ' + ('PASS' if not results['problems'] else 'FAIL: ' + '; '.join(results['problems'])))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2)
    return 0 if not results['problems'] else 1


if __name__ == '__main__':
    sys.exit(main())
