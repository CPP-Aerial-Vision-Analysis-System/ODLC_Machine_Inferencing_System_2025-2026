"""PURPOSE: Measure local 2027 report diagnostics against a private scene answer key.

This is NOT RoboNation's unspecified submission schema or official score. Coordinates are local ENU
metres. Landing-zone suitability comes from the explicitly labeled training policy, not aviation advice.
Map quality, retardant performance, operator count and real-flight safety are outside this evaluator.
"""

import argparse
import json
import math
from pathlib import Path

from sim_worlds.challenge2027.scenario import SCHEMA

REPORT_SCHEMA = 'astra.suas2027.report.v1'


def _mapping(value, name):
    """Reject JSON values that cannot represent an object."""
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be an object')
    return value


def _number(value, name, minimum=None):
    """Boolean, missing, infinite and NaN coordinates must never produce a successful metric."""
    try:
        finite = math.isfinite(value) if isinstance(value, (int, float)) else False
    except OverflowError:
        finite = False
    if isinstance(value, bool) or not finite:
        raise ValueError(f'{name} must be a finite number')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be at least {minimum}')
    return float(value)


def _xy(value, name):
    """Read two ENU coordinates with explicit diagnostics for malformed reports."""
    _mapping(value, name)
    return tuple(_number(value.get(k), f'{name}.{k}') for k in ('x', 'y'))


def _distance(a, b):
    """Horizontal distance; target radius in the primer does not specify a 3-D slant distance."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def evaluate_report(scene, report):
    """Return honest diagnostics, including unknown values; never turn missing evidence into a pass."""
    _mapping(scene, 'scene')
    _mapping(report, 'report')
    if scene.get('schema') != SCHEMA or report.get('schema') != REPORT_SCHEMA:
        raise ValueError(f'expected {SCHEMA} scene and {REPORT_SCHEMA} report')
    if 'hiker' not in scene or 'hiker' not in report:
        raise ValueError('scene and report must explicitly contain hiker (object or null)')
    target, observed = scene['hiker'], report['hiker']
    truth_xy = _xy(target, 'scene.hiker') if target is not None else None
    observed_xy = _xy(observed, 'report.hiker') if observed is not None else None
    attributes = {}
    for key in ('shirt_color', 'pants_color', 'suitcase_color'):
        if target is not None and not isinstance(target.get(key), str):
            raise ValueError(f'scene.hiker.{key} must be a color name')
        value = observed.get(key) if observed is not None else None
        if value is not None and not isinstance(value, str):
            raise ValueError(f'report.hiker.{key} must be a color name or null')
        attributes[key] = (value.strip().lower() == target[key].strip().lower()
                           if value is not None and target is not None else None)
    hiker = dict(expected=target is not None, reported=observed is not None,
                 false_alarm=target is None and observed is not None,
                 missed=target is not None and observed is None,
                 location_error_m=_distance(truth_xy, observed_xy) if truth_xy and observed_xy else None,
                 attributes_correct=attributes)
    zones = scene.get('landing_zones')
    if not isinstance(zones, list):
        raise ValueError('scene.landing_zones must be a list')
    by_id, distances = {}, {}
    for zone in zones:
        _mapping(zone, 'landing zone')
        name = zone.get('id')
        if not isinstance(name, str) or not name or name in by_id:
            raise ValueError('landing zones must have unique nonempty string IDs')
        xy = _xy(zone, f'landing zone {name}')
        if not isinstance(zone.get('valid'), bool):
            raise ValueError(f'landing zone {name}.valid must be boolean')
        by_id[name] = zone
        if truth_xy and zone['valid']:
            distances[name] = _distance(truth_xy, xy)
    nearest = min(distances.values(), default=None)
    best = sorted(name for name, distance in distances.items()
                  if nearest is not None and math.isclose(distance, nearest, abs_tol=1e-6, rel_tol=0))
    selected = report.get('landing_zone_id')
    if selected is not None and (not isinstance(selected, str) or selected not in by_id):
        raise ValueError('report.landing_zone_id must identify a scene candidate or be null')
    zone = by_id.get(selected)
    landing = dict(selected_id=selected, valid=zone['valid'] if zone else None,
                   nearest_valid_ids=best, best_selected=selected in best if best else None,
                   distance_to_hiker_m=_distance(truth_xy, _xy(zone, 'selected zone')) if truth_xy and zone else None,
                   policy='local training criteria, not official helicopter suitability')
    delivery = None
    if report.get('delivery') is not None:
        value = _mapping(report['delivery'], 'report.delivery')
        xy = _xy(value, 'report.delivery')
        altitude = value.get('release_agl_m')
        if altitude is not None:
            altitude = _number(altitude, 'report.delivery.release_agl_m', 0)
        rules = _mapping(scene.get('delivery'), 'scene.delivery')
        radius = _number(rules.get('target_radius_m'), 'scene.delivery.target_radius_m', 0)
        floor = _number(rules.get('min_agl_m'), 'scene.delivery.min_agl_m', 0)
        error = _distance(truth_xy, xy) if truth_xy else None
        delivery = dict(error_m=error, within_target_radius=error <= radius if error is not None else None,
                        release_agl_m=altitude, above_minimum_agl=altitude >= floor if altitude is not None else None,
                        retardant_verified=None, payload_survival_verified=None,
                        altitude_source='reported, not verified telemetry')
    elapsed = report.get('elapsed_s')
    if elapsed is not None:
        elapsed = _number(elapsed, 'report.elapsed_s', 0)
    return dict(schema='astra.suas2027.diagnostics.v1', official_score=False,
                world_name=scene.get('world_name'), hiker=hiker, landing_zone=landing, delivery=delivery,
                elapsed_s=elapsed, within_45min=elapsed <= 2700 if elapsed is not None else None,
                map_quality_evaluated=False)


def main(argv=None):
    """Read a private answer key and a locally defined report; print diagnostics or a clear input error."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('report', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args(argv)
    try:
        result = evaluate_report(json.loads(args.scene.read_text(encoding='utf-8')),
                                 json.loads(args.report.read_text(encoding='utf-8')))
        text = json.dumps(result, indent=2, allow_nan=False) + '\n'
        if args.out:
            args.out.write_text(text, encoding='utf-8')
        else:
            print(text, end='')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
