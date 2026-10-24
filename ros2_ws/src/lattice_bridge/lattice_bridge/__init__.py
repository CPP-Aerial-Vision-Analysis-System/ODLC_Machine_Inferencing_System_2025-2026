"""
lattice_bridge/__init__.py -- PURPOSE: the Python package of the ASTRA <-> Anduril Lattice bridge. Phase 1 (this
code) is the CORE LIBRARY: pure functions and small classes with no ROS, no network and no clock reads, so
everything is testable on any computer and can never slow down or crash a flight node.

The public names are re-exported here; the README table lists what each one does.

    from lattice_bridge import build_uav_asset, validate_entity, TargetClusterer, encode_tgt, decode_tgt

Importing this package is cheap and has no side effects.
"""

from .clustering import (
    ACTION_CREATE, ACTION_IGNORED, ACTION_NO_CHANGE, ACTION_UPDATE, Decision, TargetClusterer, TargetState,
    state_track_kwargs,
)
from .entities import (
    ALTITUDE_NOTE, ALTITUDE_WARNING, EntityValidationError, altitude_is_approximate, build_area_geo,
    build_drop_point, build_target_track, build_uav_asset, confidence_to_track_quality, find_problems,
    hae_from_amsl, validate_entity,
)
from .ids import (
    area_entity_id, drop_entity_id, make_mission_id, parse_entity_id, target_display_name,
    target_entity_id, uav_entity_id,
)
from .logutil import warn_if_altitude_approximate, warn_once
from .report_codec import TgtReport, decode_tgt, encode_tgt, next_seq
from .sinks import Sink
from .sinks.jsonl_sink import JsonlSink, Record, latest_by_entity_id, read_records
from .timeutil import format_utc, from_unix_seconds, parse_utc
from .world_areas import AreaSpec, flight_boundary, search_areas

__version__ = '0.1.0'

__all__ = [
    'ACTION_CREATE', 'ACTION_IGNORED', 'ACTION_NO_CHANGE', 'ACTION_UPDATE', 'ALTITUDE_NOTE',
    'ALTITUDE_WARNING', 'AreaSpec', 'Decision', 'EntityValidationError', 'JsonlSink', 'Record', 'Sink',
    'TargetClusterer', 'TargetState', 'TgtReport', 'altitude_is_approximate', 'area_entity_id',
    'build_area_geo', 'build_drop_point', 'build_target_track', 'build_uav_asset',
    'confidence_to_track_quality', 'decode_tgt', 'drop_entity_id', 'encode_tgt', 'find_problems',
    'flight_boundary', 'format_utc', 'from_unix_seconds', 'hae_from_amsl', 'latest_by_entity_id',
    'make_mission_id', 'next_seq', 'parse_entity_id', 'parse_utc', 'read_records', 'search_areas',
    'state_track_kwargs', 'target_display_name', 'target_entity_id', 'uav_entity_id', 'validate_entity',
    'warn_if_altitude_approximate', 'warn_once',
]
