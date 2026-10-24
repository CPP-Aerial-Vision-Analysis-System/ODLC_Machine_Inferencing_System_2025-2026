"""
lattice_bridge/test/test_constants.py -- PURPOSE: guard the one module that holds every fixed word. The expected
enum lists below are typed in a SECOND time, from the Lattice REST reference (PublishEntity, read 2026-10-24),
so an accidental edit of constants.py shows up as a failing test instead of as a rejected request later.
"""

import re
from pathlib import Path

import pytest

from lattice_bridge import constants as C

PKG_SRC = Path(__file__).resolve().parent.parent / 'lattice_bridge'

DOC_TEMPLATES = {'TEMPLATE_TRACK', 'TEMPLATE_SENSOR_POINT_OF_INTEREST', 'TEMPLATE_ASSET', 'TEMPLATE_GEO',
                 'TEMPLATE_SIGNAL_OF_INTEREST'}
DOC_DISPOSITIONS = {'DISPOSITION_UNKNOWN', 'DISPOSITION_FRIENDLY', 'DISPOSITION_HOSTILE',
                    'DISPOSITION_SUSPICIOUS', 'DISPOSITION_ASSUMED_FRIENDLY', 'DISPOSITION_NEUTRAL',
                    'DISPOSITION_PENDING'}
DOC_ENVIRONMENTS = {'ENVIRONMENT_UNKNOWN', 'ENVIRONMENT_AIR', 'ENVIRONMENT_SURFACE',
                    'ENVIRONMENT_SUB_SURFACE', 'ENVIRONMENT_LAND', 'ENVIRONMENT_SPACE'}
DOC_GEO_TYPES = {'GEO_TYPE_GENERAL', 'GEO_TYPE_HAZARD', 'GEO_TYPE_EMERGENCY', 'GEO_TYPE_ENGAGEMENT_ZONE',
                 'GEO_TYPE_CONTROL_AREA', 'GEO_TYPE_BULLSEYE', 'GEO_TYPE_ACM'}
DOC_CONNECTION = {'CONNECTION_STATUS_ONLINE', 'CONNECTION_STATUS_OFFLINE'}
DOC_HEALTH = {'HEALTH_STATUS_HEALTHY', 'HEALTH_STATUS_WARN', 'HEALTH_STATUS_FAIL', 'HEALTH_STATUS_OFFLINE',
              'HEALTH_STATUS_NOT_READY'}


@pytest.mark.parametrize('ours, documented', [
    (C.TEMPLATES, DOC_TEMPLATES), (C.DISPOSITIONS, DOC_DISPOSITIONS), (C.ENVIRONMENTS, DOC_ENVIRONMENTS),
    (C.GEO_TYPES, DOC_GEO_TYPES), (C.CONNECTION_STATUSES, DOC_CONNECTION), (C.HEALTH_STATUSES, DOC_HEALTH),
])
def test_enum_sets_match_the_documented_lists(ours, documented):
    assert set(ours) == documented


def test_no_enum_contains_the_invalid_zero_value():
    for group in (C.TEMPLATES, C.DISPOSITIONS, C.ENVIRONMENTS, C.GEO_TYPES, C.CONNECTION_STATUSES,
                  C.HEALTH_STATUSES):
        assert not [word for word in group if word.endswith('INVALID')]


def test_every_enum_constant_equals_its_own_name():
    """DISPOSITION_FRIENDLY == 'DISPOSITION_FRIENDLY': a typo in either half would break JSON silently."""
    prefixes = ('TEMPLATE_', 'DISPOSITION_', 'ENVIRONMENT_', 'GEO_TYPE_', 'CONNECTION_STATUS_', 'HEALTH_STATUS_')
    checked = 0
    for name in dir(C):
        value = getattr(C, name)
        if isinstance(value, str) and name.startswith(prefixes) and name.upper() == name:
            if name.startswith('GEO_TYPE_') and name in ('GEO_TYPE_SEARCH_AREA', 'GEO_TYPE_FLIGHT_BOUNDARY',
                                                         'GEO_TYPE_DROP_POINT'):
                continue                                  # aliases of another constant, tested below
            assert value == name, name
            checked += 1
    assert checked >= 25


def test_geo_type_aliases_point_at_real_geo_types():
    for value in (C.GEO_TYPE_SEARCH_AREA, C.GEO_TYPE_FLIGHT_BOUNDARY, C.GEO_TYPE_DROP_POINT):
        assert value in C.GEO_TYPES
    assert C.GEO_TYPE_FLIGHT_BOUNDARY == 'GEO_TYPE_CONTROL_AREA'
    assert C.GEO_TYPE_SEARCH_AREA == 'GEO_TYPE_GENERAL'


def test_the_publish_entity_body_has_39_top_level_keys_all_camel_case():
    assert len(C.ENTITY_TOP_LEVEL_KEYS) == 39
    assert all(re.fullmatch(r'[a-z][A-Za-z0-9]*', key) for key in C.ENTITY_TOP_LEVEL_KEYS)
    for key in ('entityId', 'isLive', 'expiryTime', 'provenance', 'aliases', 'ontology', 'location', 'milView',
                'geoShape', 'geoDetails', 'health', 'status', 'tracked'):
        assert key in C.ENTITY_TOP_LEVEL_KEYS


def test_our_choices_are_valid_lattice_words():
    assert C.UAV_DISPOSITION == 'DISPOSITION_FRIENDLY' and C.UAV_ENVIRONMENT == 'ENVIRONMENT_AIR'
    assert C.TARGET_DISPOSITION == 'DISPOSITION_NEUTRAL' and C.TARGET_ENVIRONMENT == 'ENVIRONMENT_SURFACE'
    assert C.INTEGRATION_NAME == 'astra_odlc'
    assert set(C.PLATFORM_TYPE_BY_KIND) == set(C.TARGET_KINDS) == set(C.LABEL_BY_KIND)
    assert C.PLATFORM_TYPE_BY_KIND == {'person': 'Person', 'tent': 'Tent'}
    assert C.PLATFORM_TYPE_UAV == 'UAV'
    assert len(C.DATA_TYPES) == 4


def test_platform_type_names_live_in_constants_only():
    """'Person', 'Tent' and 'UAV' must be typed in ONE place so confirming them in the Registry is one edit."""
    for path in sorted(PKG_SRC.rglob('*.py')):
        if path.name == 'constants.py':
            continue
        text = path.read_text(encoding='utf-8')
        for word in ("'Person'", '"Person"', "'Tent'", '"Tent"', "'UAV'", '"UAV"'):
            assert word not in text, f'{word} found in {path.name}; use constants.PLATFORM_TYPE_*'


def test_time_limits_are_consistent_with_the_lattice_rules():
    assert C.MAX_EXPIRY < C.LATTICE_MAX_EXPIRY
    assert C.LATTICE_MAX_EXPIRY.days == 30
    assert C.UAV_TTL_S == 120.0
    assert 0 < C.TARGET_EXPIRY_MARGIN_S < C.MAX_EXPIRY.total_seconds()
    assert 0 < C.AREA_TTL_S < C.MAX_EXPIRY.total_seconds()


def test_radio_message_constants():
    assert C.STATUSTEXT_MAX_CHARS == 50
    assert C.STATUSTEXT_SEVERITY_INFO == 6
    assert C.TGT_PREFIX == 'TGT'
    assert C.TGT_CODE_BY_KIND == {'person': 'p', 'tent': 't'}
    assert C.TGT_KIND_BY_CODE == {'p': 'person', 't': 'tent'}


def test_health_thresholds_are_ordered():
    assert C.BATTERY_FAIL_BELOW_PERCENT < C.BATTERY_WARN_BELOW_PERCENT
    assert C.LINK_OK_MAX_AGE_S < C.LINK_WARN_MAX_AGE_S
