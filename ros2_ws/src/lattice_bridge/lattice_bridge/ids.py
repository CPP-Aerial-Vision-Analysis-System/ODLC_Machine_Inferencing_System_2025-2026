"""
lattice_bridge/ids.py -- PURPOSE: make and read the names ("ids") of the things we publish to Lattice.
Lattice identifies an entity only by its entityId: publish the same id again and you UPDATE that entity,
publish a new id and you CREATE another one. So ids must be (1) stable for the same thing during a mission,
(2) different for different things and (3) never reused by a later mission (an old track must not be
overwritten by a new flight). Every id has the form

    astra-<mission>-<what>        <mission> = e.g. runway20261024t153012z (see make_mission_id)

    astra-<mission>-uav                      our drone
    astra-<mission>-tgt-person-1             first person target of this mission
    astra-<mission>-tgt-tent-2               second tent target
    astra-<mission>-drop-person-1            where the first person payload was released
    astra-<mission>-area-search-boundary-1   a search area (name made lowercase, '_' -> '-')

The mission id has only lowercase letters and digits (no '-'), which is what makes every id readable by
parse_entity_id() without guessing. Pure Python (re, datetime): no ROS, no I/O, no clock reads (the mission
start time is passed in).
"""

from __future__ import annotations

import re
from datetime import datetime
from typing import NamedTuple, Optional

from . import constants as C
from .timeutil import require_aware

_MISSION_ID = re.compile(r'[a-z][a-z0-9]{0,31}')
_NUMBER = re.compile(r'[1-9][0-9]{0,5}')
_PARSE = re.compile(
    r'astra-(?P<mission>[a-z][a-z0-9]{0,31})-'
    r'(?:(?P<uav>uav)'
    r'|tgt-(?P<tkind>person|tent)-(?P<tn>[1-9][0-9]{0,5})'
    r'|drop-(?P<dkind>person|tent)-(?P<dn>[1-9][0-9]{0,5})'
    r'|area-(?P<area>[a-z0-9]+(?:-[a-z0-9]+)*))'
)
_MAX_AREA_SLUG = 40
_MAX_LABEL = 12


class ParsedId(NamedTuple):
    """What parse_entity_id() found. `role` is 'uav', 'target', 'drop' or 'area'."""
    mission: str
    role: str
    kind: Optional[str] = None      # 'person' or 'tent' for targets and drops
    number: Optional[int] = None    # 1, 2, 3 ... for targets and drops
    area: Optional[str] = None      # the area slug for areas


def _alnum(text: str) -> str:
    """Lowercase letters and digits only (everything else dropped)."""
    return re.sub(r'[^a-z0-9]', '', str(text).lower())


def make_mission_id(start_time: datetime, label: Optional[str] = None) -> str:
    """Build a mission id from the mission start time (aware) and an optional short label.

    Example: make_mission_id(2026-10-24 15:30:12 UTC, 'runway') -> 'runway20261024t153012z'. Without a label
    the id starts with 'm'. The seconds make two missions started in the same minute (a simulator restart)
    still different. The label keeps only lowercase letters and digits and at most 12 of them.
    """
    moment = require_aware('start_time', start_time)
    stamp = moment.strftime('%Y%m%dt%H%M%Sz')
    prefix = _alnum(label)[:_MAX_LABEL] if label is not None else ''
    if not prefix or not prefix[0].isalpha():
        prefix = 'm' + prefix
    return prefix + stamp


def check_mission_id(mission_id) -> str:
    """Return the mission id if it is valid ('abc123': lowercase letter first, then letters/digits, <= 32)."""
    if not isinstance(mission_id, str) or _MISSION_ID.fullmatch(mission_id) is None:
        raise ValueError(f'mission id must be lowercase letters and digits starting with a letter '
                         f'(<= 32 chars), got {str(mission_id)[:40]!r}')
    return mission_id


def _check_kind(kind) -> str:
    if kind not in C.TARGET_KINDS:
        raise ValueError(f'kind must be one of {C.TARGET_KINDS}, got {str(kind)[:20]!r}')
    return kind


def _check_number(number) -> int:
    if isinstance(number, bool) or not isinstance(number, int) or _NUMBER.fullmatch(str(number)) is None:
        raise ValueError(f'number must be an integer from 1 to 999999, got {str(number)[:20]!r}')
    return number


def area_slug(area_name: str) -> str:
    """'search_boundary_1' -> 'search-boundary-1': lowercase, letters/digits, single '-' between words."""
    if not isinstance(area_name, str):
        raise ValueError('area name must be a string')
    words = re.findall(r'[a-z0-9]+', area_name.lower())
    slug = '-'.join(words)[:_MAX_AREA_SLUG].strip('-')
    if not slug:
        raise ValueError(f'area name has no letters or digits: {area_name[:40]!r}')
    return slug


def uav_entity_id(mission_id: str) -> str:
    """The drone: astra-<mission>-uav."""
    return f'astra-{check_mission_id(mission_id)}-uav'


def target_entity_id(mission_id: str, kind: str, number: int) -> str:
    """A target track: astra-<mission>-tgt-<person|tent>-<n> (n counts from 1 per kind)."""
    return f'astra-{check_mission_id(mission_id)}-tgt-{_check_kind(kind)}-{_check_number(number)}'


def drop_entity_id(mission_id: str, kind: str, number: int = 1) -> str:
    """A drop point: astra-<mission>-drop-<person|tent>-<n>."""
    return f'astra-{check_mission_id(mission_id)}-drop-{_check_kind(kind)}-{_check_number(number)}'


def area_entity_id(mission_id: str, area_name: str) -> str:
    """A search area or boundary: astra-<mission>-area-<slug of the name>."""
    return f'astra-{check_mission_id(mission_id)}-area-{area_slug(area_name)}'


def parse_entity_id(entity_id) -> Optional[ParsedId]:
    """Read an id made by this module back into its parts, or None when it is not one of ours."""
    if not isinstance(entity_id, str):
        return None
    match = _PARSE.fullmatch(entity_id)
    if match is None:
        return None
    mission = match.group('mission')
    if match.group('uav'):
        return ParsedId(mission, 'uav')
    if match.group('tkind'):
        return ParsedId(mission, 'target', match.group('tkind'), int(match.group('tn')))
    if match.group('dkind'):
        return ParsedId(mission, 'drop', match.group('dkind'), int(match.group('dn')))
    return ParsedId(mission, 'area', area=match.group('area'))


def target_display_name(kind: str, number: int) -> str:
    """The name people see in the Lattice UI (aliases.name), e.g. 'Person 1 (ASTRA)'."""
    return f'{C.LABEL_BY_KIND[_check_kind(kind)]} {_check_number(number)} (ASTRA)'
