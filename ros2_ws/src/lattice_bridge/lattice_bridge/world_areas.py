"""
lattice_bridge/world_areas.py -- PURPOSE: read the search areas and the flight boundary out of the
simulator's world files and hand them over as plain latitude/longitude corner lists, ready for
entities.build_area_geo. It knows how the two worlds describe their areas:

  * sim_worlds/config/<world>.yaml has `home: "lat,lon,alt_msl,heading"` and `search_areas:`, a list of
    rectangles in LOCAL METRES (east, north of home): center [x, y], size [along the lanes, across the
    lanes], yaw (radians; 0 = lanes run east, 1.5708 = north). Example: the "search_box" of the runway world.
  * sim_worlds/truth/<world>.yaml (the answer key) also has `flight_boundary: {latlon: [[lat, lon], ...]}`
    (suas_field only) and `altitude_rule_agl_m: {min, max}`.

On the real field the same job is done by whoever prepares the mission: any list of (lat, lon) corners
works with build_area_geo; this module is only the bridge from the simulator's files.

The functions take ALREADY PARSED dicts (load the YAML yourself), so this module needs no YAML library and
touches no files. Rectangles are turned into corners with ArduPilot's flat-earth formula (geo.offset_latlon),
the same one that made the `corners_latlon` lists in the truth files; test_world_areas.py checks that the two
agree to a few centimetres. Corners are returned counter-clockwise starting at the corner behind-left of the
lane direction.

Pure Python (math): no ROS, no I/O.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Mapping, Optional, Sequence, Tuple

from . import constants as C
from .geo import LatLon, offset_latlon, require_lat_lon, require_number


@dataclass(frozen=True)
class AreaSpec:
    """One map area: a name, a note for humans, its corners and the Lattice geo type to use."""
    name: str
    note: str
    corners_latlon: Tuple[LatLon, ...]
    geo_type: str


def parse_home(home) -> Tuple[float, float]:
    """(lat, lon) of the world's home from 'lat,lon,alt,heading' text or a {lat, lon, ...} mapping."""
    try:
        if isinstance(home, str):
            parts = [p.strip() for p in home.split(',')]
            if len(parts) < 2:
                raise ValueError('needs at least latitude,longitude')
            lat, lon = float(parts[0]), float(parts[1])
        elif isinstance(home, Mapping):
            lat, lon = home['lat'], home['lon']
        else:
            raise ValueError('expected text or a mapping')
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError(f'home must look like "36.2143,-96.0068,210.0,0" or {{lat:, lon:}}: {exc}') from None
    return require_lat_lon(lat, lon, 'home')


def rectangle_corners(home_lat: float, home_lon: float, center_xy: Sequence[float],
                      size: Sequence[float], yaw_rad: float = 0.0) -> List[LatLon]:
    """The four corners (lat, lon), counter-clockwise, of a rectangle given in local metres around home.

    `size` = [along, across] where "along" runs in the direction `yaw_rad` (0 = east, pi/2 = north).
    """
    try:
        cx, cy = (require_number('center x', center_xy[0]), require_number('center y', center_xy[1]))
        along, across = (require_number('size along', size[0], 0.0), require_number('size across', size[1], 0.0))
    except (TypeError, IndexError):
        raise ValueError('center and size must each be a pair of numbers') from None
    if along <= 0.0 or across <= 0.0:
        raise ValueError('size must be greater than 0 in both directions')
    yaw = require_number('yaw', yaw_rad, -2 * math.pi * 10, 2 * math.pi * 10)
    ux, uy = math.cos(yaw), math.sin(yaw)               # along the lanes
    vx, vy = -uy, ux                                    # across (90 degrees counter-clockwise from along)
    corners = []
    for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        east = cx + sa * along / 2.0 * ux + sb * across / 2.0 * vx
        north = cy + sa * along / 2.0 * uy + sb * across / 2.0 * vy
        corners.append(offset_latlon(home_lat, home_lon, east, north))
    return corners


def search_areas(world_config: Mapping[str, Any]) -> List[AreaSpec]:
    """All `search_areas` of a parsed config/<world>.yaml as AreaSpecs (geo type: search area)."""
    home_lat, home_lon = parse_home(world_config.get('home'))
    areas = world_config.get('search_areas') or []
    if not isinstance(areas, Sequence) or isinstance(areas, (str, bytes)):
        raise ValueError('search_areas must be a list')
    result = []
    for index, item in enumerate(areas):
        if not isinstance(item, Mapping) or not item.get('name'):
            raise ValueError(f'search_areas[{index}] needs a name')
        corners = rectangle_corners(home_lat, home_lon, item.get('center'), item.get('size'),
                                    item.get('yaw', 0.0))
        result.append(AreaSpec(str(item['name']), str(item.get('note', '')), tuple(corners),
                               C.GEO_TYPE_SEARCH_AREA))
    return result


def flight_boundary(truth: Mapping[str, Any]) -> Optional[AreaSpec]:
    """The flight boundary of a parsed truth/<world>.yaml as an AreaSpec (control area), or None if it has none.

    The altitude rule (`altitude_rule_agl_m`), when present, goes into the note so people see it in Lattice.
    """
    boundary = truth.get('flight_boundary')
    if not boundary:
        return None
    points = boundary.get('latlon') if isinstance(boundary, Mapping) else None
    if not points or not isinstance(points, Sequence) or isinstance(points, (str, bytes)):
        raise ValueError('flight_boundary needs a latlon list')
    corners = []
    for index, point in enumerate(points):
        try:
            lat, lon = point
        except (TypeError, ValueError):
            raise ValueError(f'flight_boundary.latlon[{index}] must be a (lat, lon) pair') from None
        corners.append(require_lat_lon(lat, lon, f'flight boundary corner {index}'))
    note = 'flight boundary'
    rule = truth.get('altitude_rule_agl_m')
    if isinstance(rule, Mapping) and 'min' in rule and 'max' in rule:
        note += f": fly {require_number('altitude min', rule['min']):.1f}-" \
                f"{require_number('altitude max', rule['max']):.1f} m above ground"
    return AreaSpec('flight_boundary', note, tuple(corners), C.GEO_TYPE_FLIGHT_BOUNDARY)
