"""
sim_worlds/realfield/site.py -- PURPOSE: every number that describes the REAL SUAS 2026 field (Skyway36
droneport next to the Osage Casino, north Tulsa, Oklahoma) and the world frame of the "suas_field_real"
world, in one place, with where each number comes from. No GIS libraries needed (only sim_worlds.geo).

WORLD FRAME (the same convention as every sim_worlds world):
  origin (0, 0, 0) = centre of the Runway 2 VTOL pad, on the ground  = SITL home = <spherical_coordinates>
  x = metres EAST, y = metres NORTH, z = metres UP above the pad (z = 0 at the pad).
  Real-world things (lidar, aerial photo, handbook GPS points) are placed with the exact WGS84 local tangent
  plane (geo.wgs84_to_enu), which is what Gazebo's NavSat uses, so the terrain, the photo and the search
  boundaries line up with each other to well under a metre. Heights: z = (NAVD88 height) - (home height);
  the earth's curvature (13 cm at 1.3 km) and the NAD83/WGS84 difference (about 1 m, PROJ treats it as 0)
  are ignored on purpose: ArduPilot SITL ignores them too.

WHY THE ORIGIN MOVED: the procedural "suas_field" world uses 36.2143 N, 96.0068 W as "the pad", but that point
is on the east edge of the paved strip, about 50 m south of the real pad (site_facts research). Here the
origin is the pad itself: 6.5 m south of the strip's north end, on the strip axis (OpenStreetMap way
203824325), which agrees with the pad drawn in the handbook's Appendix A (36.21476 N, 96.00670 W, +-5 m).
"""

import math

from sim_worlds import geo

WORLD = 'suas_field_real'
DEFAULT_SEED = 2026

# ------------------------------------------------------------------------------------------------------------
# The paved strip 02/20 (OpenStreetMap way 203824325 "runway, disused, ref 2; 20"), south end -> north end
# ------------------------------------------------------------------------------------------------------------
STRIP_SOUTH_LATLON = (36.2070321, -96.0102000)
STRIP_NORTH_LATLON = (36.2147961, -96.0066982)
STRIP_HEADING_DEG = 20.08          #: true heading of the strip axis (computed from the two OSM nodes)
STRIP_WIDTH_M = 18.3               #: visible slab width measured on NAIP (site_facts.md section 3)

# ------------------------------------------------------------------------------------------------------------
# SUAS runways and tents (handbook Appendix A screenshot, georeferenced by the site_facts research, +-4..5 m)
# ------------------------------------------------------------------------------------------------------------
PAD_LATLON = (36.2147411, -96.0067230)     #: Runway 2 = 40 x 40 ft VTOL pad = WORLD ORIGIN (see module text)
PAD_SIZE_M = 12.2                          #: 40 ft
RUNWAY1_CENTER_LATLON = (36.2135743, -96.0072493)  #: Appendix A centre (36.213579, -96.007265) snapped onto the strip axis
RUNWAY1_LENGTH_M, RUNWAY1_WIDTH_M = 152.4, 12.2    #: 500 x 40 ft (handbook)
TENT_PINS_LATLON = {                       #: "Runway 1 Tents" / "Runway 2 Tents" pins of Appendix A
    'runway_1_tents': (36.214799, -96.006559),
    'runway_2_tents': (36.214136, -96.006848),
}

# ------------------------------------------------------------------------------------------------------------
# Competition geometry from the RoboNation SUAS 2026 Team Handbook (identical to build_suas_field.py)
# ------------------------------------------------------------------------------------------------------------
FLIGHT_BOUNDARY_LATLON = [
    (36.2193143935478, -96.00171089172362), (36.2203876816226, -96.0041356086731),
    (36.2208031440207, -96.006817817688), (36.2205954130975, -96.0094571113586),
    (36.2198164172223, -96.01181745529171), (36.2181891563676, -96.0142421722412),
    (36.2149951257415, -96.0109269618988), (36.2128397305797, -96.0107660293579),
    (36.2128051052733, -96.00916743278502), (36.2106063669373, -96.00924253463749),
    (36.2106583063506, -96.00173234939579),
]
SEARCH_BOUNDARIES_LATLON = {       #: revised 2026-09-14 ("New search boundaries"), about 10 acres each
    'search_boundary_1': {'lat_min': 36.211822, 'lat_max': 36.213147, 'lon_min': -96.005545, 'lon_max': -96.002627},
    'search_boundary_2': {'lat_min': 36.213469, 'lat_max': 36.215795, 'lon_min': -96.004286, 'lon_max': -96.002635},
}
MIN_AGL_M, MAX_AGL_M = 45.7, 121.9         #: 150 ft .. 400 ft above the ground (handbook)

#: Grass colour of competition week (15-17 Sept 2026) relative to the NAIP photo day (18 Sept 2023): the grass
#: was greener and darker after rain on 10-13 Sept. Ratio of Sentinel-2 grass reflectance 2026-09-15 / 2023-09-18
#: over pure NAIP-grass pixels (site_facts.md section 6.3). Applied to grass pixels only (realfield/pack.py).
GRASS_GRADE_RGB = (0.77, 0.88, 0.79)

#: Sun of the NAIP photo (its shadows point north-west: about 11:00 CDT) = competition day 15 Sept 2026 at
#: 11:00 CDT (site_facts.md section 6.1): azimuth 127.9 deg, elevation 43.7 deg. The world's sun uses the
#: same direction, so the shadows painted in the photo and the shadows Gazebo draws point the same way.
SUN_DIRECTION_ENU = (-0.570, 0.444, -0.691)
ELEVATION_RULE_FT = (633.0, 748.0)         #: "between 633 ft MSL and 748 ft MSL, runway roughly 690 ft" (handbook)

#: Area of interest of the research downloads (flight boundary + about 150 m)
AOI_LAT = (36.2093, 36.2222)
AOI_LON = (-96.0160, -95.9999)

# ------------------------------------------------------------------------------------------------------------
# Ground changes since the lidar was flown (February 2017). The elevation research (elevation_lidar.md section
# 5) compared the lidar with the 2023 and 2025 photos: the DTM must be patched in two places. World metres.
# ------------------------------------------------------------------------------------------------------------
DTM_EDITS = [
    {'name': 'soil_piles_levelled', 'kind': 'level', 'box': (345.0, -500.0, 455.0, -300.0), 'window_m': 41.0,
     'feather_m': 10.0,
     'why': 'three soil piles 9.7, 6.4 and 5.4 m tall (about 16,400 m3) south-east of Search Boundary 1 in the '
            '2017 lidar are GONE (levelled pasture) in NAIP 2023, NAIP 2025 and OKMaps 2025. Recipe of the '
            'elevation research (v08): grey opening of the DTM with a 41 m window inside this box, which removes '
            'bumps narrower than 41 m and keeps the natural slope.'},
    {'name': 'new_wetland', 'kind': 'lower', 'center': (53.1, -213.9), 'radii': (42.0, 36.0), 'depth_m': 0.8,
     'why': 'a wetland basin was dug west of Search Boundary 1 after 2017 (bare excavation with an excavator in '
            'NAIP 2023, wet ground in March 2025, green in September 2026). Its depth is NOT known: 0.8 m is an '
            'estimate (the research suggested 0.5 to 1.5 m). Smooth bowl, deepest in the middle.'},
]

# ------------------------------------------------------------------------------------------------------------
# Terrain layout (world metres). 7 x 7 blocks of 264 m = 1848 m square around the flight boundary (90 m or
# more of margin on every side of the research area of interest). A block far from the action is ONE coarse
# tile; a block near it is split into 132 m quads, and a quad that touches a "detail" zone into 66 m tiles.
# Every tile is a 65 x 65 vertex grid before simplification:
#   far  264 m tile, 4.125 m grid    near  132 m, 2.0625 m grid    detail / near66  66 m, 1.03125 m grid
# Why 66 m and this exact EXTENT corner: both search boundaries are a little LONGER than 256 m (262.4 m and
# 258.2 m). With 66 m tiles whose edges sit just outside the boundaries, each boundary needs only 4 x 3 tiles
# (instead of 5 x 4 with 64 m tiles), which saves about 30 % of the sharp, memory-hungry detail pictures.
# ------------------------------------------------------------------------------------------------------------
TILE_M = 66.0
BLOCK_M = 4 * TILE_M                       #: 264 m
EXTENT = (105.1 - 4 * BLOCK_M, -144.1 - 2 * BLOCK_M,      #: x_min, y_min, x_max, y_max of the terrain
          105.1 + 3 * BLOCK_M, -144.1 + 5 * BLOCK_M)      #: = (-950.9, -672.1, 897.1, 1175.9)
GRID_N = 65                                #: vertices per tile side (64 cells) for every tile size
NEAR_ZONE_MARGIN_M = 100.0                 #: blocks within this distance of a search boundary / the pad are split
DETAIL_MARGIN_M = 0.0                      #: 66 m tiles overlapping a search boundary (+ this margin) are "detail"
PAD_DETAIL_RADIUS_M = 30.0                 #: ... and those within this distance of the pad (take-off / landing view)

#: per tile kind: tile size (m), simplification tolerance (m), picture pixels (standard / hd quality), whether
#: CC0 centimetre detail is added. Grid spacing = size / 64. Pictures: detail 6.4 cm/px (standard) or 3.2 cm/px
#: (hd, the default: about the camera's own ground pixel at 46-60 m), near66 and near 26 cm/px (= the NAIP
#: photo), far 1 m/px. Tolerance 5 cm in the search areas: smaller would mostly keep the lidar's +-5 cm noise.
TILE_KINDS = {
    'detail': {'size': TILE_M, 'tolerance': 0.05, 'tex_px': 1024, 'tex_px_hd': 2048, 'micro': True},
    'near66': {'size': TILE_M, 'tolerance': 0.10, 'tex_px': 256, 'tex_px_hd': 256, 'micro': False},
    'near': {'size': 2 * TILE_M, 'tolerance': 0.10, 'tex_px': 512, 'tex_px_hd': 512, 'micro': False},
    'far': {'size': BLOCK_M, 'tolerance': 0.25, 'tex_px': 256, 'tex_px_hd': 256, 'micro': False},
}
COLLISION_TOLERANCE = {'detail': 0.08, 'near66': 0.15, 'near': 0.25, 'far': 0.60}


def home_at(alt_msl):
    """geo.Home of the world origin (the pad) at ground height alt_msl (NAVD88 metres from the DTM)."""
    return geo.Home(PAD_LATLON[0], PAD_LATLON[1], round(float(alt_msl), 2), 0.0)


def world_xy(lat, lon, home):
    """WGS84 (lat, lon) -> world (x east, y north) metres on the exact local tangent plane (what Gazebo uses)."""
    x, y, _ = geo.wgs84_to_enu(lat, lon, home.alt, home)
    return x, y


def world_latlon(x, y, home):
    """World (x, y) metres -> exact WGS84 (lat, lon). Inverse of world_xy()."""
    lat, lon, _ = geo.enu_to_wgs84(x, y, 0.0, home)
    return lat, lon


def local_proj4():
    """PROJ string of the world frame for GIS tools (Transverse Mercator on WGS84 centred on the pad, scale 1):
    over this 1.8 km field it matches the Gazebo/ENU tangent plane to well under 1 cm horizontally."""
    return (f'+proj=tmerc +lat_0={PAD_LATLON[0]} +lon_0={PAD_LATLON[1]} +k=1 +x_0=0 +y_0=0 '
            '+datum=WGS84 +units=m +no_defs')


def flight_boundary_xy(home):
    """The 11 flight-boundary vertices in world metres (WGS84 tangent plane)."""
    return [world_xy(lat, lon, home) for lat, lon in FLIGHT_BOUNDARY_LATLON]


def search_box_xy(name, home):
    """A search boundary as (x_min, y_min, x_max, y_max) world metres (its corners are on lat/lon lines)."""
    b = SEARCH_BOUNDARIES_LATLON[name]
    x0, y0 = world_xy(b['lat_min'], b['lon_min'], home)
    x1, y1 = world_xy(b['lat_max'], b['lon_max'], home)
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def strip_axis_xy(home):
    """(south_end_xy, north_end_xy, unit_vector_south_to_north) of the paved strip in world metres."""
    s = world_xy(*STRIP_SOUTH_LATLON, home)
    n = world_xy(*STRIP_NORTH_LATLON, home)
    dx, dy = n[0] - s[0], n[1] - s[1]
    length = math.hypot(dx, dy)
    return s, n, (dx / length, dy / length)


def _box_dist(x0, y0, x1, y1, box):
    """Shortest distance (m) between two axis-aligned rectangles (0 when they touch or overlap)."""
    bx0, by0, bx1, by1 = box
    dx = max(bx0 - x1, x0 - bx1, 0.0)
    dy = max(by0 - y1, y0 - by1, 0.0)
    return math.hypot(dx, dy)


def _overlaps(x0, y0, x1, y1, box, margin):
    """True when the rectangle overlaps `box` grown by `margin` metres with a POSITIVE area (touching edges
    do not count, so a tile that only shares an edge with a search boundary does not become a detail tile)."""
    bx0, by0, bx1, by1 = box[0] - margin, box[1] - margin, box[2] + margin, box[3] + margin
    return min(x1, bx1) - max(x0, bx0) > 1e-6 and min(y1, by1) - max(y0, by0) > 1e-6


def tile_layout(home):
    """The terrain tiles: list of dicts {name, kind, x0, y0, x1, y1} (world metres), sorted by name.

    Rules (see the TERRAIN LAYOUT comment above): a 264 m block more than NEAR_ZONE_MARGIN_M from both search
    boundaries and the pad stays one 'far' tile; otherwise it is split into four 132 m 'near' quads; a quad
    that overlaps a search boundary (or comes within PAD_DETAIL_RADIUS_M of the pad) is split again into four
    66 m tiles, each 'detail' when it overlaps that zone itself, else 'near66'.
    """
    sbs = [search_box_xy(n, home) for n in SEARCH_BOUNDARIES_LATLON]
    pad_r = PAD_DETAIL_RADIUS_M
    pad_box = (-pad_r, -pad_r, pad_r, pad_r)
    x_min, y_min, x_max, y_max = EXTENT
    nbx = int(round((x_max - x_min) / BLOCK_M))
    nby = int(round((y_max - y_min) / BLOCK_M))
    half = TILE_M * 2.0

    def near_block(x0, y0, x1, y1):
        return any(_box_dist(x0, y0, x1, y1, b) <= NEAR_ZONE_MARGIN_M for b in sbs + [(0.0, 0.0, 0.0, 0.0)])

    def detail_zone(x0, y0, x1, y1):
        return any(_overlaps(x0, y0, x1, y1, b, DETAIL_MARGIN_M) for b in sbs) or \
            _overlaps(x0, y0, x1, y1, pad_box, 0.0)

    tiles = []
    for j in range(nby):
        for i in range(nbx):
            bx0, by0 = x_min + i * BLOCK_M, y_min + j * BLOCK_M
            bx1, by1 = bx0 + BLOCK_M, by0 + BLOCK_M
            if not near_block(bx0, by0, bx1, by1):
                tiles.append({'name': f'b{i}{j}', 'kind': 'far', 'x0': bx0, 'y0': by0, 'x1': bx1, 'y1': by1})
                continue
            for qj in range(2):
                for qi in range(2):
                    qx0, qy0 = bx0 + qi * half, by0 + qj * half
                    qx1, qy1 = qx0 + half, qy0 + half
                    if not detail_zone(qx0, qy0, qx1, qy1):
                        tiles.append({'name': f'b{i}{j}q{qi}{qj}', 'kind': 'near',
                                      'x0': qx0, 'y0': qy0, 'x1': qx1, 'y1': qy1})
                        continue
                    for tj in range(2):
                        for ti in range(2):
                            tx0, ty0 = qx0 + ti * TILE_M, qy0 + tj * TILE_M
                            tx1, ty1 = tx0 + TILE_M, ty0 + TILE_M
                            kind = 'detail' if detail_zone(tx0, ty0, tx1, ty1) else 'near66'
                            tiles.append({'name': f'b{i}{j}q{qi}{qj}t{ti}{tj}', 'kind': kind,
                                          'x0': tx0, 'y0': ty0, 'x1': tx1, 'y1': ty1})
    for t in tiles:                                    # round away float noise (EXTENT has one decimal)
        for k in ('x0', 'y0', 'x1', 'y1'):
            t[k] = round(t[k], 4)
    return sorted(tiles, key=lambda t: t['name'])


def near_zone_contains(x, y, home, margin=NEAR_ZONE_MARGIN_M):
    """True when (x, y) is within `margin` m of a search boundary or of the pad (used for tree detail levels)."""
    sbs = [search_box_xy(n, home) for n in SEARCH_BOUNDARIES_LATLON]
    return any(_box_dist(x, y, x, y, b) <= margin for b in sbs) or math.hypot(x, y) <= margin
