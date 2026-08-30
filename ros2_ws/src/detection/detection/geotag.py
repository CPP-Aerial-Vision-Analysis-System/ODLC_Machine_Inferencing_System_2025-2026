"""GPS coordinates carried in capture filenames.

siyi_node._generate_gps_filename() names each capture "<lat> , <lon>.jpg";
new_od reads the fix back out so main_controller can place the drop
waypoints. Kept in its own module, with no dependency beyond the standard
library, so the contract can be unit-tested without ROS, OpenCV or CUDA.
"""

import os
from typing import Optional, Tuple

DELIMITER = ' , '


def parse_latlon_from_name(filename: str) -> Tuple[Optional[float], Optional[float]]:
    """Recover (lat, lon) from a '<lat> , <lon>.jpg' filename.

    Returns (None, None) for simulation / SD-card names that carry no
    coordinates, or when either half is not a number.
    """
    stem = os.path.splitext(filename)[0]
    if DELIMITER not in stem:
        return None, None
    lat_str, lon_str = stem.split(DELIMITER, 1)
    try:
        return float(lat_str), float(lon_str)
    except ValueError:
        return None, None
