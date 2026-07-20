#!/usr/bin/env python3
"""Offline CLI for the batch orthomosaic builder — no ROS required.

Point it at any folder of mission photos (with or without .json sidecars)
and it writes <out>/<name>.jpg + .jgw + _report.json. Handy for re-running
mapping on a laptop from a copied mapping_photos folder.

Examples:
    python3 -m ortho_mapping.offline_map --dir ~/mapping_photos
    python3 -m ortho_mapping.offline_map --dir ./photos --alt 25 --gsd-cm 4
"""

import argparse
import os
import sys

from .config import (
    DEFAULT_ALTITUDE_AGL_M,
    DEFAULT_GSD_M,
    DEFAULT_HFOV_DEG,
    DEFAULT_MAX_CANVAS_MP,
    DEFAULT_MAX_REFINE_SHIFT_M,
    MappingConfig,
)
from .mapper import build_map


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Build one georeferenced map image from a folder of '
                    'GPS-tagged mission photos.')
    parser.add_argument('--dir', required=True, dest='images_dir',
                        help='folder containing the mission photos')
    parser.add_argument('--out', dest='output_dir', default=None,
                        help='output folder (default: <dir>/../mapping_output)')
    parser.add_argument('--name', dest='basename', default=None,
                        help='output basename (default: mapped_<timestamp>)')
    parser.add_argument('--gsd-cm', type=float, default=DEFAULT_GSD_M * 100.0,
                        help='output resolution, cm per pixel (default 3)')
    parser.add_argument('--alt', type=float, default=DEFAULT_ALTITUDE_AGL_M,
                        help='assumed AGL (m) for images without a sidecar '
                             f'(default {DEFAULT_ALTITUDE_AGL_M})')
    parser.add_argument('--hfov', type=float, default=DEFAULT_HFOV_DEG,
                        help='camera horizontal FOV in degrees (default 81)')
    parser.add_argument('--heading', default='auto',
                        choices=['auto', 'sidecar', 'track', 'fixed'],
                        help='heading source (default auto)')
    parser.add_argument('--fixed-heading', type=float, default=0.0,
                        help="heading used when --heading fixed")
    parser.add_argument('--yaw-offset', type=float, default=0.0,
                        help='mounting yaw correction in degrees (default 0)')
    parser.add_argument('--no-refine', action='store_true',
                        help='disable ECC seam refinement')
    parser.add_argument('--no-gimbal', action='store_true',
                        help='ignore gimbal attitude from sidecars')
    parser.add_argument('--max-shift', type=float,
                        default=DEFAULT_MAX_REFINE_SHIFT_M,
                        help='max refinement shift in metres (default 3)')
    parser.add_argument('--max-mp', type=float, default=DEFAULT_MAX_CANVAS_MP,
                        help='max canvas megapixels before coarsening GSD')
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    images_dir = os.path.abspath(os.path.expanduser(args.images_dir))
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(images_dir), 'mapping_output')
    output_dir = os.path.abspath(os.path.expanduser(output_dir))

    cfg = MappingConfig(
        images_dir=images_dir,
        output_dir=output_dir,
        output_basename=args.basename,
        gsd_m=args.gsd_cm / 100.0,
        hfov_deg=args.hfov,
        default_altitude_agl_m=args.alt,
        heading_source=args.heading,
        fixed_heading_deg=args.fixed_heading,
        yaw_offset_deg=args.yaw_offset,
        use_gimbal_attitude=not args.no_gimbal,
        refine=not args.no_refine,
        max_refine_shift_m=args.max_shift,
        max_canvas_mp=args.max_mp,
    )

    result = build_map(cfg, log=print)
    if not result.ok:
        print(f"ERROR: {result.message}", file=sys.stderr)
        for item in result.skipped[:20]:
            print(f"  skipped {item['file']}: {item['reason']}", file=sys.stderr)
        return 1
    print(result.message)
    return 0


if __name__ == '__main__':
    sys.exit(main())
