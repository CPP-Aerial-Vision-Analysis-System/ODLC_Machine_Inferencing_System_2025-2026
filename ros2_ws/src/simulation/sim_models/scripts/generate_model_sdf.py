#!/usr/bin/env python3
"""
generate_model_sdf.py -- PURPOSE: command line tool that turns the drone model source
(models/iris_a8mini/model.sdf.xacro) into a plain SDF file Gazebo can spawn.

Installed as:  ros2 run sim_models generate_model_sdf [options]

Examples:
    # default model (2K photos, SITL instance 0) to a file
    ros2 run sim_models generate_model_sdf -o /tmp/iris_a8mini.sdf
    # 4K photos, SITL instance 1 (ArduPilotPlugin on UDP 9012)
    ros2 run sim_models generate_model_sdf --camera-model a8mini --instance 1 -o /tmp/iris1.sdf
    # refresh the pre-generated models/iris_a8mini/model.sdf after editing the xacro
    ros2 run sim_models generate_model_sdf --update-default

Without -o / --update-default the SDF is printed to standard output.
Exit code 0 = written, 2 = bad arguments, 1 = xacro failed.
"""

import argparse
import os
import sys

from sim_models.model_generator import (
    CAMERA_PROFILES,
    DEFAULT_ARGS,
    default_xacro_path,
    generate_model_sdf,
)


def parse_args(argv):
    """Read the command line (see the module docstring for examples)."""
    d = DEFAULT_ARGS
    p = argparse.ArgumentParser(
        prog="generate_model_sdf",
        description="Generate the iris_a8mini SDF model from model.sdf.xacro.",
    )
    p.add_argument(
        "--camera-model",
        choices=sorted(CAMERA_PROFILES),
        default=d["camera_model"],
        help="photo profile: zr10 = 2560x1440 (default), a8mini = 3840x2160",
    )
    p.add_argument("--photo-width", type=int, default=0, help="photo width px (0 = from profile)")
    p.add_argument("--photo-height", type=int, default=0, help="photo height px (0 = from profile)")
    p.add_argument("--video-width", type=int, default=d["video_width"], help="live video width px")
    p.add_argument(
        "--video-height", type=int, default=d["video_height"], help="live video height px"
    )
    p.add_argument("--video-fps", type=int, default=d["video_fps"], help="live video frame rate Hz")
    p.add_argument(
        "--hfov", type=float, default=d["hfov"], help="horizontal field of view, radians"
    )
    p.add_argument(
        "--noise",
        type=float,
        default=d["camera_noise_stddev"],
        help="camera pixel noise standard deviation (0..1)",
    )
    port = p.add_mutually_exclusive_group()
    port.add_argument(
        "--instance",
        type=int,
        default=None,
        help="SITL instance number I (-I<I>): UDP port 9002 + 10*I",
    )
    port.add_argument("--fdm-port-in", type=int, default=None, help="ArduPilotPlugin UDP port")
    p.add_argument("--no-payloads", action="store_true", help="leave out the two payloads")
    p.add_argument(
        "--person-mass",
        type=float,
        default=d["payload_person_mass"],
        help="person drop (water bottle) mass kg",
    )
    p.add_argument(
        "--tent-mass", type=float, default=d["payload_tent_mass"], help="tent drop (beacon) mass kg"
    )
    p.add_argument(
        "--gimbal-initial-pitch",
        type=float,
        default=d["gimbal_initial_pitch"],
        help="gimbal pitch target at spawn, radians (-1.5708 = straight down)",
    )
    p.add_argument(
        "--xacro", default=None, help="model.sdf.xacro to use (default: the package one)"
    )
    out = p.add_mutually_exclusive_group()
    out.add_argument("-o", "--output", default=None, help="write the SDF to this file")
    out.add_argument(
        "--update-default",
        action="store_true",
        help="write model.sdf next to the xacro file (only with default arguments)",
    )
    return p.parse_args(argv)


def main(argv=None):
    """Generate the SDF and write/print it. Returns the process exit code."""
    a = parse_args(sys.argv[1:] if argv is None else argv)
    kwargs = dict(
        camera_model=a.camera_model,
        photo_width=a.photo_width,
        photo_height=a.photo_height,
        video_width=a.video_width,
        video_height=a.video_height,
        video_fps=a.video_fps,
        hfov=a.hfov,
        camera_noise_stddev=a.noise,
        instance=a.instance,
        fdm_port_in=a.fdm_port_in,
        payloads=not a.no_payloads,
        payload_person_mass=a.person_mass,
        payload_tent_mass=a.tent_mass,
        gimbal_initial_pitch=a.gimbal_initial_pitch,
    )
    xacro_path = os.path.realpath(a.xacro or default_xacro_path())
    if a.update_default:
        non_default = [
            k
            for k, v in kwargs.items()
            if k in DEFAULT_ARGS and v is not None and v != DEFAULT_ARGS[k]
        ]
        if non_default or a.instance is not None or a.fdm_port_in is not None:
            print(
                f"--update-default only works with the default arguments (changed: {non_default})",
                file=sys.stderr,
            )
            return 2
    try:
        sdf = generate_model_sdf(xacro_path, **kwargs)
    except ValueError as exc:
        print(f"generate_model_sdf: bad argument: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:  # noqa: BLE001 - xacro raises many kinds of errors
        print(f"generate_model_sdf: xacro failed on {xacro_path}: {exc}", file=sys.stderr)
        return 1
    target = a.output
    if a.update_default:
        target = os.path.join(os.path.dirname(xacro_path), "model.sdf")
    if target:
        with open(target, "w", encoding="utf-8") as f:
            f.write(sdf)
        print(f"wrote {target}", file=sys.stderr)
    else:
        sys.stdout.write(sdf)
    return 0


if __name__ == "__main__":
    sys.exit(main())
