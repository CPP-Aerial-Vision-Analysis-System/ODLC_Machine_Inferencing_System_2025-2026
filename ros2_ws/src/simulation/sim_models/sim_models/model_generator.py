"""
sim_models/model_generator.py -- PURPOSE: turn the drone model source models/iris_a8mini/model.sdf.xacro
into a plain SDF file that Gazebo can spawn, with the arguments you choose.

Why a generator? One xacro file serves every case:
  * camera profile: photo resolution 2560 x 1440 ("zr10", DEFAULT, what last year's video_cam expects)
    or 3840 x 2160 ("a8mini", the A8 mini's own 4K),
  * SITL instance: ArduPilotPlugin must listen on UDP port 9002 + 10 * instance,
  * with or without the payloads, payload masses, initial gimbal pitch, video size/rate, lens FOV.
The bringup can call plain `xacro` itself (the contract) or use this module / the
`ros2 run sim_models generate_model_sdf` tool, which also checks the arguments.

Public functions:
    default_xacro_path()        where model.sdf.xacro is (installed package, else source tree)
    fdm_port_for_instance(i)    9002 + 10 * i
    photo_size(camera_model)    (width, height) of a camera profile
    build_mappings(...)         checked xacro arguments as {name: string}
    generate_model_sdf(...)     the SDF text
"""

import os
import xml.dom.minidom

# Photo resolution of each camera profile: (width, height) in pixels.
CAMERA_PROFILES = {
    "zr10": (2560, 1440),  # 2K: last year's code targets the ZR10 2K profile (DEFAULT)
    "a8mini": (3840, 2160),  # 4K: the A8 mini's maximum photo resolution
}

# Default value of every xacro argument (must match model.sdf.xacro).
DEFAULT_ARGS = {
    "camera_model": "zr10",
    "photo_width": 0,  # 0 = from camera_model
    "photo_height": 0,
    "video_width": 1280,
    "video_height": 720,
    "video_fps": 25,
    "hfov": 1.4137,  # rad, 81 deg
    "camera_noise_stddev": 0.005,
    "fdm_port_in": 9002,
    "payloads": True,
    "payload_person_mass": 0.5,
    "payload_tent_mass": 0.12,
    "gimbal_initial_pitch": 0.0,
}

SITL_BASE_FDM_PORT = 9002
SITL_PORT_STEP = 10


def fdm_port_for_instance(instance: int) -> int:
    """Return the ArduPilotPlugin UDP port for SITL instance `instance` (-I<instance>): 9002 + 10 * i."""
    if instance < 0:
        raise ValueError(f"SITL instance must be >= 0 (got {instance})")
    return SITL_BASE_FDM_PORT + SITL_PORT_STEP * int(instance)


def photo_size(camera_model: str):
    """Return (width, height) of the photo camera for a camera profile ('zr10' or 'a8mini')."""
    try:
        return CAMERA_PROFILES[camera_model]
    except KeyError:
        raise ValueError(
            f"unknown camera_model {camera_model!r}; use one of {sorted(CAMERA_PROFILES)}"
        )


def default_xacro_path() -> str:
    """Return the path of model.sdf.xacro.

    Looks in the installed package first (share/sim_models/models/iris_a8mini/), then in the
    source tree next to this file (so tests and tools also work before `colcon build`).
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(
            get_package_share_directory("sim_models"), "models", "iris_a8mini", "model.sdf.xacro"
        )
        if os.path.isfile(path):
            return path
    except Exception:  # noqa: BLE001 - not installed / no ament index: use the source tree
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), "models", "iris_a8mini", "model.sdf.xacro")


def _positive_int(name, value):
    try:
        v = int(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be an integer (got {value!r})")
    if v < 0:
        raise ValueError(f"{name} must be >= 0 (got {v})")
    return v


def _positive_float(name, value, allow_zero=False):
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a number (got {value!r})")
    if v < 0.0 or (v == 0.0 and not allow_zero):
        raise ValueError(f"{name} must be > 0 (got {v})")
    return v


def build_mappings(**kwargs) -> dict:
    """Check the model arguments and return them as xacro mappings {name: string}.

    Accepts any key of DEFAULT_ARGS (missing ones take the default) plus `instance` (a SITL
    instance number, sets fdm_port_in = 9002 + 10 * instance unless fdm_port_in is also given).

    Raises:
        ValueError: on an unknown key or a bad value (with a message that says what is wrong).
    """
    unknown = set(kwargs) - set(DEFAULT_ARGS) - {"instance"}
    if unknown:
        raise ValueError(
            f"unknown model argument(s): {sorted(unknown)}; known: {sorted(DEFAULT_ARGS)}"
        )
    args = dict(DEFAULT_ARGS)
    instance = kwargs.pop("instance", None)
    args.update({k: v for k, v in kwargs.items() if v is not None})
    if instance is not None and kwargs.get("fdm_port_in") is None:
        args["fdm_port_in"] = fdm_port_for_instance(int(instance))

    photo_size(args["camera_model"])  # validates the profile name
    for key in ("photo_width", "photo_height"):
        args[key] = _positive_int(key, args[key])
    if (args["photo_width"] == 0) != (args["photo_height"] == 0):
        raise ValueError(
            "give both photo_width and photo_height, or neither (0 = from camera_model)"
        )
    for key in ("video_width", "video_height", "video_fps"):
        args[key] = _positive_int(key, args[key])
        if args[key] == 0:
            raise ValueError(f"{key} must be > 0")
    args["hfov"] = _positive_float("hfov", args["hfov"])
    if args["hfov"] >= 3.14:
        raise ValueError(f"hfov is in radians and must be < pi (got {args['hfov']})")
    args["camera_noise_stddev"] = _positive_float(
        "camera_noise_stddev", args["camera_noise_stddev"], allow_zero=True
    )
    port = _positive_int("fdm_port_in", args["fdm_port_in"])
    if not 1024 <= port <= 65535:
        raise ValueError(f"fdm_port_in must be a UDP port 1024..65535 (got {port})")
    args["fdm_port_in"] = port
    if isinstance(args["payloads"], str):
        if args["payloads"].lower() not in ("true", "false", "1", "0"):
            raise ValueError(f"payloads must be true or false (got {args['payloads']!r})")
        args["payloads"] = args["payloads"].lower() in ("true", "1")
    args["payloads"] = bool(args["payloads"])
    args["payload_person_mass"] = _positive_float(
        "payload_person_mass", args["payload_person_mass"]
    )
    args["payload_tent_mass"] = _positive_float("payload_tent_mass", args["payload_tent_mass"])
    args["gimbal_initial_pitch"] = float(args["gimbal_initial_pitch"])
    if not -2.3562 <= args["gimbal_initial_pitch"] <= 0.7854:
        raise ValueError("gimbal_initial_pitch must be inside the pitch limits -2.3562..0.7854 rad")

    out = {}
    for key, value in args.items():
        out[key] = ("true" if value else "false") if isinstance(value, bool) else str(value)
    return out


def resolved_photo_size(mappings: dict):
    """Return the (width, height) the photo camera will get for these mappings."""
    w, h = int(mappings.get("photo_width", 0)), int(mappings.get("photo_height", 0))
    if w > 0 and h > 0:
        return w, h
    return photo_size(mappings.get("camera_model", "zr10"))


def _header_text(mappings: dict) -> str:
    w, h = resolved_photo_size(mappings)
    lines, line = [], "   "
    for item in (f"{k}:={v}" for k, v in sorted(mappings.items())):
        if len(line) + len(item) > 88:
            lines.append(line)
            line = "   "
        line += " " + item
    lines.append(line)
    # XML comments must not contain two hyphens in a row, so none are used here.
    return (
        "\n  model.sdf: GENERATED FILE, do not edit by hand.\n"
        "  PURPOSE: the simulated ASTRA drone for Gazebo Harmonic (ArduPilot Iris + SIYI A8 mini\n"
        "  gimbal with a live video camera and a triggered photo camera + two droppable payloads).\n"
        "  Source: sim_models/models/iris_a8mini/model.sdf.xacro (read that file and README.md).\n"
        "  Regenerate: ros2 run sim_models generate_model_sdf [options] (see README.md).\n"
        f"  Photo camera: {w} x {h}.  ArduPilotPlugin UDP port: {mappings['fdm_port_in']}.\n"
        "  Arguments:\n" + "\n".join(lines) + "\n"
    )


_MODEL_NOTE = (
    " canonical_link: the Iris body must stay the model's reference link, because\n"
    "       ArduPilotPlugin and OdometryPublisher report the pose of the canonical link. "
)


def generate_model_sdf(xacro_path: str = None, **kwargs) -> str:
    """Run xacro on model.sdf.xacro and return the SDF text.

    Args:
        xacro_path: the .xacro file (default: default_xacro_path()).
        **kwargs: model arguments, see build_mappings() / DEFAULT_ARGS.

    Returns:
        The SDF document as a string, starting with a "GENERATED FILE" comment that lists the
        arguments used.

    Raises:
        ValueError: bad arguments.  ImportError: xacro is not installed (apt: ros-humble-xacro).
    """
    import xacro  # imported here so the rest of the module works without xacro

    path = xacro_path or default_xacro_path()
    mappings = build_mappings(**kwargs)
    doc = xacro.process_file(path, mappings=mappings)
    # Replace the source file's leading comment (written for the .xacro reader) with a header
    # that says this is a generated file and how it was made.
    factory = xml.dom.minidom.Document()
    root = doc.documentElement
    for node in list(doc.childNodes):
        if node.nodeType == node.COMMENT_NODE:
            doc.removeChild(node)
    doc.insertBefore(factory.createComment(_header_text(mappings)), root)
    # xacro leaves the trailing comments of the argument/property lines behind as orphans
    # directly under <sdf>; drop them and keep one useful note in front of <model>.
    for node in list(root.childNodes):
        if node.nodeType == node.COMMENT_NODE:
            root.removeChild(node)
    model = next((n for n in root.childNodes if n.nodeType == n.ELEMENT_NODE), None)
    if model is not None:
        root.insertBefore(factory.createComment(_MODEL_NOTE), model)
    return doc.toprettyxml(indent="  ")
