"""
test/test_model_generation.py -- PURPOSE: check that model.sdf.xacro produces the drone the simulation
contract describes, for both camera profiles, and that the committed default model.sdf is up to date.

What is checked (no simulator needed; `gz sdf -k` runs too when gz and ardupilot_gazebo are found):
  * names: model iris_a8mini, canonical link, gimbal joints/axes/limits, camera frames
  * topics: every /sim/... topic of the contract (gimbal, joint states, odometry, cameras,
    photo trigger, payload servos, payload detach)
  * camera settings: resolution per profile, 25 Hz video, 81 deg FOV, clip 0.05..3000 m, triggered photo
  * ArduPilotPlugin: UDP port per SITL instance, motor channels 0..3, COMMAND channels 10/12
  * payloads: masses, DetachableJoint wiring, placement (under the body, above the leg tips)
  * argument checking of sim_models.model_generator
  * models/iris_a8mini/model.sdf == the xacro output with default arguments

Run:  python3 -m pytest -q test/test_model_generation.py   (needs ROS 2 sourced for xacro)
"""

import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

from sim_models import model_generator as mg  # noqa: E402

XACRO = os.path.join(PKG, "models", "iris_a8mini", "model.sdf.xacro")
DEFAULT_SDF = os.path.join(PKG, "models", "iris_a8mini", "model.sdf")

xacro_mod = pytest.importorskip("xacro", reason="xacro (ros-humble-xacro) is needed")


def generate(**kwargs):
    """Return the parsed <model> element for these model arguments."""
    text = mg.generate_model_sdf(XACRO, **kwargs)
    root = ET.fromstring(text)
    assert root.tag == "sdf"
    models = root.findall("model")
    assert len(models) == 1
    return models[0]


@pytest.fixture(scope="module")
def model():
    return generate()


def plugins(model, name):
    return [p for p in model.findall("plugin") if p.get("name") == name]


def txt(el, path):
    found = el.find(path)
    assert found is not None, f"missing <{path}>"
    return (found.text or "").strip()


# ------------------------------------------------------------------ structure
def test_model_name_include_and_canonical_link(model):
    assert model.get("name") == "iris_a8mini"
    assert model.get("canonical_link") == "iris_with_standoffs::base_link"
    assert txt(model, "include/uri") == "model://iris_with_standoffs"


def test_gimbal_joints_axes_limits(model):
    joints = {j.get("name"): j for j in model.findall("joint")}
    expect = {
        "gimbal_yaw": ("0 0 1", -2.7925, 2.7925),
        "gimbal_roll": ("1 0 0", -0.5236, 0.5236),
        "gimbal_pitch": ("0 -1 0", -2.3562, 0.7854),
    }
    for name, (axis, lo, hi) in expect.items():
        j = joints[name]
        assert j.get("type") == "revolute"
        assert txt(j, "axis/xyz") == axis
        assert float(txt(j, "axis/limit/lower")) == pytest.approx(lo)
        assert float(txt(j, "axis/limit/upper")) == pytest.approx(hi)
        assert 0.0 < float(txt(j, "axis/limit/effort")) < 10.0  # finite effort
        assert float(txt(j, "axis/dynamics/damping")) > 0.0
    assert joints["gimbal_mount"].get("type") == "fixed"
    assert txt(joints["gimbal_mount"], "parent") == "iris_with_standoffs::base_link"


def test_gimbal_mass_is_a8mini(model):
    links = {lk.get("name"): lk for lk in model.findall("link")}
    total = sum(
        float(txt(links[n], "inertial/mass"))
        for n in ("gimbal_base", "gimbal_yaw_link", "gimbal_roll_link", "gimbal_pitch_link")
    )
    assert total == pytest.approx(0.095)


def test_camera_frames(model):
    frames = {f.get("name"): f for f in model.findall("frame")}
    assert frames["a8mini_camera_link"].get("attached_to") == "gimbal_pitch_link"
    assert frames["a8mini_camera_optical_frame"].get("attached_to") == "a8mini_camera_link"


# ------------------------------------------------------------------ cameras
def sensors(model):
    head = [lk for lk in model.findall("link") if lk.get("name") == "gimbal_pitch_link"][0]
    return {s.get("name"): s for s in head.findall("sensor")}


def test_video_camera(model):
    s = sensors(model)["video_camera"]
    assert s.get("type") == "camera"
    assert txt(s, "topic") == "/sim/camera/video/image"
    assert txt(s, "camera/camera_info_topic") == "/sim/camera/video/camera_info"
    assert float(txt(s, "update_rate")) == 25.0
    assert (int(txt(s, "camera/image/width")), int(txt(s, "camera/image/height"))) == (1280, 720)
    assert txt(s, "camera/image/format") == "R8G8B8"
    assert float(txt(s, "camera/horizontal_fov")) == pytest.approx(1.4137)
    assert float(txt(s, "camera/clip/near")) == pytest.approx(0.05)
    assert float(txt(s, "camera/clip/far")) == pytest.approx(3000.0)
    assert txt(s, "camera/noise/type") == "gaussian"
    assert s.find("camera/triggered") is None
    assert txt(s, "gz_frame_id") == "a8mini_camera_optical_frame"
    assert txt(s, "camera/optical_frame_id") == "a8mini_camera_optical_frame"
    assert s.find("pose").get("relative_to") == "a8mini_camera_link"


def test_photo_camera_default_is_zr10_2k(model):
    s = sensors(model)["photo_camera"]
    assert txt(s, "topic") == "/sim/camera/photo/image"
    assert txt(s, "camera/camera_info_topic") == "/sim/camera/photo/camera_info"
    assert txt(s, "camera/triggered") == "true"
    assert txt(s, "camera/trigger_topic") == "/sim/camera/photo/trigger"
    assert (int(txt(s, "camera/image/width")), int(txt(s, "camera/image/height"))) == (2560, 1440)
    assert float(txt(s, "camera/horizontal_fov")) == pytest.approx(1.4137)


def test_photo_camera_a8mini_4k_and_instance_port():
    m = generate(camera_model="a8mini", instance=1)
    s = sensors(m)["photo_camera"]
    assert (int(txt(s, "camera/image/width")), int(txt(s, "camera/image/height"))) == (3840, 2160)
    ap = plugins(m, "ArduPilotPlugin")[0]
    assert int(txt(ap, "fdm_port_in")) == 9012


def test_explicit_sizes_override_profile():
    m = generate(
        camera_model="a8mini",
        photo_width=1920,
        photo_height=1080,
        video_width=640,
        video_height=360,
        video_fps=15,
        hfov=1.2,
    )
    cams = sensors(m)
    p, v = cams["photo_camera"], cams["video_camera"]
    assert (int(txt(p, "camera/image/width")), int(txt(p, "camera/image/height"))) == (1920, 1080)
    assert (int(txt(v, "camera/image/width")), int(txt(v, "camera/image/height"))) == (640, 360)
    assert float(txt(v, "update_rate")) == 15.0
    assert float(txt(v, "camera/horizontal_fov")) == pytest.approx(1.2)


# ------------------------------------------------------------------ plugins
def test_ardupilot_plugin(model):
    aps = plugins(model, "ArduPilotPlugin")
    assert len(aps) == 1
    ap = aps[0]
    assert txt(ap, "fdm_addr") == "127.0.0.1"
    assert int(txt(ap, "fdm_port_in")) == 9002
    assert txt(ap, "lock_step") == "1"
    assert txt(ap, "imuName") == "iris_with_standoffs::imu_link::imu_sensor"
    controls = {int(c.get("channel")): c for c in ap.findall("control")}
    assert sorted(controls) == [0, 1, 2, 3, 10, 12]
    for ch in range(4):
        c = controls[ch]
        assert txt(c, "type") == "VELOCITY"
        assert txt(c, "jointName") == f"iris_with_standoffs::rotor_{ch}_joint"
        assert abs(float(txt(c, "multiplier"))) == 838.0
        assert (txt(c, "servo_min"), txt(c, "servo_max")) == ("1100", "1900")
    for ch, name, topic in (
        (10, "person", "/sim/payload/servo11"),
        (12, "tent", "/sim/payload/servo13"),
    ):
        c = controls[ch]
        assert txt(c, "type") == "COMMAND"
        assert txt(c, "cmd_topic") == topic
        assert txt(c, "jointName") == f"payload_{name}_latch_joint"
        # value relayed = 3000 * clamp(pwm / 3000, 0, 1) = pwm in microseconds
        assert (float(txt(c, "servo_min")), float(txt(c, "servo_max"))) == (0.0, 3000.0)
        assert float(txt(c, "multiplier")) == 3000.0 and float(txt(c, "offset")) == 0.0


def test_iris_aerodynamics_unchanged(model):
    lift = plugins(model, "gz::sim::systems::LiftDrag")
    assert len(lift) == 8
    assert sorted({txt(p, "link_name") for p in lift}) == [
        f"iris_with_standoffs::rotor_{i}" for i in range(4)
    ]
    assert all(txt(p, "area") == "0.002" and txt(p, "cla") == "4.2500" for p in lift)
    assert len(plugins(model, "gz::sim::systems::ApplyJointForce")) == 4


def test_gimbal_controllers(model):
    jpc = plugins(model, "gz::sim::systems::JointPositionController")
    assert len(jpc) == 3
    by_joint = {txt(p, "joint_name"): p for p in jpc}
    for axis in ("yaw", "roll", "pitch"):
        p = by_joint[f"gimbal_{axis}"]
        assert txt(p, "topic") == f"/sim/gimbal/{axis}/cmd_pos"
        assert txt(p, "use_velocity_commands") == "true"
        assert float(txt(p, "cmd_max")) == pytest.approx(1.5708)  # 90 deg/s


def test_joint_states_and_odometry(model):
    jsp = plugins(model, "gz::sim::systems::JointStatePublisher")
    assert len(jsp) == 1
    assert txt(jsp[0], "topic") == "/sim/gimbal/joint_states"
    assert [j.text for j in jsp[0].findall("joint_name")] == [
        "gimbal_yaw",
        "gimbal_roll",
        "gimbal_pitch",
    ]
    assert float(txt(jsp[0], "update_rate")) == 50.0
    odo = plugins(model, "gz::sim::systems::OdometryPublisher")
    assert len(odo) == 1
    assert txt(odo[0], "odom_topic") == "/sim/drone/odometry"
    assert txt(odo[0], "dimensions") == "3"
    assert float(txt(odo[0], "odom_publish_frequency")) == 50.0


# ------------------------------------------------------------------ payloads
def test_payload_wiring(model):
    links = {lk.get("name") for lk in model.findall("link")}
    nested = {m.get("name"): m for m in model.findall("model")}
    dj = plugins(model, "gz::sim::systems::DetachableJoint")
    assert len(dj) == 2
    for name in ("person", "tent"):
        p = [x for x in dj if txt(x, "child_model") == f"payload_{name}"][0]
        assert txt(p, "detach_topic") == f"/sim/payload/{name}/detach"
        # DetachableJoint only finds DIRECT links of the model that holds the plugin
        assert txt(p, "parent_link") in links
        child = nested[f"payload_{name}"]
        assert child.find(f"link[@name='{txt(p, 'child_link')}']") is not None
        odo = plugins(child, "gz::sim::systems::OdometryPublisher")
        assert txt(odo[0], "odom_topic") == f"/sim/payload/{name}/odometry"
        assert txt(odo[0], "dimensions") == "3"


def test_payload_masses_and_placement(model):
    nested = {m.get("name"): m for m in model.findall("model")}
    body_bottom, leg_tips = -0.0393, -0.195
    person = nested["payload_person"]
    tent = nested["payload_tent"]
    assert float(txt(person, "link/inertial/mass")) == pytest.approx(0.5)
    assert float(txt(tent, "link/inertial/mass")) == pytest.approx(0.12)
    pz = float(txt(person, "pose").split()[2])
    r = float(txt(person, "link/collision/geometry/cylinder/radius"))
    length = float(txt(person, "link/collision/geometry/cylinder/length"))
    assert 3.14159 * r * r * length == pytest.approx(0.0005, rel=0.02)  # 0.5 L bottle
    assert pz + r <= body_bottom - 0.005, "bottle must not touch the body"
    assert pz - r > leg_tips + 0.05, "bottle must hang above the leg tips"
    tz = float(txt(tent, "pose").split()[2])
    bz = float(txt(tent, "link/collision/geometry/box/size").split()[2])
    assert tz + bz / 2 <= body_bottom - 0.005 and tz - bz / 2 > leg_tips + 0.05


def test_no_payloads_option():
    m = generate(payloads=False, instance=2)
    assert m.findall("model") == []
    assert plugins(m, "gz::sim::systems::DetachableJoint") == []
    # the latches (release servos) and their ArduPilot channels stay: part of the airframe
    links = {lk.get("name") for lk in m.findall("link")}
    assert {"payload_person_latch", "payload_tent_latch"} <= links
    ap = plugins(m, "ArduPilotPlugin")[0]
    assert int(txt(ap, "fdm_port_in")) == 9022
    assert len(ap.findall("control")) == 6


# ------------------------------------------------------------------ argument checking
@pytest.mark.parametrize(
    "kwargs",
    [
        {"camera_model": "gopro"},
        {"photo_width": 1920},  # height missing
        {"hfov": 81},  # degrees by mistake
        {"fdm_port_in": 80},
        {"video_fps": 0},
        {"payload_person_mass": -1},
        {"gimbal_initial_pitch": -3.0},
        {"not_an_arg": 1},
        {"payloads": "maybe"},
    ],
)
def test_bad_arguments_rejected(kwargs):
    with pytest.raises(ValueError):
        mg.build_mappings(**kwargs)


def test_instance_port_helper():
    assert mg.fdm_port_for_instance(0) == 9002
    assert mg.fdm_port_for_instance(3) == 9032
    assert mg.build_mappings(instance=1)["fdm_port_in"] == "9012"
    assert mg.build_mappings(instance=1, fdm_port_in=9100)["fdm_port_in"] == "9100"


# ------------------------------------------------------------------ the committed default file
def canonical(el):
    """(tag, attributes, text, children) with comments and layout whitespace removed."""
    return (el.tag, sorted(el.attrib.items()), (el.text or "").strip(), [canonical(c) for c in el])


def test_committed_model_sdf_is_up_to_date():
    with open(DEFAULT_SDF) as f:
        committed = ET.fromstring(f.read())
    fresh = ET.fromstring(mg.generate_model_sdf(XACRO))
    assert canonical(committed) == canonical(fresh), (
        "models/iris_a8mini/model.sdf is out of date: run  "
        "ros2 run sim_models generate_model_sdf --update-default"
    )


@pytest.mark.skipif(shutil.which("xacro") is None, reason="xacro command not found")
def test_plain_xacro_command_gives_the_same_model(tmp_path):
    """The bringup may call `xacro` directly (contract): it must give the same model."""
    out = tmp_path / "m.sdf"
    subprocess.run(
        ["xacro", XACRO, "camera_model:=a8mini", "fdm_port_in:=9012", "-o", str(out)],
        check=True,
        timeout=60,
    )
    plain = ET.parse(str(out)).getroot()
    ours = ET.fromstring(mg.generate_model_sdf(XACRO, camera_model="a8mini", fdm_port_in=9012))
    assert canonical(plain) == canonical(ours)


# ------------------------------------------------------------------ gz sdf -k (optional)
def _ardupilot_models_dir():
    base = os.environ.get("ARDUPILOT_GAZEBO_DIR", os.path.expanduser("~/ardupilot_gazebo"))
    path = os.path.join(base, "models")
    return path if os.path.isdir(os.path.join(path, "iris_with_standoffs")) else None


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz (Gazebo) not installed")
@pytest.mark.parametrize("camera_model", ["zr10", "a8mini"])
def test_gz_sdf_check(tmp_path, camera_model):
    apg = _ardupilot_models_dir()
    if apg is None:
        pytest.skip("ardupilot_gazebo models (iris_with_standoffs) not found")
    out = tmp_path / f"iris_{camera_model}.sdf"
    out.write_text(mg.generate_model_sdf(XACRO, camera_model=camera_model))
    env = dict(os.environ)
    paths = os.pathsep.join([os.path.join(PKG, "models"), apg])
    env["GZ_SIM_RESOURCE_PATH"] = paths
    env["SDF_PATH"] = paths
    res = subprocess.run(
        ["gz", "sdf", "-k", str(out)], env=env, capture_output=True, text=True, timeout=120
    )
    assert res.returncode == 0 and "Valid" in res.stdout, res.stdout + res.stderr
