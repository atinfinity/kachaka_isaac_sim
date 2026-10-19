import math

import pytest

from kachaka_isaac_sim import urdf_tools
from kachaka_isaac_sim.urdf_tools import (
    BODY_CLEARANCE,
    CASTERS,
    EXTRA_FRAMES,
    build_sim_urdf,
    frame_pose,
    read_com,
    read_joints,
    rpy_to_quat,
    write_urdf,
)


def test_rpy_to_quat():
    assert rpy_to_quat(0, 0, 0) == pytest.approx((1, 0, 0, 0))
    h = math.sqrt(0.5)
    assert rpy_to_quat(0, 0, math.pi / 2) == pytest.approx((h, 0, 0, h))
    assert rpy_to_quat(math.pi / 2, 0, 0) == pytest.approx((h, h, 0, 0))
    # Fixed-axis roll then pitch: q = q_pitch * q_roll.
    q = urdf_tools._qmul(rpy_to_quat(0, 0.4, 0), rpy_to_quat(0.3, 0, 0))
    assert rpy_to_quat(0.3, 0.4, 0) == pytest.approx(q)


def test_xacro_subset(tmp_path):
    (tmp_path / "inc.xacro").write_text(
        '<robot xmlns:xacro="http://ros.org/wiki/xacro">'
        '<xacro:property name="r" value="0.5"/>'
        '<xacro:macro name="wheel" params="name *origin">'
        '<joint name="${name}_joint" type="fixed"><xacro:insert_block name="origin"/>'
        '<parent link="base_link"/><child link="${name}"/></joint>'
        '<link name="${name}"/>'
        "</xacro:macro></robot>")
    (tmp_path / "robot.xacro").write_text(
        '<robot name="t" xmlns:xacro="http://ros.org/wiki/xacro">'
        '<xacro:include filename="$(find pkg)/inc.xacro"/>'
        '<link name="base_link"/>'
        '<xacro:wheel name="w"><origin xyz="${r * 2} 0 0" rpy="0 0 ${pi / 2}"/></xacro:wheel>'
        "</robot>")
    robot = urdf_tools.Xacro({"pkg": tmp_path}).process_file(tmp_path / "robot.xacro")
    out = tmp_path / "t.urdf"
    write_urdf(robot, out)
    (joint,) = read_joints(out)
    assert joint["name"] == "w_joint"
    assert joint["child"] == "w"
    assert joint["xyz"] == pytest.approx((1.0, 0.0, 0.0))
    assert frame_pose([joint], "w")[1] == pytest.approx(rpy_to_quat(0, 0, math.pi / 2))


def test_frame_pose_chains_fixed_joints():
    a = {"type": "fixed", "parent": "base_link", "child": "a", "xyz": (1, 0, 0), "quat": rpy_to_quat(0, 0, math.pi / 2)}
    b = {"type": "fixed", "parent": "a", "child": "b", "xyz": (1, 0, 0), "quat": (1, 0, 0, 0)}
    xyz, quat = frame_pose([a, b], "b")
    assert xyz == pytest.approx((1, 1, 0))
    assert quat == pytest.approx(a["quat"])
    with pytest.raises(ValueError):
        frame_pose([{**a, "type": "revolute"}, b], "b")


@pytest.fixture(scope="module")
def sim_urdf(kachaka_description, tmp_path_factory):
    robot = build_sim_urdf(kachaka_description / "robot/kachaka.urdf.xacro",
                           {"kachaka_description": kachaka_description})
    path = tmp_path_factory.mktemp("urdf") / "kachaka.urdf"
    write_urdf(robot, path)
    return robot, path


def test_sim_urdf_has_no_xacro_left(sim_urdf):
    _, path = sim_urdf
    text = path.read_text()
    assert "xacro" not in text
    assert "${" not in text


def test_sim_urdf_body_clears_floor(sim_urdf):
    robot, _ = sim_urdf
    base = next(link for link in robot.iter("link") if link.get("name") == "base_link")
    body = base.find("collision")
    sz = float(body.find("geometry/box").get("size").split()[2])
    z = float(body.find("origin").get("xyz").split()[2])
    assert z - sz / 2 == pytest.approx(BODY_CLEARANCE)


def test_sim_urdf_wheels_and_casters(sim_urdf):
    robot, _ = sim_urdf
    wheels = [link for link in robot.iter("link") if link.get("name").endswith("drive_wheel_link")]
    assert len(wheels) == 2
    for wheel in wheels:
        assert wheel.find("collision/geometry/sphere") is not None
        assert wheel.find("collision/geometry/cylinder") is None
    base = next(link for link in robot.iter("link") if link.get("name") == "base_link")
    names = {col.get("name") for col in base.findall("collision")}
    assert set(CASTERS) <= names


def test_sim_urdf_frames(sim_urdf):
    _, path = sim_urdf
    joints = read_joints(path)
    links = {j["child"] for j in joints}
    for frame, parent in EXTRA_FRAMES.items():
        assert frame in links
        (xyz, quat), (pxyz, pquat) = frame_pose(joints, frame), frame_pose(joints, parent)
        assert xyz == pytest.approx(pxyz)
        assert quat == pytest.approx(pquat)
    for frame in ("laser_frame", "imu_link", "camera_front_link"):
        xyz, _ = frame_pose(joints, frame)
        assert all(abs(v) < 0.5 for v in xyz)
    assert read_com(path) != (0.0, 0.0, 0.0)
