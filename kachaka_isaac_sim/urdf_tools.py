"""Expand the official Kachaka xacro and add what the simulator needs.

This is plain Python (no Isaac Sim, no ROS) so it can run and be tested anywhere.
It supports only the xacro subset used by kachaka_description:
include, property, macro (with a *block parameter), insert_block and ${expr}.
"""

from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

XACRO_NS = "http://ros.org/wiki/xacro"
X = f"{{{XACRO_NS}}}"
_EXPR = re.compile(r"\$\{([^}]*)\}")
_FIND = re.compile(r"\$\(find ([^)]+)\)")


class Xacro:
    def __init__(self, package_paths: dict[str, Path]):
        self.package_paths = package_paths
        self.props: dict[str, object] = {"pi": math.pi}
        self.macros: dict[str, ET.Element] = {}

    # --- expressions -------------------------------------------------------
    def _eval(self, text: str, scope: dict[str, object]) -> str:
        text = _FIND.sub(lambda m: str(self.package_paths[m.group(1)]), text)

        def repl(m: re.Match) -> str:
            value = eval(m.group(1), {"__builtins__": {}}, {**self.props, **scope})  # noqa: S307
            return str(value)

        return _EXPR.sub(repl, text)

    # --- processing --------------------------------------------------------
    def process_file(self, path: Path) -> ET.Element:
        root = ET.parse(path).getroot()
        out = ET.Element(root.tag, {k: v for k, v in root.attrib.items()})
        self._process_children(root, out, {}, {})
        return out

    def _process_children(self, src: ET.Element, dst: ET.Element, scope: dict, blocks: dict) -> None:
        for child in list(src):
            tag = child.tag
            if not isinstance(tag, str):
                continue
            if tag == X + "include":
                inc = self.process_file(Path(self._eval(child.attrib["filename"], scope)))
                for c in list(inc):
                    dst.append(c)
            elif tag == X + "property":
                self.props[child.attrib["name"]] = _num(self._eval(child.attrib["value"], scope))
            elif tag == X + "macro":
                self.macros[child.attrib["name"]] = child
            elif tag == X + "insert_block":
                for c in list(blocks[child.attrib["name"]]):
                    self._process_element(c, dst, scope, blocks)
            elif tag.startswith(X):
                name = tag[len(X):]
                if name not in self.macros:
                    raise ValueError(f"unsupported xacro tag: {name}")
                self._expand_macro(self.macros[name], child, dst, scope)
            else:
                self._process_element(child, dst, scope, blocks)

    def _process_element(self, src: ET.Element, dst: ET.Element, scope: dict, blocks: dict) -> None:
        el = ET.SubElement(dst, src.tag, {k: self._eval(v, scope) for k, v in src.attrib.items()})
        el.text = src.text
        self._process_children(src, el, scope, blocks)

    def _expand_macro(self, macro: ET.Element, call: ET.Element, dst: ET.Element, scope: dict) -> None:
        new_scope = dict(scope)
        blocks: dict[str, ET.Element] = {}
        block_names = []
        for p in macro.attrib.get("params", "").split():
            if p.startswith("*"):
                block_names.append(p[1:])
            else:
                key = p.split(":=")[0]
                new_scope[key] = _num(self._eval(call.attrib.get(key, ""), scope))
        elems = [c for c in call if isinstance(c.tag, str)]
        for name, el in zip(block_names, elems):
            wrapper = ET.Element("block")
            wrapper.append(el)
            blocks[name] = wrapper
        self._process_children(macro, dst, new_scope, blocks)


def _num(text: str) -> object:
    try:
        return float(text)
    except ValueError:
        return text


def _fmt(*values: float) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _add_fixed_link(robot: ET.Element, name: str, parent: str, xyz=(0, 0, 0), rpy=(0, 0, 0)) -> ET.Element:
    """Add `name` under `parent` through a fixed joint named `<name>_sim_joint`."""
    link = ET.SubElement(robot, "link", {"name": name})
    joint = ET.SubElement(robot, "joint", {"name": f"{name}_sim_joint", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": name})
    ET.SubElement(joint, "origin", {"xyz": _fmt(*xyz), "rpy": _fmt(*rpy)})
    return link


# Caster spheres. The official URDF has none. Front pair follows the community
# Gazebo model; the rear one sits 1 mm above the floor and only stops tipping.
CASTER_RADIUS = 0.01
CASTERS = {
    "caster_front_left": (0.19, 0.075, CASTER_RADIUS),
    "caster_front_right": (0.19, -0.075, CASTER_RADIUS),
    "caster_rear": (-0.12, 0.0, CASTER_RADIUS + 0.001),
}
# Real-robot frames that the official URDF lacks. They sit at their parent's
# pose until values from a real unit are known.
EXTRA_FRAMES = {
    "camera_front": "camera_front_link",
    "camera_back": "camera_back_link",
    "tof_camera": "tof_link",
    "lidar_center_frame": "laser_frame",
}
BODY_CLEARANCE = 0.008  # m between floor and the body collision box


def build_sim_urdf(xacro_path: Path, package_paths: dict[str, Path]) -> ET.Element:
    """Return the expanded URDF with casters, extra frames and floor clearance."""
    robot = Xacro(package_paths).process_file(xacro_path)

    # Lift the body collision box off the floor so only wheels and casters touch.
    for link in robot.iter("link"):
        if link.attrib["name"] != "base_link":
            continue
        col = link.find("collision")
        box = col.find("geometry/box")
        sx, sy, sz = (float(v) for v in box.attrib["size"].split())
        x, y, _ = (float(v) for v in col.find("origin").attrib["xyz"].split())
        new_sz = sz - BODY_CLEARANCE
        box.attrib["size"] = _fmt(sx, sy, new_sz)
        col.find("origin").attrib["xyz"] = _fmt(x, y, BODY_CLEARANCE + new_sz / 2)

    # Wheels touch the floor at one point, like a rounded tyre. A flat 25 mm
    # cylinder contacts across its width and scrubs when turning in place,
    # which makes the effective wheel separation wider than 0.2 m.
    for link in robot.iter("link"):
        if not link.attrib["name"].endswith("drive_wheel_link"):
            continue
        geom = link.find("collision/geometry")
        cyl = geom.find("cylinder")
        geom.remove(cyl)
        ET.SubElement(geom, "sphere", {"radius": cyl.attrib["radius"]})

    # Casters are extra collision spheres on base_link rather than links of
    # their own: tiny separate links make the articulation solver inaccurate.
    base = next(l for l in robot.iter("link") if l.attrib["name"] == "base_link")
    for name, (x, y, z) in CASTERS.items():
        col = ET.SubElement(base, "collision", {"name": name})
        ET.SubElement(col, "origin", {"xyz": _fmt(x, y, z), "rpy": "0 0 0"})
        ET.SubElement(ET.SubElement(col, "geometry"), "sphere", {"radius": str(CASTER_RADIUS)})

    for name, parent in EXTRA_FRAMES.items():
        _add_fixed_link(robot, name, parent)
    return robot


def write_urdf(robot: ET.Element, path: Path) -> None:
    robot = copy.deepcopy(robot)
    ET.indent(robot)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(path, encoding="unicode", xml_declaration=True)


def rpy_to_quat(r: float, p: float, y: float) -> tuple[float, float, float, float]:
    """Return (w, x, y, z) for URDF fixed-axis roll, pitch, yaw."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def read_joints(urdf_path: Path) -> list[dict]:
    """Return every joint as {name, type, parent, child, xyz, quat, axis}."""
    joints = []
    for j in ET.parse(urdf_path).getroot().iter("joint"):
        origin = j.find("origin")
        xyz = tuple(float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split())
        rpy = tuple(float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split())
        axis_el = j.find("axis")
        axis = tuple(float(v) for v in (axis_el.get("xyz") if axis_el is not None else "1 0 0").split())
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz,
            "quat": rpy_to_quat(*rpy),
            "axis": axis,
        })
    return joints


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _qrot(q, v):
    w, x, y, z = q
    _, rx, ry, rz = _qmul(_qmul(q, (0.0, *v)), (w, -x, -y, -z))
    return rx, ry, rz


def frame_pose(joints: list[dict], frame: str, root: str = "base_link"):
    """Pose (xyz, quat w-x-y-z) of `frame` in `root`, through fixed joints."""
    by_child = {j["child"]: j for j in joints}
    xyz, quat = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    while frame != root:
        j = by_child[frame]
        if j["type"] != "fixed":
            raise ValueError(f"{frame} is not rigidly attached to {root}")
        # T = T_joint * T
        r = _qrot(j["quat"], xyz)
        xyz = tuple(a + b for a, b in zip(j["xyz"], r))
        quat = _qmul(j["quat"], quat)
        frame = j["parent"]
    return xyz, quat


def read_com(urdf_path: Path, link: str = "base_link") -> tuple[float, float, float]:
    for el in ET.parse(urdf_path).getroot().iter("link"):
        if el.get("name") == link:
            origin = el.find("inertial/origin")
            if origin is not None:
                return tuple(float(v) for v in origin.get("xyz", "0 0 0").split())
    return (0.0, 0.0, 0.0)
