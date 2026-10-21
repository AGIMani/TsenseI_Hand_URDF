#!/usr/bin/env python3
"""Generate the left URDF and physical STL reflection. Requires numpy."""
from pathlib import Path
import copy
import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "urdf/TsenseI_hand_right_v3_2_withid.urdf"
TARGET = ROOT / "urdf/TsenseI_hand_left_v3_2_withid.urdf"
S = np.diag([1., -1., 1.])
SHIFT = np.array([0., .008, 0.])
D = np.eye(4)
D[:3, :3] = S
F = D.copy()
F[:3, 3] = SHIFT
STL = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2")])


def rename(value):
    return (value.replace("right_hand", "left_hand")
            .replace("Right_hand", "Left_hand")
            .replace("TsenseI_hand_right", "TsenseI_hand_left")
            .replace("withid_right_", "withid_left_")
            .replace("meshes/right/", "meshes/left/"))


def vector(value):
    return np.array([float(x) for x in value.split()])


def numbers(value):
    return " ".join(format(float(x), ".17g") for x in value)


def origin(element):
    result = np.eye(4)
    if element is None:
        return result
    r, p, y = vector(element.get("rpy", "0 0 0"))
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    result[:3, :3] = [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ]
    result[:3, 3] = vector(element.get("xyz", "0 0 0"))
    return result


def reflect_origin(element, shift=False, mesh_frame=False):
    # S Rz(y) Ry(p) Rx(r) S = Rz(-y) Ry(p) Rx(-r).
    old = origin(element)
    xyz = S @ old[:3, 3]
    if shift:
        xyz += SHIFT
        if mesh_frame:
            xyz -= (S @ old[:3, :3] @ S) @ SHIFT
    element.set("xyz", numbers(xyz))
    element.set("rpy", numbers(vector(element.get("rpy", "0 0 0")) * [-1, 1, -1]))


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {path}")
    return np.frombuffer(data, dtype=STL, offset=84, count=count)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xmlwrite(path, robot):
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(path, encoding="utf-8", xml_declaration=True)


def main():
    right = ET.parse(SOURCE).getroot()
    left = copy.deepcopy(right)
    jobs = {}
    for link in left.findall("link"):
        root = link.get("name") == "base_link"
        for inertial in link.findall("inertial"):
            reflect_origin(inertial.find("origin"), shift=root)
            inertia = inertial.find("inertia")
            for key in ("ixy", "iyz"):
                inertia.set(key, format(-float(inertia.get(key)), ".17g"))
        for kind in ("visual", "collision"):
            for item in link.findall(kind):
                reflect_origin(item.find("origin"), shift=root, mesh_frame=True)
                mesh = item.find("geometry/mesh")
                if mesh is None:
                    raise ValueError("This exporter expects mesh geometry")
                if not np.array_equal(vector(mesh.get("scale", "1 1 1")), [1, 1, 1]):
                    raise ValueError("This exporter expects unit mesh scales")
                source = (SOURCE.parent / mesh.get("filename")).resolve()
                target = (TARGET.parent / rename(mesh.get("filename"))).resolve()
                assert ROOT / "meshes/left" in target.parents
                assert target != source
                if source in jobs:
                    assert jobs[source] == (target, root)
                jobs[source] = (target, root)
    for joint in left.findall("joint"):
        reflect_origin(joint.find("origin"),
                       shift=joint.find("parent").get("link") == "base_link")
        axis = joint.find("axis")
        if axis is not None:
            sign = -1 if joint.get("type") in ("revolute", "continuous") else 1
            axis.set("xyz", numbers(sign * S @ vector(axis.get("xyz"))))
    for element in left.iter():
        for key, value in list(element.attrib.items()):
            element.set(key, rename(value))
    left.insert(0, ET.Comment(
        "Left mirror of the current right URDF about base_link Y=0.004 m; "
        "physical STL reflection, corrected winding, q_left=q_right."))
    rows = []
    for source, (target, root) in sorted(jobs.items()):
        original = read_stl(source)
        mirrored = original.copy()
        xyz = original["vertices"].astype(float) @ S
        if root:
            xyz += SHIFT
        mirrored["vertices"] = xyz[:, [0, 2, 1]]
        actual = mirrored["vertices"].astype(float)
        normals = np.cross(actual[:, 1]-actual[:, 0], actual[:, 2]-actual[:, 0])
        length = np.linalg.norm(normals, axis=1)
        mirrored["normal"] = normals / np.maximum(length[:, None], 1e-30)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(b"TsenseI left mirror; metres; corrected winding".ljust(80, b"\0")
                           + struct.pack("<I", len(mirrored)) + mirrored.tobytes())
        rows.append({"source": str(source.relative_to(ROOT)),
                     "target": str(target.relative_to(ROOT)),
                     "root_frame": root, "triangles": len(mirrored),
                     "source_sha256": digest(source), "target_sha256": digest(target)})
    xmlwrite(TARGET, left)
    ros = copy.deepcopy(left)
    package = ET.parse(ROOT / "package.xml").getroot().findtext("name")
    for mesh in ros.iter("mesh"):
        mesh.set("filename", f"package://{package}/" + mesh.get("filename").removeprefix("../"))
    xmlwrite(ROOT / "urdf/TsenseI_hand_left_ros.urdf", ros)
    report = {
        "source": str(SOURCE.relative_to(ROOT)), "source_sha256": digest(SOURCE),
        "target": str(TARGET.relative_to(ROOT)), "target_sha256": digest(TARGET),
        "mirror_plane_base_y_m": .004,
        "base_point_relation": "x_left=x_right; y_left=0.008-y_right; z_left=z_right",
        "joint_relation": "q_left=q_right; limits and mass unchanged",
        "meshes": rows,
    }
    (ROOT / "checks/left_mirror_manifest.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Generated {TARGET}\nMirrored {len(rows)} STL files.")


if __name__ == "__main__":
    main()
