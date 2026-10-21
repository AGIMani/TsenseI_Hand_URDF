#!/usr/bin/env python3
"""Check geometry, inertia, topology and motion against the current right URDF."""
import json
import xml.etree.ElementTree as ET

import numpy as np

from mirror_left import ROOT, SOURCE, TARGET, S, SHIFT, D, F
from mirror_left import digest, origin, read_stl, rename, vector


def fk(robot, q):
    links = {link.get("name") for link in robot.findall("link")}
    pending = robot.findall("joint")
    children = [j.find("child").get("link") for j in pending]
    assert len(children) == len(set(children))
    assert links - set(children) == {"base_link"}
    result = {"base_link": np.eye(4)}
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in result]
        assert ready, "Disconnected tree or cycle"
        for j in ready:
            move = np.eye(4)
            if j.get("type") == "revolute":
                a = vector(j.find("axis").get("xyz"))
                assert abs(np.linalg.norm(a)-1) < 1e-12
                x, y, z = a
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                angle = q.get(j.get("name"), 0)
                move[:3, :3] += np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)
            else:
                assert j.get("type") == "fixed"
            result[j.find("child").get("link")] = (
                result[j.find("parent").get("link")] @ origin(j.find("origin")) @ move)
            pending.remove(j)
    assert set(result) == links
    return result


def inertia(link):
    e = link.find("inertial/inertia")
    return np.array([[float(e.get("i"+a+b) if a <= b else e.get("i"+b+a))
                      for b in "xyz"] for a in "xyz"])


def main():
    right, left = [ET.parse(p).getroot() for p in (SOURCE, TARGET)]
    manifest = json.loads((ROOT / "checks/left_mirror_manifest.json").read_text())
    assert digest(SOURCE) == manifest["source_sha256"]
    assert digest(TARGET) == manifest["target_sha256"]
    mesh_error = 0.
    triangle_count = 0
    for row in manifest["meshes"]:
        source, target = ROOT / row["source"], ROOT / row["target"]
        assert digest(source) == row["source_sha256"]
        assert digest(target) == row["target_sha256"]
        a, b = read_stl(source), read_stl(target)
        expected = a["vertices"].astype(float) @ S
        if row["root_frame"]:
            expected += SHIFT
        error = np.max(np.abs(expected[:, [0, 2, 1]] - b["vertices"]))
        mesh_error = max(mesh_error, float(error))
        v = b["vertices"].astype(float)
        n = np.cross(v[:, 1]-v[:, 0], v[:, 2]-v[:, 0])
        valid = np.linalg.norm(n, axis=1) > 1e-20
        n = n[valid] / np.linalg.norm(n[valid], axis=1)[:, None]
        assert np.allclose(n, b["normal"][valid], atol=1e-6)
        triangle_count += len(b)
    assert mesh_error < 1e-8
    ros = ET.parse(ROOT / "urdf/TsenseI_hand_left_ros.urdf").getroot()
    package = ET.parse(ROOT / "package.xml").getroot().findtext("name")
    for main_mesh, ros_mesh in zip(left.iter("mesh"), ros.iter("mesh"), strict=True):
        assert (TARGET.parent / main_mesh.get("filename")).is_file()
        expected_uri = f"package://{package}/" + main_mesh.get("filename").removeprefix("../")
        assert ros_mesh.get("filename") == expected_uri
        ros_mesh.set("filename", main_mesh.get("filename"))
    assert ET.tostring(ros) == ET.tostring(left)
    left_links = {l.get("name"): l for l in left.findall("link")}
    mass = 0.
    for link in right.findall("link"):
        other = left_links[rename(link.get("name"))]
        m = float(link.find("inertial/mass").get("value"))
        assert m == float(other.find("inertial/mass").get("value"))
        mass += m
        expected = (F if link.get("name") == "base_link" else D) @ origin(link.find("inertial/origin")) @ D
        assert np.allclose(origin(other.find("inertial/origin")), expected, atol=1e-12)
        assert np.allclose(inertia(other), S @ inertia(link) @ S, atol=1e-14)
        assert np.all(np.linalg.eigvalsh(inertia(other)) > 0)
        for kind in ("visual", "collision"):
            A = F if link.get("name") == "base_link" else D
            for a, b in zip(link.findall(kind), other.findall(kind), strict=True):
                assert np.allclose(origin(b.find("origin")), A @ origin(a.find("origin")) @ A, atol=1e-12)
    joints = [j for j in right.findall("joint") if j.get("type") == "revolute"]
    lj = {j.get("name"): j for j in left.findall("joint")}
    for j in joints:
        assert j.find("limit").attrib == lj[rename(j.get("name"))].find("limit").attrib
    limits = [(float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))) for j in joints]
    rng = np.random.default_rng(20261020)
    poses = [np.zeros(len(joints)), np.array(limits)[:, 0], np.array(limits)[:, 1]]
    for i, bounds in enumerate(limits):
        for value in bounds:
            pose = np.zeros(len(joints))
            pose[i] = value
            poses.append(pose)
    poses.extend(rng.uniform(np.array(limits)[:, 0], np.array(limits)[:, 1], (100, len(joints))))
    fk_error = 0.
    for pose in poses:
        q = {j.get("name"): float(v) for j, v in zip(joints, pose)}
        rf, lf = fk(right, q), fk(left, {rename(k): v for k, v in q.items()})
        for name, transform in rf.items():
            if name != "base_link":
                fk_error = max(fk_error, float(np.max(np.abs(lf[rename(name)] - F @ transform @ D))))
    assert fk_error < 1e-12
    report = {
        "passed": True, "links": len(left_links), "joints": len(left.findall("joint")),
        "movable_joints": len(joints), "stl_files": len(manifest["meshes"]),
        "triangles_checked": triangle_count, "poses_checked": len(poses),
        "max_mesh_reflection_error_m": mesh_error, "max_fk_matrix_error": fk_error,
        "total_mass_kg": mass, "relative_and_ros_mesh_paths_valid": True,
        "inertia_COM_mass_limits_and_winding_valid": True,
        "scope": "Geometric and kinematic reflection validation; ROS/RViz runtime and collision-free motion not tested.",
    }
    (ROOT / "checks/left_mirror_validation.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
