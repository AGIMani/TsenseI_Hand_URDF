"""Independent geometric regression checks for the pianist right-hand URDF."""
from pathlib import Path
import collections
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1] / 'TsenseI_hand_A3.2_withid_fingertip_pianist_speciality'
LEFT = ROOT / 'urdf/hand.urdf'
RIGHT = ROOT / 'urdf/hand_right.urdf'
REFLECT = np.diag([1., -1., 1.])
STL = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


def vec(value):
    return np.fromstring(value, sep=' ')


def transform(origin):
    out = np.eye(4)
    if origin is not None:
        out[:3, :3] = Rotation.from_euler('xyz', vec(origin.get('rpy', '0 0 0'))).as_matrix()
        out[:3, 3] = vec(origin.get('xyz', '0 0 0'))
    return out


def frames(robot, q):
    joints = robot.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    roots = {link.get('name') for link in robot.findall('link')} - children
    assert len(roots) == 1, roots
    result = {roots.pop(): np.eye(4)}
    remaining = list(joints)
    while remaining:
        progress = False
        for joint in remaining[:]:
            parent = joint.find('parent').get('link')
            if parent not in result:
                continue
            motion = np.eye(4)
            if joint.get('type') in ('revolute', 'continuous'):
                axis = vec(joint.find('axis').get('xyz'))
                motion[:3, :3] = Rotation.from_rotvec(axis * q.get(joint.get('name'), 0.)).as_matrix()
            result[joint.find('child').get('link')] = result[parent] @ transform(joint.find('origin')) @ motion
            remaining.remove(joint)
            progress = True
        assert progress, 'Disconnected or cyclic kinematic tree'
    return result


def read_stl(path):
    with path.open('rb') as f:
        header = f.read(84)
        count = int.from_bytes(header[80:84], 'little')
        data = np.fromfile(f, dtype=STL)
    assert len(data) == count and path.stat().st_size == 84 + count * 50, path
    return data


class PianistMirror(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.left = ET.parse(LEFT).getroot()
        cls.right = ET.parse(RIGHT).getroot()
        cls.llinks = {e.get('name'): e for e in cls.left.findall('link')}
        cls.rlinks = {e.get('name'): e for e in cls.right.findall('link')}
        cls.ljoints = {e.get('name'): e for e in cls.left.findall('joint')}
        cls.rjoints = {e.get('name'): e for e in cls.right.findall('joint')}

    def test_tree_joint_names_and_limits(self):
        self.assertEqual(len(self.llinks), 21)
        self.assertEqual(len(self.ljoints), 20)
        self.assertEqual(set(self.llinks), set(self.rlinks))
        self.assertEqual(set(self.ljoints), set(self.rjoints))
        for name, joint in self.ljoints.items():
            other = self.rjoints[name]
            self.assertEqual(joint.get('type'), other.get('type'))
            self.assertEqual(joint.find('parent').attrib, other.find('parent').attrib)
            self.assertEqual(joint.find('child').attrib, other.find('child').attrib)
            self.assertEqual(joint.find('limit').attrib, other.find('limit').attrib)
            np.testing.assert_allclose(vec(other.find('axis').get('xyz')), -REFLECT @ vec(joint.find('axis').get('xyz')), atol=1e-14)

    def test_frames_and_world_inertias_for_varied_joint_positions(self):
        rng = np.random.default_rng(1902)
        positions = [{}]
        for _ in range(12):
            positions.append({name: rng.uniform(float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))) for name, j in self.ljoints.items()})
        for q in positions:
            lf, rf = frames(self.left, q), frames(self.right, q)
            for name, link in self.llinks.items():
                np.testing.assert_allclose(rf[name][:3, :3], REFLECT @ lf[name][:3, :3] @ REFLECT, atol=2e-12)
                np.testing.assert_allclose(rf[name][:3, 3], REFLECT @ lf[name][:3, 3], atol=2e-12)
                li, ri = link.find('inertial'), self.rlinks[name].find('inertial')
                self.assertAlmostEqual(float(li.find('mass').get('value')), float(ri.find('mass').get('value')), places=15)
                lp, rp = lf[name] @ transform(li.find('origin')), rf[name] @ transform(ri.find('origin'))
                np.testing.assert_allclose(rp[:3, 3], REFLECT @ lp[:3, 3], atol=2e-12)
                def inertia(e):
                    a=e.find('inertia').attrib
                    return np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
                lw = lp[:3, :3] @ inertia(li) @ lp[:3, :3].T
                rw = rp[:3, :3] @ inertia(ri) @ rp[:3, :3].T
                np.testing.assert_allclose(rw, REFLECT @ lw @ REFLECT, atol=2e-14)

    def test_all_mesh_vertices_winding_and_visual_placement(self):
        referenced = {}
        for name, link in self.llinks.items():
            for kind in ('visual', 'collision'):
                left_parts, right_parts = link.findall(kind), self.rlinks[name].findall(kind)
                self.assertEqual(len(left_parts), len(right_parts))
                for lp, rp in zip(left_parts, right_parts):
                    lm, rm = lp.find('geometry/mesh'), rp.find('geometry/mesh')
                    self.assertEqual(lm.get('scale', '1 1 1'), rm.get('scale', '1 1 1'))
                    lt, rt = transform(lp.find('origin')), transform(rp.find('origin'))
                    np.testing.assert_allclose(rt[:3,:3], REFLECT @ lt[:3,:3] @ REFLECT, atol=2e-12)
                    np.testing.assert_allclose(rt[:3,3], REFLECT @ lt[:3,3], atol=2e-12)
                    source = (LEFT.parent / lm.get('filename')).resolve()
                    dest = (RIGHT.parent / rm.get('filename')).resolve()
                    self.assertTrue(source.is_file(), source)
                    self.assertTrue(dest.is_file(), dest)
                    self.assertNotEqual(source, dest)
                    self.assertTrue(dest.is_relative_to(ROOT / 'meshes_right'))
                    referenced[source] = dest
        self.assertEqual(len(referenced), 148)
        triangles = 0
        for source, dest in referenced.items():
            left, right = read_stl(source), read_stl(dest)
            self.assertEqual(len(left), len(right))
            expected = left['vertices'][:, [0,2,1], :].astype(np.float64) @ REFLECT
            np.testing.assert_allclose(right['vertices'], expected, atol=1e-9, rtol=0)
            v = right['vertices'].astype(np.float64)
            cross = np.cross(v[:,1]-v[:,0], v[:,2]-v[:,0])
            length = np.linalg.norm(cross, axis=1)
            nonzero = length > 1e-18
            if np.any(nonzero):
                np.testing.assert_allclose(right['normal'][nonzero], cross[nonzero] / length[nonzero,None], atol=2e-6)
            triangles += len(left)
        print(f'Checked {len(referenced)} meshes and {triangles:,} mirrored triangles')

if __name__ == '__main__':
    unittest.main()
