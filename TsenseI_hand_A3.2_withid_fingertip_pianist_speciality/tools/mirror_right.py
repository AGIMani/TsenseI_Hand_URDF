#!/usr/bin/env python3
"""Generate a physical right-hand mirror of the supplied pianist URDF.

Requires NumPy.  All inputs are the original left-hand assets in this folder;
outputs are hand_right.urdf, meshes_right/, and config/mirror_right.json.
"""
from pathlib import Path
import copy
import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'urdf/hand.urdf'
TARGET = ROOT / 'urdf/hand_right.urdf'
S = np.diag([1., -1., 1.])
STL = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


def numbers(xs):
    return ' '.join(format(float(x), '.17g') for x in xs)


def vector(text):
    return np.asarray([float(v) for v in text.split()])


def mirror_origin(e):
    # For URDF fixed-axis rpy, S @ R(r,p,y) @ S == R(-r,p,-y).
    if e is None:
        raise ValueError('Expected an explicit origin')
    e.set('xyz', numbers(S @ vector(e.get('xyz', '0 0 0'))))
    e.set('rpy', numbers(vector(e.get('rpy', '0 0 0')) * [-1., 1., -1.]))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mirror_mesh(source, target):
    data = source.read_bytes()
    if len(data) < 84:
        raise ValueError(f'Not a binary STL: {source}')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f'Not a binary STL: {source}')
    original = np.frombuffer(data, dtype=STL, offset=84, count=count)
    reflected = original.copy()
    reflected['vertices'] = (original['vertices'][:, [0, 2, 1], :].astype(np.float64) @ S)
    vertices = reflected['vertices'].astype(np.float64)
    normals = np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0])
    length = np.linalg.norm(normals, axis=1)
    reflected['normal'] = normals / np.maximum(length[:, None], 1e-30)
    target.parent.mkdir(parents=True, exist_ok=True)
    header = b'TsenseI pianist right-hand mirrored STL; corrected winding'.ljust(80, b'\0')
    target.write_bytes(header + struct.pack('<I', count) + reflected.tobytes())
    return count


def main():
    left = ET.parse(SOURCE).getroot()
    right = copy.deepcopy(left)
    right.set('name', left.get('name') + '_right_mirror')
    right.insert(0, ET.Comment('Geometric mirror about hand_base_link Y=0; q_right=q_left.'))
    jobs = {}
    for link in right.findall('link'):
        inertial = link.find('inertial')
        if inertial is not None:
            mirror_origin(inertial.find('origin'))
            inertia = inertial.find('inertia')
            for key in ('ixy', 'iyz'):
                inertia.set(key, format(-float(inertia.get(key)), '.17g'))
        for kind in ('visual', 'collision'):
            for part in link.findall(kind):
                mirror_origin(part.find('origin'))
                mesh = part.find('geometry/mesh')
                if mesh is None or not np.array_equal(vector(mesh.get('scale', '1 1 1')), [1, 1, 1]):
                    raise ValueError('Only unit-scale STL meshes are supported by this generator')
                old = mesh.get('filename')
                if not old.startswith('../meshes/'):
                    raise ValueError(f'Unexpected mesh path: {old}')
                new = old.replace('../meshes/', '../meshes_right/', 1)
                source = (SOURCE.parent / old).resolve()
                target = (TARGET.parent / new).resolve()
                if not source.is_relative_to(ROOT / 'meshes') or not target.is_relative_to(ROOT / 'meshes_right'):
                    raise ValueError(f'Unexpected mesh path: {old}')
                jobs[source] = target
                mesh.set('filename', new)
    for joint in right.findall('joint'):
        mirror_origin(joint.find('origin'))
        axis = joint.find('axis')
        if axis is not None:
            kind = joint.get('type')
            if kind in ('revolute', 'continuous'):
                sign = -1.
            elif kind == 'prismatic':
                sign = 1.
            else:
                raise ValueError(f'Unsupported joint type: {kind}')
            axis.set('xyz', numbers(sign * S @ vector(axis.get('xyz', '1 0 0'))))
    rows = []
    for source, target in sorted(jobs.items()):
        triangles = mirror_mesh(source, target)
        rows.append({'source': str(source.relative_to(ROOT)), 'right': str(target.relative_to(ROOT)),
                     'triangles': triangles, 'source_sha256': sha256(source), 'right_sha256': sha256(target)})
    ET.indent(right, space='  ')
    ET.ElementTree(right).write(TARGET, encoding='utf-8', xml_declaration=True)
    manifest = {'source_urdf': str(SOURCE.relative_to(ROOT)), 'source_sha256': sha256(SOURCE),
                'right_urdf': str(TARGET.relative_to(ROOT)), 'right_sha256': sha256(TARGET),
                'mirror_plane': 'hand_base_link Y = 0 m',
                'position_mapping': 'x_right=x_left, y_right=-y_left, z_right=z_left',
                'joint_mapping': 'same joint names, limits and angles: q_right=q_left',
                'status': 'geometric mirror; not independently measured right-hand hardware',
                'mesh_count': len(rows), 'meshes': rows}
    (ROOT / 'config/mirror_right.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(f'Generated {TARGET}; reflected {len(rows)} meshes.')

if __name__ == '__main__':
    main()
