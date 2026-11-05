#!/usr/bin/env python3
"""Render README previews from the actual zero-pose URDF visuals.

Requires numpy, scipy, trimesh, pyrender, Pillow and an EGL/OpenGL renderer.
Usage: PYOPENGL_PLATFORM=egl python tools/render_urdf_previews.py [ID ...]
"""
from pathlib import Path
import argparse
import gc
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import pyrender
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/previews'
MODELS = {
    'a2-left': ('TsenseI_hand_A2_0/urdf/TsenseI__hand_left_v1.urdf', 'x', False),
    'a2-right': ('TsenseI_hand_A2_0/urdf/TsenseI__hand_right_v1.urdf', 'x', True),
    'a30-left': ('TsenseI_hand_A3.0/urdf/TsenseI_hand_left_v3_0.urdf', 'x', False),
    'a30-right': ('TsenseI_hand_A3.0/urdf/TsenseI_hand_right_v3_0.urdf', 'x', True),
    'a30-withid-left': ('TsenseI_hand_A3.0_withid/urdf/TsenseI_hand_left_v3_0_withid.urdf', 'x', False),
    'a30-withid-right': ('TsenseI_hand_A3.0_withid/urdf/TsenseI_hand_right_v3_0_withid.urdf', 'x', True),
    'a32-noid-left': ('TsenseI_hand_A3.2_no_id/urdf/TsenseI_hand_left_v3_2.urdf', 'minus-y', False),
    'a32-noid-right': ('TsenseI_hand_A3.2_no_id/urdf/TsenseI_hand_right_v3_2.urdf', 'minus-y', True),
    'a32-upper-mid-thumb-left': ('TsenseI_hand_A3.2_withid_upper_mid_thumb/urdf/TsenseI_left.urdf', 'x', False),
    'a32-upper-mid-thumb-right': ('TsenseI_hand_A3.2_withid_upper_mid_thumb/urdf/TsenseI_right.urdf', 'x', True),
    'a32-collision-left': ('TsenseI_hand_A3.2_withid_no_upper5mm_no_midupper6mm_withcollision/urdf/TsenseI_hand_left_v3_2_withid.urdf', 'x', False),
    'a32-collision-right': ('TsenseI_hand_A3.2_withid_no_upper5mm_no_midupper6mm_withcollision/urdf/TsenseI_hand_right_v3_2_withid.urdf', 'x', True),
    'a32-raised-thumb-left': ('TsenseI_hand_A3.2_withid_upper_thumb5mm_noupper_mid/urdf/TsenseI_hand_left_v3_2_withid.urdf', 'x', False),
    'a32-raised-thumb-right': ('TsenseI_hand_A3.2_withid_upper_thumb5mm_noupper_mid/urdf/TsenseI_hand_right_v3_2_withid.urdf', 'x', True),
    'a32-middle-dip-left': ('TsenseI_中指DIP统一_20输出关节 (2)/urdf/TsenseI_hand_left_v3_2_withid.urdf', 'x', False),
    'a32-end': ('TsenseI_hand_A3.2_withid_end/urdf/hand_1009_20j.urdf', 'z', False),
    'a32-pianist-left': ('TsenseI_hand_A3.2_withid_fingertip_pianist_speciality/urdf/hand.urdf', 'x', False),
    'a32-pianist-right': ('TsenseI_hand_A3.2_withid_fingertip_pianist_speciality/urdf/hand_right.urdf', 'x', True),
}


def vec(value):
    return np.fromstring(value, sep=' ')


def transform(origin):
    result = np.eye(4)
    if origin is not None:
        result[:3, :3] = Rotation.from_euler('xyz', vec(origin.get('rpy', '0 0 0'))).as_matrix()
        result[:3, 3] = vec(origin.get('xyz', '0 0 0'))
    return result


def zero_frames(robot):
    joints = robot.findall('joint')
    children = {joint.find('child').get('link') for joint in joints}
    roots = {link.get('name') for link in robot.findall('link')} - children
    if len(roots) != 1:
        raise ValueError(f'Expected one root, found {roots}')
    result = {roots.pop(): np.eye(4)}
    pending = list(joints)
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent in result:
                result[joint.find('child').get('link')] = result[parent] @ transform(joint.find('origin'))
                pending.remove(joint)
                progress = True
        if not progress:
            raise ValueError('Disconnected or cyclic URDF')
    return result


def camera_pose(eye, target, up):
    z = eye - target
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((x, y, z))
    pose[:3, 3] = eye
    return pose


def generate(model_id, renderer):
    urdf, view, is_right = MODELS[model_id]
    path = ROOT / urdf
    robot = ET.parse(path).getroot()
    frames = zero_frames(robot)
    scene = pyrender.Scene(bg_color=[0.075, 0.09, 0.09, 1.], ambient_light=[0.18, 0.18, 0.18])
    named_materials = {mat.get('name'): mat.find('color').get('rgba') for mat in robot.findall('material') if mat.find('color') is not None}
    cache = {}
    limits = []
    objects = 0
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            geometry = visual.find('geometry/mesh')
            if geometry is None:
                raise ValueError(f'Unsupported non-mesh visual in {path}')
            filename = geometry.get('filename')
            if filename.startswith('package://'):
                raise ValueError(f'Unexpected package URI: {filename}')
            asset = (path.parent / filename).resolve()
            if not asset.is_file():
                raise FileNotFoundError(asset)
            scale = tuple(vec(geometry.get('scale', '1 1 1')))
            material = visual.find('material')
            color = material.find('color').get('rgba') if material is not None and material.find('color') is not None else named_materials.get(material.get('name'), '.72 .75 .78 1') if material is not None else '.72 .75 .78 1'
            rgba = tuple(vec(color))
            key = (str(asset), scale, rgba)
            if key not in cache:
                model = trimesh.load(asset, force='mesh', process=False)
                model.apply_scale(scale)
                mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=rgba, metallicFactor=0.0, roughnessFactor=0.78, doubleSided=True)
                cache[key] = (pyrender.Mesh.from_trimesh(model, material=mat, smooth=False), model.bounds.copy())
            mesh, bounds = cache[key]
            pose = frames[link.get('name')] @ transform(visual.find('origin'))
            scene.add(mesh, pose=pose)
            corners = trimesh.bounds.corners(bounds)
            limits.append(corners @ pose[:3, :3].T + pose[:3, 3])
            objects += 1
    points = np.vstack(limits)
    lo, hi = points.min(axis=0), points.max(axis=0)
    target = (lo + hi) / 2
    side = 1 if is_right else -1
    if view == 'x':
        direction = np.array([1., side * .25, .14]); up = np.array([0., 0., 1.]); height, width = hi[2]-lo[2], hi[1]-lo[1]
    elif view == 'minus-y':
        direction = np.array([side * .25, -1., .14]); up = np.array([0., 0., 1.]); height, width = hi[2]-lo[2], hi[0]-lo[0]
    elif view == 'z':
        direction = np.array([side * .25, .14, 1.]); up = np.array([0., 1., 0.]); height, width = hi[1]-lo[1], hi[0]-lo[0]
    else:
        raise ValueError(view)
    direction /= np.linalg.norm(direction)
    yfov = np.deg2rad(36.)
    span = max(height, width * 1.14)
    distance = span / (2 * np.tan(yfov / 2)) * 1.30
    pose = camera_pose(target + direction * distance, target, up)
    scene.add(pyrender.PerspectiveCamera(yfov=yfov, znear=max(distance / 1000, 0.00001), zfar=distance * 5), pose=pose)
    scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=1.15), pose=pose)
    for shift, intensity in ((np.array([.5, .3, 0.]), .45), (np.array([-.3, -.4, .1]), .25)):
        light_pose = camera_pose(target + direction * distance + shift * distance, target, up)
        scene.add(pyrender.DirectionalLight(color=np.ones(3), intensity=intensity), pose=light_pose)
    image, depth = renderer.render(scene)
    if not np.any(depth > 0):
        raise RuntimeError(f'Empty image for {model_id}')
    OUT.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(OUT / f'{model_id}.png', optimize=True)
    print(f'{model_id}: {objects} visuals, {len(cache)} meshes -> docs/previews/{model_id}.png', flush=True)
    del scene, cache
    gc.collect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('models', nargs='*', choices=list(MODELS))
    args = parser.parse_args()
    renderer = pyrender.OffscreenRenderer(viewport_width=900, viewport_height=1000)
    try:
        for model in args.models or MODELS:
            generate(model, renderer)
    finally:
        renderer.delete()

if __name__ == '__main__':
    main()
