"""Build an output-coordinate URDF by freezing follower branches at zero.

Standard-library-only, deterministic and viewer independent. This is an
explicit approximation, never a reassignment of the physical mechanism.
"""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import stat
import sys
import xml.etree.ElementTree as ET

PACKAGE = 'hand_1009_reduced_description'
IDENTITY = [[float(i == j) for j in range(4)] for i in range(4)]


def regular(path):
    for p in (path, *path.parents):
        if p.exists() or p.is_symlink():
            info = p.lstat()
            if p.is_symlink() or getattr(info, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
                raise ValueError('Reparse point is not allowed: ' + str(p))


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    regular(path)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def mm(a, b):
    return [[math.fsum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def transpose(a):
    return list(map(list, zip(*a)))


def origin(e):
    o = e.find('origin')
    xyz = list(map(float, o.get('xyz', '0 0 0').split())) if o is not None else [0., 0., 0.]
    r, p, y = list(map(float, o.get('rpy', '0 0 0').split())) if o is not None else [0., 0., 0.]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0., 0., 0., 1.]]


def fmt(v):
    return format(float(v), '.17g')


def set_origin(e, t):
    o = e.find('origin')
    if o is None:
        o = ET.Element('origin')
        e.insert(0, o)
    p = math.atan2(-t[2][0], math.hypot(t[0][0], t[1][0]))
    if math.hypot(t[0][0], t[1][0]) > 1e-12:
        r, y = math.atan2(t[2][1], t[2][2]), math.atan2(t[1][0], t[0][0])
    else:
        r, y = math.atan2(-t[1][2], t[1][1]), 0.
    o.set('xyz', ' '.join(fmt(t[i][3]) for i in range(3)))
    o.set('rpy', ' '.join(fmt(v) for v in (r, p, y)))
    if max(abs(a-b) for aa, bb in zip(origin(e), t) for a, b in zip(aa, bb)) > 1e-10:
        raise ValueError('Origin composition failed: ' + e.get('name', e.tag))


def transformed_mass(link, transform):
    element = link.find('inertial')
    if element is None:
        raise ValueError('Source inertial is missing: ' + link.get('name'))
    m = float(element.find('mass').get('value'))
    t = mm(transform, origin(element))
    r = [row[:3] for row in t[:3]]
    vals = {k: float(element.find('inertia').get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    inertia = [[vals['ixx'], vals['ixy'], vals['ixz']], [vals['ixy'], vals['iyy'], vals['iyz']], [vals['ixz'], vals['iyz'], vals['izz']]]
    return {'source_link': link.get('name'), 'mass_kg': m, 'COM_m': [t[i][3] for i in range(3)],
            'tensor_at_own_COM_kg_m2': mm(mm(r, inertia), transpose(r))}


def combined_mass(components):
    mass = math.fsum(c['mass_kg'] for c in components)
    if mass <= 0:
        raise ValueError('A reduced link has no positive accounted mass')
    center = [math.fsum(c['mass_kg']*c['COM_m'][i] for c in components)/mass for i in range(3)]
    inertia = [[0.]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            terms = []
            for c in components:
                d = [c['COM_m'][k]-center[k] for k in range(3)]
                terms.append(c['tensor_at_own_COM_kg_m2'][i][j] + c['mass_kg']*((math.fsum(v*v for v in d) if i == j else 0.)-d[i]*d[j]))
            inertia[i][j] = math.fsum(terms)
    return {'mass_kg': mass, 'COM_m': center, 'tensor_at_COM_kg_m2': inertia}


def add_inertial(link, data):
    element = ET.SubElement(link, 'inertial')
    ET.SubElement(element, 'origin', xyz=' '.join(map(fmt, data['COM_m'])), rpy='0 0 0')
    ET.SubElement(element, 'mass', value=fmt(data['mass_kg']))
    t = data['tensor_at_COM_kg_m2']
    ET.SubElement(element, 'inertia', {name: fmt(t[i][j]) for name, i, j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})


def validate_source(source, robot, controls, tree):
    links = {e.get('name'): e for e in robot.findall('link')}
    joints = {e.get('name'): e for e in robot.findall('joint')}
    if len(links) != 54 or len(joints) != 53:
        raise ValueError('This reduction requires the reviewed 54-link mechanism; review new topologies separately')
    if set(links) != {x['name'] for x in tree['links']} or set(joints) != {x['name'] for x in tree['joints']}:
        raise ValueError('Source URDF and source tree do not agree')
    names = [c['name'] for c in controls['controls']]
    if len(names) != 20 or len(set(names)) != 20 or tree['model_id'] != controls['model_id']:
        raise ValueError('Expected twenty named outputs from one source model')
    for c in controls['controls']:
        j = joints[c['name']]
        if j.get('type') != 'revolute' or c['zero_rad'] != 0:
            raise ValueError('Unexpected output type/reference: ' + c['name'])
        for attr, key in [('lower', 'lower_rad'), ('upper', 'upper_rad')]:
            if abs(float(j.find('limit').get(attr))-c[key]) > 1e-12:
                raise ValueError('Source output limits differ: ' + c['name'])
    for row in tree['joints']:
        e = joints[row['name']]
        if (e.get('type'), e.find('parent').get('link'), e.find('child').get('link')) != (row['type'], row['parent_link'], row['child_link']):
            raise ValueError('Source relationship differs: ' + row['name'])
        expected = row['origin_matrix_column_major']
        if max(abs(origin(e)[i][k]-expected[4*k+i]) for i in range(4) for k in range(4)) > 1e-10:
            raise ValueError('Source frame differs: ' + row['name'])
        if row['type'] != 'fixed' and max(abs(a-b) for a,b in zip(map(float,e.find('axis').get('xyz').split()),row['axis_joint_frame'])) > 1e-12:
            raise ValueError('Source axis differs: '+row['name'])
    # A follower is always assigned to the mounting side of its own actuator,
    # as established by the source topology/loop, never by spatial proximity.
    loops = read(source/'config/nonlinear_loops.json')['loops']
    for loop in loops:
        slider, rod = joints[loop['slider_joint']], joints[loop['rod_joint']]
        if slider.find('parent').get('link') != loop['parent_link'] or rod.find('parent').get('link') != slider.find('child').get('link'):
            raise ValueError('Follower branch no longer matches its actuator installation')
    if {j for row in loops for j in (row['slider_joint'],row['rod_joint'])} != set(joints)-set(names)-{n for n,e in joints.items() if e.get('type')=='fixed'}:
        raise ValueError('Unclassified moving source joint')
    return links, joints, names


def project(source):
    robot = ET.parse(source/'urdf/hand_1009.urdf').getroot()
    controls = read(source/'config/independent_controls.json')
    tree = read(source/'config/tree.json')
    links, joints, outputs = validate_source(source, robot, controls, tree)
    root = tree['root_link']
    owner, transforms = {root: root}, {root: copy.deepcopy(IDENTITY)}
    pending = list(joints.values())
    retained, collapsed = {}, []
    while pending:
        progress = False
        for joint in list(pending):
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in owner:
                continue
            t = mm(transforms[parent], origin(joint))
            if joint.get('name') in outputs:
                owner[child], transforms[child] = child, copy.deepcopy(IDENTITY)
                new = copy.deepcopy(joint)
                new.find('parent').set('link', owner[parent])
                set_origin(new, t)
                retained[joint.get('name')] = new
            else:
                owner[child], transforms[child] = owner[parent], t
                collapsed.append({'source_joint':joint.get('name'),'source_link':child,'target_link':owner[child],
                                  'kind':'fixed_installation_merge' if joint.get('type')=='fixed' else 'zero_frozen_follower_approximation',
                                  'reference_coordinate':0.,'reference_units':'m' if joint.get('type')=='prismatic' else 'rad',
                                  'source_to_target_matrix':t})
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('Source graph is cyclic/disconnected')
    names = [root] + [retained[n].find('child').get('link') for n in outputs]
    result = ET.Element('robot', name='hand_1009_20j')
    result.append(ET.Comment(' 20-OUTPUT REDUCTION: 28 follower coordinates are frozen at the adopted zero and rigidly lumped on their actuator mounting side. Full zero-pose geometry and accounted mass moments are retained. This is not the full mechanism, not a physical loop solver. Effort/velocity remain source parser placeholders; inertials remain engineering estimates. '))
    for material in robot.findall('material'):
        result.append(copy.deepcopy(material))
    targets = {name: ET.SubElement(result, 'link', name=name) for name in names}
    components, inventory = {name: [] for name in names}, []
    for name, link in links.items():
        target, transform = targets[owner[name]], transforms[name]
        components[owner[name]].append(transformed_mass(link, transform))
        for item in link:
            if item.tag not in ('visual', 'collision', 'inertial'):
                raise ValueError('Unsupported source link data: ' + item.tag)
            if item.tag == 'inertial':
                continue
            copied = copy.deepcopy(item)
            set_origin(copied, mm(transform, origin(item)))
            target.append(copied)
        inventory.append({'source_link':name,'target_link':owner[name], 'source_to_target_matrix':transform,
                          'source_mass_kg':components[owner[name]][-1]['mass_kg'],
                          'visuals':[e.get('name') for e in link.findall('visual')],
                          'collisions':[e.get('name') for e in link.findall('collision')]})
    inertials = []
    for name in names:
        data = combined_mass(components[name])
        add_inertial(targets[name], data)
        inertials.append({'link':name, **data, 'source_components':components[name],
                         'status':'engineering_estimate_with_zero_frozen_followers' if any(x['target_link']==name and x['kind']=='zero_frozen_follower_approximation' for x in collapsed) else 'source_engineering_estimate'})
    for name in outputs:
        result.append(retained[name])
    assert len(owner)==54 and len(names)==21 and len(retained)==20 and len(collapsed)==33
    return result, {'source_model_id':tree['model_id'],'source_kinematics_sha256':tree['kinematics_sha256'],
                    'source_output_interface_id':controls['interface_id'],
                    'policy':'freeze follower branches at zero to actuator mounting-side owner; compose fixed installations; no nearest-neighbor assignment',
                    'links':inventory,'collapsed_joints':collapsed,'inertials':inertials}, controls


def write_xml(path, robot):
    regular(path)
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-package', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True, help='New delivery directory; existing directories are rejected')
    parser.add_argument('--revision', default='raised_20j_fixed_followers_r01_20261016')
    args = parser.parse_args()
    source, delivery = args.source_package.absolute(), args.output_dir.absolute()
    regular(source); regular(delivery)
    if delivery.exists() or delivery == source or source in delivery.parents or delivery in source.parents:
        raise ValueError('Use a new output directory separate from the full source package')
    protected = {str(f.relative_to(source)).replace('\\','/'):sha(f) for f in source.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
    robot, reduction, controls = project(source)
    package = delivery/PACKAGE
    for folder in ('urdf','meshes/visual','meshes/collision','config','docs','launch','tools','provenance'):
        regular(package/folder)
        (package/folder).mkdir(parents=True, exist_ok=True)
    write_xml(package/'urdf/hand_1009_20j.urdf', robot)
    ros = copy.deepcopy(robot)
    for mesh in ros.findall('.//mesh'):
        name = mesh.get('filename')
        if not name.startswith('../meshes/'):
            raise ValueError('Unexpected source mesh URI: ' + name)
        mesh.set('filename', 'package://'+PACKAGE+'/'+name[3:])
    write_xml(package/'urdf/hand_1009_20j_ros.urdf', ros)
    resources = {}
    for mesh in robot.findall('.//mesh'):
        rel = Path(mesh.get('filename')[3:])
        origin_file, target = source/rel, package/rel
        if not origin_file.resolve().is_relative_to(source.resolve()):
            raise ValueError('Mesh escapes source package')
        regular(origin_file); regular(target)
        key = str(rel).replace('\\','/')
        if key not in resources:
            shutil.copy2(origin_file, target)
            resources[key] = sha(target)
    reduction.update(revision=args.revision,reference_pose='all twenty output coordinates and all frozen source followers zero',
                     full_mechanism_source_sha256=sha(source/'urdf/hand_1009.urdf'),
                     output_frames_and_axes_preserved=True,complete_mechanism_collision_evidence_transferred=False,
                     full_motion_mass_distribution_preserved=False,full_model_mechanical_stops_certified=False,
                     hardware_commands_configured=False,mesh_decimation_performed=False)
    dump(package/'config/reduction.json', reduction)
    dump(package/'config/independent_controls.json', controls)
    dump(package/'config/model_manifest.json', {'representation_id':args.revision,'source_model_id':controls['model_id'],
         'ROS_package':PACKAGE,'links':21,'moving_joints':20,'fixed_joints':0,'prismatic_joints':0,
         'visuals':len(robot.findall('.//visual')),'collisions':len(robot.findall('.//collision')),
         'mesh_files':resources,'mass_kg':math.fsum(row['mass_kg'] for row in reduction['inertials']),
         'geometry_policy':'All source visual and collision triangles unchanged, follower geometry frozen relative to its mounting side. Collision checks from the full model are not valid for moving reduced poses.',
         'state_source':'twenty output coordinates; actuator/encoder conversion remains external',
         'mass_scope':'same accounted baseline as the full source model, not fitted to the 1130 g physical hand with accessories',
         'effort_velocity_limits':'inherited parser placeholders, not hardware specifications'})
    snapshot = ['config/independent_controls.json','config/assembly_zero_reference.json','config/joint_name_registry.json','config/output_range_policy.json','config/working_range_policy.json','config/nonlinear_loops.json']
    for name in snapshot:
        # These are evidence/reference files, not model-state inputs for this URDF.
        shutil.copy2(source/name, package/'provenance'/Path(name).name)
    shutil.copy2(source/'docs/R06_MASS_REFINEMENT.txt', package/'docs/SOURCE_MASS_UNCERTAINTIES.txt')
    dump(package/'provenance/source_files.json', {'source_package_at_build':str(source),'source_files_sha256':protected})
    with (package/'docs/逐刚体质量与质心.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['刚体','质量_g','质心x_m','质心y_m','质心z_m','来源刚体数','状态'])
        for row in reduction['inertials']:
            writer.writerow([row['link'], row['mass_kg']*1000, *row['COM_m'], len(row['source_components']), row['status']])
    package_xml = f'''<?xml version="1.0"?>
<package format="3">
  <name>{PACKAGE}</name><version>0.1.0</version>
  <description>Twenty-output hand approximation. Follower branches frozen at zero, inertia aggregated from the complete model. No hardware driver.</description>
  <maintainer email="unassigned@example.invalid">Unassigned project maintainer</maintainer>
  <license>UNSPECIFIED</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ament_index_python</exec_depend><exec_depend>launch</exec_depend><exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend><exec_depend>joint_state_publisher</exec_depend><exec_depend>joint_state_publisher_gui</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
'''
    (package/'package.xml').write_text(package_xml, encoding='utf-8')
    (package/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.8)
project({PACKAGE})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config docs launch tools provenance DESTINATION share/${{PROJECT_NAME}} PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${{PROJECT_NAME}})
ament_package()
''',encoding='utf-8')
    (package/'launch/display.launch.py').write_text('''"""Publish only the twenty reduced output joints, using standard ROS 2 nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    root=Path(get_package_share_directory('hand_1009_reduced_description'))
    xml=(root/'urdf/hand_1009_20j_ros.urdf').read_text(encoding='utf-8')
    description={'robot_description':xml, 'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[description]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[description],condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[description],condition=UnlessCondition(LaunchConfiguration('gui'))),
    ])
''',encoding='utf-8')
    shutil.copy2(Path(__file__), package/'tools/build_reduced.py')
    readme = f'''# 抬高版：20关节简化模型

来源：完整机构包 `{source.name}`；模型 ID `{controls['model_id']}`；简化修订 `{args.revision}`。

本包有21个刚体、20个旋转关节，没有独立动子或连杆关节，也没有 mimic 或自定义随动依赖。
20个关节的名称、零位、正方向和上下限沿用来源。拇指一级0～120°，二级0～100°；两级弯曲上限以源模型当前定义为准。
其余范围以 config/independent_controls.json 中的原始弧度值为准，未四舍五入为新的限位。

## 使用

- 普通URDF查看器：加载 urdf/hand_1009_20j.urdf，同时提供 meshes 文件夹，保持目录结构。
- ROS 2：将本目录放入工作空间 src，运行 `colcon build --packages-select {PACKAGE}`，source 安装环境后运行 `ros2 launch {PACKAGE} display.launch.py`。这会启动标准20关节面板与TF发布，不含真机控制器。无界面可加 `gui:=false`。
- ROS资源入口为 urdf/hand_1009_20j_ros.urdf，使用 package://{PACKAGE}/ 路径。
- 两份URDF的运动、几何和惯量相同，仅资源路径不同。

## 简化方式与适用范围

14套动子和连杆，共28个随动坐标，均冻结在来源零姿态；沿来源机构树归并到各自电机的安装侧。
另外5个真实固定安装链接按精确变换合并。没有按空间距离寻找所谓“最近刚体”。
零姿态的全部可视/碰撞零件保留，网格没有减面。指节运动仍然正确；弯曲后固定等效的动子不会伸缩，连杆可能与另一端脱开或交叠。
这些是本简化模型的已知近似，不应把它当成真实机构连接异常，也不应据此倒推真实电机行程。
本轮制作的是20关节简化版，尚未制作进一步减面的网页轻量版。

## 质量惯量

所有54个来源刚体均恰好归入一个目标刚体。旋转张量至目标坐标系后，用平行轴定理合并，保留非对角项。
总计质量沿用来源，约1052.982463g；未强行凑到1130g。
零姿态的整手质心和惯量应与完整模型一致；活动件冻结后，其他姿态的质量分布与动力学为近似。
来源的电机分重、F4质量、材料和内部质量分布等估算仍然存在，见 docs/SOURCE_MASS_UNCERTAINTIES.txt。
逐刚体质量表见 docs/逐刚体质量与质心.csv；完整张量和归并来源见 config/reduction.json。

## 限制

本包没有编码器标定或硬件驱动。effort/velocity数值继承来源的解析占位值，不是电机性能参数。
碰撞网格保持来源精度，但其随动方式已简化；完整模型的r07运动碰撞结论不能直接用于本包。
部分允许关节组合本身可能干涉。当前关节范围保持用户采用值，没有根据碰撞扫描缩小。
本包用于运动模型交换和展示；实际执行、动力学或避障应用需按用途配置与验证。

## 后续同步生成

完整模型仍是关节参数来源。本包只读取它，不修改完整模型，也不覆盖完整模型的最新交付记录。
来源更新后使用Python标准库运行：

```text
python tools/build_reduced.py --source-package <新完整包目录> --output-dir <新的交付目录> --revision <新的简化修订名>
```

生成器拒绝覆盖已有目录。拓扑发生变化时会要求重新审查策略。
provenance/source_files.json 记录来源哈希，config/reduction.json 记录每个刚体的归并方式。
'''
    (package/'README.md').write_text(readme, encoding='utf-8')
    if any(sha(source/name)!=digest for name,digest in protected.items()):
        raise ValueError('Source changed while building; do not publish this output')
    print(json.dumps({'package':str(package),'links':21,'revolute_joints':20,'visuals':len(robot.findall('.//visual')),
                      'collisions':len(robot.findall('.//collision')),'source_unchanged':True,
                      'mass_g':math.fsum(row['mass_kg'] for row in reduction['inertials'])*1000}, ensure_ascii=False))


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
