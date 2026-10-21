"""Independent FK and mass-moment audit for the 20-output approximation."""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import os
import stat
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'outputs/hand_1009_raised_collision_checked_20261016_r07/hand_1009_description'
DELIVERY = ROOT/'outputs/hand_1009_raised_20j_simplified_20261016_r01'
PACKAGE = DELIVERY/'hand_1009_reduced_description'
WORK = ROOT/'work/hand_1009_20j_reduction_20261016'
VISUAL = ROOT/'work/visualizations/hand-1009-20j-verification-20261016'
sys.dont_write_bytecode = True
sys.path.insert(0,str(SOURCE/'tools'))
sys.path.insert(0,str(ROOT/'work/urdf_real_control_20261010/urdf_tree_dependencies'))
from model_state_adapter import ModelStateAdapter
from urdf_parser_py.urdf import URDF


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def regular(path):
    for p in [path,*path.parents]:
        if p.exists() or p.is_symlink():
            s=p.lstat()
            assert not p.is_symlink() and not getattr(s,'st_file_attributes',0)&stat.FILE_ATTRIBUTE_REPARSE_POINT,str(p)


def save(path,data):
    regular(path)
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def transform(element):
    o=element.find('origin')
    result=np.eye(4)
    if o is not None:
        result[:3,:3]=Rotation.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_matrix()
        result[:3,3]=list(map(float,o.get('xyz','0 0 0').split()))
    return result


def fk(robot,positions):
    joints=list(robot.findall('joint'))
    children={j.find('child').get('link') for j in joints}
    roots={l.get('name') for l in robot.findall('link')}-children
    assert len(roots)==1
    world={roots.pop():np.eye(4)}
    while joints:
        progress=False
        for joint in list(joints):
            parent,child=joint.find('parent').get('link'),joint.find('child').get('link')
            if parent not in world:continue
            motion=np.eye(4)
            if joint.get('type')!='fixed':
                axis=np.array(list(map(float,joint.find('axis').get('xyz').split())))
                q=positions.get(joint.get('name'),0.)
                if joint.get('type')=='prismatic':motion[:3,3]=axis*q
                else:motion[:3,:3]=Rotation.from_rotvec(axis*q).as_matrix()
            assert child not in world
            world[child]=world[parent]@transform(joint)@motion
            joints.remove(joint);progress=True
        assert progress
    assert len(world)==len(robot.findall('link'))
    return world


def moments(robot,world):
    contributions=[]
    for link in robot.findall('link'):
        e=link.find('inertial');assert e is not None
        mass=float(e.find('mass').get('value'));assert mass>0
        t=world[link.get('name')]@transform(e)
        v={k:float(e.find('inertia').get(k)) for k in ('ixx','ixy','ixz','iyy','iyz','izz')}
        tensor=np.array([[v['ixx'],v['ixy'],v['ixz']],[v['ixy'],v['iyy'],v['iyz']],[v['ixz'],v['iyz'],v['izz']]])
        eigen=np.linalg.eigvalsh(tensor)
        assert np.all(eigen>0) and eigen[2]<=eigen[0]+eigen[1]+1e-12
        contributions.append((mass,t[:3,3],t[:3,:3]@tensor@t[:3,:3].T))
    mass=math.fsum(m for m,_,_ in contributions)
    com=sum(m*c for m,c,_ in contributions)/mass
    tensor=sum(i+m*((c-com)@(c-com)*np.eye(3)-np.outer(c-com,c-com)) for m,c,i in contributions)
    return mass,com,tensor


def geometry(robot,world,tag):
    result={}
    for link in robot.findall('link'):
        for e in link.findall(tag):
            assert e.get('name') not in result
            result[e.get('name')]=(e.find('geometry/mesh').get('filename'),world[link.get('name')]@transform(e))
    return result


def main():
    global SOURCE,DELIVERY,PACKAGE,WORK,VISUAL
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-package',type=Path,default=SOURCE)
    parser.add_argument('--delivery-dir',type=Path,default=DELIVERY)
    parser.add_argument('--work-dir',type=Path,default=WORK)
    parser.add_argument('--visual-dir',type=Path,default=VISUAL)
    args=parser.parse_args()
    SOURCE=args.source_package.absolute();DELIVERY=args.delivery_dir.absolute();PACKAGE=DELIVERY/'hand_1009_reduced_description'
    WORK=args.work_dir.absolute();VISUAL=args.visual_dir.absolute()
    sys.stdout.reconfigure(encoding='utf-8')
    for path in [WORK,VISUAL,PACKAGE]:regular(path)
    WORK.mkdir(exist_ok=True);VISUAL.mkdir(exist_ok=True)
    source=ET.parse(SOURCE/'urdf/hand_1009.urdf').getroot()
    reduced=ET.parse(PACKAGE/'urdf/hand_1009_20j.urdf').getroot()
    ros=ET.parse(PACKAGE/'urdf/hand_1009_20j_ros.urdf').getroot()
    source_controls=read(SOURCE/'config/independent_controls.json')['controls']
    contract=read(PACKAGE/'config/joint_name_map.json') if (PACKAGE/'config/joint_name_map.json').exists() else None
    to_public=contract['source_to_public'] if contract else {c['name']:c['name'] for c in source_controls}
    to_source={v:k for k,v in to_public.items()}
    controls=[dict(c,name=to_public[c['name']]) for c in source_controls]
    active_controls=read(PACKAGE/'config/independent_controls.json')
    for expected,actual in zip(controls,active_controls['controls']):
        assert all(actual[k]==v for k,v in expected.items()),expected['name']
    assert len(active_controls['controls'])==20
    if contract:
        assert active_controls['control_count']==active_controls['state_count']==20
        assert active_controls['state_only_names']==[]
        assert active_controls['state_joint_names']==[r['name'] for r in controls]
    # Use the ROS URDF parser separately from ElementTree/build code.
    for name in ['hand_1009_20j.urdf','hand_1009_20j_ros.urdf']:
        parsed=URDF.from_xml_file(str(PACKAGE/'urdf'/name))
        assert len(parsed.links)==21 and len(parsed.joints)==20 and parsed.get_root()=='hand_base'
        assert all(j.type=='revolute' for j in parsed.joints)
    assert len(reduced.findall('link'))==21 and len(reduced.findall('joint'))==20
    assert not reduced.findall('.//mimic') and not reduced.findall('.//ros2_control')
    originals={j.get('name'):j for j in source.findall('joint')}
    for j in reduced.findall('joint'):
        original=originals[to_source[j.get('name')]]
        assert j.get('type')==original.get('type')=='revolute'
        assert j.find('axis').attrib==original.find('axis').attrib
        assert j.find('limit').attrib==original.find('limit').attrib
        assert j.find('child').attrib==original.find('child').attrib
    assert (PACKAGE/'urdf/hand_1009_20j.urdf').read_text(encoding='utf-8') == (PACKAGE/'urdf/hand_1009_20j_ros.urdf').read_text(encoding='utf-8').replace('package://hand_1009_reduced_description/meshes/','../meshes/')
    for robot in [reduced,ros]:
        for mesh in robot.findall('.//mesh'):
            name=mesh.get('filename')
            path=PACKAGE/name[len('package://hand_1009_reduced_description/'):] if name.startswith('package://') else (PACKAGE/'urdf'/name).resolve()
            assert path.is_file() and path.resolve().is_relative_to(PACKAGE.resolve())
            regular(path)
    adapter=ModelStateAdapter(SOURCE)
    zero={c['name']:0. for c in controls}
    cases=[{'name':'zero','pose':zero}]
    for c in controls:
        for suffix,q in [('lower',c['lower_rad']),('middle',(c['lower_rad']+c['upper_rad'])/2),('upper',c['upper_rad'])]:
            cases.append({'name':c['name']+'_'+suffix,'pose':dict(zero,**{c['name']:q})})
    cases.extend([{'name':'all_upper','pose':{c['name']:c['upper_rad'] for c in controls}},
                  {'name':'all_lower','pose':{c['name']:c['lower_rad'] for c in controls}}])
    rng=np.random.default_rng(20261016)
    cases.extend({'name':'random_'+str(i),'pose':{c['name']:float(rng.uniform(c['lower_rad'],c['upper_rad'])) for c in controls}} for i in range(64))
    max_fk=max_zero_geometry=0.
    max_com_difference_mm=max_tensor_relative=0.
    worst_com=worst_tensor=None
    fixture_rows=[]
    zero_moments={}
    for case in cases:
        pose=case['pose'];full_state=adapter.convert({to_source[n]:q for n,q in pose.items()})['positions']
        full_world=fk(source,full_state);reduced_world=fk(reduced,pose)
        for name,t in reduced_world.items():max_fk=max(max_fk,float(np.max(abs(t-full_world[name]))))
        full_mass,full_com,full_i=moments(source,full_world)
        reduced_mass,reduced_com,reduced_i=moments(reduced,reduced_world)
        assert abs(full_mass-reduced_mass)<1e-14
        comdiff=float(np.linalg.norm(full_com-reduced_com)*1000)
        idiff=float(np.linalg.norm(full_i-reduced_i)/np.linalg.norm(full_i))
        if comdiff>max_com_difference_mm:max_com_difference_mm,worst_com=comdiff,case['name']
        if idiff>max_tensor_relative:max_tensor_relative,worst_tensor=idiff,case['name']
        if case['name']=='zero':
            zero_moments={'mass_kg':reduced_mass,'full_COM_m':full_com.tolist(),'reduced_COM_m':reduced_com.tolist(),
                          'COM_difference_m':float(np.linalg.norm(full_com-reduced_com)),
                          'full_tensor_at_COM_kg_m2':full_i.tolist(),'reduced_tensor_at_COM_kg_m2':reduced_i.tolist(),
                          'tensor_max_difference_kg_m2':float(np.max(abs(full_i-reduced_i)))}
            assert zero_moments['COM_difference_m']<1e-12 and zero_moments['tensor_max_difference_kg_m2']<1e-14
            for tag in ['visual','collision']:
                before,after=geometry(source,full_world,tag),geometry(reduced,reduced_world,tag)
                assert set(before)==set(after) and len(after)==303
                for name,(uri,t) in before.items():
                    assert uri==after[name][0]
                    max_zero_geometry=max(max_zero_geometry,float(np.max(abs(t-after[name][1]))))
        if case['name'] in ['zero','all_upper','index_mcp_flexion_upper','middle_dip_flexion_upper','thumb_base_rotation_1_upper','thumb_base_rotation_2_upper']:
            fixture_rows.append({'name':case['name'],'pose':pose,'world':{name:t.T.reshape(-1).tolist() for name,t in reduced_world.items()}})
    assert max_fk<1e-12 and max_zero_geometry<1e-12
    reduced_map=read(PACKAGE/'config/reduction.json')
    mapped=[row['source_link'] for row in reduced_map['links']]
    assert len(mapped)==len(set(mapped))==54
    assert set(mapped)=={e.get('name') for e in source.findall('link')}
    assert len([x for x in reduced_map['collapsed_joints'] if x['kind']=='zero_frozen_follower_approximation'])==28
    for row in reduced_map['inertials']:
        assert abs(row['mass_kg']-sum(c['mass_kg'] for c in row['source_components']))<1e-14
    mesh_count=0
    for rel,digest in read(PACKAGE/'config/model_manifest.json')['mesh_files'].items():
        assert sha(PACKAGE/rel)==digest==sha(SOURCE/rel);mesh_count+=1
    protected=read(PACKAGE/'provenance/source_files.json')['source_files_sha256']
    assert all(sha(SOURCE/name)==digest for name,digest in protected.items())
    # Compile launch and regeneration script without writing bytecode or launching foreground nodes.
    for path in [PACKAGE/'launch/display.launch.py',PACKAGE/'tools/build_reduced.py']:
        compile(path.read_text(encoding='utf-8'),str(path),'exec')
    assert ET.parse(PACKAGE/'package.xml').getroot().findtext('name')=='hand_1009_reduced_description'
    proof={'passed':True,'source_model_id':read(PACKAGE/'config/model_manifest.json')['source_model_id'],
           'representation_id':reduced_map['revision'],'links':21,'moving_joints':20,'prismatic_joints':0,'fixed_joints':0,
           'visuals':303,'collisions':303,'source_links_counted_exactly_once':54,'source_mesh_files_unchanged':mesh_count,
           'source_package_unchanged':True,'all_output_axes_limits_preserved':True,
           'all_output_names_match_adopted_policy':True,'renamed_output_count':sum(a!=b for a,b in to_public.items()),
           'URDF_parser_passed_both_resource_variants':True,'all_mesh_resources_resolve':True,
           'source_vs_reduced_output_FK_poses':len(cases),'max_output_FK_matrix_error':max_fk,
           'max_zero_visual_collision_matrix_error':max_zero_geometry,'zero_pose_moments':zero_moments,
           'all_link_tensors_positive_and_triangle_inequality':True,
           'nonzero_pose_approximation_sample':{'poses':len(cases)-1,'max_COM_difference_mm':max_com_difference_mm,'worst_COM_case':worst_com,
                                              'max_whole_hand_COM_tensor_relative_Frobenius_difference':max_tensor_relative,'worst_tensor_case':worst_tensor,
                                              'not_an_error_bound_for_all_poses':True},
           'real_ROS2_launch_executed':False,'hardware_control_or_full_mechanism_collision_validated':False}
    save(WORK/'verification.json',proof)
    save(PACKAGE/'provenance/verification.json',proof)
    save(WORK/'browser_fixtures.json',{'controls':controls,'fixtures':fixture_rows,'package':str(PACKAGE),'screenshots':str(VISUAL)})
    # Record the real paths used for every screenshot, before any browser writes one.
    save(WORK/'visual_path_checks.json',{'directory':str(VISUAL),'no_reparse_ancestors':True})
    print(json.dumps(proof,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
