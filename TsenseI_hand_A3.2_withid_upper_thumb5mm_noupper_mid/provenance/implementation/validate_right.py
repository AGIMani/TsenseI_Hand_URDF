from common import *
import ast,copy,importlib.util,shutil,csv,io
from urdf_parser_py.urdf import URDF

def main():
    lr=ET.parse(LEFT/'urdf/TsenseI_hand_left_v3_2_withid.urdf').getroot();rr=ET.parse(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf').getroot();parsed=URDF.from_xml_file(str(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf'))
    assert parsed.get_root()=='base_link' and len(parsed.links)==22 and len(parsed.joints)==21
    assert sum(j.type=='revolute' for j in parsed.joints)==20 and sum(j.type=='fixed' for j in parsed.joints)==1 and not any(j.mimic for j in parsed.joints)
    assert not list(rr.iter('dynamics'))
    assert all('left_hand' not in e.get('name','').lower() for e in [*rr.findall('link'),*rr.findall('joint')])
    leftnames={j.get('name'):j for j in lr.findall('joint')};rightnames={j.get('name'):j for j in rr.findall('joint')}
    for n,j in leftnames.items():
        r=rightnames[renamed(n)];a=j.find('limit');b=r.find('limit');assert (a is None and b is None) or a.attrib==b.attrib
        assert r.get('type')==j.get('type')
    # All vertices, normals, and winding have an independently predictable mirror.
    lm=read(LEFT/'config/structure_mesh_manifest.json')['meshes'];rm=read(OUT/'config/structure_mesh_manifest.json')['meshes'];maxmesh=0.;normalerror=0.;triangles=0;checked_normals=0
    assert len(lm)==len(rm)==367
    for a,b in zip(lm,rm):
        assert b['name']==renamed(a['name']) and b['target_link']==renamed(a['target_link']) and b['urdf_collision_enabled']
        x=stl(LEFT/a['mesh']);y=stl(OUT/b['mesh']);assert len(x)==len(y);triangles+=len(x)
        ideal=x['vertices'].astype(float)@S
        if a['target_link']=='base_link':ideal+=SHIFT
        err=float(np.max(np.abs(y['vertices'].astype(float)-ideal[:,[0,2,1]])));maxmesh=max(maxmesh,err);assert err<2e-8
        # Recomputed normals handle pre-existing very thin triangles whose stored
        # source normals disagree with their float-rounded vertex orientation.
        v=y['vertices'].astype(float);area=np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0]);length=np.linalg.norm(area,axis=1);valid=length>1e-16
        d=np.sum(area[valid]*y['normal'][valid],axis=1)/length[valid]
        assert np.min(d,initial=1.)>.9999,(b['name'],np.min(d));checked_normals+=int(valid.sum())
        normalerror=max(normalerror,float(np.max(abs(d-1),initial=0.)))
        assert sha(OUT/b['mesh'])==b['mesh_sha256']
    assert normalerror<1e-6 and triangles==1839012
    # Full left audited pose sets, including two-thumb rotation combinations.
    poses=read(LEFT/'checks/collision_rigid_20.json')['poses'];assert len(poses)==1011
    maxframe=0.;maxthumb=0.;lf0=fk(lr);rf0=fk(rr)
    for pose in poses:
        lf=fk(lr,pose['q_output_rad']);rf=fk(rr,{renamed(n):v for n,v in pose['q_output_rad'].items()})
        for n,t in lf.items():
            if n=='base_link':continue
            ideal=F@t@D;err=float(np.max(abs(ideal-rf[renamed(n)])));maxframe=max(maxframe,err)
            if 'thumb' in n:maxthumb=max(maxthumb,err)
        assert np.max(abs(rf['base_link']-np.eye(4)))==0
    assert maxframe<1e-11
    # Internal 14-loop tree gets the same command values and signed strokes.
    ls=ET.parse(LEFT/'provenance/full_linkage_DISPLAY_ONLY_source.urdf.txt').getroot();rs=ET.parse(OUT/'provenance/right_full_linkage_AUDIT_ONLY.urdf.txt').getroot();BC=matrix(read(LEFT/'checks/r02_baseline/visual_conversion_manifest.json')['base_from_CAD_column_major'])
    ll=read(LEFT/'config/nonlinear_loops.json')['loops'];rl=read(OUT/'config/nonlinear_loops.json')['loops'];mapping=read(LEFT/'config/name_mapping.json')['output_joints'];reverse={v:k for k,v in mapping.items()};maxinternal=0.;maxslider=0.
    for pose in poses:
        q={reverse[n]:v for n,v in pose['q_output_rad'].items()};a=nonlinear(q,ll);b=nonlinear({source_name(n):v for n,v in q.items()},rl);lf=fk(ls,a);rf=fk(rs,b)
        for n,t in lf.items():
            if n=='hand_base':continue
            maxinternal=max(maxinternal,float(np.max(abs(F@BC@t@D-rf[source_name(n)]))))
        for loop in ll:maxslider=max(maxslider,abs(a[loop['slider_joint']]-b[source_name(loop['slider_joint'])]))
    assert maxinternal<1e-11 and maxslider<1e-12
    # Mass/COM/full inertia: same mass, shifted/reflected COM, correct products.
    leftlinks={l.get('name'):l for l in lr.findall('link')};rightlinks={l.get('name'):l for l in rr.findall('link')};inertial_checks=[]
    for n,l in leftlinks.items():
        r=rightlinks[renamed(n)];a=l.find('inertial');b=r.find('inertial');mass=float(a.find('mass').get('value'));assert mass==float(b.find('mass').get('value'))
        c=origin(a.find('origin'))[:3,3];rc=origin(b.find('origin'))[:3,3];I=inertia(a.find('inertia'));RI=inertia(b.find('inertia'));expected=S@c+(SHIFT if n=='base_link' else 0);cg=float(np.linalg.norm(rc-expected));ig=float(np.max(abs(RI-S@I@S)));eig=np.linalg.eigvalsh(RI)
        assert cg<1e-12 and ig<1e-15 and eig[0]>0 and eig[-1]<=eig[0]+eig[1]+1e-12 and np.allclose(eig,np.linalg.eigvalsh(I),rtol=1e-12,atol=1e-18)
        inertial_checks.append({'link':renamed(n),'mass_kg':mass,'COM_mirror_error_m':cg,'inertia_mirror_error_kg_m2':ig,'principal_inertias_kg_m2':eig.tolist()})
    assert abs(sum(r['mass_kg'] for r in inertial_checks)-.952)<1e-12
    for filename in ['TsenseI_hand_right_v3_2_withid.urdf','TsenseI_hand_right_ros.urdf']:
        doc=ET.parse(OUT/'urdf'/filename).getroot()
        assert len(list(doc.iter('visual')))==len(list(doc.iter('collision')))==367
        for mesh in doc.iter('mesh'):
            name=mesh.get('filename');p=OUT/name.removeprefix('package://'+PACKAGE+'/') if name.startswith('package://') else OUT/'urdf'/name;guard(p);assert p.is_file() and p.resolve().is_relative_to(OUT.resolve()) and mesh.get('scale')=='1 1 1'
            assert 'left' not in name
    # The mirror plane preserves the four-hole pattern and the Z=-4mm face.
    definition=read(OUT/'config/mirror_definition.json');a=np.array([x['center_mm'] for x in definition['left_mount_holes']]);b=np.array([x['center_mm'] for x in definition['right_mount_holes']]);gap=max(min(np.linalg.norm(x-y) for y in a) for x in b);assert gap<1e-4
    required={'P_THUMB_PALM_MOUNT':'base_link','P0353_OUTPUT':'right_hand_thumb_rota_block_link','P0334_OUTPUT':'right_hand_thumb_rotaback_link2','P0336':'right_hand_thumb_rota_block_link','P0386':'right_hand_thumb_rotaback_link2'}
    for pid,link in required.items():assert all(r['target_link']==link for r in rm if r['native_part_id']==pid) and any(r['native_part_id']==pid for r in rm)
    import xacro
    expanded=ET.fromstring(xacro.process_file(str(OUT/'urdf/TsenseI_hand_right_mock.urdf.xacro')).toxml());controlnames=[j.get('name') for j in expanded.findall('ros2_control/joint')];assert len(controlnames)==20 and all(n.startswith('right_hand_') for n in controlnames)
    for p in (OUT/'launch').glob('*.py'):ast.parse(p.read_text('utf-8'))
    for p in list((OUT/'config').glob('*.yaml'))+list((OUT/'rviz').glob('*.rviz')):yaml.safe_load(p.read_text('utf-8'))
    srdf=ET.parse(OUT/'config/TsenseI_hand_right.srdf').getroot();assert srdf.find("group[@name='right_hand']") is not None and srdf.find("group[@name='left_hand']") is None
    controllers=yaml.safe_load((OUT/'config/ros2_controllers_mock.yaml').read_text('utf-8'));assert controllers['hand_controller']['ros__parameters']['joints']==controlnames
    for row in read(OUT/'provenance/left_source_hashes.json')['sources']:assert sha(Path(row['absolute_source']))==row['sha256']
    result={'passed':True,'right_hand_derivation':'geometric reflection of left r03 about base Y=4mm, not independently supplied right CAD','links':22,'output_joints':20,'fixed_joints':1,'visual_meshes':367,'collision_meshes':367,'triangles':triangles,'mass_kg':.952,'pose_count':len(poses),'output_frame_mirror_checks':len(poses)*20,'internal_tree_frame_mirror_checks':len(poses)*(len(lf)-1),'max_output_frame_mirror_error':maxframe,'max_thumb_frame_mirror_error':maxthumb,'max_internal_frame_mirror_error':maxinternal,'max_slider_displacement_change_m':maxslider,'max_mesh_mirror_rounding_m':maxmesh,'max_STL_normal_alignment_error':normalerror,'triangle_normals_orientation_checks':checked_normals,'negative_mesh_scales':0,'inertial_checks':inertial_checks,'mounting_hole_layout_set_gap_mm':float(gap),'thumb_required_parts_preserved':required,'mock_control_names_checked':controlnames,'dynamics_tags':0,'main_URDF_sha256':sha(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf')}
    save(OUT/'checks/right_mirror_validation.json',result);validate_strokes();validate_mass_tool()
    print(json.dumps({k:v for k,v in result.items() if k not in ['inertial_checks','mock_control_names_checked']},ensure_ascii=False))

def validate_strokes():
    spec=importlib.util.spec_from_file_location('right_stroke',OUT/'tools/stroke_conversion.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    ll=read(LEFT/'config/nonlinear_loops.json')['loops'];rl=read(OUT/'config/nonlinear_loops.json')['loops'];mapping=read(OUT/'config/name_mapping.json')['output_joints'];errors={'closure_m':0.,'inverse_rad':0.,'signed_stroke_difference_m':0.};rows=[];table=[]
    for a,b in zip(ll,rl):
        for q in np.linspace(*a['review_output_bounds_rad'],901):
            l=mod.forward(a,float(q));r=mod.forward(b,float(q));back=mod.inverse(b,r['signed_mover_displacement_m'])
            errors['closure_m']=max(errors['closure_m'],r['closure_error_m']);errors['inverse_rad']=max(errors['inverse_rad'],abs(back-q));errors['signed_stroke_difference_m']=max(errors['signed_stroke_difference_m'],abs(l['signed_mover_displacement_m']-r['signed_mover_displacement_m']))
        r90=mod.forward(b,np.pi/2);n=mapping[b['output_joint']];rows.append({'joint':n,'control':b['control'],'signed_stroke_at_90_mm':r90['signed_mover_displacement_m']*1000,'required_motion_at_90_mm':r90['required_motion_mm'],'nominal_bounds_rad':b['review_output_bounds_rad'],'hardware_verified':False})
        for deg in range(0,91,5):table.append({'joint':n,'angle_deg':deg,**mod.forward(b,np.deg2rad(deg))})
    assert errors['closure_m']<1e-10 and errors['inverse_rad']<1e-8 and errors['signed_stroke_difference_m']<1e-12
    save(OUT/'config/actuator_mapping.json',{'linear_drives':rows,'rotary_drives':[renamed(n) for n in read(LEFT/'config/actuator_mapping.json')['rotary_drives']],'position_reference':'mirrored CAD zero; signed strokes unchanged','force_relation':'tau=F*ds/dq, signed virtual work; motor force and output torque are different units','speed_relation':'v_mover=q_dot*ds/dq','hardware_verified':False})
    save(OUT/'checks/right_stroke_validation.json',{'passed':True,'linear_drives':14,'samples':12614,'errors':errors})
    b=io.StringIO();w=csv.DictWriter(b,fieldnames=list(table[0]));w.writeheader();w.writerows(table);write(OUT/'docs/angle_stroke_table.csv',b.getvalue())
    for filename in ['joint_actuator_limits.csv']:
        write(OUT/'docs'/filename,renamed((LEFT/'docs'/filename).read_text('utf-8')))

def validate_mass_tool():
    test=guard(W/'mass_tool_check');test.mkdir(exist_ok=True)
    for directory in ['config','provenance','urdf','checks']:(test/directory).mkdir(exist_ok=True)
    for filename in ['config/measured_link_masses.json','config/inertial_parameters.json','provenance/part_mass_moment_ledger_density_1.json','urdf/TsenseI_hand_right_v3_2_withid.urdf']:
        shutil.copy2(OUT/filename,test/filename)
    spec=importlib.util.spec_from_file_location('rebuild_right',OUT/'tools/rebuild_inertials.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);result=mod.run(test);assert len(result)==22
    a=ET.parse(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf').getroot();b=ET.parse(test/'urdf/TsenseI_hand_right_v3_2_withid.urdf').getroot();maxcom=0.;maxI=0.
    for l,r in zip(a.findall('link'),b.findall('link')):
        maxcom=max(maxcom,float(np.max(abs(origin(l.find('inertial/origin'))-origin(r.find('inertial/origin'))))))
        maxI=max(maxI,float(np.max(abs(inertia(l.find('inertial/inertia'))-inertia(r.find('inertial/inertia'))))))
    assert maxcom<1e-10 and maxI<1e-12
    save(OUT/'checks/right_inertial_rebuild_tool_validation.json',{'passed':True,'links':22,'isolated_copy':str(test),'max_COM_origin_matrix_difference':maxcom,'max_inertia_tensor_difference':maxI})

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
