from common import *
import copy,shutil,csv,io

def transform_robot(left):
    r=copy.deepcopy(left);r.set('name',renamed(r.get('name')))
    for link in r.findall('link'):
        oldname=link.get('name');root=oldname=='base_link';link.set('name',renamed(oldname))
        A=F if root else D
        for inertial in link.findall('inertial'):
            e=inertial.find('origin');set_origin(e,A@origin(e)@D);set_inertia(inertial.find('inertia'),S@inertia(inertial.find('inertia'))@S)
        for kind in ['visual','collision']:
            for e in link.findall(kind):
                if 'name' in e.attrib:e.set('name',renamed(e.get('name')))
                # Root meshes themselves use affine F; other meshes use S.
                o=e.find('origin');set_origin(o,A@origin(o)@np.linalg.inv(A))
                for mesh in e.iter('mesh'):mesh.set('filename',renamed(mesh.get('filename')))
    for j in r.findall('joint'):
        parent=j.find('parent').get('link');set_origin(j.find('origin'),(F if parent=='base_link' else D)@origin(j.find('origin'))@D)
        axis=j.find('axis')
        if axis is not None:
            a=np.array(list(map(float,axis.get('xyz').split())));axis.set('xyz',nums((-S if j.get('type') in ['revolute','continuous'] else S)@a))
        j.set('name',renamed(j.get('name')))
        for e in [j.find('parent'),j.find('child')]:e.set('link',renamed(e.get('link')))
    r.insert(0,ET.Comment('Right-hand geometric mirror of left r03; base Y=0.004m plane. Physical winding and normals reflected. Joint axes transform as axial vectors; same signed output command mirrors motion. Effort/velocity are authorized placeholders; no dynamics tags.'))
    return r

def main():
    guard(OUT);guard(LEFT);assert not OUT.exists(),'Preserve an existing right-hand delivery'
    for folder in ['urdf','meshes/right','config','launch','rviz','tools','docs','checks','provenance/left_r03_source','provenance/implementation']:(OUT/folder).mkdir(parents=True,exist_ok=True)
    # Baseline data retained explicitly as source evidence, never as active RHS data.
    sourcefiles=['urdf/TsenseI_hand_left_v3_2_withid.urdf','config/structure_mesh_manifest.json','config/inertial_parameters.json','config/coordinate_frames.json','config/name_mapping.json','config/nonlinear_loops.json','config/actuator_mapping.json','checks/r03_model_validation.json','checks/ROS2_runtime_validation.json','checks/collision_rigid_20.json','checks/collision_nonlinear.json','provenance/native_solid_mass_properties_density_1.json','provenance/part_mass_moment_ledger_density_1.json','provenance/full_linkage_DISPLAY_ONLY_source.urdf.txt','provenance/link质量_来源.xlsx']
    sources=[]
    for rel in sourcefiles:
        src=guard(LEFT/rel);target=OUT/'provenance/left_r03_source'/rel
        if target.suffix=='.urdf':target=target.with_suffix('.urdf.txt')
        guard(target);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target);sources.append({'relative_source':rel,'absolute_source':str(src),'sha256':sha(src),'snapshot':target.relative_to(OUT).as_posix()})
    left=ET.parse(LEFT/'urdf/TsenseI_hand_left_v3_2_withid.urdf').getroot();right=transform_robot(left)
    xmlwrite(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf',right)
    ros=copy.deepcopy(right)
    for mesh in ros.iter('mesh'):mesh.set('filename','package://'+PACKAGE+'/'+mesh.get('filename').removeprefix('../'))
    xmlwrite(OUT/'urdf/TsenseI_hand_right_ros.urdf',ros)
    manifest=read(LEFT/'config/structure_mesh_manifest.json');mr=[];maxerror=0.
    for row in manifest['meshes']:
        a=stl(LEFT/row['mesh']);b=a.copy();root=row['target_link']=='base_link'
        reflected=a['vertices'].astype(float)@S
        if root:reflected+=SHIFT
        reflected=reflected[:,[0,2,1],:];b['vertices']=reflected.astype('<f4')
        actual=b['vertices'].astype(float);n=np.cross(actual[:,1]-actual[:,0],actual[:,2]-actual[:,0]);length=np.linalg.norm(n,axis=1)
        b['normal']=(n/np.maximum(length[:,None],1e-30)).astype('<f4')
        maxerror=max(maxerror,float(np.max(np.abs(reflected-b['vertices']))))
        new=copy.deepcopy(row);new['name']=renamed(row['name']);new['mesh']=renamed(row['mesh']);new['target_link']=renamed(row['target_link']);new['source_link']=source_name(row['source_link'])
        new['source_visual_to_target_column_major']=flat((F if root else D)@matrix(row['source_visual_to_target_column_major'])@D)
        new['left_source_mesh']=row['mesh'];new['left_source_mesh_sha256']=sha(LEFT/row['mesh']);new['geometry_derivation']='left native mesh reflected about local Y, triangle winding reversed; root uses base Y=4mm affine reflection; normals recomputed from actual right float vertices'
        write_stl(OUT/new['mesh'],b);new['mesh_sha256']=sha(OUT/new['mesh']);mr.append(new)
    modelid='right-mirror-'+manifest['model_id'];manifest.update({'model_id':modelid,'meshes':mr});save(OUT/'config/structure_mesh_manifest.json',manifest)
    save(OUT/'config/collision_mesh_manifest.json',{'model_id':modelid,'rows':[{k:r[k] for k in ['name','mesh','target_link','source_link','category','native_part_id','urdf_collision_enabled','detail_motion_in_20_joint_URDF','collision_basis']} for r in mr],'geometry':'Reflected complete left-r03 native surfaces; cavities preserved, triangle winding corrected.'})
    # Copy scalar/interface policies; every active name is RHS.
    for filename in ['joint_limits.json','hardware_parameters_pending.json','mesh_materials.json','controller_joint_order.json','measured_link_masses.json','all_joint_90_strokes.json','output_range_policy.json']:
        data=names(read(LEFT/'config'/filename))
        if filename=='measured_link_masses.json':
            source=read(LEFT/'config'/filename)
            for old,new in zip(source['rows'],data['rows']):new['source_name']=old['source_name'];new['name_match']='mapped from adopted left-hand mass table; mirror preserves mass'
            data['user_policy']+=' Right mirror reuses these masses; no independent right-hand weighing.'
        if isinstance(data,dict):data['right_hand_derivation']='Mirror of current left r03; hardware data remain provisional.'
        if 'model_id' in data:data['model_id']=modelid
        save(OUT/'config'/filename,data)
    mapping=read(LEFT/'config/name_mapping.json')
    newmapping={'output_joints':{source_name(k):renamed(v) for k,v in mapping['output_joints'].items()},'control_to_named_joint':names(mapping['control_to_named_joint']),'source_links_to_output_links':{source_name(k):renamed(v) for k,v in mapping['source_links_to_output_links'].items()},'fixed_part_overrides':names(mapping['fixed_part_overrides']),'thumb_F2':'right_hand_thumb_MCP_joint','thumb_F1':'right_hand_thumb_PIP_joint'}
    save(OUT/'config/name_mapping.json',newmapping)
    # Source internal tree is rebased from CAD into the right base_link coordinates.
    source=ET.parse(LEFT/'provenance/full_linkage_DISPLAY_ONLY_source.urdf.txt').getroot();BC=matrix(read(LEFT/'checks/r02_baseline/visual_conversion_manifest.json')['base_from_CAD_column_major']);rsource=ET.Element('robot',name='right_hand_full_linkage_AUDIT_ONLY')
    for l in source.findall('link'):ET.SubElement(rsource,'link',name=source_name(l.get('name')))
    for j0 in source.findall('joint'):
        j=copy.deepcopy(j0);par=j.find('parent').get('link');set_origin(j.find('origin'),(F@BC if par=='hand_base' else D)@origin(j.find('origin'))@D)
        ax=j.find('axis')
        if ax is not None:ax.set('xyz',nums((-S if j.get('type') in ['revolute','continuous'] else S)@np.array(list(map(float,ax.get('xyz').split())))))
        j.set('name',source_name(j.get('name')))
        for e in [j.find('parent'),j.find('child')]:e.set('link',source_name(e.get('link')))
        rsource.append(j)
    xmlwrite(OUT/'provenance/right_full_linkage_AUDIT_ONLY.urdf.txt',rsource)
    loops=read(LEFT/'config/nonlinear_loops.json')
    for row in loops['loops']:
        for key in ['H_parent_m','A0_parent_m','B0_parent_m','slide_axis_parent']:row[key]=(S@np.array(row[key])).tolist()
        row['axis_parent']=(-S@np.array(row['axis_parent'])).tolist()
        for key in ['output_joint','slider_joint','rod_joint','parent_link']:row[key]=source_name(row[key])
        row['geometry_source']='Reflected left r03 candidate mechanism, same assembly branch; right physical installation not independently measured.'
    loops['model_id']=modelid;loops['handedness']='right';loops['right_derivation']='polar vectors S*v; rotational axis -S*a. The signed mover displacement and output command values are preserved.'
    save(OUT/'config/nonlinear_loops.json',loops)
    # Mirrored moments keep the mass rebuilding tool self-contained.
    moments=read(LEFT/'provenance/native_solid_mass_properties_density_1.json')
    for row in moments.values():
        row['center_normalized_mesh_m']=(S@np.array(row['center_normalized_mesh_m'])).tolist();row['inertia_at_COM_density_1']=(S@np.array(row['inertia_at_COM_density_1'])@S).tolist();row['right_hand_derivation']='reflected normalized geometry frame; native_file and native labels refer to original left source evidence'
    save(OUT/'provenance/native_solid_mass_properties_density_1.json',moments)
    ledger=read(LEFT/'provenance/part_mass_moment_ledger_density_1.json')
    for row in ledger['parts']:
        root=row['target_link']=='base_link';row['target_link']=renamed(row['target_link']);row['center_m']=(S@np.array(row['center_m'])+(SHIFT if root else 0)).tolist();row['unit_inertia']=(S@np.array(row['unit_inertia'])@S).tolist();row['normalized_geometry_to_link_column_major']=flat((F if root else D)@matrix(row['normalized_geometry_to_link_column_major'])@D)
    save(OUT/'provenance/part_mass_moment_ledger_density_1.json',ledger)
    params=read(LEFT/'config/inertial_parameters.json')
    for row in params['links']:
        root=row['link']=='base_link';row['link']=renamed(row['link']);row['center_of_mass_link_m']=(S@np.array(row['center_of_mass_link_m'])+(SHIFT if root else 0)).tolist();row['center_of_mass_base_zero_m']=(S@np.array(row['center_of_mass_base_zero_m'])+SHIFT).tolist()
        I=S@np.array(row['inertia_at_COM_link_axes_kg_m2'])@S;row['inertia_at_COM_link_axes_kg_m2']=I.tolist();values,axes=np.linalg.eigh(I)
        if np.linalg.det(axes)<0:axes[:,-1]*=-1
        row['principal_moments_kg_m2']=values.tolist();row['principal_axes_in_link']=axes.tolist()
    params['model_id']=modelid;params['handedness']='right';params['density_model']+=' Right values are geometric mirror estimates, not measured right-hand COM/inertias.'
    save(OUT/'config/inertial_parameters.json',params)
    for directory in ['launch','rviz','tools','examples']:
        (OUT/directory).mkdir(exist_ok=True)
        for src in (LEFT/directory).iterdir():
            if src.is_file():write(OUT/directory/renamed(src.name),renamed(src.read_text('utf-8')))
    for src in (LEFT/'config').glob('*.yaml'):write(OUT/'config'/renamed(src.name),renamed(src.read_text('utf-8')))
    for filename in ['TsenseI_hand_left.srdf','20个控制关节_命名.txt']:write(OUT/'config'/renamed(filename),renamed((LEFT/'config'/filename).read_text('utf-8')))
    write(OUT/'urdf/TsenseI_hand_right_mock.urdf.xacro',renamed((LEFT/'urdf/TsenseI_hand_left_mock.urdf.xacro').read_text('utf-8')))
    write(OUT/'package.xml',renamed((LEFT/'package.xml').read_text('utf-8')).replace('Raised 5mm,','Right geometric mirror, raised 5mm,'))
    write(OUT/'CMakeLists.txt',renamed((LEFT/'CMakeLists.txt').read_text('utf-8')))
    generate_frames(right)
    holes=read(LEFT/'checks/r02_baseline/mount_hole_alignment.json')['current_mount_holes'];right_holes=[]
    for row in holes:
        r=copy.deepcopy(row);r['center_mm']=(S@np.array(row['center_mm'])+SHIFT*1000).tolist();right_holes.append(r)
    save(OUT/'config/mirror_definition.json',{'left_package':LEFT_PACKAGE,'right_package':PACKAGE,'left_model_id':read(LEFT/'config/nonlinear_loops.json')['model_id'],'right_model_id':modelid,'base_coordinate_relation':'x_R=x_L, y_R=0.008-y_L, z_R=z_L','mirror_plane_base_m':{'normal':[0,1,0],'point':[0,.004,0]},'S':S.tolist(),'F_column_major':flat(F),'root_mesh_and_COM':'S*p+[0,0.008,0]','other_link_mesh_and_COM':'S*p','root_child_joint_origin':'F*T*D','other_joint_origin':'D*T*D','revolute_axis':'-S*axis; angular axes are axial vectors','prismatic_axis':'S*axis','inertia':'S*I*S^T; ixy and iyz reverse sign, ixz unchanged','joint_command_relation':'q_right=q_left for every corresponding output; same bounds and signed mover strokes','STL_winding':'vertices [0,2,1], normals S*n, positive mesh scales 1 1 1','left_mount_holes':holes,'right_mount_holes':right_holes,'native_right_CAD_supplied':False,'max_mesh_rounding_m':maxerror})
    save(OUT/'provenance/left_source_hashes.json',{'sources':sources})
    save(OUT/'config/collision_policy.json',{'collision_mesh_count':367,'geometry':'Geometric mirror of all left r03 native collision surfaces; not convex-filled','reference_model':'20-output rigid approximation with fixed zero-pose auxiliary details','planning_exclusions':'Direct adjacent links only; cross-finger and nonadjacent thumb/palm remain checked','mirror_audit':'Left r03 collision results transferred by proven isometry; right FCL spot checks and full pose-transform checks performed separately','hardware_verified':False,'continuous_collision_certificate':False})
    print(json.dumps({'package':str(OUT),'mesh_count':len(mr),'max_mesh_rounding_m':maxerror,'main_URDF':str(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf')},ensure_ascii=False))

def generate_frames(robot):
    frames=fk(robot);rows=[];b=io.StringIO();writer=csv.writer(b);writer.writerow(['joint','type','parent','child','origin_xyz_m','origin_rpy_rad','axis_joint_frame','lower_rad','upper_rad'])
    for j in robot.findall('joint'):
        axis=j.find('axis');lim=j.find('limit');row={'name':j.get('name'),'type':j.get('type'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),'origin':j.find('origin').attrib,'axis':axis.attrib if axis is not None else None,'child_zero_in_base_column_major':flat(frames[j.find('child').get('link')])};rows.append(row)
        writer.writerow([row['name'],row['type'],row['parent'],row['child'],row['origin']['xyz'],row['origin']['rpy'],axis.get('xyz') if axis is not None else '',lim.get('lower') if lim is not None else '',lim.get('upper') if lim is not None else ''])
    save(OUT/'config/tree.json',{'root_link':'base_link','links':[l.get('name') for l in robot.findall('link')],'joints':rows});save(OUT/'config/coordinate_frames.json',{'root':'base_link','length_unit':'m','angle_unit':'rad','mass_unit':'kg','inertia_unit':'kg*m^2','positive_angle':'Right-hand rule about each right-hand URDF joint axis; signed commands preserved relative to left model','mirror_plane_base_y_m':.004,'joints':rows});write(OUT/'docs/joint_frames.csv',b.getvalue())
    params=read(OUT/'config/inertial_parameters.json');lines=['右手各link质量、质心、惯量：单位kg、m、kg*m^2；掌部150g+腕座17g单独采用；总952g。']
    for row in params['links']:lines.append(row['link']+'\n'+json.dumps(row,ensure_ascii=False))
    write(OUT/'config/质量_质心_惯量参数表.txt','\n\n'.join(lines)+'\n')

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
