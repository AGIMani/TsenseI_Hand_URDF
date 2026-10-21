"""Publish the verified reduced variant without replacing the full release."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import argparse,hashlib,json,shutil,subprocess,sys,zipfile,xml.etree.ElementTree as ET
from build_hand_20j_simplified import regular,sha,dump,read

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/'work/hand_1009_20j_reduction_20261016'
DELIVERY=ROOT/'outputs/hand_1009_raised_20j_simplified_20261016_r01'
PACKAGE=DELIVERY/'hand_1009_reduced_description'
SOURCE=ROOT/'outputs/hand_1009_raised_collision_checked_20261016_r07/hand_1009_description'


def compare_files(a,b,names):
    for rel in names:assert sha(a/rel)==sha(b/rel),rel


def resource_check(package):
    count=0
    for name in ['hand_1009_20j.urdf','hand_1009_20j_ros.urdf']:
        robot=ET.parse(package/'urdf'/name).getroot()
        assert len(robot.findall('link'))==21 and len(robot.findall('joint'))==20
        assert all(j.get('type')=='revolute' for j in robot.findall('joint'))
        for mesh in robot.findall('.//mesh'):
            uri=mesh.get('filename')
            f=package/uri[len('package://hand_1009_reduced_description/'):] if uri.startswith('package://') else (package/'urdf'/uri).resolve()
            assert f.is_file() and f.resolve().is_relative_to(package.resolve())
            regular(f);count+=1
    return count


def main():
    global WORK,DELIVERY,PACKAGE,SOURCE
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work-dir',type=Path,default=WORK)
    parser.add_argument('--delivery-dir',type=Path,default=DELIVERY)
    parser.add_argument('--source-package',type=Path,default=SOURCE)
    parser.add_argument('--archive-name',default='hand_1009_20关节简化版_r01.zip')
    parser.add_argument('--web-archive-name')
    args=parser.parse_args()
    WORK=args.work_dir.absolute();DELIVERY=args.delivery_dir.absolute();PACKAGE=DELIVERY/'hand_1009_reduced_description';SOURCE=args.source_package.absolute()
    archive=DELIVERY/args.archive_name
    regular(archive)
    assert not archive.exists(),'Publish a new revision rather than overwriting an existing delivery'
    if args.web_archive_name:assert not (DELIVERY/args.web_archive_name).exists()
    sys.stdout.reconfigure(encoding='utf-8')
    latest_path=ROOT/'outputs/hand_1009_latest_delivery.json'
    original_full_pointer=latest_path.read_bytes()
    reduced_pointer_path=ROOT/'outputs/hand_1009_latest_reduced_delivery.json'
    previous_reduced=read(reduced_pointer_path) if reduced_pointer_path.exists() else None
    previous_reduced_bytes=reduced_pointer_path.read_bytes() if reduced_pointer_path.exists() else None
    assert Path(read(latest_path)['package'])==SOURCE
    proof=read(WORK/'verification.json');browser=read(WORK/'browser_verification.json')
    assert proof['passed'] and browser['passed'] and browser['loaded']['genericSliderCount']==20
    assert browser['loaded']['loadedVisualMeshes']==303 and not browser['page_errors']
    assert not browser['loaded']['specialLinkageActive']
    assert all(u in browser['blocked_external_urls'] for u in browser['requests_failed'])
    protected=read(PACKAGE/'provenance/source_files.json')['source_files_sha256']
    assert all(sha(SOURCE/rel)==h for rel,h in protected.items())
    unchanged=read(PACKAGE/'config/model_manifest.json')['mesh_files']
    assert all(sha(PACKAGE/rel)==h==sha(SOURCE/rel) for rel,h in unchanged.items())
    regenerate=WORK/'regeneration_test/hand_1009_reduced_description'
    # The independent build must reproduce every generated file before QA additions.
    generator_files=[str(f.relative_to(regenerate)) for f in regenerate.rglob('*') if f.is_file() and '__pycache__' not in f.parts and f.suffix not in ['.pyc','.pyo']]
    compare_files(PACKAGE,regenerate,generator_files)
    shutil.copy2(WORK/'browser_verification.json',PACKAGE/'provenance/browser_verification.json')
    viewer_evidence=WORK/'official_viewer_source.json'
    if not viewer_evidence.exists():viewer_evidence=ROOT/'work/hand_1009_20j_reduction_20261016/official_viewer_source.json'
    shutil.copy2(viewer_evidence,PACKAGE/'provenance/official_viewer_source.json')
    audit=PACKAGE/'provenance/audit_source';regular(audit);audit.mkdir(exist_ok=True)
    for script in ['verify_hand_20j_simplified.py','verify_hand_20j_browser.cjs','package_hand_20j_simplified.py']:
        shutil.copy2(ROOT/'scripts'/script,audit/script)
    if (WORK/'naming_verification.json').exists():
        naming=read(WORK/'naming_verification.json');assert naming['passed']
        proof['naming_migration']=naming
        naming_script='verify_hand_20j_unified_r03.py' if naming.get('same_182_structurally_named_mesh_files_as_full_r08') else 'verify_hand_20j_public_names.py'
        shutil.copy2(ROOT/'scripts'/naming_script,audit/naming_script)
    (audit/'README.txt').write_text('Audit reference only. Verification scripts refer to the source r07 package and workspace dependencies. The delivered URDF needs no audit scripts or nonlinear state adapter. tools/build_reduced.py is the self-contained standard-library generator; supply the full mechanism package as --source-package.\n',encoding='utf-8')
    approx=proof['nonzero_pose_approximation_sample'];moments=proof['zero_pose_moments']
    note=f'''20关节简化版核对结果（2026-10-16）
修订：{proof['representation_id']}

21个刚体，20个旋转关节；无动子、连杆活动关节；原始303个可视零件和303个碰撞实例均保留。
54个来源刚体恰好归并一次，182份网格文件逐字节未变；完整r07未改动。
质量合计{moments['mass_kg']*1000:.9f}g，未拟合到1130g。
127个姿态对比：20个输出关节的姿态与完整模型一致，最大矩阵元素误差{proof['max_output_FK_matrix_error']:.3e}。
零姿态外观与碰撞实例放置一致，最大矩阵元素误差{proof['max_zero_visual_collision_matrix_error']:.3e}。
零姿态整手质心差{moments['COM_difference_m']:.3e}m；整手惯量矩阵最大元素差{moments['tensor_max_difference_kg_m2']:.3e}kg·m²，均仅有浮点误差。
所有合并后刚体惯量正定，并满足主惯量三角不等式。

固定等效的实际影响：
126个非零抽样姿态中，与完整机构相比，整手质心偏差最大{approx['max_COM_difference_mm']:.6f}mm；
整手绕自身质心的惯量矩阵相对Frobenius范数差最大{approx['max_whole_hand_COM_tensor_relative_Frobenius_difference']*100:.6f}%。
这是本次抽样结果，不是所有姿态的误差上界；局部刚体及关节等效动力学误差不能用该整手数字代替。
固定连杆和动子可能脱开或交叠，反映简化表达，不是完整机构的真实运动。

加载验证：
官方Robot Viewer通用加载器在本地隔离无头浏览器中实际加载了21刚体、20关节、303个可视网格。
通用关节面板只有20个滑块，无专用联动模块；6个姿态的浏览器加载矩阵与独立计算一致。
标准ROS URDF解析器通过两份URDF；所有相对路径和ROS包资源路径均解析到交付网格。
尚未在真实ROS 2运行环境启动launch，未进行硬件控制、编码器标定或简化模型碰撞可行性验证。
网页减面轻量包尚未制作。源模型中质量、材料、分重、effort/velocity占位等不确定性继续保留。
'''
    (PACKAGE/'docs/本版核对结果.txt').write_text(note,encoding='utf-8')
    proof.update(standard_generator_independently_reproduced_files=len(generator_files),
                 generic_viewer_import_passed=True,generic_viewer_independent_sliders=20,
                 generic_viewer_visual_meshes=303,browser_special_linkage_adapter_used=False)
    dump(PACKAGE/'provenance/verification.json',proof)
    manifest_path=PACKAGE/'provenance/delivery_manifest.json'
    all_files={str(f.relative_to(PACKAGE)).replace('\\','/'):{'sha256':sha(f),'bytes':f.stat().st_size}
               for f in sorted(PACKAGE.rglob('*')) if f.is_file() and f!=manifest_path and '__pycache__' not in f.parts and f.suffix not in ['.pyc','.pyo']}
    dump(manifest_path,{'representation_id':proof['representation_id'],'source_model_id':proof['source_model_id'],'files':all_files})
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for f in sorted(PACKAGE.rglob('*')):
            if f.is_file() and '__pycache__' not in f.parts and f.suffix not in ['.pyc','.pyo']:regular(f);z.write(f,f.relative_to(DELIVERY))
    portable=WORK/'portable_check';regular(portable);portable.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        for n in z.namelist():assert (portable/n).resolve().is_relative_to(portable.resolve())
        z.extractall(portable)
    moved=portable/'hand_1009_reduced_description'
    compare_files(PACKAGE,moved,[*all_files,'provenance/delivery_manifest.json'])
    resource_references=resource_check(moved)
    # Run the delivered generator in an unrelated cwd, not the repository script.
    regenerated=WORK/'portable_regenerated'
    result=subprocess.run([sys.executable,str(moved/'tools/build_reduced.py'),'--source-package',str(SOURCE),'--output-dir',str(regenerated)],cwd=WORK,capture_output=True,text=True,encoding='utf-8')
    assert result.returncode==0,result.stdout+result.stderr
    compare_files(PACKAGE,regenerated/'hand_1009_reduced_description',generator_files)
    assert latest_path.read_bytes()==original_full_pointer
    assert all(sha(SOURCE/rel)==h for rel,h in protected.items())
    web_info=None
    if args.web_archive_name:
        web_archive=DELIVERY/args.web_archive_name;regular(web_archive)
        web_files=['urdf/hand_1009_20j.urdf',*unchanged]
        if (PACKAGE/'config/joint_name_map.json').exists():web_files+=['config/joint_name_map.json','config/naming_policy.json','config/independent_controls.json']
        for rel in ['config/mesh_name_map.json','config/structural_mesh_naming_policy.json','config/range_update_r08.json','docs/20关节名称与范围.csv','docs/网格结构名称对应表.csv']:
            if (PACKAGE/rel).exists():web_files.append(rel)
        web_note='网页上传包：加载 hand_1009_reduced_description/urdf/hand_1009_20j.urdf，并提供同包 meshes 文件夹。\n仅20个活动关节，四指侧摆命名为 mcp_abduction。网格未减面；动子、连杆固定等效。零位、方向、范围、质量惯量与配套完整对外交付包一致。\n这是一份运动模型，未包含产品硬件驱动或真机标定。\n'
        with zipfile.ZipFile(web_archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
            for rel in sorted(web_files):z.write(PACKAGE/rel,'hand_1009_reduced_description/'+rel)
            z.writestr('使用说明.txt',web_note.encode('utf-8'))
        web_extracted=WORK/'web_upload_portable';regular(web_extracted);web_extracted.mkdir(exist_ok=True)
        with zipfile.ZipFile(web_archive) as z:
            for n in z.namelist():assert (web_extracted/n).resolve().is_relative_to(web_extracted.resolve())
            z.extractall(web_extracted)
        web_package=web_extracted/'hand_1009_reduced_description'
        compare_files(PACKAGE,web_package,web_files)
        main_robot=ET.parse(web_package/'urdf/hand_1009_20j.urdf').getroot()
        assert len(main_robot.findall('joint'))==20 and len(list(web_package.rglob('*.urdf')))==1
        for mesh in main_robot.findall('.//mesh'):assert (web_package/'urdf'/mesh.get('filename')).resolve().is_file()
        web_info={'zip':str(web_archive),'zip_sha256':sha(web_archive),'zip_bytes':web_archive.stat().st_size,
                  'single_URDF':True,'resource_references_resolved':len(main_robot.findall('.//mesh')),'geometry_decimated':False,
                  'URDF_and_meshes_byte_identical_to_customer_package':True}
    finished=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds')
    proof.update(portable_extraction_files_identical=True,portable_resource_references_resolved=resource_references,
                 delivered_generator_executed_from_unrelated_directory=True,full_latest_pointer_unchanged=True,
                 archive_sha256=sha(archive),archive_bytes=archive.stat().st_size,completed_China=finished)
    if web_info:proof['web_upload_archive']=web_info
    dump(DELIVERY/'verification.json',proof)
    pointer={'representation':'20-output fixed-follower approximation','representation_id':proof['representation_id'],
             'source_model_id':proof['source_model_id'],'source_full_package':str(SOURCE),'package':str(PACKAGE),
             'URDF':str(PACKAGE/'urdf/hand_1009_20j.urdf'),'ROS_URDF':str(PACKAGE/'urdf/hand_1009_20j_ros.urdf'),
             'zip':str(archive),'zip_sha256':proof['archive_sha256'],'zip_bytes':proof['archive_bytes'],
             'verification':str(DELIVERY/'verification.json'),'README':str(PACKAGE/'README.md'),
             'mass_g':moments['mass_kg']*1000,'controls':20,'links':21,'source_full_package_unchanged':True,
             'follower_geometry':'frozen at adopted zero to actuator mounting side',
             'web_mesh_decimation_performed':False,'completed_China':finished}
    if previous_reduced:pointer['previous_reduced_package']=previous_reduced['package']
    naming_file=PACKAGE/'config/joint_name_map.json'
    if naming_file.exists():
        names=read(naming_file);pointer.update(public_interface_id=names['public_interface_id'],
                                             naming_policy_id=names['policy_id'],joint_name_map=str(naming_file))
    if web_info:pointer['web_upload_archive']=web_info
    assert (reduced_pointer_path.read_bytes() if reduced_pointer_path.exists() else None)==previous_reduced_bytes,'Another task published a reduced model during packaging'
    dump(reduced_pointer_path,pointer)
    print(json.dumps(pointer,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
