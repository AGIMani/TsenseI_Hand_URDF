from common import *
import copy,shutil,zipfile,hashlib

def transfer_collision(mode):
    source=read(LEFT/('checks/collision_'+mode+'.json'));result=names(copy.deepcopy(source))
    for pair in result['pairs']:
        for k in ['source_links']:pair[k]=[source_name(n) for n in pair[k]]
        if 'representative_contact_base_mm' in pair:pair['representative_contact_base_mm']=(S@np.array(pair['representative_contact_base_mm'])+SHIFT*1000).tolist()
    result['model_id']=read(OUT/'config/nonlinear_loops.json')['model_id'];result['main_URDF_sha256']=sha(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf')
    result['left_source_audit_elapsed_s']=result.pop('elapsed_s',None)
    result['right_hand_derivation']='Transferred by geometric isometry from the left r03 1011-pose triangle audit, after all mesh vertices and all pose frames were independently mirror-verified. Root mesh reflection includes <=1.312nm float rounding.'
    result['FCL_full_1011_pose_audit_rerun_on_right']=False
    result['independent_right_FCL_spot_checks']='checks/right_FCL_spot_validation.json: 6 poses per motion model, all crossing pairs match left source.'
    result['method']='Mirror-isometry transfer of the left FCL BVH evidence; independently spot-checked on new right meshes. '+source['method']
    save(OUT/('checks/collision_'+mode+'_mirror_transfer.json'),result)
    return result

def main():
    mirror=read(OUT/'checks/right_mirror_validation.json');strokes=read(OUT/'checks/right_stroke_validation.json');runtime=read(OUT/'checks/ROS2_runtime_validation.json');spot=read(OUT/'checks/right_FCL_spot_validation.json');mass=read(OUT/'checks/right_inertial_rebuild_tool_validation.json')
    assert mirror['passed'] and strokes['passed'] and mass['passed'] and spot['all_crossing_pairs_match_source']
    assert runtime['headless_display_launch_passed'] and runtime['joint_state_name_count']==20 and all(n.startswith('right_hand_') for n in runtime['joint_states']['names'])
    if 'STL_normals_mirror_error' in mirror:mirror['max_STL_normal_alignment_error']=mirror.pop('STL_normals_mirror_error');save(OUT/'checks/right_mirror_validation.json',mirror)
    definition=read(OUT/'config/mirror_definition.json');definition['STL_winding']='Vertices [0,2,1]; normals recomputed from actual right float vertices; positive mesh scales 1 1 1';save(OUT/'config/mirror_definition.json',definition)
    rigid=transfer_collision('rigid_20');nonlinear=transfer_collision('nonlinear')
    runtime['observed_startup_diagnostic']='KDL TF parser warns that root base_link inertia is unused by KDL; the physical URDF retains all 22 masses/inertias. TF/joint-state checks passed.'
    runtime['test_shutdown_diagnostic']='Audit stopped launch by SIGINT after validation; child joint_state_publisher may log KeyboardInterrupt during teardown.'
    save(OUT/'checks/ROS2_runtime_validation.json',runtime)
    write(OUT/'README.txt',f'''TsenseI 右手 · 抬高5mm · 中指DIP统一 · 20输出关节完整模型 r03（2026-10-13）

这是当前左手r03完整包的几何镜像右手版。没有另行提供、重建或核验独立右手实物CAD。左右各自为独立包，左手文件保持原样。

查看器主文件：urdf/TsenseI_hand_right_v3_2_withid.urdf。保留整个同级meshes目录，所有外观和碰撞网格在meshes/right。
ROS文件：urdf/TsenseI_hand_right_ros.urdf，使用package://tsensei_hand_right_raised5mm_20_outputs_description/路径。两个URDF表达同一模型；只有这两个文件以.urdf结尾，历史/内部审计树以.urdf.txt保存。

模型：22个link、21个joint，即20个旋转输出关节+1个固定腕座连接。输出名全部为right_hand_*；腕座为Right_hand_palm_link，固定关节Right_hand_palm_joint；根仍为base_link。四指各MCP、swing、PIP、DIP；拇指为rota_block、rotaback_joint2、MCP/F2、PIP/F1。完整命名见config/20个控制关节_命名.txt。

几何：367份结构STL，367份碰撞几何；外观、碰撞、关节位置、旋转轴、质心和完整惯量同步转换。镜像后的三角绕序反转，法线由右手实际顶点重新计算；无负mesh scale。拇指两级输出法兰、一级抬高连接件、安装支架、固定在手掌的一级电机安装座全部保留。

镜像平面：base_link坐标的Y=4mm平面，沿手掌宽度方向做左右镜像，保持腕部安装孔布局和Z=-4mm安装面。
x_R=x_L；y_R=8mm-y_L；z_R=z_L。
腕部安装孔集合左右最大拟合差{mirror['mounting_hole_layout_set_gap_mm']:.8g}mm。该平面取自左手已经核验的安装孔中心，不是把手沿掌面翻转。

控制：同名对应关节q_R=q_L，因此相同有符号指令产生镜像动作。旋转轴按轴矢量转换，axis_R=-S*axis_L；角度界限原值保留，包括拇指二级-1.74..0rad。14路直线电机的有符号动子位移、0..90°所需行程与左手一致。普通20关节URDF依然将内部动子和连杆冻结在零位；原非线性机构参数与独立正反算工具另附。

质量惯量：总952g；掌部base_link150g不含腕座，Right_hand_palm_link17g单独采用；四指各155g，拇指165g。采用原质量表的对应重量，镜像不改变质量、体积和主惯量。质心镜像，完整惯量I_R=S*I_L*S^T，其中ixy、iyz变号，ixz不变。原质量表中left名称作为来源证据保留，活动模型使用right名称。质量沿用原表，COM/惯量沿用几何等效密度估算，均非独立右手实测。

限制与动态：effort=1Nm、velocity=1rad/s仍是用户确认的展示/模拟占位值，真实角度范围、力矩/推力、速度、有效动子行程、通道和编码器零位仍在config/hardware_parameters_pending.json待实测。未加入关节摩擦和阻尼。

配套：ROS2显示启动和RViz、20输出模拟控制接口、MoveIt/OMPL几何规划分组、零位、坐标和角度—行程表、质量重算工具都已改为右手名称及参数。见docs/使用说明.txt。ROS2的模拟控制入口不连接硬件；规划入口不执行轨迹。

验证：367份网格全部顶点镜像对照，{mirror['triangle_normals_orientation_checks']}个非退化三角法线/绕序对照；1,011组姿态、20,220次输出坐标对照和53,583次内部机构坐标对照，拇指误差不超过{mirror['max_thumb_frame_mirror_error']:.3g}（矩阵分量）。全部22个质量、质心和惯量验证通过，质量重算工具在独立副本验证通过。14路行程12,614个点正反解及左右行程一致性通过。在ROS2 Jazzy环境实际构建、加载并发布20个right_hand关节状态、20个运动TF及1个固定TF；安装的ROS URDF SHA与本包一致。GUI/RViz、mock控制与MoveIt运行未验收，所需缺失依赖和检查范围列在checks/ROS2_runtime_validation.json。

碰撞：镜像保留左手的碰撞问题和名义范围限制。左手1,011组姿态的审计结果经上述完整镜像等价核验转换到右手；右手两种运动模型另对各6个代表姿态做独立FCL检查，全部相交对与左手一致。并未重新对右手运行全部1,011组FCL。右手零位同样有掌盖与拇指外壳相交，运动范围内存在拇指与其他手指相交，未验收为全程无碰撞。完整复现见docs/碰撞检查结论.txt及checks/collision_*_mirror_transfer.json。

预览：preview/left_right_comparison.png，左侧为原左手，右侧为镜像右手；含零位和拇指组合姿态。
所有交付路径和各级父目录均为E盘真实普通目录，无目录联接或符号链接。
''')
    instructions=renamed((LEFT/'docs/使用说明.txt').read_text('utf-8'))
    instructions=instructions.replace('当前URDF是20输出刚体近似','右手由左手r03沿base_link的Y=4mm平面镜像；root mesh/COM采用y=8mm-y，局部非根坐标采用Y变号，旋转轴按轴矢量变换。相同关节指令和同一有符号行程产生对应镜像动作。\n\n当前URDF是20输出刚体近似')
    write(OUT/'docs/使用说明.txt',instructions)
    lines=['右手镜像与碰撞检查（2026-10-13）','',
        '右手模型来源：左手r03沿base_link的Y=4mm平面做几何镜像；无独立右手实物CAD核验。',
        '367份网格全部顶点、22刚体的COM/惯量、20个输出轴/坐标及原14套机构按反射公式转换。全部1,011组输出和非线性姿态的矩阵镜像等价通过。',
        '碰撞表来源：左手原始FCL审计，经过几何等距映射转换为右手。没有重新对右手完整扫描1,011组FCL；右手各模型6个代表姿态的独立FCL检查与左手相交集合完全一致。',
        '原三角表面保留所有腔体、法兰、支架、小件及内部零位几何；20输出主URDF的内部动子/连杆仍为零位刚体细节。',
        '主URDF刚体模型：原1,011组中共'+str(rigid['nonadjacent_output_crossing_pair_count'])+'对非相邻网格相交，其中涉及拇指'+str(rigid['thumb_nonadjacent_output_crossing_pair_count'])+'对。',
        '原非线性机构：原1,011组中共'+str(nonlinear['nonadjacent_output_crossing_pair_count'])+'对非相邻输出网格相交，其中涉及拇指'+str(nonlinear['thumb_nonadjacent_output_crossing_pair_count'])+'对。',
        '零位仍发现P0227掌盖与P0364拇指外壳相交。镜像不会消除原几何的相交；关节名义范围不代表已确认的安全范围。',
        'SRDF仅排除直接相邻关节的装配接触，未排除不同手指、非相邻拇指/手掌或所有零位碰撞。',
        '相交余量指标是原审计的三角平面双侧筛选值，不是穿透深度或干涉体积。硬件装配、真实行程和连续碰撞安全均待确认。',
        '完整右手复现角度和反射后的接触点见checks/collision_rigid_20_mirror_transfer.json、checks/collision_nonlinear_mirror_transfer.json。']
    write(OUT/'docs/碰撞检查结论.txt','\n'.join(lines)+'\n')
    # Add the preview directory to installation; no symlink installation is used.
    p=OUT/'CMakeLists.txt';text=p.read_text('utf-8').replace('launch rviz docs DESTINATION','launch rviz docs preview DESTINATION');write(p,text)
    save(OUT/'checks/right_preview_validation.json',{'file':'preview/left_right_comparison.png','inspected':True,'method':'Full native surface Y/Z projection of the actual left/right URDF geometry at zero and matching thumb combination; plotted with Python matplotlib and visually inspected','real_directory':str(OUT/'preview'),'no_browser_import_claimed':True})
    for p in W.glob('*.py'):guard(p);shutil.copy2(p,OUT/'provenance/implementation'/p.name)
    save(OUT/'checks/completion_summary.json',{'right_geometric_mirror':True,'native_right_CAD_supplied':False,'links':22,'output_joints':20,'mass_kg':.952,'visual_collision_meshes':367,'joint_command_relation':'right q equals corresponding left q','signed_linear_motor_strokes_unchanged':True,'mirror_verified_poses':1011,'independent_FCL_spot_pose_queries':12,'ROS2_build_and_20_joint_state_TF_publication_passed':True,'mock_and_MoveIt_runtime_checked':False,'effort_velocity':'authorized display/mock placeholders','joint_dynamics_added':False,'inherits_left_collision_findings':True,'main_URDF_sha256':sha(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf')})
    files=sorted(p for p in OUT.rglob('*') if p.is_file())
    for p in [OUT,*OUT.rglob('*')]:guard(p)
    save(OUT/'checks/real_path_validation.json',{'all_files_and_ancestors_are_ordinary':True,'root':str(OUT),'files_checked':len(files),'no_junction_or_symlink':True})
    files=sorted(p for p in OUT.rglob('*') if p.is_file() and p!=OUT/'checks/file_manifest.json')
    save(OUT/'checks/file_manifest.json',{'algorithm':'SHA256','files':[{'path':p.relative_to(OUT).as_posix(),'bytes':p.stat().st_size,'sha256':sha(p)} for p in files]})
    archive=guard(DELIVERY/'TsenseI_右手_抬高5mm_20关节_完整模型_r03.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted(OUT.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(DELIVERY).as_posix())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for p in sorted(OUT.rglob('*')):
            if p.is_file():assert hashlib.sha256(z.read(p.relative_to(DELIVERY).as_posix())).hexdigest()==sha(p)
    digest=sha(archive);write(archive.with_suffix('.zip.sha256'),digest+'  '+archive.name+'\n')
    result={'package':str(OUT),'archive':str(archive),'sha256':digest,'bytes':archive.stat().st_size,'main_URDF':str(OUT/'urdf/TsenseI_hand_right_v3_2_withid.urdf'),'active_URDFs':[p.relative_to(OUT).as_posix() for p in OUT.rglob('*.urdf')],'preview':str(OUT/'preview/left_right_comparison.png'),'CRC_and_all_files_verified':True}
    save(W/'delivery_right_r03.json',result);print(json.dumps(result,ensure_ascii=False))

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
