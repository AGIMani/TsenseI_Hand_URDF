"""Apply the public naming policy while reusing the verified mass/tree reducer."""
import argparse
import copy
import csv
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys

HERE=Path(__file__).resolve().parent
DELIVERED=(HERE/'build_reduced_base.py').exists()
BASE=HERE/'build_reduced_base.py' if DELIVERED else HERE/'build_hand_20j_simplified.py'
POLICY=HERE.parent/'config/naming_policy.json' if DELIVERED else HERE/'hand_description_templates/public_joint_naming.json'
CONVERTER=HERE/'convert_joint_names.py' if DELIVERED else HERE/'hand_description_templates/convert_joint_names.py'
spec=importlib.util.spec_from_file_location('hand20_reducer_base',BASE)
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)


def annotate(controls,registry,policy):
    by_name={r['name']:r for r in registry['outputs']}
    result=[]
    for old in controls['controls']:
        row=copy.deepcopy(old);original=old['name'];source=by_name[original]
        row['name']=policy['renames'].get(original,original)
        row['source_joint_name']=original
        display=source['display_name'] if '外展/内收' in source['display_name'] else source['display_name'].replace('侧摆','外展/内收（侧摆）')
        row.update(display_name=display,positive_definition=source['positive_definition'],
                   motion_kind='abduction_adduction' if row['name'].endswith('_abduction') else 'flexion_extension' if row['name'].endswith('_flexion') else 'mechanical_base_rotation',
                   lower_deg=math.degrees(row['lower_rad']),upper_deg=math.degrees(row['upper_rad']))
        result.append(row)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-package',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    revision_default=base.read(HERE.parent/'config/model_manifest.json')['representation_id'] if DELIVERED else 'raised_20j_unified_mesh_thumb90_r03_20261016'
    parser.add_argument('--revision',default=revision_default)
    parser.add_argument('--naming-policy',type=Path,default=POLICY)
    args=parser.parse_args()
    policy=base.read(args.naming_policy)
    if len(policy['renames'])!=4 or len(set(policy['renames'].values()))!=4:
        raise ValueError('Review any change beyond the four adopted MCP naming changes')
    source=args.source_package.absolute();base.regular(source)
    original_project=base.project
    data={}
    def project(path):
        robot,reduction,source_controls=original_project(path)
        rows=annotate(source_controls,base.read(path/'config/joint_name_registry.json'),policy)
        source_to_public={r['source_joint_name']:r['name'] for r in rows}
        public_to_source={v:k for k,v in source_to_public.items()}
        if len(public_to_source)!=20:raise ValueError('Public joint names are not unique')
        if len([a for a,b in source_to_public.items() if a!=b]) not in [0,4]:raise ValueError('Unexpected partial naming migration')
        for j in robot.findall('joint'):j.set('name',source_to_public[j.get('name')])
        controls={'schema_version':2,'model_id':source_controls['model_id'],
                  'interface_id':policy['public_interface_id'],'representation_id':args.revision,
                  'source_interface_id':source_controls['interface_id'],
                  'control_count':20,'state_count':20,'coordinate_space':'output_joint_angles',
                  'controls':rows,'state_joint_names':[r['name'] for r in rows],'state_only_names':[],
                  'hardware_commands_enabled':False,'range_revision_id':source_controls.get('range_revision_id'),
                  'coordinate_rule':'Source numeric coordinates retained exactly; this name change does not redefine sign or zero.'}
        contract={'schema_version':1,'policy_id':policy['policy_id'],'public_interface_id':policy['public_interface_id'],
                  'source_interface_id':source_controls['interface_id'],'source_model_id':source_controls['model_id'],
                  'source_to_public':source_to_public,'public_to_source':public_to_source,
                  'legacy_to_public':policy['renames'],'legacy_interface_ids':policy.get('legacy_interface_ids',[]),
                  'renamed_count':sum(a!=b for a,b in source_to_public.items()),
                  'q_public_equals_q_source':True,'angle_units_unchanged':True}
        reduction.update(public_naming_policy_id=policy['policy_id'],public_interface_id=policy['public_interface_id'],
                         source_to_public_joint_names=source_to_public)
        data.update(rows=rows,contract=contract,source_controls=source_controls)
        return robot,reduction,controls
    base.project=project
    saved=sys.argv
    try:
        sys.argv=[str(BASE),'--source-package',str(source),'--output-dir',str(args.output_dir),'--revision',args.revision]
        base.main()
    finally:sys.argv=saved
    package=args.output_dir.absolute()/base.PACKAGE
    base.dump(package/'config/naming_policy.json',policy)
    base.dump(package/'config/joint_name_map.json',data['contract'])
    for rel in ['config/mesh_name_map.json','config/structural_mesh_naming_policy.json','config/range_update_r08.json','docs/网格结构名称对应表.csv']:
        if (source/rel).exists():shutil.copy2(source/rel,package/rel)
    # Runtime references use public names; original full profiles stay only
    # under provenance, explicitly labelled as source evidence.
    loops=base.read(source/'config/nonlinear_loops.json')
    for row in loops['loops']:
        row['output_joint']=data['contract']['source_to_public'][row['output_joint']]
    base.dump(package/'config/motor_coordinate_reference.json',{
        'interface_id':policy['public_interface_id'],'source_model_id':data['source_controls']['model_id'],
        'not_encoder_or_hardware_calibration':True,'numeric_transmissions_unchanged':True,
        'output_to_source_channels':[{'joint_name':r['name'],'source_joint_name':r['source_joint_name'],
                                    'source_control':r['source_control'],'physical_role':r['physical_role']} for r in data['rows']],
        'linear_actuator_loops':loops,
        'rotary_reference_units':'output joint radians; motor encoder scale/offset not provided'})
    examples=package/'examples';base.regular(examples);examples.mkdir()
    base.dump(examples/'zero_angles_rad.json',{r['name']:0. for r in data['rows']})
    base.dump(examples/'zero_joint_states.json',{'name':[r['name'] for r in data['rows']],
              'position':[0.]*20,'velocity':[],'effort':[]})
    base.dump(examples/'flexion_45deg.json',{'interface_id':policy['public_interface_id'],
              'angles_rad':{r['name']:math.pi/4 if r['motion_kind']=='flexion_extension' else 0. for r in data['rows']}})
    with (package/'docs/20关节名称与范围.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.writer(f);writer.writerow(['公开关节名称','中文描述','源模型关节名称','下限_rad','上限_rad','下限_deg','上限_deg','正方向约定'])
        for r in data['rows']:writer.writerow([r['name'],r['display_name'],r['source_joint_name'],r['lower_rad'],r['upper_rad'],r['lower_deg'],r['upper_deg'],r['positive_definition']])
    shutil.copy2(CONVERTER,package/'tools/convert_joint_names.py')
    shutil.copy2(BASE,package/'tools/build_reduced_base.py')
    shutil.copy2(Path(__file__),package/'tools/build_reduced.py')
    # The inherited model's same shape/physics is released as a new interface revision.
    import xml.etree.ElementTree as ET
    xml=ET.parse(package/'package.xml');xml.getroot().find('version').text='0.3.0'
    xml.write(package/'package.xml',encoding='utf-8',xml_declaration=True)
    cmake=package/'CMakeLists.txt'
    cmake.write_text(cmake.read_text(encoding='utf-8').replace('config docs launch tools provenance','config docs launch tools provenance examples'),encoding='utf-8')
    manifest=base.read(package/'config/model_manifest.json')
    manifest.update(public_interface_id=policy['public_interface_id'],naming_policy_id=policy['policy_id'],
                    naming_map='config/joint_name_map.json',state_count=20,
                    public_names_changed=data['contract']['renamed_count'])
    base.dump(package/'config/model_manifest.json',manifest)
    readme=package/'README.md'
    text=readme.read_text(encoding='utf-8').replace('# 抬高版：20关节简化模型','# 抬高版：统一关节及结构网格命名的20关节模型')
    text=text.replace('20个关节的名称、零位、正方向和上下限沿用来源。','20个关节统一采用当前公开命名；四指侧摆为 mcp_abduction，屈伸为 flexion。零位、正方向和上下限沿用来源。')
    thumb_bounds={r['name']:r['upper_deg'] for r in data['rows'] if r['name'] in ['thumb_mcp_flexion','thumb_ip_flexion']}
    if thumb_bounds and all(abs(v-90)<1e-10 for v in thumb_bounds.values()):
        text+='\n\n本批更新：拇指MCP和IP弯曲上限均为精确90°，下限和零位不变。网格文件名与完整机构版一致，按结构命名；旧G标识只用于来源追溯。\n'
    text+='''

## 对外关节名称和旧版兼容

四指侧摆依次使用 index_mcp_abduction、middle_mcp_abduction、ring_mcp_abduction、little_mcp_abduction。
abduction表示外展/内收自由度，正方向仍按来源轴的右手定则，不能仅凭名字推定为“正值=张开”。
14个屈伸轴继续使用 flexion；拇指末端使用 IP，两个根部机械轴保留 thumb_base_rotation_1/2。
关节名不加入未经确认的手侧或品牌。完整名称与范围见 docs/20关节名称与范围.csv。

config/independent_controls.json 只描述当前20输入/20状态；没有完整机构的28个附加状态。
config/joint_name_map.json 提供完整20名称双向映射，所有坐标转换系数为1、零偏为0。
config/motor_coordinate_reference.json 使用新名称对应原有电机通道；原有非线性常数原样保留。
provenance 中的旧名称是来源记录，不是本包控制接口，不要把完整机构48状态记录直接输入本简化模型。

旧版输出姿态可以使用标准库工具转换：

```text
python tools/convert_joint_names.py old_pose.json --direction to-public --output public_pose.json
python tools/convert_joint_names.py public_pose.json --direction to-source --output source_pose.json
```

支持纯角度字典、angles_rad/angles_deg对象以及JointState JSON。只改名称，不改变数值、单位或数组顺序；旧名和新名同时指向同一个关节时会拒绝。
零姿态和屈曲45度的显示示例放在 examples/；示例不表示该组合姿态已验证无碰撞。

本版可用于网页查看器加载和对外运动模型交换，尚未做网格减面，也不代表已经完成产品真机控制适配或动力学标定。
'''
    readme.write_text(text,encoding='utf-8')
    print(json.dumps({'public_package':str(package),'renamed_joints':data['contract']['renamed_count'],
                     'policy_id':policy['policy_id'],'control_and_state_count':20,'zero_axes_limits_inertia_unchanged':True},ensure_ascii=False))


if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');main()
