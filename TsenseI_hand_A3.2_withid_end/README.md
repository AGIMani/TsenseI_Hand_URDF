# 抬高版：统一关节及结构网格命名的20关节模型

来源：完整机构包 `hand_1009_description`；模型 ID `683b80cc0f91bb4a-review-raised-shellzero-r05-5c3ee9ae`；简化修订 `raised_20j_unified_mesh_thumb90_r03_20261016`。

本包有21个刚体、20个旋转关节，没有独立动子或连杆关节，也没有 mimic 或自定义随动依赖。
20个关节统一采用当前公开命名；四指侧摆为 mcp_abduction，屈伸为 flexion。零位、正方向和上下限沿用来源。拇指一级0～120°，二级0～100°；两级弯曲上限以源模型当前定义为准。
其余范围以 config/independent_controls.json 中的原始弧度值为准，未四舍五入为新的限位。

## 使用

- 普通URDF查看器：加载 urdf/hand_1009_20j.urdf，同时提供 meshes 文件夹，保持目录结构。
- ROS 2：将本目录放入工作空间 src，运行 `colcon build --packages-select hand_1009_reduced_description`，source 安装环境后运行 `ros2 launch hand_1009_reduced_description display.launch.py`。这会启动标准20关节面板与TF发布，不含真机控制器。无界面可加 `gui:=false`。
- ROS资源入口为 urdf/hand_1009_20j_ros.urdf，使用 package://hand_1009_reduced_description/ 路径。
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


本批更新：拇指MCP和IP弯曲上限均为精确90°，下限和零位不变。网格文件名与完整机构版一致，按结构命名；旧G标识只用于来源追溯。


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
