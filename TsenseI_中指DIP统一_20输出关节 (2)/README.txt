当前中指DIP统一版：20输出关节URDF
打开 urdf/TsenseI_hand_left_v3_2_withid.urdf，并同时提供 meshes 文件夹。所有网格路径均为相对路径，可移动整个文件夹。

22个link、21个joint：20个可动输出关节和1个掌板固定关节。没有直线电机动子或连杆的独立关节控制。
joint/link名称逐一对应参考包左手版本：mid为中指、pinky为小拇指、thumb_MCP为F2、thumb_PIP为F1。
14个弯曲输出：0～pi/2 rad；4个侧摆：-0.35～0.35 rad；拇指两旋转：0～2.09与0～1.74 rad。

几何和20个输出的运动学参数来自 hand_middle_dip_standard_description 当前模型，未使用参考包的旧中指DIP外形。
原307个visual全部保留并合并为22个STL；动子和连杆并入所属壳体，作为当前零位的固定外观，不模拟内部非线性联动。
这是手指姿态运动学模型。精确动子/连杆运动须使用原机构模型及非线性状态更新。

base_link为根并包含实际安装件外形。沿用源包已有的base_link到旧CAD安装变换。
base_link原点在旧CAD坐标中为 1.4999987781048649 -89.138007230269267 -30.599999538365971 mm。
Left_hand_palm_link为掌板，通过Left_hand_palm_joint固定在base_link上。
config/tree.json与这份实际URDF结构一致；config/name_mapping.json记录旧名到新名。

examples为20关节的零位和全部弯曲90度姿态。关节角单位rad，STL长度单位m。
未引用参考包的质量、惯性、碰撞模型或真机配置；effort/velocity沿用源模型的显示占位值。
验证：162姿态，3240次输出坐标检查，49734次固定外观变换检查。
checks/export_validation.json记录校验；输入两个文件夹未修改。
