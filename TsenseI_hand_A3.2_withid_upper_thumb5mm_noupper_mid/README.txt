TsenseI 右手 · 抬高5mm · 中指DIP统一 · 20输出关节完整模型 r03（2026-10-13）

这是当前左手r03完整包的几何镜像右手版。没有另行提供、重建或核验独立右手实物CAD。左右各自为独立包，左手文件保持原样。

查看器主文件：urdf/TsenseI_hand_right_v3_2_withid.urdf。保留整个同级meshes目录，所有外观和碰撞网格在meshes/right。
ROS文件：urdf/TsenseI_hand_right_ros.urdf，使用package://tsensei_hand_right_raised5mm_20_outputs_description/路径。两个URDF表达同一模型；只有这两个文件以.urdf结尾，历史/内部审计树以.urdf.txt保存。

模型：22个link、21个joint，即20个旋转输出关节+1个固定腕座连接。输出名全部为right_hand_*；腕座为Right_hand_palm_link，固定关节Right_hand_palm_joint；根仍为base_link。四指各MCP、swing、PIP、DIP；拇指为rota_block、rotaback_joint2、MCP/F2、PIP/F1。完整命名见config/20个控制关节_命名.txt。

几何：367份结构STL，367份碰撞几何；外观、碰撞、关节位置、旋转轴、质心和完整惯量同步转换。镜像后的三角绕序反转，法线由右手实际顶点重新计算；无负mesh scale。拇指两级输出法兰、一级抬高连接件、安装支架、固定在手掌的一级电机安装座全部保留。

镜像平面：base_link坐标的Y=4mm平面，沿手掌宽度方向做左右镜像，保持腕部安装孔布局和Z=-4mm安装面。
x_R=x_L；y_R=8mm-y_L；z_R=z_L。
腕部安装孔集合左右最大拟合差1.9514391e-06mm。该平面取自左手已经核验的安装孔中心，不是把手沿掌面翻转。

控制：同名对应关节q_R=q_L，因此相同有符号指令产生镜像动作。旋转轴按轴矢量转换，axis_R=-S*axis_L；角度界限原值保留，包括拇指二级-1.74..0rad。14路直线电机的有符号动子位移、0..90°所需行程与左手一致。普通20关节URDF依然将内部动子和连杆冻结在零位；原非线性机构参数与独立正反算工具另附。

质量惯量：总952g；掌部base_link150g不含腕座，Right_hand_palm_link17g单独采用；四指各155g，拇指165g。采用原质量表的对应重量，镜像不改变质量、体积和主惯量。质心镜像，完整惯量I_R=S*I_L*S^T，其中ixy、iyz变号，ixz不变。原质量表中left名称作为来源证据保留，活动模型使用right名称。质量沿用原表，COM/惯量沿用几何等效密度估算，均非独立右手实测。

限制与动态：effort=1Nm、velocity=1rad/s仍是用户确认的展示/模拟占位值，真实角度范围、力矩/推力、速度、有效动子行程、通道和编码器零位仍在config/hardware_parameters_pending.json待实测。未加入关节摩擦和阻尼。

配套：ROS2显示启动和RViz、20输出模拟控制接口、MoveIt/OMPL几何规划分组、零位、坐标和角度—行程表、质量重算工具都已改为右手名称及参数。见docs/使用说明.txt。ROS2的模拟控制入口不连接硬件；规划入口不执行轨迹。

验证：367份网格全部顶点镜像对照，1838773个非退化三角法线/绕序对照；1,011组姿态、20,220次输出坐标对照和53,583次内部机构坐标对照，拇指误差不超过6.66e-16（矩阵分量）。全部22个质量、质心和惯量验证通过，质量重算工具在独立副本验证通过。14路行程12,614个点正反解及左右行程一致性通过。在ROS2 Jazzy环境实际构建、加载并发布20个right_hand关节状态、20个运动TF及1个固定TF；安装的ROS URDF SHA与本包一致。GUI/RViz、mock控制与MoveIt运行未验收，所需缺失依赖和检查范围列在checks/ROS2_runtime_validation.json。

碰撞：镜像保留左手的碰撞问题和名义范围限制。左手1,011组姿态的审计结果经上述完整镜像等价核验转换到右手；右手两种运动模型另对各6个代表姿态做独立FCL检查，全部相交对与左手一致。并未重新对右手运行全部1,011组FCL。右手零位同样有掌盖与拇指外壳相交，运动范围内存在拇指与其他手指相交，未验收为全程无碰撞。完整复现见docs/碰撞检查结论.txt及checks/collision_*_mirror_transfer.json。

预览：preview/left_right_comparison.png，左侧为原左手，右侧为镜像右手；含零位和拇指组合姿态。
所有交付路径和各级父目录均为E盘真实普通目录，无目录联接或符号链接。
