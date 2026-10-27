# TsenseI Hand URDF 多版本模型说明

本仓库保存 TsenseI 灵巧手 A2.0、A3.0、A3.2 及多种结构调整版，包含 URDF、外观网格，以及部分版本的 MuJoCo、关键点、碰撞和 ROS 配置。本文按当前实际文件整理，重点说明版本选择和差异。

> 不同目录不是同一模型的简单重命名。切换版本时，请一并使用该目录内的网格、关节配置和关键点标注。`withid` / `no_id` 沿用原目录标记；这里不将 ID 推定为控制器 ID 或关节编号。

## 1. 版本对照

下表数量由各目录 `urdf/` 中的实际 XML 统计；左右手一致的版本只列单手数量。`visual / collision / inertial` 分别是外观元素、碰撞元素和含惯量定义的 link 数量，并不表示独立自由度或有效碰撞检测对数。

| 目录 | 主要区别 | 手侧 | link / 可动 / 固定关节 | visual / collision / inertial |
| --- | --- | --- | --- | --- |
| `TsenseI_hand_A2_0` | A2.0 旧版基线，21 个旋转关节 | 左、右 | 23 / 21 / 1 | 23 / 23 / 23 |
| `TsenseI_hand_A3.0` | A3.0 基线，改为 20 个旋转关节；带左右 MJCF 和关键点配置 | 左、右 | 22 / 20 / 1 | 22 / 22 / 22 |
| `TsenseI_hand_A3.0_withid` | A3.0 带 ID 外壳版本，独立外壳几何与关键点标注 | 左、右 | 22 / 20 / 1 | 22 / 21 / 21 |
| `TsenseI_hand_A3.2_no_id` | A3.2 无 ID 版本，独立的关节范围、映射与镜像配置 | 左、右 | 22 / 20 / 1 | 22 / 22 / 22 |
| `TsenseI_hand_A3.2_withid_upper_mid_thumb` | 拇指抬高 5 mm、中指加长 6 mm；保留模型派生与单位修正记录 | 左、右 | 22 / 20 / 1 | 22 / 22 / 22 |
| `TsenseI_hand_A3.2_withid_no_upper5mm_no_midupper6mm_withcollision` | 拇指不抬高、中指不加长；当前内容已导入中指 DIP 统一的 20 输出模型，并配有 MJCF 碰撞代理 | 左、右 | 22 / 20 / 1 | 22 / 22 / 0 |
| `TsenseI_hand_A3.2_withid_upper_thumb5mm_noupper_mid` | 拇指抬高 5 mm、中指不加长的结构版；逐零件外观、碰撞、质量惯量和镜像资料 | 左、右 | 22 / 20 / 1 | 367 / 367 / 22 |
| `TsenseI_中指DIP统一_20输出关节 (2)` | 中指 DIP 统一的姿态模型；307 个来源 visual 合并为 22 个 STL | 仅左手 | 22 / 20 / 1 | 22 / 0 / 0 |
| `TsenseI_hand_A3.2_withid_end` | 抬高模型的 20 关节归并版，统一对外名称；固定安装件并入刚体 | 单份模型，名称不带手侧 | 21 / 20 / 0 | 303 / 303 / 21 |

`end` 是目录名，不代表它可以直接替代全部其他版本；其关节命名、刚体数量和接口均有变化。根据设计说明，文件名中的拇指 `upper` 表示拇指抬高 5 mm，中指 `upper_mid` / `midupper6mm` 表示中指加长 6 mm；`noupper_mid` / `no_midupper6mm` 表示中指不加长。中指的变化是长度增加，不是整体抬高。上述尺寸含义由设计者确认，本文未另行测量 STL。

## 2. 各版本具体改动

### A2.0 → A3.0

- 每手可动关节由 21 个变为 20 个，link 由 23 个变为 22 个。
- A2.0 拇指使用 `thumb_rota_joint`、`thumb_swing_joint` 等名称；A3.0 使用 `thumb_rota_block_joint`、`thumb_rotaback_joint2` 等名称，旧关节列表不能直接复用。
- A3.0 的 `hand_profile.yaml` 记录小指按功能整理为 MCP 屈伸、swing 侧摆、PIP、DIP；记录的转换方案为 MCP/PIP 连杆转换与 DIP 直接旋转，硬件接口处于关闭状态。

### A3.0 → A3.0_withid

- 配置记录表明，两者标准关节坐标系、轴和范围一致，但带 ID 版使用独立外壳几何与 21 点标注。
- 带 ID 版不能直接继承裸版的人工关键点标定。
- 带 ID 版配置的碰撞范围为不同手指之间，使用 15 个手指胶囊，不包含手掌、腕部和安装壳体。
- 右手为 XZ 平面镜像模型，不是独立实测右手 CAD。当前 URDF 的 collision/inertial 数量也与无后缀版不同，见对照表。

### A3.2_no_id

- 仍是 20 个输出关节，但采用 A3.2 的关节范围与 SDK 映射资料。
- `hand_profile.yaml` 记录 MCP/PIP/DIP 范围为 `0.005..1.565 rad`；侧摆模型范围为 `-0.35..0.35 rad`，指令范围为 `-0.34..0.34 rad`。
- 此版配置 `mirror_axis: x`；A3.0 系列配置为 `y`，不能直接套用相同镜像规则。
- 小指关节名按功能整理，link/mesh 名仍保留原链名称，不能只看网格名称判断关节功能。

### A3.2_withid_upper_mid_thumb

- 结构方案：拇指抬高 5 mm，同时中指加长 6 mm。
- `model_derivation.json` 记录来源名称 `A3.1V6 26.9.27(6)`，以及小指 `SWING` 到 `swing` 的名称整理。
- `base_link.STL` 来源单位为毫米，记录了 `0.001` 缩放修正；不能对整个模型再次统一乘以此系数。
- 记录了根坐标平移、标准掌部位置和右手临时镜像定义。
- 与后续中指 DIP 统一、拇指 5 mm 版相比，关节的 `origin xyz/rpy` 实际存在差别，不只是网格文件名不同。

### A3.2_withid_no_upper5mm_no_midupper6mm_withcollision

- 结构方案：拇指不抬高，中指不加长，即不采用上述 5 mm 抬高和 6 mm 加长。
- 当前 `hand_profile.yaml` 明确记录：已导入中指 DIP 统一的 20 输出 CAD，保留其关节坐标系、限位和米制网格。
- 当前左右 URDF 各有 22 个 collision，但没有 inertial。目录中存在 `inertial_parameters.json` 不代表 URDF 已具备动力学惯量。
- 配套 MJCF 碰撞方案与 URDF 网格碰撞需分别理解：`qa/README.md` 描述每手 10 个近端/中间指节胶囊，加 5 个完整末端外壳凸包，只检查不同手指之间的碰撞。
- 末端凸包由整个末端 link 的视觉网格构建，替代仅拟合指尖最后 1 mm 截面的旧胶囊，以覆盖完整指腹。
- 右手由 XZ 镜像生成；关键点从旧零位投影，配置仍标注需要人工重标定。`qa/README.md` 中的历史测试与外部工具路径是来源记录，并非本仓库本次运行结果。

### A3.2_withid_upper_thumb5mm_noupper_mid

- 结构方案：仅拇指抬高 5 mm，中指不加长。
- 当前左右模型均保留 367 个结构外观元素和 367 个碰撞元素，相比合并为 22 个外观的版本，结构零件划分更细。
- 20 个可动输出加 1 个固定腕座连接；内部动子和连杆冻结为零位外观，普通 URDF 不模拟完整非线性机构运动。
- 原说明记录单手总质量 952 g，并提供质量、质心、惯量及重算工具。质心与惯量含几何等效密度估算，不能当作全部实测值。
- 右手镜像平面为 `base_link` 下的 `Y = 4 mm`，即 `x_R=x_L, y_R=8 mm-y_L, z_R=z_L`；对应关节采用 `q_R=q_L`。
- 附有行程转换、镜像验证及碰撞审计。原审计明确存在零位和部分姿态的碰撞，不能把“有碰撞网格”等同于“全范围无碰撞”。
- **当前文件与旧说明有出入：**旧 `README.txt` 侧重右手并提到 ROS 专用 URDF，但当前 `urdf/` 仅有左右两份 `*_v3_2_withid.urdf`；所述 `*_ros.urdf` 和 mock xacro 当前缺失。现存 launch/config 的存在不代表 ROS 显示、模拟控制和规划入口已经完整可运行。

### 中指 DIP 统一的 20 输出版

- 仅提供左手模型，保留传统 `left_hand_*` 命名，`mid` 为中指、`pinky` 为小指。
- 14 个弯曲输出范围为 `0..π/2 rad`，4 个侧摆为 `-0.35..0.35 rad`，拇指两旋转为 `0..2.09 rad` 和 `0..1.74 rad`。
- 307 个来源 visual 合并为 22 个 STL；内部动子和连杆并入所属壳体，不作为独立控制关节。
- 当前 URDF 无 collision、无 inertial，适合姿态展示和运动学模型交换。它与带碰撞代理版、367 零件版不是同一份完整配置。

### A3.2_withid_end

- 采用 21 个刚体、20 个旋转关节，没有额外固定关节，也没有 mimic。
- 来源的 14 套动子/连杆共 28 个随动坐标冻结在零位，并合并 5 个固定安装链接；保留 303 个外观和碰撞元素。
- 公开关节改名为 `index_mcp_flexion`、`middle_mcp_abduction` 等；中指改用 `middle`、小指改用 `little`、拇指末端使用 `IP`，不带左右手前缀。
- `config/joint_name_map.json` 提供名称映射，`tools/convert_joint_names.py` 可转换旧姿态名称；映射系数为 1、零偏为 0，不转换角度单位。
- 原说明记录总质量约 1052.982463 g，与 952 g 版来源不同。拇指 MCP/IP 弯曲上限更新为精确 90°。
- 普通版与 ROS 版 URDF 表达相同模型，仅资源路径不同。内部机构冻结后的非零姿态动力学仍属近似。

## 3. 实际加载入口

以下均相对于对应版本目录；加载时保留该目录内的完整网格路径。

| 版本 | `urdf/` 下的入口 |
| --- | --- |
| A2.0 | `TsenseI__hand_left_v1.urdf` / `TsenseI__hand_right_v1.urdf`（`TsenseI` 后为双下划线） |
| A3.0 | `TsenseI_hand_left_v3_0.urdf` / `TsenseI_hand_right_v3_0.urdf` |
| A3.0_withid | `TsenseI_hand_left_v3_0_withid.urdf` / `TsenseI_hand_right_v3_0_withid.urdf` |
| A3.2_no_id | `TsenseI_hand_left_v3_2.urdf` / `TsenseI_hand_right_v3_2.urdf` |
| upper_mid_thumb | `TsenseI_left.urdf` / `TsenseI_right.urdf` |
| no_upper5mm_no_midupper6mm_withcollision | `TsenseI_hand_left_v3_2_withid.urdf` / `TsenseI_hand_right_v3_2_withid.urdf` |
| upper_thumb5mm_noupper_mid | `TsenseI_hand_left_v3_2_withid.urdf` / `TsenseI_hand_right_v3_2_withid.urdf` |
| 中指 DIP 统一版 | `TsenseI_hand_left_v3_2_withid.urdf` |
| withid_end | `hand_1009_20j.urdf`；ROS 资源版为 `hand_1009_20j_ros.urdf` |

多个目录的 URDF **同名但内容不同**，建议始终用“版本目录 + 文件名”定位。仓库根目录的 `TsenseI_hand_A3.2_withid_upper_thumb5mm_noupper_mid.zip` 是单独压缩包；本文按解压目录编写，未确认压缩包与目录逐文件一致。

## 4. 配套文件用途

| 文件或目录 | 用途 |
| --- | --- |
| `urdf/`、`meshes/`、`meshes_right/` | 运动树与几何资源；网格路径以各 URDF 的引用为准 |
| `hand_profile.yaml` | 左右模型入口、重定向设置、镜像轴和硬件接口状态 |
| `joint_groups*.yaml`、`q_init*.yaml` | 关节分组与初始角度 |
| `robot_keypoints_21*.yaml` | 21 点映射/标注；文件存在不代表完成实物标定 |
| `mjcf/`、`collision_geometry.yaml` | MuJoCo 模型及其碰撞代理 |
| `source_mapping.yaml`、`model_derivation.json` | 来源、坐标和生成变换记录 |
| `config/`、`docs/` | 版本专属接口、参数、映射和说明 |
| `checks/`、`qa/`、`provenance/` | 已有验证结果、预览及来源证据，须结合记录的版本和检查范围使用 |
| `*.bak_*` | 历史备份，不作为默认加载入口 |

## 5. 选择与切换

- 复现旧模型：选择对应 A2.0/A3.0 目录，不混用另一版关键点或关节顺序。
- 使用中指 DIP 统一的轻量姿态模型：选择 `TsenseI_中指DIP统一_20输出关节 (2)`。
- 使用已有 MJCF 手指间碰撞代理：查看 `no_upper5mm_no_midupper6mm_withcollision` 及其 `hand_profile.yaml`、`qa/README.md`。
- 查看拇指抬高 5 mm、中指不加长的细分结构、左右镜像和行程资料：选择 `upper_thumb5mm_noupper_mid`。
- 使用统一公开关节名的模型交换或 ROS 展示包：查看 `withid_end` 及其独立 README。

切换时核对关节名称、顺序、限位、零位、镜像方式和资源路径。20 个输出关节不等于完整内部机构；有 inertial 不等于已完成动力学标定；所有控制和碰撞结论均应限定到其记录对应的版本。
