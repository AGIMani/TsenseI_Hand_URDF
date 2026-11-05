# A3.2 钢琴专用指尖模型

此目录保留上传的左手模型，并提供由它生成的几何镜像右手。两者使用相同的 21 个 link 和 20 个旋转输出关节，各有 297 个 visual、297 个 collision 和 21 个惯量定义。截图均为 URDF 全零关节姿态。

| 手侧 | URDF | 资源 |
| --- | --- | --- |
| 左手，上传的原始模型 | [`urdf/hand.urdf`](urdf/hand.urdf) | `meshes/visual/`、`meshes/collision/` |
| 右手，几何镜像 | [`urdf/hand_right.urdf`](urdf/hand_right.urdf) | `meshes_right/visual/`、`meshes_right/collision/` |

## 镜像约定

以 `hand_base_link` 的 **Y = 0 m** 平面镜像，即 `x_right = x_left`、`y_right = -y_left`、`z_right = z_left`。这是本目录的镜像约定，与仓库中其他版本的镜像平面不一定相同；装配时应按照实际安装面与孔位检查基座坐标。

右手各关节保留与左手相同的名称、角度限位和数值约定：`q_right = q_left` 时得到镜像姿态。每个旋转轴按轴矢量规则转换；外观与碰撞 STL 的顶点、三角面绕序和法线均已镜像，质心和完整惯量张量也已转换。两个模型中的 link 和 joint 同名，如需同时放入同一机器人描述，必须先分别加上名称前缀并同步更新引用。

右手是从左手推导的几何模型，并非独立实测的右手 CAD 或硬件标定。源模型质量合计约 1.05338 kg；其配置将质量列为工程估算，不能当作完整实物称重或右手实测数据。关节范围、力矩、速度、碰撞与硬件正方向仍需在目标实物和应用中验证。

## 文件说明

- `config/` 中原有文件是上传模型的来源和左手配置，**不应当作已转换的右手质量、零位或控制器标定**。`config/mirror_right.json` 单独记录镜像关系和输入、输出网格的 SHA-256。
- `tools/mirror_right.py` 可从原始左手 URDF 和网格重新生成右手 URDF、右手网格和镜像清单（需要 NumPy）：

  ```bash
  python3 tools/mirror_right.py
  ```

- 原始文件夹中的 `launch/display.launch.py` 引用了未随文件夹提供的 ROS 包、URDF 和发布脚本；此文件不能直接作为已验证的 ROS 启动入口。可使用支持相对 STL 路径的 URDF 查看器打开上表中的两个 URDF。

仓库根目录的 `tests/test_pianist_mirror.py` 检查左右运动树、限位、不同关节姿态的坐标与惯量，以及所有网格的镜像顶点和面绕序。测试需要 NumPy 和 SciPy：

```bash
python3 -m unittest discover -s tests -p 'test_pianist_mirror.py' -v
```
