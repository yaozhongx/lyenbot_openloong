# Lyenbot 适配说明

## 1. 当前结论

本项目已经完成 Lyenbot 模型导入、只读 URDF 加载、名称化关节映射、MuJoCo 自动生成、Pinocchio/MuJoCo 一致性检查、保守 PD 站立和双足 WBC 站立。

当前验收状态：

| 项目 | 状态 |
|---|---|
| 保留 AzureLoong 原运行路径 | 已完成，原有目标构建通过 |
| 原始 Lyenbot URDF 不修改 | 已完成，哈希一致 |
| 23 关节名称/限制读取 | 已完成 |
| MuJoCo 模型自动生成 | 已完成 |
| Pinocchio/MuJoCo 静态一致性 | 已完成 |
| PD 稳定站立 10 秒 | 已完成 |
| WBC 双足站立 10 秒 | 已完成 |
| 23 关节动态逐个方向测试 | 未完成 |
| 低速 WBC 连续行走 10 步 | 未完成 |
| MPC 约 0.15 m/s 连续 10 步 | 未完成 |
| 真机接口 | 未开始，缺少硬件资料 |

因此，当前可以表述为：

```text
Lyenbot 的模型与站立控制路径已经跑通；连续行走尚未跑通。
```

## 2. 使用的模型

当前 Lyenbot 主 URDF：

```text
models/lyn03-V07-urdf_C01-260414/lyn03-V07-C01-260326.urdf
```

SHA-256：

```text
86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22
```

该文件是机械参数唯一真源，禁止修改。

上游 OpenLoong 参考工程的本机路径约定为：

```text
~/Openloong/openloong-dyn-control
```

该路径仅用于对照原始实现，不是当前 Lyenbot 运行时模型路径。

当前不使用以下备选文件作为主控制模型：

- `*_capsule.urdf`；
- `*_sphere.urdf`；
- `*_show.urdf`；
- `urdf/` 子目录中的其他版本。

这些文件来自原始模型包，但没有按当前配置完成全部一致性和站立验证。

MuJoCo 实际加载：

```text
models/lyn03-V07-urdf_C01-260414/generated/scene_lyenbot.xml
```

该 scene 包含自动生成的 `Lyenbot.xml`。它是派生物，不是机械参数真源。

## 3. 不修改 URDF 的原则

### 3.1 只能由 URDF 提供

- link 质量、质心、惯量；
- joint 拓扑、origin、axis；
- position、velocity、effort 限制；
- 碰撞/可视几何；
- 活动关节和固定 Frame；
- Pinocchio `idx_q/idx_v`。

### 3.2 由语义配置提供

- 哪些 joint 是左腿、右腿、左臂、右臂、腰部和头部；
- 哪个 Frame 是左右足底；
- 哪个 body 是基座；
- MuJoCo sensor 名称；
- URDF、scene、PVT 配置路径。

### 3.3 由仿真/控制配置提供

- `kp/kd` 和滤波频率；
- MuJoCo damping、frictionloss、armature、floor friction；
- 步态周期、步高、目标速度；
- touchdown 阈值；
- 初始关节姿态；
- WBC/MPC 权重。

如果发现 URDF 数据异常，应让检查器报错并记录；不能通过修改 URDF 的轴、限位、惯量或 effort 让控制器“看起来能跑”。

## 4. AzureLoong 与 Lyenbot 拓扑差异

| 关节组 | AzureLoong | Lyenbot |
|---|---:|---:|
| 左臂 | 7 | 5 |
| 右臂 | 7 | 5 |
| 头部 | 2 | 0 |
| 腰部 | 3 | 1 |
| 左腿 | 6 | 6 |
| 右腿 | 6 | 6 |
| 合计 | 31 | 23 |

Lyenbot 每条腿按原始 URDF 语义处理：

```text
hip_pitch
→ hip_roll
→ hip_yaw
→ knee_pitch
→ ankle_pitch
→ ankle_roll
```

不得为了匹配 AzureLoong 旧顺序调整 URDF。

Lyenbot 没有头部任务；每条手臂只有 5 DoF，因此当前采用 10 个手臂关节的命名姿态跟踪，不强行执行双手完整 6D IK。腰部仅包含 `waist_yaw_joint`。

## 5. Lyenbot 关节语义

当前 23 个活动关节按配置顺序为：

```text
左腿：
left_hip_pitch_joint
left_hip_roll_joint
left_hip_yaw_joint
left_knee_pitch_joint
left_ankle_pitch_joint
left_ankle_roll_joint

右腿：
right_hip_pitch_joint
right_hip_roll_joint
right_hip_yaw_joint
right_knee_pitch_joint
right_ankle_pitch_joint
right_ankle_roll_joint

腰部：
waist_yaw_joint

左臂：
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_elbow_yaw_joint
left_elbow_pitch_joint
left_elbow_roll_joint

右臂：
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_elbow_yaw_joint
right_elbow_pitch_joint
right_elbow_roll_joint
```

实际名称以 `common/robot_configs/lyenbot.json` 为准。修改配置时必须与主 URDF 中的 joint name 逐字一致。

重要 Frame：

```text
left_foot_contact_point
right_foot_contact_point
```

足端运动学、Jacobian、初始高度和 MuJoCo touch site 都以这两个 URDF Frame 为基础。

## 6. 配置文件与接口

### 6.1 `RobotModelConfig`

相关文件：

```text
common/robot_model_config.h
common/robot_model_config.cpp
common/robot_configs/lyenbot.json
common/robot_configs/azureloong.json
```

主要接口：

```cpp
RobotModelConfig loadRobotModelConfig(const std::string &configPath);
RobotModelConfig azureLoongDefaultConfig();
```

`JointLayout` 保存活动关节和各语义关节组的名称集合。

配置路径按 JSON 文件所在目录解析，减少程序启动目录变化造成的路径错误。

### 6.2 PVT 配置

文件：

```text
common/lyenbot_joint_ctrl_config.json
```

只保存：

- `kp`；
- `kd`；
- `PVT_LPF_Fc`；
- `gear`。

位置、速度和最大力矩不从该 JSON 读取，而由 Pinocchio 从 URDF 提供。

## 7. Pinocchio 适配

相关文件：

```text
common/urdf_model_loader.h/.cpp
algorithm/pino_kin_dyn.h/.cpp
```

### 7.1 只读加载

原始 URDF 中有 4 个数值属性包含尾部空白。加载器只在内存 XML 字符串中 trim，再调用 Pinocchio，不写回源文件。

建立两个模型：

- 浮动基座模型：完整站立、行走和动力学；
- 固定基座模型：局部 IK 和身体坐标系计算。

### 7.2 广义坐标维数

Lyenbot 有 23 个活动关节：

```text
nq = 7 + 23 = 30
nv = 6 + 23 = 29
```

Pinocchio 使用 `JointModelFreeFlyer()` 增加根部自由度。

### 7.3 显式索引映射

对每个配置 joint：

```text
joint name
  → joint id
  → idx_q / idx_v
  → motorQIndex / motorVIndex
```

电机数组与 `q/dq` 之间通过这些映射转换，不再假定 23 个电机一定连续位于某个硬编码区间。

### 7.4 Frame 与限制

- 左右足、髋、手和基座按名称查询 Frame；
- position、velocity、effort 从 Pinocchio 模型数组读取；
- workspace/限位检查按 `motorQIndex/motorVIndex` 对应到正确关节。

## 8. MuJoCo 生成与适配

### 8.1 生成命令

从仓库根目录执行：

```bash
./build/generate_mjcf common/robot_configs/lyenbot.json
```

生成文件：

```text
models/lyn03-V07-urdf_C01-260414/generated/Lyenbot.base.xml
models/lyn03-V07-urdf_C01-260414/generated/Lyenbot.xml
models/lyn03-V07-urdf_C01-260414/generated/scene_lyenbot.xml
models/lyn03-V07-urdf_C01-260414/generated/normalized_source.urdf
models/lyn03-V07-urdf_C01-260414/generated/generation_manifest.txt
```

### 8.2 浮动基座

生成的 `base_link` 下包含：

```xml
<freejoint name="float_base"/>
```

初始 keyframe 基座位姿：

```text
位置：[0, 0, 0.850693] m
姿态：[1, 0, 0, 0]（MuJoCo wxyz）
```

高度由 Lyenbot 初始关节姿态和足底 Frame 正运动学计算，使足底位于地面 `z=0`。

### 8.3 actuator

23 个活动关节各生成一个 torque motor：

```text
gear = 1
ctrlrange = [-effort, +effort]
```

effort 来自 URDF。`gear=1` 表示控制器输出按关节侧扭矩解释，不代表真实电机减速比为 1。

### 8.4 sensor

生成：

```xml
<framequat .../>
<velocimeter .../>
<gyro .../>
<accelerometer .../>
<touch name="lf-touch" .../>
<touch name="rf-touch" .../>
```

`imu` site 位于 `base_link`。左右 touch site 由 URDF 足底 Frame 放置。

当前实际使用：

- quaternion → RPY 和广义姿态；
- gyro/accelerometer → DataBus；
- touch → `fL/fR.z`，用于触地判断；
- velocimeter 已存在，但基座线速度仍由位置差分得到。

### 8.5 足底碰撞

C01 输入版本的足底碰撞不足以直接提供稳定支撑面，因此生成器在派生 MJCF 中增加透明薄盒碰撞几何。该尺寸是仿真参数，不修改 URDF link 或惯量。

## 9. MuJoCo 接口映射

相关文件：

```text
sim_interface/MJ_interface.h/.cpp
```

### 9.1 反馈

按 joint name 查询：

```text
jnt_qposadr → joint position
jnt_dofadr  → joint velocity
```

同时读取：

- 基座世界位置；
- 基座姿态；
- IMU 加速度和角速度；
- 左右 touch；
- 基座线速度。

首帧位置差分速度设为零，避免初始化尖峰。

### 9.2 控制

不能使用：

```cpp
mj_data->ctrl[i] = tau[i];
```

正确流程是按 joint id 查 actuator id：

```cpp
mj_data->ctrl[jntId_dctl[i]] = tauIn.at(i);
```

因此 XML actuator 排序变化不会改变受控关节。

## 10. PVT 适配

相关文件：

```text
common/PVT_ctrl.h/.cpp
common/lyenbot_joint_ctrl_config.json
```

PVT 计算基本形式：

```text
tau = kp * (q_des - q_cur) + kd * (dq_des - dq_cur)
```

其中：

- `kp/kd` 来自控制配置；
- position/velocity/effort 限制来自 URDF；
- 输出力矩按 URDF effort 饱和；
- 每个配置名称缺失时应直接报错，不能静默使用零增益。

## 11. WBC 与步态适配

相关文件：

```text
algorithm/wbc_priority.h/.cpp
algorithm/gait_scheduler.h/.cpp
algorithm/foot_placement.h/.cpp
```

### 11.1 WBC 拓扑差异

- 手臂任务维数由配置中的 10 个手臂关节生成；
- 头部列表为空时不创建 Lyenbot 头部任务；
- 腰部排除、加权和姿态任务只作用到 `waist_yaw_joint`；
- 足底接触使用配置的左右 contact Frame；
- 站立进入 WBC 时锁存左右足底世界位姿作为 anchor；
- Lyenbot 使用较低刚度、较高阻尼的保守站立增益。

### 11.2 步态调度

Lyenbot demo 启用：

- `useMeasuredContact`；
- touchdown force 阈值；
- 最小落地相位；
- “摆脚必须先离地，再允许落地事件”；
- 双支撑 transfer。

这些开关默认不改变 AzureLoong 旧路径。

### 11.3 FootPlacement

新增步长 clamp。默认值为无穷大以保留 AzureLoong 行为，Lyenbot demo 显式设置有限值，避免瞬时速度尖峰生成不可达落脚点。

当前仍需检查并移除剩余 AzureLoong 专用偏置和腿部尺寸常量。

## 12. 分阶段 demo

相关文件：

```text
demo/lyenbot_staged.cpp
```

构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target generate_mjcf model_check lyenbot_staged
```

模型生成和检查：

```bash
./build/generate_mjcf common/robot_configs/lyenbot.json
./build/model_check common/robot_configs/lyenbot.json
```

无 GUI：

```bash
(cd build && ./lyenbot_staged pd)
(cd build && ./lyenbot_staged wbc)
(cd build && ./lyenbot_staged walk)
(cd build && ./lyenbot_staged walk mpc)
```

GUI：

```bash
(cd build && ./lyenbot_staged pd --gui)
(cd build && ./lyenbot_staged wbc --gui)
(cd build && ./lyenbot_staged walk --gui)
(cd build && ./lyenbot_staged walk mpc --gui)
```

安全停止条件包括 NaN、基座高度过低和 roll/pitch 过大。不得为了让测试继续而删除安全检查。

## 13. 模型一致性检查

`model_check` 检查：

- `nv == 6 + 活动关节数`；
- `nq == nv + 1`；
- 配置 joint/Frame 是否存在；
- 每个 joint 是否为 1 DoF；
- position/velocity/effort 是否有效；
- MuJoCo joint/actuator 是否逐名对应；
- MuJoCo range/ctrlrange 是否与 URDF 一致；
- Pinocchio 足底 Frame 与 MuJoCo touch site 是否一致。

已通过结果：

```text
nq=30
nv=29
nu=23
左右足底位置误差约 4.65e-7 m
MODEL_CHECK_OK
```

静态映射通过不等于动态方向测试通过；后者仍在 TODO 中。

## 14. 已验证运行结果

### 14.1 PD

```text
LYENBOT_STAGE_OK
time=10.001
base_z=0.848443
```

结论：保守 PD 可以保持初始站姿 10 秒。

### 14.2 WBC

```text
第 3 秒 WBC 接管
接管后持续约 10 秒
最终 base_z=0.848591
```

结论：双足 WBC 站立阶段通过。

### 14.3 行走

当前在首步单支撑阶段出现侧向失稳并安全停机，无 NaN。该阶段未通过，详细见 `DEBUG_LOG.md` 的 D008。

### 14.4 MPC

调用路径已经预留，但 WBC 行走尚未通过，因此不能进行最终 MPC 验收。

## 15. 当前禁止的处理方式

- 修改原始 URDF joint axis；
- 修改原始 URDF position/velocity/effort 限制；
- 修改原始 URDF 惯量来“调稳定”；
- 虚构真实减速比；
- 放大 effort 以掩盖控制问题；
- 关闭 NaN、姿态或高度安全检查；
- 把短暂未跌倒描述成连续行走成功；
- WBC 单步未通过时直接反复调 MPC；
- 用固定数字下标代替已经建立的名称映射。

## 16. 关键文件索引

| 文件 | 作用 |
|---|---|
| `common/robot_configs/lyenbot.json` | Lyenbot 路径、语义和仿真初值 |
| `common/robot_model_config.*` | 配置加载与语义结构 |
| `common/urdf_model_loader.*` | 只读 URDF 兼容加载 |
| `algorithm/pino_kin_dyn.*` | Pinocchio 映射、运动学和动力学 |
| `demo/generate_mjcf.cpp` | URDF 到 MuJoCo 生成 |
| `demo/model_check.cpp` | 跨引擎一致性检查 |
| `sim_interface/MJ_interface.*` | MuJoCo 反馈/actuator 映射 |
| `common/PVT_ctrl.*` | 关节控制与 URDF 限幅 |
| `algorithm/wbc_priority.*` | WBC 拓扑与接触任务适配 |
| `algorithm/gait_scheduler.*` | 触地和支撑状态切换 |
| `algorithm/foot_placement.*` | 摆动足轨迹和步长限制 |
| `demo/lyenbot_staged.cpp` | 分阶段运行和安全检查 |
| `doc/lyenbot_model_adaptation_zh.md` | 逐文件教学报告 |

## 17. 文档联动规则

以后修改 Lyenbot 相关代码时：

1. 修改前说明问题、原因、方案、文件和影响；
2. 修改后更新 `CHANGELOG.md`；
3. 若涉及 Bug 排查，更新 `DEBUG_LOG.md`；
4. 若改变长期架构，更新 `DESIGN_NOTES.md`；
5. 若改变 Lyenbot 接口、模型或运行方式，更新本文件；
6. 若产生新任务或完成任务，更新 `TODO.md`；
7. 最后重新校验主 URDF SHA-256。

## 18. 历史文档来源与保留策略

本文件从以下历史内容提炼：

- `LYENBOT_PROJECT.md`（内容已完全迁移，原文件已删除）；
- `Tutorial.md`；
- `doc/lyenbot_model_adaptation_zh.md`；
- `doc/conversation_handoffs/lyenbot_adaptation_handoff_zh.md`（内容已完全迁移，原文件已删除）；
- `README-zh.md` 中的 OpenLoong 架构说明。

深入学习每个修改语句的作用时，继续阅读 `doc/lyenbot_model_adaptation_zh.md`。

`Tutorial.md`、README、Doxygen/Sphinx 输入和第三方文档具有独立用途，继续保留。

## 19. 新对话接续说明

新对话不需要完整读取五份工程记录。先阅读最小范围：

```text
1. 本文件的“当前结论”
2. TODO.md 的“当前验收总览”
3. 与当前目标对应的 TODO 条目
```

之后按任务需要读取：

- 排查已知问题：读取 `DEBUG_LOG.md` 对应编号；
- 修改架构或数据流：读取 `DESIGN_NOTES.md` 对应章节；
- 查询历史原因：读取 `CHANGELOG.md` 对应日期或模块；
- 学习逐语句实现：读取 `doc/lyenbot_model_adaptation_zh.md` 对应章节。

完整规则见根目录 `AGENTS.md` 的“工程文档按需读取规范”。

精简接续提示：

```text
请遵守 AGENTS.md。先查看 LYENBOT_ADAPTATION.md 的“当前结论”、TODO.md 的
“当前验收总览”和当前任务条目；其他文档按需读取。继续调试 Lyenbot walk
首步单支撑失稳。原始 URDF 必须只读。不要改动 record/matlabReadDataScript.txt
或 build-debug/，除非我明确要求。
```
