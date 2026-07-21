# Lyenbot 适配说明

## 1. 当前结论

### 当前模型状态

当前 Lyenbot 主模型已经接入 OpenLoong-Dyn-Control，模型、站立控制路径、左右准静态原地单步和低速纯 WBC 连续行走已经跑通。上述行走结果来自 `lyenbot_staged` 适配验收脚手架；当前主线改为适配原项目 `walk_wbc` demo，MPC 暂缓。

### 已完成适配

本项目已经完成 Lyenbot 模型导入、只读 URDF 加载、名称化关节映射、MuJoCo 自动生成、Pinocchio/MuJoCo 一致性检查、23 关节动态方向测试、保守 PD 站立、双足 WBC 站立、左右准静态原地单步和低速纯 WBC 连续 10 步验收。

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
| 23 关节动态逐个方向测试 | 已完成，23/23 通过 |
| 左右准静态原地单步 | 已完成，两侧均完成抬落和恢复站立 |
| 低速 WBC 连续行走 10 步 | 已完成，`65.001 s` 完成 12 次支撑切换 |
| 原 `walk_wbc` 流程的 Lyenbot 适配 | 进行中，默认 StateEst 已完成 65.001 秒和 17 次切换；ankle-pitch 实际位置动态限位未通过 |
| MPC 约 0.15 m/s 连续 10 步 | 暂缓，等待原 `walk_wbc` 适配完成 |
| 真机接口 | 未开始，缺少硬件资料 |

### 当前问题

- MuJoCo 足底由原 collision mesh 与新增规则 box 共同承载，WBC 矩形 CoP 约束和实际复合支撑面尚未完全一致；声明接触模式与实测接触失配时仍可能出现 `qp_status=0` 但输出幅值异常的解；
- 纯 WBC 默认基线仍使用保守的 `0.03 m/s` 和 3 秒双支撑；新增 2.25 秒协调换载对照已通过，但双支撑期间仍暂停前向速度，尚不能代表 MPC 或更高速动态行走性能；
- 默认稳定 walk 仍使用约 `±0.40 rad` 躯干侧倾；新增中等侧倾 opt-in 候选在保留约 `34.5 mm` 足高和 `0.675 rad` 摆动膝屈曲的同时，把最大绝对 roll 降到 `0.377610 rad`，并通过 65 秒、15 次切换和全部关节零越界；默认参数尚未替换；
- 当前竖直 Bezier 在实际约 `phi=0.75` 触地时尚未到虚拟 `1.4` 端点；提前做平滑端点、触地搜索和持续确认会改变后续双支撑锚点，现有在线候选均未通过；这些结论作为原 `walk_wbc` 适配的回归边界保留；
- 原 `walk_wbc` 的完整输入已接回；正式 Lyenbot 入口现与 staged 一致，在常规 DSt 暂停前向 joystick 请求，避免两脚刚性约束期间 `base_pos_des.x` 继续推进。默认 StateEst 已完成 65.001 秒和 17 次切换；
- 原 `demo/walk_wbc.cpp` 继续保留 AzureLoong 默认行为；新的 `lyenbot_walk_wbc` 已替换 scene、URDF、PVT、23 actuator、`18/26` WBC、关节名和模型重力前馈，并接入初始双支撑、完整复合足接触和已验证的中等侧倾/中心化摆腿。第三次换载的异常 QP 已不再复现，但 ankle-pitch 在落地冲击下仍有实际位置软限位穿透；
- MPC 行走尚未建立与已通过纯 WBC 基线等速、同判据的对照，且按当前计划暂缓；
- 真机接口缺少执行器、编码器、通信和传感器资料。

因此，当前结论可以表述为：

```text
Lyenbot 的模型、站立、左右准静态单步、staged 低速纯 WBC 和原 walk_wbc 连续控制链已经跑通；原 walk_wbc 尚需完成踝关节动态限位验收，MPC 暂缓。
```

### 下一步

1. 对齐正式入口与 staged 的 ankle-pitch 落地阶段日志，定位动态穿透来自触地速度、足底几何、踝目标还是闭链负载；
2. 保持已通过的 DSt 前向暂停、StateEst 参数和原模块顺序，不再叠加已证伪的三段接触状态机、载荷同步或时间驱动候选；
3. 候选先同时通过 20 秒稳定性和 23 关节零越界，再运行 65 秒至少 10 次切换；
4. 不修改 URDF 限位、不放宽安全阈值，也不以降低换支撑次数换取表面余量；
5. 原 `walk_wbc` 动态限位验收完成后，再重新启动低速 MPC 计划。

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

`JointLayout` 保存活动关节和各语义关节组的名称集合。`RobotModelConfig` 还可提供足底 `contactHalfLength/contactHalfWidth`；Lyenbot 当前分别为 `0.08 m/0.04 m`，用于建立与法向力耦合的 CoP 约束。

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
- Lyenbot 配置化路径使用 26 个 QP 约束，以 `|tx|<=halfWidth*fz`、`|ty|<=halfLength*fz` 表达真实足底 CoP 边界；未配置足底尺寸的 AzureLoong 旧路径继续使用原 22 约束；
- 双支撑卸载阶段可通过 DataBus 的 `wbc_swing_foot_fz_max` 连续收紧未来摆脚法向力硬上限；
- 站立进入 WBC 时锁存左右足底世界位姿作为 anchor；
- Lyenbot 使用较低刚度、较高阻尼的保守站立增益。

Lyenbot 的合法单支撑参考不能只靠横移直立基座获得。独立姿态扫描表明，需要联合选择 base-y、base height 和支撑侧 roll，才能同时满足固定双足锚点 IK、URDF 腿部限位、膝最小屈曲和 CoM 支撑范围。该结论目前通过单步 demo 上游参考适配实现，没有改变 WBC 的任务优先级结构。

### 11.2 步态调度

Lyenbot demo 启用：

- `useMeasuredContact`；
- touchdown force 阈值；
- 最小落地相位 `0.75`；
- “摆脚必须先离地，再允许落地事件”；
- 双支撑 transfer；
- 新支撑脚完整实测 `fz` 低于 `5 N` 时冻结双支撑转移进度，接触恢复后继续。

`minimumTransferContactForce` 默认值为 0，因此新增接触门控不改变 AzureLoong 旧路径。Lyenbot 连续 walk 在调度前把按全部足部 geom 汇总的复合足法向力写入 `fL[2]/fR[2]`，避免只使用规则 contact box 标量漏掉实际承载。

### 11.3 FootPlacement

新增步长 clamp 和可配置的 `forwardOffset`、`inwardOffset`、`landingHeightOffset`、`lateTouchdownStretchStep`。默认值保持原 AzureLoong 常量；Lyenbot demo 显式取消原后向偏置、保持较宽站距、使用 `-10 mm` 落地高度并禁用相位末端逐周期向下累加，避免把超时等待变成无界下降参考。当前同侧最大步长为 `0.10 m`。

当前 Lyenbot 连续 walk 的 `stepHeight` 保持 `0.025 m`。`0.035/0.040 m` 虽能跑满 65 秒，但分别出现 3/25 个 ankle-pitch 实际位置越界样本；`0.050 m` 在 `61.100 s` 触发基座加速度安全停止，均已撤销。原 AzureLoong `walk_wbc` 的 `0.12 m` 只能作为上游对照，不能绕过 Lyenbot 闭环限位验收直接复制。

当前 7 阶竖直 Bezier 的峰值/虚拟落地 phase 为 `0.2/1.4`，连续基线实际 touchdown phase 为 `0.743~0.759`。尝试 `0.30/0.85` 平滑端点、支撑脚相对落地高度、最大 `6 mm` 有界触地搜索和 `20/50 ms` 连续确认后，分别出现不触地、冲击后接触丢失或 `33.503 s` 安全停止，所有运行时代码均已撤销。该结果说明当前提前触地涉及复合足几何和双支撑锚点，不能仅替换曲线端点。

协调基线的摆动膝实际/期望最大屈曲约为 `0.576/0.605 rad`。现有 6D 足端任务已经基本确定 6-DoF 腿姿态，没有独立膝冗余；后续需离线联合搜索足端高度、足 pitch 与膝踝余量，不能直接添加膝角硬任务。

纯 WBC 目标速度限制为 `0.03 m/s`，双支撑期间暂停前向命令，使横向承重转移与下一步前向累积解耦；MPC 路径仍使用配置速度，需在 P1 单独验收。

为区分“验证脚手架”和“正式上游流程迁移”，`lyenbot_staged walk` 默认行为继续作为安全回归基线；可选参数 `--upstream-wbc-inputs` 同时恢复原 `walk_wbc` 的 `des_delta_q/des_dq/des_ddq` 运动种子，并按 MuJoCo 总质量计算双脚名义 `Fr_ff`。该模式禁止与 `mpc` 同时使用，默认诊断写入 `../record/lyenbot_walk_upstream_inputs_diagnostic.csv`。

在此基础上，`--coordinated-transfer` 启用 2.25 秒协调换载：base-y、roll、诊断用 CoM-y 和双脚名义 `Fr_ff` 使用同一平滑 phase，新支撑脚从总重 10% 的名义预载开始。该参数必须与 `--upstream-wbc-inputs` 同时使用，默认诊断写入 `../record/lyenbot_walk_coordinated_transfer_diagnostic.csv`。`Fr_ff` 只作为 QP 名义工作点，最终方案不会用计划 phase 强制限制旧支撑脚实际法向力。

## 12. 分阶段 demo

相关文件：

```text
demo/lyenbot_staged.cpp
demo/lyenbot_posture_feasibility_test.cpp
```

构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target generate_mjcf model_check lyenbot_staged lyenbot_posture_feasibility_test
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
(cd build && ./lyenbot_staged step-left)
(cd build && ./lyenbot_staged step-right)
./build/lyenbot_posture_feasibility_test common/robot_configs/lyenbot.json
```

GUI：

```bash
(cd build && ./lyenbot_staged pd --gui)
(cd build && ./lyenbot_staged wbc --gui)
(cd build && ./lyenbot_staged walk --gui)
(cd build && ./lyenbot_staged walk mpc --gui)
```

安全停止条件包括 NaN、QP 非零状态、QP 修正后浮动基座加速度前 6 维无穷范数超过 `100`、基座高度过低和 roll/pitch 过大。QP 不可行或输出幅值异常后不得继续使用该解，也不得为了让测试继续而删除安全检查。

正式 `walk` 从 10 秒开始在双支撑中平滑进入协调姿态，13 秒开始低速纯 WBC 步态。左右支撑参考按当前足位置镜像生成 base-y 和 roll，base height 保持 `0.80 m`；每次落脚后使用 3 秒双支撑转移，且新支撑脚复合足 `fz<5 N` 时冻结转移。纯 WBC 运行窗口为 65 秒，能够覆盖至少 10 次完整支撑切换。禁用 MPC 时不再调用 `MPC::dataBusWrite()`，避免其覆盖 joystick/步态生成的 `base_pos_des/des_dq`。

walk 的膝位置目标保留 `0.03 rad` 最小屈曲。全部关节位置参考在 URDF 边界内预留 `0.030 rad`；左右 ankle-pitch 因承受落脚冲击使用 `0.060 rad` 参考余量，并继续采用限位前 `0.02 rad` 的越限方向前馈衰减。该保护位于 Lyenbot staged 适配层，不修改 URDF 和通用 WBC。

`step-left` 表示左脚抬落、右脚支撑；`step-right` 表示右脚抬落、左脚支撑。当前时间表为：

- `8~11 s`：平滑联合转移 CoM-y、base height 和支撑侧 roll；当前验收参考为 `base_z=0.80 m`、roll `±0.40 rad`，CoM 横向目标为支撑脚中心向中线内移 `4 mm`；
- `11~13 s`：保持协调姿态并完成双支撑 wrench 卸载；
- `13 s` 起：保持 `DSt` 双足 wrench 约束但只锁未来支撑脚运动学；完整实测摆脚 `fz<5 N` 连续 `20 ms` 后进入正式单支撑；
- 摆脚先抬高 `25 mm` 再原地落下；确认离地至少 `2 s` 后，要求左右复合足完整实测 `fz` 均超过 touchdown 阈值并连续 `50 ms`，避免首次冲击反弹被误判为稳定触地；
- touchdown 后保持落地姿态 `0.5 s`，再用 `3 s` 平滑恢复 roll、base height 和 CoM-y；恢复完成且双足持续接触后累计稳定时间。

总时长为 24 秒，不调用 GaitScheduler、FootPlacement 或 MPC。

当前诊断 CSV 为 271 列，包含 `contact_release`、WBC 规划/结果左右 6D wrench、由 MuJoCo contact API 按 ankle-roll body 汇总全部 geom 的实测左右 6D wrench 和 CoP，以及每关节进入 PVT 前的有效前馈 `tau_ff` 与最终 `tau_out`。启用关节限位保护后，`tau_ff` 是经过越限方向衰减的值，原始 WBC 力矩需查看安全 sidecar 的 `wbc_tau`。修正后站立总法向力约 `312.3 N`，与模型重量一致；完整 wrench 不整体接入 DataBus，仅由单步 demo 使用其法向力确认释放状态。

单步安全停止时还会生成 `<diagnostic>.safety.csv`：1 kHz 环形缓存只保留失败前最近 `100 ms`，记录完整状态、任务层 `ddq`、QP、wrench 和电机输出。该追踪已将原右脚 `11.294 s` 尖峰定位到 `PosRot` 驱动左膝穿过 0 rad 直腿奇异。单步和正式 walk 适配层现对左右膝位置目标使用 `0.03 rad` 最小屈曲保护；普通 `pd/wbc` 不受影响。

两种模式均已通过验收。离线姿态扫描在每侧 `15457` 个样本中分别得到 `546/558` 个满足 IK、关节范围、膝屈曲和 CoM 支撑边界的候选；最小 roll `0.05 rad` 的候选腿部限位余量仅 `0.000713 rad`，不适合作为动态参考，而 roll `0.40 rad`、`base_z=0.80 m` 的候选可把 CoM 带到支撑脚中心附近。采用该协调姿态后：

- `step-left`：`13.028 s` 确认释放，完成腾空、落脚和恢复，稳定 `4.870 s`，支撑足最大滑移 `0.064 mm`，最大 roll/pitch 为 `0.404231/0.006186 rad`；
- `step-right`：`13.027 s` 确认释放，完成腾空、落脚和恢复，稳定 `4.867 s`，支撑足最大滑移 `0.062 mm`，最大 roll/pitch 为 `0.404443/0.006660 rad`。

两侧均输出 `LYENBOT_SINGLE_STEP_OK`。详细证据见 `DEBUG_LOG.md` 的 D008。

历史排查中的时序和力矩分解表明：延后释放只会等量延后失败，提前到 11.5 秒释放会因支撑 CoP 尚在反向边缘而更早失败；PVT 没有抵消 WBC ankle-roll 前馈，但双脚闭链接触时实测单脚 wrench 仍不跟随开环计划值。这些结果排除了继续调整固定时刻或单一 CoM 偏置的路线。

关节诊断还证明原较晚基线依赖 ankle-roll 穿过 `±0.1326 rad` URDF 范围。staged 适配层现夹紧全部 WBC 位置目标，并在限位前 `0.02 rad` 衰减越限方向前馈；当时的左右有效失败基线为 `14.239/14.247 s`。合法目标下 13 秒的 CoM 位于支撑脚中心外约 `46~47 mm`，超过 `40 mm` 足底半宽，因此将问题定位到关节行程内的直立横移姿态不可行，而不只是释放后的 CoP 建立滞后。

进一步验证过两类 joint-limit-aware 候选，均未保留。把 URDF 限位作为 `static_Contact` 后的高优先级等式任务时，无论硬激活还是连续速度阻尼，`step-left` 都在 `9.726 s` 形成 QP 不可行；说明现有等式优先级不能直接表达关节可行域。捕获点门控在 13 秒会因 `|y_cp|` 超过 `35 mm` 而阻止释放；配合 50% 两段卸载可安全等待到 20 秒，但没有完成抬脚。提前在 `12.5 s` 的短暂安全窗口放行后，`12.879 s` 确认离地，仍于 `13.728 s` 横向失稳。在线一维参考投影也因单支撑可行区不是原直立参考到零误差之间的简单线段而失败。这些候选均已撤销。

离线三维姿态扫描最终给出了满足接触和 ankle-roll 行程的可行参考，并完成左右单步验收。后续正式 walk 仍不能退回固定时刻、单一 CoM 偏置或新增硬等式扫参，而应把“协调可行姿态 + 完整实测接触确认”扩展为连续状态机。

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

动态映射使用独立 headless 目标 `lyenbot_joint_mapping_test` 验证。测试从同一 `home` keyframe 运行无激励基线和单关节 `0.02 rad` 增量轨迹，通过 `MJ_Interface` 与 PVT 实际控制路径比较两条轨迹的增量响应，避免把重力下的自然姿态变化误判为串扰。

已通过结果：

```text
JOINT_MAPPING_SUMMARY passed=23 total=23
LYENBOT_JOINT_MAPPING_OK
最大非目标增量=0.006887 rad
```

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

准静态原地单步已经通过：`step-left/right` 均完成释放、腾空、原地落脚和恢复站立，落地后稳定时间分别为 `4.870/4.867 s`，支撑足最大滑移分别约 `0.064/0.062 mm`。

正式纯 WBC 连续 walk 已通过。复现命令为：

```bash
(cd build && ./lyenbot_staged walk --diagnostic=/tmp/lyenbot_walk_10step_ankle_margin.csv)
```

结果为 `LYENBOT_STAGE_OK time=65.001 base_z=0.799837 steps=12`。5601 个 100 Hz 样本中 `qp_status` 全为 0，最大等式残差 `1.137e-13`、最大不等式违反 `2.574e-12`；最小基座高度 `0.797363 m`，最大绝对 roll/pitch 为 `0.410503/0.007819 rad`。全部 23 个关节实际 position/velocity/effort 均未越界，最小位置余量为左 ankle-pitch 的 `0.002458 rad`，最小速度/effort 余量分别为 `10.1331 rad/s` 和 `24.1835 Nm`。

旧基线中的禁用态 MPC 覆盖、Azure 专用 FootPlacement 偏置、复合足触地遗漏和接触丢失时仍推进 transfer 已分别隔离或配置化。连续 WBC 稳定基线完成后，先处理 P0-6 步态质量，再进入低速 MPC 对照。

步态质量补充验证表明，当前稳定基线尚不能直接通过单变量调参改善外观：协调 roll 降到 `0.35 rad` 的候选在约 `19.4~19.8 s` 停止；锁存旧 touchdown-y 的候选在原姿态下于 `31.449 s` 停止；提高 `stepHeight` 则导致踝位置越界或长期失稳。因此运行参数仍保持已通过的 `base_z=0.80 m`、`roll=0.40 rad`、`stepHeight=0.025 m` 和 ankle-pitch 参考余量 `0.060 rad`。后续先改进轨迹端点、膝屈曲冗余参考和协调姿态生成，再进入 MPC 对照。

上游 WBC 输入对照模式的复现命令为：

```bash
(cd build && ./lyenbot_staged walk --upstream-wbc-inputs --diagnostic=/tmp/lyenbot_walk_upstream_inputs.csv)
```

Lyenbot 总重为 `312.327 N`，名义 `Fr_ff` 为每脚 `156.164 N`。该模式在现有 3 秒 transfer 下输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.799714 steps=12`；5601 个样本中 QP 全成功、23 关节实际 position/velocity/effort 全部零越界。`des_dq_x` 峰值 `0.013036 m/s`，说明运动种子已接回，但仍受双支撑暂停限制。只把 transfer 缩短至 `1.5 s` 的候选在 `19.357 s` 停止并已撤销，因此下一步必须联合迁移横向姿态、CoM 和 wrench 转移。

2.25 秒协调换载对照的复现命令为：

```bash
(cd build && ./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --diagnostic=/tmp/lyenbot_walk_coordinated_transfer.csv)
```

结果为 `LYENBOT_STAGE_OK time=65.001 base_z=0.800002 steps=16`。QP 全成功、23 关节动态限制零越界；最大实测横向加速度 `4.665 m/s²`、最大 WBC 横向 QP 加速度 `0.457 m/s²`，均低于 3 秒上游输入对照的 `5.829/0.626 m/s²`。最大绝对 roll 仍为 `0.4025 rad`，所以该模式改善了节奏和换支撑瞬态，但尚未解决侧倾与抬膝观感，P0-6 继续进行。

排查中，新支撑脚零名义载荷会与接触门控形成 phase 零点死锁，只完成 1 次切换；将旧支撑脚法向力上限强制绑定计划换载则在 `19.575 s` 失稳。最终仅保留 10% 名义预载和平滑 `Fr_ff`，实际载荷分配仍由 QP 决定。

摆脚端点候选未改变当前运行方式：`0.30/0.85` 平滑竖直轨迹及配套触地搜索/确认没有通过完整验收，最终源码已恢复原 `0.2/1.4` 轨迹。恢复后再次运行 2.25 秒协调换载，精确输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.800002 steps=16`。

离线摆腿联合扫描已完成。当前 `0.80 m/0.40 rad` 姿态族中，`35 mm` 足高和约 `0.10 rad` 摆动期足 pitch 能提高屈膝并保留静态 ankle-pitch 余量。对应 opt-in 命令为：

```bash
(cd build && ./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --swing-quality-candidate --gui)
```

headless 验证输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.800118 steps=15`；QP 全成功、23 关节动态限制零越界。摆动膝实际/期望最大屈曲为 `0.669/0.703 rad`，实际摆动足最高点 `34.38 mm`，但最大绝对 roll 仍为 `0.402687 rad`，第一次落地后 ankle-pitch 最小位置余量仅 `0.001669 rad`，因此该模式是视觉候选而非默认基线。

离线余量更大的 `base_z=0.82 m、roll=0.30 rad、base-y≈±0.09 m` 姿态直接上线后在 `15.888 s` 触发 `qp_base_ddq_max=239.375`，相关运行时代码已撤销。这证明减侧倾不能只替换静态姿态，必须联合处理落地后的动态换载。

进一步诊断表明，低侧倾失败时单支撑 CoP-y 已贴近 `±40 mm` 边界，且实时 hip/base 位置会把摆脚继续向外推。最终保留的中等侧倾候选命令为：

```bash
(cd build && ./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --swing-quality-candidate --moderate-roll-centered-swing-candidate --gui)
```

该模式使用 `base_z=0.805 m`、`roll=0.375 rad`，并只在该 opt-in 路径禁止摆脚相对本步起点继续向外扩张。headless 结果为 `LYENBOT_STAGE_OK time=65.001 base_z=0.802865 steps=15`；最大绝对 roll `0.377610 rad`，摆动膝实际/期望最大值 `0.674746/0.707833 rad`，实际足高 `34.52 mm`。QP 全成功、23 关节动态限制零越界，最小位置余量 `0.012601 rad`，15 次 touchdown 足间距稳定约 `0.207 m`。最大 WBC 横向 QP 加速度 `0.561 m/s²` 高于仅摆腿候选。该结果保留为原 `walk_wbc` 适配的视觉和动态对照；当前不直接进入 MPC，也不继续在 staged 中静态降低 roll。

### 14.4 原 `walk_wbc` 流程适配

已新增独立 `lyenbot_walk_wbc`，原 AzureLoong `walk_wbc` 文件和目标保持不变。新入口支持从项目根目录或 `build/` 解析 Lyenbot 配置，默认 headless，可显式传入 `--gui`、`--stand-only`、`--duration=<秒>`；`--measured-state` 只用于状态估计 A/B，不是正式验收模式。

配置化构建和回归均已通过。状态估计足 Frame 高度使用独立的 `state_estimation.foot_frame_ground_height`：Azure 未配置时默认 `0.07 m`，Lyenbot 鞋底接触 Frame 显式配置为 `0.0 m`，不再与 `gait.foot_height=0.0407 m` 混用。MuJoCo accelerometer specific force 还按完整姿态旋转并补回世界重力，默认 StateEst 双足站立已通过 13 秒。

高频审计确认 StateEst 输出为世界系速度，`Pin_KinDyn` 转为局部广义速度后再旋回世界系的最大误差约 `6.7e-16`。原 `getTrustRegion_wt_h()` 误把 `LegState` 与 `MotionState::Stand` 比较，现改为明确的 `DSt`；Lyenbot 还通过 `state_estimation.angular_velocity_measurement_noise_scale=0.0001` 提高原 `Eul_W_filter` 的 gyro 观测带宽，默认值 `1.0` 保持旧机器人行为。诊断可用 `--state-est-diagnostic=<CSV>`，采样周期可用 `--state-est-diagnostic-period=<秒>` 调整。

GaitScheduler 现以默认关闭的初始 `DSt` 管理 3 秒准备；该 phase 使用原 Stand WBC 控制 CoM，随后恢复 Walk WBC。常规 touchdown 后使用 2.25 秒协调换载。完整复合足 `fz`、中等侧倾/中心化摆腿、35 mm 足高、0.10 rad 足 pitch 和固定 `-0.010 m` 世界落地高度均由 Lyenbot 配置接入，不迁入 staged 的完整状态机。

常规 DSt 还在 Lyenbot 入口暂停前向 joystick 请求：不能只清零 `des_delta_q/des_dq/des_ddq`，因为 `joystick.dataBusWrite()` 仍会推进 `base_pos_des.x`，而 Walk `PosRot` 会在双脚固定时继续跟踪它。该最小输入适配解决了原 `t=13.083 s` 的第三次换载异常 QP，不改变共享 GaitScheduler、WBC 接触状态或 AzureLoong 行为。

当前运行证据：

```text
./build/lyenbot_walk_wbc --stand-only --duration=13
LYENBOT_WALK_WBC_OK time=13.001 base_z=0.848605 transitions=0

./build/lyenbot_walk_wbc --duration=13 --measured-state
LYENBOT_WALK_WBC_OK time=13.001 base_z=0.811143 transitions=2

./build/lyenbot_walk_wbc --duration=13
LYENBOT_WALK_WBC_OK time=13.001 transitions=2

./build/lyenbot_walk_wbc --duration=65
LYENBOT_WALK_WBC_OK time=65.001 base_z=0.806754 transitions=17
```

默认 StateEst/WBC/PVT 链现已完成 65 秒和 17 次切换。最终 10 ms 诊断共 301 列、6110 行且坏行 0；QP 最大基座加速度约 `13.6688`，最大等式/不等式残差约 `2.27e-13/3.07e-12`。因此第三次换载控制阻塞已解决，但动态限位总验收尚未完成。

扩展后的 `--state-est-diagnostic` 还记录实际送入 WBC 的当前/下一支撑状态、contact-release、完整实测/WBC wrench、运动学与 QP 基座加速度、QP 残差、任务误差和接触 Jacobian 奇异值。1 kHz 复现表明 `t=13.083 s` 的运动学基座加速度最大值仅约 `13.3`，QP 修正却约 `19534.1`；接触 Jacobian 最小奇异值约 `0.139`、QP 等式残差约 `1.8e-11`。因此根因不是 StateEst、运动学秩崩溃或求解失败，而是 DSt 声明双脚刚性接触时与实测接触集合不一致。

直接替换 WBC `legState` 的多个滞回候选会破坏 `static_Contact/SwingLeg` 配套语义；三阶段接触集、时间驱动换载和载荷同步换载也分别在 `12.696/11.917/10.906 s` 附近失败，均已撤销。最终修复来自输入时序而不是接触状态重写：常规 DSt 将前向请求降为 0，单支撑恢复 `0.03 m/s`。

正式入口的结束行现审计全部 23 关节。最终 65 秒速度/力矩零越界，但位置有 15655 个越界采样；最差 `left_ankle_pitch_joint=-0.562484 rad`，比 URDF 下限低 `0.028484 rad`，当时目标 `-0.474 rad`、测得力矩为正向恢复。ankle-pitch 参考余量 `0.070/0.080 rad`、提高 `kd` 和限位附近前馈衰减均未同时通过稳定性与切换次数，已撤销。因此 P0-7 继续保持进行中。

### 14.5 MPC

调用路径已经预留，但按当前开发路线暂缓。恢复条件是原 `walk_wbc` 的 Lyenbot 入口完成 65 秒、至少 10 次支撑切换及 23 关节动态限制验收；之后再以该正式 WBC 路径建立 `0.03 m/s` 低速 MPC 对照。

## 15. 当前禁止的处理方式

- 修改原始 URDF joint axis；
- 修改原始 URDF position/velocity/effort 限制；
- 修改原始 URDF 惯量来“调稳定”；
- 虚构真实减速比；
- 放大 effort 以掩盖控制问题；
- 关闭 NaN、姿态或高度安全检查；
- 把短暂未跌倒描述成连续行走成功；
- WBC 连续 10 步未通过时直接反复调 MPC；
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
| `demo/lyenbot_walk_wbc.cpp` | 保留原模块顺序的 Lyenbot 配置化 `walk_wbc` 入口 |
| `demo/lyenbot_posture_feasibility_test.cpp` | 离线单支撑姿态可行域扫描 |
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
“当前验收总览”和当前任务条目；其他文档按需读取。左右准静态单步已通过，
继续推进 P0-5 的 Lyenbot WBC 连续 10 步验收。原始 URDF 必须只读。不要改动
record/matlabReadDataScript.txt 或 build-debug/，除非我明确要求。
```
