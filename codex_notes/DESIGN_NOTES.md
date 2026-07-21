# 设计说明

## 1. 文档定位

本文件记录项目长期稳定的系统架构、模块边界、数据流和设计原因。具体某次修改见 `CHANGELOG.md`，Bug 排查见 `DEBUG_LOG.md`，Lyenbot 细节见 `LYENBOT_ADAPTATION.md`。

## 2. 项目目标

本项目基于 OpenLoong Dynamics Control，保留原有 AzureLoong 的 MPC/WBC 人形机器人控制框架，同时通过适配层支持 Lyenbot。

设计目标：

- 保留 OpenLoong 原有控制框架和 demo；
- 机器人机械模型以 URDF 为真源；
- 用配置表达控制语义；
- 用名称映射消除危险数字下标；
- MuJoCo 模型可以从 URDF 重复生成；
- 模型、站立、WBC、行走和 MPC 分阶段验证；
- 为未来真机接口保留清晰边界。

## 3. 总体模块关系

```text
                  ┌──────────────────────────────┐
                  │ 原始 URDF：机械参数唯一真源 │
                  └──────────────┬───────────────┘
                                 │
                  ┌──────────────┴───────────────┐
                  │                              │
                  ▼                              ▼
      Pinocchio 浮动/固定基座模型       generate_mjcf 派生 MuJoCo
                  │                              │
                  │                              ▼
                  │                   scene + model + actuator/sensor
                  │                              │
                  └──────────────┬───────────────┘
                                 ▼
MuJoCo ── MJ_Interface ── DataBus ── Pin_KinDyn ── WBC/MPC/步态
  ▲                              │                         │
  │                              └──── PVT/力矩限幅 ───────┘
  └──────────────── actuator id 显式映射 ─────────────────┘
```

各模块主要职责：

| 模块 | 职责 | 不应承担的职责 |
|---|---|---|
| URDF | 拓扑、几何、质量、惯量、关节轴和机械限制 | PD 增益、步态参数、控制语义 |
| `RobotModelConfig` | 机器人名称语义、路径和非机械仿真参数 | 覆盖 URDF 机械参数 |
| MuJoCo | 接触和刚体物理仿真、虚拟传感器 | 成为机械参数的人工第二真源 |
| `MJ_Interface` | MuJoCo 地址查询、反馈读取、控制输出 | WBC/MPC 算法决策 |
| `DataBus` | 模块间统一状态与指令交换 | 隐式改变关节排列 |
| Pinocchio | 运动学、动力学、Frame、Jacobian、限制读取 | 仿真接触积分 |
| PVT | 关节位置/速度误差转力矩与限幅 | 生成步态与接触规划 |
| GaitScheduler | 支撑腿/摆动腿状态切换 | 计算完整动力学 |
| FootPlacement | 摆动足目标与轨迹 | 直接输出 actuator 命令 |
| WBC | 优先级任务、广义加速度与接触力分配 | 修改机械模型限制 |
| MPC | 较长时域的基座/接触力规划 | 替代底层关节控制 |

## 4. 三类参数的所有权

### 4.1 URDF 机械参数

只能从 URDF 经 Pinocchio 读取：

- link 质量、质心和惯量；
- joint 父子拓扑、origin 和 axis；
- position、velocity、effort 限制；
- 固定 Frame 和碰撞/可视几何；
- 活动关节的 `idx_q/idx_v`。

任何 JSON、C++ 常量或 MuJoCo 派生 XML 都不得覆盖这些参数并声称它们是真实机械值。

### 4.2 机器人语义

标准 URDF 不知道“左腿”“右脚”“腰部任务”等控制含义，因此由 `RobotModelConfig`/JSON 表达：

- 哪些关节属于左腿、右腿、左臂、右臂、腰和头；
- 哪个 Frame 是左右足底；
- 哪个 body 是基座；
- MuJoCo sensor 的名称；
- 使用哪个 URDF、scene 和 PVT 配置。

语义配置保存名称，不保存 `q[19]` 之类的机械下标。

### 4.3 控制与仿真参数

URDF 不提供以下数据，允许独立配置：

- PVT `kp/kd` 和滤波频率；
- MuJoCo timestep、damping、frictionloss、armature；
- 地面摩擦、步态周期、步高、目标速度；
- touchdown 阈值；
- WBC/MPC 权重和任务增益；
- 仿真初始关节姿态。

这些值应明确标记为仿真或控制初值，不能直接当作实机参数。

## 5. 浮动基座与广义坐标

人形机器人站立和行走时，躯干不固定在世界上，因此动力学模型使用浮动基座。

对于有 `N` 个 1 DoF 活动关节的机器人：

```text
nq = 7 + N
nv = 6 + N
```

位置向量：

```text
q = [世界系基座位置 xyz,
     世界系基座四元数 xyzw（Pinocchio 约定）,
     关节位置]
```

速度向量：

```text
v = [基座线速度 xyz,
     基座角速度 xyz,
     关节速度]
```

四元数有 4 个数但只有 3 个独立旋转自由度，因此 `nq=nv+1`。

MuJoCo keyframe 的四元数使用 `wxyz`，Pinocchio 广义坐标使用 `xyzw`。接口转换必须显式处理，不能直接复制 4 个元素。

Pinocchio free-flyer 的基座平移位置在父坐标系（世界系）表达，而基座速度计算使用局部约定。`Pin_KinDyn::dataBusRead()` 会根据当前旋转完成必要坐标转换。

项目同时建立固定基座模型，用于身体坐标系下的局部 IK/Frame 计算；站立、行走和完整动力学仍使用浮动基座模型。

## 6. Frame、Jacobian 与动力学

Frame 是附着在 link 上的参考坐标系。足底控制必须使用接触 Frame，而不是仅使用踝关节原点。

Frame 速度近似满足：

```text
v_frame = J(q) v
```

Frame 加速度满足：

```text
a_frame = J(q) a + dJ(q,v) v
```

浮动基座刚体动力学写作：

```text
M(q) ddq + C(q,dq)dq + G(q) = S^T tau + J_contact^T F
```

其中：

- `M`：质量矩阵；
- `C*dq`：科氏力和离心力；
- `G`：重力项；
- `tau`：驱动关节力矩；
- `F`：足底接触力/力矩；
- `S`：从广义自由度中选择驱动关节；
- `J_contact`：接触 Frame Jacobian。

## 7. 名称映射设计

### 7.1 Pinocchio 映射

流程：

```text
配置中的 URDF joint name
        ↓
model.getJointId(name)
        ↓
idx_qs[jointId] / idx_vs[jointId]
        ↓
motorQIndex / motorVIndex
```

这样电机数组可以按语义配置排序，而 Pinocchio 仍按自身模型顺序计算。

### 7.2 MuJoCo 反馈映射

流程：

```text
joint name
  ↓ mj_name2id
joint id
  ↓ jnt_qposadr / jnt_dofadr
qpos/qvel 地址
```

### 7.3 MuJoCo 控制映射

流程：

```text
joint id
  ↓ actuator_trnid
actuator id
  ↓
mj_data->ctrl[actuatorId]
```

不能使用 `ctrl[i]` 假设 actuator 排列等于控制数组排列。

## 8. MuJoCo 派生模型设计

MuJoCo 文件由生成工具从 URDF 派生：

```text
主 URDF
  ├─ normalized_source.urdf  仅用于兼容转换
  ├─ Lyenbot.base.xml        URDF 转换基础结果
  ├─ Lyenbot.xml             actuator/sensor/freejoint/keyframe
  ├─ scene_lyenbot.xml       地面、灯光和视觉环境
  └─ generation_manifest.txt 来源哈希和生成命令
```

生成物可删除后重建，但原始 URDF 不得由生成器反写。

首版 actuator 使用：

```text
gear = 1
ctrlrange = [-URDF effort, +URDF effort]
```

含义是控制器直接输出关节侧扭矩。真实减速比未知时，不在仿真中编造电机侧参数。

## 9. 传感器设计

MuJoCo `<sensor>` 是虚拟测量，不改变 URDF 机械结构。

| sensor | 作用 | 当前使用情况 |
|---|---|---|
| `framequat` | 基座姿态四元数 | 已读取并转换为 RPY/广义姿态 |
| `velocimeter` | IMU site 局部线速度 | 已生成和查询，尚未直接用于 `baseLinVel` |
| `gyro` | IMU site 角速度 | 已写入 DataBus |
| `accelerometer` | IMU site 加速度 | 已写入 DataBus |
| 左右 `touch` | 足底法向接触力标量 | 用于落地检测，写入 `fL/fR.z` |

完整 WBC 接触 wrench 不能仅靠 touch 标量获得，后续需要 contact force API 或完整力/力矩传感方案。

## 10. DataBus 数据流

OpenLoong 使用模块化的 `read → compute → write` 模式：

```text
MJ_Interface.read sensors
       ↓
MJ_Interface.write DataBus
       ↓
Pin_KinDyn.read DataBus
       ↓
Pin_KinDyn.compute kinematics/dynamics
       ↓
Pin_KinDyn.write DataBus
       ↓
Gait / FootPlacement / WBC / MPC read-compute-write
       ↓
PVT read desired/current state and compute torque
       ↓
MJ_Interface sends torque to mapped actuators
```

DataBus 的广义状态遵循 Pinocchio 维数和语义；电机数组与广义向量之间必须经过显式映射。

WBC 在 `computeTau()` 内直接计算 QP 诊断量，并通过 DataBus 输出：

- `wbc_qp_equality_residual_inf`：前 6 个浮动基座等式约束的无穷范数残差；
- `wbc_qp_inequality_violation_max`：全部 QP 不等式约束的最大违反量。
- `wbc_ddq_final`：优先级运动学任务输出的广义加速度；
- `wbc_ddq_qp`：QP 浮动基座动力学修正后的广义加速度。
- `wbc_swing_foot_fz_max`：双支撑过渡中未来摆脚的时变法向力硬上限；默认无穷大，仅由需要显式卸载的路径设置。
- `wbc_contact_release`：默认关闭的接触释放适配信号。置位时保持 `DSt` 的双足 wrench 约束，但运动学静态接触仅保留 `legStateNext` 指定的未来支撑脚，使摆脚任务可以解除物理接触。

残差必须在 WBC 内部使用实际 `A/lbA/ubA/xOpt` 计算。不能在 demo 中使用 `wbc_ddq_final` 反推，因为该字段最终承载运动学任务的 `ddq_final_kin`，不是 QP 修正后的内部 `eigen_ddq_Opt`。

`lyenbot_staged` 的安全追踪采用 1 kHz、最近 `100 ms` 的内存环形缓存，只在安全停止时写 sidecar CSV。它读取 WBC 已有任务中间结果，不向 DataBus 增加调试专用大向量，也不改变普通 100 Hz 诊断的采样周期。任务层追踪已证明右脚双支撑尖峰首先来自 `PosRot` 对伸直膝奇异位形的放大，而非 QP。普通诊断还逐关节记录 `tau_ff` 与 `tau_out`，用于区分 WBC 前馈和 PVT 反馈，不参与控制。

优先级任务的加速度递推必须保证低优先级增量位于累计父任务零空间。由于动态加权伪逆包含 `M^-1`，仅对 `J*N` 求伪逆不能保证结果仍位于 `N` 的值域；加速度增量需要显式左乘当前累计零空间 `N`。根任务 `N=I`，不受此修复影响。

控制模块的数据写入必须服从启用状态。特别是禁用态 MPC 的 `dataBusWrite()` 会把期望状态写成当前状态，调用方若无条件执行会覆盖 WBC/步态模块刚写入的轨迹目标；非 MPC 运行路径不得调用该写出步骤。

## 11. WBC 与 MPC 控制流程

### 11.1 WBC

WBC 按优先级处理：

- 足底接触约束；
- 基座姿态和高度；
- 质心位置/速度；
- 摆动足轨迹；
- 冗余关节和手臂姿态。

当前双足接触仍按两个 6D wrench 建模。QP 变量保持 18 个：

- 6 个浮动基座加速度修正；
- 12 个双足 wrench；

未提供足底尺寸的旧路径保留 22 个约束：6 个浮动基动力学等式和 16 个原有接触不等式。提供足底半长/半宽的配置化路径使用 26 个约束：6 个动力学等式和每只脚 10 个接触不等式，包括摩擦锥、法向力、yaw 力矩以及下列耦合 CoP 边界：

```text
|tx| <= foot_half_width  * fz
|ty| <= foot_half_length * fz
```

固定 roll/pitch 力矩上下限不能描述低法向力时的真实支撑多边形，因此 Lyenbot 必须使用耦合形式。双支撑卸载期间可进一步收紧未来摆脚的 `fz` 上限；进入单支撑后，非支撑脚全部 10 个不等式边界归零。

Lyenbot 单步和正式 walk 路径对左右膝位置反馈目标保留 `0.03 rad` 最小屈曲角，避免 `PosRot` 伪逆在膝穿过 0 rad 时进入直腿几何奇异。该保护属于 staged 适配层，不修改 URDF 限位（URDF 允许少量负角度），也不改变通用 PVT。

Lyenbot staged 的 WBC 输出还需要显式遵守 URDF 位置范围，因为 MuJoCo joint limit 是允许有限穿透的软约束，而现有 PVT 只读取 `minPos/maxPos`、不主动夹紧位置目标。当前适配层将全部 PVT 位置目标夹紧到 URDF 范围，并在距限位 `0.02 rad` 内衰减仅会继续向外推动的 WBC 前馈；反向恢复力矩保持不变。该保护只阻止控制主动越限，不能替代 WBC 内的 joint-limit-aware 运动学/不等式约束，也不能消除双脚闭链外力造成的软限位穿透。

普通诊断的 `tau_ff` 记录经过上述保护、实际送入 PVT 的有效前馈；安全 sidecar 的 `wbc_tau` 保留原始 WBC 输出，二者不可混用。若合法关节行程下 CoM 已超出支撑多边形，应判定参考姿态不可行，而不是通过取消限位保护恢复更晚的失败时刻。

现有 `PriorityTasks` 逐层累计的是等式任务。关节位置上下限本质上是可行域不等式，不能直接在 `static_Contact` 后添加“回到软限位边界”的高优先级等式：Lyenbot 单步实测中，即使采用连续速度阻尼激活，该组合也会在接触横移阶段形成空可行交集并使 QP 不可行。正式 joint-limit-aware 方案应采用以下边界之一：

- 在 WBC 之前，把 CoM、base 和姿态参考投影到接触约束与 URDF 关节范围的联合可行域；
- 在具备不等式能力的优化层加入 position/velocity damper，并使用显式松弛量和可观测的违反度；
- 若只能使用现有等式优先级，则限位只能用于生成可行参考，不能作为接触任务后的额外硬任务。

单步验收采用了第一条边界的离线实现：`lyenbot_posture_feasibility_test` 固定左右足世界锚点，联合扫描 base-y、base height 和 roll，再依次检查双腿 IK、URDF 腿部范围、膝最小屈曲和 CoM 相对支撑脚范围。扫描结果说明，可行集合不是原直立参考到零任务误差之间的一维线段；在线只缩放 CoM-y 或 CoM-y+roll 的投影会漏掉必须同时降低 base height 的区域。当前单步上游参考使用扫描得到的协调姿态，WBC 优先级和 QP 结构保持不变。

这一离线扫描器是参考设计诊断和验收证据，不是运行时规划器。正式 Lyenbot walk 已在上游适配层按当前左右足位置镜像生成 base-y/roll 参考，并使用固定 `0.80 m` 高度完成低速 WBC 验收；这些数值仍只属于 staged 验收基线，不是通用步态常量。后续 MPC 或更高速行走应把扫描边界或等价在线可行性求解配置化，不能把当前幅值直接扩展到所有速度。

连续步态的静态姿态可行性、闭环稳定性和视觉质量是三个不同判据。新增的 `0.25/0.35 rad` 离线扫描档位只能说明某个固定足锚点姿态可由 IK 到达；实测中把 roll 从 `0.40 rad` 降到 `0.35 rad` 仍在第二次换支撑前失稳。类似地，FootPlacement 的 `stepHeight` 只是足端峰值参数，不是独立抬膝任务：从 `0.025 m` 提高到 `0.035~0.050 m` 会改变摆动腿 IK、踝关节动态余量和落地冲击，不能仅凭脚抬得更高就判定更优。后续质量优化应分别处理轨迹端点连续性、摆动腿冗余姿态和协调 base/CoM/wrench 参考，并继续以完整闭环验收为最终依据。

连续 walk 的 base-y 参考当前按实时支撑脚世界位置生成，这是现有 FootPlacement 和接触修正闭环的一部分。单步恢复阶段可以锁存 touchdown 端点，因为随后进入固定双足恢复；连续行走中却不能直接沿用该策略。实测将 touchdown-y 跨整个后续转移固定后，支撑几何误差逐步累积并在 `31.449 s` 触发异常 QP 基座加速度。因此，若未来需要消除实时参考追随，必须同时定义落地点状态估计、支撑锚点更新时机和足滑移处理，不能只替换一个 y 数据源。

`lyenbot_staged` 的职责是验证适配层，不是替代原 `walk_wbc` 另建最终控制器。默认 `walk` 保留为已经通过的安全回归基线；`--upstream-wbc-inputs` 是显式迁移对照开关，用于在不改变默认基线的情况下恢复原 demo 的 WBC 输入语义。该开关同时注入：

```text
des_delta_q = 期望平面速度 × dt
des_dq      = 期望平面速度
des_ddq     = 5 ×（期望速度 - 实际速度）
Fr_ff       = 双脚各承担 Lyenbot 总重的一半
```

前三项在 `PriorityTasks::computeAll()` 中作为最高优先级解的种子，随后被接触、基座、摆脚和冗余任务逐层投影；不能把只桥接某一个速度字段视为原流程等价迁移。`Fr_ff` 是 QP 的名义工作点，必须按当前模型质量计算，不能复制 AzureLoong 每脚 `370 N` 的绝对值。新模式与 MPC 互斥，因为 MPC 会写回同一组字段。

完整输入组合在现有 3 秒 transfer 下已经通过 65 秒验收，但这只证明数据流兼容，不代表步态时序已经复刻。将 transfer 单独缩短到 `1.5 s` 会在第二次换支撑时失败，说明速度种子、横向 base/CoM 参考、接触 wrench 转移和状态机时序构成一个协调集合。下一步应联合迁移这一集合，而不是继续把 staged 中的单个常量逐项逼近上游值。

`--coordinated-transfer` 是上述联合迁移的下一层 opt-in 适配。它要求同时启用 `--upstream-wbc-inputs`，使用 `2.25 s` 双支撑，并以同一个余弦平滑 phase 生成 base-y、roll、可观测 CoM-y 和双脚名义 `Fr_ff`。新支撑脚在 phase 零点先承担总重 10%，避免“实测接触不足冻结 phase、零 phase 又不给载荷”的门控死锁。默认 `walk` 和 3 秒上游输入对照均不受影响。

现有 Walk 任务优先级跟踪 `base_pos_des`，不消费 `pCoMDes`；因此协调模式写入的 CoM-y 当前是规划语义和诊断量，不能宣称为新的闭环 CoM 任务。若未来要真正增加 Walk CoM 任务，必须单独审查它与 base、接触和摆脚任务的优先级可行性，不能在 staged 中隐式加入。

协调换载还确认了 `Fr_ff` 的设计边界：它是浮动基动力学 QP 的名义 wrench 工作点，不是期望实测载荷。把旧支撑脚法向力上限强制绑定到名义 phase 会在 `19.575 s` 失稳；保持约束开放、让 QP 根据动力学修正时则完成 `65.001 s/16` 次切换。计划 wrench 与实际接触闭环必须保持语义分离。

线性倒立摆捕获点可作为接触释放的必要条件，但不能单独触发释放。最终状态机至少还应同时检查摆脚完整实测载荷、支撑脚实测 CoP、关节限位余量，并保证门控等待时仍保留与双支撑一致的非零摆脚计划载荷。上述条件只有在单支撑参考已通过可行性投影后才有意义。

`lyenbot_staged` 以命名 contact box 所属 ankle-roll body 为足部归属，汇总该 body 上全部 geom 的 `mj_contactForce()`，转换到世界系并以 Pinocchio 足 Frame 计算力矩和 CoP。不能只统计命名 box，因为生成模型还保留原 URDF foot collision mesh。完整 wrench 不整体写回通用 DataBus；当前只在 Lyenbot 单步 demo 内用其法向力连续低于 `5 N` 达 `20 ms` 的条件确认释放，再通过布尔适配信号通知 WBC，避免未验收的完整坐标/符号约定扩散到通用闭环。

`Fr_ff` 和 WBC QP 输出 wrench 是浮动基动力学前馈/优化结果，不是实测接触力闭环目标。双脚同时闭链接触时，即使支撑踝前馈和最终 PVT 输出方向一致，单脚实测 roll wrench 也可能因另一只脚的接触反力而不跟随；因此不能用延长等待代替载荷闭环，也不能仅凭计划 CoP 判断可释放。后续释放条件至少需要同时检查完整实测摆脚载荷、支撑脚 CoP 以及 CoM 位置/横向速度。

单步接触切换顺序为：

```text
Walk+DSt 双足卸载
  → DSt + wbc_contact_release（QP 双足 wrench、运动学仅锁未来支撑脚）
  → 完整实测摆脚 fz < 5 N 持续 20 ms
  → LSt/RSt 正式单支撑
  → 摆脚完成原地竖直轨迹
  → 双足完整实测 fz 达阈值持续 50 ms
  → 保持 0.5 s，再用固定端点平滑恢复姿态 3 s
```

触地持续确认用于过滤首次冲击反弹；恢复过程锁存 touchdown 时刻的 CoM 和双足中心作为插值端点，避免每个控制周期重算起点导致参考漂移。只有恢复插值完成且双足接触有效时，才累计落地后稳定时间。

当前 MuJoCo 足底是原 collision mesh 与规则 box 的复合接触面，而 WBC 仍使用规则半长/半宽矩形约束。两者在横向尺寸接近，但前后范围和中心不同。接触几何所有权完成设计前，不得把规则 box 的 CoP 当作完整足部 CoP，也不得直接删除原 mesh；后者已验证会破坏 PD 站立。

正式连续 walk 的接触转移采用两级适配：GaitScheduler 的触地事件仍要求摆脚曾离地、达到最小相位并超过 touchdown 阈值；进入双支撑后，`minimumTransferContactForce` 再检查待承重脚的完整复合足 `fz`。低于门槛时冻结 `transferPhi` 而不回退状态，恢复接触后继续。该参数默认 0，因此 AzureLoong 原时间驱动 transfer 不变；Lyenbot staged 设为 `5 N`。

原 `walk_wbc` 的 1 kHz 审计进一步明确：调度器 `DSt` 只表示计划换载窗口，不等价于实测双脚接触。一个 DSt 内的物理接触集合实际是：

```text
旧支撑脚单接触 → 双脚接触 → 新支撑脚单接触
```

`minimumTransferContactForce` 当前只控制 phase 计时，不能同步改变 WBC 的 12D 静态接触和双脚 wrench 约束。直接在 demo 层把 `legState` 改成某一侧单支撑也不成立，因为该字段会联动 `static_Contact`、`SwingLeg`、接触锚点和 QP 力约束；实测多个候选均产生回归。原有 `wbc_contact_release` 是更窄的适配边界，但正式连续 walk 必须同时表达待落脚脚未确认和旧支撑脚已卸载两端，并为释放脚提供语义一致的 SwingLeg 目标、wrench 上限和滞回切换。单边释放候选已证明不足，未保留为运行功能。

FootPlacement 中原 AzureLoong 的前向/横向偏置、落地高度和相位末端下降步长已变成带旧默认值的参数。机器人专用数值只在 Lyenbot demo 注入，避免共享算法继续散布模型常量。Lyenbot 的 staged 与正式 `lyenbot_walk_wbc` 均在常规双支撑阶段暂停前向 joystick 命令，使前向落脚累积与横向承重转移解耦。该暂停不仅清零 WBC 速度种子，还必须阻止 `JoyStickInterpreter` 继续推进 `base_pos_des.x`；否则两脚刚性约束期间的 `PosRot` 任务仍会要求基座前进，并在实测载荷抢先转移时把 QP 推向接触边界。该逻辑属于 Lyenbot 入口输入适配，不修改共享 GaitScheduler 或原 AzureLoong 时序。

当前 FootPlacement 竖直 Bezier 的虚拟落地相位为 `1.4`，而 Lyenbot 通常约在 `phi=0.75` 提前触地。该差异不能只按“端点不连续”处理：实测表明，把轨迹改为 `0.85` 平滑端点后，会同时改变落地穿透参考、接触冲击、GaitScheduler touchdown 声明时机和双支撑足锚点。即使加入有界触地搜索与 `20/50 ms` 确认，仍会出现接触丢失或横向动态累积。因此未通过的参数化和状态机修改已全部撤销；后续轨迹设计必须把足端几何、接触确认和双支撑锚点作为同一个混合系统分析。

Lyenbot 单腿的 6 个关节被 6D 足端任务和基座参考共同确定，现有 Walk `RedundantJoints` 只覆盖头部/腰部，不存在可直接增加的独立“膝冗余任务”。若在线叠加膝角等式，会与摆脚位姿任务争夺同一自由度。增加可见屈膝应先离线搜索足端高度、足 pitch、base 姿态与膝/踝余量的联合可行轨迹，再选择需要放松或重排的任务，而不是把膝角直接加入当前优先级链。

离线摆腿扫描现已把每侧 6D 足端目标展开为高度、前向位移、横向外移和 pitch 四个维度，并同时检查双腿 URDF 位置余量、摆动膝屈曲和 ankle-pitch 余量。在线 `--swing-quality-candidate` 不改变优先级结构，只在 staged 适配层把扫描得到的 `35 mm/0.10 rad` 足端组合写入原 `SwingLeg` 任务；pitch 使用两段余弦平滑窗并在实测 touchdown 最小 phase `0.75` 之前的 `0.70` 回零，因此落地仍沿用原水平足姿态与接触状态机。该候选证明足姿态可用于重新分配膝踝角度，但并不提供额外自由度。

静态联合可行域仍不能直接充当在线基座规划器。`0.82 m/0.30 rad/±0.09 m` 姿态虽然具有更大的 IK 位置余量，第一次落地后的动态换载仍产生 `239.375 rad/s²` 量级的 QP 基座加速度并安全停止。低侧倾规划至少还需要把落地状态、双支撑 CoM 轨迹、实测载荷转移和基座加速度连续性加入同一约束；当前离线扫描器只负责几何候选筛选。

低侧倾日志还揭示了 FootPlacement 横向落脚与 base 状态之间的正反馈：当准静态 CoM 已接近支撑面内缘时，base 向内偏离会通过实时 hip 位置把摆脚目标继续推向外侧，摆腿质量又进一步把 CoM 拉离支撑脚。`--moderate-roll-centered-swing-candidate` 只在 staged 层禁止摆脚相对本步起点继续向外扩张，以验证这条反馈；`0.375 rad` 候选通过后证明该边界有效。它不是最终通用落脚规划器，正式方案应以支撑脚坐标系中的目标步宽、动态 CoM 和接触预测共同生成横向落脚点。

从 `0.40→0.375 rad` 可以保持 CoP 和实际落脚余量，而 `0.35 rad` 即使中心化摆脚仍会在多步后累积失稳，说明当前准静态 WBC 路径已接近可用 CoP 边界。若要得到明显更接近人体的直立躯干，下一步应由 MPC 或等价动态质心规划利用动量和预测接触，而不是继续把静态 CoM 投影强制留在单脚中心。

关节限位采用“参考余量 + 实际动态审计”的适配层边界。正式 walk 的普通关节位置参考在 URDF 边界内预留 `0.030 rad`，落脚载荷敏感的左右 ankle-pitch 预留 `0.060 rad`；余量只改变 PVT 参考，不修改机械限位或 WBC QP。`lyenbot_walk_wbc` 结束行必须汇总全部 23 关节的 position/velocity/effort 越界和最小余量。当前 65 秒控制基线的速度/力矩零越界，但 ankle-pitch 在落地冲击下仍有最多 `0.028484 rad` 的软限位穿透；提高参考余量、PVT `kd` 或直接衰减前馈都会改变闭链触地与换支撑，不能视为透明安全层。只有三类实际动态限制均零越界，才能把正式流程视为完全通过。

#### 11.1.1 原 `walk_wbc` 的适配边界

`lyenbot_staged` 已证明 Lyenbot 的模型、映射、WBC 拓扑、接触切换和关节保护能够闭环运行，但它自行组织了站立、移重心、卸载和安全验收阶段。它是适配验证脚手架，不是原 `walk_wbc` 外层控制循环已经完成适配的证据，也不应继续扩展成另一套正式控制器。

原 `demo/walk_wbc.cpp` 的正式迁移采用“双入口、同模块顺序”的边界：保留 AzureLoong 原目标和默认行为；为 Lyenbot 增加显式配置入口，仍按原 demo 的状态估计、动力学、步态调度、落脚规划、WBC、PVT 和 MuJoCo 写回顺序运行。初期不抽取大规模共享框架，先通过独立入口隔离回归风险；两条路径均稳定后再评估公共循环去重。

Lyenbot 入口允许复用 staged 已验证的适配层能力，包括名称化关节/actuator 映射、配置化 WBC 接触拓扑、复合足接触观测、URDF 关节目标保护、异常 QP 输出防护和诊断日志。不得把 staged 的完整阶段状态机作为 `walk_wbc` 的主步态规划器，也不得把其中的 `base_z/roll/stepHeight` 候选常量直接伪装成原项目默认参数。

静态审计确认 `StateEst` 的滤波和力估计维度可从 `DataBus::model_nv` 动态取得，但初始化和高度观测原先固定使用 `0.07 m` 足 Frame 离地高度。该值已通过保留旧默认值的 `state_estimation.foot_frame_ground_height` 参数化。Lyenbot 的足 Frame 是鞋底 `left/right_foot_contact_point`，因此注入 `0.0 m`；它不能与表示踝/足几何的 `gait.foot_height=0.0407 m` 混用。同时仍必须核对 `LSt/RSt` 与左右接触数组的语义，不能仅因代码可构建就判定状态估计已适配。

首轮 A/B 表明高度语义修正后，`StateEst` 的 z 估计在 WBC 接管前与 MuJoCo 只差约 `0.45 mm`，但 x 速度在 `t=3.001 s` 为 `0.126529 m/s`，与传感值约 `-0.003213 m/s` 明显不一致；默认路径随后在 `t=3.783 s` 触发基座 QP 加速度保护。跳过 estimator 状态覆盖的显式诊断模式可完成 13 秒双足站立，因此下一步只审查足端相对速度、角速度补偿、坐标系和 KF 参数，不应先改 WBC 权重。诊断模式不能作为正式验收模式。

同一真值状态路径按原 demo 在 3 秒直接令 GaitScheduler 进入 `LSt`，于 `t=3.563 s` 失稳。Lyenbot 的初始换支撑必须仍由 GaitScheduler 管理，但需增加默认关闭的初始双支撑 transfer，并由 Lyenbot 入口在该 phase 上提供一致的 base/CoM/名义 wrench 与实测接触门控。该设计属于原调度器的参数化适配，不等于迁入 staged 的整套阶段状态机。

该设计现已落地：GaitScheduler 负责 3 秒初始 `DSt` phase，入口只在该 phase 把 WBC 任务选择桥接为原 Stand 栈，使 `pCoMDes` 参与准备；完成后仍由调度器进入原 `LSt/RSt` 和 2.25 秒常规双支撑。调度器开关默认关闭，因此 AzureLoong 的直接首步语义未改变。

Lyenbot 正式入口的触地事件使用按足 body 汇总的完整 MuJoCo `fz`，并迁入已经通过 staged 闭环的中等侧倾、中心化摆腿和足 pitch 参数。FootPlacement 的固定世界落地高度是可选适配：Lyenbot 足 Frame 明确位于鞋底接触点，目标设为 `-0.010 m`；旧机器人未配置时仍使用 `base_z-legLength+offset`。这样估计器高度误差不会继续改变接触穿透参考。

高频 A/B 已明确状态边界：StateEst 的基座位置/速度在世界系表达，`Pin_KinDyn::dataBusRead()` 只在进入 Pinocchio 前把 6D 基座速度转为局部系；旋回世界系误差约 `6.7e-16`。`lyenbot_walk_wbc` 的 opt-in CSV 在该边界前后同时采样，并记录 KF `X/P/K/innovation`，不参与控制。

`Eul_W_filter` 仍是原姿态/角速度融合器。机器人配置只允许通过默认值为 `1.0` 的 `angular_velocity_measurement_noise_scale` 缩放 gyro 测量协方差；Lyenbot 使用 `0.0001` 以满足其 WBC 接管瞬态带宽，AzureLoong 未配置时保持原 Q/R。修正 `LegState` 与 `DSt` 的比较后，默认 StateEst 已在 13 秒完成 2 次支撑切换。65 秒扩展于第三次换载出现异常 QP 解，后续设计焦点转为接触锚点、任务误差和 wrench 一致性，不再把这一问题归因于世界/局部速度转换。

MPC 不是当前迁移依赖。只有 Lyenbot `walk_wbc` 在相同 headless 判据下完成 65 秒和至少 10 次支撑切换，才以该结果重新建立 MPC 对照；在此之前，staged 的 MPC 预留路径不作为当前开发主线。

### 11.2 MPC

MPC 负责较长时域的基座状态和接触力规划，WBC 负责把规划转为满足全身约束的广义加速度与关节力矩。

staged 纯 WBC 连续 10 步已经通过，但 MPC 当前暂缓。原 `walk_wbc` 的 Lyenbot 适配完成后，MPC 的第一条基线必须保持与新的正式 WBC 路径相同的低速和验收判据，先隔离 `MPC::dataBusWrite()` 对基座参考、速度和 wrench 的影响；低速性能不劣于 WBC 后，才能逐级提高速度。

## 12. AzureLoong 兼容原则

- 保留旧构造函数；
- 旧构造函数使用 AzureLoong 默认配置；
- Lyenbot 通过显式配置构造；
- Lyenbot 新状态机和足底触觉开关默认不改变 AzureLoong 行为；
- 新增步长限制的默认值保持无穷大，只有 Lyenbot demo 显式启用；
- 修改后必须至少完成 AzureLoong 目标构建回归。

## 13. GUI 资源所有权

```text
UIctr：GLFW window、MuJoCo scene/context、像素缓存、录像文件
demo：mjModel、mjData
```

`UIctr::Close()` 只请求窗口关闭；对象析构按所有权释放资源。GUI 不得删除属于 demo 的物理模型对象。

## 14. 开发约束

- 修改前说明问题、原因、方案、涉及文件和影响范围；
- 遵守最小修改原则；
- 不删除已有功能；
- 优先增加适配层；
- 所有工程记录使用中文；
- 修改后更新 `CHANGELOG.md`；
- Bug 排查更新 `DEBUG_LOG.md`；
- 新任务更新 `TODO.md`；
- Lyenbot 专项接口变化更新 `LYENBOT_ADAPTATION.md`；
- 原始 Lyenbot URDF 必须保持哈希不变。
