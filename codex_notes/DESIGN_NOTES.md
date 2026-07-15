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

## 11. WBC 与 MPC 控制流程

### 11.1 WBC

WBC 按优先级处理：

- 足底接触约束；
- 基座姿态和高度；
- 质心位置/速度；
- 摆动足轨迹；
- 冗余关节和手臂姿态。

当前双足接触仍按两个 6D wrench 建模。配置化路径的 QP 保留 18 个变量和 22 个约束的原项目结构：

- 6 个浮动基座加速度修正；
- 12 个双足 wrench；
- 6 个浮动基动力学等式；
- 16 个摩擦锥、法向力和接触力矩不等式。

### 11.2 MPC

MPC 负责较长时域的基座状态和接触力规划，WBC 负责把规划转为满足全身约束的广义加速度与关节力矩。

MPC 不应在 WBC 单步行走尚不稳定时提前调参，否则无法区分规划问题与底层接触控制问题。

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
