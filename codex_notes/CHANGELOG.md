# 修改记录

## 维护规则

本文件记录工程代码、配置、模型生成流程和测试行为的实际变更。以后每次代码修改都必须新增一条记录，不覆盖历史条目。

每条记录使用以下格式：

```text
## 日期

YYYY-MM-DD

## 修改目标

## 修改文件

- 文件路径：
- 修改函数：
- 涉及模块：

## 修改内容

### 修改1

原代码作用：

修改后：

修改原因：

影响分析：

## 设计说明

说明：

- 为什么这样修改；
- 是否存在其他方案。

## 编译测试

编译命令：

运行命令：

测试结果：

存在问题：
```

---

## 日期

2026-07-21

## 修改目标

补齐原 `walk_wbc` 在常规双支撑期间的前向输入适配，建立 65 秒连续运行基线，并补充 23 关节动态限制审计。

## 修改文件

- 代码文件：`demo/lyenbot_walk_wbc.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：`JoyStickInterpreter`、DSt 输入适配、PVT 目标保护、运行验收诊断。

## 修改内容

### 修改1：常规 DSt 默认暂停前向基座参考

原入口虽然在 DSt 清零 `des_delta_q/des_dq/des_ddq`，但 `joystick.dataBusWrite()` 仍持续推进 `base_pos_des.x`。现在沿用 staged 已验证的适配边界：初始换载完成后，只在常规 DSt 把请求速度降为 0，使用 `0.2 s` 减速参数；单支撑恢复 `0.03 m/s`。原 GaitScheduler、FootPlacement、WBC 接触状态和 AzureLoong 目标均未修改。

### 修改2：增加正式入口的动态限制审计

结束行现汇总全部 23 个实际关节的 position/velocity/effort 越界计数和最小余量，并记录最差位置关节、时刻、实际值、目标值和测得力矩。审计只读，不参与控制。

### 修改3：撤销未通过候选

三阶段 DSt 接触集、时间驱动换载、实测载荷同步换载、延长站立准备、ankle-pitch `0.070/0.080 rad` 参考余量、提高 ankle-pitch `kd`，以及限位附近前馈力矩衰减均未同时通过稳定性和动态限制验收，运行时代码已全部撤销。最终 ankle-pitch 参考余量保持 `0.060 rad`。

## 设计说明

第三次换载的直接触发因素不是必须重写 DSt 状态机，而是双脚刚性约束期间仍存在移动的前向基座位置参考。暂停该输入后，实测换载与计划参考重新保持可解，默认 StateEst 路径可连续运行 65 秒。关节动态限位是独立验收维度，不用放宽 URDF 限位或牺牲换支撑次数掩盖。

## 编译测试

- `cmake --build . --target lyenbot_walk_wbc walk_wbc lyenbot_staged model_check -j2`（在 `build/`）：通过；
- `./model_check`（在 `build/`）：输出 `MODEL_CHECK_OK`；
- `./lyenbot_walk_wbc --duration=13`：通过，`transitions=2`；
- `./lyenbot_walk_wbc --duration=65 --state-est-diagnostic=/tmp/lyenbot_walk_wbc_final_accepted_65.csv --state-est-diagnostic-period=0.01`：输出 `LYENBOT_WALK_WBC_OK time=65.001 base_z=0.806754 transitions=17`；
- 65 秒诊断共 301 列、6110 行，坏行 0；QP 最大基座加速度约 `13.6688`，最大等式/不等式残差约 `2.27e-13/3.07e-12`；
- 速度和力矩越界均为 0；位置越界 15655 个采样，最差为 `left_ankle_pitch_joint=-0.562484 rad`，相对 URDF 下限越界 `0.028484 rad`。因此连续控制基线通过，但 P0-7 动态限位总验收仍未完成。

---

## 日期

2026-07-21

## 修改目标

高频定位原 `walk_wbc` 第三次换载的 QP 基座加速度突增，扩展只读诊断，并撤销未通过的接触模式桥接候选。

## 修改文件

- 代码文件：`demo/lyenbot_walk_wbc.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：原 `walk_wbc` 入口诊断、DSt 计划状态与实测接触集合边界、WBC/QP 接触约束。

## 修改内容

### 修改1：扩展原流程的 opt-in 高频诊断

`--state-est-diagnostic` 现额外记录计划/实际送入 WBC 的支撑状态、完整实测与 WBC 结果 wrench、运动学 `ddq_final_kin`、QP 基座修正、QP 等式/不等式残差、`static_Contact/PosRot/SwingLeg` 任务误差以及接触 Jacobian 奇异值。默认不开启，不参与控制。

### 修改2：定位异常来自 QP 接触模型而非优先级运动学奇异

`t=13.083 s` 前的 1 kHz 记录显示：最终基座加速度最大值约 `19532.8`，而运动学解最大值仅约 `13.3`，异常全部来自 QP 基座修正；接触 Jacobian 最小/最大奇异值约 `0.139/2.68`，QP 等式残差约 `1.8e-11`、状态码为 0。此时双脚规划 wrench 同时逼近力/CoP/yaw 约束，说明求解器成功解出了声明接触模型下的极端解，而不是数值求解失败。

### 修改3：撤销未通过的接触桥接候选

依次验证了直接替换 WBC `legState`、带滞回的左右脚实测接触状态、按旧/新支撑区分 SwingLeg 目标，以及复用原 `wbc_contact_release` 的单边待落脚释放。候选最迟仍在 `13.095 s` 安全停止，且部分方案更早回归；相关参数和控制逻辑已全部删除。最终只保留诊断扩展，默认 13 秒基线恢复原结果。

## 设计说明

GaitScheduler 的 `DSt` 是计划换载阶段，实测接触集合在其中实际经历“旧单支撑→双接触→新单支撑”。只处理待落脚脚缺载，或在 demo 层直接改写 `legState`，都会破坏 WBC 的接触、SwingLeg、锚点和 wrench 约束配套语义。下一方案必须沿原 `wbc_contact_release` 接口完整表达三段接触序列，并保持所有配套字段一致。

## 编译测试

- `cmake --build build --target lyenbot_walk_wbc walk_wbc lyenbot_staged model_check -j2`：通过；
- `./build/model_check common/robot_configs/lyenbot.json`：通过，输出 `MODEL_CHECK_OK`；
- `./build/lyenbot_walk_wbc --duration=13`：通过，`base_z=0.817094 m`、`transitions=2`；
- 1 kHz 诊断复现默认阻塞：`t=13.083 s`、`qp_base_ddq_max=19532.767`；
- 未通过的 `wbc_contact_release` 单边候选：`t=13.095 s`、`qp_base_ddq_max=608.770`，已撤销；
- `git diff --check`：通过。

---

## 日期

2026-07-18

## 修改目标

完成原 `walk_wbc` Lyenbot 入口的高频 StateEst 审计，使默认估计器路径通过 13 秒、左右各一次摆动和 2 次支撑切换，并把后续阻塞定位到第三次换载的 WBC/QP 瞬态。

## 修改文件

- 代码文件：`algorithm/StateEst.h/.cpp`、`common/robot_model_config.h/.cpp`、`common/robot_configs/lyenbot.json`、`demo/lyenbot_walk_wbc.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：StateEst 支撑信任区、姿态/角速度滤波、StateEst 与 Pin_KinDyn 坐标边界、原流程高频 A/B 诊断。

## 修改内容

### 修改1：修正 StateEst 支撑状态语义

`getTrustRegion_wt_h()` 接收 `LegState`，原代码却与 `MotionState::Stand` 比较；两套枚举当前都从 0 编号，实际会把 `LSt` 误作站立并造成左右支撑不对称。现改为与 `DataBus::DSt` 比较，并把内部 `legState` 字段改成实际枚举类型。校正前创新 `Y-CX` 作为只读诊断量保存，滤波校正公式不变。

### 修改2：增加原流程高频状态边界诊断

`lyenbot_walk_wbc` 新增可选 `--state-est-diagnostic=<CSV>` 和 `--state-est-diagnostic-period=<秒>`。日志同时记录 MuJoCo 世界系状态、送入 Pinocchio 的状态、Pinocchio 局部速度及旋回世界系结果、StateEst `X/P/K/Y-CX/freeAcc`、接触、WBC 参考和解；默认不开启，不参与控制。实测 Pinocchio 局部速度旋回误差最大约 `6.7e-16`，排除坐标转换实现错误。

### 修改3：配置化提高 Lyenbot 角速度观测带宽

新增默认值为 `1.0` 的 `state_estimation.angular_velocity_measurement_noise_scale`，只缩放原 `Eul_W_filter` 的 gyro 测量协方差；旧配置和 AzureLoong 参数不变。Lyenbot 使用 `0.0001`，使 WBC 接管瞬态的 pitch 角速度由明显滞后恢复到接近 MuJoCo gyro，并最终使默认 StateEst 路径通过 13 秒、2 次支撑切换。

### 修改4：撤销未通过的接管扭矩渐变

曾试验 0.5 秒初始化 PD 到 WBC/PVT 的输出渐变。它把 3～3.5 秒足端速度创新峰值从约 `0.8315` 降到 `0.0131`，但真值路径在 `t=10.230 s` 回归失败，因此配置字段和扭矩混合代码已完整撤销。

## 设计说明

保留原 `StateEst → Pin_KinDyn → GaitScheduler → FootPlacement → WBC → PVT` 顺序。角速度调整仍使用原 `Eul_W_filter`，只通过机器人配置改变测量协方差；`--measured-state` 和 CSV 都是诊断旁路，不作为正式控制模式。

## 编译测试

- `cmake --build build --target lyenbot_walk_wbc model_check walk_wbc lyenbot_staged -j2`：通过；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`；
- `./build/lyenbot_walk_wbc --stand-only --duration=13`：通过，`base_z=0.848605 m`；
- `./build/lyenbot_walk_wbc --duration=13 --measured-state`：通过，`base_z=0.811143 m`、`transitions=2`；
- `./build/lyenbot_walk_wbc --duration=13`：通过，`base_z=0.817094 m`、`transitions=2`；
- `./build/lyenbot_walk_wbc --duration=65`：未通过，`t=13.083 s` 第三次换载双支撑时 `qp_base_ddq_max=19532.767`；P0-7 整体仍未完成。

---

## 日期

2026-07-18

## 修改目标

推进原 `walk_wbc` 的 Lyenbot 首次换支撑适配，使真值状态对照完成左右摆动和两次支撑切换，并把默认路径的剩余阻塞收敛到 touchdown 后的 `StateEst` 动态估计。

## 修改文件

- 代码文件：`algorithm/gait_scheduler.h/.cpp`、`algorithm/foot_placement.h/.cpp`、`algorithm/StateEst.h/.cpp`、`common/robot_model_config.h/.cpp`、`common/robot_configs/lyenbot.json`、`demo/lyenbot_walk_wbc.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：初始双支撑、协调换载、复合足接触、FootPlacement、StateEst A/B 和 WBC 模式桥接。

## 修改内容

### 修改1：由 GaitScheduler 管理初始双支撑

新增默认关闭的 `enableInitialDoubleSupportTransfer` 和独立初始/常规双支撑时长。Lyenbot 在 3 秒 WBC 接管后先执行 3 秒初始双支撑，再进入原 `LSt/RSt` 调度；落地后的常规换载使用 2.25 秒。初始阶段临时复用原 Stand WBC 任务栈，使 `pCoMDes` 真正参与横向准备，完成后恢复 Walk WBC。

### 修改2：迁入已验证的 Lyenbot 行走适配参数

配置化接入 `base_z=0.805 m`、roll `0.375 rad`、CoM/base 横向偏置、35 mm 抬脚、0.10 rad 摆脚 pitch 和禁止摆脚相对本步起点继续向外扩张。FootPlacement 新增默认禁用的固定世界落地高度，Lyenbot 使用 `-0.010 m`，避免实时基座估计误差改变落地穿透深度。

### 修改3：接入完整复合足接触

`lyenbot_walk_wbc` 按左右足 body 汇总 MuJoCo 全部 contact force，并把完整 `fz` 提供给 GaitScheduler。实测触地阈值、最小相位和双支撑门控分别使用配置值、`0.75` 和 `5 N`。原 touch 标量不再是正式入口的唯一触地依据。

### 修改4：保持和撤销边界

保留此前通过 13 秒站立的 StateEst 完整姿态重力补偿。曾试验 touchdown 持续确认和接触高度/速度硬投影，但默认 13 秒仍失败，相关构造参数、配置项和运行逻辑已全部撤销，不作为主线实现。

## 编译测试

- `cmake --build build --target lyenbot_walk_wbc walk_wbc lyenbot_staged -j2`：通过；
- `./build/lyenbot_walk_wbc --stand-only --duration=13`：通过，`time=13.001`；
- `./build/lyenbot_walk_wbc --duration=13 --measured-state`：通过，`time=13.001`、`transitions=2`；
- `./build/lyenbot_walk_wbc --duration=13`：未通过，约 `t=11.007 s` 在第二次 touchdown 后触发安全停止；
- 结论：模型、GaitScheduler、FootPlacement、WBC、PVT 和接触切换的真值状态链已越过首次换支撑；默认 StateEst 动态接触估计仍是 P0-7 阻塞，`--measured-state` 仍只作 A/B。

---

## 日期

2026-07-18

## 修改目标

建立保留原 `walk_wbc` 模块调用顺序的 Lyenbot 配置化入口，完成首轮双足 WBC 接管 A/B，并定位 `StateEst` 与初始单支撑的独立阻塞。

## 修改文件

- 代码文件：`CMakeLists.txt`、`demo/lyenbot_walk_wbc.cpp`、`algorithm/StateEst.h/.cpp`、`common/robot_model_config.h/.cpp`、`common/robot_configs/lyenbot.json`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：原 `walk_wbc` 循环、状态估计足 Frame 语义、配置加载、WBC 接管时机和 headless 诊断。

## 修改内容

### 修改1：新增 Lyenbot `walk_wbc` 独立入口

原代码作用：

`demo/walk_wbc.cpp` 全局加载 AzureLoong scene，并固定 URDF/PVT 路径、31 actuator 布局、`18/22` WBC 维度、手臂向量、步态常量和每脚 `370 N` 前馈力。

修改后：

新增 `lyenbot_walk_wbc` 目标，不修改原 `walk_wbc.cpp`。新入口按原 demo 的“仿真步进 → 传感 → StateEst → Pinocchio → GaitScheduler/FootPlacement → WBC → PVT → 扭矩写回”顺序运行，使用 `RobotModelConfig` 构造 Lyenbot 模型、23 关节映射、`18/26` WBC 和 PVT，名义足端力由模型总重 `312.327 N` 推导。支持默认 headless、`--gui`、`--stand-only` 和 `--duration=<秒>`。

修改原因：

staged 已验证适配层，但其外层状态机不能代替原项目行走 demo 的适配证据。独立入口先隔离 AzureLoong 回归风险，再逐级迁移原控制流程。

影响分析：

原 `walk_wbc` 源文件和默认目标不变；新行为只存在于 `lyenbot_walk_wbc`。

### 修改2：参数化 StateEst 足 Frame 离地高度

原代码作用：

`StateEst` 初始化和高度观测固定假设左右足 Frame 世界高度为 `0.07 m`。

修改后：

构造函数增加默认值为 `0.07 m` 的 `footHeightIn`，并新增可选配置 `state_estimation.foot_frame_ground_height`。Lyenbot 跟踪的是鞋底 `left/right_foot_contact_point`，显式设置为 `0.0 m`；`gait.foot_height=0.0407 m` 继续只表示踝/足几何，二者不混用。

修改原因：

错误注入 `0.0407 m` 会抬高状态估计的支撑面，导致 WBC 误降基座。足 Frame 的世界观测高度与踝到鞋底的机械高度是不同语义。

影响分析：

所有旧 `StateEst(dt)` 调用继续使用 `0.07 m`，AzureLoong 默认行为不变。

### 修改3：按实际接管时机初始化 full-foot WBC

原代码作用：

原 demo 在前 3 秒初始化 PD 期间也计算 WBC；AzureLoong 旧接触任务不会锁存完整双足世界位姿。

修改后：

Lyenbot 新入口仍在前 3 秒运行传感、状态估计和动力学，但到 PVT 实际接管时才首次求解 WBC并锁存 full-foot 锚点；QP 状态和输出幅值保护也只在 WBC 实际接管后启用，物理高度、姿态和有限值检查始终启用。

修改原因：

若在 `StateEst` 初始化前求解，full-foot WBC 会锁存零基座坐标下的错误足锚点，3 秒执行时立即产生异常基座加速度。

影响分析：

仅影响新 Lyenbot 入口，不改变通用 WBC 或原 demo。

### 修改4：增加状态源 A/B 诊断

新增显式 `--measured-state`，只用于跳过 `StateEst` 的状态覆盖，并打印物理/估计高度和速度。该模式用于区分状态估计与 WBC/PVT 问题，不作为正式验收路径。

## 设计说明

首轮实现没有把 staged 的完整阶段状态机复制到新 demo。A/B 证明双足配置化主链可用，同时发现默认 estimator 和原时序初始单支撑是两个独立适配项。后续先修正 `StateEst` 速度，再通过 GaitScheduler 的默认关闭参数实现初始双支撑换载。

## 编译测试

- `cmake --build build --target walk_wbc lyenbot_walk_wbc lyenbot_staged model_check -j2`：全部通过；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`，足底跨引擎误差约 `4.65e-7 m`；
- `./lyenbot_staged wbc`：回归输出 `LYENBOT_STAGE_OK time=13.001 base_z=0.848603 steps=0`；
- `./lyenbot_walk_wbc --stand-only --measured-state --duration=13`：输出 `LYENBOT_WALK_WBC_OK time=13.001 base_z=0.850663 transitions=0`；
- 默认 estimator 站立：`t=3.001 s` 的 `state_vx=0.126529 m/s`，而传感值约 `-0.003213 m/s`；于 `t=3.783 s` 触发 `qp_base_ddq_max=100.317`；
- 真值状态默认 walk：3 秒立即进入 `LSt`，于 `t=3.563 s` 触发 `qp_base_ddq_max=554.864`；
- 主 URDF SHA-256 仍为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`；
- `git diff --check`：通过。

存在问题：默认 `StateEst` 双足站立尚未通过；GaitScheduler 尚未提供 Lyenbot 初始双支撑换载；因此 P0-7 保持进行中，MPC 继续暂缓。

---

## 日期

2026-07-18

## 修改目标

根据当前开发决策暂缓 MPC，把主线从继续扩展 `lyenbot_staged` 调整为适配原项目 `walk_wbc` demo，并重新定义实施边界和验收计划。

## 修改文件

- 记录文件：`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/CHANGELOG.md`；
- 涉及模块：开发路线、`walk_wbc` 适配边界、staged 职责和 MPC 前置条件。

## 修改内容

### 修改1：将原 `walk_wbc` 适配设为当前 P0 主线

原计划：

以 staged 中等侧倾候选为 WBC 对照，直接进入 `0.03 m/s` MPC 基线。

修改后：

新增 P0-7，先保留 AzureLoong 原入口，再建立 Lyenbot 配置化 `walk_wbc` 入口，按原 demo 的模块调用顺序适配状态估计、模型维度、步态、WBC、PVT 和 MuJoCo 写回。验收分为双足站立、首次换支撑、65 秒至少 10 次切换三个阶段。

修改原因：

`lyenbot_staged` 已验证适配组件和安全边界，但其外层阶段状态机不是原 `walk_wbc` 控制循环。原 demo 仍固定 AzureLoong 模型路径、活动关节布局、WBC 维度、关节名、足底高度和固定重力前馈，不能将 staged 通过描述为原项目行走 demo 已适配。

影响分析：

本轮只修改工程记录和计划，不改变任何代码、模型、配置、运行命令或已通过的 staged 验收结果。

### 修改2：暂缓 MPC 并明确 staged 职责

`lyenbot_staged` 保留为稳定回归、安全诊断和候选参数对照，不继续叠加正式控制流程。MPC 的恢复条件改为 P0-7 完成，之后使用新的 Lyenbot `walk_wbc` 结果作为 WBC 对照。

## 设计说明

初期采用双入口隔离 AzureLoong 与 Lyenbot：保持原 `walk_wbc` 目标不变，新增 Lyenbot 配置化入口；待两条路径均稳定后再评估抽取共享循环，避免在适配尚未通过时先进行大范围重构。staged 已验证的名称映射、复合足接触、关节保护和安全诊断可作为适配层迁移，但其完整阶段状态机不作为正式步态规划器。

## 编译测试

本轮无代码、配置或模型修改，未执行编译和运行测试。已通过静态审计确认计划覆盖原 `walk_wbc` 的模型路径、WBC 维度、关节布局、控制常量和 `StateEst` 足底高度差距；既有 staged 测试结论保持不变。

存在问题：Lyenbot `walk_wbc` 代码入口尚未建立；`StateEst` 左右接触语义、初始化足底高度以及原循环中的全部 AzureLoong 常量仍需在 P0-7 实现阶段逐项验证。

---

## 日期

2026-07-18

## 修改目标

用离线 6D 足端/膝踝联合可行域替代单变量抬脚试错，并在不改变已通过基线的前提下增加可复现的摆腿质量候选。

## 修改文件

- 代码文件：`demo/lyenbot_posture_feasibility_test.cpp`、`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：离线双腿 IK、摆动足高度/前向/横向/pitch 扫描、Lyenbot staged 摆腿参考。

## 修改内容

### 修改1：扩展离线摆腿联合可行域扫描

原代码作用：

`lyenbot_posture_feasibility_test` 只扫描固定双足锚点下的 base-y、base-z 和 roll，用于准静态单支撑姿态筛选。

修改后：

保留原扫描，并新增左右对称的完整 6D 摆动足扫描。扫描高度 `0~0.060 m`、前向位移 `0~0.080 m`、外移 `0~0.040 m` 和 pitch `-0.30~0.30 rad`，记录 IK 成功率、URDF 腿部位置余量、摆动膝角和 ankle-pitch 余量；分别比较当前 `0.80 m/0.40 rad` 姿态和离线余量更大的 `0.82 m/0.30 rad` 姿态族。

修改原因：

Lyenbot 单腿 6 DoF 在固定 base 和 6D 足端任务后没有独立膝冗余，必须先证明足高、足姿态和膝踝余量可同时满足。

影响分析：

扫描器只读模型并离线运行，不改变 WBC、FootPlacement、URDF 或任何默认运行路径。

### 修改2：增加 opt-in 摆腿质量候选

原代码作用：

协调换载基线使用 `stepHeight=0.025 m` 且摆动足 pitch 始终为 0；实际摆动膝最大屈曲约 `0.576 rad`。

修改后：

新增 `--swing-quality-candidate`，要求同时启用 `--upstream-wbc-inputs --coordinated-transfer`。该候选使用 `stepHeight=0.035 m`，并在 `phi=0~0.20` 平滑增加足 pitch 至 `0.10 rad`，随后在 `phi=0.70` 前平滑回零，避免把非零 pitch 带入约 `phi=0.75` 的实测触地窗口。候选使用独立日志 `../record/lyenbot_walk_swing_quality_candidate.csv`，默认和已有对照模式均不变。

修改原因：

离线扫描表明当前姿态族中 `35 mm/0.10 rad` 组合能够提高膝屈曲并保留 ankle-pitch 余量；此前只增高到 35 mm 会出现少量 ankle-pitch 越界。

影响分析：

候选完整运行 65 秒并满足零越界，但 ankle-pitch 最小实际余量 `0.001669 rad` 小于协调换载基线的 `0.002225 rad`，因此暂作为显式质量候选，不替换默认参数。

### 修改3：撤销未通过的低侧倾在线候选

离线 `base_z=0.82 m、roll=0.30 rad、base_y≈±0.09 m` 姿态具有更大的静态关节余量，但直接接入协调换载后在 `15.888 s` 触发 `qp_base_ddq_max=239.375`。相关运行时参数和 CLI 已完全撤销，只保留失败诊断文件；后续必须联合设计落地后的动态 CoM 和实测载荷转移，不能把静态 IK 解直接当作在线轨迹。

### 修改4：增加中等侧倾与中心化摆腿候选

低侧倾失败日志表明，`0.30 rad` 姿态在第一步单支撑早期已把支撑脚 CoP-y 推到约 `-38 mm`，接近 `±40 mm` 边界；同时实时 hip/base 位置又把摆动脚目标向外推，形成 CoM/摆腿正反馈。先测试的 `0.35 rad` 中心化候选把失败延后到 `59.071 s/13` 次切换，但实际足间距最终由约 `0.21 m` 扩大到 `0.38 m`，该参数和 CLI 名称未保留。

最终新增 `--moderate-roll-centered-swing-candidate`，要求同时启用摆腿质量候选。该模式采用两端中值 `base_z=0.805 m`、`roll=0.375 rad`、CoM 内移 `0.0147 m` 和 base 外移 `0.00025 m`；单支撑中允许摆脚向支撑侧收回，但禁止相对 `swingStartPos_W` 继续向外扩张。默认日志为 `../record/lyenbot_walk_moderate_roll_centered_swing.csv`。

该候选完成 65 秒和 15 次切换。15 次 touchdown 后的左右足间距稳定在 `0.2072~0.2130 m`，未出现 `0.35 rad` 候选的累积扩张。最大绝对 roll 降至 `0.377610 rad`，摆动膝和足高改善保留，最小实际位置余量提高到 `0.012601 rad`。保留边界是最大 WBC 横向 QP 加速度为 `0.561027 m/s²`，高于仅摆腿候选的约 `0.422 m/s²`，所以该模式是视觉和关节余量折中，不代表所有动态瞬态同时改善。

## 设计说明

本轮没有新增独立膝关节任务。摆动期足 pitch 是 Lyenbot staged 适配层中的 opt-in 参考，触地前回到原来的水平足姿态。离线可行性只用于筛选候选，65 秒闭环和全部关节动态限制仍是在线保留条件。

中等侧倾候选仍属于 staged 适配层，不修改通用 FootPlacement。限制摆脚向外扩张是为了切断实时 hip/base 与落脚 y 的低侧倾正反馈；后续正式规划应以相对支撑脚的目标步宽和动态 CoM 统一生成落脚点，而不是永久依赖单边 clamp。

## 编译测试

- `cmake --build build --target lyenbot_posture_feasibility_test lyenbot_staged -j2`：通过；
- `./lyenbot_posture_feasibility_test`：左右对称扫描完成，输出 `LYENBOT_POSTURE_FEASIBILITY_SCAN_OK`；
- `./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --swing-quality-candidate`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.800118 steps=15`；
- 5601 个控制样本中 QP 全成功，最大等式残差 `3.553e-14`、最大不等式违反 `3.087e-12`，23 关节 position/velocity/effort 越界均为 0；
- 摆动膝实际/期望最大屈曲由 `0.576/0.605 rad` 提高到 `0.669/0.703 rad`，实际摆动足最高点由 `24.85 mm` 提高到 `34.38 mm`；最大绝对 roll 仍为 `0.402687 rad`；
- 低侧倾候选在 `15.888 s` 安全停止，运行时代码已撤销。
- `0.35 rad` 中心化摆腿候选在 `59.071 s` 触发 `qp_base_ddq_max=180.24`，失败参数和 CLI 名称未保留；
- `./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --swing-quality-candidate --moderate-roll-centered-swing-candidate`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.802865 steps=15`；
- 中等侧倾候选最大等式残差 `2.132e-14`、最大不等式违反 `2.746e-12`，23 关节 position/velocity/effort 越界均为 0；摆动膝实际/期望最大值 `0.674746/0.707833 rad`，实际足高 `34.52 mm`，最大绝对 roll `0.377610 rad`；最小 position/velocity/effort 余量分别为 `0.012601 rad`、`9.65941 rad/s`、`19.7544 Nm`。

存在问题：侧倾已定量降低约 6.2%，但 `0.378 rad` 仍明显大于自然动态步态；继续降低会受单支撑 CoP 余量限制，下一步应转向动态 CoM/MPC 对照，而不是继续静态缩小 roll。

---

## 日期

2026-07-16

## 修改目标

分析 Lyenbot 连续 walk 的摆脚竖直轨迹端点和膝屈曲观感，验证平滑落地端点、有限触地搜索和持续接触确认能否在不增加摆脚高度的情况下改善落地连续性。

## 修改文件

- 实验后完全恢复：`algorithm/foot_placement.h`、`algorithm/foot_placement.cpp`、`algorithm/gait_scheduler.h`、`algorithm/gait_scheduler.cpp`、`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：FootPlacement 竖直 Bezier、复合足 touchdown、双支撑锚点和摆动腿膝踝耦合。

## 修改内容

### 修改1：完成现有摆脚轨迹和膝屈曲定量诊断

原代码作用：

FootPlacement 使用峰值 phase `0.2`、虚拟落地 phase `1.4` 的 7 阶 Bezier 生成竖直轨迹；GaitScheduler 在摆脚曾离地、phase 达到 `0.75` 且实测法向力超过 `20 N` 时声明 touchdown。

修改后：

最终运行时代码保持原行为。离线和日志分析确认 Bezier 为 `B(s)=21s^5-35s^6+15s^7`；2.25 秒协调换载的实际 touchdown phase 为 `0.743333~0.759333`，在 `phi=0.75` 时 `B=0.300420`、`dB/dphi=-1.582338`，竖直参考仍有下降速度。摆动膝实际/期望最大屈曲为 `0.576341/0.604845 rad`。

修改原因：

需要区分“膝没有执行”和“足端抬升较小且被躯干侧倾掩盖”。数据表明膝关节已基本跟随足端任务；当前问题不是缺少一个未接线的膝指令。

影响分析：

不改变控制行为。后续不应在固定 base 和 6D 足端任务之外直接叠加膝角硬任务，因为 6-DoF 腿没有剩余的独立膝冗余自由度。

### 修改2：撤销未通过的平滑落地候选

原代码作用：

现有虚拟 `1.4` 端点与 `-10 mm` 落地偏置、摆脚跟踪误差和复合足碰撞共同形成约 `phi=0.75` 的提前触地。

修改后：

最终源码仍保留原轨迹。排查中测试并完全撤销：峰值/落地 phase `0.30/0.85`、相对支撑脚 `-6 mm` 落地端点、最大 `6 mm` 有界余弦触地搜索，以及 `20/50 ms` touchdown 连续确认。

修改原因：

base 相对端点会随 base-z 升高而失去触地深度；支撑脚相对端点在 `phi=1` 时实测法向力仅 `4.29 N`；加入搜索后会出现瞬时触地进入双支撑再丢失接触；50 ms 确认候选最终在 `33.503 s` 触发 `qp_base_ddq_max=101.723`。这说明曲线端点、接触声明和双支撑锚点必须联合设计。

影响分析：

失败候选未保留，不改变默认或 2.25 秒协调换载模式。下一步转为离线扫描足端高度、足 pitch、膝屈曲与 ankle-pitch 余量的联合可行域。

## 设计说明

当前视觉上的膝抬升不足不能用独立膝任务直接修正。摆脚位姿、base 姿态、复合足接触和膝踝关节余量构成同一约束集合；应先找到具备余量的轨迹族，再决定在线任务需要放松的维度。

## 编译测试

- 各轨迹候选均完成 `walk_wbc` 与 `lyenbot_staged` 编译；最终运行时代码恢复后，`cmake --build build --target walk_wbc lyenbot_staged lyenbot_posture_feasibility_test model_check -j2` 全部通过；
- base 相对平滑端点候选在 `23.811 s` 出现 QP 失败；
- 支撑脚相对 `-6 mm` 端点在 `phi=1` 时摆脚实测 `fz=4.29 N`，不能形成 touchdown；
- 有界搜索、无持续确认候选运行 65 秒但只完成 2 次切换；
- `50 ms` 持续确认候选在 `33.503 s` 触发 `qp_base_ddq_max=101.723`；
- 完全恢复运行时代码后，`./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --diagnostic=/tmp/lyenbot_walk_coordinated_after_swing_revert.csv` 输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.800002 steps=16`；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`；
- 主 URDF SHA-256 仍为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`；
- `git diff --check`：通过。

存在问题：现有轨迹实际提前触地且侧倾仍约 `0.40 rad`；需要离线联合可行域工具支持下一轮设计，P0-6 保持进行中。

---

## 日期

2026-07-16

## 修改目标

在完整上游 WBC 输入对照基础上，联合迁移更短双支撑时序、横向姿态参考和名义足端 wrench，改善连续步态换支撑节奏与横向瞬态，同时保持默认基线和 23 关节动态限位验收不变。

## 修改文件

- 代码文件：`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：Lyenbot staged 双支撑时序、横向姿态插值、名义足端力前馈、接触门控和连续步态诊断。

## 修改内容

### 修改1：增加 2.25 秒协调换载对照模式

原代码作用：

`--upstream-wbc-inputs` 已恢复原 `walk_wbc` 的完整运动种子和双脚各半重 `Fr_ff`，但仍使用 3 秒线性横向转移。单独缩短至 `1.5 s` 会在第二次换支撑后失稳。

修改后：

新增 `--coordinated-transfer`，且要求同时启用 `--upstream-wbc-inputs`。该模式使用 `2.25 s` transfer，并以同一个余弦平滑 phase 生成 base-y、roll、可观测 CoM-y 和双脚名义 `Fr_ff`；新支撑脚在双支撑开始时先获得总重 10% 的名义预载。模式使用独立默认日志 `../record/lyenbot_walk_coordinated_transfer_diagnostic.csv`，启动时输出时长和预载比例。新增适配逻辑均标记 `// LYENBOT MODIFY`。

修改原因：

1.5 秒失败日志显示约 `0.237 m` 横向基座目标、姿态和固定各半重前馈没有形成一致换载过程。先采用 2.25 秒中间档，可以在不直接逼近原项目即时换脚的情况下验证联合参考生成。

影响分析：

仅显式新参数生效；默认 `walk`、3 秒上游输入对照、MPC、公共 WBC 架构和模型均不改变。现有 Walk 任务不消费 `pCoMDes`，因此本轮 CoM-y 是规划语义和诊断量，没有隐式增加新的 WBC 任务。

### 修改2：保留名义预载，撤销实载硬约束

原代码作用：

第一版联合换载令新支撑脚名义载荷从零开始，并将旧支撑脚法向力上限绑定到计划 phase。

修改后：

最终方案让新支撑脚从总重 10% 名义预载开始，只平滑迁移 `Fr_ff`；不再按计划 phase 收紧旧支撑脚实际法向力上限，实际载荷分配仍由动力学 QP 决定。

修改原因：

零名义载荷会与 `5 N` 实测接触门控形成死锁：门控冻结零 phase，零 phase 又不能增加载荷，65 秒只完成 1 次切换。加入预载后解除死锁，但强制旧支撑脚卸载仍在 `19.575 s` 触发 `qp_base_ddq_max=109.813`；撤掉硬上限后完成完整验收。这证明 `Fr_ff` 是 QP 名义工作点，不是实测载荷闭环目标。

影响分析：

两个失败候选均未保留。最终 opt-in 模式只改变名义工作点，不扩展通用接触约束，不把未验收的实测 wrench 闭环写入 WBC。

## 设计说明

本轮继续通过 staged 适配层迁移原项目数据流，没有重写最终控制器。协调模式把“姿态 phase”和“名义 wrench phase”对齐，但明确保留计划 wrench、QP 优化 wrench 和实测接触 wrench 三者的语义边界。

## 编译测试

- `cmake --build build --target walk_wbc lyenbot_staged lyenbot_posture_feasibility_test model_check -j2`：全部通过；
- 在 `build/` 运行 `./lyenbot_staged walk --upstream-wbc-inputs --coordinated-transfer --diagnostic=/tmp/lyenbot_walk_coordinated_transfer_2250_nominal.csv`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.800002 steps=16`；
- 5601 个样本中 QP 全成功，最大等式残差 `3.553e-14`、最大不等式违反 `2.975e-12`，23 关节实际 position/velocity/effort 零越界；最小余量分别为 `0.002225 rad`、`10.13335 rad/s` 和 `24.25641 Nm`；
- 相对 3 秒上游输入对照，最大实测横向加速度由 `5.829` 降到 `4.665 m/s²`，最大 WBC 横向 QP 加速度由 `0.626` 降到 `0.457 m/s²`，65 秒前进距离由 `0.1318` 增到 `0.1693 m`；
- 零预载候选稳定停在双支撑且只有 1 次切换；旧支撑脚硬法向力上限候选在 `19.575 s` 安全停止，两者均未保留；
- 默认模式回归输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.799837 steps=12`；
- 单独传入 `--coordinated-transfer` 时按预期拒绝运行，提示必须同时启用 `--upstream-wbc-inputs`；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`；
- 主 URDF SHA-256 仍为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`；
- `git diff --check`：通过。

存在问题：最大 roll 仍约 `0.4025 rad`，摆动腿膝屈曲和抬脚观感尚未改善；P0-6 保持进行中。

---

## 日期

2026-07-16

## 修改目标

在不改变已通过默认连续步态基线的前提下，增加原 `walk_wbc` WBC 输入组合的显式迁移对照模式，验证当前 Lyenbot 适配层能否接收完整运动种子和名义足端重力前馈，并确定后续恢复原项目时序的联合修改边界。

## 修改文件

- 代码文件：`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：staged 连续 WBC 输入适配、名义足端力前馈、步态迁移诊断和默认基线回归。

## 修改内容

### 修改1：增加完整上游 WBC 输入对照模式

原代码作用：

默认 `lyenbot_staged walk` 将 `des_delta_q/des_dq/des_ddq` 和 `Fr_ff` 保持为零，主要验证 Lyenbot 接触切换、WBC QP、PVT 和关节动态限位；它没有复现原 `walk_wbc` 同时提供运动种子与名义接触力的输入语义。

修改后：

新增 opt-in 参数 `walk --upstream-wbc-inputs`。该模式按原 demo 的组合关系生成 `des_delta_q/des_dq/des_ddq`，并根据 MuJoCo 模型总重计算名义足端前馈：Lyenbot 总重 `312.327 N`，双脚各承担 `156.164 N`。对照模式使用独立默认诊断文件 `../record/lyenbot_walk_upstream_inputs_diagnostic.csv`，并禁止与尚未验收的 `mpc` 模式同时启用。新增适配逻辑已标记 `// LYENBOT MODIFY`。

修改原因：

此前只桥接局部速度字段不能代表原项目 WBC 输入链，且已被验证会破坏当前协调姿态。完整输入组合必须作为一个显式、可回退的对照项验证，不能直接覆盖默认安全基线；AzureLoong 的固定每脚 `370 N` 也不能照搬到质量不同的 Lyenbot。

影响分析：

行为变化只在显式传入 `--upstream-wbc-inputs` 时生效。默认 `walk`、单步、站立、原 `walk_wbc` 以及 MPC 路径不变；该入口用于继续迁移原项目控制流程，不作为独立最终控制器。

### 修改2：验证并撤销单独缩短支撑转移时间

原代码作用：

当前连续基线使用 `3.0 s` 双支撑转移，并在转移期间暂停前向速度。

修改后：

最终代码继续保留 `3.0 s`。排查中曾只把上游输入对照模式的转移时间缩短到 `1.5 s`，该候选在 `19.357 s` 因基座 QP 加速度超过安全阈值停止，已完全撤销。

修改原因：

结果表明原项目的运动种子、横向 base/CoM 参考、接触 wrench 转移和状态机时序必须成套迁移，单独缩短时间会在换支撑阶段制造不一致。

影响分析：

失败候选未进入最终源码。下一步将以已通过的完整输入对照模式为基线，联合迁移横向参考、wrench 和较短转移时序。

## 设计说明

`lyenbot_staged` 仍是 Lyenbot 适配验收脚手架：默认路径负责稳定回归，opt-in 路径负责逐项接回原 OpenLoong 数据流。此次结果只证明完整上游 WBC 输入能够通过当前适配层，不代表当前保守横移和摆脚观感已经复现原项目。

## 编译测试

- `cmake --build build --target walk_wbc lyenbot_staged lyenbot_posture_feasibility_test model_check -j2`：全部通过；
- 在 `build/` 运行 `./lyenbot_staged walk --upstream-wbc-inputs --diagnostic=/tmp/lyenbot_walk_upstream_inputs.csv`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.799714 steps=12`；
- 对照模式 5601 个样本中 QP 全成功，最大等式残差 `3.553e-14`、最大不等式违反 `3.491e-12`，全部 23 关节实际 position/velocity/effort 零越界；
- 对照模式 `des_dq_x` 峰值为 `0.013036 m/s`，说明输入链已恢复，但仍受当前 3 秒转移和暂停策略限制；
- `1.5 s` 转移候选在 `19.357 s` 触发 `qp_base_ddq_max=109.551` 安全停止，已撤销；
- 默认模式回归 `./lyenbot_staged walk --diagnostic=/tmp/lyenbot_walk_default_after_upstream_mode.csv`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.799837 steps=12`；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`；
- `./lyenbot_staged walk mpc --upstream-wbc-inputs`：按预期拒绝互斥模式并输出 `mpc and --upstream-wbc-inputs cannot be enabled together`；
- 主 URDF SHA-256 仍为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`；
- `git diff --check`：通过。

存在问题：横向姿态、CoM、wrench 与原项目步态时序尚未联合迁移，P0-6 保持进行中。

---

## 日期

2026-07-16

## 修改目标

针对 GUI 中躯干侧倾明显、摆脚和膝关节抬升不明显的问题，补全姿态可行域诊断并确定连续 WBC 步态质量调参的安全边界；不降低既有 65 秒稳定性和 23 关节动态限位验收标准。

## 修改文件

- 代码文件：`demo/lyenbot_posture_feasibility_test.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：离线姿态可行域扫描、连续 walk 横向参考、FootPlacement 摆脚高度和关节动态限位验证。

## 修改内容

### 修改1：补全离线 roll 扫描档位

原代码作用：

`lyenbot_posture_feasibility_test` 扫描 `0.05/0.10/0.15/0.20/0.23/0.30/0.40 rad` 的协调姿态。

修改后：

增加 `0.25/0.35 rad` 两档只读扫描输出，用于比较当前 `0.40 rad` 稳定姿态与较小侧倾候选的 base-y、base-z、CoM 相对支撑脚位置和可行裕量。

修改原因：

用户观察到连续 walk 躯干侧倾明显，需要先确认中间 roll 档位的静态可行域，再做闭环候选验证。

影响分析：

只影响独立诊断程序的输出覆盖，不修改运行时控制器、模型或已通过基线。

### 修改2：撤销未通过的连续步态候选

原代码作用：

稳定基线使用 `base_z=0.80 m`、`roll=0.40 rad`、实时支撑脚 y、`stepHeight=0.025 m` 和 ankle-pitch 参考余量 `0.060 rad`。

修改后：

运行时最终仍保持上述已通过参数。排查期间测试并撤销：`roll=0.35 rad` 协调姿态、touchdown-y 锁存、`stepHeight=0.035/0.040/0.050 m`，以及 `0.065/0.070 rad` ankle-pitch 参考余量。

修改原因：

这些候选分别导致约 19~31 秒或 61.1 秒安全停止，或者虽跑满 65 秒但仍有 ankle-pitch 实际位置越界，不能替换零越界基线。

影响分析：

失败候选未保留在 `lyenbot_staged`。本轮确定后续应从摆脚轨迹端点连续性、膝关节冗余姿态和协调 base/CoM/wrench 参考入手，不再单独放大高度或缩小 roll。

## 设计说明

离线固定足锚点 IK 可行不等于连续换支撑闭环稳定；足端峰值提高也不等于抬膝任务得到改善。质量候选仍必须同时满足 65 秒运行、QP 残差、接触切换以及全部 23 关节实际 position/velocity/effort 零越界。

## 编译测试

- `cmake --build build --target lyenbot_posture_feasibility_test -j2`：通过；
- `./build/lyenbot_posture_feasibility_test common/robot_configs/lyenbot.json`：通过，新增 `0.25/0.35 rad` 档位正常输出；
- 恢复原运行参数后，在 `build/` 运行 `./lyenbot_staged walk --diagnostic=/tmp/lyenbot_walk_baseline_recheck.csv`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.799837 steps=12`；
- `stepHeight=0.040 m` 候选：65 秒、12 次切换、QP 全成功，但 25 个 right ankle-pitch 位置越界样本，最大越界 `0.001984 rad`；
- `stepHeight=0.035 m` 候选：65 秒、12 次切换、QP 全成功，但 3 个 left ankle-pitch 位置越界样本，最大越界 `0.000932 rad`；
- `stepHeight=0.050 m` 候选：`61.100 s` 触发 `qp_base_ddq_max=399.545` 安全停止；
- `cmake --build build --target walk_wbc lyenbot_staged lyenbot_posture_feasibility_test model_check -j2`：全部通过；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`；
- 主 URDF SHA-256 仍为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`；
- `git diff --check`：通过。

存在问题：当前稳定步态的躯干侧倾和摆腿观感尚未改善，P0-6 保持进行中。

---

## 日期

2026-07-16

## 修改目标

完成 Lyenbot 低速纯 WBC 连续 10 步验收，并保证完整接触切换、QP 输出和 23 关节 URDF position/velocity/effort 限制均满足判据。

## 修改文件

- 代码文件：`algorithm/gait_scheduler.h`、`algorithm/gait_scheduler.cpp`、`algorithm/foot_placement.h`、`algorithm/foot_placement.cpp`、`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：纯 WBC/MPC 隔离、协调步态参考、FootPlacement 参数化、复合足接触转移、关节动态限位和连续步态验收。

## 修改内容

### 修改1：隔离禁用态 MPC 并迁移协调步态参考

原代码作用：

`walk` 即使未启用 MPC 也周期调用 `MPC::dataBusWrite()`，禁用态结果会覆盖 joystick 和步态路径产生的 `base_pos_des/des_dq`；正式 walk 也没有采用单步已验证的 base-y/base height/roll 协调原则。

修改后：

仅在 `enableMpc` 时运行 MPC 计算和写回。Lyenbot walk 在 10~13 秒平滑进入协调左支撑姿态，13 秒后按当前支撑足镜像生成 base-y 和 roll，保持 `base_z=0.80 m`；纯 WBC 限速 `0.03 m/s`，双支撑转移期间暂停前向命令。验收窗口由 40 秒延长到 65 秒，以覆盖至少 10 次完整支撑切换。

修改原因：

禁用模块不得改写启用控制链的数据；按当前 `1.5 s` 摆动和 `3 s` 双支撑周期，原 27 秒有效行走窗口理论上只能覆盖约 6 步。

影响分析：

行为变化仅限 Lyenbot staged walk。MPC 启用路径仍保留，下一阶段单独验收。

### 修改2：参数化 FootPlacement 的机器人专用常量

原代码作用：

FootPlacement 内部硬编码 AzureLoong 的 `-0.07 m` 前向偏置、`+0.04 m` 横向内收、`-0.035 m` 落地高度和相位末端每周期 `-0.002 m` 下降量。

修改后：

新增 `forwardOffset`、`inwardOffset`、`landingHeightOffset` 和 `lateTouchdownStretchStep` 公共参数，默认值保持原行为。Lyenbot 显式设置为 `0.0/-0.04/-0.010/0.0`，同侧最大步长设为 `0.10 m`。

修改原因：

原偏置使 Lyenbot 前进落脚后退且收窄站距，末端逐周期下降还会在触地未被识别时形成无界参考。

影响分析：

通用默认值不变；Lyenbot 数值集中在适配层并标记 `// LYENBOT MODIFY`。

### 修改3：使用复合足接触门控双支撑转移

原代码作用：

GaitScheduler 一旦检测到瞬时 touchdown 就按固定时间推进 transfer；Lyenbot 规则 contact box 的标量可能漏掉 collision mesh 承载，也无法在新支撑脚反弹失去接触时暂停横移。

修改后：

Lyenbot walk 在调度前把按全部足部 geom 汇总的完整法向力写入 `fL[2]/fR[2]`。GaitScheduler 新增默认值为 0 的 `minimumTransferContactForce`，Lyenbot 设为 `5 N`；待承重脚低于门槛时冻结 `transferPhi`，恢复后继续。最早触地相位使用 `0.75`。

修改原因：

第二次落脚曾在瞬时触地后失去左脚接触，但时间驱动参考继续向左移动，最终出现异常大基座加速度。接触门控使声明状态与实测承载重新一致。

影响分析：

参数默认关闭，不改变 AzureLoong；Lyenbot 只使用已校验符号的完整法向力，尚未把未经完整一致性验收的 6D wrench 扩散到通用 WBC。

### 修改4：完成连续模式关节动态限位

原代码作用：

位置目标只夹在 URDF 边界上，MuJoCo compliant joint 在第一次落脚时仍使左 ankle-pitch 最多穿透下限 `0.010266 rad`。

修改后：

walk 保留双膝 `0.03 rad` 最小屈曲；普通关节 PVT 参考在 URDF 边界内预留 `0.030 rad`，左右 ankle-pitch 预留 `0.060 rad`，并继续衰减仅会向越限方向推动的 WBC 前馈。

修改原因：

落脚时支撑踝承受约 `303 N`，需要在软物理限位前留出闭环恢复空间。两侧专用余量仍远小于 ankle-pitch 总行程，且最终 effort 余量充足。

影响分析：

只影响 Lyenbot walk 的 PVT 参考，不修改 URDF、MuJoCo joint range 或共享 WBC QP。

## 设计说明

本轮继续保留 OpenLoong 的 GaitScheduler、FootPlacement、WBC 和 PVT 主流程，只在共享模块增加带原默认值的参数，并由 Lyenbot staged 适配层注入机器人专用数值。曾尝试直接桥接 `base_vel_des` 到 WBC 增量任务、触地前连续确认 50 ms，以及不成套迁移的 `0.82 m/0.30 rad` 静态姿态；分别因任务语义错误、切换过晚和横向参考不一致而撤销。

## 编译测试

- `cmake --build build --target lyenbot_staged -j2`：通过；
- `cmake --build build --target walk_wbc lyenbot_staged -j2`：原项目 `walk_wbc` 与 Lyenbot 目标均构建通过；
- `./build/model_check common/robot_configs/lyenbot.json`：输出 `MODEL_CHECK_OK`，`nq=30`、`nv=29`、23 个活动关节、足端误差约 `4.65e-7 m`；
- 在 `build/` 运行 `./lyenbot_staged walk --diagnostic=/tmp/lyenbot_walk_10step_ankle_margin.csv`：输出 `LYENBOT_STAGE_OK time=65.001 base_z=0.799837 steps=12`；
- 5601 个诊断样本中 QP 全部成功，最大等式残差 `1.137e-13`、最大不等式违反 `2.574e-12`，最小基座高度 `0.797363 m`，最大绝对 roll/pitch `0.410503/0.007819 rad`；
- 全部 23 关节实际 position/velocity/effort 无越界，最小位置、速度和 effort 余量分别为 `0.002458 rad`、`10.1331 rad/s`、`24.1835 Nm`；
- `git diff --check`：通过。

存在问题：MPC 尚未建立 `0.03 m/s` 等速基线；复合足完整 6D wrench 与 WBC 矩形 CoP 的自动一致性仍未验收。

---

## 日期

2026-07-16

## 修改目标

在 URDF 合法关节行程内生成可用单支撑参考，完成 Lyenbot 左右准静态原地单步的释放、腾空、触地和恢复站立验收。

## 修改文件

- 代码文件：`CMakeLists.txt`、`demo/lyenbot_posture_feasibility_test.cpp`、`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 实验后完全回退文件：`common/data_bus.h`、`algorithm/wbc_priority.h`、`algorithm/wbc_priority.cpp`；
- 涉及模块：单支撑姿态可行性、Lyenbot 单步参考、完整实测接触确认和落地恢复状态机。

## 修改内容

### 修改1：增加离线准静态姿态可行域扫描器

原代码作用：

单步参考保持基座直立，仅把 CoM 横向移动到支撑脚附近；加入 URDF 位置目标保护后，13 秒时 CoM 仍在支撑脚中心外约 `46~47 mm`，没有合法单支撑裕量。

修改后：

新增独立 headless 目标 `lyenbot_posture_feasibility_test`。工具固定初始左右足世界锚点，联合扫描 base-y、base height 和支撑侧 roll，并检查双腿 IK、URDF 腿部位置限位、双膝最小 `0.03 rad` 屈曲及 CoM 支撑范围。每侧扫描 `15457` 个样本，左/右侧分别得到 `546/558` 个支撑合法候选。

修改原因：

严格等式 joint-limit 任务会与接触任务形成不可行交集；在线只缩放 CoM-y 或 CoM-y+roll 也不能覆盖需要同时降低基座的非一维可行域。独立扫描可以先回答“机器人几何上是否存在合法单支撑姿态”，且不改变 OpenLoong WBC 主架构。

影响分析：

新增目标只读模型并输出诊断，不进入运行时控制链，不影响 AzureLoong 和现有 demo。

### 修改2：单步采用协调可行姿态

原代码作用：

`8~11 s` 只横移 CoM，基座保持额定高度和零 roll；摆脚释放后实际 CoP 无法追上外移 CoM，左右均在约 1 秒内横向发散。

修改后：

Lyenbot `step-left/right` 在 `8~11 s` 同时平滑调整 CoM-y、base height 和支撑侧 roll。当前验收参考使用 `base_z=0.80 m`、roll `±0.40 rad`，CoM 目标为支撑脚中心向中线内移 `4 mm`；touchdown 后再返回额定高度和零 roll。相关逻辑均标记 `// LYENBOT MODIFY`。

修改原因：

扫描表明最小 roll `0.05 rad` 的候选关节余量只有 `0.000713 rad`；`0.15 rad` 候选虽把左单支撑延长到约 `1.618 s`，CoP 仍会到达脚边缘。`0.40 rad`、`0.80 m` 组合能够把 CoM 带到支撑脚中心附近并形成动态裕量。

影响分析：

仅修改 `isSingleStep(stage)` 分支；普通 `pd/wbc/walk` 和 OpenLoong 控制流程不变。该固定幅值只作为准静态验收参考，连续 walk 迁移时需动态生成并配置化。

### 修改3：消除触地反弹误判并固定恢复端点

原代码作用：

首次触地冲击即可结束单支撑，恢复参考又可能在瞬态中缺少固定起点，导致已经完成抬落的候选在恢复阶段再次失稳。

修改后：

摆脚确认离地至少 `2 s` 后，要求左右复合足完整实测 `fz` 同时超过 touchdown 阈值并持续 `50 ms` 才确认落地。确认时锁存 CoM 和双足中心；先保持落地姿态 `0.5 s`，再以固定端点在 `3 s` 内平滑恢复 roll、base height 和 CoM-y。恢复完成且双足接触有效后才累计稳定时间，单步总时长相应延长到 24 秒。

修改原因：

把首次冲击反弹与稳定复合足接触分离，并避免移动插值起点导致恢复参考漂移。

影响分析：

只影响 Lyenbot 单步验收状态机；完整 wrench 仍未整体写入通用 DataBus。

## 设计说明

本轮保留 OpenLoong 的 WBC 任务优先级和 QP 结构，把机器人特定的单支撑可行性放在上游参考适配层。曾验证在线标量投影、CoM-y+roll 投影和 `0.02 rad` 软余量投影，左脚分别约在 `14.080/14.043/13.877 s` 停止；这些候选及临时 DataBus/WBC 字段均已完全撤销。结果证明可行参考需要多维协调，不能用原参考的一维缩放代替。

## 编译测试

- `cmake --build build --target lyenbot_posture_feasibility_test -j1`：通过；
- `./build/lyenbot_posture_feasibility_test common/robot_configs/lyenbot.json`：左右均输出 `LYENBOT_POSTURE_FEASIBILITY_SCAN_OK`，支撑合法样本分别为 `546/558`；
- `cmake --build build --target lyenbot_staged -j1`：通过；
- `./lyenbot_staged step-left --diagnostic=/tmp/lyenbot_step_left_scanned_posture_recovery.csv`：输出 `LYENBOT_SINGLE_STEP_OK`，`release_time=13.028 s`、`stable_duration=4.870 s`、`max_stance_slip=0.0000643348 m`；
- `./lyenbot_staged step-right --diagnostic=/tmp/lyenbot_step_right_scanned_posture_recovery.csv`：输出 `LYENBOT_SINGLE_STEP_OK`，`release_time=13.027 s`、`stable_duration=4.867 s`、`max_stance_slip=0.0000615915 m`；
- 普通 `wbc` 同轮回归：输出 `LYENBOT_STAGE_OK time=13.001 base_z=0.848603 steps=0`；
- 主 URDF SHA-256 复核为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`，与生成清单一致，原始 URDF 未修改；
- 当前完成 P0-2 和 P0-4 的准静态验收；连续 WBC 10 步仍未验证，转入 P0-5。

---

## 日期

2026-07-16

## 修改目标

验证 joint-limit-aware 高优先级任务和捕获点释放门控能否解决合法关节行程下的首步横向失稳。

## 修改文件

- 实验文件：`algorithm/wbc_priority.h`、`algorithm/wbc_priority.cpp`、`demo/lyenbot_staged.cpp`；
- 记录文件：`codex_notes/CHANGELOG.md`、`codex_notes/DEBUG_LOG.md`、`codex_notes/DESIGN_NOTES.md`、`codex_notes/TODO.md`、`codex_notes/LYENBOT_ADAPTATION.md`；
- 涉及模块：WBC 运动学优先级、单步双支撑卸载和接触释放条件。

## 修改内容

### 修改1：排除高优先级等式型关节限位任务

曾在 `static_Contact` 之后加入基于 URDF 上下限的 ankle-roll 等式型限位任务。右踝进入软限位区后，`step-left` 在 `9.726 s`、仍处于 Stand 横移阶段时出现 `qp_status=-2`；将限位激活改为连续速度阻尼后仍在相同时刻失败。说明当前接触等式与额外关节等式没有可行交集，问题不是权重或开关不连续。该候选已完全撤销，WBC 公共源码不保留本轮实验逻辑。

### 修改2：排除仅靠捕获点门控和两段卸载的时序方案

曾使用线性倒立摆捕获点 `y_cp=y_com+y_dot/omega`，要求其位于支撑脚半宽减 `5 mm` 的范围内才允许释放。固定 13 秒门槛会阻止释放，但继续把摆脚计划载荷降为零后仍在 `19.388 s` 失稳；把支撑目标内移量由 `10 mm` 改为 `25 mm` 也未改善。将卸载改为“释放前最多 50%，释放后再完成 50%”可安全运行到 20 秒，但没有抬脚，不能通过验收。把最早放行时刻提前到 `12.5 s` 后，左脚在 `12.501 s` 请求释放、`12.879 s` 确认离地并进入单支撑，仍于 `13.728 s` 横向失稳。

这些候选均已撤销。结果证明捕获点可以作为后续安全条件，但不能替代合法关节行程内的单支撑参考可行性；继续调整固定时刻只会阻止动作或平移失败。

## 设计说明

当前优先级运动学任务以等式形式累计。把关节限位直接作为接触任务后的高优先级等式，会把本来应是可行域边界的问题变成额外硬目标。后续应在进入 WBC 前把 CoM/base/姿态参考投影到关节与接触共同可行域，或在支持不等式的优化层加入带松弛量的 joint-limit 约束。捕获点、实测摆脚载荷和支撑 CoP 应作为释放条件，而不是用来掩盖不可行参考。

## 编译测试

- `cmake --build build --target lyenbot_staged -j1`：各候选及最终回退版本均通过；
- 严格/平滑 joint-limit 等式任务：均在 `9.726 s` 使 `step-left` QP 不可行，已撤销；
- 13 秒捕获点门控：未释放，`19.388 s` 安全停止；
- 50% 两段卸载：运行至 20 秒但未释放，验收失败；
- 12.5 秒捕获点放行：`12.879 s` 确认离地，`13.728 s` 安全停止；
- 恢复有效基线后重新运行 `step-left`：`13.306 s` 确认释放、`14.239 s` 安全停止，与修改前精确一致；P0-4 仍未通过。

---

## 日期

2026-07-16

## 修改目标

补齐 Lyenbot staged 的 URDF 关节位置目标和越限方向前馈保护，确认准静态横移是否依赖 ankle-roll 越限姿态。

## 修改文件

- 文件路径：`demo/lyenbot_staged.cpp`
- 涉及模块：Lyenbot WBC 输出安全适配、单步关节限位诊断

## 修改内容

发现原有效单步基线在释放前已要求左右 ankle-roll 约 `-0.186~-0.189 rad`，实际约 `-0.147~-0.162 rad`，而 URDF/MuJoCo 范围仅为 `±0.1326 rad`。原 `14.517/14.494 s` 基线因此依赖越限姿态，不能作为满足验收条件的有效结果。

Lyenbot staged 的 WBC 输出适配层现做两级保护：

- 将全部 PVT 位置目标夹紧到 Pinocchio 从 URDF 读取的 `motorMinPos/motorMaxPos`；
- 在距位置上下限 `0.02 rad` 的软区间内，仅线性衰减会继续向限位外推动的 WBC 前馈力矩；到达或越过限位时该方向前馈为零，反向恢复力矩不受影响。

该保护不修改共享 PVT、WBC QP 或 AzureLoong 路径，也不替代正式 joint-limit inequality。普通诊断中的 `tau_ff` 现在表示进入 PVT 前、经过限位保护后的有效前馈；原始 WBC 力矩仍可在安全 sidecar 的 `wbc_tau` 中查看。

本轮还试验并撤销了三项候选：状态驱动的 Stand 脚中心预释放在 `12.656 s`、尚未满足释放条件时产生运动学/QP 尖峰；卸载阶段 `0.05 rad` 躯干侧倾在 `11.988 s` 产生 `4219.43` 的 QP 基座加速度；提前把未来摆脚从双足硬接触降为普通 SwingLeg 保持后，CoM 裕量没有改善。这些结果共同说明不能继续通过固定姿态或松弛时序扫参绕过关节行程约束。

## 编译测试

- `cmake --build build --target lyenbot_staged -j1`：通过；
- `wbc`：运行至 `13.001 s`，`base_z=0.848603 m`，通过；
- `step-left`：`13.306 s` 确认释放，`14.239 s` 安全停止；
- `step-right`：`13.303 s` 确认释放，`14.247 s` 安全停止；
- 左右在 13 秒时的 CoM 均已位于支撑脚中心外约 `46~47 mm`，超过 `40 mm` 足底半宽，表现镜像一致；
- MuJoCo 软限位仍允许闭链外力造成约 `10~14 mrad` 短时穿透，但控制前馈已转为恢复方向。P0-4 仍未通过。

---

## 日期

2026-07-16

## 修改目标

确认双支撑到单支撑阶段的 CoP 建立滞后是否来自时序或 PVT 力矩抵消，并补齐关节前馈力矩诊断。

## 修改文件

- 文件路径：`demo/lyenbot_staged.cpp`
- 涉及模块：Lyenbot 单步时序试验、100 Hz 关节力矩诊断

## 修改内容

普通诊断 CSV 在每个关节的 `q_des/q/dq_des/dq/tau_out` 基础上增加 `tau_ff`，记录 WBC 写入 PVT 的前馈关节力矩。CSV 由 248 列扩展为 271 列，可由 `tau_out-tau_ff` 得到 PVT 位置/速度反馈与滤波的合力矩；控制计算不变。

试验过两项单步时序候选，均未保留：

- 在 13~14 秒保持最终双支撑卸载状态、把接触释放延后到 14 秒。保持期内摆脚实测载荷仅由约 `42 N` 降至 `39 N`，支撑脚实测 CoP 仅由约 `-0.7 mm` 到 `2.8 mm`，CoM 相对支撑脚反而由约 `25.9 mm` 漂到 `28.0 mm`；失败从 `14.517 s` 等量平移到 `15.520 s`，故恢复 13 秒释放。
- 把动态接触释放提前到 11.5 秒，使 wrench 卸载与抬脚重叠。此时支撑脚实际 CoP 尚在反向边缘约 `-42 mm`，释放于 `11.914 s` 完成后，机器人在 `12.847 s` 更早停止；故恢复顺序卸载基线。

新增力矩分解证明 PVT 未抵消 WBC 的支撑脚 ankle-roll 前馈。`step-left` 在 13.0 秒的右踝 roll 前馈约 `-11.74 Nm`，PVT 反馈约 `-3.83 Nm`，最终输出约 `-15.58 Nm`；双支撑时右脚实测 `tx` 仍仅约 `-0.19 Nm`，而摆脚离地后才追到约 `11.5~11.7 Nm`。因此计划 wrench 在双脚闭链接触中只是动力学前馈，不构成实测接触力闭环。

## 编译测试

- `cmake --build build --target lyenbot_staged -j1`：通过；
- 恢复基线后的 `step-left`：`13.301 s` 确认释放、`14.517 s` 安全停止，与修改前一致；
- `wbc`：运行至 `13.001 s`，`base_z=0.848603 m`，验收通过；
- P0-4 仍未通过，下一步转向完整实测 wrench 的双支撑载荷闭环与基于 CoM/CoP 裕量的释放条件。

---

## 日期

2026-07-16

## 修改目标

定位并消除右脚单步在双支撑卸载阶段的毫秒级加速度尖峰。

## 修改文件

- 文件路径：`demo/lyenbot_staged.cpp`
- 涉及模块：Lyenbot 单步安全诊断、膝关节奇异保护

## 修改内容

新增只在 `step-left/step-right` 中启用的 1 kHz 安全环形追踪。内存仅保留最近 `100 ms`，正常运行不新增高频文件；触发安全停止时写出 `<diagnostic>.safety.csv`，包含完整 `q/dq`、优先级运动学与 QP `ddq`、各 Walk task 的目标关节加速度、规划/参考/实测 wrench、QP 状态/残差和电机力矩。普通 248 列、100 Hz CSV 不变。追踪表头使用确定维数并逐行检查列数，WBC 尚未启用的 0~3 秒不采样。

高频证据表明右脚旧基线在 `11.293 s` 的 `kin_ddq_9` 达到 `4040.64`，对应 `left_knee_pitch_joint`；该尖峰首次出现在 `PosRot` 任务层，`static_Contact` 层约为零，QP 只对浮动基前 6 维做修正。左膝实际角度同时由约 `0.041 rad` 伸直到 `-0.001 rad`，证明 QP/接触力尖峰是膝关节穿过 0 rad 几何奇异后的结果。

单步适配层现将左右膝 `motors_pos_des` 的最小屈曲角限制为 `0.03 rad`。该值避开 0 rad 奇异点，同时比曾试验的 `0.10 rad` 更少改变原左脚支撑几何。锁存切换瞬间 `rpy` 的候选使右脚失败提前到 `11.246 s`，已撤销。

## 编译测试

- `lyenbot_staged` 增量构建通过；
- `step-right`：旧 `11.294 s` 双支撑尖峰消失，`13.298 s` 完成释放确认，至 `14.494 s` 才因单支撑横向发散停止；
- `step-left`：`13.301 s` 完成释放确认，`14.517 s` 停止，与原 `14.522 s` 基线基本一致；
- 两侧目前均进入相同的单支撑横向发散类别，P0-4 仍未通过。

---

## 日期

2026-07-15

## 修改目标

审计原 AzureLoong 行走基线和 Lyenbot 实际足底 contact，修正 MuJoCo 实测 wrench/CoP 的漏计问题。

## 修改文件

- 文件路径：`common/data_bus.h`、`algorithm/wbc_priority.h/.cpp`、`demo/lyenbot_staged.cpp`
- 涉及模块：Lyenbot 接触诊断、WBC 接触释放适配

## 修改内容

`measuredFootWrenches()` 原先只汇总命名的 `lf/rf-tc-collision` box。现改为先取得该 geom 所属 ankle-roll body，再汇总同一 body 上全部 geom 的 contact force/torque。完整 wrench 写入 248 列诊断 CSV，其法向力还在单步 demo 内用于确认接触释放；完整向量不写回通用 DataBus。

验收 demo 的安全门新增 QP 浮动基座加速度幅值检查：`wbc_ddq_qp.head<6>()` 的无穷范数必须有限且不超过 `100`，否则即使 `qp_status=0` 也立即停止并打印 `qp_base_ddq_max`。该保护不修改 WBC 求解，只阻止物理异常解继续进入仿真执行。

新增默认关闭的 `wbc_contact_release` 适配信号。单步释放阶段保持 `legState=DSt`，QP 继续使用双足 wrench 约束，但 WBC 运动学静态接触只锁定 `legStateNext` 指定的未来支撑脚并启用摆脚任务；完整实测摆脚 `fz<5 N` 连续 `20 ms` 后才正式切换 `LSt/RSt`。CSV 新增 `contact_release`，由 247 列扩展为 248 列。

## 设计说明

生成的 Lyenbot 每只脚同时包含规则 contact box 和 URDF 转换保留的 ankle-roll collision mesh。旧诊断只统计 box，站立时左右实测法向力总和约 `176 N`，与 `31.8376 kg` 模型的 `312.3 N` 重量不符；按 body 汇总后总法向力约 `312.3 N`，确认旧值漏计了 mesh 接触。

曾试验恢复 Walk `PosRot` 的 6D/仅横向速度反馈、把支撑脚参考 CoP 显式设为内侧 `30 mm`，均使 QP 更早不可行或对结果无影响，已全部撤销。也曾在生成层禁用原 foot mesh、只保留规则 box；该模型在 PD 阶段 `t=2.124 s` 就因 pitch 超过 1 rad 失败，已撤销并重新生成恢复原接触模型。

完整 wrench 还证明切换前约 `37 N` 摆脚载荷符合双脚静力分配：CoM 位于支撑脚内侧约 `24 mm`、双脚间距约 `225 mm`，估算摆脚载荷约 `33 N`。为验证能否准静态卸载，曾把 CoM 目标改到支撑脚中心并增加 2 秒纯 Stand 保持；尚未进入 Walk/卸载就于 `t=13.691 s` 因 pitch 超限，故已恢复内侧 `10 mm` 的有效基线。当前不能靠延长等待实现零载荷，后续需设计允许约 `30~40 N` 初始载荷的受控动态离地。

另试验两阶段快速抬脚：前 `0.2 s` 抬高 `8 mm`，随后平滑升至 `25 mm`。完整实测摆脚 `fz<1 N` 的时刻由约 `13.29 s` 提前到 `13.13 s`，但 QP 不可行也由约 `14.525 s` 提前到 `14.379 s`，未形成净改善，轨迹修改已撤销。有效基线仍使用 1 秒平滑抬升。

又试验在切换前 0.5 秒将 CoM 目标短时推向支撑脚中心。该微小参考变化在 `t=12.700 s` 触发摆脚提前失去实测接触，而控制状态仍是双支撑；QP 状态和残差仍显示成功，但输出横向基座加速度约 `+81.7 m/s²`、支撑脚规划法向力约 `1398 N`，随后于 `t=14.021 s` 失败且未完成 airborne 验收。该候选已撤销，并暴露出固定接触状态与实际接触不一致时缺少输出幅值保护的问题。

原始 AzureLoong `walk_wbc` 也不能作为当前成功基线：实际运行中 `t=1.018 s` pitch 已超过 1 rad，而 WBC/步态要到 3 s 才启动，足底标量传感器持续为零。该结果只记录现状，不在 Lyenbot P0-4 中修复旧 demo。

## 编译测试

- `generate_mjcf`、`model_check`、`lyenbot_staged` 增量构建通过；
- 重新生成后 `model_check` 通过：`nq=30`、`nv=29`、`nu=23`，足底位姿误差约 `4.65e-7 m`；
- 恢复复合足底接触后 `./lyenbot_staged wbc` 通过至 `13.001 s`，`base_z=0.848603 m`；
- 完整 contact 诊断下，单步切换前摆脚实际仍承载约 `37 N`，右支撑脚完整实际 CoP 约在中心，和 WBC 约 `38 mm` 的计划 CoP 不一致；P0-4 仍未通过。
- 恢复 10 mm/20 秒单步基线后 `lyenbot_staged` 增量构建通过；中心 CoM 保持试验未保留。
- 两阶段快速抬脚候选增量构建通过；`step-left` 于 `t=14.379 s`、`qp_status=-2` 安全停止，差于有效基线，候选已撤销。
- 横向预脉冲候选增量构建通过；`step-left` 于 `t=14.021 s` 失败，且双支撑接触失配时出现极端但 `qp_status=0` 的输出，候选已撤销。
- QP 输出幅值保护增量构建通过；`./lyenbot_staged wbc` 正常通过至 `13.001 s`；恢复有效轨迹的 `step-left` 于 `t=14.522 s` 截获 `qp_status=0`、`qp_base_ddq_max=130.844` 的异常解，比原 `t=14.525 s` 的 QP 不可行更早安全停止。
- 接触释放状态机完整增量构建通过；`./lyenbot_staged wbc` 仍通过至 `13.001 s`。
- `step-left` 在 `13.308 s` 完成实测释放确认后才由 `DSt` 切换 `RSt`，证明状态顺序正确；但仍于 `14.523 s` 截获 `qp_base_ddq_max=131.928`，P0-4 未通过。
- `step-right` 在释放阶段开始前即于 `11.294 s` 截获 `qp_base_ddq_max=200.657`；将默认关闭路径恢复为原左右选择后结果完全相同，确认这是既有双支撑卸载不对称问题，不是新状态机回归。

---

## 日期

2026-07-15

## 修改目标

为 P0-4 增加物理足底 CoP 约束、双支撑载荷卸载和 MuJoCo 实测接触 wrench 诊断，并在 QP 不可行时安全停止验收 demo。

## 修改文件

- 文件路径：`common/robot_model_config.h/.cpp`
- 文件路径：`common/robot_configs/lyenbot.json`
- 文件路径：`common/data_bus.h`
- 文件路径：`algorithm/wbc_priority.h/.cpp`
- 文件路径：`demo/lyenbot_staged.cpp`
- 涉及模块：机器人配置、WBC 接触约束、Lyenbot 准静态单步与诊断

## 修改内容

Lyenbot 配置新增足底半长 `0.08 m`、半宽 `0.04 m`。配置化 WBC 路径由 22 个约束扩展为 26 个约束，用 `|tx| <= foot_half_width * fz`、`|ty| <= foot_half_length * fz` 耦合约束替代与法向力无关的固定 roll/pitch 力矩边界；未传机器人配置的 AzureLoong 旧构造路径继续保留 22 约束行为。

准静态单步新增 `Walk+DSt` 载荷转移阶段：以支撑足为参考合并双足初始 wrench，连续把规划摆脚法向力上限卸载到零，再切换单支撑。DataBus 新增 `wbc_swing_foot_fz_max` 传递该时变硬上限。

诊断层使用 `mj_contactForce()` 汇总左右足碰撞 geom 的实际 6D wrench，并计算实际 CoP；CSV 由 219 列扩展为 247 列。安全检查新增 `qp_status`，首次 QP 不可行即停止，禁止继续使用旧解。

单步预备阶段当前使用支撑脚内侧 `10 mm` 的 CoM 目标，时间表为 `8~11 s` 横移、`11~13 s` 卸载、`13 s` 进入单支撑，总时长 20 秒。任务从 Stand CoM 跟踪切换到 Walk base 跟踪时补偿当前 base-CoM 横向偏差。

## 设计说明

原固定 `|tx| <= 15 Nm` 在 `fz=10 N` 时允许约 `1.5 m` 的虚假 CoP，不能代表 Lyenbot 的 `80 mm × 40 mm` 半尺寸足底。耦合 CoP 约束属于配置化适配，避免改变 AzureLoong 旧路径。

实测表明 WBC 规划摆脚法向力能够在切换前降至零，但 MuJoCo 实际接触仍延迟约 `0.25 s` 才完全卸载；支撑脚实际 CoP 随后才移向边界，横向速度已开始发散。固定 CoM 偏置 20 mm、10 mm 和脚中心目标均不能完成单步，下一步需要显式设计 CoM/CoP 动态过渡，而不是继续调整固定偏置或放宽物理接触约束。

## 编译测试

编译命令：

```bash
cmake --build build --target lyenbot_staged -j1
```

测试结果：

- `lyenbot_staged` Release 增量构建通过；
- Release 全量构建通过，包括保留旧 22 约束构造方式的 AzureLoong demo 目标；
- `./lyenbot_staged wbc` 回归通过，运行至 `13.001 s`，`base_z=0.848603 m`，全程 `qp_status=0`；
- `step-left` 在 10 mm 目标下仍未通过，首次 QP 不可行发生于 `t=14.525 s`；安全停止时 roll `-0.0695 rad`、pitch `-0.0077 rad`，说明已在旧解放大跌倒前终止；
- P0-4 保持“排查中”，没有执行右脚最终验收，因为左脚已不满足共同验收条件。

---

## 日期

2026-07-15

## 修改目标

继续定位 P0-4 单支撑横向发散，修复 WBC 优先级加速度零空间递推，并补充接触切换诊断。

## 修改文件

- 文件路径：`algorithm/priority_tasks.cpp`
- 文件路径：`algorithm/wbc_priority.cpp`
- 文件路径：`common/data_bus.h`
- 文件路径：`demo/lyenbot_staged.cpp`
- 涉及模块：WBC 优先级任务、QP 诊断、Lyenbot 准静态单步

## 修改内容

动态加权伪逆生成的低优先级加速度增量现在左乘累计零空间 `N`，避免 `M^-1` 将修正量带出父任务零空间并破坏支撑足/基座高优先级任务。该修复是机器人无关的共享 WBC 修复。

DataBus 新增 `wbc_ddq_qp`，CSV 新增 MuJoCo 基座实际加速度、WBC 运动学基座加速度、QP 修正后基座加速度及两者差值，共形成 219 列当前诊断格式。

准静态单步在单支撑切换时重新锁存摆脚起点和支撑脚 anchor，消除旧 `t=8 s` 足端位置造成的毫米级向下轨迹跳变；阶段改为 `8~10 s` 移重心、`10~12 s` 双足静止保持、`12 s` 切换单支撑，总时长延长为 19 秒。

## 设计说明

新增诊断证明原实现中低优先级任务会使最终运动学 `ddq_y` 反向；零空间修复后运动学命令持续朝向基座目标，但 QP 在支撑足 roll wrench 接近上限时仍会反向修正，单步验收尚未通过。测试过支撑足中心和内偏 10 mm 的 CoM 目标，均未改善，因此恢复内偏 20 mm，不保留试探常量。

## 编译测试

编译命令：

```bash
cmake --build build --target lyenbot_staged -j2
```

测试结果：

- `lyenbot_staged` Release 构建通过；
- Release 全量构建通过，包括原有 AzureLoong demo 目标；
- Lyenbot 双足 WBC 回归通过，`13.001 s` 最终 `base_z=0.848603 m`；
- 摆脚切换轨迹已连续，切换前两秒内基座速度收敛；
- 左右单步仍在 touchdown 前镜像失稳，P0-4 未通过；
- 下一步需要显式双足载荷卸载，不能继续用 CoM 偏置常量代替接触力转移。

---

## 日期

2026-07-15

## 修改目标

实现 P0-2 左右脚准静态原地单步测试，并记录当前未通过结果。

## 修改文件

- 文件路径：`demo/lyenbot_staged.cpp`
- 文件路径：`codex_notes/CHANGELOG.md`
- 文件路径：`codex_notes/DEBUG_LOG.md`
- 文件路径：`codex_notes/TODO.md`
- 文件路径：`codex_notes/LYENBOT_ADAPTATION.md`
- 涉及模块：Lyenbot staged demo、WBC 单支撑验收

## 修改内容

新增 `step-left/step-right` 模式，以 2 秒平滑 CoM 转移、原地竖直足端轨迹和 touchdown 后双支撑保持，将单支撑问题与 GaitScheduler、FootPlacement、MPC 分离。安全停机与正常结束均可自动输出 airborne、touchdown、稳定时间、支撑足滑移和姿态极值。

CSV 增加 WBC 左右 6D wrench，用于确认单支撑时接触力侧别和约束结果。测试中曾评估 Walk `PosRot` 速度阻尼，但未改变镜像发散方向，因此没有保留该共享行为修改。

## 设计说明

CoM 转移目标和 floating-base 位置目标分开处理：切换到单支撑时锁存实际基座位置，避免把 CoM 横向目标错误地作为基座位置目标。新增模式只作用于 Lyenbot staged 验收路径。

## 编译测试

编译命令：

```bash
cmake --build build --target lyenbot_staged -j2
```

运行命令：

```bash
cd build
./lyenbot_staged step-left --diagnostic=/tmp/lyenbot_step_left.csv
./lyenbot_staged step-right --diagnostic=/tmp/lyenbot_step_right.csv
```

测试结果：

- Release 编译通过；
- 全量 `cmake --build build -j2` 通过；
- 两侧均检测到摆脚离地；
- `step-left` 于 `t=11.547 s`、roll `-1.00203 rad` 安全停机；
- `step-right` 于 `t=11.609 s`、roll `+1.00011 rad` 安全停机；
- touchdown 和落地后 3 秒保持均未完成，P0-2 验收未通过；
- QP、wrench 侧别、初期滑移和 ankle effort 证据将阻塞收敛到 WBC 单支撑横向控制。

存在问题：

P0-2 测试已实现但验收未通过；下一步进入 P0-4，使用相同左右模式定位 WBC 单支撑横向闭环根因。

---

## 日期

2026-07-15

## 修改目标

完成 P0-3：增加首步失败诊断数据并定位失稳类别。

## 修改文件

- 文件路径：`demo/lyenbot_staged.cpp`
- 文件路径：`common/data_bus.h`
- 文件路径：`algorithm/wbc_priority.h`
- 文件路径：`algorithm/wbc_priority.cpp`
- 文件路径：`codex_notes/CHANGELOG.md`
- 文件路径：`codex_notes/DEBUG_LOG.md`
- 文件路径：`codex_notes/DESIGN_NOTES.md`
- 文件路径：`codex_notes/TODO.md`
- 文件路径：`codex_notes/LYENBOT_ADAPTATION.md`
- 涉及模块：Lyenbot staged demo、WBC QP 诊断、DataBus

## 修改内容

### 修改1：增加 100 Hz 行走 CSV

`lyenbot_staged walk` 默认写出 `../record/lyenbot_walk_diagnostic.csv`，并支持 `--diagnostic=<路径>`。日志记录步态、基座、WBC 目标、CoM、足端位姿/速度、摆脚目标、touch/contact、QP、全部关节状态/力矩、ankle effort 余量和支撑足滑移，不改变控制计算。

### 修改2：输出真实 QP 约束残差

WBC 使用实际 `A/lbA/ubA/xOpt` 计算浮动基座等式残差无穷范数和不等式最大违反量，通过 DataBus 提供给诊断层。该修改是机器人无关的共享诊断增强。

### 修改3：定位首步失稳的数据覆盖类别

复现确认未启用 MPC 时仍调用 `MPC::dataBusWrite()`，禁用态 MPC 覆盖 WBC 横向期望；摆脚随后提前触地并发生接触丢失。QP 始终成功，ankle 饱和晚于接触丢失。

## 设计说明

诊断层只读取状态；QP 残差在拥有求解器矩阵和实际解的 WBC 内部计算。P0-3 只完成记录和归类，不提前改变 P0-2/P0-4 的控制行为。

## 编译测试

编译命令：

```bash
cmake --build build --target lyenbot_staged -j2
```

运行命令：

```bash
cd build
./lyenbot_staged walk --diagnostic=/tmp/lyenbot_walk_diagnostic.csv
```

测试结果：

- Release 编译通过；
- 全量 `cmake --build build -j2` 通过，原有 demo 与新增测试目标均成功链接；
- 按预期在 `t=13.478 s` 复现既有安全停机；
- CSV 共 449 行、186 列，覆盖 `t=9.00~13.47 s`；
- `qp_status=0`，等式残差最大约 `6.82e-13`，不等式最大违反量约 `2.42e-12`；
- 日志可将失败定位到轨迹目标被覆盖及摆脚提前触地类别。

存在问题：

D008 尚未修复；下一步按 P0-2 建立左右脚准静态原地单步，再进入 P0-4 修复。

---

## 日期

2026-07-15

## 修改目标

完成 P0-1：23 关节动态逐关节映射测试。

## 修改文件

- 文件路径：`CMakeLists.txt`
- 文件路径：`demo/lyenbot_joint_mapping_test.cpp`
- 文件路径：`codex_notes/CHANGELOG.md`
- 文件路径：`codex_notes/TODO.md`
- 文件路径：`codex_notes/LYENBOT_ADAPTATION.md`
- 修改函数：新增独立测试程序及其轨迹运行、差分比较逻辑
- 涉及模块：MuJoCo、`MJ_Interface`、PVT、Lyenbot 关节映射验收

## 修改内容

### 修改1：增加动态映射验收目标

原代码作用：

`model_check` 只检查 joint、actuator、限制和足端位姿的静态一致性，不能验证实际控制响应方向。

修改后：

新增 headless 目标 `lyenbot_joint_mapping_test`。测试通过真实 `MJ_Interface` 和 PVT 路径，每次只改变一个关节的位置期望，并自动汇总 23 项结果。

修改原因：

在进入行走失稳调试前排除控制数组、actuator 名称和关节正方向错误。

影响分析：

只新增测试入口，不改变现有模型、控制器、WBC、步态或 demo 行为。

### 修改2：使用无激励基线识别增量响应

原代码作用：

初版测试若直接以稳定后的实际位置重新设定期望，会撤掉维持重力平衡所需的 PD 偏差，把踝关节自然姿态变化误判为串扰。

修改后：

从相同 `home` keyframe 分别运行无激励基线和单关节 `0.02 rad` 增量轨迹，逐时刻比较两条轨迹，仅统计目标激励造成的增量响应。

修改原因：

隔离自由基座、重力和接触引起的公共运动，同时保留真实闭环控制路径。

## 设计说明

独立测试避免把验收逻辑混入 `lyenbot_staged`；差分基线比放宽串扰阈值更能反映单一关节指令的实际影响。

## 编译测试

编译命令：

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug --target lyenbot_joint_mapping_test -j2
```

运行命令：

```bash
cd build-debug
./lyenbot_joint_mapping_test
```

测试结果：

- 编译通过；
- 23/23 关节全部通过；
- 全部目标响应方向正确，反向峰值均为 0；
- 最大非目标增量为 `0.006887 rad`，低于 `0.01 rad` 判据；
- 程序输出 `LYENBOT_JOINT_MAPPING_OK` 并返回 0。

存在问题：

P0-1 已完成；首步单支撑横向失稳仍需按 P0-3、P0-2、P0-4 顺序继续排查。

---

## 日期

2026-07-15

## 修改目标

重新整理工程文档体系，建立长期维护使用的中文工程记录目录 `codex_notes/`。

## 修改文件

- 文件路径：`codex_notes/CHANGELOG.md`
- 文件路径：`codex_notes/DEBUG_LOG.md`
- 文件路径：`codex_notes/DESIGN_NOTES.md`
- 文件路径：`codex_notes/TODO.md`
- 文件路径：`codex_notes/LYENBOT_ADAPTATION.md`
- 文件路径：`AGENTS.md`
- 删除文件：`LYENBOT_PROJECT.md`
- 删除文件：`doc/conversation_handoffs/lyenbot_adaptation_handoff_zh.md`
- 修改函数：无，仅进行文档新增、整理和去重
- 涉及模块：工程文档与开发流程

## 修改内容

### 修改1：建立五类工程记录

原代码作用：

项目已有 README、Tutorial、Doxygen/Sphinx 文档、Lyenbot 教学报告和对话交接文档，但缺少统一的长期工程记录目录。

修改后：

新增 `codex_notes/`，将信息按修改记录、调试记录、设计说明、待办任务和 Lyenbot 专项适配分开维护。

修改原因：

减少跨文档重复，确保新对话和后续开发者能够快速找到当前事实、历史原因和未完成任务。

影响分析：

第一阶段只新增文档。完成逐项迁移核对后，删除了两份内容已被完全覆盖的旧入口文件；详细教学报告、上游文档、规范文件、文档生成输入和第三方文档全部保留。不影响构建和运行。

### 修改2：清理已完全迁移的旧文档

原代码作用：

- `LYENBOT_PROJECT.md` 保存早期 Lyenbot 适配目标和本机上游参考路径；
- `doc/conversation_handoffs/lyenbot_adaptation_handoff_zh.md` 保存一次对话的进度交接。

修改后：

- 项目目标、模型选择和上游参考迁入 `LYENBOT_ADAPTATION.md`；
- 已完成修改和验证迁入 `CHANGELOG.md`；
- 调试结论迁入 `DEBUG_LOG.md`；
- 未完成任务和 Git 阻塞迁入 `TODO.md`；
- 架构与数据流迁入 `DESIGN_NOTES.md`；
- 新对话接续说明迁入 `LYENBOT_ADAPTATION.md`；
- 两份旧文件删除，避免继续维护重复内容。

修改原因：

两份旧文件已没有独有技术内容，继续保留会形成多个“当前状态”入口并产生同步偏差。

影响分析：

新的统一入口是 `codex_notes/`。逐语句教学报告 `doc/lyenbot_model_adaptation_zh.md` 没有删除，因为其细节没有被完整迁移。

### 修改3：增加工程文档按需读取规则

原代码作用：

`LYENBOT_ADAPTATION.md` 的接续提示要求新对话一次读取五份工程记录；`AGENTS.md` 没有规定分层读取策略，并且末尾代码围栏未闭合。

修改后：

- 补全 `AGENTS.md` 中的 `// LYENBOT MODIFY` 标记和代码围栏；
- 增加“默认只读 AGENTS、按任务类型读取对应章节”的规范；
- 新对话只先读取适配结论、验收总览和当前 TODO；
- 调试、设计、历史和逐语句教学文档改为按需读取；
- 精简 `LYENBOT_ADAPTATION.md` 的新对话提示。

修改原因：

五份工程记录合计超过千行，每次完整读取会占用大量上下文，也会引入与当前任务无关的信息。

影响分析：

不改变文档事实和代码行为，只优化 Codex 的上下文加载方式。用户明确要求全面审查时仍允许读取全部文档。

## 设计说明

说明：

- `CHANGELOG.md` 只保存实际变更和测试结果；
- `DEBUG_LOG.md` 保存问题现象、排查过程和解决状态；
- `DESIGN_NOTES.md` 保存稳定的系统设计与数据流；
- `TODO.md` 保存带优先级和验收条件的未完成任务；
- `LYENBOT_ADAPTATION.md` 保存 Lyenbot 专项事实与适配接口；
- 迁移校验前保留全部原文档；校验后只删除内容已完全迁移的两份旧入口，其余原文档继续保留。

## 编译测试

编译命令：

```text
本次仅修改 Markdown，不需要编译。
```

运行命令：

```bash
git diff --check -- codex_notes
rg -n '^#' codex_notes/*.md
```

测试结果：

- 五份目标文档均已创建并使用中文维护；
- Markdown 标题结构检查通过；
- 文档关节名称与 `common/robot_configs/lyenbot.json` 核对一致；
- 删除的两份旧 Markdown 均已逐项确认完成迁移；
- 未完全迁移或具有独立用途的 Markdown 均保留；
- 没有修改代码、URDF、配置和构建产物。

存在问题：

- 无文档格式阻塞；控制与行走功能的未完成项见 `TODO.md`。

---

## 日期

2026-07-14

## 修改目标

保留 AzureLoong 运行路径，新增基于只读 Lyenbot URDF 的配置化 MuJoCo/Pinocchio 仿真适配，并建立分阶段验证入口。

对应 Git 提交：

```text
9239c663795792fbd60eeee4c24d850c0a8a7fbd
feat: 增加 Lyenbot 只读 URDF 仿真适配
```

## 修改文件

- 文件路径：`common/robot_model_config.h/.cpp`
- 文件路径：`common/urdf_model_loader.h/.cpp`
- 文件路径：`common/robot_configs/azureloong.json`
- 文件路径：`common/robot_configs/lyenbot.json`
- 文件路径：`common/lyenbot_joint_ctrl_config.json`
- 文件路径：`algorithm/pino_kin_dyn.h/.cpp`
- 文件路径：`algorithm/wbc_priority.h/.cpp`
- 文件路径：`algorithm/gait_scheduler.h/.cpp`
- 文件路径：`algorithm/foot_placement.h/.cpp`
- 文件路径：`common/PVT_ctrl.h/.cpp`
- 文件路径：`sim_interface/MJ_interface.h/.cpp`
- 文件路径：`sim_interface/GLFW_callbacks.h/.cpp`
- 文件路径：`demo/generate_mjcf.cpp`
- 文件路径：`demo/model_check.cpp`
- 文件路径：`demo/lyenbot_staged.cpp`
- 文件路径：`CMakeLists.txt`
- 文件路径：`doc/lyenbot_model_adaptation_zh.md`
- 修改函数：涉及配置加载、URDF 加载、状态映射、动力学、WBC、PVT、步态、模型生成、GUI 生命周期及 demo 主循环
- 涉及模块：模型、仿真、运动学、动力学、全身控制、步态调度、关节控制、GUI、测试

## 修改内容

### 修改1：将 URDF 设为机械参数真源

原代码作用：

原项目主要针对 AzureLoong，模型路径、关节数量、关节名称和部分限制与原模型结构紧密耦合。

修改后：

固定使用 Lyenbot 主 URDF，通过 Pinocchio 读取拓扑、质量、惯量、轴向、位置/速度/effort 限制和 Frame。原始 URDF 不写回。

修改原因：

确保项目适配机器人模型，而不是修改机器人模型来适配项目。

影响分析：

Lyenbot 的机械限制统一来自 URDF；外部 JSON 不允许覆盖这些限制。

### 修改2：增加机器人语义配置和名称映射

原代码作用：

旧代码包含 AzureLoong 专用名称列表和多个固定数字下标。

修改后：

新增 `RobotModelConfig` 和 `JointLayout`。关节组、足底 Frame、基座和传感器使用名称配置，Pinocchio 与 MuJoCo 地址在运行时查询。

修改原因：

AzureLoong 有 31 个受控关节，Lyenbot 只有 23 个；两者的手臂、腰部和头部拓扑不同，固定下标会作用到错误关节。

影响分析：

Lyenbot 使用配置化路径；旧构造接口继续使用 AzureLoong 默认布局。

### 修改3：增加 URDF 到 MuJoCo 的可重复生成工具

原代码作用：

旧模型替换教程依赖人工导入和维护 XML。

修改后：

新增 `generate_mjcf`，生成浮动基座、23 个 torque actuator、IMU、左右足底 touch、场景和初始 keyframe，并记录源 URDF 哈希。

修改原因：

避免人工维护两套机械参数产生漂移。

影响分析：

`generated/*.xml` 是可重建派生文件，不能代替原始 URDF。

### 修改4：修复 MuJoCo actuator 映射

原代码作用：

扭矩曾按 `ctrl[i]` 写出，隐含 actuator 顺序等于电机数组顺序。

修改后：

按 joint 名称找到 joint id，再通过 `actuator_trnid` 找到 actuator id，最终写入 `ctrl[actuatorId]`。

修改原因：

XML 排列变化时，顺序耦合可能把扭矩施加到错误关节。

影响分析：

控制输出与 XML actuator 排列解耦。

### 修改5：适配 Lyenbot 拓扑与 WBC

原代码作用：

原任务假设双臂各 7 DoF、头部 2 DoF、腰部 3 DoF。

修改后：

Lyenbot 使用双臂各 5 DoF 的关节姿态任务、不加入头部任务、只处理一个 waist-yaw；左右腿及足底 Frame 按 URDF 名称处理。

修改原因：

避免强行将 AzureLoong 的任务维数套到 Lyenbot。

影响分析：

PD 和双足 WBC 站立可运行；连续行走仍需调试。

### 修改6：增加阶段化测试与 GUI

原代码作用：

缺少 Lyenbot 专用的模型一致性和分阶段运行入口。

修改后：

新增 `model_check`、`lyenbot_staged pd/wbc/walk` 和可选 `--gui`。同时整理 GLFW/MuJoCo 资源所有权，避免双重释放。

修改原因：

将模型错误、关节控制错误、WBC 错误和步态错误分阶段隔离。

影响分析：

PD/WBC 可无 GUI 自动验证，也可在 GUI 中观察。

## 设计说明

说明：

- 机械参数、语义配置和控制/仿真参数分离；
- 采用名称映射而不是复制一套完全独立的控制核心；
- 保留 AzureLoong 旧接口以降低回归风险；
- actuator 首版使用 `gear=1`，不虚构真实减速比；
- 失败阶段安全停止，不通过放大 effort、关闭限位或修改 URDF 掩盖问题。

其他方案：

- 可以复制一套 Lyenbot 专用控制代码，但会长期保留危险数字下标并造成两套算法分叉，因此未采用；
- 可以手工维护 MuJoCo XML，但容易与 URDF 的惯量和限制失去同步，因此改为生成工具。

## 编译测试

编译命令：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

运行命令：

```bash
./build/generate_mjcf common/robot_configs/lyenbot.json
./build/model_check common/robot_configs/lyenbot.json
(cd build && ./lyenbot_staged pd)
(cd build && ./lyenbot_staged wbc)
```

测试结果：

- 原始 URDF SHA-256 保持为 `86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22`；
- `GENERATE_MJCF_OK`；
- `MODEL_CHECK_OK`；
- Pinocchio/MuJoCo 均为 `nq=30`、`nv=29`，MuJoCo `nu=23`；
- 左右足底同姿态位置误差约 `4.65e-7 m`；
- PD 运行 `10.001 s`，最终 `base_z=0.848443 m`；
- WBC 第 3 秒接管后站立 10 秒，最终 `base_z=0.848591 m`；
- AzureLoong 原有目标重新构建通过。

存在问题：

- 23 关节动态逐关节方向测试尚未补齐；
- WBC 低速行走在首步单支撑阶段侧向失稳；
- MPC 连续行走 10 步尚未通过。
