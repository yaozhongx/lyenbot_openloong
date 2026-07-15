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
