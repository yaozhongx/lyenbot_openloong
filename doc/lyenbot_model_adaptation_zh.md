# Lyenbot 只读 URDF 适配：实现、原理与逐语句教学报告

## 1. 当前结论（先看这里）

本轮已经完成模型导入、配置化、URDF 限制读取、MuJoCo 可重复生成、Pinocchio/MuJoCo 映射检查、AzureLoong 编译回归、保守 PD 站立和双足 WBC 站立。原始 URDF 没有修改。

验收状态：

| 阶段 | 状态 | 实测结果 |
|---|---|---|
| A：URDF/Pinocchio | 通过 | `nq=30`、`nv=29`、23 个活动关节；限制、惯量和 Frame 可读取 |
| B：生成 MuJoCo/一致性 | 通过 | `nq=30`、`nv=29`、`nu=23`；足底位置误差分别约 `4.64746e-7 m`、`4.6466e-7 m` |
| C：名称和 actuator 映射 | 静态检查通过 | 23 个 joint/actuator 逐名匹配；动态逐关节扰动测试仍应补充 |
| D：保守 PD | 通过 | 运行 `10.001 s`，最终 `base_z=0.848443 m` |
| E：双脚 WBC | 通过 | 第 3 秒接管后持续 10 秒，最终 `base_z=0.848591 m` |
| F：低速 WBC 行走 | 未通过 | 首步单支撑时侧向控制失稳，安全停机，无 NaN |
| G：MPC、0.15 m/s、10 步 | 未通过 | MPC 接入路径存在，但不能在 F 未通过时宣称最终验收 |

因此，“稳定站立 10 秒”已经完成；“连续行走 10 步”尚未完成。本报告不会把失败包装成成功。

原始文件 SHA-256（实施前后相同）：

```text
86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22
```

原文件：`models/lyn03-V07-urdf_C01-260414/lyn03-V07-C01-260326.urdf`。

## 2. 三类参数必须分开

### 2.1 URDF 机械参数：唯一真源

下列数据只从 URDF 经 Pinocchio 读取，配置文件不能覆盖：

- link 质量、质心、惯量；
- joint 父子拓扑、原点、轴向；
- position、velocity、effort 限制；
- 固定 Frame 和碰撞几何；
- 23 个活动关节的 `idx_q/idx_v`。

URDF 中 4 个 XML 数值属性带尾部空格，旧版 urdfdom 会丢失相应惯量。`readNormalizedUrdfXml()` 只对内存字符串的属性首尾空白做 trim，再交给 Pinocchio；它不写回文件。这是“兼容解析”，不是修改机械数据。

### 2.2 语义配置：URDF 标准没有表达的含义

`common/robot_configs/lyenbot.json` 只告诉控制器：

- 哪些名字属于左腿、右腿、左臂、右臂和腰；
- 哪个固定 Frame 是左右足底；
- 哪个 body 是基座；
- MuJoCo 传感器叫什么。

关节索引由名字查询得到，JSON 中没有 `q[19]` 这类危险数字。

### 2.3 仿真/控制参数：明确不是机械真值

下列参数是首版仿真初值：

- PVT 的 `kp/kd/LPF`；
- MuJoCo `timestep/damping/frictionloss/armature/friction`；
- 初始关节姿态、摆动周期、摆脚高度、步长限制；
- 落地检测阈值、双支撑换重心时间；
- WBC 的任务增益。

actuator `gear=1`，控制器输出视为关节侧扭矩；`ctrlrange` 严格等于 URDF effort 的正负值。

## 3. 两个模型的关节差异

AzureLoong 为 31 个受控关节：14 臂、2 头、3 腰、12 腿。Lyenbot 为 23 个受控关节：10 臂、0 头、1 腰、12 腿。

Lyenbot 的 Pinocchio 顺序为：

```text
左腿: left_hip_pitch, left_hip_roll, left_hip_yaw,
      left_knee_pitch, left_ankle_pitch, left_ankle_roll
右腿: right_hip_pitch, right_hip_roll, right_hip_yaw,
      right_knee_pitch, right_ankle_pitch, right_ankle_roll
腰部: waist_yaw
左臂: left_shoulder_pitch, left_shoulder_roll, left_elbow_yaw,
      left_elbow_pitch, left_elbow_roll
右臂: right_shoulder_pitch, right_shoulder_roll, right_elbow_yaw,
      right_elbow_pitch, right_elbow_roll
```

Lyenbot 没有头部任务；每臂只有 5 DoF，因此只做命名关节姿态跟踪，不执行 AzureLoong 双手 6D IK。腰部只有 yaw，所有排除列和权重均通过名字生成的速度索引处理。

## 4. 数据流和基础原理

```text
只读 URDF
  ├─ Pinocchio：拓扑、Frame、J/dJ、M/C/G、限制
  └─ generate_mjcf：派生 Lyenbot.xml/scene/keyframe/actuator/sensor
                        ↓
MuJoCo qpos/qvel/sensor ── MJ_Interface 名称映射 ── DataBus（Pinocchio 顺序）
                        ↓
Pin_KinDyn：运动学/动力学 ── WBC/MPC ── 期望 q/dq/tau
                        ↓
PVT（URDF effort 限幅） ── actuator_id 映射 ── MuJoCo ctrl
```

浮动基模型有 7 个基座位姿坐标（3 平移 + 四元数）和 6 个基座速度，所以 23 关节时 `nq=7+23=30`、`nv=6+23=29`。

Frame 是附着在 link 上的坐标系。本项目用 `left_foot_contact_point/right_foot_contact_point` 计算足底位置和 Jacobian。Jacobian 满足近似关系 `v_frame = J(q)dq`；`dJ*dq` 是加速度约束中不能长期忽略的项。

动力学写成：

```text
M(q) ddq + C(q,dq)dq + G(q) = S^T tau + J^T F
```

WBC 先按优先级求满足足底接触、基座姿态、质心和手臂姿态的 `ddq`，再用 18 变量 QP 修正 6 个浮动基加速度与 12 个双足 wrench。22 个约束由 6 个浮动基动力学等式与 16 个摩擦锥/法向力/接触力矩不等式组成。

## 5. 新接口

```cpp
RobotModelConfig loadRobotModelConfig(const std::string &configPath);
RobotModelConfig azureLoongDefaultConfig();
std::string readNormalizedUrdfXml(const std::string &urdfPath, int *count = nullptr);
void buildFloatingBaseModelFromUrdf(...);
void buildFixedBaseModelFromUrdf(...);
```

`JointLayout` 保存名字集合；`Pin_KinDyn::motorQIndex/motorVIndex` 是由 Pinocchio 名字查询生成的显式映射。`MJ_Interface` 分别查询 joint 的 `qposadr/dofadr` 和驱动该 joint 的 actuator id。

## 6. 逐文件、逐语句说明

这里按“一个可执行语句或同类重复语句”为单位解释。JSON 中 23 个同构关节条目不重复写 23 次相同原理，但每个名字都在第 3 节列出。

### 6.1 `CMakeLists.txt`

| 语句 | 作用 | 不加的后果 | 验证 |
|---|---|---|---|
| `add_executable(model_check ...)` | 建立无 GUI 模型检查命令 | 无法自动检查限制和映射 | 运行 `model_check` |
| `target_link_libraries(model_check core mujoco ...)` | 同时读取 Pinocchio 与 MuJoCo | 无法做跨引擎足底比较 | 输出 `foot_pose_error` |
| `add_executable(generate_mjcf ...)` | 建立 URDF→MJCF 生成器 | 只能人工维护 XML | 删除 generated 后可重建 |
| `add_executable(lyenbot_staged ...)` | 建立 PD/WBC/Walk 分阶段入口 | 失败阶段无法隔离 | `pd/wbc/walk` 参数 |

### 6.2 `common/robot_model_config.h/.cpp`

- `JointLayout` 的 7 个 vector：保存活动、双臂、双腿、腰和头的 URDF 名字；空 `head` 直接使 Lyenbot 不创建头任务。
- `RobotModelConfig` 的路径字段：统一 URDF、scene 和 PVT JSON 的来源，避免 demo 中散落相对路径。
- Frame/sensor 字段：把标准 URDF 缺少的控制语义显式化。
- gait/simulation 字段：明确保存非机械调参，避免污染 URDF。
- `readStrings/readDoubles`：逐项把 JSON 数组转成 C++ vector。
- `resolvePath`：相对路径以配置文件目录为基准，而不是依赖启动目录。
- `loadRobotModelConfig`：读取、解析并校验初始姿态数量必须等于活动关节数量；遗漏校验会导致数组错位。
- `simulation.get(..., default)`：Azure 配置没有新增字段时沿用默认值，保持旧路径兼容。
- `azureLoongDefaultConfig`：旧构造函数仍走 Azure 默认布局。

### 6.3 两份 robot JSON

- `azureloong.json`：复刻旧 31 关节名字和旧 Frame/sensor 语义，只用于兼容旧构造接口。
- `lyenbot.json`：按 URDF 原生拓扑列出 23 个名字；左右脚直接指向 URDF 已有 contact Frame。
- `initial_joint_positions`：是控制初值而非编码器零位；腿部采用轻微屈膝。
- `nominal_base_height=0.850693`：由初始关节姿态的 URDF 正运动学计算，使足底 z 为 0；原 0.78 会使足底穿地约 70.7 mm。
- `swing_time/touchdown_force` 等：是尚需继续调试的步态参数，不能当作实机参数。

### 6.4 `common/urdf_model_loader.h/.cpp`

- 二进制读取完整 XML：避免 locale 改写。
- 扫描 `="..."` 属性值并 trim：只修复解析兼容性。
- `trimmedAttributeCount`：把异常数量暴露给检查器和 manifest；当前为 4。
- `buildModelFromXML(..., JointModelFreeFlyer())`：构造浮动基模型。
- 固定基重载：用于身体坐标系下的 IK 和足端量。
- 文件中没有任何输出流写回 URDF；遗漏这一约束会违反只读真源原则。

### 6.5 `demo/generate_mjcf.cpp`

- `sha256()`：调用系统 `sha256sum` 读取源文件摘要；摘要写进 MJCF 注释与 `generation_manifest.txt`。
- `normalized_source.urdf`：只作为 MuJoCo 转换临时派生输入，原始 URDF 不变。
- meshdir 变换：使 generated 子目录能找到原 meshes。
- `wrapWorldBodyInFloatingBase`：MuJoCo 的 URDF 编译器会把根 link 固结到 world，此函数用 URDF 基座惯量重新包一层 freejoint。
- `insertSimulationDefaults`：只加入 timestep、数值阻尼、摩擦损失和 armature。
- `siteXml`：按 Pinocchio 足底固定 Frame 放置 touch site；额外透明薄盒为 C01 简化碰撞提供稳定支撑面，尺寸是仿真碰撞参数，不改变 URDF link。
- `motorXml`：每个名字生成一个 `gear=1` torque motor；正负 ctrlrange 来自 `model.effortLimit[idx_v]`。
- sensor 块：生成 quaternion、velocimeter、gyro、accelerometer 和左右 touch。
- keyframe：基座四元数用 MuJoCo 的 `wxyz`，关节按配置/Pin 顺序写入。
- scene 视觉块复用原项目的 `statistic`、蓝色渐变 skybox、棋盘地面、haze、headlight、全局视角和定向灯；地面摩擦仍读取 Lyenbot 仿真配置，因此视觉一致化不会覆盖动力学参数。
- 最终重新加载 scene：检查 `nq/nv/nu`，生成失败立即返回非零。

生成物位于 `models/lyn03-V07-urdf_C01-260414/generated/`。它们可以删除重建，不是第二份机械真源。

### 6.6 `demo/model_check.cpp`

- `require`：任何缺失或不一致抛异常并以非零退出。
- `nv == actuated+6`、`nq == nv+1`：检查浮动基维数。
- `requireFrame`：检查基座、双脚和双手语义名字确实存在。
- joint 循环：逐名检查唯一性、1 DoF、位置上下界、速度正值、effort 正值，并打印 `idx_q/idx_v`。
- `neutral + initialJointPositions`：构造同一初始姿态并计算足底和 CoM。
- MuJoCo 循环：逐名比较 position range 与 actuator ctrlrange。
- `lf-tc/rf-tc` 比较：Pinocchio Frame 与 MuJoCo site 的位置误差必须小于 `1e-6 m`。

### 6.7 `sim_interface/MJ_interface.h/.cpp`

- 新构造函数接收 `RobotModelConfig`；旧构造函数委托 Azure 默认配置。
- joint id 查询后读取 `jnt_qposadr/jnt_dofadr`：XML 排列改变也不会错位。
- actuator 循环比较 `actuator_trnid`：不再假设 actuator 名字或序号。
- `ctrl[jntId_dctl[i]]`：修复旧 `ctrl[i]` 的顺序耦合。
- 传感器 id 全部按配置查询，缺失必报错。
- 首帧 `baseLinVel=0`：避免从全零缓存到 0.85 m 产生数百 m/s 假速度。
- `basePos/baseLinVel` 写回 DataBus：原代码注释掉这 6 行会使状态估计和安全检查读取零。
- touch sensor 写入 `fL/fR.z`：让 Lyenbot 落地检测使用实际仿真接触而非 Azure 阈值。

### 6.8 `algorithm/pino_kin_dyn.h/.cpp`

- 配置构造函数同时建立浮动基/固定基模型；旧字符串构造函数委托 Azure 配置。
- 所有足、手、髋和基座实体改用 Frame id；固定 contact point 不再误当活动 joint。
- `motorQIndex/motorVIndex`：对每个 URDF 名字读取 `idx_qs/idx_vs`。
- `motorMaxTorque/Speed/Pos/MinPos`：直接复制 Pinocchio 从 URDF 得到的数组。
- `dataBusRead`：按显式映射把电机数组放进广义 `q/dq`，不再假定固定数字段。
- `dataBusWrite`：把运动学、动力学和 Frame 结果统一送回 DataBus。
- `getFrameJacobian/getFrameJacobianTimeVariation`：足底任务真正作用于 contact Frame。
- IK 初值按名字写入；腰部列按配置排除；Lyenbot 手臂只使用关节姿态任务。
- workspace limit 用 `motorQIndex` 查限制；否则改变关节顺序后会限制错误电机。

### 6.9 `common/PVT_ctrl.h/.cpp` 与 Lyenbot PVT JSON

- 新构造函数接收关节名和 URDF 产生的 4 个限制向量。
- JSON 只读 `kp/kd/PVT_LPF_Fc/gear`；不允许 JSON 覆盖 effort、velocity 或 position。
- 每个名字缺失立即抛异常，防止默认零增益静默运行。
- `maxTor/maxVel/maxPos/minPos` 复制自 Pinocchio。
- `calMotorsPVT` 的饱和仍使用 `maxTor`，所以实际输出不会超过 URDF effort。

### 6.10 `algorithm/wbc_priority.h/.cpp`

- 可选 config/model 参数：旧调用保持原数字布局，Lyenbot 使用名字生成索引。
- `gather`：按索引抽取任意长度手臂/腰/头状态。
- `zeroColumns`：按名字生成的速度索引排除自由度。
- Lyenbot `targetArmQ` 为 10 维；头列表为空时不把 `HeadRP` 加入优先级。
- `useFullFootContact`：配置化模型使用完整双足 6D 接触；Azure 保留旧接触投影。
- 站立 contact anchor：首次进入 WBC 时锁存左右足底世界位姿，并以 `errX/derrX` 消除离散漂移；这是 WBC 从约 8 秒跌倒变为 10 秒通过的关键。
- Lyenbot 站立增益采用较低刚度/较高阻尼；Azure 常数不变。
- QP 对配置化模型 reset、使用上次解作初值并放宽计算预算；失败时不会用错 actuator。
- walk stance/double-support anchor、较保守 Swing/Base 增益已加入，但尚未达到最终行走验收。

### 6.11 `algorithm/gait_scheduler.*` 和 `foot_placement.*`

- 默认 `FzThrehold` 恢复 280，Azure 行为不变。
- `useMeasuredContact`、`minimumTouchdownPhase`、`swingWasAirborne` 仅由 Lyenbot demo 启用：摆脚必须先离地再允许落地事件。
- `enableDoubleSupportTransfer` 默认 false：Azure 不进入新增状态；Lyenbot 落地后进入 DSt 并用 `transferPhi` 换重心。
- FootPlacement 的步长 clamp 默认无穷大：Azure 无变化；Lyenbot demo 显式设置有限值，阻止速度尖峰生成不可达落脚点。

### 6.12 `demo/lyenbot_staged.cpp`

- `pd/wbc/walk` 三个参数隔离阶段；任一状态 NaN、基座低于 0.35 m、roll/pitch 超过 1 rad 即安全停止。
- keyframe reset 后调用 `mj_forward`：否则首帧 body/site/sensor 位姿还没更新。
- PVT 用 config 名字和 URDF 限制构造。
- WBC 用 config/model 构造，所以不出现 Lyenbot 数字下标。
- 第 3 秒 WBC 接管时锁存当前 CoM 和当前高度，避免 9 mm 高度阶跃。
- `tailToStd` 只用于 Pinocchio 广义向量末尾的 23 个电机自由度。
- walk 可选第三参数 `mpc`；`walk` 是阶段 F，`walk mpc` 是阶段 G。
- 每秒打印姿态、基座、CoM、足底、步态相位、QP 和最大扭矩，失败日志可复现。

### 6.13 `sim_interface/GLFW_callbacks.*` 的 GUI 生命周期修复

- `window/image/file` 初始化为空：构造中途失败时析构仍安全。
- `glfwInitialized/renderResourcesInitialized`：只释放真正初始化过的 GLFW 和 MuJoCo 渲染资源。
- `~UIctr()`：关闭录像文件，释放像素缓存、render context、scene、window，最后终止 GLFW。
- `createWindow` 检查窗口创建结果；没有可用显示服务器时明确报错。
- `Close()` 只调用 `glfwSetWindowShouldClose`：不再从 GUI 回调删除属于 demo 的 `mjModel/mjData`，避免双重释放。
- `lyenbot_staged` 用 `std::unique_ptr<UIctr>` 保证正常结束和异常安全停止都会先释放 GUI，再释放 MuJoCo model/data。

用户原有 `record/matlabReadDataScript.txt` 修改和 `build-debug/` 未纳入本实现，也未覆盖。

## 7. 运行命令

从仓库根目录：

```bash
cmake -S . -B build
cmake --build build -j2

./build/generate_mjcf common/robot_configs/lyenbot.json
./build/model_check common/robot_configs/lyenbot.json

cd build
./lyenbot_staged pd
./lyenbot_staged wbc
./lyenbot_staged walk
./lyenbot_staged walk mpc
```

### 7.1 MuJoCo GUI 模式

GUI 是可选参数；不加 `--gui` 时仍运行无界面自动测试。必须从 `build/` 目录启动：

```bash
cd /home/yzx/Openloong/lyenbot_openloong
(cd build && ./lyenbot_staged pd --gui)
(cd build && ./lyenbot_staged wbc --gui)
(cd build && ./lyenbot_staged walk --gui)
(cd build && ./lyenbot_staged walk mpc --gui)
```

GUI 每约 `1/60 s` 仿真时间渲染一次。鼠标左键旋转、右键平移、中键或滚轮缩放；数字键 `1` 暂停/继续，数字键 `2` 单步。关闭窗口会输出 `LYENBOT_GUI_CLOSED`，不会冒充阶段验收成功。

生成的 Lyenbot scene 已与原项目 `models/scene.xml` 对齐主要视觉环境：蓝色渐变天空、棋盘地面、雾化、反射材质、灯光、azimuth/elevation 和 statistic 显示尺度。机器人自身颜色仍来自 Lyenbot URDF，这是模型材料差异，不应复制 AzureLoong link 材质覆盖。

`UIctr::Close()` 现在只设置窗口关闭标志；`UIctr` 析构函数负责释放 GLFW window、MuJoCo scene/context 和录像缓存，而 `lyenbot_staged` 继续独占 `mjModel/mjData` 的生命周期。这避免关闭窗口时 GUI 和 demo 重复释放模型。

### 7.2 为什么已有 build 还可以执行 `cmake -S . -B build`

该命令不是“删除并重建 build”，而是 CMake 的配置/更新步骤：

- `-S .` 指定源码根目录；
- `-B build` 指定已经存在或需要创建的构建目录；
- CMake 会读取 `build/CMakeCache.txt`，检查 `CMakeLists.txt` 是否变化并增量更新生成文件；
- 已编译的 `.o` 和未失效的可执行文件会保留；
- `cmake --build build -j2` 才执行编译，而且也只重编译发生变化的源文件及其依赖。

若 `build/` 已经使用 Release 配置完成，平时修改 `.cpp/.h` 后只需：

```bash
cmake --build build -j2
```

修改 `CMakeLists.txt`、增加 target 或不确定缓存是否已更新时，再执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

只有要彻底换编译器、处理损坏缓存或主动做干净构建时，才需要另建目录，例如 `build-release/`；正常增量编译不需要删除现有 `build/`。

注意：配置中的相对路径以 JSON 所在目录解析，但 Azure 旧默认构造仍沿用原项目从 `build/` 启动的约定。

## 8. 常见报错怎么理解

### `normalized_attribute_whitespace=4`

不是修改 URDF；表示加载器在内存中修剪了 4 个属性尾部空格。原文件 hash 应保持不变。

### `Geom with duplicate name ''`

MuJoCo 的 URDF 导入器提示多个未命名 geom。生成仍成功；这些名字不参与控制映射。

### `Safety stop`

这是预期保护，不应删除。先看时间、`base_z/rpy`、`feet`、`leg/phi` 和最大扭矩定位是接触、映射还是增益问题。

### QP working set 报错

检查 `qp_status`，不能只看控制器有没有输出。配置化路径会 reset 并给更充足预算；若持续发生，需记录对应状态和约束残差。

## 9. 后续完成 10 步的推荐顺序

当前不要直接继续调 MPC。应按以下顺序：

1. 新增 23 关节逐个小角度测试，自动比较 MuJoCo 正方向与 Pinocchio Frame 位移方向。
2. 为 Lyenbot 单独实现准静态一步测试：双支撑移 CoM → 单脚抬起 → 原地落下 → 双支撑，先不前进。
3. 记录单支撑期间真实 contact wrench、支撑足滑移、CoM 投影和 ankle effort 余量。
4. 将 FootPlacement 的 Azure 常量 `xOff/yOff/zOff` 彻底移入 Lyenbot 配置，并根据 URDF 足底/髋 Frame 计算初值。
5. 单步稳定后才启用连续双支撑 transfer，再验收 10 步。
6. 最后启用 MPC，先 0.03 m/s，再逐级提高到 0.15 m/s；每一级保存日志。

禁止的“捷径”包括：改 URDF 轴、改 joint limit、虚构减速比、放大 effort、关闭安全检查，或把跌倒前发生过 10 次相位切换当作 10 个有效脚步。

## 10. 本轮验证记录

```text
URDF SHA-256:
86cc7c32154cfc938337ec7eaf77a42c3146fc39b91f214fb87ec5c2c874ce22

MODEL_CHECK_OK
mujoco_nq=30 mujoco_nv=29 mujoco_nu=23
foot_pose_error=4.64746e-07 4.6466e-07

PD:
LYENBOT_STAGE_OK time=10.001 base_z=0.848443 steps=0

WBC:
LYENBOT_STAGE_OK time=13.001 base_z=0.848591 steps=0

AzureLoong regression:
all original CMake executable targets built successfully

Walk:
failed during first-step/single-support lateral stabilization; safety stop worked
```
