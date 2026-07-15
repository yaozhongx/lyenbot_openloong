# Lyenbot OpenLoong 项目 Codex 开发规范

## 1. 项目说明

本项目是基于 OpenLoong-Dyn-Control 的 Lyenbot 机器人适配工程。

主要目标：

- 保留 OpenLoong 原有控制框架；
- 替换为 Lyenbot URDF 模型；
- 建立对应 MuJoCo 仿真模型；
- 适配 Pinocchio 动力学模型；
- 适配 MJ_Interface、DataBus、WBC、MPC 控制流程；
- 完成人形机器人稳定站立、行走控制。


---

# 2. 文档语言规范

所有 Codex 生成和维护的工程文档必须使用中文。

包括：

- CHANGELOG.md
- DEBUG_LOG.md
- DESIGN_NOTES.md
- TODO.md
- LYENBOT_ADAPTATION.md


要求：

- 标题使用中文；
- 内容使用中文；
- 技术名词可以保留英文；
- 代码、函数名、变量名保持原格式。


例如：

允许：

> MuJoCo actuator 顺序与 Pinocchio joint 顺序不一致。


禁止：

> Modify actuator mapping because...


所有工程记录必须保证后续开发人员可以理解。


---

# 3. 代码修改原则


## 修改前

任何代码修改前必须说明：

1. 当前问题；
2. 问题原因分析；
3. 修改方案；
4. 涉及文件；
5. 影响范围。


不得直接修改代码而不解释。


---

## 修改原则

必须遵守：

- 最小修改原则；
- 保留 OpenLoong 原有架构；
- 不随意重构无关模块；
- 不删除已有功能；
- 优先采用适配层。


新增 Lyenbot 相关代码需要添加标记：

```cpp
// LYENBOT MODIFY
```

标记应放在 Lyenbot 专用适配逻辑附近。若修改的是机器人无关的共享修复，不应机械地给每一行添加标记，应在 `CHANGELOG.md` 说明影响范围。

---

# 4. 工程文档按需读取规范

## 默认行为

新对话默认只需要先阅读本文件 `AGENTS.md`，不得每次启动都完整读取 `codex_notes/` 下的五份长文档。

开始任务后，应先判断任务类型，再使用标题搜索定位相关章节，例如：

```bash
rg -n '^#{1,4} ' codex_notes/*.md
```

只读取解决当前任务所需的文件和章节。

## 首次接续 Lyenbot 开发

只读取以下最小内容：

1. `codex_notes/LYENBOT_ADAPTATION.md` 的“当前结论”；
2. `codex_notes/TODO.md` 的“当前验收总览”；
3. 与用户当前目标对应的 TODO 条目。

不要因为用户提到 Lyenbot，就自动读取全部五份文档。

## 按任务类型读取

| 当前任务 | 需要读取 | 不需要默认读取 |
|---|---|---|
| 了解当前进度 | `LYENBOT_ADAPTATION.md` 当前结论、`TODO.md` 验收总览 | 完整 CHANGELOG、完整教学报告 |
| 排查已知 Bug | `DEBUG_LOG.md` 对应问题编号，再读相关代码 | 其他无关调试条目 |
| 修改架构或数据流 | `DESIGN_NOTES.md` 对应章节、相关源码 | 完整 DEBUG_LOG |
| 修改 Lyenbot 模型/映射/WBC | `LYENBOT_ADAPTATION.md` 对应章节、相关 TODO 和源码 | 无关历史记录 |
| 查看历史修改原因 | `CHANGELOG.md` 对应日期或模块 | 全部历史条目 |
| 学习逐语句原理 | 用户明确需要时读取 `doc/lyenbot_model_adaptation_zh.md` 对应章节 | 默认不读取整份教学报告 |

## 文档更新要求

代码修改完成后按实际影响更新文档：

- 所有代码修改：更新 `codex_notes/CHANGELOG.md`；
- Bug 排查或解决：更新 `codex_notes/DEBUG_LOG.md`；
- 架构、模块关系或数据流变化：更新 `codex_notes/DESIGN_NOTES.md`；
- 任务状态变化：更新 `codex_notes/TODO.md`；
- Lyenbot 模型、映射、运行方式或控制适配变化：更新 `codex_notes/LYENBOT_ADAPTATION.md`。

只更新受影响的文档和章节，避免把同一段内容复制到多个文件。

## 例外

只有在用户明确要求“全面审查”“文档整理”“生成交接总结”时，才扫描或读取全部工程记录。
