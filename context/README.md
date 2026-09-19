# 项目背景索引

**更新：2026-09-19。**

当前目标平台是 **3 AAV + 1 USV + 1 UUV**。  
当前主推进是三 AAV 的任务驱动近岸监测闭环；七机配置保留为 M2 / 回归基线。

| 文件 | 作用 |
|---|---|
| [01 项目背景](01_project_background.md) | 项目目标、最终平台组成、研究边界 |
| [02 当前状态](02_current_status.md) | 现在真实做到了什么、哪些尚未接通 |
| [03 系统架构](03_system_architecture.md) | 目标功能架构与当前实现架构 |
| [04 输入输出](04_inputs_outputs.md) | MonitoringRequest、Executor、Action、观测与交付 |
| [05 状态与通信](05_state_communication.md) | 本机状态、邻机消息、数据产生/接收、未来通信约束 |
| [06 动力学与理论](06_dynamics_and_theory.md) | 平台模型、安全与理论边界 |
| [07 复现与集成](07_reproduction_integration.md) | 上游复现与项目适配如何区分 |
| [08 第一场景](08_first_scenario_benchmark.md) | 近岸监测冻结实例与验收 |
| [09 实施路线](09_implementation_plan.md) | 当前下一步和停止边界 |
| [10 研究问题](10_research_novelty.md) | 候选创新问题，不预先宣布新理论 |
| [11 术语](11_glossary.md) | Executor、Observation、Delivery 等统一含义 |
| [12 文献](12_literature_review.md) | 当前真正影响设计的核心成果 |
| [13 引用](13_references.md) | 核心论文、源码和资料入口 |
| [14 决策与未知](14_decisions_and_unknowns.md) | 已冻结决定与尚待验证项 |
| [15 最新交接](15_handoff.md) | 最新实现、测试、已知边界、唯一下一步 |
| [16 上游复现台账](16_upstream_reproduction_status.md) | 各候选项目的独立复现状态 |

使用原则：

- 当前事实以 `02`、`15` 和 `docs/WORKLOG.md` 为准。
- 稳定决策以 `14` 为准。
- 目标架构不等于当前已实现系统。
- V2 历史文件和旧 `plan.md` 中与最新用户决定冲突的内容不再作为当前事实。
