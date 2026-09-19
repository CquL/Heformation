# 海上异构无人集群：项目背景与 AI 协作入口

**更新：2026-09-19**

本目录用于给 AI 和开发者提供项目的稳定背景、当前事实、接口边界、文献依据与接续入口。它不是机器人运行时协议，也不是“写在这里就代表已经实现”。

## 当前目标

最终目标系统：

```text
3 AAV + 1 USV + 1 UUV
```

母船负责上层任务与资源协调，各平台本地执行运动规划、控制和状态闭环。

当前主推进不是七机业务系统，而是：

```text
近岸监测 AAV 子系统
→ 3 AAV
→ 单机 / 三机编队两种执行方式
→ 高层请求驱动
→ 实际 qn 状态决定监测完成
```

七机配置保留用于 M2 与回归。

## 当前代码状态基线

整理本背景时远端：

```text
repo: CquL/Heformation
branch: main
commit: 79036d66178deacada2b4f4a30dc588899eeb094
```

已完成的重要工程事实：

- 三机声明式编队图；
- 单机 / 组级规划模式切换；
- 单机→组级→单机 Action 全生命周期探针；
- 任务线串行计划；
- 共享物理成员预测状态；
- `observed AND received` 交付不变式；
- 全区间队形与岸线走廊判据；
- 240 项单元测试通过。

尚未完成的重要连接：

- runner 的 Executor 规划模式；
- 按 PlanItem 选择真实 Action endpoint；
- 用户确认开始；
- 一次完整近岸监测在线闭环；
- 任务层实时覆盖/交付展示；
- M2 最后两个实跑场景；
- USV/UUV 在线后端和真实通信。

## 目录职责

```text
AGENT/
├── AGENT.md             稳定工程规则与最小入口
└── PROJECT_CONTEXT.md   本说明

context/
├── 01_project_background.md
├── 02_current_status.md
├── 03_system_architecture.md
├── 04_inputs_outputs.md
├── 05_state_communication.md
├── 06_dynamics_and_theory.md
├── 07_reproduction_integration.md
├── 08_first_scenario_benchmark.md
├── 09_implementation_plan.md
├── 10_research_novelty.md
├── 11_glossary.md
├── 12_literature_review.md
├── 13_references.md
├── 14_decisions_and_unknowns.md
├── 15_handoff.md
└── 16_upstream_reproduction_status.md
```

`06/07/16` 仍可作为理论、复现和独立上游台账，不应把其中历史候选自动提升为当前主链。

## 最小阅读入口

```text
AGENT/AGENT.md
→ context/02_current_status.md
→ context/15_handoff.md
```

然后按问题阅读对应专题。

## 当前路线

```text
先把三机任务线真正贯通
→ 完成近岸监测 AAV 子系统验证
→ 独立收尾七机 M2
→ 再接 USV/UUV 平台后端
→ 再进入真实通信窗口 / 中继 / 能量联合安排
→ 用实验暴露的问题定义自己的研究方法
```

不要因为目标架构包含通信、USV、UUV 或能量，就把它们写成已经在线实现。
