# Calvo & Capitán 2024：延迟传播、等待吸收、repair 与全量重规划

## 文献记录

- 作者：Álvaro Calvo, Jesús Capitán
- 年份：2024（arXiv:2411.02062v1；本笔记读的是 v3, 2025-11-26）
- 发表：arXiv preprint arXiv:2411.02062 [cs.RO]（尚未见正式会议/期刊出处）
- 链接：https://arxiv.org/abs/2411.02062
- 本地 PDF：`papers/calvo-2024-delay-propagation.pdf`（10,224,834 B，sha256 `4ca83f3…bbeaf`）
- 阅读范围：全文（pdftotext 正文；本笔记不引用未读内容）

## 该文献的保证与成立假设

论文不是一条“repair 一定成功”的定理，其有效范围由执行架构和 repair 过程本身定义：

- 集中式 Monitor 持续接收各 Behavior Manager 的完成/失败/延迟反馈；延迟在“任务完成时该机器人能否按计划到达下一个任务起点”这一检查点上被发现（§VI）。
- repair 的前提是**不重新分配任务**：保持任务归属，只改时间；目标是最小化 makespan 扩展（§VI-A）。
- 延迟先由该机器人后续 slot 的**等待时间**吸收（`Tr,s^w` 减少），剩余延迟沿时间轴向前传播；传播到同步/中继协调点时，由联盟中剩余延迟最大的机器人决定新的共同时间（Algorithm 4–6）。
- 只有三类情形才全量重规划：出现新任务、机器人故障导致计划不可行、repair 失败（§VI）。
- 任务进行中不打断、不重新分配（除非联盟人数不足导致无法完成）。
- 实验层面：仿真 UAV 太阳能电站巡检场景、Gurobi 小规模对照与在线 repair 演示；这些数字不构成我们系统的保证。

**与本项目的直接差异（必须显式记录）**：本版 `wait_time = 0`，所以 Calvo 式“先吸收等待时间”这一吸收机制在本项目默认关闭；语义上任何正延迟都不应被假设为可吸收。

## 该文献改变当前哪个接口定义？

1. **`DelayEvent` / `planRepair`**：当前 `DelayEvent` 只携带 `planned_finish`、`actual_finish`，`plan_repair` 只沿 `item.coalition[0]` 的队列平移后继。Calvo 的语义是“延迟属于每个机器人 slot，先吸收、后传播，并在同步/中继点重新对齐”；因此接口至少应表达：
   - 本次 repair 是“已吸收”（等待时间被消耗）还是“已传播”（后续任务顺延）；
   - 传播影响到的**所有联盟成员队列**，而不只是代表机器人；
   - repair 失败与“计划失效需全量重算”的显式结果（现在 `plan_repair` 在无法平移时直接抛 `ValueError`，没有结构化的失败判据）。
2. **executor/resource model**：Calvo 的调度单位是“每个机器人一串 slot（含等待时间）”，多个机器人通过同步协调点对齐。本项目 `PlanItem` 是 execution 级、联盟为元组、`wait_time=0`，没有 per-robot slot 和同步协调点的显式结构；文献约束我们：要想复用其修复语义，必须能按成员展开队列，并标明哪些 slot 之间存在同步/依赖。
3. **不改变的部分**：`Plan.items` 仍是唯一权威、repair 只改时间不换人、同一 `execution_id` 事件只处理一次——这些与 Calvo 一致，不需要改。

## 该文献要求增加哪个反例测试？

**测试：`test_repair_propagates_delay_to_every_coalition_member_queue`**（建议放 `integration/mrta_python/tests/test_repair.py`）

- **setup**：构造 `Plan`：`E1` 为七机/多机任务，`coalition=("d0","d1")`，`planned_finish=10.0`，`wait_time=0`，`status="RUNNING"`；`E2` 为 `d1` 的单独后继（`coalition=("d1",)`，`planned_start=10.0`，`planned_finish=13.0`）；`E3` 为 `d0`/`d1` 的后续任务（`coalition=("d0","d1")`，`planned_start=10.0`）。
- **stimulus**：对 `E1` 调用 `process_completion(...)`，事件 `actual_finish=13.0`（相对计划延迟 3 s，超过 tolerance），再调用 `plan_repair`。
- **expected observation**：对 `E1.coalition` 的**每一个**成员，其队列中所有 `PLANNED` 后继的 `planned_start` 都必须 `>= 13.0`（`E2.planned_start >= 13.0` 且 `E3` 在两个成员视角下一致）；不得出现“`E3` 平移了、`E2` 仍停在 10.0”的半修复计划。若当前模型无法展开 per-member 队列，则 `plan_repair` 必须返回结构化失败（如 `repair_ok=False, reason="MEMBER_QUEUE_UNREPRESENTED"`）而不是抛出未捕获异常或返回不一致计划。
- **为什么是反例**：现在 `plan_repair` 只遍历 `representative_robot = item.coalition[0]` 的队列，`d1` 的独立后继 `E2` 不会被平移，正好命中 Calvo 指出的“延迟只在一个机器人时间线上传播”的错误实现。

## 我们实际复用什么（不含未采用的充电/relay/碎片化）

- repair 只在完成事件后触发、只改时间不换人，与当前 `process_completion` 的冻结语义一致。
- 三类全量重规划触发条件（新任务、计划失效、repair 失败）可直接作为 `planRepair` 的返回状态设计依据。
- “延迟在任务完成检查点检测”与我们的组级 `DelayEvent` 产生时机一致；`event_id`/`execution_id` 幂等去重可以保留。
- 协调点由剩余延迟最大者对齐的规则，是多机 `FormationAction` 组级延迟传播的可借鉴语义。
- 其集中式 Monitor 监督模型与本项目 runner/Action server 单资源、单调度器的边界一致，不引入第二套执行框架。
- 明确记录差异：等待吸收关闭（`wait_time=0`）、电池/充电/relay/fragmentation/动态联盟人数不在本版范围。
