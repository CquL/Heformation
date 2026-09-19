# GRSTAPS 2021/2022：任务、分配、调度与运动可行性的信息交换

> **2026-09-19 更新：本轮实际访问 SAGE 在线正文并核对 §4.4、§4.5、§5 实验设置及 §6.1。** 本地仍无 PDF，未声称阅读全文。旧摘要级记录保留为历史访问情况，不再断言当前正文不可访问。

本轮核对：调度考虑作业时长、转场及协作运动，运动查询会反馈调度。原实验采用二维 Lazy PRM，不含动力学，并明确不解决机器人之间的运动碰撞；不能据此声称三维 qn 集群已具备安全保证。来源：[SAGE 正文](https://journals.sagepub.com/doi/full/10.1177/02783649211052066)，对应 §4.4/4.5、§5 设置与 §6.1。我们仅借鉴反馈关系，不移植其调度器。

以下是此前摘要级笔记，其阅读限制和阶段建议属于历史记录。

## 文献记录

- 作者：Andrew Messing*, Glen Neville*, Seth Hutchinson, Sonia Chernova, Harish Ravichandar（* 同等贡献）
- 年份：2022（IJRR；在线版 2021）
- 发表：The International Journal of Robotics Research (IJRR), 2022，DOI 10.1177/02783649211052066
- 落地页（全文付费）：https://star-lab.cc.gatech.edu/papers/messing-grstaps/
- 本地 PDF：无（`pdf_status: not-available`）
- 阅读范围：仅摘要（落地页 Abstract；未读正文）

## 该文献的保证与成立假设（据摘要，不是定理）

- 摘要把问题形式化为 **STAP-STC**（Simultaneous Task Allocation and Planning with Spatiotemporal Constraints）：同时回答 what（任务规划）、how（运动规划）、who（任务分配）、when（调度）四个问题。
- 方法是 GRSTAPS：任务规划、分配、调度、运动规划**交错进行**（interleave），以多层搜索在模块之间共享信息；任务层采用 agent-agnostic planning（先不绑定具体机器人），分配层用搜索同时满足规划约束和任务需求并优化调度。
- 摘要声称在仿真应急响应场景中做消融与对比实验，结论是其性能在计算时间、解质量和问题覆盖率上优于消融基线与当时的最先进时序规划器。
- **无法核实的部分**：摘要未给出求解器的完备性/最优性假设、时间/运动可行性接口的具体形式、边界条件或证明；本笔记不宣称任何这类性质。若后续要作为选型依据，必须获取正文并重新核验。

## 该文献改变当前哪个接口定义？

1. **`travel_time_provider` / `used_reference`**：GRSTAPS 的核心主张正是"分配-调度-运动"必须**双向交换信息**，而不是上层拿一个标称旅行时间做静态调度、下层事后执行。本项目当前 `travel_time_provider(executor, from_target_ref, to_target_ref)` 用编队中心距离/名义速度返回单一标量估计；这可以作为第一版估计来源，但接口语义必须写明它是**估计值、不是可行性证明**，并且计划时间在运动执行反馈回来之前是暂定的。
2. **`PlanItem` 时间语义**：文献约束我们：任务的 `planned_start/planned_finish` 需要能表达"基于标称估计的暂定值"，并在实际执行（DelayEvent/repair 或重新查询）后更新；不能把暂定时间当成已验证的时空约束。
3. **不改变的部分**：`Plan.items` 作为唯一权威、固定联盟与串行单资源模型不需要改；GRSTAPS 的 agent-agnostic planning 与本项目的固定七机联盟不同，不要求照搬。

## 该文献要求增加哪个反例测试？

**测试：`test_nominal_travel_time_estimate_is_not_schedule_feasibility_proof`**（建议放 `integration/mrta_python/tests/test_schedule.py` + `test_repair.py` 的串联用例）

> 说明：GRSTAPS 正文未读，以下测试是按摘要主张（交错、信息共享、时空约束）设计的**我们自己的**反例测试，不是论文中的实验或定理。

- **setup**：stub 的 `travel_time_provider` 对某条 leg 返回 2.0 s；同一 leg 在仿真中真实耗时 6.0 s（例如队形通过狭窄区段或逆流）。计划先按 2.0 s 排定 `E_next.planned_start`。
- **stimulus**：执行该 leg，产生 `DelayEvent(actual_finish = planned_finish + 4.0)`；调用 `process_completion`/`plan_repair`，然后检查调度器是否仍以旧的 2.0 s 估计作为后续依据。
- **expected observation**：修复后的计划中，所有依赖该 leg 的后继 `planned_start >= actual_finish`（延迟被真实传播）；同时任务层必须能标记“该 leg 的运动可行性尚未重新核验”（如 `schedule_feasibility=UNVERIFIED`），不得把 `travel_time=2.0` 的标称值继续当作满足时空约束的证据。若只有时间平移、没有可行性状态，则测试要求至少显式记录这个缺口。
- **为什么是反例**：它阻止"标称估计 = 可行"的静默假设，正是 GRSTAPS 摘要中"交错与信息共享"要解决的问题；也防止我们仅凭 repair 后的时间表就宣称联合规划已闭环。

## 我们实际复用什么

- "分配/调度与运动可行性必须交换信息"作为接口设计方向：当前保留标称估计 + 实际延迟反馈两条通道，不虚构局部规划器的提前代价查询能力（与 AGENT.md §5 一致）。
- 调度时间基于估计、可修复的语义：`planRepair` 更新 `planned_start/finish` 是正常路径，而不是异常。
- 多层搜索/agent-agnostic planning 只作为后续任务-运动联合优化（如 D-ITAGS 对照或本组研究增量）的参照，不改变本轮 Calvo v9 受限端口为唯一在线调度器。
- 其"运动可行性反馈可能需要重新求解分配"的思想，支持我们在 P3 之前保持固定七机联盟、不提前实现动态联盟。
- 明确边界：文献为付费墙、仅摘要阅读；不作为当前选型、保证或验收门槛的依据。
