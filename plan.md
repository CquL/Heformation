# 第一阶段实施计划：Calvo v9 restricted-domain Python port 与七机 AIR 闭环

更新：2026-09-17。按用户最新冻结决定实施；完成状态以 `context/02_current_status.md` 和 `context/15_handoff.md` 的实际运行记录为准。

## 1. 目标与范围

当前唯一主链为：

```text
Calvo v9 restricted-domain Python port
→ FormationAction
→ Swarm-Formation 七机 AIR 编队
→ PositionCommand
→ qn RBF/PD、执行器与 6DOF 动力学
→ 实际 Odometry 完成判定
→ DelayEvent
→ planRepair
→ 更新 Plan
```

原始 MATLAB 项目保留在 `upstream/Calvo-MRTA/`，本项目移植版位于 `integration/mrta_python/`。正式名称固定为 **Calvo v9 restricted-domain Python port**；它不代表完整 MATLAB 等价复现。

第一阶段固定七台成员 `0…6` 为一个执行单元，只执行 `FORMATION_MOVE_AND_HOLD`。本版 `wait_time = 0`；不添加人为等待，正等待吸收与多逻辑执行单元资源竞争留待后续扩展。仍实现自由任务排序、能力匹配、时间计划和实际延迟传播。

固定映射：`Relayability = 0`、`nf = 1`、`N = required_agent_count`、`Hr = capability compatibility matrix`、`Te = service_time`、`tmax = deadline`。电池为不形成约束的固定仿真假设；充电、fragmentation、relay、动态联盟人数关闭。这些兼容字段只在确有需要时进入内部计算，不进入 ROS 外部接口。

不实现 version 13 剩余任务重新分配。D-ITAGS/Gurobi、OmniPlan ROS2、Primitive/EGO 替换、WATER/TRANSITION、编队拆分、电池与充电、DANCERS 完整联合仿真均属于后续对照或扩展；不构成本轮前置条件。

## 2. MRTA 与唯一计划状态

沿用用户冻结的数据字段：

| 类型 | 字段 |
|---|---|
| `Agent` | `id: str`、`capabilities: frozenset[str]`、`available_from: float` |
| `Task` | `task_id: str`、`required_capabilities: frozenset[str]`、`required_agent_count: int`、`service_time: float`、`deadline: float`、`target_ref: str` |
| `PlanItem` | `execution_id: str`、`task_id: str`、`coalition: tuple[str, ...]`、`planned_start: float`、`planned_finish: float`、`travel_time: float`、`wait_time: float`、`service_time: float`、`status: str` |
| `Plan` | `items: list[PlanItem]` |
| `DelayEvent` | `event_id: str`、`execution_id: str`、`task_id: str`、`planned_finish: float`、`actual_finish: float` |

`Plan.items` 是唯一权威计划状态。`robot_queues`、`coalitions`、任务起止时间、makespan 和 waiting_times 均即时派生，不维护独立可变副本。

`planned_start` 表示执行单元占用任务并开始转场；`travel_time` 是转场到作业区的估计时间；`wait_time` 表示到达后的同步等待，本版恒为零；`service_time` 是实际编队驻留要求。保持：

```text
planned_finish = planned_start + travel_time + wait_time + service_time
```

空间只通过 `target_ref` 引用。独立的 `travel_time_provider(executor, from_target_ref, to_target_ref)` 以编队中心欧氏距离除以名义编队速度估算，不向 MRTA 核心写入 Swarm 或 ROS 依赖。未来替换路径估计或历史耗时模型时保持该边界。

按 v9 原码逐项移植启用的动态奖励：deadline、execution time、introduced makespan、introduced waiting time、capability/compatibility ratio、travel time，以及固定 seed 的可重复 tie-break。当前 waiting term 按零等待退化；不使用另一套自拟优先级公式替代原奖励。

校验能力兼容、固定七人成员集合、时间恒等式和同一执行单元任务无重叠。截止时间沿用 v9 模型语义，不擅自增加一般任务前置关系、release time 或时间窗。初始计划和每次修复后均校验，不能用内部队列掩盖 `Plan.items` 不一致。

## 3. ROS1 Action 与执行反馈

在现有 Noetic Docker 中使用普通 ROS1 actionlib，不使用 rosbridge_suite，也不迁移 ROS2。`Formation.action` 仅包含：

```text
Goal: task_id, geometry_msgs/PointStamped formation_center, duration hold_duration
Feedback: phase = MOVING | HOLDING
Result: actual_start_time, actual_finish_time, reason
```

目标必须满足 `frame_id = world`、`z = 0.5 m`，否则 `ABORTED / INVALID_TARGET`。节点 YAML 或启动参数提供 `agent_ids = [0,1,2,3,4,5,6]`、`relative_slots`、`swarm_scale`、`epsilon_p`、`epsilon_v`、`odom_timeout`、`execution_timeout`。

任务适配器只校验任务、发布一次现有 Swarm 组级目标、订阅七台 qn Odometry、监测完成并返回结果。它不发 `PositionCommand`，不直接控制 qn，不修改 Swarm 规划器。正常任务串行派发，每次动作使用独立 `execution_id`，并与 actionlib GoalID 对应；计划时间与实际时间统一后再比较。

每台最终位置为 `formation_center + swarm_scale * relative_position_i`。监测实际最大位置误差、最大速度，以及每台 Odometry 时间戳。只有所有成员同时满足位置、速度、新鲜度要求，并连续保持 `hold_duration`，才返回 `SUCCEEDED`；任何一次条件失效均不能累计为有效驻留。

`planning/finish` 只记为 `planner_nominal_finish_time`。名义结束后实际状态未达标时继续监测，直到真实完成或 `execution_timeout`。顶层失败码仅为 `INVALID_TARGET`、`ODOMETRY_TIMEOUT`、`EXECUTION_TIMEOUT`；位置/速度未收敛、提前名义结束、失败成员及误差/间距等进入 diagnostics 与实验记录。

## 4. 延迟修复

用实际完成结果生成一次组级 `DelayEvent`，计算 `delta_t = actual_finish - planned_finish`。只有 `delta_t > delay_tolerance` 且该事件未处理时调用 repair；零延迟、负延迟、容差内延迟及重复事件不改变计划。

内部映射为 `execution_id → task_id → delayed_robot → delayed_slot → delay_seconds`。slot 仅在 Python 内部存在。七机同做一个任务时只处理一次延迟，不分别给七名成员叠加；不得把同一增量同时写入 `service_time`、`available_from` 和 `delay_seconds`。

repair 只传播已有协作关系上的时间影响，保护已完成任务，保持执行中任务成员不变，允许更新其预计结束；未开始任务可更新计划时间，但不重新分配成员或顺序。本版零等待下，正延迟直接传播到后续任务；无法产生合法修复时明确报告失败，保留可诊断结果。

小延迟和大延迟均在真实执行结果驱动下测试，不能只修改结果时间戳制造闭环成功。后续引入合法正等待时，再验收 `delay <= wait_time` 的等待吸收及剩余延迟传播；不把该项记为本版已通过。

## 5. 实施顺序与验收

1. 审核并完善现有模型、Plan 派生视图、输入与无重叠校验；实现受限 v9 动态奖励、旅行时间估计和可复现 tie-break。
2. 完成幂等 DelayEvent 与 v9 repair 时间传播子集，先运行 Test B 的可手算自由任务算例。
3. 完善 FormationAction、七机实际 Odometry 完成判定与异常路径，完成 Test A 单动作 AIR 验证。
4. 执行固定 `T1：A 区移动驻留 → T2：B 区移动驻留 → T3：返回起点驻留`，核对独立 GoalID、实际结果与计划时间。
5. 完成 Test C：MRTA 派发决定顺序和时刻，实际小/大延迟触发一次 repair，后续执行使用更新后的 Plan；保存可复查记录。

| 测试 | 必须验证 | 不能据此声称 |
|---|---|---|
| Test B：Python MRTA | 手算奖励/顺序/时间、能力不匹配拒绝、固定联盟、无重叠、Plan 派生一致、已完成保护、零延迟不变、重复事件幂等、固定 seed 可重现、正延迟传播 | 完整 MATLAB 等价、一般前置关系优化、多执行单元资源竞争 |
| Test A：FormationAction | 一次组级目标经过 Swarm→PositionCommand→qn→七机 Odometry 后返回结果；连续驻留；名义 finish 不提前成功；过期里程计不能成功；非法高度/超时失败 | MRTA 算法效果、水下或跨介质验证 |
| Test C：完整闭环 | 三个独立 execution_id；任务顺序和时间来自 Plan；真实实际完成时刻产生 DelayEvent；修复确实改变剩余 Plan 并影响后续派发；事件只计一次 | 固定 A→B→Return 是 Calvo 前置约束优化，或零等待例验证了等待吸收 |

实验保存至 `experiments/`：必要的 `metrics.json`、CSV、rosbag、命令与简短结果说明；指标不重复塞入 Action Result。记录实际命令/状态、名义与实际完成差、编队误差、速度、最小机间距及失败原因。测试未执行或失败时如实记录。

第一阶段只有在受限 v9 算例、七机 Action 实际完成、A→B→Return、真实正延迟闭环、幂等性及记录均通过后才完成。当前计划文档落地不代表这些验收已经通过。
