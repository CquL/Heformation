# APEX-MR 2025：预测时间与实际动作释放条件

## 文献记录

- 作者：Philip Huang*, Ruixuan Liu*, Shobhit Aggarwal, Changliu Liu, Jiaoyang Li（* 同等贡献）
- 年份：2025
- 发表：Robotics: Science and Systems (RSS) 2025, DOI 10.15607/RSS.2025.XXI.098；arXiv:2503.15836（v3, 2025-08）
- 链接：https://arxiv.org/abs/2503.15836
- 本地 PDF：`papers/mrta/apex-mr-2025.pdf`（15,267,831 B，sha256 `49973e7…6ab0`）
- 阅读范围：全文（pdftotext 正文）

## 该文献的保证与成立假设

- 输入是一套顺序的多机器人任务 + 运动计划；APEX-MR 把它后处理为**多模态 temporal plan graph (TPG)**：type-1 边连接同一机器人前后节点，type-2 边表示跨机器人前驱关系（任务依赖或运动依赖）——"机器人 i′ 必须等 i 到达某节点后才能动"（§III-C）。
- 保证：**TPG 无环 ⇔ 执行调度无死锁**；动作可安全执行的条件是“不存在任何未执行节点的入边”（§III-C, §III-D）。
- 执行时 TPG 托管在中央服务器，逐节点发给各机器人 action queue；节点时间戳只用于构图，**执行阶段忽略**，因此控制器延迟/不确定性不会破坏调度（§III-D）。
- 运动依赖来自对所有跨机器人节点对的碰撞检查（含所持对象与机器人运动学变化）；若漏检一对可能碰撞的节点，安全性就失去依据。等待位姿假设：机器人在 support pose 等待时不会阻塞他人（§III-C 脚注 1）。
- 可选 shortcut 只能删除冗余等待，且每一步都做碰撞检查以保持无环/无碰撞（§III-C "Further Optimization"）。
- 实验为双臂 LEGO 组装（仿真 + 实机），证明异步执行减少等待；不是本项目七机 AIR/海洋平台的现成验证。

## 该文献改变当前哪个接口定义？

1. **`FormationAction` 的释放条件**：APEX-MR 的规则是“前驱实际执行完成 → 后继才可执行”，时间戳不构成释放条件。本项目 `Formation.action` 只有 `task_id`、`formation_center`、`hold_duration`；runner 必须在内部持有显式的“前驱完成”门控（真实 Action Result + 组级完成判定），不能因为 `planned_start` 到了就 dispatch 下一个 Goal。
2. **`DelayEvent` / `planRepair`**：TPG 把“延迟”处理为偏序约束的自然等待，而不是把全局时间轴平移；只有当延迟破坏依赖（例如必须同步的动作）时才需要改计划。本项目目前只有一个组级 `DelayEvent`，缺少“哪个成员/哪个节点延迟、影响哪些后继”的粒度；文献约束我们：至少要能区分“不影响后继释放的延迟”和“必须改变后续时间的延迟”，并把前者记为 observation 而非 repair 触发。
3. **`executor/resource model`**：APEX-MR 的 action queue 是 per-robot 的，释放由入边决定；本项目 ActionServer 是单一资源、一次一个组级 Goal（plan.md P1）。二者不冲突，但意味着多成员依赖目前被折叠成一个组级事件；若后续要表达部分成员先动，需要先扩行动作粒度，而不是在现有 Goal 里塞时间戳。
4. **`group-level completion`**：组级完成 = 所有成员的 action 节点都执行完（对应七机连续到位驻留），与 plan.md 的动力学完成判定一致，不需要改成时间戳判定。

## 该文献要求增加哪个反例测试？

**测试：`test_action_release_requires_predecessor_completion_not_planned_time`**（建议放 `integration/qn_aav_simulator/tests/test_formation_action_server.py` 或 runner 测试）

- **setup**：构造两个串行任务 `E1`、`E2`（同一七机联盟），`E2.planned_start == E1.planned_finish`；`E1` 处于 `ACTIVE`，其 Action Result 尚未返回（无 `actual_finish`）；让模型/ROS 时间前进到超过 `E2.planned_start`。
- **stimulus**：调用 runner 的调度/tick 逻辑（或推进仿真时钟并等待）。
- **expected observation**：不得发送第二个 Goal；ActionServer 仍为 `ACTIVE`（不是 `READY_IDLE`），`E2.status` 仍为 `PLANNED` 而不是 `RUNNING`，不得建立 `E2` 的 `trajectory_id` adoption；只有收到 `E1` 的终态 Result（成功或失败）之后，才允许释放/重规划 `E2`。若代码存在“计划时间到就派发”的路径，该测试必须失败。
- **为什么是反例**：它直接检验 APEX-MR 的核心规则——计划时间戳不是释放条件；也对应 plan.md 中“repair-on/off 都必须等待上一动作真实完成”。

## 我们实际复用什么

- 前驱完成后继才释放的偏序执行语义；与本项目“完成驱动 + 单资源 ActionServer”天然一致。
- 延迟可以由偏序自然吸收、不必每次全量重规划的思想：用于决定 `DelayEvent` 何时真正触发 `planRepair`。
- “时间戳只作规划用、执行时忽略”的原则，支持我们不以 `PositionCommand.stamp` 作为任务开始/完成证据（plan.md P0 已禁止）。
- TPG 无环 ⇔ 无死锁的条件，可作为后续多资源/多执行单元扩展时的静态检查（当前串行单资源场景不需要）。
- 语义边界：APEX-MR 假设等待位姿非阻塞、依赖图完整且碰撞检查覆盖所有可能碰撞节点对；这些假设在本项目没有等价验证，不能把它的安全结论直接搬到海上异构集群。
