# Robust MADER 2023/2024：通信延迟与异步轨迹发布

## 文献记录

- 作者：Kota Kondo, Reinaldo Figueroa, Juan Rached, Jesus Tordesillas, Parker C. Lusk, Jonathan P. How
- 年份：2024（RA-L 9(2):1476–1483；accepted 2023-11；arXiv:2303.06222, 2023）
- 发表：IEEE Robotics and Automation Letters (RA-L)
- 链接：https://arxiv.org/abs/2303.06222
- 本地 PDF：`papers/safety/robust-mader-2023.pdf`（5,798,090 B，sha256 `8779863…7151f`）
- 阅读范围：全文（pdftotext 正文）

## 该文献的保证与成立假设

- 场景：去中心化、异步轨迹规划；每台 agent 独立触发优化，把轨迹广播给邻居作为约束（§II）。
- 三个机制：**Delay Check (DC)**、**两步轨迹发布**（先广播 `traj_opt`，通过 DC 后再广播/执行 `traj_comm`）、**trajectory-storing-and-checking**（对已提交与刚优化的轨迹都做碰撞检查）（§II-A）。
- 关键假设：存在已知的通信延迟上界 `δmax`，且要求 `δmax ≤ δDC`（Delay Check 时长）。`δactual` 是实际延迟，`δDC` 可能因发现冲突而提前结束（§II-A, Table III）。
- 保证：若 DC 期间检测到冲突，agent 继续执行上一段已提交的 `traj_comm`（已知无碰撞），丢弃新轨迹；若 DC 期间无冲突，则提交 `traj_opt`。论文用 12 种消息到达时序枚举证明两两 agent 的碰撞安全性（递归可行性），证明不依赖 `δO`/`δC` 的具体时长（§II-A）。
- 明确局限：**不处理完全通信丢包**（"we do not address complete communication drops as described in Table I"），只处理有界延迟。
- 硬件实验：6 机 mesh 网络、平均延迟 49.8 ms（最大 483 ms）、最高速度 5.8 m/s，2 个动态障碍物；100% 无碰撞 vs 对照 83%。这些数字是其实验条件，不是我们网络的参数。

## 该文献改变当前哪个接口定义？

1. **`used_reference` / `source_trajectory_id`**：RMADER 区分 `traj_opt`（刚优化、尚未确认）与 `traj_comm`（已提交、实际执行）。本项目 plan.md 要求 `qn source_trajectory_id` 必须与派发前的 `trajectory_id` 归属一致；文献进一步约束：接口必须能区分"发布了优化轨迹"和"采用了已提交轨迹"，`used_reference` 只能来自后者。否则延迟到达的 `traj_opt` 会被误判为"qn 已采用新任务参考"。
2. **`safety_outcome` / `experiment_validity`**：RMADER 的安全性依赖显式通信契约（`δmax ≤ δDC`）。本项目若不记录延迟上界/DC 等价的检查窗口，`safety_outcome` 只能是 `NOT_VERIFIED`，不能因为轨迹编号匹配就声称安全；丢包情形必须按 `UNKNOWN_LOCKED`/超时处理，不能沿用 RMADER 的保证。
3. **`DelayEvent`**：延迟消息可能携带旧计划；事件去重不能只看"新消息到达"，必须绑定轨迹编号/提交状态。`trajectory_id` 与 `execution_id` 的映射需要抵抗乱序到达（旧轨迹后到不得覆盖新轨迹的归属）。
4. **`group-level completion`**：MADER 属于运动层对照，不定义任务完成语义；对本项目组级完成判定无直接约束（保持 plan.md 的动力学判定不变）。

## 该文献要求增加哪个反例测试？

**测试：`test_late_optimized_trajectory_does_not_count_as_adoption`**（建议放 `integration/qn_aav_simulator/tests/test_qn_telemetry.py` 或 `trajectory_adoption` 的单元测试）

- **setup**：为某一成员构造 planning/发布序列：`execution_id=E`，派发后先出现 `trajectory_id=N_opt`（optimized，未 commit），随后一条**延迟消息**才到达并携带同一个 `N_opt`（或更旧的 `trajectory_id`）；qn 的 `source_trajectory_id` 自始至终没有指向一个"先 commit 后执行"的轨迹。
- **stimulus**：运行 adoption/telemetry 判定（`trajectory_adoption.py` 的采用检查 + `qn_telemetry.py` 的诊断解析），并把 `used_reference` 快照与 `source_trajectory_id` 对齐。
- **expected observation**：采用判定必须为"未确认"（`REFERENCE_ADOPTION_UNCONFIRMED`），不得因为 `source_trajectory_id` 等于延迟到达的 `N_opt` 或 `PositionCommand.stamp` 更新就置 PASS；`used_reference` 不得在积分中途被该延迟消息改写（必须保持派发时冻结快照）；在未建立 `δmax ≤ δDC` 等价检查窗口的情况下，`safety_outcome` 必须为 `NOT_VERIFIED`。
- **为什么是反例**：延迟/乱序消息 + optimized/committed 不分的实现会错误地确认新任务参考已被采用；这正是 RMADER 用两步发布要消除的失败模式，也直接对应 plan.md P0 的 `source_trajectory_id` 归属验收。

## 我们实际复用什么

- "已提交轨迹才是执行参考"的语义：用于收紧 `source_trajectory_id` 与 `used_reference` 的判定，不引入 MADER 规划器本体。
- 延迟上界作为安全前提的写法：我们可以在通信扩展阶段记录 `δmax`/检查窗口，在此之前安全结论保持 `NOT_VERIFIED`。
- 消息乱序/延迟到达时的"继续执行上一份已验证参考"原则，与 plan.md 冻结 `used_reference` 快照、禁止积分后重读 latest command 的规则一致。
- 明确边界：不处理完全丢包（我们以 Odometry/Result 超时 → `UNKNOWN_LOCKED` 处理）；不引入其 C++ 规划器、动态障碍算法或网格通信实现。
- RMADER 的实验数字（49.8 ms 平均延迟、5.8 m/s、100% vs 83%）只作为通信受限对照的背景，不作为本项目验收门槛。
