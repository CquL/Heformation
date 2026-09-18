# Swarm-Formation 2022：编队槽位、重组、局部轨迹与上层联盟边界

## 文献记录

- 作者：Lun Quan, Longji Yin, Tingrui Zhang, Mingyang Wang, Ruilin Wang, Sheng Zhong, Xin Zhou, Yanjun Cao, Chao Xu, Fei Gao
- 年份：2022（arXiv:2210.04048；本笔记读的是 v2, 2023-08-07），期刊版 2023
- 发表：IEEE Transactions on Robotics (T-RO) 2023；完整标题为 “Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments”
- 链接：https://arxiv.org/abs/2210.04048
- 本地 PDF：`papers/swarm-formation-2022.pdf`（17,807,767 B，sha256 `6a4e6b1…900d7`）
- 阅读范围：全文（pdftotext 正文）

## 该文献的保证与成立假设

- 系统是**分布式**的：每台机器人运行相同模块，机器人之间只交换轨迹等必要信息；高层不保留一个逐时刻同步所有机器人的集中控制器。
- 编队相似度使用可微的图度量，对旋转、平移、缩放不变（Sim(3)）；"保持编队"是软代价，不是任意环境下的硬约束。
- 冲突不可维持时通过 **global-remap-local-replan (GRLR)** 低频触发重组；局部轨迹优化以固定频率运行，只做定时间戳的相互避让。
- **ALAS**（formation ALignment and tASk assignment）：先按“编队约束感知”权重求对齐 `(s,d)`，再解当前机器人到目标槽位的分配 `σ`（本质是带权匹配，不是固定编号槽位）。这意味着任务执行期间成员与槽位的对应关系可以被重排。
- 全局路径由双向 RRT* 在 `z={p,s}` 空间搜索，需要全局环境信息；局部优化依赖 ESDF/障碍梯度与邻机轨迹。
- 论文实验参数（不是我们系统的保证）：相似度误差阈值 `esim,d=0.05`、约束感知阈值 `gd=2/N`、最大速度 1.0 m/s、最大加速度 6.0 m/s²、规划时域 7.5 m、采样 0.5 s；密集环境实验规模至数十台。
- 明确局限：编队重组需要快速、可靠地交换轨迹；网络延迟只被承认存在，没有给出任意延迟下的安全性证明。

## 该文献改变当前哪个接口定义？

1. **`FormationAction`（Goal 只有 `formation_center` + `hold_duration`）**：Swarm 的编队是由槽位集合 + 尺度 + （旋转）对齐定义的；`formation_center` 单点目标无法表达尺度/旋转/形状变化，也无法请求 ALAS 式槽位重排。若第一版保持现有 Goal，则必须在接口文档中明确：上层只能请求“编队中心 + 时长”，成员槽位/分配完全由 Swarm 决定；上层不得把“成员 i 必到槽位 i”写死为完成条件。
2. **`group-level completion`**：Swarm 在 GRLR 触发时会改变成员—槽位分配，并产生新的局部轨迹而**不产生新任务**。本项目的组级完成判定和 `trajectory_id` 采用检查必须按“当前 adopted 的成员目标/参考”判定，而不是按派发前保存的固定槽位；否则一次合法重排会被误判为不到位，或一次槽位互换会被误判为完成。
3. **`used_reference` / `source_trajectory_id`**：任务执行中的局部重规划要保留同一 `execution_id` 上下文（plan.md 已写明“若任务期间发生局部重规划，保留同一 execution 上下文”）；Swarm 的 GRLR 给出了这种“新轨迹、同任务”的具体来源，正式化了该规则。
4. **PositionCommand 语义**：上层 PositionCommand/编队目标是规划参考；Swarm 的槽位分配是分布式求解结果，不是上层保证。上层不能把“发出编队中心”当成“已指定每台 qn 的最终位置”。

## 该文献要求增加哪个反例测试？

**测试：`test_formation_completion_rejects_shape_only_pass_under_slot_swap`**（建议放 `integration/qn_aav_simulator/tests/test_formation_monitor.py` 或 `test_formation_action_server.py`）

- **setup**：用固定 7 机、`center=(0,0,0.5)`、`scale` 与默认槽位构造 `FormationMonitor`；准备一段完整驻留窗口的 Odometry：d0、d3–d6 位于各自槽位，**d1 与 d2 交换位置**（即 7 个空间点集合仍构成同一编队形状），速度接近 0，stamp 新鲜，模型时间满足 `hold_duration`；各成员的 `used_reference`/`source_trajectory_id` 仍是派发时的参考，没有新轨迹编号。
- **stimulus**：把该窗口喂给 monitor 并调用完成判定（`evaluate`/`verdict`），同时计算编队形状误差与每成员最终槽位误差。
- **expected observation**：`task_outcome != PASS`；d1、d2 的 final slot error 必须为非零并阻止组级完成；不允许因为“形状误差/Sim(3) 相似度达标”就置 `task_outcome=PASS` 或 `experiment_validity=VALID`。若系统确实在窗口内发生了 Swarm 式 slot 重排，则必须能从 adopted `trajectory_id` 证明新分配，否则记 `REFERENCE_ADOPTION_UNCONFIRMED`。
- **为什么是反例**：Swarm 原文的 ALAS 说明槽位分配可以变；如果我们的完成判定退化成只看编队形状（或只看 7 个点的集合），上面的槽位互换会被错误接受。该测试锁死“per-member adopted reference，而不是 shape-only”这一语义。

## 我们实际复用什么

- 已复现的七机 AIR 编队基线与 `PositionCommand` 数据流保持不变；本笔记只约束上层接口语义。
- “任务执行中的局部重排 ≠ 新任务”用于 `trajectory_id` 归属：新轨迹编号仍属于同一 `execution_id`。
- Swarm 把编队保持当作可放宽的软约束、把重排当作恢复手段，支持本项目“协同不是全体始终同步编队”的定义。
- 相似度/约束感知阈值（0.05、2/N）可作为后续编队误差指标的参照，但不直接写入验收门槛。
- 分布式轨迹交换、ESDF 局部优化、GRLR 的适用边界可作为后续对照实验（重排、开阔区）而不是第一版前置条件。
- 网络延迟只被承认、未被证明：提醒我们不要从 Swarm 实验外推通信受限下的安全保证。
