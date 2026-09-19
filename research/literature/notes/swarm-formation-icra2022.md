# Swarm-Formation ICRA 2022：我们那个 `weight_formation` 到底在优化什么

## 文献记录

- 作者：Lun Quan*, Longji Yin*, Chao Xu, Fei Gao（* 同等贡献）
- 年份：2022
- 发表：IEEE ICRA 2022；arXiv:2109.07682
- 链接：https://arxiv.org/abs/2109.07682
- 本地 PDF：`papers/formation/swarm-formation-icra2022.pdf`（1,805,119 B，sha256 `fb5ce4ba4c45fa1a…`）
- 阅读范围：全文（pdftotext 正文）
- 与项目的对应：**这是当前开源源码 `upstream/Swarm-Formation` 直接对应的那篇**（README 指明 v1.1 对应 ICRA2022；T-RO 版是 arXiv:2210.04048，已另存）。

## 1. 它解决什么问题

- Problem：空中集群按指定队形飞行时，既有规划策略**在杂乱环境中缺乏避障能力**。
- Inputs：多项式轨迹参数、期望队形、障碍物、机间距离。
- Outputs：时空联合优化后的无碰撞编队轨迹。
- Assumptions：可微的**队形相似度**度量；分布式集群系统已有轨迹交换。

## 2. 它的任务、资源、通信和运动模型

- Task：无任务层，只有"保持队形并避障"的运动目标。
- Agent / capability：同构多旋翼集群，无能力差异。
- Resource：无资源/占用概念。
- Communication：分布式集群系统交换轨迹（本项目的 `broadcast_traj_from/to_planner` 即来自这一支）。
- Motion：多项式轨迹上的**队形相似度代价 + 碰撞代价 + 可行性代价**共同优化。
- Execution feedback：无；规划一次成轨迹。

## 3. 对我们哪个接口或约束有影响

1. **`optimization/weight_formation` / `formation_type` 的确切含义**：它是**当前集群图与期望队形的可微相似度距离**的权重，
   不是"编队松紧"这类直觉参数。这解释了我们探针实测的现象——**单独一台成员时集群图不完整**，
   相似度项会把该成员拉向一个虚构的编队构型（我们观测到它飞向原点）。
   → 结论：成员入口必须 `use_formation=false`，这一点现在有了原论文依据，不只是经验修补。
2. **`required_agent_count=3` 不等于"编队任务已完成"**：该代价只保证**几何形状相似**，
   不包含任何任务语义、到达判据或观测判据。编队任务必须另有组级完成条件（我们计划里已写）。
3. **编队的"存在"是代价项，不是模式开关**：因此"这次运行用了编队"只能由**实际队形误差**证明，
   不能由"注册了编队 executor"证明。

## 4. 它要求我们增加什么实验或反例

- **反例：`test_single_member_plans_without_formation_cost`**：单成员派发时若 `use_formation` 仍为真，
  成员会被拉向非目标位置（我们已实测到飞向原点）。断言成员终点等于指令目标。
- **反例：`test_formation_phase_requires_shape_error_not_membership`**：编队阶段即使成员数正确，
  若实际队形形状误差超阈值也不得判完成。
- **实验：`formation_weight` 语义核对**：记录启用/停用编队代价时同一编队指令的实际队形误差，
  用于说明"编队确实参与了"。
