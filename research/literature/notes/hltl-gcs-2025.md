# H-LTL/GCS 2025：显式依赖与连续可行运动建模

## 文献记录

- 作者：Zhongqi Wei*, Xusheng Luo*, Changliu Liu（* 同等贡献）
- 年份：2025
- 发表：Robotics: Science and Systems (RSS) XXI, 2025，DOI 10.15607/RSS.2025.XXI.099
- 链接：https://www.roboticsproceedings.org/rss21/p099.html
- 本地 PDF：`papers/task_motion/hltl-gcs-2025.pdf`（8,077,812 B，sha256 `b556f92…b87d1`）
  - 注意：RSS 官方 PDF 是**无文本层的图像版**（`pdftotext` 输出为空，`pdffonts` 无字体）。正文阅读使用同一论文的 arXiv 版本 v2（https://arxiv.org/abs/2504.18899，有文本层）；本地保存的仍是 RSS 官方开放版。
- 阅读范围：全文（通过 arXiv:2504.18899v2 文本版逐段阅读；标题、作者、摘要与 RSS 页面一致）

## 该文献的保证与成立假设

- 任务规格用 hierarchical sc-LTL 表达；任务分配不是预先给定，而是嵌入在 product graph 的边上——即"谁做哪件事"由搜索决定（§I, §V）。
- 运动层用 Graph of Convex Sets (GCS)：用凸集覆盖构型空间，凸集由采样运动规划器（IRIS-RRT）生成；任务-运动联合问题写成 MICP（混合整数凸规划）后在 product graph 上求最短路（§V）。
- **Theorem 6.1（Soundness）**：返回的路径 p 满足 hierarchical sc-LTL 规范 Φ；该方向对返回的解直接成立（证明构造状态-规格序列并逐层验证，§VI）。
- **Theorem 6.3（Completeness）**：假设 Φ 是**相容的**（不存在互相矛盾的 leaf specification，例如 `♢a` 与 `¬a U b`），则"up to the space decomposition and trajectory parameterization"返回满足 Φ 的路径。其含义是：需要凸集覆盖足够、Bézier 曲线阶次足够，MICP 才保证找到路径；这不是对任意离散化都成立的保证（§VI）。
- 协作搬运/交接用 Big-M 二值约束（论文取 M=2）保证物体归属连续与交接点唯一；图规模用剪枝启发式控制（§V, §VI）。
- 实验覆盖平面机器人、机械臂、四足、传送带与 28 自由度协作场景；不是海上/水下平台或 qn 动力学的验证。

## 该文献改变当前哪个接口定义？

1. **`PlanItem` 依赖 / `Plan.items`**：H-LTL/GCS 把"任务时序与分配"放在同一个图搜索里，顺序是解的属性而不是输入。本项目第一版没有 LTL 规范，所以**当前冻结的完成判据与依赖字段不需要改**；但文献约束我们：若未来要表达任务间显式顺序/依赖，必须新增结构化的依赖关系（例如 `depends_on` 或规格级约束），不能只靠 `planned_start/finish` 的相对大小隐含表达——时间可以平移，顺序约束不能。
2. **`travel_time_provider`**：该文的运动可行性以 GCS 凸集在同一搜索中前瞻判定；本项目只有标量旅行时间估计。文献把"标量估计 ≠ 可行性证明"这一点放大到接口层：要么保留并标注估计语义，要么明确增加可行性查询适配，二者不能混为一谈。
3. **`group-level completion` / `experiment_validity`**：其完备性保证带"规格相容 + 空间分解/轨迹参数化足够"的前提。对应到我们：任何"计划一定可完成"的说法必须同时给出依赖模型与运动可行性的成立条件；缺失时 `experiment_validity` 不得为 `VALID`。
4. **不改变的部分**：我们不做 LTL 任务规格、不做多臂/交接；其分配嵌入边、soundness 证明结构仅作为后续研究参照。

## 该文献要求增加哪个反例测试？

**测试：`test_explicit_order_dependency_rejects_out_of_order_plan`**（建议放 `integration/mrta_python/tests/test_validation.py` 或 `test_schedule.py`）

- **setup**：构造带显式依赖的两任务计划：`T1`（先巡检 A 区）→ `T2`（返回/回收），依赖语义为"T2 必须晚于 T1 完成"；构造一份 `Plan` 让 `T2.planned_start` 早于 `T1.planned_finish`（或让 repair 把 `T2` 平移到 `T1` 之前）。
- **stimulus**：调用 `validate_plan` 并把计划交给 runner 调度。
- **expected observation**：`validate_plan`/调度前检查必须拒绝该计划（抛校验错误或返回 `INVALID`），runner 不得派发 `T2`；同一联盟成员的队列中 `T1` 必须严格早于 `T2`；不得因为两个 `planned_start` 数值"看起来可行"就接受。若当前模型根本没有依赖字段，测试应明确失败并指向该缺口，而不是默默通过。
- **为什么是反例**：H-LTL/GCS 的顺序来自规格而非时间；本测试锁定"依赖是显式约束、不是时间表的副产品"，同时防止 `plan_repair` 在平移后继时破坏顺序。

## 我们实际复用什么

- "任务分配可以作为搜索结果、不必预先绑定"的思想；本项目的固定七机联盟是 v1 约束，不阻碍后续把分配放宽为规划变量（与 plan.md 顺序一致）。
- soundness/completeness 的表述方式：先声明解的可用性，再单独声明完备性所需假设（相容规格、覆盖/参数化），提醒我们任何联合规划主张都要写清前提。
- GCS/MICP 与采样规划器结合的架构可作为后续"连续可行运动 + 任务层"对照，不进入第一版。
- 剪枝启发式只作参考；我们不引入 LTL 与凸集分解代码，避免超出冻结范围。
- 显式记录"当前只读结论成立的边界"：不做 LTL 规格时，该文不能改变 v1 的完成判定，只能约束未来依赖接口。
