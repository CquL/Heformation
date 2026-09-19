# CARIC 2025：巡检任务怎么定义，什么算"真正观测完成"

## 文献记录

- 作者：Muqing Cao, Thien-Minh Nguyen, Shenghai Yuan, Andreas Anastasiou, Angelos Zacharia, Savvas Papaioannou, Panayiotis Kolios, Christos G. Panayiotou, Marios M. Polycarpou, Xinhang Xu, Mingjie Zhang, Fei Gao, Boyu Zhou, Ben M. Chen, Lihua Xie
- 年份：2025（arXiv v2；比赛在 IEEE CDC 2023 与 IROS 2024 Workshop 举办）
- 发表：arXiv:2501.06566 [cs.RO]
- 链接：https://arxiv.org/abs/2501.06566 ；官方框架 https://ntu-aris.github.io/caric/
- 本地 PDF：`papers/caric-2025.pdf`（3,603,818 B，9 页，sha256 `c4c2ab5724f25699…`）
- 阅读范围：全文（pdftotext 正文）

## 它解决什么问题

异构多机**巡检**任务的基准：一群 UAV（explorer 带 LiDAR、photographer 带云台相机）在**没有先验模型**的情况下探索结构体、发现兴趣点、并对高风险点做近距离高分辨率成像；比赛比较各队的探索、巡检与任务分配策略。它把"巡检完成度"定义成可计分的量，而不是"飞到过哪些点"。

## 它改变我们哪个接口或约束

1. **观测完成判据必须按"质量分数 + 兴趣点 + 去重"定义，而不是按区域或到位**。评分
   `Q = Σ_i max_j q_ij`，`q_ij = max_k q_{ij,k}`，`q_{ij,k} = q_seen · q_blur · q_res`：
   `q_seen∈{0,1}` 要求兴趣点落在相机视场内**且视线不被遮挡**，`q_blur` 由曝光时间内的像面位移决定，
   `q_res` 是分辨率（MMPP）与期望值之比。我们计划里的 `C_delivered` 只用了"水平半径 2.5 m + 驻留 1 s"，
   缺 `q_blur`/`q_res` 这两维；应至少把**质量因子**写成可声明参数并在结果里分开报告，而不是把
   "在视场内"直接等同于"有效观测"。
2. **去重规则得到文献支持**：`max_j` 表示一个兴趣点只取**所有平台中的最好一次**，不按平台累加。
   这正是我们计划里"一次有效观测可满足多个点并去重"的对应规则；应明确写成"每点取全局最优一次"。
3. **碰撞使成绩整体作废**：论文明确"only the scores of UAVs that successfully completed the mission without collision be considered"。
   这与我们"`coverage_fraction=1` 不能覆盖中途出现过的安全失败"一致，且更强：安全失败不是扣分，是**不予计分**。
4. **评分由控制站按"收到的结果"结算**，论文把控制站描述为"tallies the final score for the mission"。
   支持我们把 `C_delivered` 的分子定义为"已有效观测**且结果已被接收**"，采集与接收分开记录。
5. **通信是显式约束**：机间通信仅限视距，框架用 `ppcomrouter` 按 LoS **中继或丢弃**消息。
   这与我们"USV 作为中继"的目标直接相关，意味着中继是**消息层的既有语义**，不是我们新造的概念。
6. **任务分配的工作量估计不能用体积代理**：论文的失败分析指出按包围盒体积分区会导致机器人把时间
   花在"小但复杂"的结构上；基于视点/工作量的分配明显更好。我们计划里的 `service_time` 是手填常数，
   这直接说明它只是一个占位，不能当作真实工作量。
7. **"拆成多个小组不一定更快"**：论文明确"separating a small number of robots into multiple teams may not yield
   efficiency gains compared to a single team due to the difficulty of workload allocation among teams"。
   这是我们"不要为拆队而拆队"的外部依据，也支持当前"单机任务默认不交给编队单元"的默认规则。
8. **单次运行不可当结论**：同场景不同运行分数差可超过 1000，且出现"整个任务期内全队空闲"的失败运行。
   这为"M2/任务线的 n=1 结果不能当能力证明"提供了直接外部证据。

## 它要求增加什么反例或实验

**反例：`test_coverage_requires_quality_not_just_presence`**

- setup：一个兴趣点，平台在其水平半径内**但**（a）视线被已声明障碍遮挡，或（b）速度使像面位移超过 1 px/曝光，或（c）分辨率低于 `r_des`。
- expected：该兴趣点计 0 或按质量因子折扣，`C_delivered < 1`，`objective_outcome=NOT_MET`；当前只按"半径 + 驻留"判定的实现必须失败。
- 为什么：它把"飞到了"与"真正观测到"分开，正是 CARIC 评分的前三维。

**反例：`test_collision_voids_the_coverage_claim`**

- setup：一次运行 `coverage_fraction=1.0`，但中途出现一次 `safety_outcome=FAIL`。
- expected：请求不得判 PASS；输出的三个字段必须同时呈现，且报告里"任务完成"不得只引用覆盖率。
- 为什么：对应 CARIC"未碰撞者才计分"。

**实验：把 `service_time` 换成工作量估计的对照**

- 同一任务集跑两次：一次用手填常数 `service_time`，一次用按观测点数估计的工作量；比较实际 makespan 与超期。
- 为什么：CARIC 的结论是体积/常数代理会显著劣化分配；这条至少要能说明我们的估计是占位还是有效。
