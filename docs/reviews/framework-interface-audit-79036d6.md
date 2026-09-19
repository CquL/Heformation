# 冻结计划与框架、接口审查

2026-09-19；代码基线 `79036d66178deacada2b4f4a30dc588899eeb094`。

结论：继续当前三机请求闭环路线，保留 Calvo 受限调度、Swarm、qn 和 ROS1 Action。当前主要问题是任务语义和执行接线不完整，以及部分验收函数的前置条件没有落实；没有证据支持现在更换整个框架或建立统一消息总线。冻结计划方向成立，但“只剩 runner 接线”低估了剩余工作。

审查范围：读取 AGENT 当前入口、context 当前状态/架构/输入输出/通信/场景/路线/研究/文献/交接及历史边界；核对 integration 主链、补丁、测试、launch、上游消息及编队图实现；查阅论文相关正文和官方协议。没有逐行审计所有第三方库，没有重跑 ROS Action、完整请求或七机 M2。已有未提交背景文档修改保留。

## 1. 实施状态与证据

| 交付 | 本轮核对结果 | 证据级别 |
| --- | --- | --- |
| 三节点期望图、模式复位 | 补丁与三机配置存在；七机类型 1 保留 | 源码；此前 WORKLOG 记载 Action 实跑通过 |
| 单机→组级→单机 | 已有探针与此前成功记录；同位置目标问题仍开放 | 历史 Action 实跑记录，本轮未复跑 |
| 串行计划、成员预计位置 | 实现与反例测试存在 | 本轮纯 Python 测试通过 |
| observed AND received | `delivered_fraction()` 已直接维持不变式 | 本轮纯 Python 测试通过 |
| 区间形状、走廊、累计回退 | 基本公式存在；起点、时间完整性和调用链仍缺 | 模块部分完成，不能称岸线任务通过 |
| 请求→端点→完成→复测 | runner 仍是旧路径，未贯通 | 未完成 |
| CLI 确认、任务权威 dashboard | 无当前请求执行链 | 未完成 |
| 七机 M2 | 历史失败和前置修复保留；最终两类实跑未完成 | 独立待办 |

本轮命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest integration/mrta_python/tests integration/qn_aav_simulator/tests -q
```

结果 `240 passed in 1.07s`。首次不禁用自动插件时，宿主 ROS2 Jazzy 的 `launch_testing` 因缺 `lark` 导入失败；禁用无关插件后正常。这不是 ROS1 Noetic 实跑或 C++ 重编译证据。

补充纯函数反例见同目录 `framework-interface-audit-79036d6-counterexamples.json`。覆盖夹具预置 observed=True 仅用于隔离几何函数缺陷，不代表真实观测。

## 2. 应在下一次完整请求运行前解决的断点

### 2.1 runner 未接线，返回路径还会丢锁

`integration/qn_aav_simulator/scripts/formation_mission_runner.py`：

- 83–103：旧 Task、七个 Agent、`build_plan()`。
- 123–135：唯一 `formation_action` client、固定 goal/result 订阅和 `range(7)`。
- 248–282：readiness、baseline、配置证据仍绑定 `/formation_action_server`。
- 338–394：采用证据仍固定七机，并另写了一套弱于 `TrajectoryAdoptionTracker` 的 ID 判断。
- 413–444：选中 unit 只进入 context；实际发送仍使用固定 client；`finally` 无条件清除占用。
- 445–450：超时后才置 `UNKNOWN_LOCKED`，此时占用集合已经丢失。

当前循环失败后通常 break，因此不能声称已观察到同进程发生了下一次冲突派发；但资源表示确实与锁定语义矛盾。四个服务器各有自身状态机，另一个共享成员端点不会自动继承前一个端点的锁。执行器入口、正常 Result、异常返回及后续批次都必须服从同一 runner 的物理成员占用；未知或不安全不能解锁。启动下一 runner 也不能把仍生效的旧参考当空闲。

端点不仅影响 `send_goal`，还影响 goal/result envelope、readiness、adoption、证据文件与时间基线。优先使用 Result 已给出的 `evidence_file`，避免继续假设单一 server/index。

### 2.2 Executor 完成反馈不能直接套用七机修复函数

`integration/mrta_python/repair.py:92` 调 `validate_plan()`；`validation.py:75` 要求七个固定成员。实测把合法单机 ExecutorPlan 传给 `process_completion()` 抛出：

```text
ValueError: plan coalition must contain seven unique agents
```

`repair.py:53` 还沿代表成员队列传播延迟，不能表达“不共享成员也全局串行”的后继。下一步在已有完成处理代码内区分两种计划语义：保留七机回归；任务线按已确定串行顺序更新未开始项、去重真实完成事件，并用实际成员状态重新播种后续估价。不要仅增加 build_executor_plan 的调用然后仍走原完成函数，也不必引入一般偏序调度器。

### 2.3 冻结请求并未完整描述岸线观测

`config/monitoring_request_coastal.yaml:54` 引用 `s1/s2/s3`，但 regions 只定义 a1–c2，三点没有坐标/权重来源。`task_line.py:316` 只加载 regions，`to_plan_tasks():245` 全部转成单机任务。实际 `expand()` 只产生 `zone_A-obs0 / zone_B-obs1 / zone_C-obs2`。

需要在同一个请求/实例里提供三点定义并校验引用唯一性、坐标、权重；把集结 path_start 和沿岸到 path_end 作为三机阶段的实际动作接入现有执行流程。单纯给三类巡查任务接上客户端不会出现岸线阶段。

`load_formation_phase():307` 未读取 `corridor_half_width_m/backtrack_tolerance_m/arrival_tolerance_m`。实测 YAML 写半宽 0.2，加载结果仍为 1.5。已有字段应按真实配置读取，不再增加另一份配置权威。

### 2.4 区间函数存在可复现假通过

`task_line.py:132` 的 `evaluate_formation_phase_over_interval()` 只要求两帧，遍历时忽略 `_time`；未证明首帧在 path_start、全员稳定和参考归属已确认。实测：

| 输入 | 当前输出 | 要求 |
| --- | --- | --- |
| 中心从 x=-24 到 -18，跳过起点 -30 | complete=True | 不得代表完整区间 |
| 两帧 t=0 和 t=999，中间无数据 | complete=True | 缺失区间不得通过 |
| 第一帧 t=1，第二帧 t=0 | complete=True | 不得直接接受倒序区间 |
| 每步后退 0.08，累计 1.6 m 后重回终点 | complete=False | 正确 |

用已有 `time_alignment.py`、`experiment_verdict.py` 的 SampleLedger/成员新鲜度规则绑定阶段边界、样本完整性和实际 qn 状态；覆盖证据也必须限定本阶段、本次执行。当前函数只查 observed，不查 receipt、安全和 validity，故还需任务层联合终判。无需另建采样系统。

现有 `test_cumulative_backtracking...` 的名字/注释称“小步累计”，实际输入是一次回退 9 m；本轮额外的小步反例确认实现确实正确，应让测试夹具和名称一致。

### 2.5 单机安全范围不能缩成只有被派发成员

`formation_action_server.py:256` 只订阅 `self.agent_ids`；1006 创建同范围 member_samples；1392 用这些样本评价安全。单机端点只含一个成员，无法检查它与两台未被分配、仍真实存在的待命机之间的距离。

纯函数隔离反例：只给一台样本且场景无障碍，安全输出 PASS；将同一位置另一台机器的样本加入，输出 FAIL。该反例证明输入范围不足，不代表发生过真实碰撞。

目标路由仍只到执行成员，但安全检查必须涵盖相关物理机队，缺其他机器状态不能把机间安全判为通过。复用当前安全判据和三机实际状态，不新增安全算法。全局串行只排除任务并发，不能排除与待命机碰撞。

### 2.6 无运动任务的采用证据仍未闭合

`trajectory_adoption.py:201` 要求新 ID、属于派发后轨迹集合且模型步前进；这是防旧参考误认的必要证据，但同位置 hold/补测可能不生成新轨迹。历史探针通过把组中心移到所有成员都必须运动的位置避开了此边界。

应在现有 planner/Action 生命周期中为新任务被接受的保持参考建立可归属证据；优先复用原生 trajectory_id、GoalID 与本地映射。不能只改时间戳或凭位置正确放行。若仍无法安全切换，按冻结计划提供互斥静态配置分别验证并明确缺少同运行闭环。

### 2.7 探针与编队启用日志尚可加强

`probe_action_routing.py:126` 的 result 判定只判断 SUCCEEDED；release 只检查 result 有本任务 ID 且 task_outcome 非零，没有显式要求 TASK_PASS/SAFETY_PASS/VALIDITY_VALID；181 的循环在某段失败后仍尝试下一段，最后才汇总非零退出。下一段应由上一段成功 Result 与可释放判定共同放行。

`traj_opt_declared_formation.patch` 的启用日志仍放在 `if (similarity_error > 0)` 内。此前三台日志出现是正证据，但理想零代价时无日志不能证明未启用。将“越过 size 守卫且执行相似度计算”的一次计数放在非零代价分支之外。此外声明缺槽位当前静默默认 0，应拒绝退化/缺失声明。

## 3. 文献支持什么，不支持什么

| 工作及当前采用状态 | 对本系统的实质约束 | 结论 |
| --- | --- | --- |
| [Calvo T-RO](https://arxiv.org/html/2411.02062v3)，受限 Python 移植 | 实际完成、终点偏差引起后续耗时变化；修复有适用条件 | 保留反馈修复；不能称完整复现电池、接替与同步模型 |
| [GRSTAPS IJRR 2022](https://star-lab.cc.gatech.edu/papers/messing-grstaps/)，本轮仅官方摘要 | what/how/who/when 相互影响 | 分层合理；名义直线距离估价不是运动可行性证明 |
| [APEX-MR RSS 2025](https://arxiv.org/html/2503.15836v3)，参考 | 计划与执行在不确定性下协调，原问题是合作装配 | 借鉴执行依赖；不必为了串行基线搬入整套异步执行图 |
| [Swarm ICRA 2022](https://arxiv.org/html/2109.07682v2)，运动后端已接入 | §III-A 相似度对平移、旋转和缩放不变 | 本项目固定朝向/尺度的 E_form 是额外任务验收，不能用代价启用替代 |
| [CARIC](https://arxiv.org/html/2501.06566v2)，观测原则参考 | §II-D 按收到的观测质量结算，逐点取最佳分数 | 二值覆盖代理可用，但不能说已复现 CARIC 相机评分 |
| [CoCoPlan RA-L 2026](https://arxiv.org/html/2601.10116v1)，研究参考 | 联合安排任务与间歇通信事件已有研究 | 未来研究必须解释超出该模型的海上异构约束，不能只用联合框架图声称创新 |

Swarm 本地 `upstream/Swarm-Formation/src/planner/swarm_graph/src/swarm_graph.cpp:53` 用距离平方和归一化 Laplacian，源码与论文一致。把 weight_formation 调大不能让这个项获得原本没有的朝向敏感性。保持现有算法先跑严格判据；失败要定位在目标/阶段组织还是后端约束表达，不能放宽验收来证明成功。

当前全局串行是可信执行基线，主动牺牲并发，不能作为异构协同效率的最终证明。没有必要在当前四个静态单元上先换大求解器；将来论文比较应包含简单贪心、现有受限方法及与研究问题相匹配的方法，并控制相同动力学、通信和观测假设。

## 4. 接口复用与可精简部分

需要区分三种东西：Python 进程内数据结构、ROS 消息、真实跨平台链路上的数据包。dataclass 多一个字段不等于无线链路多发一个字段。

| 对象 | 当前判断 | 建议 |
| --- | --- | --- |
| `Formation.action` | Goal 仅 task_id、PointStamped、hold_duration；Result 含终态原因/安全/有效性 | 必要薄适配，继续使用；任务覆盖保持上层 |
| `PolyTraj / PositionCommand` | Swarm 已有多项式、时间、轨迹 ID 与运动参考 | 直接复用，不复制 UniversalTrajectory |
| `nav_msgs/Odometry` | 实际状态原生类型 | 继续作为真实状态输入 |
| 标准与 swarm_compat 两个 Odometry | 标准 twist 为机体系；旧 Swarm 消费世界系速度 | 兼容边界必要，不能简单按重复字段删除；可只在本机保留兼容通道 |
| `Executor / ExecutionUnit` | 规划视图与端点路由视图职责不同，但重复成员/能力可能漂移 | 从同一注册配置派生；不要求合成大类或再加接口层 |
| task_id / execution_id / GoalID / trajectory_id | 分别对应任务、执行尝试、Action 尝试、本地轨迹 | 保留身份区分与映射；无需把所有 ID 都塞进每个高频包 |
| `contracts.py` 的 energy=1、current_task_id、空 PlatformAdapterCmd、environment 等 | 主 qn 路径未实际消费部分字段；当前是内部适配形状 | 标成未使用/内部占位，不作为能力或能量证据；清理应按真实调用者，当前不扩大到重构 |
| `desired_jerk/desired_attitude` 等 | 原生/内部类型存在不代表当前 qn AIR 使用 | 保持原生类型；不要为了“接口齐全”要求用户填写 |
| 旧 `formation_shape_error/evaluate_formation_phase` | 末态距离判据仍保留，与新全程向量判据含义不同 | 历史用途明确；任务线唯一调用正确区间判据，避免两套 success 权威 |

已有开源包不是没有数据协议：本地 `upstream/Calvo-Execution/action/NewTask.action`、`TaskResult.action`、`msg/Task.msg` 已有任务、应答与结果。但 Task 同时携带 monitor/inspect/deliver/recharge/wait/generic 参数；仅替换消息不能获得它依赖的 Behavior Manager、几何映射和本项目 qn 采用证据。对当前到点+保持 Action，整体替换更复杂。来源：[官方执行仓库](https://github.com/multirobot-use/mrta_execution_architecture)。

未来 USV/UUV 若采用 LSTS，可以复用 IMC 的 PlanControl、PlanControlState、EstimatedState、TransmissionRequest/Status；后者已有目的节点、通信介质、发送 deadline。无需自行再造这些底层包。来源：[官方 IMC.xml](https://raw.githubusercontent.com/LSTS/imc/master/IMC.xml)。

若需要水声紧凑编码，可评估 Goby/DCCL：已有字段范围/精度/长度和最大字节预算，而非自己发明二进制编码。它提供编码/通信工具，不提供本项目的任务分配与交付语义。来源：[DCCL 官方 IDL](https://libdccl.org/5.0/md_src_2doc_2markdown_2page02__idl.html)、[Goby3 通信库](https://goby.software/3.0/md_doc100_acomms.html)。这些是后续后端选择时的复用依据，本轮不安装或迁移。

## 5. 参数必要性与数据传输

最明显的“参数有值但没有完成语义”的地方是 `observation_coverage.py:213`：`observed = best_score > 0.0`。模糊函数在正容差、有限速度下通常仍大于零。隔离实测 score=0.0025 仍 observed=True，二值覆盖会计为完整一项。

这不等于模糊参数物理影响小，而是当前二值判定抹掉了差异。CARIC 使用连续质量分数；本项目若要二值“有效”，应明确最低质量要求及其驻留口径，然后决定是否让质量在连续窗口内维持。不应随意替用户填质量阈值，也不能声称仅有正分数就是高质量有效观测。

| 参数 | 当前作用与边界 |
| --- | --- |
| footprint、LOS、min_dwell | 决定可见性和连续驻留，必须保留 |
| exposure / blur_tolerance | 决定 score；当前对多数正分数的二值 observed 无区分力；固定曝光的纯代理中只通过二者比值影响 blur，可将其中一个固定在模型配置，真实相机接入后再分别标定 |
| nominal_standoff / max_distance_from_altitude | 决定分辨率衰减；后者在展开用垂直距离，在评价用三维距离，需统一含义或明确展开只是候选覆盖，不承诺有效观测 |
| service_time / min_dwell | 前者是调度与保持时长，后者是有效观测要求，不是重复字段；应校验可行性 |
| deadline | 当前调度器是 soft deadline，只影响启发式/迟到报告；若用户要求硬截止，任务最终结果必须明确判定，不能把已有字段当硬约束 |
| formation_size / slots / scale | 固定一次系统配置；数量与成员、槽位一致，不应每次目标重复传全表 |
| corridor / arrival / shape / backtrack | 不同几何失败维度，不能合成一个 epsilon；现有 phase loader 漏读应修正 |
| 对齐窗口、状态新鲜度、模型步、采用 ID | 影响安全与证据归属，不能为了少几个参数删除 |
| e_budget | 工程预算；没有对应可达性模型证明时不是 FaSTrack 式理论界 |

`qn_aav_node.py:280` 每个 outer step 发布标准/兼容 Odometry、used reference 和 DiagnosticArray；`simulator.xml:112` outer_dt=0.01，即配置 100 Hz。diagnostics:347 起还重复携带位置、速度、常量和字符串键值。它们在本地验收有消费者，不能未经核对直接降频，否则破坏时间对齐；也不应默认全部越过未来 RF/水声链路。

跨链路负担取决于 `Σ(消息序列化大小 × 频率 × 实际跨链路副本数)`，还包括协议开销、重传、排队，不能仅数接口字段。本轮未测 ROS 实际包长/带宽/CPU，因此不声称已经发生网络瓶颈。当前更值得优化的是静态参数重复高频发送、诊断/图像/点云错误越过链路，以及多个观察者重复维护全历史，而非删除低频 Action 中的一个 ID。

下一阶段任务运行状态最少需要映射：请求/执行尝试→单元及成员→动作/结果→观测点与质量→接收记录→锁定/失败。复用现有任务对象和结果记录即可。当前 `CoverageResult` 只有按点的 observed 和 received 集合；跨多批次时必须按请求/执行限定归属，不能沿用同名点旧 receipt。真实通信前才补所用链路需要的目的节点、数据大小、时限、接收时间/去重，不预建全能数据包。

## 6. 建议继续实施顺序

1. 补全冻结请求：岸线三点、三机阶段动作、配置读取、质量/截止时间口径。保证确认页展示的是完整请求及实际支持限制。
2. 在现有 runner 内接两种 planning mode，按 unit 选择真实客户端及完整证据/readiness 命名空间；接 Executor 完成反馈和实际成员状态。
3. 同时落实未知终态不解锁、单机动作仍检查相关全机队安全；失败段不得推进下一段。修同位置采用边界，不能以移动目标掩盖。
4. 集结/稳定/参考采用后绑定观测起点，复用既有采样完整性检查；区间、覆盖、交付、安全、validity 共同决定任务完成。
5. CLI 显示并等待确认；未确认只待命；执行中拒绝新批次。一次运行覆盖单机→岸线编队→收到结果→必要时一次复测。
6. dashboard 读取同一个任务层权威结果，展示资源占用和失败；七机 M2 另行完成原盒子时间线、扫描中断与元数据。

这仍在冻结范围内，不需要新增算法、调度结构、协议层或仿真器。新缺陷表明“任务线接通”包含正确完成反馈、观测定义及安全范围，不能压缩成只改端点名字。

文档需注意：context/03 等新文档已区分目标与实现，但 context/AGENT.md、06/07 等仍有旧七机表述/旧 R 编号，prompt/wenxianSKILL.md 也保留历史状态文字；以后同步时明确历史标签和论文标题/DOI，避免把旧编号对应到新文献。当前文献与源码采用状态应按 manifest 和实际调用链说明。研究候选保留，USV/UUV、真实通信、能量与跨介质均不能标为已完成。
