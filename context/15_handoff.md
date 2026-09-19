# 最新交接：七机AIR任务闭环按plan.md P0–P2跑通

**2026-09-18｜当前工作：按用户冻结的[plan.md](../plan.md)实施受限Calvo v9 + ROS1七机AIR任务闭环，P0–P2主链已完成一次可复查的完整运行。**

## 本次目标与实际结果

本轮把plan.md的P0–P2要求落到代码与运行证据上，没有更换Swarm、没有修改qn控制器/动力学/消息、没有增加控制层。

实际运行（同一镜像，唯一差别是`repair_mode`）：

| 实验目录 | repair | 结果 |
|---|---|---|
| `experiments/20260918-mission-e` | on | T1/T2/T3全部`task_outcome=PASS`、`safety_outcome=PASS`、`experiment_validity=VALID`；`verification.json` 220项检查0失败 |
| `experiments/20260918-mission-f` | off | 同上；`plan_updated=false`，但派发仍等待每次动作真实完成 |

两次运行的任务前基线都是30 s连续七机对齐，`valid_sample_ratio=1.0`，模型-ROS累计偏差
0.00054 s（门槛0.05 s），七机跨节点偏差0.00079 s；模型时间驻留5.02/5.01/5.03 s（要求5.0 s）。
单元测试：`integration/qn_aav_simulator/tests`与`integration/mrta_python/tests`共109项通过。

## 本轮实现与修复（按plan.md条目）

- **P0.1 状态语义**：qn节点从一个状态快照发布两个话题。标准`/drone_i_qn/odometry`为世界位姿+机体系
  twist；兼容`~odometry_swarm_compat`保留Swarm原本依赖的世界系线速度并重映射到
  `/drone_i_visual_slam/odom`。验证器实测两者关系残差0.0 m/s。
- **P0.2 参考使用过程**：外层`backend.step()`步长与内部积分子步分开统计，命令在每次控制循环前
  冻结成不可变快照；发布`used_reference_pose/twist/diagnostics`，诊断含
  `source_trajectory_id`、`source_command_stamp`、`used_outer_step`、模型区间与
  `velocity/acceleration_directly_consumed`。
- **P0.3 参考采用**：改用trajectory_id归属判定（派发前轨迹号→派发后新轨迹号→qn
  `source_trajectory_id`），组级目标只由授权发布者发一次；无法归属记
  `REFERENCE_ADOPTION_UNCONFIRMED`，不再用时间戳更新当证据。
- **P0.4 时间有效性**：共同ROS时间网格上持续记录累计模型-ROS偏差、跨节点偏差、模型/ROS与
  模型/墙钟速率；任务开始前要求30 s基线合格，任务期间继续检查；驻留完成另加模型时间门槛。
- **P0 地图/健康**：就绪分三级（话题存在、消息已收到、输入对本次规划有效），CPU模式只等点云与
  规划接口，`known_empty_map=true`才接受空地图；运行期检查全局地图、本地点云、七路Odometry、
  规划器健康与qn状态更新时间。
- **P1.1 回调分离**：`goal_callback`只做校验→原子检查`READY_IDLE`并占资源→接受/拒绝→交给单一
  工作循环→返回；忙时拒绝而不抢占。
- **P1.2 资源状态机**：`BOOTING→READY_IDLE→ACTIVE→HOLDING→SUCCEEDED→READY_IDLE`，异常
  `UNKNOWN_LOCKED`；取消不调用`set_preempted()`、不释放资源；只有完成驻留才释放。
- **P1 判定与有效性**：分别输出`task_outcome`/`safety_outcome`/`experiment_validity`，指标区分
  最终槽位误差、跟踪误差、速度误差、驻留速度、编队形状误差，缺失样本用
  `valid_sample_count`/`valid_sample_ratio`/`max_continuous_gap_s`/`alignment_failure_count`
  记账；碰撞与净距检查独立，且标注为离散采样安全检查。
- **P2 连续任务**：A→B→Return每个动作保存`execution_id`、Action GoalID、trajectory_id集合、
  计划/实际/名义完成时间；沿用`schedule.py`、`repair.py`、`process_completion()`，同一
  DelayEvent只处理一次，repair前后计划可从`plan_history`复查。

## 本轮暴露并修掉的真实缺陷（每个都有对应失败实验目录）

窗口化样本被当成单样本使用导致T1执行崩溃；时间对齐报告在锁内做O(网格×样本)计算饿死回调；
`readiness.note_planner_health`从未被调用；就绪判断用了探测前的旧时钟；诊断100 Hz只喂20 Hz网格
造成假缺失；网格尾部超出最后共同样本被判缺失；任务级窗口被误判`INVALID`；空点云未覆盖
“已发布但为空”的地图；诊断写在释放资源之前；runner把忙时拒绝当成时间戳非法；
`group_goal_publish_count`自计一次；`used_reference`用±50 ms最近邻比较了不同样本；
验证器检查了一个不存在的话题名；兼容话题与自身比较（空检查）；`planner_nominal_finish`、
`plan_before`、`induced_reference_displacement`未产生；`trajectory_ids`在采集之前就取值导致
每个动作都为空。

## 已知偏差与限制（不得写成已解决）

- qn 12ODE第三行（升沉）符号与原模型一致地保留：严格`R(q)*v_body`与兼容世界系速度差
  1.7–2.4 m/s，而模型自身映射差0.0 m/s。改它等于改动力学，plan.md禁止；已在
  [QN_INTEGRATION](../docs/QN_INTEGRATION.md)记录，需要机身↔世界变换的消费者必须显式处理第三行符号。
- 串行单资源下`dispatch_changed`在repair-on/off都为真，只能读作“实际释放时间偏离计划时间”，
  不能当作repair效果的证据；repair效果要从`plan_updated`与修复后的`planned_start`看。
- qn外层循环100 Hz、积分1000 Hz是本次记录的负载结果，不是配置保证；七台qn节点各占约60%单核。
- 安全结论是离散采样检查；未做连续时间保证，也未做跨介质。
- **未解决（本轮可视化暴露）**：安全检查里没有地面/高度下界项。`experiments/20260918-mission-g-fault`
  的 drone_3 在 T1 期间实测降到 z = -1.276 m（穿地），而 T1 仍判 `task_outcome=PASS`，因为完成判定只看
  驻留窗口内的位置/速度，安全检查只算机间距离与点云净距。该次下潜发生在我手工 kill 节点之前，不是注入
  故障导致的。其余两次运行的七台最低高度分别为 0.057 m / 0.084 m 以上。是否把高度下界纳入
  `safety_outcome`（并重跑受影响运行）需要用户确认后再改。
- P3多执行单元、多资源竞争尚未开始（plan.md明确要求在P0–P2通过后单独增加离线模型）。

## M1–M3 收敛版实施结果（2026-09-18）

按冻结的《Heformation 框架验证实施基线》实施，结论按里程碑分开陈述。

**M1（正确性修复）：完成并通过验收。**
`experiments/20260918-mission-m1` 七机在线任务链三任务全部
`task_outcome=PASS` / `safety_outcome=PASS` / `experiment_validity=VALID`，
独立验证器 **242 项检查 0 失败**。要点与证据：

- 坐标映射集中在 `odometry.py` 一处：保留模型姿态与机体角速度，发布
  `R(q)^T v_world` 作为机体系线速度，使标准消费者精确满足 `dp/dt = R(q)v`；
  8 项确定性反例（单位四元数纯轴 + 非零 roll/pitch/yaw、姿态导数与 ω 一致）
  通过。判定依据来自实测：前飞加速时 pitch 为低头，且机体角速度与 `R(q)^T dR/dt`
  一致，说明模型姿态与角速度是物理量，只有机体线速度需要修正。
- 恢复固定外层步长（`outer_dt_s` 唯一权威，默认 10 ms），模型时钟自驱；
  命令在步边界采用并有界缓存，突发/乱序锁存。实测模型-ROS 累计偏差 0.0033 s
  （门槛 0.05 s），该数字现在是真实测量而非由驱动方式决定。
- AIR 域在线锁存：本运行三任务 `max_medium_flag=0.0`，最低高度
  0.170/0.170/0.097 m，均高于 `hg_m/2 = 0.085 m`。
- 净距改为表面净距（减去 `platform_radius_m`），机间表面净距 1.79–3.07 m。
- 样本台账期望网格独立生成，`valid_sample_ratio=1.0`，实测采样周期 0.0500 s。
- 异常不释放：任务未被接受时不释放资源、不派发下一任务。
- `release_lag_s` / `plan_revision` / 派发实际读到的 `planned_start` 已记录，
  `executor_id=aav_formation` 与派发端点一致。
- 基线资格改为记录"准入时的那一份快照"（30.9 s，完整），修掉了历史上因样本
  窗口滑动导致记录值缩短的问题。

**M1 旧数据复评**：三个旧 bag 用 `reevaluate_air_domain.py` 按录制时的消息语义
解码、只补 AIR 域检查，结果均为 `air_domain_ok=false`：
`mission-e` `max_flag=0.165 / min_height=0.057`，`mission-f` `0.005 / 0.084`，
`mission-g-fault` `1.000 / -1.276`。原 `verification.json` 未被改写，复评写入
`revaluation.json` 并复制到 `evidence/`。

**M3（静态路由 + 离线算例）：完成。**
`formation_air.yaml` 增加 `executors` 静态路由（一个在线单元 + Action 端点），
runner 校验计划联盟与在线单元成员一致、记录分配与实际执行一致；`executors.py`
改为每个单元独立维护转场位置与可用时刻（含"A 去远处、B 留在原地"的反例测试）。
离线五平台（3 AAV + 1 USV + 1 UUV）用三个单平台 AAV 做三类算例
（只有 USV 合格 / 三台 AAV 都合格但代价不同 / 两个 AIR 任务竞争），
新增 5 项测试。**不宣称**物理资源竞争已验证。

**M2（有障碍闭环）：未通过，如实记录。**
`experiments/20260918-mission-m2`（obstacle on，障碍在编队路径上 x=-23, y=0）：
T1 通过；T2 `task_outcome=PASS` 但 `safety_outcome=FAIL`
（障碍净距 0.108 m < 0.20 m），随后资源被锁存、不再派发 T3；
本地点云对七台中的三台确有消息（说明感知链在补丁后确实接通），
但成员到障碍盒的最近距离为 **0.0 m**，即实际穿过了障碍。
结论：本配置下尚未证明规避能力，M2 不能算通过。
根因之一已定位并修复：上游 CPU 渲染器只接受第一条全局地图
（`if (has_global_map) return;`），后注入的障碍永远进不了规划器；
现以构建期补丁 `integration/swarm_qn_bridge/patches/pcl_render_node_accept_map_updates.patch`
在镜像构建时应用（`upstream/` 快照未改，补丁已记录）。

**证据**：`evidence/` 下按运行保存 config/metrics/verification/逐动作诊断与复评，
rosbag 不入库（记录绝对路径与大小）。

> 每次改动与实验的「计划 / 实际 / 效果」逐条记在 [docs/WORKLOG.md](../docs/WORKLOG.md)；本文件只保留当前状态与下一步。

## M2 定位与修复：第 1-2 步结果（2026-09-18）

按冻结的《M2 定位与修复实施基线》执行，进度到第 2 步为止。

**第 1 步（地图与几何修复）：完成。**
- 场景话题 `/scene/global_cloud` 由场景发布器独占，渲染器/Action/readiness/验证器共用一处配置。
- 修掉一个真实缺陷：上游 CPU 渲染器在**全局地图明确为空**时崩溃（对空点云做体素化和建树），进程直接死掉；此前所有运行的"本地点云零消息"就是它。用构建期补丁（`integration/swarm_qn_bridge/patches/`）改为：空地图=已初始化且障碍集为空；一次有效扫描无返回点时发布带时间戳的空点云，帧改为 `world`。
  实测：7 个渲染器进程存活，本地点云 ~12 Hz 且带真实时间戳，`ready=true` 且 `known_empty_map=false`——扫描链由数据验证，不再靠豁免。
- 盒子点云与实体几何同源：原先采样只覆盖 `round(size/res)` 个点，漏掉声明的 +x/+y/+z 面；现在精确覆盖 `[-size/2, +size/2]`。
- 判据分离：实际用 `d_actual ≥ d_required`（半径只减一次），参考用 `d_ref ≥ d_required + e_budget`（`e_budget` 运行前声明）；参考不足只作余量结论，不算实际碰撞；无盒子时盒子检查记 `NOT_APPLICABLE`（不等于 `NOT_VERIFIED`）。
- 平面仍只作解析边界、不加点云，且按**全程**最小高度判定；违规在循环内即时锁存并结束任务。

**第 2 步（无障碍基线重评）：按新规则不通过，停在这一层。**
- 0.5 m 巡航时，qn 在任务开始约 0.5 s 内的竖直瞬态使最低成员到 0.237 m，球包络下沿到 −0.011 m，穿过声明的 z=0 平面；把规划速度从 1.5 降到 1.0 m/s 后同样是 0.237 m，说明**不是参考强度问题**。
- 据此前已接受的"先压参考、不够再提高度"，做了最小目标 z 接口补丁（上游 replan FSM 原本硬编码 0.5，现读取目标 z），把场景巡航高度声明为 0.8 m，并同步 Action 校验、初始高度与 centers。
- 结果 `experiments/20260918-m2-baseline-clean`：三任务全部 `task_outcome=PASS` / `safety_outcome=PASS` / `experiment_validity=VALID`，最低高度 0.380/0.502/0.426 m，`obstacle_check=NOT_APPLICABLE`，基线 30.20 s。
- 累计时间门槛曾不过：`model/ROS rate` 0.99928，72.8 s 累计偏差 57 ms > 50 ms。**已定位并解决**：循环本身是 100.0 Hz（p99 步长 10.07 ms），偏差来自两次 16–32 ms 的停顿——bag 写 home 盘的间歇写回；固定步长不补算，停顿就永久留在偏差里。隔离实验（不录 bag）偏差 3.5 ms；照常录制但落到 tmpfs 偏差 0.64 ms、门槛通过、三任务全 PASS/PASS/VALID（`experiments/20260918-m2-baseline-shm`）。启动脚本已改为内存盘暂存 + 结束后搬回实验目录。
- 因此第 2 步的停止条件已满足，第 3、4 步解锁。局部扫描中断所需的"运行中扫描停更即判故障"规则已在代码中实现并通过单测。

## 全链路可视化（2026-09-18）

用户要求"可视化仿真整个实验，从任务分配到编队控制"。原有的 `plot_formation_experiment.py`
是按主题分三张图，没有一条把分配→派发→飞行→控制→安全→时间放在同一时间轴上的视图，也没有回放。

新增（只读，不重仿真；不在记录里的数就不画）：

- `plot_mission_overview.py`：一张总览图，7 个面板——任务甘特（含 30 s 基线窗口、计划 vs 实际、
  派发、驻留、完成事件）、分配路由（执行单元/成员/端点/`trajectory_id` 前后/采用判定/`release_lag_s`/
  是否修复）、XY 轨迹与每个任务的槽位布局、槽位误差（标出驻留窗口，`epsilon_p` 只在驻留段把关）、
  对 qn 实际采用参考的跟踪误差、高度与 AIR 域、实际净距、时间漂移。
- `animate_mission_replay.py`：GIF 回放，场景 + 槽位 + 障碍 + 有界尾迹，右侧联动槽位误差与高度曲线。
- `scripts/docker_render_figures.sh experiments/<run> [fps]`：一条命令出上面两张图。
- 两者已加入 `CMakeLists.txt` 的 `catkin_install_python`。

出图过程中修掉的语义错误（都是"看图才发现"）：无障碍场景仍画盒子净距曲线（必须用
`obstacle_scenario` 门控，否则会显示一条并不存在的负净距）；槽位误差在运输段天然很大，不能当作
跟踪误差信号；`action_task_outcome` 是枚举整数，字符串判定要取每任务 `verdict`。

当前能看到的（`experiments/20260918-m2-baseline-shm`）：T1/T2/T3 各晚 9.1/10.5/4.3 s，
`release_lag` 0.000/0.069/0.065 s，`plan_revision=3`、`updated_plan_used=true`、
`dispatch_changed=false`（串行单资源下是预期结果），三任务 `PASS/PASS/VALID`，
最大模型-ROS 漂移 0.00064 s。167 项单元测试通过。

下一步仍是 M2 第 3 步（原盒子场景定位，盒子保持 `(-23,0,0.5)` / `(1,1,1.2)` 不动，不缩小障碍换通过）
与第 4 步（局部扫描中断）。

## P3 离线多资源扩展（2026-09-18 完成）

plan.md 的 P3 要求在 P0–P2 通过后**单独**增加离线执行单元模型。本轮按其定义实现：

- 新增 `integration/mrta_python/executors.py`：`Executor(executor_id, physical_agent_ids,
  capabilities, available_from, nominal_speed_mps)`、`ExecutorPlan`/`ExecutorPlanItem`、
  `ExecutorTravelTimeProvider`、`eligible_executors`、`build_executor_plan`、
  `validate_executor_inputs`/`validate_executor_plan`。
- 复用已移植的 v9 奖励序；唯一推广是每个单元维护自己的队列完成时间，候选的最早开始时间就是
  该单元队列的结束时间（只有一个单元时与原来的全局队列完全一致）。资格由能力与人数决定，
  代价在合格单元之间决定，同一单元上的任务按队列串行，平局按 `executor_id` 打破。
- plan.md 要求的三类算例都有命名回归测试：只有A有资格；A/B都有资格但代价不同；两个任务竞争
  同一执行单元（另有“存在第二个合格单元时不会被迫排队”的对照）。
- 边界：只在计划层验证资源选择与竞争，不拆分当前七机物理编队，未接入 ROS 物理链路，也不复制
  完成/修复语义（`process_completion`/`plan_repair` 仍是固定联盟语义）。

验证：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=integration python -B -m pytest -q
-p no:cacheprovider integration/mrta_python/tests/test_executors.py` → 11 项通过；两个包的
全部单元测试 128 项通过。

## 文献研究

`research/literature/manifest.yaml`与`research/literature/notes/`（7篇中文笔记）已按plan.md优先级
完成，每篇回答“改变哪个接口定义”和“要求增加哪个反例测试”；开放PDF放在
`research/literature/papers/`（已被`.gitignore`排除）。GRSTAPS无开放版本，笔记按摘要整理并标注“仅摘要”。

## 下一次最小入口

plan.md 的 P0–P3 已完成。下一步的候选（按 plan.md 未列入 P3、需另行确认才动手）：把离线执行
单元层接入真实链路前的额外验证、正等待吸收（Calvo 式等待时间）以及跨介质模型；也可以先做本轮
尚未在 ROS 层覆盖的取消/执行超时故障注入。这些都属于新范围，动手前应与用户确认优先级。

## 后续每次只需更新这些事实

本次目标；实际上游/版本；做了什么；真实命令、结果与日志位置；未完成/失败；选型是否变化；
下一项最小工作。其他细节可留在原生日志或实验目录。

## 当前提醒

第一版目标是完整联合协调闭环，原始复现只是第一步。本次通过的是固定七机AIR、单一联盟、
单一执行单元的串行任务链；正等待吸收、多执行单元与跨介质仍未验证。不要因为链路跑通就
把“联合协调闭环已完成”写成事实。

## Skill 资料库补充交接（2026-09-17）

按用户确认清单，使用系统`skill-installer`并显式指定`--ref`、`--path`和`--dest`，
将9个来源的21个Skill下载到`/home/lhj/skill-library/`。原始目录、附带资源和适用许可已保留；
21个主入口及31个本地索引链接核对通过。来源版本、使用条件、仅收录链接的工具和待编写建议见
[中文索引](/home/lhj/skill-library/README.md)。没有最终下载失败项；未接入宿主、运行附带脚本或安装依赖。
当前交付为独立资料库，后续实际启用按具体任务另行选择，不改变海上集群工程现有实施状态。

## P0–P2 验收顺序逐项对照（plan.md 11 项）

| # | 验收项 | 本轮证据 | 证据类型 |
|---|---|---|---|
| 1 | 标准qn Odometry与Swarm兼容输入语义分别正确 | 两个话题的frame/child_frame、位姿一致、机体系twist与世界系线速度关系实测残差0.0 m/s；兼容速度与发布位姿数值导数99分位差0.004–0.005 m/s | ROS运行（7机×3任务）+验证器 |
| 2 | qn实际采用参考可追溯到trajectory_id | 3个任务×7台全部`ADOPTED`，adopted_trajectory_id落在该动作记录的trajectory_id集合内 | ROS运行+验证器 |
| 3 | ActionServer忙时不会自动抢占当前任务 | 忙/未就绪状态机路径与拒绝语义的单元测试；历史运行`20260918-mission-a-reject-t3`记录过一次真实`REJECTED_NOT_READY`，改版runner改为等待`READY_IDLE`后重试 | 单元测试+历史ROS运行；本轮串行runner未制造并发Goal |
| 4 | 取消、失联和超时不会错误释放资源 | 故障注入运行`20260918-mission-g-fault`（任务中途kill drone_3 qn节点）：T2以`reason=2 ODOMETRY_TIMEOUT`中止，`run_state=UNKNOWN_LOCKED`、`resource_released=false`，后续任务不派发；取消与执行超时为单元测试 | ROS运行（失联）+单元测试（取消/超时） |
| 5 | 模型时间、ROS时间与七机累计偏差通过门槛 | 30.1 s任务前基线合格；全程累计偏差0.00054 s、跨节点0.00079 s、模型/ROS速率1.0000001、时间偏差导致的参考位移0.00086 m（限0.25 m） | ROS运行+验证器 |
| 6 | 单次七机任务完成真实到位和模型时间驻留 | 三任务`task_outcome=PASS`；最终槽位误差0.226/0.023/0.001 m；模型驻留5.02/5.01/5.03 s（要求5.0 s） | ROS运行+验证器 |
| 7 | A→B→Return完成目标交接 | 三个execution_id/GoalID/目标点依次派发，`updated_plan_used`对每个动作成立 | ROS运行+验证器 |
| 8 | 三个verdict分开输出 | 逐动作诊断与验证器`verdicts`分别给出；碰撞场景的PASS/FAIL分离由`test_experiment_verdict.py`覆盖 | ROS运行+单元测试 |
| 9 | DelayEvent幂等，repair前后计划可复查 | `events == processed_events`（3个）、同一事件只处理一次；`plan_history`每条含`plan_before`与修复后计划 | ROS运行+验证器 |
| 10 | repair-on/off的作用能否被解释 | 见上文：两模式都等待真实完成；差异体现在`plan_updated`与修复后的`planned_start`，`dispatch_changed`不是repair效果读数 | ROS运行（-e与-f对照） |
| 11 | 无法证明新参考归属时不得标记为完整有效 | `decide()`在adoption未确认时给出`experiment_validity=INCOMPLETE`（`test_experiment_verdict.py`）；历史运行`20260918-mission-a-adoption-bug`记录过真实的`REFERENCE_ADOPTION_UNCONFIRMED`→INCOMPLETE | 单元测试+历史ROS运行 |

以上第3、4项中的“取消/超时”只有单元测试证据，尚未做ROS层故障注入；不得写成已在真实链路上验证。
