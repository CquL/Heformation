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
