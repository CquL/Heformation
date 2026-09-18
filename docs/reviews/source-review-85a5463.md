# Heformation：源码审查、任务—编队对接与修复方案

> **历史文档，针对 `CquL/Heformation@85a5463338e0bd732129286e926e8281a745f315`。**
> 后续部分结论已被当前实施更新：AIR 域边界、点云/感知链、样本对齐口径与
> 执行单元路由都在 `8362db8` 之后有了新的证据和实现，现状以
> [QN_INTEGRATION](../QN_INTEGRATION.md) 与 [context/15_handoff.md](../../context/15_handoff.md)
> 为准。本文件保留原样，只用于记录当时的审阅结论。

## 0. 审阅范围与证据边界

审阅快照：`CquL/Heformation@85a5463338e0bd732129286e926e8281a745f315`。

当前主线保持：受限 Calvo v9 Python 实现 → ROS1 FormationAction → 固定七机 Swarm-Formation → qn AIR → 实际 Odometry → 完成反馈 → 时间修复。本文不恢复旧 HUC，不要求重新选择 D-ITAGS，不增加新的强制安装清单。

本次工作是主链源码、launch/config、构建、测试和相关原论文/官方代码的核对。没有运行用户的 ROS/Docker，没有重放用户的 rosbag，没有修改或推送仓库。用户报告的两次 `FormationAction server unavailable` 是运行事实输入；当前公开 `experiments/` 仅有 README，本次无法独立复核那两次完整日志与镜像内容。以下区分源码事实、条件性推论和改进建议。

## 1. 总结

当前已有可以保留的受限任务计划器、Action、组级监视器、任务客户端、结果去重和 rosbag 检查代码，但没有成功的完整物理任务链记录。首要工作不是换开源框架，而是修复启动条件与实际后端不一致的问题，然后核准时间与动作语义。

最重要的新增发现：

1. 默认 CPU 地图渲染只发布点云 `cloud`，Action 就绪门控却强制等待七路 `depth`。在当前默认构建与启动组合下，这是足以使服务始终不启动的源码级阻塞。
2. 就绪测试使用 mock，并明确断言订阅 depth、不订阅 cloud，测试也复制了同一个错误假设。
3. 固定七机、零等待、逐个等待 Action 完成的场景中，repair 改变预测计划，不一定改变机器人下一次实际启动。需要区分信息反馈闭合与决策效果。
4. qn 固定积分步长与按 ROS 时间取样的参考轨迹存在时间一致性风险。未核实实时运行比例之前，不能把所有延迟都解释为平台性能。
5. 当前 Action 取消会返回 PREEMPTED，但没有停止 Swarm/qn。与“不支持运行中抢占”的既定边界不一致。

## 2. 现状与能力范围

| 部分 | 当前源码/记录支持的能力 | 当前不支持的结论 |
|---|---|---|
| Swarm + qn | 既有七机 AIR 基础运动运行记录，保留原生 PositionCommand/Odometry 边界 | 完整任务系统、跨介质系统、所有真实轨迹安全已验收 |
| Python v9 子集 | 七人固定联盟的动态任务排序、名义时间计算 | 一般异构资源分配、充电、工作接替、任意联盟选择 |
| FormationAction | 目标校验、组级目标发布、实际状态驻留判定、反馈/结果代码 | 当前组合已经能启动并完成一次真实任务 |
| plan_repair | 在固定顺序与联盟下传播正延迟、更新未开始任务时间 | 重分配、重排、几何重规划、障碍故障恢复 |
| 验证器 | rosbag 的目标、结果、七机驻留和计划变化检查代码 | 全路线无碰撞、动力学跟踪保证、调度性能优势 |

`plan.md` 当前已经显式冻结 `wait_time=0`，禁止正等待；所以“等待吸收”不是当前实现漏报通过的能力，而是尚未纳入当前问题范围。

## 3. P0：Action 不可用的直接源码证据

### 3.1 默认实际发布的地图数据

`upstream/Swarm-Formation/src/uav_simulator/local_sensing/CMakeLists.txt` 设置：

```cmake
set(ENABLE_CUDA false)
```

因此生成的 `pcl_render_node` 使用 `src/pointcloud_render_node.cpp`，不是 CUDA 深度图实现。这个源文件发布：

```cpp
nh.advertise<sensor_msgs::PointCloud2>("pcl_render_node/cloud", 10);
```

它不发布 depth。仿真 launch 将该私有主题映射为 `/drone_i_pcl_render_node/cloud`。`advanced_param.xml` 中也有规划器 `grid_map/cloud` 的 remap。

### 3.2 Action 等待的却是另一种输出

`integration/qn_aav_simulator/scripts/formation_action_server.py` 对每台机器人订阅：

```text
/drone_i_pcl_render_node/depth
```

只有 `map_seen` 且七台全部进入 `depth_streams_seen`，再通过其他检查之后，才执行 `self.server.start()`。超时后节点退出。`formation_air.launch` 将其设为 required 节点。

因此，默认构建下即使七台 qn 正常发布 Odometry，Action 仍无法通过门控执行 `start()`；客户端的 `server unavailable` 不等于 ROS actionlib 库坏了。

这是当前源码组合的明确问题。实际两次运行是否使用相同镜像，需用保留的 `image-id.txt`、`launch.log` 和最后一条 `readiness_reason` 复核。不能仅凭客户端报错排除其他启动问题。

### 3.3 修法

- 按实际启用的地图输入建立 readiness，不把 depth 写死为所有后端必需条件。
- 读取真实 remap/配置，确认规划器接收的是 PointCloud2 还是 depth；不为了满足错误门控启用 CUDA 或伪造传感器。
- 不只做字符串 depth→cloud 替换。CPU renderer 在附近没有点时可能不发布局部点云，必须区分“地图未初始化”与“已知地图中的空区域”，避免把空区域误当启动失败。
- 现有配置一致性、七台状态有效性可以保留，但节点命名、地图就绪条件应来自选定后端，不依赖无关的可视化或感知输出。
- 可将 Action 服务启动和可执行状态分开：服务能够报告尚未就绪，但在必需条件满足前不得下发运动。此项是实现建议，不要求另建消息系统。
- 客户端失败时保存服务端具体 readiness 原因，不能只留下 `server unavailable`。

在同一 ROS master、已经 source 运行环境的终端中，启动期间检查：

```bash
rosparam get /formation_action_server/readiness_reason
rostopic info /drone_0_pcl_render_node/depth
rostopic info /drone_0_pcl_render_node/cloud
rosnode info /drone_0_ego_planner_node
```

观察“主题存在”时还要看 publisher，不能把只有 Action 订阅者的 depth 主题误判为有有效发布。

### 3.4 测试为什么没有发现

`test_formation_action_server.py` 主要以模拟 ROS 模块构造执行器，并手动设置地图及 depth 就绪。其 `test_startup_observes_the_renderers_actual_depth_topics` 更明确断言七路 depth 存在且不订阅 cloud。

建议替换这项错误断言，并只增加一个针对实际默认构建的启动—Action 连接集成测试。无需大规模新增 smoke、哈希或检查框架。mock 用于状态转换与边界测试，不能证明实际 topic 连接。

## 4. MRTA 子集：有原算法依据，但资源选择已经退化

### 4.1 当前实际计算

`validation.py` 要求恰好七台机器人、每任务恰好需要七台、联盟及其顺序不变，拒绝任何正 `wait_time`。

`schedule.py` 每轮对剩余任务比较：

```text
(-urgent, introduced_makespan, 0, -service_time, -1, travel_time)
urgent = deadline < current_makespan + 1.55 * service_time
```

对照原 MATLAB `heuristicTaskAllocator.m` 的版本9奖励段，在不拆分、无接替、零联合等待、全部七台均合格、相同转场抽象的范围内，倒数奖励转为正成本排序有清楚的代数依据。

但这只说明受限排序规则可对应。Python 随机序列、初始可用时刻处理和真实运动模型并未因此与 MATLAB 等价。原始函数会针对每个候选任务选择机器人、考察电池和同步引入的等待；这些在当前实现中没有对应决策空间。

### 4.2 正确的当前名称

固定七机执行单元上的、基于 Calvo v9 退化奖励的任务排序与零等待时间传播基线。

这不是无价值：它可以验证任务真正驱动下层，以及时间预测如何被实际结果修正。但不能用这一个配置验证异构资源调度优势。

### 4.3 后续如何恢复真正分配

当前物理链修好后，先单独建立小规模计划层用例：多个独立执行单元、不同资格/可用时刻、不同转场与服务代价。有些任务只能由部分执行单元完成，至少存在一个真实可选分配。

不能仅删除 `len(agents)==7` 就认为支持一般 MRTA。当前全部任务绑定全体成员、代表成员队列修复和全局串行 dispatch 都依赖固定联盟假设。

两个建模层要区分：

- 任务分配：任务由哪个平台或固定队伍承担。
- 队内槽位分配：承担该任务的成员在队形中占什么位置。

当前把七名成员聚合为一次组级 Action 是合适的。之后若将固定队伍当作规划器的执行单元，需要明确这是本项目的资源聚合模型；其成员、时间占用与组级代价不能凭空相加，同一物理机器人不能同时属于两个执行中的资源组。

## 5. 时间与状态：哪些是预测，哪些才是事实

### 5.1 当前名义值可解释，但不含物理保证

`TravelTimeProvider` 计算编队中心欧氏距离 / 同一个名义速度，未使用 executor 参数估计平台差异，也不查询障碍或真实路径。

当前 YAML 中 start=-26、A=-24、B=-22、Return=-26，速度1.5 m/s，三个驻留均5秒。在任务顺序 T1、T2、T3 下，名义完成时刻分别约6.333、12.667、20.333秒。这是根据配置的手工代入，不是实际运行结果。

`planned_start` 已被定义为资源占用与转场开始，这个语义可以保留。建议将估计写清为：

```text
预计完成 = 计划开始 + 预计转场 + 预计稳定到位 + 必需驻留
```

当前 `travel_time` 没有体现加减速、七机调整、绕行和稳定到位。即使 max_vel 与名义速度一致，也不能由距离/速度证明截止时间可满足。

### 5.2 使用已有 provider 边界改善，不重写总系统

`build_plan` 已接受可调用的 `travel_time_provider`，这是合适的解耦点。

推进方式：

1. 固定配置，得到真实转场/稳定到位/驻留时间记录；失败单独统计，不只保留成功案例。
2. 用已有地图的可行路线长度和实际运动配置计算更合理的先验耗时；必要时使用离线或无副作用的原规划器查询。
3. 通过独立验证场景评估预测误差与迟到，不把在同一条路线拟合的误差当作泛化结果。
4. 地图、编队尺度、速度参数或平台改变时，旧估计不可不加判断地复用。

不能在搜索候选任务时向在线七机发布真实目标来“查询代价”；查询不能改变当前生效动作。

### 5.3 优先核查时间推进一致性

`qn_aav_node.py` 按默认100 Hz每次推进固定0.01秒，后端再以0.001秒子步积分；状态时间戳却读取 `rospy.Time.now()`，主循环用 `Rate.sleep()`。当前 launch 没有建立专门统一的 `/clock` 驱动。

这不是断言已经发生时间错误，但存在明确的条件性风险：机器负载使循环只达到40 Hz时，1秒ROS墙钟只推进约0.4秒模型时间，traj_server参考时间仍可能推进1秒。这样产生的“延迟”混合了计算欠实时与模型跟踪，不宜都归为平台能力不足。

先记录每个平台：

```text
模型累计积分时间 / 对应ROS经过时间
```

必须来自实际积分步数，不能把rosbag接收条数直接当积分次数。第一轮可以维持实时模式并验证比例接近1；若无法稳定实时，再在仿真驱动层协调时钟，qn方程和控制律保持不变。不能只设置 use_sim_time=true 而没有时钟源。

### 5.4 名义轨迹、实际状态与指标不要混淆

当前 monitor 的 `max_position_error` 是对最终槽位的距离，不是全过程轨迹跟踪误差。命名应改清楚，或在报告中明确其含义。

控制跟踪误差应比较同一时间基准下的 `actual_position(t)` 与 `reference_position(t)`。机间距离使用不同成员最近一次样本时，是异步采样估计，不等于连续时间最小安全间距；离线应时间对齐并注明采样精度。

当前 qn 消费位置与航向，内部参考速度从位置差分产生，不能把 PositionCommand 中有 acceleration/yaw_dot 说成原qn控制器已直接使用这些前馈。现有速度/加速度限值诊断也不等于物理状态硬限幅。

## 6. repair 改计划不一定改变执行：需要因果对照

`repair.py` 的实际作用是对固定队列执行：

```text
start_next = max(old_start_next, actual_release)
finish_next = start_next + travel + service
```

这在当前零等待范围内有合理含义，并且正确保留已完成的名义计划历史、用事件另存实际时间。事件去重以及与dispatch时计划对齐的做法可以保留。

但 `formation_mission_runner.py` 本来就逐次等待前一项 Action 真实完成，之后才发下一项。假设原S2=F1，T1实际在C1>F1结束，客户端R1>=C1才收到结果，则：

```text
不开repair：T2最早派发 = max(R1, 原S2) = R1
开repair：  T2最早派发 = max(R1, 新S2=C1) = R1
```

这说明修复预测计划与改变下一任务实际派发，是两个不同结果。它不否定当前信息闭环的工程价值，但不能据“Plan时间改变”宣称控制更优或资源调度效果。

增加相同反馈下 repair-on/off 对照，记录预测更新时间、下一任务真正的派发时间及最终完成时间。若二者一致，报告为预测计划维护，不掩饰。之后用真实同步等待、独立资源竞争或可选择任务顺序检验决策价值，不为了通过测试人为增加无业务意义的等待。

当前 `validate_plan` 明确禁止正等待，README也记录这一范围。所以不能把小于0.1秒的容差忽略称为“等待裕度吸收”。

## 7. Action接口方向正确，但取消/失败必须与物理状态一致

### 7.1 保留现在的轻接口

`Formation.action` 的Goal只有任务ID、带坐标系的中心目标、驻留时间，结果为起止时间与原因，反馈为阶段。可以保留，不需要重新设计一套统一合同或把高频位置放回任务包。

`Plan.items` 单一权威状态与派生视图、qn单一状态源、任务与ROS原生GoalID关联也应保留。

### 7.2 正在执行的取消与既定方案不符

当前 Action 每轮将 `is_preempt_requested()` 传给monitor，monitor直接返回PREEMPTED；没有对应的Swarm停止/保持动作。runner等待结果超时也会调用cancel。

这意味着任务结果可以变成取消，而平台仍执行原目标。当前runner异常退出会阻止它继续派任务，但并未建立真正的物理退出语义。

第一阶段的最小策略：拒绝新的运行中抢占，排队任务可取消；活动任务取消请求不应自动释放资源。先完成当前合法动作或进入已经验证的退出状态。未实现这种退出时，明确报告“不支持运行中取消”，不能把API的PREEMPTED当作平台已经停稳。

默认SimpleActionServer的新Goal策略需要一起处理；只要求正常客户端不发新Goal，不足以成为全接口保证。

### 7.3 失败不能都留在执行日志里

当前runner遇到非成功结果即失败退出，没有失败→剩余问题更新→新方案这条路径。这与“当前只做成功完成后的时间修复”一致，但不能称完整故障重规划。

近期先分清两类诊断：规划后端未在限定条件下提供可执行参考；已有参考但平台未在允许时间内完成。没有轨迹不必然意味着几何无解，地图未就绪、计算超时等也可能造成同样现象。不要用timeout直接证明目标被障碍阻断。

失败时保留失败原因、真实最后状态和资源仍占用/未知的事实。重新分配机制后置，不能把故障的七机组拆成任意六机继续调用原动作。

## 8. 配置与验证：修来源，不堆检查

当前同一组槽位存在于Swarm YAML、Action YAML、monitor默认字典，初始位置还手写在launch中。Action已有 `_planner_configuration_matches`，并不是完全没有防护。

建议以Swarm实际加载的槽位/scale为一个来源，Action和评分脚本读取本次解析出的配置。仅保留一次明确的一致性验证；不要继续复制数据后增加更多不同实现的比对。

初始center与实际七机配置应核对。每次新任务只发中心和驻留；每机终端槽位从同一份配置推得。

原验证器已有较好的真实Odometry驻留重算，但范围还要补清：

- “到位并驻留通过”不等于整个移动过程无碰撞。
- 当前最小机间距只记录，不参与成功判定；障碍净距未加入全程检查。
- verify脚本要求mission中至少有一次repaired，适合专项延迟试验，不适合定义所有正常任务试验的通过条件。
- `Return` 目前是普通任务，当前参数得到它在最后，不代表Calvo模型天然有“全部任务完成后再返航”的约束。固定顺序测试与自由任务排序要分别标识。

## 9. 如何把资源调度真实地用到编队任务

任务分配不应该直接输出七条电机命令；它选择可执行单位、任务和时间。编队后端负责组内槽位与连续运动。

阶段性模型建议：

```text
任务：需要哪种动作、在哪个区域、服务多久
资源：哪个已经验证的固定组或单机能执行
代价：该资源从当前状态到目标的预计时间/耗能
执行：组级Action → 成员槽位 → Swarm → qn
反馈：实际完成/失败 → 剩余计划
```

联合任务服务开始通常需要所有必要成员到位，组级估计不能取平均到达时间代替最晚到达条件。已有固定组共同运动时，可直接估计整组完成转场与稳定的时间。

队伍不是一个没有尺寸的中心点。本配置尺度2、相对槽位半径约2，队伍展开半径约4米。中心路线可行不代表全部成员路径可行。可先用保守队伍几何范围进行候选路线筛查；筛查未通过只表示该保守表示未证明可行，不能证明允许变形后的编队无路可走。

未来两组真实执行还要分离目标命名空间和身份，并保持跨组避碰信息，不能让一个 `/move_base_simple/goal` 同时驱动本应执行不同任务的两组。

## 10. 针对这些缺口最值得借鉴的论文与源码

下列是方法和源码依据，不是本轮新增安装清单。原文保证只在原文假设下使用。

| 文献 | 可以借鉴的内容 | 本项目具体采用方式 | 不应误用 |
|---|---|---|---|
| Calvo & Capitán, T-RO 2025 | 任务分配、机器人执行管理、延迟监测、repair与重新规划分离 | 补齐行为结果、剩余问题和时间组件的连接，逐段对照v9原码 | 不把当前退化子集说成原完整资源模型 |
| Quan等, T-RO 2023 | 编队对齐、槽位分配、重组和全局/局部规划协调 | 将任务执行联盟与队内槽位匹配区分，使用队伍几何而非只有中心 | 不默认当前公开版本覆盖论文所有模块 |
| GRSTAPS, IJRR 2022 | 任务/分配/调度/运动的交错反馈 | 使用已有travel_time_provider边界引入运动代价与可行性 | 不必恢复旧HUC，也不必重启其失败依赖安装 |
| APEX-MR, RSS 2025 | 根据实际完成依赖释放动作，处理不确定执行时间 | 参考任务完成触发与时序依赖，减少无意义全局等待 | 机械臂停止/等待等假设不能自动推广水下平台 |
| H-LTL/GCS, RSS 2025 | 明确把任务逻辑、分配和连续运动可行域联系起来 | 为后续显式任务依赖与运动可行性建立建模对照 | 不把画图或字段新增说成该方法已实现 |
| FaSTrack, CDC 2017 | 规划模型与真实跟踪模型之间的误差界 | 对qn实际跟踪范围与规划安全余量建立接口依据 | 有限测试最大误差不是确定性安全界 |
| Robust MADER, RA-L 2024 | 异步共享轨迹与有界延迟下的安全更新 | 通信扩展阶段比较轨迹更新和旧新计划兼容 | 不以此声称任意断联均安全，也不当额外串行安全层 |

### 本轮特别有用的新参考：APEX-MR

其RSS2025论文与官方执行文档明确：规划阶段的时间用于构建计划图，而实际执行根据前驱节点是否完成释放动作。这里要借的是“依赖与预测时间分开”的设计，而不是把全部LEGO/MoveIt系统移植进来。它具有双臂验证，不能将其保证直接平移到海上编队。

## 11. 优先修复表

| 优先级 | 修改点 | 本次可检查的完成标准 |
|---|---|---|
| P0 | Action readiness与默认CPU云接口对齐，移除错误depth测试 | 默认镜像启动后Action可连接，未就绪原因可见，不伪造地图/状态 |
| P0 | 保留失败镜像对应及launch日志 | 确认两次运行与源码的对应，不再只保存客户端通用报错 |
| P1 | 取消/抢占与资源释放语义 | 活动目标不被第二请求悄悄覆盖，不因API终止释放仍运动的组 |
| P1 | 时间推进与真实完成计时 | qn积分时间和轨迹时间关系可解释，单动作真实完成与bag一致 |
| P1 | 槽位一个配置来源 | Swarm、Action和评分用同一份解析后配置，拒绝坐标/高度不一致 |
| P2 | 单动作→连续任务→实际反馈repair | 同一流程成功记录，不把mock/服务启动当七机物理通过 |
| P2 | repair开关因果对照 | 区分预测变化和实际派发变化，不宣称无实验支持的效率收益 |
| P3 | provider加入运动信息或校准 | 独立测试预测误差，路径查询无执行副作用，失败不被返回成正常成本 |
| P3 | 多执行单元计划场景 | 真实资格与资源选择存在，非仅删除seven校验；之后才多组物理实验 |

无需等所有上游、海洋模型、通信仿真全部完成。也不应为了尽快闭环，弱化所有组级完成判定、直接复制位置或者把超时当成功。

## 12. 建议的五项实验与报告口径

1. 默认地图模式真实启动与Action连接：只证明接口启动。
2. 一项MOVE_AND_HOLD：七机实际状态、连续有效驻留、目标一次派发；不声称调度效果。
3. 固定A→B→Return：动作接续与物理反馈；不声称自由任务排序。
4. 受限v9自由任务+反馈：计划实际决定顺序，反馈维护预测；repair-on/off区分实际决策收益。
5. 多执行单元离线分配：至少一个资格约束和一个资源竞争影响结果；单独标记为计划层，不当作多域实物验证。

对所有成功运动试验分别报告：目标到达误差、参考轨迹跟踪误差、组内形状误差、采样机间距、障碍净距、平台限幅/诊断、预计与实际任务时间。这些指标含义不同，不用一个“最大位置误差”混合代替。

## 13. 本阶段创新定位

先修上述确定问题并形成可靠基线。后续可以研究“编队几何与实际执行能力反馈到资源调度”的增量，例如：不同成员组和编队在不同通道里的实际完成成本，如何影响任务选择、组建和时序。必须通过有效对照证明，这不是简单多张图融合或字段叠加。

当前不声称提出并验证统一控制理论；但也不要求所有子模块从零原创。新的方法应来自已有方法接口处可复现的缺口，并明确其模型、约束、算法和条件性保证。

## 14. 可追溯来源

### 当前项目的固定源码

下列链接均固定到审阅快照，便于之后比较修改。

- [plan.md](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/plan.md)
- [当前状态](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/context/02_current_status.md)
- [Action服务](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/scripts/formation_action_server.py)
- [任务客户端](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/scripts/formation_mission_runner.py)
- [组级监视器](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/src/qn_aav_simulator/formation_monitor.py)
- [Action定义](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/action/Formation.action)
- [启动配置](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/launch/formation_air.launch)
- [YAML配置](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/config/formation_air.yaml)
- [计划构造与旅行时间](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/mrta_python/schedule.py)
- [时间修复](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/mrta_python/repair.py)
- [约束检查](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/mrta_python/validation.py)
- [默认地图编译分支](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/upstream/Swarm-Formation/src/uav_simulator/local_sensing/CMakeLists.txt)
- [CPU点云发布器](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/upstream/Swarm-Formation/src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp)
- [规划器输入remap](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/upstream/Swarm-Formation/src/planner/plan_manage/launch/advanced_param.xml)
- [Action mock测试](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/tests/test_formation_action_server.py)
- [真实bag验证器](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/scripts/verify_formation_experiment.py)
- [qn状态包装节点](https://github.com/CquL/Heformation/blob/85a5463338e0bd732129286e926e8281a745f315/integration/qn_aav_simulator/scripts/qn_aav_node.py)

### 原论文与上游代码

- Calvo, Á., Capitán, J. *Heterogeneous Multirobot Task Allocation for Long-Endurance Missions in Dynamic Scenarios*. T-RO 2025. DOI: 10.1109/TRO.2025.3626651. [全文](https://arxiv.org/html/2411.02062v3) · [规划源码](https://github.com/multirobot-use/mrta_heuristic_planner) · [执行源码](https://github.com/multirobot-use/mrta_execution_architecture)
- [本轮对照的v9奖励原码](https://github.com/multirobot-use/mrta_heuristic_planner/blob/c69509e8d2370f55824519dda0a13252e1817415/src/heuristicTaskAllocator.m)
- Quan等. *Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments*. T-RO 2023. DOI: 10.1109/TRO.2023.3301295. [全文](https://arxiv.org/html/2210.04048v2) · [相关官方仓库](https://github.com/ZJU-FAST-Lab/Swarm-Formation)
- Messing等. *GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling*. IJRR 2022. DOI: 10.1177/02783649211052066. [作者页及代码入口](https://star-lab.cc.gatech.edu/papers/messing-grstaps/)
- Huang等. *APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly*. RSS 2025. DOI: 10.15607/RSS.2025.XXI.098. [会议论文页](https://www.roboticsproceedings.org/rss21/p098.html) · [全文](https://arxiv.org/html/2503.15836v3) · [代码](https://github.com/intelligent-control-lab/APEX-MR) · [执行文档](https://github.com/intelligent-control-lab/APEX-MR/blob/main/docs/04-execution.md)
- Wei, Z., Luo, X., Liu, C. *Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems*. RSS 2025. DOI: 10.15607/RSS.2025.XXI.099. [会议论文页](https://www.roboticsproceedings.org/rss21/p099.html) · [全文](https://arxiv.org/html/2504.18899v2) · [代码](https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS)
- Herbert等. *FaSTrack: a Modular Framework for Fast and Guaranteed Safe Motion Planning*. CDC 2017. [论文](https://arxiv.org/abs/1703.07373)
- Kondo等. *Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay in Dynamic Environments*. RA-L 2024. DOI: 10.1109/LRA.2023.3342561. [全文](https://arxiv.org/html/2303.06222v6) · [代码](https://github.com/mit-acl/rmader)

主链源码事实以上述固定版本为准；外部论文和代码用于针对性对照，不代表其全部工程在本地已运行。源自本报告的修复建议需要另行实施和实测。
