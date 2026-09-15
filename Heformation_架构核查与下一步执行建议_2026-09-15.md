# Heformation：当前架构核查与下一步执行建议

日期：2026-09-15  
核查对象：`CquL/Heformation`  
固定快照：`405eb2ce6dc7e487c0b1a87d693d21270767afc8`  
范围：仓库目录、当前V2实施计划、Docker构建、launch overlay、Swarm规划/轨迹执行关键代码、qn适配器与Python后端、现有冒烟测试，以及相关上游论文/源码定位。

> 这是远程静态源码核查与实施建议，不是本次重新运行Docker、重新执行七机试验或验证Python与Simulink逐采样等价的报告。用户和仓库记录的既有试验结果，与本次直接从源码确认的行为分别描述。未读取旧HUC，不要求继承其接口。没有修改在线仓库。

## 1. 结论

当前项目应定位为：**Swarm-Formation公开编队运动基线，已经通过原生PositionCommand/Odometry边界接入qn AAV的Python控制与动力学；已有AIR示例运行记录，完整任务调度闭环尚未接入。**[S1][S2]

继续沿`context/09_implementation_plan.md`的V2路线推进，但更新近期待办：先把真实完成判定、命令消费语义和运行记录补清，再把一个原生上层任务框架接入；Primitive-Planner作为并行新增的导航基线，不取代尚未比较的编队能力。不要把复现T-RO 2023所有高级模块、安装全部候选或者先完成所有海洋平台，设成接入上层的前置条件。[S3]

## 2. 当前目录已经重组，不是仍待重组

仓库实际使用：

```text
upstream/Swarm-Formation/       上游源码
integration/qn_aav_simulator/  qn数值后端与ROS节点
integration/swarm_qn_bridge/   simulator.xml覆盖文件
models/qn/qn.slx               原始模型
scripts/                      Docker构建、演示、冒烟测试
context/                      V2背景与实施计划
```

宿主机目录不再是原始根目录`src/planner/...`布局。Docker构建先复制上游形成`/workspace`中的catkin工作区，再覆盖：

```text
integration/swarm_qn_bridge/simulator.xml
  -> /workspace/src/src/planner/plan_manage/launch/simulator.xml
```

同时复制qn包到容器工作区。因此，当前不需要再进行一轮目录迁移。按当前脚本构建，保持上游副本与集成覆盖的边界即可。[S1][S4][S5]

正确的上游提交是：

```text
967a4bdfae949e994691f8ffc87dbb0147cbebb7
```

它与Heformation的当前提交是两个不同版本标识。用户长文末尾的另一个短尾串不是这个提交的正确完整写法。本轮读取时，该提交仍是官方Swarm-Formation默认分支的最新提交，日期2024-01-24，提交内容为Dockerfile/Makefile合并；这不是“完整T-RO 2023算法实现已发布”的证明。[S6]

## 3. 当前实际运行链

```text
RViz 2D Nav Goal / 原生目标
       |
各无人机EGOReplanFSM
       | 本机Odometry、地图、已收到的邻机PolyTraj
       v
Swarm-Formation轨迹优化与编队项
       v
分段五次多项式PolyTraj
       v
traj_server
       v
quadrotor_msgs/PositionCommand
       v
qn_aav_node.py
       v
位置+航向参考；位置参考差分形成内部速度参考
       v
qn RBF/PD -> 十路执行器一阶动态 -> 六自由度数值积分
       v
本机Odometry + medium_flag
       v
规划器/局部地图/可视化反馈
```

这里的`EGOReplanFSM`类名来自当前Swarm代码，不表示另接了一个EGO-Planner-v2系统。`visual_slam/odom`是沿用的topic名称，不表示项目现在研发或运行了新的SLAM算法。[S7][S8]

## 4. 必须修正的语义与验证范围

### 4.1 PositionCommand有字段，不等于qn控制律消费全部字段

| 内容 | 当前实现 |
|---|---|
| position | 直接作为ROUTE_POSITION参考 |
| yaw | 进入qn参考 |
| velocity | 节点接收并传入数据对象；活动控制分支用连续位置参考的离散差分构造内部速度参考；消息velocity用于结果诊断 |
| acceleration | 节点接收，但后端明确记录`acceleration_directly_consumed=False` |
| yaw_dot | 当前ROS回调未读取 |

因此不要描述成“所有PVA+yaw/yaw_dot都已逐项进入原控制器”。这也不自动说明适配错误：保留qn原控制语义可能有意如此。不能为了补齐字段而直接添加前馈，然后继续宣称控制律未改变。[S8][S9]

### 4.2 原生planning/finish不是实际到达的验收信号

`ego_replan_fsm.cpp`在局部目标接近全局目标、名义轨迹执行时间接近结束时发布`planning/finish=true`。该分支没有检查qn实际Odometry是否进入最终容差区。这在接入有跟踪动态的qn后，不能直接用于上层任务完成判定。[S7]

建议仅增加薄动作结果判定：对于到达后停留任务，检查本机实际位置误差、实际速度和持续时间；编队任务检查全部实际参与者。使用明确的当前目标/任务关联，避免旧布尔完成信号被新任务误用。指标阈值由具体动作设定，不把“静止后才完成”强行用于所有巡航任务。

```text
名义轨迹结束 -> 可作为提示
实际运动满足任务完成条件 -> 向上层报告SUCCESS及真实耗时
```

这是当前接任务层前最重要的适配工作，不需要新建一套复杂消息协议。

### 4.3 七机冒烟测试不等于完整性能验收

`docker_test_qn_swarm.sh`实际检查：7个qn节点存在；drone_0能收到位置命令；20秒后drone_0的x变化超过0.05m。它没有自动断言全部七机最终到位、全程无碰撞、全过程误差、执行器裕度或0.00062m指标。[S10]

0.00062m来自用户与`QN_INTEGRATION.md`中的既有运行记录，本次没有重算。该最终误差不能作为全程跟踪误差上界。增加一个记录/评价脚本，读取现有命令与实际状态并持久化结果即可，不必重建测试系统。[S2]

最小记录：每机参考与实际位置/速度、目标容差进入时间、最小机间距、障碍净距、已有执行器诊断、仿真时间与墙钟运行时间。

### 4.4 当前手动编队目标入口并非任意三维目标入口

`formationWaypointCallback()`拒绝z低于-0.1的输入，随后把编队中心z固定为0.5m；各成员目标由固定相对位置生成。普通手动单机入口也有自己的固定高度逻辑。[S7]

这不等于底层轨迹优化只有二维，但说明**当前demo入口不能直接接水下目标**。后续需要在薄目标适配和对应平台规划能力确认后开放，而不是只把目标z改成负数。

### 4.5 当前七机编队不是任意任务联盟API

默认launch启动ID 0—6，FSM读取7个相对槽位。当前入口不是“任意agent_id列表+可变队形”的任务级接口。上层即使输出了3机组和2机组，原七机demo也不会自动按这个分配拆队。[S7][S11]

接入时先明确最小动作单元：单机动作、固定编队动作或者已支持的子队动作。只给下层下发实际能表达的任务；动态成员变更单独验证。固定七机作为一个执行单元的接口试验可以做，但不能把它独自称为充分验证了任意任务分配。

### 4.6 其余必要但应局部处理的接口检查

| 检查 | 已读源码事实 | 处理建议 |
|---|---|---|
| 速度/加速度限制 | qn步进传入2m/s、8m/s²；后端主要用作越限诊断，不裁剪实际状态 | 与规划器参数区分，记录实际越限；不要把诊断参数称为保证 |
| 能源 | AgentState有默认energy=1.0，当前ROS步进没有能耗更新或电量输出 | 接Calvo前明确能源来源；第一版可用明确标注的续航模型，不能从6DOF存在推导已有电池模型 |
| 诊断输出 | 后端产生执行器与极值diagnostics，但ROS节点只输出odom/medium | 优先保存已有diagnostics，无须新增大量消息 |
| Odometry坐标 | 位置/线速度按world发布，角速度为body，child_frame_id为base_link | 记录并校核原生消费约定；Swarm当前直接把linear读为世界速度，不能只改发布端旋转而不改接收端 |
| 仿真时钟 | qn每循环固定推进dt，轨迹服务器以ROS当前时间取样 | 检查CPU负载时是否实时运行；尚未证明已有漂移，不先增加复杂同步系统 |
| 邻机消息 | 接收PolyTraj时检查abs(now-start_time)是否超过0.25s | 这是实现门限，不是经证明的最大通信时延；后续延迟实验记录被门限丢弃与网络丢包的区别 |

以上依据ROS节点、后端和FSM。Odometry标准规定twist对应child_frame_id，而此工程有上游惯例兼容问题，应同时检查两端，不宜盲改。[S7][S8][S9][S12][S13]

### 4.7 Python模型接通不等于原Simulink逐步等价

文档称数值方程取自qn.slx。实际AIR示例可以支持软件链路和该设置下的运动结果；不能代替同输入、同初态、同积分规则下的Python/Simulink数值对照。当前采用STATIC_TRIM初始化，其与原模型初始化的对应也需记录。未获得对照证据前，称“Python转写模型的AIR闭环”，不称“实物验证”或“完整跨介质模型已认证”。[S2][S9]

## 5. 高飞团队更新与现有基线是什么关系？

### 5.1 Swarm-Formation与T-RO 2023

论文《Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments》确实研究了更完整的编队导航，包括编队对齐/槽位匹配和Global-Remap-Local-Replan等。[P1]

但论文中的能力不能直接归给当前仓库所有运行路径。当前代码入口仍需与论文模块逐项对照；本轮没有确认一个独立、完整覆盖这些模块的新官方仓库。看到Munkres或swarm_graph名称，也不证明已经调用了T-RO对应完整方法。

### 5.2 Primitive-Swarm / Primitive-Planner

《Primitive-Swarm: An Ultra-Lightweight and Scalable Planner for Large-Scale Aerial Swarms》，T-RO 2025；对应官方代码`ZJU-FAST-Lab/Primitive-Planner`。[P2][S14]

它是轻量多机导航、避碰和规模扩展路线，不是严格编队保持的直接升级包。论文还明确讨论任意时刻换运动基元会产生加速度不连续。对qn而言，规划更快不自动意味着跟踪更好，必须接相同下层比较。

结论：按V2并行复现原始Primitive例子，再评估与qn的薄接入；在形成编队能力对照前，不删除当前Swarm基线，也不让两个规划器同时发同一机器人的命令。

### 5.3 EGO-Planner-v2

官方README定位为Science Robotics 2022《Swarm of micro flying robots in the wild》的Swarm Playground，包含formation等工作空间。[S15]

因此不能把名字中的v2理解为“Swarm-Formation的2023或2025全功能升级版”。它是另一条明确的公开实现，应按能力与代码比较。

### 5.4 更近期编队研究

2025年《Number Adaptive Formation Flight Planning via Affine Deformable Guidance in Narrow Environments》以及2026年FLIP预印本值得跟踪，分别涉及人数变化/可变形引导以及点云配准式编队规划。[P3][P4]

这些工作在本轮所检材料中没有被确认具备可直接替换当前系统的完整官方代码和本地复现证据。因此列为进一步核查项，不作为当前接任务层的阻塞条件，也不冒称正式发表状态或已可部署。

## 6. 是否继续按计划MD？

按`context/09_implementation_plan.md`的V2六步推进，不采用已经废止的旧V1。[S3]

建议把近期待办调整为：

| 顺序 | 当前动作 | 最小交付 |
|---|---|---|
| 1 | 固化405eb2ce的Swarm+qn AIR基线，补真实完成判定和输入消费说明 | 一个命令/实际状态/完成时间的可重复记录；不重写规划器 |
| 2A | 单独复现Calvo资源任务小例；有一般任务前置需求时对照D-ITAGS小例 | 冻结一个在线主调度器及其支持的任务类型 |
| 2B | 并行复现Primitive官方导航例子，核对更近期编队公开能力 | 保留原始结果，再决定是否接qn比较；不阻塞2A |
| 3 | 原生任务执行器连接当前运动后端 | 任务请求→执行→实际完成反馈，只有必要动作映射 |
| 4 | 让一项执行延迟或绕行改变上层安排 | 实际耗时导致时序、队列或执行者变化的例子 |
| 5 | 分别扩展水下/跨介质和受限通信 | 分模型、分通信条件报告结果，不一次开启全部扰动 |

不把“先完整复现T-RO 2023全部重组能力”设为上层前置。新后端是否替换由同一任务、同一qn后端、同一环境和信息条件下的比较决定。

## 7. 上层选择与最小完整架构

Calvo适合当前以资格、能源、持续时间、联合作业和工作接替为主的任务；其论文原问题不包含一般任务前置关系与任意开始时间窗。D-ITAGS更直接面向任务依赖、调度和运动交错修复。必须依据第一场景选择，不把两个在线调度器叠加。[P5][P6]

```text
外部任务请求、平台/资源配置、任务地图
                      ↓
母船侧一个任务规划/调度器（Calvo 或 D-ITAGS）
                      ↕ 实际耗时、任务结果、执行延迟
上游原生执行器 + 薄动作适配
                      ↓
当前Swarm编队动作 / 经验证后选择的其他运动动作
                      ↓
原生轨迹服务器与PositionCommand
                      ↓
qn控制器、执行器与动力学
                      ↓
实际Odometry和任务完成判定
```

qn是AAV模型。后续验证它的WATER/TRANSITION分支时，不需要先把该AAV的动力学换成Fossen；另增加USV/UUV时，再选对应的平台模型。Stonefish是仿真基础设施，不是自动替代任务、规划和控制的总系统。

第一条完整任务闭环允许先在已验证AIR域建立，但不以此宣称完成海面/海下或异构物理验收。

## 8. 当前不应做的事

- 不返回旧HUC，不做新一轮大目录迁移。
- 不因“论文较新”就覆盖唯一已经跑通的编队基线。
- 不把Primitive作为严格编队算法的同义词。
- 不把所有PositionCommand字段、diagnostics字段和energy字段当作已实际实现的能力。
- 不把planning/finish或drone_0移动测试当作真实任务完成证据。
- 不为集成新增大批重复消息和多层通用封装。
- 不把既有论文保证直接传递给新控制器、qn模型和通信条件。

## 9. 来源

### 仓库固定版本

[S1] [Heformation README](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/README.md)

[S2] [QN_INTEGRATION.md](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/docs/QN_INTEGRATION.md)

[S3] [当前V2实施计划](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/context/09_implementation_plan.md)

[S4] [Dockerfile.qn](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/docker/Dockerfile.qn)

[S5] [simulator.xml overlay](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/integration/swarm_qn_bridge/simulator.xml)

[S6] [Swarm-Formation固定上游提交](https://github.com/ZJU-FAST-Lab/Swarm-Formation/commit/967a4bdfae949e994691f8ffc87dbb0147cbebb7)

[S7] [EGOReplanFSM](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/upstream/Swarm-Formation/src/planner/plan_manage/src/ego_replan_fsm.cpp)

[S8] [qn_aav_node.py](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/integration/qn_aav_simulator/scripts/qn_aav_node.py)

[S9] [qn_python_backend.py](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/integration/qn_aav_simulator/src/qn_aav_simulator/qn_python_backend.py)

[S10] [七机冒烟测试](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/scripts/docker_test_qn_swarm.sh)

[S11] [normal_hexagon.launch](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/upstream/Swarm-Formation/src/planner/plan_manage/launch/normal_hexagon.launch)

[S12] [contracts.py](https://github.com/CquL/Heformation/blob/405eb2ce6dc7e487c0b1a87d693d21270767afc8/integration/qn_aav_simulator/src/qn_aav_simulator/contracts.py)

[S13] [ROS Odometry.msg语义](https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg)

[S14] [Primitive-Planner官方仓库](https://github.com/ZJU-FAST-Lab/Primitive-Planner)

[S15] [EGO-Planner-v2官方README](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)

### 论文

[P1] Quan等，Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments，T-RO 2023。[作者全文](https://arxiv.org/html/2210.04048v2)。

[P2] Hou等，Primitive-Swarm: An Ultra-Lightweight and Scalable Planner for Large-Scale Aerial Swarms，T-RO 2025，DOI 10.1109/TRO.2025.3573667。[作者全文](https://arxiv.org/html/2502.16887v1)。

[P3] Zhou等，Number Adaptive Formation Flight Planning via Affine Deformable Guidance in Narrow Environments，2025预印本。[作者论文](https://arxiv.org/abs/2509.18636)。

[P4] Zhou等，FLIP: Real-Time and Resilient Formation Planning for Large-Scale DIstributed Swarms via Point Cloud Registration，2026预印本。[作者全文](https://arxiv.org/html/2605.29704v1)。

[P5] Calvo、Capitán，Heterogeneous Multirobot Task Allocation for Long-Endurance Missions in Dynamic Scenarios，T-RO 2025。[作者全文](https://arxiv.org/html/2411.02062v3)；[规划代码](https://github.com/multirobot-use/mrta_heuristic_planner)；[执行架构](https://github.com/multirobot-use/mrta_execution_architecture)。

[P6] Neville等，D-ITAGS: A Dynamic Interleaved Approach to Resilient Task Allocation, Scheduling, and Motion Planning，RA-L 2023。[作者论文](https://arxiv.org/abs/2209.13092)；[官方代码](https://github.com/GT-STAR-Lab/D-ITAGS)。
