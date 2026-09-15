# 论文与源码核查记录：第一版选型依据

**核查日期：2026-09-15。范围：不读取旧HUC；依据用户上传V1明确撤回其继承条款。**

## 阅读状态的含义

“正文重点核读”表示已检查对应方法、假设、理论或实验章节，不表示独立复算了全部证明与结果。“代码核对”表示读取官方README/指定源码，不代表编译运行。没有把搜索摘要或第三方GitHub评论当作正式实验结论。

本轮直接使用论文原文、作者机构/会议页面与GitHub官方仓库检索，不以有限条数的文献插件输出作为检索全集。未宣称穷尽所有相关论文；优先深入与选型决定直接相关的工作。

## A. 重点证据表

| 工作 | 本轮重点核读 | 代码证据 | 对采用决策的影响 |
|---|---|---|---|
| Swarm-Formation ICRA2022 | 与后续论文对照、原生目标和消息机制 | 官方README、分支、已有源码接口 | 已复现基线继续保留；不当完整任务系统 |
| 同作者T-RO2023 | 编队代价解耦、固定时间采样、ALAS、GRLR、全局编队路径与实验 | 作者代码入口指向原仓库；未确认独立完整后续实现 | 不声称已有完整T-RO源码可直接下载 |
| Primitive-Swarm T-RO2025 | 运动基元、碰撞查表、在线选择、规模/长时/真实实验、局限 | README及pp_replan_fsm.cpp；ROS pub/sub与目标入口 | 新增首要运动后端复现，不冒充编队算法 |
| Number Adaptive Formation 2025 | DG/DVS、PAAS、同配置和5G假设、实验 | 未确认官方完整仓库 | 前沿参照，不做首轮依赖 |
| CAT-ORA T-RO2025 | 问题定义、A1–A6、算法和理论适用边界 | README、原生服务和C++调用示例 | 开阔区域编队重排可选件，不替代复杂障碍导航 |
| D-ITAGS RA-L2023 | 正文及Figure1、模型、交错求解、修复、理论界和实验 | README，C++/OMPL/Gurobi依赖和使用说明缺口 | 最直接的联合协调参照；不假称一键部署成熟系统 |
| Calvo T-RO2025 | §III模型、分配/修复方法、执行架构和实验 | 两仓库README、规划/执行Action、Task消息 | 首选资源任务基线；任意前置关系不属原模型 |
| RSS2025 H-LTL/GCS | 模型、产品图、正确性/完备性条件、实验 | 官方RSS页面、代码README与例子 | 新近强理论参照；不引入全部机械臂/GPU场景 |
| RMADER RA-L2024 | 延迟检查、轨迹发布、保证假设、实验 | 官方Readme.md与软件依赖 | 延迟对照，不保证任意断联 |
| DMPC-Swarm 2025 | 通信/计算组织、信息跟踪器、Assumption1和安全证明、16机实验 | 官方仓库入口与README | 控制—通信联合的重要反例：已有相关理论；水下不能直接继承 |
| CoCoPlan RA-L2026 | 正式出版记录、任务与团队间歇通信机制 | 本轮未确认完整官方源码 | 近邻查新/算法对照，不列必装依赖 |
| DANCERS SIMPAR2025 | 官方软件/作者说明和框架用途 | 官方README及会议入口 | 联合仿真候选，非控制理论与水声全栈 |
| Stonefish ICRA2025 | 论文概览及官方库能力说明 | 官方README明确2019基础和2025更新 | 海洋物理仿真候选，不强制采用视觉学习模块 |
| Fossen模型/GNC | 作者官方模型和软件说明 | 官方源码入口 | 海面/水下建模依据，不包装成2025原创系统 |

## B. 关键核查结论

### B1. Primitive-Swarm的数字与边界

论文报告1000台仿真与8台真实飞行，不能写成1000台实飞。1000机实验为减少ROS连接开销使用进程共享内存。官方源文件当前定义USE_SHARED_MEMORY为false，存在原生轨迹消息收发；因此也不能反过来说整套源码只能通过共享内存运行。

它主要改进在线规划开销与可扩展性。论文结论主动说明任意时刻重规划会带来加速度不连续。严格编队形状维护、完整海洋动力学和任意通信丢失的证明不是该工作提供的能力。

- 正文版本：https://arxiv.org/html/2502.16887v1
- 原生入口：https://github.com/ZJU-FAST-Lab/Primitive-Planner/blob/main/src/planner/plan_manage/src/pp_replan_fsm.cpp
- 论文/源码见[R03]。

### B2. T-RO2023的“task assignment”不是业务任务调度

ALAS里的assignment针对机器人与编队目标槽位；GRLR协调编队重组和局部规划。它不等于解决“谁去检查哪个风机、能源如何安排”。局部轨迹目标仍含障碍、机间和速度/加速度罚项，不能仅因论文标题有Robust就当作任意通信失效与真实欠驱动动力学的严格安全保证。

作者页面的Code链接与原Swarm仓库相同。本轮搜索ALAS等符号未定位出完整后续实现，但搜索不到不是证明未公开，因此记录为“完整源码覆盖未核实”，而不是“作者没有开源”。

- 正文版本：https://arxiv.org/html/2210.04048v2
- 作者入口：https://zhouxin.me/
- 论文/源码见[R02]。

### B3. CAT-ORA保证范围必须保留

A1–A6包含机器人/目标可互换、起终静止、直线路径与等效时间参数化、球体模型、足够起终间距，以及起终点凸包内无其他障碍。最优性与安全结论在这些条件下讨论，不能直接转为任意异构平台在狭窄障碍环境中的全局最优。

代码提供get_assignment、get_reshaping_trajectories服务与C++库接口，适合作为一个明确动作调用，不需要另造庞大编队系统。

- 正文版本：https://arxiv.org/html/2412.00603v2
- 论文/源码见[R05]。

### B4. 上层两类任务模型不能默默混为一种

Calvo正文§III明确“不含任务前置关系和特定开始时间窗”。任务接替relay是执行工作交班，并非无线中继。其配套执行框架中的低层具体控制仅提供简单示例。

D-ITAGS原文Figure1明确分配、MILP调度、运动规划及可行性/耗时反馈；§III任务网络包含前置与互斥关系。资源分析中的分配数量不能替代电池能量模型。它的定理是算法/问题模型层面的结果，不是所有控制闭环的稳定性结论。

因此先依据任务需要选一个在线上层。可以借另一个的模型或执行器，但不得宣称“两者全能力已经自动相加”。

- Calvo正文：https://arxiv.org/html/2411.02062v3
- D-ITAGS正文：https://arxiv.org/pdf/2209.13092
- 论文/源码见[R08–R10]。

### B5. RSS2025已经直接连接任务逻辑与连续运动

H-LTL/GCS将任务自动机与可行运动的凸集表示组合，并把分配包含在求解中。正确性、完备性相对于论文规定的兼容性、空间分解和轨迹表示等条件，不是对未知海洋环境的无条件保证。实现依赖Drake、MONA、ltlf2dfa和MOSEK，例子集中在移动/机械臂交接。

它是“任务图与运动图融合是否新颖”的强近邻，不能跳过；但第一版没有必要把机械臂对象交接、整套Drake环境一并搬进海上系统。

- 正式会议：https://www.roboticsproceedings.org/rss21/p099.html
- 正文：https://arxiv.org/html/2504.18899v2
- 论文/源码见[R11]。

### B6. 延迟与丢包已有不同处理路线

RMADER的核心假设涉及已提交轨迹及最大消息时延；任意断联不在其原保证范围。DMPC-Swarm通过信息跟踪与消息恢复安排DMPC，正文Assumption1需要可用的实际—名义跟踪界，还依赖同步通信/计算结构；强扰动破坏该假设时原保证不成立。

DMPC-Swarm报告16台Crazyflie和外部计算单元，不是每台航行器全部在机载独立计算的水下系统。两者都值得复现，但不能各取一个安全模块随意串联后仍照搬原定理。

- RMADER：https://arxiv.org/html/2303.06222v6
- DMPC-Swarm：https://link.springer.com/article/10.1007/s10514-025-10211-w

### B7. 新不等于更贴合本项目

DG预印本显式使用同配置航行器和高可靠5G。Primitive论文的千机证明对象主要是算力可扩展性。CAT-ORA的强保证来自受约束的无障碍重排问题。DANCERS是仿真基础设施。把这些差别写清楚，才能以问题选择代码，而不是以年份替换一切。

## C. 最小接口的可定位源码

| 需求 | 原生文件 | 本轮实际看到的语义 |
|---|---|---|
| 请求上层规划 | `mrta_execution_architecture/action/HeuristicPlanning.action` | scenario_id、available_agents、remaining_tasks → success、TaskQueue[] |
| 给一台机器人下发任务列表 | `action/NewTaskList.action` | agent_id、Task[] → ack与status |
| 任务结束反馈 | `action/TaskResult.action` | Task、FAILURE/SUCCESS/HALTED、ack/status；另有原应用近查参数 |
| 任务表示 | `msg/Task.msg` | 原生id/type及不同动作参数；并非完全通用海上格式 |
| 通用时长示例 | `msg/GenericTaskParams.msg` | Td、Tw、Te；仅这些字段并不提供任意路径几何 |
| Primitive目标、本机状态与邻机轨迹 | `src/planner/plan_manage/src/pp_replan_fsm.cpp` | 原生Odometry订阅、目标订阅、trajectory广播、轨迹/primitive输出 |
| 编队重排库接口 | `ctu-mrs/catora/README.md` | 起终配置、运动约束、dt → 槽位匹配或轨迹 |

这张表说明哪些接口可继承。它不意味着这些不同项目已经互相认识其消息类型。必要的适配是目标/动作/完成语义的映射，不是重新定义每一层的状态和轨迹数据包。

## D. 未采用或延后理由

| 候选 | 当前决策 |
|---|---|
| EGO-Planner-v2 | 有价值的已存在多机导航线，不将“v2”误写成2025新方法或完整业务任务系统 |
| GRSTAPS整仓库 | 研究依据保留，但不为一张总框架同时引入多个重型上层工程 |
| RSS2025整套机械臂/Drake例子 | 可独立做小例子，不作为第一版强制运行环境 |
| CAT-ORA | 无开阔区域重排需求时暂不安装 |
| AMSwarmX | 等复杂障碍问题成为主要失败源后纳入统一对照 |
| DG 2025 | 发表/开源状态未确证，先阅读不锁为依赖 |
| DANCERS | 先完成原生消息闭环再接，不与其他网络连接框架重复建设 |
| 石鱼视觉/学习扩展 | 仅使用所需海洋物理与接口，不扩展到本项目不负责的图像感知 |

## E. 正文与源码可用性不等于保证继承

本轮没有运行新增仓库，没有测试用户机器环境，没有独立重建所有实验曲线，没有证明组合系统。后续每一项结论分别标记：论文提出、代码找到、原例复现、集成通过、定理条件符合。不能互相替代。

---

## 参考文献与官方入口

### [R01] Swarm-Formation / Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments

Quan et al.. **ICRA 2022**。

- 论文/作者资料：https://arxiv.org/abs/2109.07682
- 官方源码：https://github.com/ZJU-FAST-Lab/Swarm-Formation


### [R02] Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments

Quan et al.. **T-RO 2023, 39(6):4785–4804; 10.1109/TRO.2023.3301295**。

- 论文/作者资料：https://arxiv.org/abs/2210.04048
- 官方源码：https://github.com/ZJU-FAST-Lab/Swarm-Formation


### [R03] Primitive-Swarm: An Ultra-Lightweight and Scalable Planner for Large-Scale Aerial Swarms

Hou et al.. **T-RO 2025, 41:3629–3648; 10.1109/TRO.2025.3573667**。

- 论文/作者资料：https://arxiv.org/abs/2502.16887
- 官方源码：https://github.com/ZJU-FAST-Lab/Primitive-Planner


### [R04] Number Adaptive Formation Flight Planning via Affine Deformable Guidance in Narrow Environments

Yuan Zhou, Jialiang Hou, Guangtong Xu, Fei Gao. **arXiv 2509.18636, 2025；本轮未核实正式发表状态**。

- 论文/作者资料：https://arxiv.org/abs/2509.18636
- 官方完整源码：本轮未确认，不据此推断一定未公开。


### [R05] CAT-ORA: Collision-Aware Time-Optimal Formation Reshaping for Efficient Robot Coordination in 3-D Environments

Kratky et al.. **T-RO 2025, 41:2950–2969; 10.1109/TRO.2025.3547296**。

- 论文/作者资料：https://arxiv.org/abs/2412.00603
- 官方源码：https://github.com/ctu-mrs/catora


### [R06] AMSwarmX: Safe Swarm Coordination in CompleX Environments via Implicit Non-Convex Decomposition of the Obstacle-Free Space

Adajania et al.. **ICRA 2024**。

- 论文/作者资料：https://arxiv.org/abs/2310.09195
- 官方源码：https://github.com/learnsyslab/AMSwarmX


### [R07] GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling

Messing et al.. **IJRR 2022; 10.1177/02783649211052066**。

- 论文/作者资料：https://doi.org/10.1177/02783649211052066
- 官方源码：https://github.com/GT-STAR-Lab/GRSTAPS


### [R08] D-ITAGS: A Dynamic Interleaved Approach to Resilient Task Allocation, Scheduling, and Motion Planning

Neville, Chernova, Ravichandar. **RA-L 2023；arXiv初版2022**。

- 论文/作者资料：https://arxiv.org/abs/2209.13092
- 官方源码：https://github.com/GT-STAR-Lab/D-ITAGS


### [R09] Heterogeneous Multirobot Task Allocation for Long-Endurance Missions in Dynamic Scenarios

Calvo, Capitán. **T-RO 2025, 41:6494–6513; 10.1109/TRO.2025.3626651**。

- 论文/作者资料：https://arxiv.org/abs/2411.02062
- 官方源码：https://github.com/multirobot-use/mrta_heuristic_planner


### [R10] Mission Execution Architecture for Multi-UAV Teams

multirobot-use；对应Calvo等研究. **R09配套执行软件；另引用ICUAS 2022, 10.1109/ICUAS54217.2022.9836234**。

- 论文/作者资料：https://doi.org/10.1109/TRO.2025.3626651
- 官方源码：https://github.com/multirobot-use/mrta_execution_architecture


### [R11] Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems

Zhongqi Wei, Xusheng Luo, Changliu Liu. **RSS 2025; 10.15607/RSS.2025.XXI.099**。

- 论文/作者资料：https://www.roboticsproceedings.org/rss21/p099.html
- 官方源码：https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS


### [R12] Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay in Dynamic Environments

Kondo et al.. **RA-L 2024, 9(2):1476–1483; 10.1109/LRA.2023.3342561；另有ICRA 2023前序版本**。

- 论文/作者资料：https://arxiv.org/abs/2303.06222
- 官方源码：https://github.com/mit-acl/rmader


### [R13] DMPC-Swarm: Distributed Model Predictive Control on Nano UAV Swarms

Gräfe, Eickhoff, Zimmerling, Trimpe. **Autonomous Robots 2025, 49:28; 10.1007/s10514-025-10211-w**。

- 论文/作者资料：https://link.springer.com/article/10.1007/s10514-025-10211-w
- 官方源码：https://github.com/Data-Science-in-Mechanical-Engineering/DMPC-Swarm


### [R14] CoCoPlan: Adaptive Coordination and Communication for Multi-Robot Systems in Dynamic and Unknown Environments

Zhang et al.. **RA-L 2026, 11(3):3270–3277; 10.1109/LRA.2026.3656769**。

- 论文/作者资料：https://arxiv.org/abs/2601.10116
- 官方完整源码：本轮未确认，不据此推断一定未公开。


### [R15] DANCERS: A Physics and Network Co-Simulator for Communicating Multi-Robot Systems

Balaguer et al.. **SIMPAR 2025; 10.1109/SIMPAR62925.2025.10979148**。

- 论文/作者资料：https://doi.org/10.1109/SIMPAR62925.2025.10979148
- 官方源码：https://github.com/Chroma-CITI/DANCERS


### [R16] ROS-NetSim: A Framework for the Integration of Robotic and Network Simulators

Calvo-Fullana et al.. **RA-L 2021, 6(2):1120–1127; 10.1109/LRA.2021.3056347**。

- 论文/作者资料：https://arxiv.org/abs/2101.10113
- 官方完整源码：本轮未确认，不据此推断一定未公开。


### [R17] Stonefish: Supporting Machine Learning Research in Marine Robotics

Grimaldi et al.. **ICRA 2025；原始Stonefish论文为OCEANS 2019**。

- 论文/作者资料：https://arxiv.org/abs/2502.11887
- 官方源码：https://github.com/patrykcieslak/stonefish


### [R18] Fossen Marine Systems Simulator / Python Vehicle Simulator

Thor I. Fossen. **模型与GNC软件；理论参考Fossen 2021第2版专著，不作为新顶会方法**。

- 论文/作者资料：https://www.fossen.biz/pythonVehicleSim/
- 官方源码：https://github.com/cybergalactic/PythonVehicleSimulator
