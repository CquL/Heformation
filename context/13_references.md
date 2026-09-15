# 来源、论文与官方代码入口

**整理日期：2026-09-15。下列R01–R18原样保留V2核查记录中的题名、年份、链接及未确认状态，仅增加定位锚点。此次文档更新未重新在线核验、下载或运行这些项目。**

## 本资料包的直接依据

<a id="s00"></a>
**[S00] 用户在当前对话中的最新决定。**不再读取/继承旧HUC；先复现权威开源成果；原生接口最少适配；第一版保留完整联合规划协调器；实验后确定创新；不做业务感知。

<a id="s01"></a>
**[S01]** [第一版实施方案V2：开源组合与最小接口](sources/implementation_plan_v2_2026-09-15.md)。2026-09-15既有文档，本包完整保留，不修改原文来制造新的核查结论。

<a id="s02"></a>
**[S02]** [论文与源码核查记录](sources/literature_audit_2026-09-15.md)。2026-09-15既有记录，“本轮核读”指该记录产生时的研究，不代表此次整理重新执行了研究。

<a id="s03"></a>
**[S03]** [新方案接续说明](sources/handoff_v2_2026-09-15.md)。来自V2资料包，说明哪些旧决定已失效。

<a id="s04"></a>
**[S04]** 用户提供的[控制指标截图](sources/report_control_requirements.png)与[路径规划成果截图](sources/report_planning_patent.png)。仅支撑可见字段，不补写完整申报书中未提供的内容。

## 来源优先与未知处理

旧V1材料不是当前执行依据。V2中的未完成事项仍是未完成；最新用户指令与稳定范围优先，未来有新选择需同步当前状态和交接。原论文与代码事实需按版本核对，无法确认就保留未知，不把“未找到”写成“没有公开”。

所有官方链接为文献与复现入口，不表示软件已安装、当前仍兼容或可直接组合。进一步的在线查新属于后续任务，不在此次资料重组范围。

## 文献与代码：沿用V2的编号

<a id="r01"></a>

### [R01] Swarm-Formation / Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments

Quan et al.. **ICRA 2022**。

- 论文/作者资料：https://arxiv.org/abs/2109.07682
- 官方源码：https://github.com/ZJU-FAST-Lab/Swarm-Formation


<a id="r02"></a>

### [R02] Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments

Quan et al.. **T-RO 2023, 39(6):4785–4804; 10.1109/TRO.2023.3301295**。

- 论文/作者资料：https://arxiv.org/abs/2210.04048
- 官方源码：https://github.com/ZJU-FAST-Lab/Swarm-Formation


<a id="r03"></a>

### [R03] Primitive-Swarm: An Ultra-Lightweight and Scalable Planner for Large-Scale Aerial Swarms

Hou et al.. **T-RO 2025, 41:3629–3648; 10.1109/TRO.2025.3573667**。

- 论文/作者资料：https://arxiv.org/abs/2502.16887
- 官方源码：https://github.com/ZJU-FAST-Lab/Primitive-Planner


<a id="r04"></a>

### [R04] Number Adaptive Formation Flight Planning via Affine Deformable Guidance in Narrow Environments

Yuan Zhou, Jialiang Hou, Guangtong Xu, Fei Gao. **arXiv 2509.18636, 2025；本轮未核实正式发表状态**。

- 论文/作者资料：https://arxiv.org/abs/2509.18636
- 官方完整源码：本轮未确认，不据此推断一定未公开。


<a id="r05"></a>

### [R05] CAT-ORA: Collision-Aware Time-Optimal Formation Reshaping for Efficient Robot Coordination in 3-D Environments

Kratky et al.. **T-RO 2025, 41:2950–2969; 10.1109/TRO.2025.3547296**。

- 论文/作者资料：https://arxiv.org/abs/2412.00603
- 官方源码：https://github.com/ctu-mrs/catora


<a id="r06"></a>

### [R06] AMSwarmX: Safe Swarm Coordination in CompleX Environments via Implicit Non-Convex Decomposition of the Obstacle-Free Space

Adajania et al.. **ICRA 2024**。

- 论文/作者资料：https://arxiv.org/abs/2310.09195
- 官方源码：https://github.com/learnsyslab/AMSwarmX


<a id="r07"></a>

### [R07] GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling

Messing et al.. **IJRR 2022; 10.1177/02783649211052066**。

- 论文/作者资料：https://doi.org/10.1177/02783649211052066
- 官方源码：https://github.com/GT-STAR-Lab/GRSTAPS


<a id="r08"></a>

### [R08] D-ITAGS: A Dynamic Interleaved Approach to Resilient Task Allocation, Scheduling, and Motion Planning

Neville, Chernova, Ravichandar. **RA-L 2023；arXiv初版2022**。

- 论文/作者资料：https://arxiv.org/abs/2209.13092
- 官方源码：https://github.com/GT-STAR-Lab/D-ITAGS


<a id="r09"></a>

### [R09] Heterogeneous Multirobot Task Allocation for Long-Endurance Missions in Dynamic Scenarios

Calvo, Capitán. **T-RO 2025, 41:6494–6513; 10.1109/TRO.2025.3626651**。

- 论文/作者资料：https://arxiv.org/abs/2411.02062
- 官方源码：https://github.com/multirobot-use/mrta_heuristic_planner


<a id="r10"></a>

### [R10] Mission Execution Architecture for Multi-UAV Teams

multirobot-use；对应Calvo等研究. **R09配套执行软件；另引用ICUAS 2022, 10.1109/ICUAS54217.2022.9836234**。

- 论文/作者资料：https://doi.org/10.1109/TRO.2025.3626651
- 官方源码：https://github.com/multirobot-use/mrta_execution_architecture


<a id="r11"></a>

### [R11] Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems

Zhongqi Wei, Xusheng Luo, Changliu Liu. **RSS 2025; 10.15607/RSS.2025.XXI.099**。

- 论文/作者资料：https://www.roboticsproceedings.org/rss21/p099.html
- 官方源码：https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS


<a id="r12"></a>

### [R12] Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay in Dynamic Environments

Kondo et al.. **RA-L 2024, 9(2):1476–1483; 10.1109/LRA.2023.3342561；另有ICRA 2023前序版本**。

- 论文/作者资料：https://arxiv.org/abs/2303.06222
- 官方源码：https://github.com/mit-acl/rmader


<a id="r13"></a>

### [R13] DMPC-Swarm: Distributed Model Predictive Control on Nano UAV Swarms

Gräfe, Eickhoff, Zimmerling, Trimpe. **Autonomous Robots 2025, 49:28; 10.1007/s10514-025-10211-w**。

- 论文/作者资料：https://link.springer.com/article/10.1007/s10514-025-10211-w
- 官方源码：https://github.com/Data-Science-in-Mechanical-Engineering/DMPC-Swarm


<a id="r14"></a>

### [R14] CoCoPlan: Adaptive Coordination and Communication for Multi-Robot Systems in Dynamic and Unknown Environments

Zhang et al.. **RA-L 2026, 11(3):3270–3277; 10.1109/LRA.2026.3656769**。

- 论文/作者资料：https://arxiv.org/abs/2601.10116
- 官方完整源码：本轮未确认，不据此推断一定未公开。


<a id="r15"></a>

### [R15] DANCERS: A Physics and Network Co-Simulator for Communicating Multi-Robot Systems

Balaguer et al.. **SIMPAR 2025; 10.1109/SIMPAR62925.2025.10979148**。

- 论文/作者资料：https://doi.org/10.1109/SIMPAR62925.2025.10979148
- 官方源码：https://github.com/Chroma-CITI/DANCERS


<a id="r16"></a>

### [R16] ROS-NetSim: A Framework for the Integration of Robotic and Network Simulators

Calvo-Fullana et al.. **RA-L 2021, 6(2):1120–1127; 10.1109/LRA.2021.3056347**。

- 论文/作者资料：https://arxiv.org/abs/2101.10113
- 官方完整源码：本轮未确认，不据此推断一定未公开。


<a id="r17"></a>

### [R17] Stonefish: Supporting Machine Learning Research in Marine Robotics

Grimaldi et al.. **ICRA 2025；原始Stonefish论文为OCEANS 2019**。

- 论文/作者资料：https://arxiv.org/abs/2502.11887
- 官方源码：https://github.com/patrykcieslak/stonefish


<a id="r18"></a>

### [R18] Fossen Marine Systems Simulator / Python Vehicle Simulator

Thor I. Fossen. **模型与GNC软件；理论参考Fossen 2021第2版专著，不作为新顶会方法**。

- 论文/作者资料：https://www.fossen.biz/pythonVehicleSim/
- 官方源码：https://github.com/cybergalactic/PythonVehicleSimulator

返回：[文献能力矩阵](12_literature_review.md) · [资料索引](README.md)。
