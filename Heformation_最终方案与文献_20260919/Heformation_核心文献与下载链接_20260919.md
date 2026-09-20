# Heformation 核心文献与下载入口（2026-09-19）

## 使用范围

本表是围绕本项目关键设计问题整理的重点文献和官方工具索引，不是全领域穷尽性目录。条目明确区分：开放正文相关章节已核对、摘要/元数据、既有库的历史阅读记录。

**未将本表条目统一宣称为“本轮全部阅读全文”。本交付没有把论文 PDF 下载到用户电脑，也没有将网站阅读自动标为本地下载。** 直接 PDF、作者全文和出版社/机构入口分别注明；出版端可能要求机构登录。网络可读不代表可以重新公开分发 PDF。

P 编号仅用于本次方案，不覆盖仓库已有 R 编号。新增算法必须先回答“是否改变当前需求/约束/实验”，不是照表全部接入。平台原型、综述和 benchmark 不冒充算法顶刊，模型软件不冒充新论文。

## 总表

|编号|方向|成果/正式来源|全文或下载入口|本轮核对范围|
|---|---|---|---|---|
|P01|任务与调度|[Heterogeneous Multi-robot Task Allocation for Long-Endurance Missions in Dynamic Scenarios](https://arxiv.org/abs/2411.02062v3)<br>IEEE T-RO 2025；arXiv v3 2025-11-26|[PDF/正文/下载页](https://arxiv.org/pdf/2411.02062v3)|开放正文，核对问题定义、启发式和执行修复章节|
|P02|任务与运动|[GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling](https://journals.sagepub.com/doi/10.1177/02783649211052066)<br>IJRR 2022；DOI 10.1177/02783649211052066|[PDF/正文/下载页](https://journals.sagepub.com/doi/full/10.1177/02783649211052066)|期刊 HTML 正文，重点核对调度、运动查询及局限；未保存本地 PDF|
|P03|任务修复|[D-ITAGS: A Dynamic Interleaved Approach to Resilient Task Allocation, Scheduling, and Motion Planning](https://arxiv.org/abs/2209.13092)<br>IEEE RA-L 2023；arXiv 初版 2022|[PDF/正文/下载页](https://arxiv.org/pdf/2209.13092)|开放 PDF 正文相关章节|
|P04|执行释放|[APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly](https://arxiv.org/abs/2503.15836v2)<br>RSS 2025|[PDF/正文/下载页](https://arxiv.org/pdf/2503.15836v2)|开放正文，重点异步执行图与释放条件|
|P05|统一任务运动表达|[Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems](https://www.roboticsproceedings.org/rss21/p099.html)<br>RSS 2025；DOI 10.15607/RSS.2025.XXI.099|[PDF/正文/下载页](https://arxiv.org/pdf/2504.18899v2)|官方会议页与 arXiv 正文相关章节|
|P06|异构任务调度|[Heterogeneous Multi-robot Task Allocation and Scheduling via Reinforcement Learning](https://www.marmotlab.org/publications/73-RAL2025-HetMRTA.pdf)<br>IEEE RA-L 2025，10:2654–2661|[PDF/正文/下载页](https://www.marmotlab.org/publications/73-RAL2025-HetMRTA.pdf)|作者公开 PDF，核对任务、联合到齐、等待和实验定义|
|P07|分布式任务规划|[A Decentralised Strategy for Heterogeneous AUV Missions via Goal Distribution and Temporal Planning (DHRTA)](https://ojs.aaai.org/index.php/ICAPS/article/view/6738)<br>ICAPS 2020；DOI 10.1609/icaps.v30i1.6738|[PDF/正文/下载页](https://ojs.aaai.org/index.php/ICAPS/article/view/6738/6592)|本轮仅官方条目与摘要；PDF 直取未成功|
|P08|编队规划|[Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments](https://arxiv.org/abs/2109.07682v2)<br>ICRA 2022|[PDF/正文/下载页](https://arxiv.org/pdf/2109.07682v2)|开放正文相关章节与当前上游代码核对|
|P09|编队规划扩展|[Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments](https://arxiv.org/abs/2210.04048v2)<br>IEEE T-RO 2023，39(6):4785–4804；DOI 10.1109/TRO.2023.3301295|[PDF/正文/下载页](https://arxiv.org/pdf/2210.04048v2)|开放正文，核对编队对齐、分配/重组及轨迹层|
|P10|连续轨迹|[Geometrically Constrained Trajectory Optimization for Multicopters](https://arxiv.org/abs/2103.00190v4)<br>IEEE T-RO 2022；MINCO/GCOPTER 相关|[PDF/正文/下载页](https://arxiv.org/pdf/2103.00190v4)|本轮核对官方元数据与摘要；已有库列有全文笔记|
|P11|编队重排|[CAT-ORA: Collision-Aware Time-Optimal Formation Reshaping for Efficient Robot Coordination in 3-D Environments](https://arxiv.org/abs/2412.00603v2)<br>IEEE T-RO 2025；DOI 10.1109/TRO.2025.3547296|[PDF/正文/下载页](https://arxiv.org/pdf/2412.00603v2)|本轮官方摘要/元数据；未全文精读|
|P12|任务与通信|[CoCoPlan: Adaptive Coordination and Communication for Multi-Robot Systems in Dynamic and Unknown Environments](https://arxiv.org/abs/2601.10116v1)<br>IEEE RA-L 2026，11(3):3270–3277；DOI 10.1109/LRA.2026.3656769|[PDF/正文/下载页](https://arxiv.org/pdf/2601.10116v1)|开放正文，核对任务/通信事件、约束与假设|
|P13|中继与数据调度|[Multi-Robot Data Gathering Under Buffer Constraints and Intermittent Communication](https://scholars.duke.edu/publication/1322925)<br>IEEE T-RO 2018，34(4):1082–1097；DOI 10.1109/TRO.2018.2830370|[PDF/正文/下载页](https://arxiv.org/pdf/1706.02092)|开放正文，核对有限缓存、源/中继、通信和规划算法|
|P14|通信质量暴露|[ACHORD: Communication-Aware Multi-Robot Coordination With Intermittent Connectivity](https://arxiv.org/abs/2206.02245)<br>IEEE RA-L 2022；arXiv:2206.02245|[PDF/正文/下载页](https://arxiv.org/pdf/2206.02245)|开放正文相关通信/系统章节|
|P15|轨迹通信安全|[Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay in Dynamic Environments](https://arxiv.org/abs/2303.06222)<br>IEEE RA-L 2024，9(2):1476–1483；DOI 10.1109/LRA.2023.3342561|[PDF/正文/下载页](https://arxiv.org/pdf/2303.06222)|开放正文，核对提交轨迹与延迟检查假设|
|P16|规划跟踪安全|[FaSTrack: a Modular Framework for Fast and Guaranteed Safe Motion Planning](https://arxiv.org/abs/1703.07373)<br>CDC 2017|[PDF/正文/下载页](https://arxiv.org/pdf/1703.07373)|开放正文，核对相对动力学、误差界与安全控制条件|
|P17|跨介质轨迹|[Trajectory Planning for Hybrid Unmanned Aerial Underwater Vehicles with Smooth Media Transition](https://arxiv.org/abs/2112.13819v1)<br>Journal of Intelligent & Robotic Systems 2021|[PDF/正文/下载页](https://arxiv.org/pdf/2112.13819v1)|开放正文，重点闭环 RRT、介质转换区与仿真|
|P18|跨介质实机|[TJ-FlyingFish: Design and Implementation of an Aerial-Aquatic Quadrotor with Tiltable Propulsion Units](https://arxiv.org/abs/2301.12344v2)<br>ICRA 2023|[PDF/正文/下载页](https://arxiv.org/pdf/2301.12344v2)|开放正文，核对驱动、介质模式和控制设计|
|P19|跨介质实机|[Dipper: A Dynamically Transitioning Aerial-Aquatic Unmanned Vehicle](https://www.roboticsproceedings.org/rss17/p048.html)<br>RSS 2021；DOI 10.15607/RSS.2021.XVII.048|[PDF/正文/下载页](https://www.roboticsproceedings.org/rss17/p048.pdf)|官方摘要与元数据；大 PDF 本轮读取失败|
|P20|跨介质海试|[Nezha-IV: A Hybrid Aerial Underwater Vehicle in Real Ocean Environments](https://onlinelibrary.wiley.com/doi/10.1002/rob.22274)<br>JFR 2024，41(2):420–442，online 2023；DOI 10.1002/rob.22274|未取得开放全文；从左侧官方入口申请/访问|官方摘要/文献表；本轮未取得完整 PDF|
|P21|跨介质控制|[WuKong: Design, Modeling and Control of a Compact Flexible Hybrid Aerial-Aquatic Vehicle](https://researchportal.hkust.edu.hk/en/publications/wukong-design-modeling-and-control-of-a-compact-flexible-hybrid-a/)<br>IEEE RA-L 2025，10(2):1417–1424；DOI 10.1109/LRA.2024.3521659|未取得开放全文；从左侧官方入口申请/访问|官方机构摘要/元数据；未取得全文|
|P22|跨介质统一控制|[Adaptive Dynamic Surface Control for a Hybrid Aerial Underwater Vehicle With Parametric Dynamics and Uncertainties](https://faculty.sjtu.edu.cn/zengzheng/en/lwcg/412166/content/57310.htm)<br>IEEE JOE 2020，45(3):740–758；DOI 10.1109/JOE.2019.2903742|[PDF/正文/下载页](https://doi.org/10.1109/JOE.2019.2903742)|作者机构元数据及公开摘要级线索；本轮未取得全文|
|P23|海况转换策略|[Surfing Algorithm: Agile and Safe Transition Strategy for Hybrid Aerial Underwater Vehicle in Waves](https://doi.org/10.1109/TRO.2023.3319928)<br>IEEE T-RO 2023，39(6):4262–4278；DOI 10.1109/TRO.2023.3319928|未取得开放全文；从左侧官方入口申请/访问|题名/期刊/年份核对及公开摘要线索；全文未取得|
|P24|跨介质实机|[Aerial–aquatic robots capable of crossing the air–water boundary and hitchhiking on surfaces](https://spiral.imperial.ac.uk/entities/publication/97ee3cf3-a8b6-4416-affc-05d983a98a29)<br>Science Robotics 2022，7:eabm6695；DOI 10.1126/scirobotics.abm6695|[PDF/正文/下载页](https://nubot-uaav.github.io/static/pdf/multirotor_based_aavs/scirobotics.abm6695%20(1).pdf)|研究团队资料库 PDF 正文；未将图像读取失败处当图证据|
|P25|跨介质文献导航|[Transition Strategy for Unmanned Aerial-Aquatic Vehicles (UAAVs): A Survey](https://onlinelibrary.wiley.com/doi/10.1002/rob.70138)<br>JFR 2026，43(3):2444–2476；DOI 10.1002/rob.70138|[PDF/正文/下载页](https://nubot-uaav.github.io/)|官方摘要/参考文献与作者分类资料站；正文未取得|
|P26|跨介质前沿补充|[Wukong-Omni: Design, Modeling and Control of a Multi-mode Robot for Air, Land, and Underwater Exploration with All-in-One Propulsion Unit](https://arxiv.org/abs/2603.02602v1)<br>arXiv:2603.02602，2026 预印本|[PDF/正文/下载页](https://arxiv.org/pdf/2603.02602v1)|开放正文浏览，列为前沿补充而非本轮控制选型|
|P27|巡检评价|[Cooperative Aerial Robot Inspection Challenge: A Benchmark for Heterogeneous Multi-UAV Planning and Lessons Learned](https://arxiv.org/abs/2501.06566v2)<br>2025 benchmark 报告，arXiv:2501.06566；比赛/Workshop 不等于期刊发表|[PDF/正文/下载页](https://arxiv.org/pdf/2501.06566v2)|开放正文，核对观测、去重、接收与评分|
|P28|USV 中继实证|[Xiroi II, an Evolved ASV Platform for Marine Multirobot Operations](https://www.mdpi.com/1424-8220/23/1/109)<br>Sensors 2023，23(1):109；DOI 10.3390/s23010109|[PDF/正文/下载页](https://pmc.ncbi.nlm.nih.gov/articles/PMC9824324/)|既有库登记全文；本轮新直取受网站访问限制，未新增本地 PDF|
|P29|海洋异构任务|[Collaboration of Heterogeneous Marine Robots Toward Multidomain Sensing and Situational Awareness on Partially Submerged Targets](https://doi.org/10.1109/JOE.2022.3156631)<br>IEEE JOE 2022；DOI 10.1109/JOE.2022.3156631|未取得开放全文；从左侧官方入口申请/访问|既有库摘要级；本轮未取得全文|
|P30|海洋系统示例|[The SeaClear system: An intelligent multi-robot solution for autonomous cleanup of marine debris on the seabed](https://research.tudelft.nl/en/publications/the-seaclear-system-an-intelligent-multi-robot-solution-for-auton/)<br>Engineering Applications of Artificial Intelligence 2026；DOI 10.1016/j.engappai.2026.114094|[PDF/正文/下载页](https://doi.org/10.1016/j.engappai.2026.114094)|官方期刊/机构记录；提供开放版本入口，未完成正文精读|
|P31|海洋仿真|[Stonefish: Supporting Machine Learning Research in Marine Robotics](https://arxiv.org/abs/2502.11887v2)<br>ICRA 2025；arXiv:2502.11887|[PDF/正文/下载页](https://arxiv.org/pdf/2502.11887v2)|开放正文相关仿真功能与架构章节|
|T01|海洋模型与GNC|[Python Vehicle Simulator / Fossen Marine Craft Model](https://www.fossen.biz/html/marineCraftModel.html)<br>官方模型软件；理论依据 Fossen 2021 专著，不是新顶会论文|[PDF/正文/下载页](https://github.com/cybergalactic/PythonVehicleSimulator)|官方模型说明、README 与 otter 控制/动力学接口代码|
|P32|混合系统理论|[Stability of Switched Systems with Average Dwell-Time](https://web.ece.ucsb.edu/~hespanha/published/)<br>CDC 1999；Hespanha & Morse|[PDF/正文/下载页](https://www.ece.ucsb.edu/~hespanha/published/avedwell.pdf)|作者全文 PDF，核对驻留时间与 Lyapunov 切换条件|

## 如何用于本项目

### P01 · Heterogeneous Multi-robot Task Allocation for Long-Endurance Missions in Dynamic Scenarios

**借鉴：** 保留 Calvo 受限移植为调度基础；时序、成员能力与执行反馈。

**边界：** 原文明确无一般前置关系和开始时间窗口；任务 relay 是接替，不是通信中继；扩展版不冒充完整原始实现。

**源码：** https://github.com/multirobot-use/mrta_heuristic_planner

### P02 · GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling

**借鉴：** 在调度内交换运动可行性与耗时，使用有限查询和缓存。

**边界：** 原实现二维运动，未解决机器人间运动碰撞；引用不赋予本项目连续安全保证。

**源码：** https://github.com/GT-STAR-Lab/GRSTAPS

### P03 · D-ITAGS: A Dynamic Interleaved Approach to Resilient Task Allocation, Scheduling, and Motion Planning

**借鉴：** 能力、时序、互斥和局部修复对照。

**边界：** 不要求替换当前 Calvo；原工程的求解器与许可依赖不能忽略。

**源码：** https://github.com/GT-STAR-Lab/D-ITAGS

### P04 · APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly

**借鉴：** 实际前驱完成后释放动作，允许无依赖动作并行。

**边界：** 装配与多机器人执行图的安全/无死锁性质有其构造假设，不能自动转移给任意任务图。

### P05 · Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems

**借鉴：** 复杂任务与连续可行域的形式化参考。

**边界：** 当前有限场景不需要再串接一套 HLTL/GCS 求解器。

**源码：** https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS

### P06 · Heterogeneous Multi-robot Task Allocation and Scheduling via Reinforcement Learning

**借鉴：** 多能力资源到齐、等待时间与 makespan 对照。

**边界：** 不为新颖性改用 RL；二维移动代价不能直接用于水下/跨介质。

### P07 · A Decentralised Strategy for Heterogeneous AUV Missions via Goal Distribution and Temporal Planning (DHRTA)

**借鉴：** 如有效任务书要求分布式任务分配，作为针对性补充。

**边界：** 不是当前中心化调度已满足分布式验收的证据。

### P08 · Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments

**借鉴：** AAV AIR 单机/编队后端，原生图相似度与邻机轨迹协调。

**边界：** 编队项为优化代价，不等于业务必要性、固定朝向或实际安全证明。

**源码：** https://github.com/ZJU-FAST-Lab/Swarm-Formation

### P09 · Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments

**借鉴：** 理论与当前源码覆盖边界。

**边界：** 论文全部后续机制不能自动称为当前快照已接入；不当作水下控制器。

**源码：** https://github.com/ZJU-FAST-Lab/Swarm-Formation

### P10 · Geometrically Constrained Trajectory Optimization for Multicopters

**借鉴：** 理解现有轨迹参数化，不新增串行规划层。

**边界：** 不是任务调度器，也不证明 qn 跟踪误差界。

**源码：** https://github.com/ZJU-FAST-Lab/GCOPTER

### P11 · CAT-ORA: Collision-Aware Time-Optimal Formation Reshaping for Efficient Robot Coordination in 3-D Environments

**借鉴：** 后续确有重排瓶颈时的对照。

**边界：** 不等同通用多障碍导航；当前不接第二编队后端。

**源码：** https://github.com/ctu-mrs/catora

### P12 · CoCoPlan: Adaptive Coordination and Communication for Multi-Robot Systems in Dynamic and Unknown Environments

**借鉴：** 任务与通信机会共同安排，非全时连接。

**边界：** 恢复连通不等于任意数据已传完；原运动模型不是跨介质动力学；不宣称该思想首创。

### P13 · Multi-Robot Data Gathering Under Buffer Constraints and Intermittent Communication

**借鉴：** UUV/USV 数据产生—缓存—交付与会合调度的先例。

**边界：** 已有方法不能包装成新创新；原二维/简化碰撞及上传假设须区分。

### P14 · ACHORD: Communication-Aware Multi-Robot Coordination With Intermittent Connectivity

**借鉴：** 通信质量影响上层决策，消息优先级与网络监测参考。

**边界：** 不应为五平台场景复制完整网络中间件。

### P15 · Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay in Dynamic Environments

**借鉴：** 已发布/已提交/实际采用轨迹的区别。

**边界：** 已知延迟上界及消息假设，不覆盖任意长期丢包；不直接替换 Swarm。

**源码：** https://github.com/mit-acl/rmader

### P16 · FaSTrack: a Modular Framework for Fast and Guaranteed Safe Motion Planning

**借鉴：** 以经过验证的跟踪误差界反推规划安全余量。

**边界：** 实测最大误差不是 HJ 不变界；现有 qn PD/RBF 不能自动继承保证。

### P17 · Trajectory Planning for Hybrid Unmanned Aerial Underwater Vehicles with Smooth Media Transition

**借鉴：** 分域规划加转换区约束，先验证动作可执行再计算任务代价。

**边界：** HyDrone 的转换区参数、控制和模型不能原封不动移植 qn。

### P18 · TJ-FlyingFish: Design and Implementation of an Aerial-Aquatic Quadrotor with Tiltable Propulsion Units

**借鉴：** 空气与水下控制分配、转换不是改 z 目标。

**边界：** 机械构型/倾转推进与 qn 未必匹配，不能照抄控制分配。

### P19 · Dipper: A Dynamically Transitioning Aerial-Aquatic Unmanned Vehicle

**借鉴：** 跨介质动作必须单独验证；固定翼案例参考。

**边界：** 未声称全文阅读；固定翼折翼不是 qn 四旋翼后端。

### P20 · Nezha-IV: A Hybrid Aerial Underwater Vehicle in Real Ocean Environments

**借鉴：** 同一平台空气、水下、过渡三个工作域的实测边界。

**边界：** 参考海试设计而非继承具体海况和深度能力。

### P21 · WuKong: Design, Modeling and Control of a Compact Flexible Hybrid Aerial-Aquatic Vehicle

**借鉴：** 跨域混合控制与水下姿态控制对照。

**边界：** 不是当前 qn 已有能力，也不是 Wukong-Omni 同一论文。

### P22 · Adaptive Dynamic Surface Control for a Hybrid Aerial Underwater Vehicle With Parametric Dynamics and Uncertainties

**借鉴：** 统一跨介质控制研究查新不可遗漏的近邻工作。

**边界：** 不能在未阅读全文时引用具体定理；不声称其控制器适用于 qn。

### P23 · Surfing Algorithm: Agile and Safe Transition Strategy for Hybrid Aerial Underwater Vehicle in Waves

**借鉴：** 波浪中的转换时机是可执行性问题而非零代价动作。

**边界：** 本轮不据此给出具体算法或保证；需要作者/机构全文补读。

### P24 · Aerial–aquatic robots capable of crossing the air–water boundary and hitchhiking on surfaces

**借鉴：** 跨域运动及附着方式的实机先例。

**边界：** 特殊机构能力不赋给当前装备；下载副本的使用许可按来源遵守。

### P25 · Transition Strategy for Unmanned Aerial-Aquatic Vehicles (UAAVs): A Survey

**借鉴：** 继续追踪跨介质原型、推进、入水出水策略的权威入口。

**边界：** 作者站部分 BibTeX 年份与期刊上线时间不同；以期刊为准；综述不替代原论文。

### P26 · Wukong-Omni: Design, Modeling and Control of a Multi-mode Robot for Air, Land, and Underwater Exploration with All-in-One Propulsion Unit

**借鉴：** 多模态平台设计与控制的后续查新。

**边界：** 未核实正式发表；不当作已接入主链或工程必需。

### P27 · Cooperative Aerial Robot Inspection Challenge: A Benchmark for Heterogeneous Multi-UAV Planning and Lessons Learned

**借鉴：** 保留观测与接收分离；用现有几何代理做任务层验证。

**边界：** 不重新加入被撤销的任意曝光/模糊/像素分辨率参数。

### P28 · Xiroi II, an Evolved ASV Platform for Marine Multirobot Operations

**借鉴：** USV/ASV 在 AUV 与地面端之间转接并移动支援。

**边界：** 不是顶刊；硬件/有效带宽必须由本项目配置或标定，不直接套用。

### P29 · Collaboration of Heterogeneous Marine Robots Toward Multidomain Sensing and Situational Awareness on Partially Submerged Targets

**借鉴：** 跨域互补观测与水面支援的任务组织。

**边界：** 不得将摘要级推断写成作者完整实现细节。

### P30 · The SeaClear system: An intelligent multi-robot solution for autonomous cleanup of marine debris on the seabed

**借鉴：** 复杂海洋协作任务分工与系统集成参考。

**边界：** 不能把缆控、供电和操纵机构假设改名后当作自主水声系统。

### P31 · Stonefish: Supporting Machine Learning Research in Marine Robotics

**借鉴：** 需要海洋传感器/更高保真环境时的后端候选。

**边界：** 当前先采用轻量数值后端；不能和 qn 同时推进同一个 AAV。

**源码：** https://github.com/patrykcieslak/stonefish

### T01 · Python Vehicle Simulator / Fossen Marine Craft Model

**借鉴：** 优先复用 USV/UUV 数值动力学和原生 GNC，薄 ROS 接口接入。

**边界：** Otter/REMUS 是参考艇型不是用户装备；REMUS 型不能假定原地悬停；坐标与模式要转换。

**源码：** https://github.com/cybergalactic/PythonVehicleSimulator

### P32 · Stability of Switched Systems with Average Dwell-Time

**借鉴：** 统一分析不同模式控制律，而不是强行一个控制器控制所有平台。

**边界：** 各模式稳定不推出任意切换稳定；过渡模式不满足条件时不得套用定理。

## 先读哪一组

1. 调度与任务语义：P01、P02、P04、P12、P13。最重要的是确认已有能力，防止把一般“任务—通信”组合称为新贡献。
2. 跨介质落地：P17、P18；P20–P23 作为下一轮必须取得全文的硬件/控制近邻，P25 为原论文导航。
3. 编队与安全：P08、P09、P16、P32。保留当前图机制，不恢复无需求的固定走廊与朝向约束。
4. 平台与实验：T01、P31、P27。先完成数值执行闭环，不把海洋美术和真实相机算法作为任务系统前置。

Surfing Algorithm、JOE ADSC、Nezha-IV、WuKong 等尚未拿到全文，不能据其摘要编写数学证明或照搬控制参数。Dipper 的官方 PDF 入口已经列出，但本轮大文件读取失败，没有假装读完。

## 资料保存建议

继续复用 `research/literature/{manifest.yaml,papers/,notes/}`。文件标记以实际 PDF 与实际阅读为准。默认不推送受版权保护的论文；本表本身可以作为链接索引。每篇笔记只记录：问题与假设、可复用机制、不能继承的保证、对应本项目接口/反例。
