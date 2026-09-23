# Heformation：任务规划基座与相关开源系统调研

核查日期：2026-09-22。仓库代码快照：CquL/Heformation，main@8274cd73039cdc37e9f0980f785936894d3de508。

## 1. 结论与使用边界

科研算法基座建议选 ITAGS / D-ITAGS 的能力驱动、分配—调度—运动交错搜索体系。现有 Heformation 继续作为工程载体，Calvo 受限移植保留为基线和回归，不立即删除、不整体重构控制执行链。不能将现有完整候选枚举直接改名为 D-ITAGS。运行原版 D-ITAGS 仍有 Gurobi 和原始依赖要求，替换求解器后的实现须标明为派生版本。

任务图、数据依赖、模式与编队约束不应因“统一能力建模”而删除。需要收敛的是重复的状态、重复的计划和相互覆盖的决策环，而不是物理上不同的约束。静态能力、状态相关动作可执行性、多人协作角色、资源容量和数据因果性应在同一个计划模型中表达。

本台账是针对本项目问题的定向调研，不是穷尽全部文献的系统综述。“正文核读”指核读公开正文相关问题定义、方法、假设与实验，不等于重新证明论文或运行作者实验。“源码核查”只代表核对公开仓库及其声明，不代表已成功安装。未确认公开完整源码，不等于断言不存在。本文没有修改用户仓库，也没有在用户机器运行仿真。

## 2. 当前代码核查

- `integration/mrta_python/models.py`：Agent.capabilities 与 Task.required_capabilities 已存在，使用 FrozenSet[str]；另有任务前置、指定成员与队伍大小规则。
- `integration/mrta_python/executors.py:922-938`：eligible_executors 使用能力集合包含关系与物理成员数匹配。这是能力标签驱动，不是 ITAGS 的通用定量 trait 矩阵和自动联盟合成。
- 同文件约950-1108行：无 execution_candidates 时使用 Calvo v9 派生排序；有 execution_candidates 时进入完整候选搜索。
- 同文件约1110-1280行：候选会更新成员位置、模式、可用时刻及原生模型终态；支持协作活动、锁定、前置和完整计划检查。
- `ExecutionCandidate` 已有 FEASIBLE / INFEASIBLE / UNKNOWN、终态、运动证据和数据产品。
- `context/02_current_status.md` 最新记录：水下作业—USV支援—母船接收已有阶段运行记录；完整AAV AIR/待命查询接线、模式选择、返回复查及受限远端状态消息仍未全部完成。
- 部分 README、文件头仍保留早期七机/离线说明，应按最新实现与最新状态记录辨别，不能直接当作当前系统的完整描述。

代码快照入口：
https://github.com/CquL/Heformation/tree/8274cd73039cdc37e9f0980f785936894d3de508

## 3. 核心论文与源码台账

### R01 Calvo：长航时异构任务分配
- 标题：Heterogeneous Multirobot Task Allocation for Long-Endurance Missions in Dynamic Scenarios
- 作者：Álvaro Calvo, Jesús Capitán
- 发表：IEEE Transactions on Robotics，2025，41:6494–6513。
- DOI：https://doi.org/10.1109/TRO.2025.3626651
- 公开正文：https://arxiv.org/html/2411.02062v3
- 规划源码：https://github.com/multirobot-use/mrta_heuristic_planner
- 执行源码：https://github.com/multirobot-use/mrta_execution_architecture
- 本轮核查：正文相关章节与官方源码说明。
- 可借鉴：能力兼容、同步联盟、充电、任务分片与接替、执行延迟修复。
- 边界：原问题明确不含一般任务前置关系与任务开始窗口；relay是作业接替，不是通信中继。不能把原版全部能力当成当前受限Python移植的能力。

### R02 Calvo：原始优化模型
- 标题：Optimal Task Allocation for Heterogeneous Multi-robot Teams with Battery Constraints
- 发表：ICRA 2024，7243–7249。
- DOI：https://doi.org/10.1109/ICRA57147.2024.10611147
- 源码：同R01规划仓库。
- 本轮核查：官方仓库的论文与书目信息。
- 用途：原始电池/联盟优化模型的出处，不作为新增在线求解器。

### R03 ITAGS
- 标题：An Interleaved Approach to Trait-Based Task Allocation and Scheduling
- 发表：IROS 2021。
- 论文：https://arxiv.org/abs/2108.02773
- 作者论文页：https://star-lab.cc.gatech.edu/papers/neville-itags/
- 源码关系：GRSTAPS / D-ITAGS仓库包含该方法谱系；不将不同版本自动视为同一原始复现。
- 本轮核查：作者论文页，结合D-ITAGS正文核对方法关系。
- 用途：机器人能力需求、联盟分配与调度的交错搜索基础。

### R04 D-ITAGS
- 标题：D-ITAGS: A Dynamic Interleaved Approach to Resilient Task Allocation, Scheduling, and Motion Planning
- 作者：Glen Neville, Sonia Chernova, Harish Ravichandar
- 发表：IEEE Robotics and Automation Letters，2023，8(2):1037–1044；作者页同时标注IROS 2023。
- DOI：https://doi.org/10.1109/LRA.2023.3234824
- 全文：https://arxiv.org/pdf/2209.13092
- 作者论文页：https://star-lab.cc.gatech.edu/papers/neville-ditags/
- 源码：https://github.com/gneville6/D-ITAGS
- 本轮核查：公开全文、架构图、求解与修复机制及官方仓库。
- 可借鉴：trait需求驱动联盟搜索，分配—调度—运动反馈，动态扰动后的定向修复。
- 边界：通信丢失作为扰动，不等于有限字节量、缓存、两跳交付的联合规划；没有原生AAV跨介质与编队控制器。研究库依赖OMPL/Gurobi，部分使用说明仍为TODO。

### R05 GRSTAPS
- 标题：GRSTAPS: Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling
- 发表：International Journal of Robotics Research，2022，41(2):232–256；在线发表于2021。
- DOI：https://doi.org/10.1177/02783649211052066
- 作者论文页：https://star-lab.cc.gatech.edu/papers/messing-grstaps/
- 源码：https://github.com/amessing/grstaps
- 本轮核查：出版社正文相关部分、作者页与源码出处。
- 可借鉴：WHAT/WHO/WHEN/HOW交错决策，比只接受固定任务集合更包含任务规划。
- 边界：论文实验的二维路线与运动模型不能提供本项目6DOF跟踪或跨平台连续安全保证。

### R06 状态/环境相关能力
- 标题：A Resilient and Energy-Aware Task Allocation Framework for Heterogeneous Multirobot Systems
- 发表：IEEE Transactions on Robotics，2022，38(1):159–179；在线发表于2021。
- DOI：https://doi.org/10.1109/TRO.2021.3102379
- 全文：https://arxiv.org/pdf/2105.05586
- 源码：https://github.com/gnotomista/multi_robot_task_allocation
- 本轮核查：公开正文的feature-capability-task映射、问题类型及官方源码。
- 可借鉴：硬件特征与可用能力区分，故障和环境改变能力；任务分配与控制联系。
- 边界：核心属于瞬时任务分配，不是我们需要的完整长时序资源—通信调度。不能为复用它而替换既有qn/PVS控制器。

### R07 动态trait的已有研究
- 标题：Approximated Dynamic Trait Models for Heterogeneous Multi-Robot Teams
- 发表：IROS 2020。
- 作者论文页：https://star-lab.cc.gatech.edu/papers/neville-dynamic-traits/
- 本轮核查：作者论文页/摘要；未确认独立官方完整源码。
- 可借鉴：动态trait的统计近似。
- 边界：“能力动态”本身早已有研究，不能作为首次贡献；该方法不等于状态转移动作的长期规划。

### R08 TRAITS
- 标题：Modeling and Optimizing the Provisioning of Exhaustible Capabilities for Simultaneous Task Allocation and Scheduling
- 作者：Jinwoo Park, Harish Ravichandar, Seth Hutchinson
- 发表：AAMAS 2026，作者页标注Oral。
- 全文：https://arxiv.org/html/2602.13866v1
- 作者页：https://star-lab.cc.gatech.edu/papers/Park-TRAITS/
- 源码：https://github.com/jinwoop/TRAITS
- 本轮核查：公开正文及官方源码说明。
- 可借鉴：可消耗/不可消耗能力、提供速率、任务持续时间与资源消耗联合建模。
- 边界：论文是离线框架，不是原生在线有限通信方案；不能因为发表更新就默认最适合当前主链。

### R09 层次时序逻辑任务分配
- 标题：Simultaneous Task Allocation and Planning for Multi-Robots under Hierarchical Temporal Logic Specifications
- 作者：Xusheng Luo, Changliu Liu
- 发表：IEEE Transactions on Robotics，2025，41:5040–5059。
- DOI：https://doi.org/10.1109/TRO.2025.3598139
- 论文：https://arxiv.org/abs/2401.04003
- 源码：https://github.com/XushengLuo92/Hierarchical-LTL-STAP
- 本轮核查：作者/实验室记录、论文信息和仓库动作模型/使用说明。
- 可借鉴：任务逻辑、动作前置与任务分配一起求解；适合未来复杂任务分支。
- 边界：不是水声容量与交付调度系统；不为少量固定监测模板强制引入复杂逻辑编译链。

### R10 层次任务分解
- 标题：Decomposition-Based Hierarchical Task Allocation and Planning for Multi-Robots Under Hierarchical Temporal Logic Specifications
- 发表：IEEE Robotics and Automation Letters，2024，9(8):7182–7189。
- DOI：https://doi.org/10.1109/LRA.2024.3412589
- 源码：https://github.com/XushengLuo92/Hierarchical-LTL
- 本轮核查：作者/实验室与官方源码信息。
- 用途：任务分解—分配—执行的另一条成熟建模路径，不是当前必须接入的第二求解器。

### R11 H-LTL + GCS
- 标题：Hierarchical Temporal Logic Task and Motion Planning for Multi-Robot Systems
- 作者：Zhongqi Wei, Xusheng Luo, Changliu Liu
- 发表：Robotics: Science and Systems，2025。
- 官方会议页：https://www.roboticsproceedings.org/rss21/p099.html
- 论文：https://arxiv.org/abs/2504.18899
- 源码：https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS
- 本轮核查：官方会议摘要、作者页和源码依赖/案例；不标为全文复现。
- 可借鉴：任务分配嵌入逻辑图、连续运动可行域、协作交接。
- 边界：采用GCS/Drake/MONA/MOSEK体系，若完整替换将涉及现有运动规划表示；并非即插即用的Swarm上层任务调度器。

### R12 前置任务协作
- 标题：Multi-Robot Coordination and Cooperation with Task Precedence Relationships
- 发表：ICRA 2023，5800–5806。
- DOI：https://doi.org/10.1109/ICRA48891.2023.10160998
- 论文：https://arxiv.org/abs/2209.14417
- 本轮核查：论文摘要及出版社/后续正文引文信息；未确认完整官方源码。
- 可借鉴：任务前置、联盟投入与任务收益间的耦合。

### R13 CoCoPlan
- 标题：CoCoPlan: Adaptive Coordination and Communication for Multi-robot Systems in Dynamic and Unknown Environments
- 发表：IEEE Robotics and Automation Letters，2026，11(3):3270–3277。
- DOI：https://doi.org/10.1109/LRA.2026.3656769
- 全文：https://arxiv.org/html/2601.10116v1
- 本轮核查：公开正文、论文元数据；未确认完整官方求解器GitHub。
- 可借鉴：任务、执行者、通信事件位置与时间共同规划；前置、互斥与并发。
- 边界：team-wise通信事件与我们定向UUV→USV→母船的数据交付不同；网络重新连通不自动等于有限数据全部送达。

### R14 有限缓存与间歇交付
- 标题：Multirobot Data Gathering Under Buffer Constraints and Intermittent Communication
- 作者：Meng Guo, Michael M. Zavlanos
- 发表：IEEE Transactions on Robotics，2018，34(4):1082–1097。
- DOI：https://doi.org/10.1109/TRO.2018.2830370
- 全文：https://arxiv.org/pdf/1706.02092
- 源码：https://github.com/MengGuo/interm_data_gather
- 本轮核查：公开正文和官方仓库。
- 可借鉴：源机器人、转运/中继机器人、缓存、约定会合与断联期间执行。
- 边界：公开仓库README说明主要是集中式对照实现，不等于论文完整分布式算法；软件年代较早，依赖Python2等历史环境。

### R15 ACHORD
- 标题：ACHORD: Communication-Aware Multi-Robot Coordination With Intermittent Connectivity
- 发表：IEEE Robotics and Automation Letters，2022，7(4):10184–10191。
- DOI：https://doi.org/10.1109/LRA.2022.3193240
- 论文：https://arxiv.org/abs/2206.02245
- 作者机构记录：https://authors.library.caltech.edu/records/14zzf-2vp04
- 本轮核查：论文/机构记录；未确认整个系统完整开源。
- 可借鉴：通信性能暴露给任务决策，间歇网络的信息优先级与协调。
- 边界：不是现成的全部跨域任务、数据运输和编队统一求解器。

### R16 NASA/JPL MOSAIC
- 标题：Mars On-Site Shared Analytics Information and Computing
- 发表：ICAPS 2019，29(1):707–715。
- 官方文档：https://nasa.github.io/MOSAIC/
- 源码：https://github.com/nasa/MOSAIC
- 默认分支最近提交核查：https://github.com/nasa/MOSAIC/commit/af396ec450bd9f6f95fc5c603e13964035e05cd6
- 本轮核查：官方说明及GitHub提交元数据。所见默认分支最新提交日期为2021-10-28；文档标注2.0.0与2020版权。
- 可借鉴：异构计算任务DAG、时变通信链路、资源/能量/时延调度。
- 边界：主要是计算任务共享，不是物理编队作业框架；建模仍有参考价值，软件不宜未经验证就作为新主运行时。

### R17 MMRS：海洋通信受限协调
- 标题：Coordination of marine multi robot systems with communication constraints
- 作者：Antoni Martorell-Torres, José Guerrero-Sastre, Gabriel Oliver-Codina
- 发表：Applied Ocean Research，2024，142:103848。
- DOI：https://doi.org/10.1016/j.apor.2023.103848
- 机构全文：https://repositori.uib.cat/items/467ac689-de89-40cb-afae-6fe74a796579
- 源码：https://github.com/martorelltorres/MMRS_stack
- 本轮核查：出版社开放正文的协调机制/假设/实验与官方源码说明。
- 可借鉴：AUV区域划分与覆盖，ASV跟踪收集数据，依据实测声学通信特性确定支援距离，COLA2/ROS执行。
- 边界：论文基础方案存在预分配和周期访问假设；不能把ASV局部排斥距离当成所有平台全程碰撞避免。该刊不是T-RO/RA-L，保留是因为场景非常相关。

### R18 MMRS：2026年声学通信感知算法
- 标题：Acoustic communication-aware coordination algorithm for multi-robot systems in underwater exploration
- 发表：International Journal of Systems Science，2026，在线发表。
- DOI：https://doi.org/10.1080/00207721.2026.2647373
- 出版社：https://www.tandfonline.com/doi/abs/10.1080/00207721.2026.2647373
- 源码：https://github.com/martorelltorres/MMRS_stack
- 本轮核查：出版社可访问摘要与Code and data availability statement、仓库；全文页面直接打开受限，不声称全文核读。
- 可借鉴：OWA/ARTM聚合通信质量、空间分布、任务紧急性，ASV动态选择服务对象。
- 边界：未见同一系统原生包含AAV入出水选择、Swarm编队执行与三类平台完整联合求解。不是机器人顶会论文，但非常值得作为海洋协作实验基线。

### R19 ROSMC
- 标题：ROSMC: A High-Level Mission Operation Framework for Heterogeneous Robotic Teams
- 发表：ICRA 2023。
- DOI：https://doi.org/10.1109/ICRA48891.2023.10161133
- 机构记录与全文：https://elib.dlr.de/194917/
- 源码：https://github.com/DLR-RM/rosmc
- 本轮核查：公开正文的系统/操作边界及官方源码。
- 可借鉴：任务编辑、同步、监控、远程异构机器人自主技能执行与操作界面。
- 边界：是mission operation框架，不能直接等同自动最优任务分配求解器；历史ROS/Python版本需要单独核验。

### R20 LSTS整套海洋工具链
- 代表论文：Advancing multi-vehicle deployments in oceanographic field experiments
- 发表：Autonomous Robots，2019，43(6):1555–1574；在线发表于2018。
- DOI：https://doi.org/10.1007/s10514-018-9810-x
- 作者机构记录：https://pearl.plymouth.ac.uk/bms-research/2055/
- 官方架构：https://www.lsts.pt/about
- 源码：https://github.com/LSTS/neptus 、 https://github.com/LSTS/imc 、 https://github.com/LSTS/dune
- 本轮核查：机构论文记录、官方工具链介绍及开源出处。
- 可借鉴：Neptus任务指挥、IMC通信协议、DUNE平台执行/导航控制；跨水下、水面和空中系统的协调集成。
- 边界：成熟工具链不等于原生提供本项目全部约束的全局联合优化。整套迁入现有ROS/qn系统会带来新的接口迁移。

### R21 异构多编队执行
- 标题：A Decoupled Solution to Heterogeneous Multi-Formation Planning and Coordination for Object Transportation
- 作者：Weijian Zhang, Charlie Street, Masoumeh Mansouri
- 发表：Robotics and Autonomous Systems，2024，180:104773。
- DOI：https://doi.org/10.1016/j.robot.2024.104773
- 源码：https://github.com/HyPAIR/Heterogeneous-formation-controller
- 本轮核查：出版社/作者机构摘要及源码说明。
- 可借鉴：编队生成、多编队规划与协调、异构运动条件和编队之间碰撞处理。
- 边界：物体运输场景，不是海洋通信数据交付任务。该刊为RAS，不标成T-RO。

### R22 CARIC
- 标题：Cooperative Aerial Robot Inspection Challenge: A Benchmark for Heterogeneous Multi-Uncrewed-Aerial-Vehicle Planning and Lessons Learned
- 发表：IEEE Robotics & Automation Magazine，2025。
- DOI：https://doi.org/10.1109/MRA.2025.3584341
- 论文：https://arxiv.org/abs/2501.06566
- 官方项目：https://ntu-aris.github.io/caric/
- 基线源码：https://github.com/ntu-aris/caric_baseline
- 本轮核查：官方项目、论文信息和开源基线出处。
- 可借鉴：异构传感器/飞机协作，区域检查、轨迹执行及任务质量评价。
- 边界：这是RA-M，不是RA-L；竞赛平台与某一参赛队求解器不能混称为同一个端到端算法。没有原生海上跨介质链。

### R23 新MOSAIC（与NASA不是同一个）
- 标题（v3）：MOSAIC: Modular Supervised Autonomy for Intelligent Coordination of Heterogeneous Robotic Teams
- 发表：IEEE Transactions on Field Robotics，2026；arXiv页面注明已接收并给出期刊DOI。
- DOI：https://doi.org/10.1109/TFR.2026.3710657
- 最新正文：https://arxiv.org/html/2601.23038v3
- 版本记录：https://arxiv.org/abs/2601.23038
- 本轮核查：公开正文与发表状态；未确认完整官方任务协调器源码。
- 可借鉴：兴趣点任务、机器人技能、预测状态、多深度候选规划、不同自主级别与完整异构野外系统。
- 边界：不是NASA/JPL计算调度器；轮式/足式科研探测场景，不是现成空—水—潜编队。v3题名与早期Scalable题名有变化，应引用确定版本。

### R24 海洋真实任务输出
- 标题：Ocean front detection and tracking using a team of heterogeneous marine vehicles
- 发表：Journal of Field Robotics，2021，38(6):854–881。
- DOI：https://doi.org/10.1002/rob.22014
- 出版社：https://onlinelibrary.wiley.com/doi/10.1002/rob.22014
- 本轮核查：出版社论文信息、摘要与可访问内容；未确认完整官方源码。
- 可借鉴：海洋科学目标驱动的异构任务分配、环境预测与现场执行，而不是只展示队形动画。
- 边界：其环境估计和任务结构与本项目定点巡查不同，不直接取代现有执行器。

### R25 Q-ITAGS
- 标题：Q-ITAGS: Quality-Optimized Spatio-Temporal Heterogeneous Task Allocation with a Time Budget
- 发表：ISRR 2024，作者实验室记录。
- 论文：https://arxiv.org/abs/2404.07902
- 本轮核查：论文摘要及作者发表记录；未确认独立完整源码。
- 可借鉴：任务质量与团队分配的时空权衡。
- 边界：题名time budget是任务/使命时限，不能误读为求解器运行时间预算；当前未校准观测质量模型不宜直接加入质量优化。

### R26 GMP
- 标题：GMP: A Genetic Mission Planner for Heterogeneous Multirobot System Applications
- 作者：Branko Miloradović, Baran Çürüklü, Mikael Ekström, Alessandro Vittorio Papadopoulos
- 发表：IEEE Transactions on Cybernetics，2022，52(10):10627–10638；在线发表于2021。
- DOI：https://doi.org/10.1109/TCYB.2021.3070913
- 作者研究页：https://www.idt.mdu.se/~aps01/research/RoboticsAndAutomation/
- 本轮核查：出版社/作者书目与摘要；未确认完整官方GitHub。
- 可借鉴：异构使命的统一任务抽象、MILP基准与遗传求解。
- 边界：不是现成海上有限通信与跨介质编队运行栈，不因名称含mission planner就当成全系统。

## 4. 建议的最小研究增量

1. 保留静态硬件/技能声明，将可执行性明确绑定到当前/预测状态、角色和动作前置条件；不把所有约束都塞进一个可加的trait向量。
2. 延续现有ExecutionCandidate，将作业、模式转换、通信支援与编队作为同一候选中的动作/角色安排，而不是几个独立求解器先后覆盖方案。
3. 借鉴D-ITAGS交错搜索与定向修复，让候选检查失败能够改变执行者、次序、模式和支援安排。有效终态及下游成本必须参与搜索。
4. 任务层只能使用实际收到的信息。有限业务数据传输已经有阶段实现，不等于远端状态与计划确认也受到同样通信约束。
5. 继续使用既有Swarm/qn/PVS执行链、实际Result和物理资源占用。接口只补真实缺项。
6. 对照应包含Calvo派生基线、D-ITAGS原版/注明的适配版本、海洋MMRS策略及小规模完整候选精确参考。所有方法按同一实际交付定义评价。

## 5. 理论与实验边界

不能把“动态能力”“加入通信”“任务图+信息图”本身写成新颖性。应研究计划内模式变化如何改变后续能力、通信支援及数据到达如何改变后续任务可释放性，以及受限信息下怎样高效修复仍可执行的计划。

主要实验量应为：必做任务实际交付率、最后所需数据到达时刻、规划与修复时间、模式转换的实际成功/拒绝、资源等待、实际运动安全和预测偏差。未经校准的能耗/图像质量不可写成真实优化收益。枚举候选内最优不等于任意连续轨迹空间全局最优；名义安全也不等于闭环连续安全。
