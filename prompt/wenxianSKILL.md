# 异构无人集群科研与工程参考 Skill

## Name
heterogeneous-unmanned-cluster-research-map

## Purpose

用于“异构无人集群项目”中的科研调研、架构设计、工程实现与实验复核。

目标系统：
- 3 AAV
- 1 USV
- 1 UUV

本 Skill 的核心作用不是继续无边界搜集论文，而是：
1. 记录已经调研和参考过的顶刊顶会、开源框架、Benchmark 与工程工具；
2. 区分“已采用 / 已复现 / 已全文阅读 / 已重点参考 / 候选 / Benchmark / 历史参考”；
3. 在新增任务分配、通信、编队、路径规划、USV/UUV 接入、任务完成判定等功能前，优先复核已有研究，避免重复调研；
4. 防止把“论文里有”误写成“项目已经实现”。

---

## When To Use

当用户提出以下请求时优先使用本 Skill：

- 设计或修改异构无人集群总体架构；
- 设计任务分解、任务分配、资源调度、角色选择；
- 设计 AAV 单机 / 编队执行；
- 设计任务与运动联合规划；
- 设计通信受限、中继、会合、缓存、信息依赖；
- 设计 USV/UUV 在海上监测中的角色；
- 设计安全、避障、轨迹规划与跟踪误差实验；
- 设计海上监测任务、巡检完成判据、Benchmark；
- 选择仿真平台或后端；
- 判断一个研究点是否已有类似顶刊顶会工作；
- 需要下载、复核或补充核心论文全文时；
- 需要说明“当前真正进入主链的模块”和“仅作为研究参考的模块”时。

不用于：
- 单个代码 bug 的局部修复；
- 与本项目无关的通用机器人问题；
- 无需文献依据的简单工程操作。

---

# 1. 项目目标架构

```text
高层任务请求
↓
任务展开 / Task–Information Graph
↓
能力匹配 / 资源调度 / 角色选择
↓
任务—通信—运动协调
↓
平台运动规划
├─ AAV：Swarm-Formation
├─ USV：后续水面后端
└─ UUV：后续水下后端
↓
控制 / 动力学
↓
实际状态与任务结果
↓
任务修复 / 重规划
```

当前母船承担中心任务规划与资源协调。

当前在线执行主链：

```text
高层任务请求（本轮开始做）
↓
任务展开
↓
Calvo restricted executor scheduling
↓
Executor / resource ownership
↓
FormationAction
↓
Swarm-Formation
↓
PositionCommand
↓
qn controller + dynamics
↓
actual Odometry
↓
完成 / 安全 / 观测结果
↓
plan repair / conditional revisit
```

---

# 2. 状态标签

所有论文、框架和工具必须明确标注以下状态之一：

- **已采用**：已经进入当前工程主链；
- **已复现**：已在项目中复现或运行；
- **已全文阅读**：已获得并阅读全文；
- **已重点参考**：直接影响接口、约束或实验设计；
- **候选**：未来可能采用，但当前未进入主链；
- **Benchmark / 工具**：用于验证、仿真、评分或工程支撑；
- **历史参考**：曾调研，但当前不是主要路线。

禁止将“论文建议 / 框架能力”描述为“本项目已经实现”。

---

# 3. 已调研成果总表

## 3.1 任务分配、资源调度、Task-Motion Planning

| 成果 | 来源 | 主要参考点 | 当前定位 |
|---|---|---|---|
| Calvo / Capitán heterogeneous MRTA | IEEE T-RO | 异构能力匹配、任务分配、时序、执行反馈、plan repair | 当前任务层核心依据，受限 Python port 已进入主链 |
| mrta_heuristic_planner | 开源代码 | Calvo 调度逻辑工程实现 | 已参考 / 部分移植 |
| mrta_execution_architecture | 开源代码 | 计划执行与反馈架构 | 已重点参考 |
| GRSTAPS | IJRR 2022 | 任务分配、调度与运动可行性联合 | 已重点参考 |
| D-ITAGS | RA-L / IROS 方向 | 异构联盟形成、调度、运动代价 | 已深入调研，因 Gurobi 依赖暂不接主链 |
| ITAGS | 前序工作 | Iterative Task Allocation + Graph Search | 参考 |
| Q-ITAGS | 后续工作 | ITAGS 扩展 | 参考 |
| TRAITS | heterogeneous MRTA | 能力 / 联盟型任务调度 | 参考 |
| APEX-MR | RSS 2025 | 异步执行、计划释放、实际完成反馈 | 当前任务执行语义重点参考 |
| H-LTL / GCS | RSS 2025 | 层次时序任务与连续运动规划 | 后续复杂任务依赖候选 |
| CBBA | 经典 MRTA | 分布式拍卖 | baseline / 参考 |
| Auction | 经典 | 拍卖式任务分配 | baseline |
| Hungarian | 经典 | 一对一分配 | baseline |
| Greedy | 经典 | 简单启发式调度 | baseline |
| OR-Tools | Google | 调度与组合优化 | 已调研 / 工程工具 |
| STRATA | MRTA | 多机器人调度 | 参考 |
| NASA MOSAIC | NASA | 分布式资源 / 任务规划 | 参考 |
| OmniPlan | 任务规划 | 多智能体任务组织 | 参考 |
| MARL-MRTA / GATAR / LRGO 等 | 学术工作 | 学习型与优化型 MRTA 对照 | 扩展参考 |

### 核心结论

任务层不能只判断“谁有能力”，还必须考虑：
- 资源可用时间；
- 运动代价与运动可行性；
- 实际完成反馈；
- 数据是否已经到达；
- 后续任务依赖。

---

## 3.2 UAV 编队、轨迹规划与避障

| 成果 | 来源 | 主要参考点 | 当前定位 |
|---|---|---|---|
| Swarm-Formation | ICRA 2022 / T-RO 2023 | 编队轨迹优化、稠密环境 formation flight | 当前 AAV 主运动规划后端 |
| EGO-Swarm | ICRA 2021 | 多机实时轨迹规划、广播轨迹、局部重规划 | Swarm 系统重要基础 |
| EGO-Planner | 开源 | 无 ESDF 局部轨迹优化 | 已参考 |
| GCOPTER | IEEE T-RO 2022 | 几何约束连续轨迹优化 | 重点参考 |
| MINCO | 连续轨迹优化 | minimum-control-effort polynomial trajectory | 参考 |
| FIRI | 凸安全走廊 | 安全凸区域生成 | 参考 |
| Fast-Planner | ICRA / 开源 | kinodynamic + trajectory optimization | 参考 |
| FASTER | RA-L / ICRA | 动态环境安全轨迹规划 | 参考 |
| MADER | 多机规划 | 分布式轨迹协调 | 重点参考 |
| Robust MADER / RMADER | IEEE RA-L | 通信延迟下安全轨迹交换 | 通信受限规划重点参考 |
| AMSwarmX | ICRA 2024 | 复杂环境多无人机规划 | 候选 baseline |
| Primitive-Swarm / Primitive-Planner | 开源 | motion primitive 多机规划 | 候选 |
| PUMA | 多机规划 | 多机器人轨迹规划 / 冲突处理 | 参考 |
| LSC | 多机器人规划 | 安全走廊 / line-of-sight | 参考 |
| CAT-ORA | 多机器人规划 | 冲突感知轨迹优化 / 分配 | 候选 |
| ORCA / RVO2 | 经典 | reciprocal collision avoidance | 项目早期安全层参考 |
| CBF / CBF-QP | 控制理论 | 安全约束与控制投影 | 项目安全层重要参考 |
| GCBF+ | T-RO 方向 | 图神经网络 + CBF 多智能体安全控制 | 研究候选 |
| CBS / ECBS | MAPF | 离散多机器人冲突搜索 | baseline |
| SIPP | MAPF | Safe Interval Planning | 对照 |
| libMultiRobotPlanning | GitHub | CBS / ECBS 开源实现 | 工具 |
| CrazyChoir | ROS2 | 多 UAV 分布式编队控制 | 架构参考 |
| MRS UAV System | CTU | 多 UAV 工程栈 | 工程参考 |
| Crazyswarm2 | ROS2 | 多机实验 / 控制 | 实验平台参考 |

---

## 3.3 规划—控制—动力学与安全

| 成果 | 来源 | 主要参考点 | 当前定位 |
|---|---|---|---|
| FaSTrack | CDC 2017 等 | planning model 与 tracking model 的误差边界 | 当前 e_budget 思想重要理论参考 |
| CBF / Barrier Certificates | 控制理论 | 安全集 / 安全约束 | 重点参考 |
| safe-control-gym | RA-L 2022 | 安全控制 benchmark | 候选 |
| ReducedOrderModelCBFs | 控制领域 | 低阶规划模型与高阶动力学安全耦合 | 参考 |
| Neural-Fly | Science Robotics 2022 | 学习扰动补偿 | 控制方向参考 |
| acados | MPC 求解器 | 非线性 MPC | 候选工具 |
| OSQP | QP 求解器 | CBF-QP / 优化控制 | 工程工具 |
| qn / 6DOF plant | 当前项目 | AAV 动力学与控制执行 | 当前权威动态后端 |

核心原则：

```text
规划轨迹安全 ≠ 实际动力学轨迹自动安全
```

因此当前 M2 的 `d_ref` 与 `d_actual` 必须分别评价。

---

## 3.4 通信受限、间歇通信、中继与信息流

| 成果 | 来源 | 主要参考点 | 当前定位 |
|---|---|---|---|
| ACHORD | IEEE RA-L | 将通信质量 / 带宽暴露给任务决策 | 核心思想参考 |
| CoCoPlan | IEEE RA-L 2026 | 任务与间歇通信联合优化 | 后续任务—通信联合规划重点参考 |
| Guo / Zavlanos 系列 | IEEE T-RO 等 | intermittent connectivity、会合通信 | 重要理论参考 |
| Kantaros / Zavlanos 系列 | T-RO / Automatica 等 | temporal logic + intermittent connectivity | 重点参考 |
| Robust MADER | IEEE RA-L | 通信延迟下轨迹协调 | 已列入运动层 |
| DANCERS | 开源 / 论文 | 通信感知机器人仿真 | 候选实验工具 |
| ROS-NetSim | ROS 网络仿真 | 延迟、丢包、带宽 | 工程候选 |
| ns-3 | 网络仿真 | 网络性能 | 后续工具 |
| OMNeT++ | 网络仿真 | 通信系统 | 后续工具 |
| Aqua-Sim NG | 水下网络仿真 | underwater acoustic network | UUV 通信候选 |
| Goby3 | 海洋机器人中间件 | 水声通信、DCCL、海洋系统 | 海洋通信重要参考 |
| DCCL | Goby | 低带宽消息编码 | 参考 |
| LSTS IMC | 海洋机器人协议 | 多平台通信 / 控制消息 | 重点参考 |
| DUNE | LSTS | 海洋机器人运行时 | 平台架构参考 |
| Neptus | LSTS | 多机器人任务规划 / 指挥 | 母船 / 控制站参考 |
| AquaChat++ | underwater comm | 水声通信 | 参考 |

关键任务约束：

```text
t_start >= max(
    resource_available,
    required_information_received
)
```

“任务完成”与“任务产生的数据已到达下一阶段”必须区分。

---

## 3.5 海上 UAV–USV–UUV 异构协同

| 成果 | 来源 | 主要参考点 | 当前定位 |
|---|---|---|---|
| MIMRee | JFR / Offshore wind project | 海上风机多机器人维护、任务规划、ROSPlan | 海上任务组织重点参考 |
| SeaClear | EU 项目 / 论文 | UAV–USV–UUV 跨域搜索、感知、作业接力 | 跨域任务流程参考 |
| Xiroi II | Sensors 2022 | ASV 作为 AUV ↔ 地面站中继，改善声学链路 | USV 中继角色直接参考 |
| USV–AUV FIM + RL | IEEE TMC 2025 | USV 支撑 AUV 定位与任务执行 | USV/UUV 协同重点参考 |
| UAV–USV ISAC joint inspection | IEEE TWC 2025 | 感知、通信、轨迹、目标调度联合优化 | 任务—通信—运动联合设计参考 |
| heterogeneous marine robot collaboration | IEEE JOE | UAV/USV/UUV 多域感知与态势感知 | 重点参考 |
| PANDORA | EU underwater robotics | AUV inspection、任务执行 | 参考 |
| SWARMs | EU project | 多海洋机器人任务管理 | 参考 |
| LSTS ecosystem | FEUP/LSTS | AUV/USV 任务执行、通信、指挥 | 系统架构重要参考 |
| Energy–Information–Decision coupling | 2026 maritime work | 能量、通信、任务决策联合优化 | 新近参考 |

USV 未来可承担的角色：

```text
SURFACE_INSPECTION
RELAY
RENDEZVOUS
DATA_AGGREGATION
LOCALIZATION_SUPPORT
ENERGY_SUPPORT
```

---

## 3.6 海上任务与巡检 Benchmark

| 成果 | 类型 | 主要参考点 | 当前定位 |
|---|---|---|---|
| CARIC | cooperative aerial inspection benchmark | 有效观测、巡检完成、数据交付 / 评分 | 当前近岸监测任务定义重点 |
| MBZIRC Maritime | robotics challenge | 海上多机器人任务设计 | 场景参考 |
| ATLANTIS | offshore wind robotics | 海上风电机器人作业与验证 | 场景 / 验证参考 |
| OceanGym | marine environment | 海洋机器人场景 / 资产 | 候选 benchmark |
| NOAA NCRMP | 珊瑚监测规范 | 珊瑚调查工作流 | 任务语义参考 |
| CUREE | ecological monitoring | 根据观测结果追加任务 | 任务层参考 |
| DARPA SubT | multi-robot challenge | 通信中断、分布式协作 | 跨领域 benchmark 参考 |

CARIC 只用于参考任务定义、观测与评分原则，不替代当前 qn / Swarm 后端。

---

## 3.7 海洋 / 异构仿真平台

| 框架 | 平台能力 | 当前定位 |
|---|---|---|
| Stonefish | USV/UUV、水动力、海洋环境、传感器、ROS | 未来 USV/UUV 高保真后端优先候选 |
| stonefish_ros | ROS bridge | 后续接入候选 |
| VRX | USV / WAM-V / 海上任务 | USV benchmark 重点候选 |
| DAVE | underwater Gazebo simulation | UUV / 声呐 / 水下环境候选 |
| UUV Simulator | Gazebo classic | 水下机器人旧框架 | 历史参考 |
| Gazebo / GZ Sim | 通用机器人仿真 | 平台候选 |
| PX4 SITL | UAV autopilot | 后续实机前验证 |
| ArduPilot | UAV/USV/UUV | 多平台控制候选 |
| Isaac Sim | 高保真仿真 | 感知需求较强时参考 |
| AirSim | UAV/vehicle | 历史参考，不作为长期主栈 |
| Webots / CoppeliaSim | 通用机器人 | 对照 |

原则：
- 不同时接入 Stonefish + VRX + DAVE；
- 每个物理平台只有一个 authoritative dynamic state source。

---

## 3.8 系统 / 任务执行框架

| 框架 | 主要参考点 | 当前定位 |
|---|---|---|
| ROSPlan | PDDL 任务规划与执行监控 | MIMRee / PANDORA 等系统参考 |
| Behavior Tree | 本地任务执行 | 参考 |
| Aerostack2 | ROS2 multi-UAV mission architecture | 架构参考 |
| LSTS DUNE / Neptus | 海洋多机器人 mission/runtime | 海上系统参考 |
| ROS1 actionlib | Action 执行 | 当前工程主链 |
| FormationAction | 项目自定义适配 | 当前任务 → Swarm 接口 |

---

# 4. 当前真正进入项目主链的模块

| 模块 | 当前采用 |
|---|---|
| 任务分配 / 调度 | Calvo 受限移植 |
| UAV 编队运动 | Swarm-Formation |
| 任务→运动接口 | FormationAction |
| AAV 控制 / 动力学 | qn |
| ROS 执行 | ROS1 Noetic / actionlib |
| 场景 | 解析 scene publisher |
| 安全验证 | 实际 qn 状态 + analytic obstacle truth |
| 可视化 | RViz + mission dashboard |
| USV/UUV | 当前只有目标资源模型，无真实执行端点 |

---

# 5. 当前优先全文下载与复核

不是所有已调研成果都要立即下载全文。

## P0：当前任务线优先

1. CARIC  
   - 巡检任务如何定义；
   - 什么叫有效观测；
   - 数据交付与完成判据。

2. Calvo T-RO  
   - 任务分配、执行反馈、plan repair 的真实边界。

3. Swarm-Formation ICRA 2022  
   - 与当前开源源码直接对应；
   - 编队目标与执行语义。

4. Swarm-Formation T-RO 后续  
   - 编队重组和扩展能力；
   - 与当前源码的差异。

5. GRSTAPS IJRR 2022  
   - 任务分配如何查询运动可行性与旅行时间。

6. APEX-MR RSS 2025  
   - 异步执行、实际完成、任务释放条件。

7. Xiroi II  
   - USV 中继；
   - ASV–AUV 协同；
   - 数据汇聚。

8. USV–AUV TMC 2025  
   - 水面平台如何支援水下定位与任务执行。

9. CoCoPlan RA-L 2026  
   - 后续任务—通信窗口—中继联合安排。

## P1：对应功能进入实现时深入

- FaSTrack
- Robust MADER
- MIMRee
- SeaClear
- H-LTL / GCS
- Stonefish / VRX / DAVE 技术论文与文档
- CAT-ORA
- Primitive-Swarm
- CBF / GCBF+

---

# 6. 论文全文处理规则

开放全文保存到：

```text
research/literature/papers/
```

结构化笔记保存到：

```text
research/literature/notes/
```

无法获得全文时：
- 明确标记为 `abstract-only`；
- 不根据摘要推断正文算法细节；
- 不把摘要级材料写成“已全文复核”。

推荐目录：

```text
research/
└── literature/
    ├── manifest.yaml
    ├── papers/
    │   ├── mrta/
    │   ├── task_motion/
    │   ├── formation/
    │   ├── safety/
    │   ├── communication/
    │   ├── maritime/
    │   └── benchmark/
    ├── notes/
    └── RESEARCH_MAP.md
```

---

# 7. 每篇论文统一提取模板

每篇核心论文只回答以下四类内容：

```markdown
# Paper

## 1. 它解决什么问题
- Problem formulation
- Assumptions
- Inputs
- Outputs

## 2. 它的任务、资源、通信和运动模型
- Task
- Agent / capability
- Resource
- Communication
- Motion
- Execution feedback

## 3. 对我们哪个接口或约束有影响
- Mission Request
- Task
- Executor
- Task–Information Graph
- TravelTimeProvider
- FormationAction
- Swarm
- qn
- observation / delivery
- planRepair

## 4. 它要求我们增加什么实验或反例
- 通信断开
- 执行延迟
- 资源冲突
- 轨迹不可行
- 数据尚未到达
- 单机 / 编队切换
- 安全失败
```

不要写成长篇泛化综述。

---

# 8. 使用本 Skill 的研究流程

### Step 1：定位问题层
先判断属于：
- Task / MRTA
- Task–Motion
- Formation
- Safety
- Communication
- Maritime heterogeneous cooperation
- Benchmark
- Simulation backend

### Step 2：检查已调研成果
先查本 Skill 总表，避免重复检索。

### Step 3：区分当前采用与研究参考
明确：
- 当前项目已经用了什么；
- 哪些只是论文提出；
- 哪些只是候选。

### Step 4：优先复核直接相关全文
如果接口设计会被论文直接影响，优先读取 P0/P1 全文和现有笔记。

### Step 5：只提取可落实的约束
论文结论必须落到以下至少一项：
- 接口；
- 状态变量；
- 任务依赖；
- 资源约束；
- 安全条件；
- 实验 / 反例。

### Step 6：避免算法堆叠
若现有主链已经能完成目标，不因发现新论文就自动换算法或新增模块。

---

# 9. 当前近岸监测任务线应优先参考

当前“近岸监测 AAV 子系统验证”实现前，重点复核：

```text
CARIC
→ 观测与交付完成判据

Calvo
→ 调度与执行修复

Swarm-Formation
→ 单机 / 编队运动边界

GRSTAPS
→ 任务—运动可行性反馈

APEX-MR
→ 实际完成与任务释放

Xiroi II
→ USV 中继角色

USV–AUV TMC
→ 水面支援水下任务

CoCoPlan
→ 后续通信联合调度
```

当前不因这些论文新增：
- 自然语言输入；
- 动态组队；
- 并行调度；
- 高保真载荷仿真；
- USV/UUV 动力学；
- ROS2 / 仿真器迁移。

---

# 10. 关键研究原则

1. **任务层必须知道下层运动是否可行，以及实际执行何时完成。**
2. **规划参考安全不等于实际动力学轨迹安全。**
3. **数据生成与数据接收必须区分。**
4. **通信可以成为任务与角色选择的一部分，而不是透明管道。**
5. **USV 可以是移动中继、数据汇聚、定位支援节点，而不只是水面执行器。**
6. **Swarm-Formation 是 AAV 编队运动后端，不是整个异构任务分配系统。**
7. **七机是开发 / 回归基线，最终目标平台是 3 AAV + 1 USV + 1 UUV。**
8. **当前主链只有真实接入的模块才可描述为“已实现”。**
9. **发现新论文时，先判断是否改变现有接口、约束或实验，再决定是否纳入。**
10. **下载论文不是为了堆算法，而是为了减少拍脑袋定义任务接口、完成条件和协同关系。**
