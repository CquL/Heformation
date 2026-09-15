# 异构无人集群第一版实施方案 V2：先复现、最小集成、再形成新方法

**日期：2026-09-15｜状态：修订方案，尚未本地复现新增组件。**

> 本版替代2026-09-14版中“继承旧HUC、审计其接口、在其目录继续扩展”的实施决定。旧HUC不再作为新系统的代码基础、接口规范或理论依据；本轮没有读取该仓库。用户已确认的复现起点仅为Swarm-Formation。文献、源码核对不等于本地运行通过；组合系统也不自动继承原论文的全部保证。

## 1. 本版确定的方向

目标仍是**母船统筹的海上异构无人集群任务与编队作业系统**，包括任务分配、资源调度、必要的协同/编队、路径与轨迹规划、本地控制和执行反馈。图像识别、珊瑚/风机缺陷诊断、载荷选型、SLAM与定位算法研发不在范围内。

第一版直接保留完整的**联合规划协调器—各平台执行闭环**。不再把项目缩成“先做几个相互独立的轨迹演示，以后才补任务层”。但也不把第一版等同于“任务、连续轨迹、路由、所有模式切换一次性全局最优”。

实施顺序为：

```text
原论文与源码核对 → 原始小例子独立复现
        → 保持原生接口的最小组合系统
        → 同一任务场景下测试通信、资源与执行变化
        → 根据真实缺口冻结研究问题
        → 修改算法、明确假设、证明与对照实验
```

**当前三条所谓“创新”降级为研究问题清单，不作为已经成立的原创贡献。**任务—信息依赖、通信中继和状态时效都存在相关先行研究；研究增量应在组合系统暴露的问题上建立。[R07–R14]

## 2. 从V1删除什么，保留什么

| V1内容 | V2决定 |
|---|---|
| P0首先审计旧HUC，复用其Coordinator、Envelope、Sxx模块 | 删除。不访问旧仓库，不继承其接口体系 |
| 强制以旧Python/ROS 2外壳承载全部上游 | 删除。原始复现先使用上游测试环境，最终环境由组合测试决定 |
| 预先定义大量TaskContract、Certificate、Envelope、权限/缓存对象 | 删除前置要求。先用上游Task、Action、Odometry、轨迹和反馈接口 |
| 第一版只有顺序式调度，联合规划后补 | 修改。第一版必须有分配/时序与运动可行性或实际耗时的反馈 |
| 固定必须改Swarm-Formation才能原创 | 删除。与新后端对照后再决定保留/替换 |
| 一开始优化移动中继、跨域路由、大文件缓存、安全信息截止时间 | 降为后续研究。第一版通信首先是执行条件和测试变量 |
| 已知地图、控制侧作业目标、平台独立闭环 | 保留 |
| 母船统一统筹，不要求异构平台全时同步 | 保留 |
| 新方法需要模型、对照与证明 | 保留，但先由基线缺口驱动，不预先宣布新理论 |

V1及旧AGENT背景中凡写“继续继承旧HUC”的内容均不再约束新工程。无需先检查旧HUC才能开展新方案。

## 3. 选型结论：上层一个主解算器，下层按任务选后端

### 3.1 第一轮主线建议

**上层优先复现Calvo/Capitán的T-RO 2025资源调度及其配套执行系统；同时用D-ITAGS小实例核对真正的任务—调度—运动交错机制。**

默认以资源受限巡检为第一场景时，Calvo模型与执行架构作为组合起点；D-ITAGS不是第二个同时下命令的调度器，而是联合协调结构的有源码参照和可替换上层。

选择有明确边界：

| 上层候选 | 更适合的问题 | 不能谎称已包含的能力 |
|---|---|---|
| Calvo T-RO 2025 + 原生执行框架 [R09,R10] | 电池/续航、充电、任务拆分与工作接替、联合任务、执行延迟后的修复 | 原问题不含任意任务前置关系和任意开始时间窗；不是海上通信联合优化 |
| D-ITAGS RA-L 2023 [R08] | 显式任务依赖、异构能力、联盟形成、调度与运动可行性/旅行时间交错、定向修复 | 不能把资源分配次数等同于完整能源/充电管理；不含完整海洋控制 |

**若第一版必须把任意任务前置关系作为优化变量，在线主调度器改选D-ITAGS，而不是在Calvo执行器里增加等待条件后声称“已经联合优化任务图”。**若以电池和长时作业为核心，则选Calvo，不把复杂前置关系硬塞进首个原始复现。

这是范围与上游模型的匹配，不是按论文新旧排名。第一轮两个最小原始例子运行后冻结一个在线主解算器；其他作为独立对照。

### 3.2 运动、编队与控制

- **Primitive-Swarm / Primitive-Planner，T-RO 2025 [R03]**：新增优先复现项，用于独立航行与多机相互避让的UAV后端；它不是严格编队维护的直接替代品。
- **Swarm-Formation ICRA 2022 [R01]**：保留已复现结果和相应编队功能，不因年份删除；与同作者T-RO 2023 [R02]逐项核对源码覆盖。
- **CAT-ORA T-RO 2025 [R05]**：只在“开阔无障碍区域编队重排”是第一版明确动作时加入；不把它当一般复杂障碍导航器。
- **RMADER RA-L 2024 [R12]**：通信延迟实验的针对性对照。不是放到其他规划器之后再优化一次的固定串行模块。
- **AMSwarmX ICRA 2024 [R06]**：复杂障碍场景的备用对照；不要求第一轮全部安装。

同一机器人同一时刻只有一个生效的运动后端。不同后端最初分实验运行；任务内切换只有在时间、初始状态、控制接口与安全切换行为明确后再开启。

### 3.3 高飞团队更新的准确解释

T-RO 2023是原Swarm-Formation的直接研究延伸，增加编队重组、对齐/槽位分配与全局—局部协调。作者的代码链接仍指向Swarm-Formation仓库。本轮没有核实一个独立且完整覆盖这些模块的新仓库，因此不能把原始ICRA代码直接标为“完整T-RO版本”。[R01,R02]

T-RO 2025 Primitive-Swarm有明确官方仓库`ZJU-FAST-Lab/Primitive-Planner`，代码与论文名称不同。它采用离线运动基元与在线选择，适合轻量多机导航；不是“Swarm-Formation-v2”。论文中1000机为仿真，8机为真实飞行；大规模仿真使用共享内存交换轨迹，不能当作受限无线通信验证。论文也明确说明重规划时加速度不连续的局限。[R03]

2025年Number Adaptive Formation/DG预印本更直接处理数量变化和狭窄环境编队，但本轮未确认正式发表和官方完整源码；正文假设同配置平台、使用高可靠5G传输，不应作为水下受限通信的现成解决方案。[R04]

### 3.4 海面/水下模型与仿真

UAV后端不能仅调小速度便成为UUV后端。水面/水下分支可先用Fossen提供的公开运动模型与控制例子做数值基线，进一步评估Stonefish的原生ROS接口与水动力场景。[R17,R18]

Stonefish有OCEANS 2019基础工作，也有ICRA 2025更新；它是海洋仿真基础设施，不是自带任意多UUV任务编队控制的完整系统。不因其提供视觉渲染而把感知算法纳入本项目。原生ROS分支与仿真库版本须匹配。

海洋后端本轮只完成资料选型，尚未确认一套与上述UAV后端同接口且已跑通的完整海上控制链。**这项实际适配与模型验证仍需做，不能被“用了开源框架”省略。**

## 4. 第一版总体架构：功能完整，接口精简

```text
结构化任务 + 可用平台/资源 + 已知地图
                      ↓
             母船侧联合规划协调器
   ┌────────────────────────────────────┐
   │ 任务分配与时间/资源安排             │
   │       ↕ 可行性、旅行时间、执行延迟  │
   │ 运动规划查询 / 编队作业安排         │
   │       ↕                            │
   │ 保留原解 / 修复 / 重新分配          │
   └────────────────┬───────────────────┘
                    ↓ 每平台任务序列、目标与时序
           原生任务执行器 / 行为树
                    ↓ 薄动作适配
        已选运动后端 + 原生轨迹交换机制
                    ↓ 原生参考/控制接口
              平台控制器与动力学
                    ↓ 状态、成功/失败、耗时
              回传至母船协调器

跨平台消息：原生ROS通信 → 可选受限通信测试/网络联合仿真
本机状态闭环：不经母船网络
```

### 4.1 “联合”必须体现为实际反馈

第一版至少验证一次以下闭环：原任务分配对应的路线绕行/等待增加 → 运动可行性或执行耗时返回 → 上层调整时序、分配或任务队列。不能只把独立模块放在一个矩形框内便称联合规划。

D-ITAGS原文Figure 1和方法章节提供“分配—MILP调度—运动查询—可行性/耗时反馈”的直接参照。Calvo提供中央任务规划和本地执行修复的实现。两者支持我们的组织方式，但**它们与Primitive/Swarm的组合是待实施集成，不是原作者已经验证过的同一系统**。[R08–R10]

运动后端原本仅有在线目标接口、没有“完整转场代价查询”时，不能假定它已有该服务。可先使用上游已有全局路径查询或原MRTA旅行时间估计，并由实际执行延迟触发修复；新增提前可行性查询时单独记录为适配工作。不能声称原始局部规划器能预知未来全部避让时间。

### 4.2 完整架构与统一全局最优的区别

第一版可以具有完整协调架构，同时使用交错、事件反馈和已有启发式求解。这是可执行的工程基线。它不自动具备以下结论：任意通信丢失下安全、所有异构动力学下可执行、任务/路径/网络共同全局最优。

### 4.3 母船与节点的职责

母船负责全局任务目标、分配和时间资源安排；节点按原生框架执行动作、跟踪和必要的局部避让。使用分布式局部规划器时保留其原通信机制，不要求母船替代所有高频控制。母船只能基于实际收到的反馈做在线更新。

## 5. 必要输入与接口：优先复用，而不是先造数据包体系

### 5.1 三类输入即可描述第一版

1. **任务**：动作/任务类型、作业位置或区域、所需平台资格/数量、工作时长或路径要求、截止时间；任务关系只采用所选上层实际支持的形式。
2. **配置**：机器人列表、已有模型/工作域、运动限制、能源与地图。传感器未知不写虚构传感器型号。
3. **在线反馈**：各节点实际发布的本机状态、任务完成/失败、能源和必要邻机轨迹。

配置项不是每次通信都发送的数据包。日志字段也不应强迫每层重复包装。

### 5.2 已读到的上游接口

Calvo执行仓库原生`action/HeuristicPlanning.action`：[R10]

```text
请求：scenario_id, available_agents[], remaining_tasks[]
结果：success, TaskQueue[] planning_result
反馈：status
```

原生`action/NewTaskList.action`：

```text
请求：agent_id, Task[] task_list
结果：ack
反馈：status
```

原生`action/TaskResult.action`已有任务和FAILURE/SUCCESS/HALTED结果，另带原场景的近查字段。不应把该应用字段原样扩成我们的传感器依赖。

Primitive-Planner原生`pp_replan_fsm.cpp`提供：[R03]

```text
本机状态输入：odom_world
目标输入：/goal_with_id，或预置waypoints（取决于flight_type）
邻机轨迹输入：planning/broadcast_primitive_recv
对外轨迹广播：planning/broadcast_primitive_send
后端输出：planning/selected_path_id、planning/polynomial_traj等
```

具体namespace/remap和消息类型以锁定版本的launch与头文件为准。这里列的是已查到的源码名称，不冒充本地运行topic清单。

### 5.3 第一轮只新增必要的薄适配

```text
任务动作 → 运动后端原生目标/协作请求
运动执行结果 → 上层原生任务结果/延迟反馈
```

另在通信测试阶段将原生发布/接收接入可控网络条件。没有语义缺口就不新增协议。

编队任务需要指定成员和目标队形，但若原后端只支持固定编队规模/ID，这个限制必须先在原始例子中核实，不能认为任意联盟可直接复用。原框架只发布局部完成事件时，也不能把它等同于整个作业路线完成。

### 5.4 最小一致性要求，不扩大为新框架

只有以下事情必须明确：坐标/单位、时间基准、机器人ID、谁发布本机状态、谁拥有生效任务、完成/取消含义、后端输出的控制层级。优先采用上游字段；确实缺失时才补充。不要预先引入统一证书、几十种Envelope、复杂缓存授权层。

## 6. 通信约束：第一版保留，但先作为条件，不先成为巨大优化问题

海上母船与航行器信息交换是系统工作条件，所以通信需要进入测试。第一版先关注：任务指令、邻机轨迹/状态、任务成功失败、能源/故障反馈。没有业务图像处理，就不把大图像传输作为核心负载。

原生发布订阅已经解决程序通信，不能据此推出海上网络时延、丢失和容量都已处理。第一轮保留原消息内容，在发送/接收边界注入固定延迟、抖动、丢失或短时断链；同时保留理想通信对照。

本机真实仿真状态可以通过自己的Odometry接口给自己的控制器；邻机状态只在消息到达时更新。已知静态地图可预装。禁止用全局仿真真值绕过远程通信。故障脚本不提前告诉规划器。

**物理多跳不由ROS topic名称自动实现。**固定中继和可达边可作为场景配置；移动中继与会合的最优选择留到基线证明其必要之后。没有设备依据就不默认AAV具备射频—声学网关。

RMADER提供延迟处理的原生算法对照，保证依赖时延上界和已提交安全轨迹；DMPC-Swarm处理消息丢失，但有特定同步通信/计算结构、跟踪误差和初始可行轨迹假设，不能机械搬到欠驱动水下平台。[R12,R13]

DANCERS提供物理—网络双向联合仿真的基础设施，ROS-NetSim提供同类方法依据。DANCERS是SIMPAR论文，不写成T-RO；默认UAV/Wi-Fi示例也不写成完整水声仿真。ROS 1与ROS 2连接器兼容性需实际测试，不要求第一轮同时迁移全部工程。[R15,R16]

## 7. 第一版场景与最小对照

选择**母船组织的近海多区段巡检/调查作业**。作业位置与动作由外部给定，包含多于可用平台数量的子任务，从而产生真实的分配和等待选择。

任务类型仅为已实现控制动作：目标转场、区段跟随、区域路线执行、平台可行的定域作业、合格子团队编队转场、返回。作业能力按平台配置，不设置未知图像识别条件。

第一版最终目标为不少于5个真实参与任务的仿真航行器，覆盖项目需要的工作域。中间可先用UAV完成整个任务闭环调试，但该阶段仅叫UAV接口集成测试，不能报为空水潜系统验收。欠驱动水下模型、海面模型和跨介质切换须逐项确认；不靠改类型字符串宣称异构。

首轮测试四组：

| 组别 | 要检验什么 |
|---|---|
| 正常任务 | 分配、时间安排、运动执行和反馈是否形成闭环 |
| 绕行/等待增加 | 下层耗时变化是否真的改变上层计划 |
| 节点退出/任务追加 | 原生修复或重规划能否处理，未收到指令者是否仍执行旧任务 |
| 消息延迟/短时断链 | 避让、编队和任务接续如何退化，是否出现真实失效 |

不强制所有机器人一直成队。原生规划器失败、任务未完成或安全条件违反都应计入结果；不能只挑成功视频。

## 8. 实施计划：按可运行交付推进，不先写几十份接口文件

| 步骤 | 做什么 | 可检查交付 |
|---|---|---|
| 1 原始复现 | 保留Swarm基线；运行Primitive原例；运行Calvo最小分配/修复例；运行D-ITAGS小例 | 每个原始例子的命令、配置、版本、结果，失败则说明原因 |
| 2 冻结上层 | 根据V1是能源优先还是任务依赖优先，选择Calvo或D-ITAGS为唯一在线主求解器 | 一份明确选型决定；另一个只做对照 |
| 3 原生接口组合 | 使用既有执行器/行为树；新增任务到目标、结果到反馈的薄适配 | 一次完整任务从请求到完成，不保留旧HUC依赖 |
| 4 联合闭环 | 将运动不可行/旅行时间或执行延迟送回上层 | 一个因真实绕行/延迟而改变任务计划的可重复例子 |
| 5 异构与通信测试 | 引入已验证海洋平台分支与受控网络；按需评估Stonefish/DANCERS | 明确哪些工作域和链路已实际通过，哪些仍是模型假设 |
| 6 缺口与新方法 | 对相同场景比较基线，挑选最有价值且未被已有工作覆盖的问题 | 新模型/算法差异、实验和证明义务，而不是命名新框架 |

步骤1中多个小复现可以并行，但不把所有上游整仓库混进同一个工作空间。没有许可证或旧依赖无法运行时要记录，而不是悄悄重写求解器后宣称复现原方法。

### 环境策略

Primitive、Swarm、Calvo执行框架等公开说明均有ROS Noetic环境。第一轮可用隔离的Ubuntu20.04/ROS Noetic环境重现原结果；不因此决定新工程永远停留在该版本。DANCERS等ROS 2候选单独隔离。最终迁移须在原始结果稳定后进行，不同时做算法改动、接口重命名和操作系统迁移。

### 代码组织建议

```text
maritime_mission_system/
  upstream/       # 固定版本的原始成果，少改动
  integration/    # 只放确实需要的薄适配
  scenarios/      # 第一版任务、平台、地图、事件配置
  experiments/    # 复现与统一对照入口
  docs/           # 选型理由、接口对照、结果记录
```

这是建议的新工程，不是旧HUC目录改名。上游内部节点、消息、控制与模拟器不重复实现。多个模拟器联用时统一时钟，每个平台只由一个动力学实例推进；不可把两个后端都连到同一个执行器。

## 9. 怎样判断能否成为新方法

现在不再承诺“任务图+信息图融合就是创新”，也不预先把通信时效当唯一方向。优先从三个可测问题筛选：

- **计划与运动不一致**：上层分配可行，但编队重组、避让或转场耗时使原时序失效；现有修复是否足够，能否获得更好的可行性反馈？
- **计划与接收信息不一致**：母船已经重新分配，但部分节点还执行旧目标；原协议适用假设在海上哪里失效？
- **统一运动假设不适用**：UAV的停止、侧移、对称避让策略不能在另一平台兑现；具体需要修改什么约束或协作规则？

每个问题先对照原论文。某方法明确不解决的问题，不作为故意削弱基线的唯一依据；应同时比较该问题的专门方法，如RMADER/DMPC、D-ITAGS或CoCoPlan。[R08,R12–R14]

证明也分层：分配/调度的可行性和近似界、运动规划的假设与安全、平台跟踪条件、异步执行的一致性。改了模型/控制器/通信机制，就重新核对上游证明适用条件。

**有保证的子算法拼起来，不会自动得到有保证的全系统。**所需的新理论可能在这些接口条件上产生，但先建立最小组合基线再决定。

## 10. 需要保留的证据，精简到实用程度

每次实验保存：上游commit与改动、实际配置/随机种子、运行命令、原生ROS日志或bag、任务结果、轨迹/状态、必要消息时序、指标摘要。日志可离线丰富；不要为了日志再要求每层传输巨大的新协议。

正式比较至少看：任务完成与超期、总作业时间、资源使用、实际最小间距、跟踪/编队误差、规划修复耗时、失联与消息延迟条件。理论假设不满足的实验可以报告经验结果，但不能继续引用该定理作为保证。

## 11. 当前必须明确的未完成事项

1. 新选框架尚未在用户机器上完成本地复现。
2. 本轮未核实T-RO 2023全部模块对应的独立公开实现。
3. Calvo/D-ITAGS与Primitive/Swarm的组合不是现成官方集成，需要薄适配。
4. 完整空水潜控制后端及AAV切换能力尚未验证。
5. 尚未确立原创贡献或完成新理论证明。

这些是明确的工作项，不是必须先制造复杂基础设施的理由。

## 12. 立即执行的下一步

**先跑四个清楚的小例子：已有Swarm编队、Primitive多机导航、Calvo资源任务分配/修复、D-ITAGS任务—调度—运动例。随后冻结一个上层，接一条原生任务执行链，第一版就验证“任务分配—运动—反馈—重新安排”。**

不再审计旧HUC，不先写统一Envelope/Certificate系统，不一次安装所有候选论文代码，不把复现与集成成果提前命名成统一新理论。

---

## 参考文献与代码入口

以下为本轮直接查阅的论文/官方说明及已确认源码入口；读到源码不等于本地复现。详细证据范围见同包《论文与源码核查记录》。

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
