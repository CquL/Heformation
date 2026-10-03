# Heformation 项目目标、当前事实与 server715 接手说明

更新日期：2026-10-03。本文件面向直接在 **server715** 上继续工作的用户和 Codex。最新主线为 A/B/C 三业务共用的任务规划与协同控制；**B 已完成一轮监测闭环，完整 C 和最终动态返航总验收尚未完成。**用户现已决定转到服务器接手，本地本轮执行停止，不会继续启动新仿真。

## 1. 项目到底要完成什么

我们部门承担三项内容：

1. 异构无人多航行器集群任务规划与协同控制。
2. 异构无人航行器集群路径自主规划。
3. 异构无人航行器跨域自适应控制。

工程上建立的是一个**面向任务的异构多航行器动态规划与协同执行系统**：接收用户任务和当前平台状态，选择成员、方法、并行关系及支援，生成联合方案；执行端组织真实模型运动，反馈实际进度、动作终态和结果接收，再继续、调整、驻留或返航。

固定平台为 **3 台同型同能力 AAV＋1 台 USV＋1 台 UUV**。三台 AAV 身份可以互换；AIR 与 WATER 端点属于同一物理成员，不能重复占用。

我们只负责任务分配、资源调度、运动规划、编队与跨介质执行、控制侧完成报告，以及接收外部结果后的任务调整。专业损伤识别、珊瑚健康判断、真实视觉／声呐质量评估和精细三维重建由外部模块承担。当前模拟感知和控制报告不能当作这些专业业务的完成证明。

最终业务流程为：

```text
UI 选择业务、区域／设施和作业范围
    ↓
根据实际平台状态快速生成完整联合 Plan
    ↓
用户确认
    ↓
平台并行运动、完成规定巡视段，USV 提供共享支援
    ↓
实际合格进度＋匹配 Goal 的动作终态＋母船结果实收
    ↓
目标区驻留
    ↓
用户选择下一任务、排队任务或共同返航
```

执行中、驻留时和返航中都要能接收新任务。执行中新增任务由用户选择“排队”或“立即替换”；替换要经过原执行端的安全终态交接。任务完成后默认驻留，收到明确返航指令才返航。AAV 返回使用原 Swarm 编队；UI 与独立 RViz 展示同一任务事实。

## 2. A、B、C 的业务定义

业务范围来自 `context/` 中的项目申报书与子课题03实施方案。三个业务复用同一系统，只在任务展开和作业几何上不同。

| 业务 | 用户输入 | 生成的控制侧作业 | 完成判断 |
|---|---|---|---|
| A：近海资源环境调查／珊瑚礁监测 | 鼠标圈选监测区域及范围要求 | 到达区域后执行扫描运动；按实际运动和模拟感知累计已覆盖情况 | 规定控制作业、必要结果实收及实际终态；不判断专业监测质量 |
| B：海上风电装备巡检 | 选择风机和水上／水下部位、层数、偏距等 | AAV 巡视塔筒／机舱／静止叶片；UUV 或 AAV 巡视水下基础；USV 共享支援 | 各必做作业的合格区间完成、动作终态匹配、报告实收 |
| C：海上作业平台／海底管路巡检 | 选择海上设施平台、外侧／水下结构和连续管段 | 平台外轮廓、水下可达外部结构、沿管路安全偏置巡视及指定管件局部作业 | 同上；管路中心线是设施几何，机器人执行偏置运动 |

C 中“平台”指海上设施／作业平台，不指航行器平台。第一版使用当前缩比场景、静止且姿态已知的风机和可达外部结构，不进入桁架内部。高度、深度、偏距和速度是控制演示任务要求，不作为专业检测规范。

B/C 共用三种运动生成方式：**分层轮廓、沿线／条带、局部作业**。默认三层可在确认前调整；设施几何给出作业意图，从实际位置到入口、段间连接和绕障由执行端处理。不能把全程路线手写成输入，也不能只用“到达若干点”替代规定作业。

完整设计和文献依据见 [B/C 模板设计](inspection-business-templates-20261001.md)。项目文件均已纳入 GitHub；业务来源、方法借鉴、上游源码复用和本工程设计应分别陈述。

## 3. 当前架构与输入输出

总体架构已经冻结，不需要再搭两套系统或新增通用任务引擎。

| 层 | 输入 | 输出／职责 |
|---|---|---|
| UI／请求展开 | 业务、区域／设施、范围、方法与工作要求 | 请求 ID、对象／几何版本、必做 work、允许方法及完成条件 |
| 联合分配与资源调度 | 请求、能力、当前状态、已接纳活动和共享支援条件 | 一份现有 Plan：成员、方法、动作链、接续／并行关系、支援关联及估计时间 |
| 任务权威／会话 | 用户确认后的 Plan、实际状态、Action 结果、收件 | 派发原 Goal、占用／释放真实成员、排队／替换、调整、驻留与返航 |
| 本地规划与控制 | 当前短段作业意图、实际位姿、局部感知、相关成员参考／状态 | 可执行短段参考、原生动力学控制输入、实际运动和反馈 |
| 进度／结果接收 | 实际位置、速度、模式、规定朝向、work 版本、GoalID | 合格区间并集、控制执行报告、本地产生／动作完成／母船实收的分别记录 |
| 可视化 | 同一任务权威的 Plan 和实际反馈 | Qt 二维选区／方案／状态、独立 RViz 实际运动与 AIR/WATER 模式 |

### 联合求解的边界

仍为联合求解：成员、执行方法、接续／并行与支援共同确定。固定的是候选展开顺序，实际执行不能因此串行。

规划和修复预算为 **10 墙上秒**，使用任务级估计排序，取得首个完整可派发方案即返回。不要求找全局最优，不逐候选模拟全部 Swarm／qn／Otter 运动。预计工期用于调度参考，不能当真实结束时刻。当前 B/C 候选排序使用成员可用时间、当前位置到作业入口的距离／名义速度、作业长度／参考速度与局部作业时间，估计只用于排序；成员、接续与共享支援仍须形成完整可派发 Plan。执行中 PVS 的短窗口原生指令＋coast 查询属于局部运动执行，不是把整项任务提前重跑。

### 后端与完成量

- AAV AIR：官方 Swarm 源码加项目补丁，输出参考，由 qn 原生模型跟踪。
- AAV WATER：原 qn 动力学与前进式水下控制，保留 ENTER／EXIT 交接。
- UUV：固定 qn WATER，使用同一水下动力学分支；当前生产任务不使用 REMUS 执行。
- USV：Otter／PVS 原生模型及原本地 Action 执行。
- AIR 可以声明面向结构的 yaw；WATER 航向主要沿路线切向，不增加独立侧移控制。
- 轮廓按实际合格弧段、沿线按长度区间、局部作业按连续合格时间累计；重复经过不重复计数，连接／绕障／暂停／样本跳变不补齐缺段。
- 账本绑定 `(request_id, work_id, version)`。新请求从零计量；原请求合法修复只保留仍属于同一有效定义的区间。环境地图记忆和作业进度是两回事。
- 通信采用任务级共享支援：支援实际到位、服务有效后，合法报告在实际更新中接收。不计算字节和逐跳链路，仍保留报告去重、GoalID、实际终态和资源锁。

“名义末点／Plan 预估末点”是预先估计的动作结束位置；“实际终点驻留”是作业与实际终态已经验收后，在没有指定停车点时保持实测结束位置。规定巡视范围、深度／高度、偏距和明确会合点仍必须完成。“重定时”是在运动层调整当前短轨迹时长，保留边界位置／速度／加速度并重新检查轨迹。

## 4. 关键代码位置

以下路径均相对服务器项目根目录。

| 文件 | 作用 |
|---|---|
| `integration/qn_aav_simulator/scripts/mission_console.py` | A/B/C 选择、设施／区域、Plan 确认、队列／替换与人工返航 UI |
| `integration/qn_aav_simulator/src/qn_aav_simulator/monitoring_request.py` | 请求解析、A/B/C 模板与设施作业展开 |
| `integration/mrta_python/executors.py` | 现有 Task／执行步骤／Plan；B/C `build_inspection_executor_plan` 联合分配和共享支援 |
| `integration/qn_aav_simulator/scripts/formation_mission_runner.py` | 唯一任务权威、Action 派发、资源锁、实收、动态会话、驻留与共同返航 |
| `integration/qn_aav_simulator/src/qn_aav_simulator/inspection_work.py` | 唯一新增共享运行文件：作业几何与进度计算，不承担 ROS／会话管理 |
| `integration/qn_aav_simulator/scripts/formation_action_server.py` | AIR Swarm 目标、实际终态与 AIR 作业计量 |
| `integration/qn_aav_simulator/src/qn_aav_simulator/platform_action.py` | qn WATER、AAV 跨介质作业与原生终态 |
| `integration/qn_aav_simulator/scripts/pvs_node.py` | USV 原生动作、本地指令＋coast 查询、共享支援 |
| `integration/qn_aav_simulator/src/qn_aav_simulator/observation_coverage.py` | `LocalSurveyMap`、实测局部导航地图及 A 覆盖相关计算 |
| `integration/qn_aav_simulator/scripts/scene_publisher.py` | 当前三业务场景、模拟感知、模型时钟、RViz 实际状态 |
| `integration/swarm_qn_bridge/patches/swarm_peer_safety_contract.patch` | 当前 native Swarm 参考／避碰／限速和刹停修正 |
| `integration/qn_aav_simulator/action/Formation.action`、`PlatformTask.action` | 原有两个 Action，包含可选 work 引用 |
| `integration/qn_aav_simulator/config/five_scene_offshore.yaml` | 当前三业务场景与同源运动配置 |
| `integration/qn_aav_simulator/config/monitoring_request_wind.yaml`、`monitoring_request_platform_pipeline.yaml` | B/C 可加载任务模板 |
| `scripts/docker_run_three_class_qualification.sh` → `docker_run_joint_request.sh` | 当前实时入口，脚本名保留历史命名，运行的是共同任务链 |
| `scripts/docker_build_qn.sh`、`docker/Dockerfile.qn` | 原生消息／代码与镜像构建 |

## 5. 已经完成到哪里：以 r67 实际账本为准

A 区域监测主链已有历史完整运行记录，并作为 B/C 的基础。不能把它解释为任意选区都成功或完整三维重建已完成；近期 native 修正后的当前镜像没有新增 A 全业务验收。

最新服务器运行目录为：

`/data/lhj/codes/Heformation/experiments/20261003-server715-facility-live-r67/`

r67 运行镜像为 `swarm-formation-qn:facility-motion-limits`，ID：

`sha256:d08bc27f52bb92c96ac659bc0c544001b88b8cc007cec4311caf8732da04867b`

运动源码检查点 `2fb9a37c`，后续文稿检查点 `54521d0e`。后续接手使用最新 `main`；这些哈希用于溯源，不要求回退。

| 本次事件 | 实际结果 | 证据／边界 |
|---|---|---|
| 完整 B：`b-3ed031dd17a6` | 两项 AIR 各86合格区间；两基础各三层60区间；四项100%；母船4/4实收；13/13 step 已验证成功、6复合活动 SUCCEEDED；零锁 HOLDING | `mission-0000-final.json`。完成的是监测闭环，最终会话返航仍未验收 |
| B→C 排队接续：`c-64c8f62387bc-4db8f2e9db` | 已确认队列并从 B 实际末态激活执行 | 后来执行中被复查请求替换，`mission-0001-final.json` 为 `CANCELED_BY_REPLACEMENT`，不能算完整 C 通过 |
| 执行中立即替换／管件2复查：`c-817dfb11ac83-4cd75eea80` | 求解器选 AAV 完成 ENTER／LOCAL4s／EXIT；进度100%、一报告实收、实际 AIR／IDLE、零锁 HOLDING | `mission-0002-final.json`、`valve2-holding-final-snapshot.json`、`valve2-native-after-receipt.json`。证明这次真实动态交接与局部复查闭环 |
| 新完整 C：`c-0ecb3c060fc9-d961452285` | 从零独立计量；在 AIR15.59%、水下结构18.56%、管路0 时出现成员近距故障；零实收，四锁 `UNKNOWN_LOCKED` | `metrics.json`、`failed-experiment-end.json`、相遇窗口。完整 C 未通过 |

B 初次完整 Plan 约0.0104墙上秒；C 排队激活时约0.0023秒；新完整 C 接受时约0.0022秒。当前主要阻塞在实际执行与相互避碰，不能再把它解释为必须遍历最优分配导致规划慢。

旧世界 `c62317e94fda` 和 runner 已退出，`execution.bag` 正常闭合，约3.91GB；当前没有本轮启动的仿真或读取进程。旧失败仍为 `verified_stop=false`：结束仿真环境不等于已经证明安全停稳，也不释放历史未知锁。不要继续等待旧 exec／容器，也不要用关闭后的 metrics 当作正在运行的实时状态。

## 6. 当前问题与修正状态

### P0：水下 AAV 与 USV 局部执行缺动态成员输入，导致新完整 C 失败

同次已闭合 bag 的相遇证据已经提取，文件：

- `aav-usv-encounter-window.json`
- `aav-usv-encounter-read.py`
- `execution.bag`

同时间戳 `1791026494.161680` 的实际状态：

| 成员 | 模式／执行 | 实际位置 m | 速度 | 当时目标／参考 |
|---|---|---|---|---|
| drone1 | WATER／WATER_PATH／PLATFORM | `(19.650987, -12.158541, -1.200000)` | 约0.2215m/s | 平台基础第一层第11段；局部目标 `(20.275305, -12.909579, -1.2)`；采用参考 `(19.770397, -12.136695, -1.2)` |
| USV | SURFACE／SURFACE_PATH／PVS_NATIVE | `(21.130299, -12.625875, -0.037713)` | 约0.00022m/s，几乎停住 | 局部目标 `(22.875, -12.875, -0.037713)`；整体管路支援点 `(31.105175, -19.363440, 0)` |

该样本净距约0.499313m，随后主监控报告0.493485m，停止后的同一短窗最低值约0.310419m。USV 查询仍显示 `FEASIBLE / NATIVE_COMMAND_AND_COAST`。汇总 `runtime_monitor_min_clearance_m` 停留在失败前的0.5004786m，不能据此宣称安全通过。

**已确定的输入缺口**：`platform_action.py` 的 WATER 局部导航、`pvs_node.py` 的原生指令＋coast 查询只消费静态 survey 导航地图，没有把其他物理成员的当前状态与运动纳入局部避碰。AIR Swarm 的邻机功能不会自动覆盖这两个分支；上层 actual fleet monitor 在近距出现后才发现问题。

**动态成员修正尚未实施。**下一步应复用现有 `LocalSurveyMap` 和 PVS 本地查询，在原执行端补新鲜成员状态／运动检查，并核对支援进场、巡视段和 EXIT 段的空间关系。不要用全队串行化、固定 AAV 身份、随意分高度或取消近距判定代替这个缺口。具体支援区域／进场修改仍须由几何与同次输入证明。

0.5m 是当前场景声明的成员保护净距，0.2m 为环境障碍净距；数值为工程演示配置，不是文献给出的通用海上标准。判定使用实际位置和声明机体包络，不等同于精确模型表面发生接触。

### P1：执行中一次完整 C 预览未找到完整分配，拒因仍待核对

管件2复查执行中提交过一次完整 C 预览，返回未找到完整分配；该预览失败没有使活动任务失败。全部 AAV 当时处于 WATER、导致 AIR work 没有候选，是一个待验证解释，**尚未完成该时刻三成员模式核对**。

相关入口为 `formation_mission_runner.py` 的动态预览／激活和 `executors.py:build_inspection_executor_plan`。核对 queue 和 replace 的状态语义：排队时可以使用已接纳动作链的合法预计末态做任务级估计，真正激活仍须从实际状态重求解；立即替换需要实际合法模式交接。不要未经取证就新增拒绝门槛或泛化成“WATER 时不能新增任务”。

### P1：AAV 前进后退的明确输入源已修，最终实际审计未完成

r61 已证明原 native 急停 `EMERGENCY_STOP → callEmergencyStop(odom_pos_)` 会把位置参考改到落后的实际位置并令参考速度为零。AAV3 的轨迹815→816参考向后跳约0.370707m，实际前冲后折返约0.179302m。

定点零速急停构造来自上游 Swarm；将它直接接到有惯性的 qn、未做好参考交接，是本工程集成责任。它不是任务模板要求 AAV 为同步而后撤。

已在原补丁修改并编译：从当前已提交参考的 P/V/A 连续减速、必要安全参考保持；校核实际将下发的短轨迹速度／加速度并在原边界内重定时；作业后驻留接实测终点。真实应急无法连续刹停时仍保留原物理应急路径与明确原因。

r67 已实际加载这些修正并恢复整项 B 作业。但当前整段的原始目标、`pos_cmd`、PolyTraj、采用参考与实际运动审计**尚未完成**，不能宣称所有实际折返已消除。审计要区分规定巡视转弯、真实必要应急和同一意图下的参考回跳。

### 已修过的相关阻塞，不要重新当成待实现

模拟扫描的原始纳秒采集时间已保留，修复浮点重建造成的 scan／odom 精确配对失败；A* 端点调整无有效进展已改为明确失败；原速度边界判定的近切点数值误拒绝已使用同库极值计算修正；常速／常加速度最大值退化分支已修。r67 已编译加载。完整任务中若出现新证据再定位，不重跑旧单机系列。

## 7. 接手后还要完成什么，按什么顺序推进

1. **先修 P0 的真实执行输入缺口。**以 r67 相遇窗口核对原导航／本地查询，把动态成员状态与运动接入原执行端，处理支援位置／进场与水下作业相遇；只改阻塞任务链的代码。
2. **完成完整 C。**本轮规定范围为平台 AIR 三层／60段、水下外部结构三层／60段、管路连续范围0..3（3个沿线段＋3个LOCAL4s）；各必做作业实际完成；报告实收、原生 Action 实际终态、零锁 HOLDING。此演示覆盖求解器选择的一次 AAV 入水／作业／出水，但不固定身份、不强制所有 C 都必须跨介质。
3. **完成同会话动态与返回验收。**B→C 排队、执行中立即替换／复查、完成后驻留、人工 Swarm 三 AAV 编队返回、返航中替换局部任务后继续完成，再明确共同返航，最终五方实际 HOME、零锁、`end_session`。已有 r67 动态部分是事实，但不能拼成未完成的整条最终验收。
4. **核对实际参考与运动、UI/RViz。**使用同次输入、采用参考、实际 Odometry／模式／结果核对后退与避碰；保留能对应请求和角色的实际 Qt＋RViz 画面。截图和轨迹回放不能替代实跑终态。
5. **同步源码与必要记录。**新链贯通后按实际调用关系清理退出主链的硬编码、重复转换和无消费者字段；保留 A 有效代码与 Swarm／qn／PVS 模型、源码来源及许可证。只推 `main`，实验数据不推送。

判断每个动作的标准是是否帮助完成这条业务链。不新建 Action 包装层、协调服务、能力注册中心或第二套任务状态。不开展单机资格系列、无关回归、旧长序列或测试数量竞赛；必要小检查只针对重复占用、进度跳增、旧报告解锁等直接错误。

## 8. 服务器、磁盘、镜像和运行入口

- 项目目录：`/home/server715/data/lhj/codes/Heformation`。
- 实际数据盘路径：`/data/lhj/codes/Heformation`；两者指向同一项目。
- GitHub：<https://github.com/CquL/Heformation>，分支 `main`。
- Docker 已安装可用，数据根 `/data/lhj/docker`，驱动 `overlay2`；原镜像／工具缓存位于 `/data/lhj/runtime-cache/Heformation/`，未放进系统盘项目仓库。
- 宿主 Ubuntu22.04、64核、约125GiB RAM、双RTX4090；ROS Noetic 使用原 Docker 环境。
- 当前运行使用软件渲染 `JOINT_GPU_RENDER=false`。图形显示到服务器桌面，Qt 和 RViz 分开；在服务器桌面／远程桌面看窗口，不会自动投到旧本地电脑。
- 桌面实测 `DISPLAY=:0`、`XAUTHORITY=/home/server715/.Xauthority`，中文 Noto 字体可用。
- 当前可复用 native 镜像 `swarm-formation-qn:facility-motion-limits` 为 r67 的 d08bc27f；默认 `joint-wip` 标签仍是更早镜像，不应误认为包含全部最新 native 修正。

当前同源运动配置：AIR planner0.6／nominal0.5m/s（用户要求降半后保留）、AAV／UUV WATER0.225m/s、USV nominal0.6m/s／60N、PVS 本地预算2墙上秒；模型时钟目标4×，实际速率受五个模型完成步和计算能力约束。不得通过重设真实位姿／速度、清零惯性或更换物理方程伪造提速／成功。

在服务器桌面终端使用原入口，例如从 B 模板启动：

```bash
cd /home/server715/data/lhj/codes/Heformation
export DISPLAY=:0
export XAUTHORITY=/home/server715/.Xauthority

JOINT_IMAGE=swarm-formation-qn:facility-motion-limits \
JOINT_SIM_SPEED=4 \
JOINT_GPU_RENDER=false \
JOINT_REGION_UI=true \
JOINT_TASK_UI=true \
JOINT_CONTINUOUS_SESSION=true \
JOINT_REQUEST_FILE=/workspace/src/src/qn_aav_simulator/config/monitoring_request_wind.yaml \
bash scripts/docker_run_three_class_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-server715-handoff"
```

这是当前已存在入口与镜像，不表示 C 缺口已经修复。先修对应输入，再用新目录实际验证。C 的模板路径改为 `/workspace/src/src/qn_aav_simulator/config/monitoring_request_platform_pipeline.yaml`；控制台也能选择业务和范围。先在 UI 生成初始任务／场景，ROS 与独立 RViz 随后加载，再核对并确认正式 Plan。控制台关闭不会被当成物理停止命令。

Python 源码由入口挂载，新的进程使用当前源；进行中的进程不是热更新。改 Action 消息或 native C++／补丁后，要用 `scripts/docker_build_qn.sh` 构建并记录实际镜像 ID，再明确选用新镜像；仅 `git pull` 不会重建 native 二进制。不要为了加载源码重复启动一个尚未结束的世界。

运行观察以 `metrics.json`、`nominal-plan.json`、`mission-000*-final.json`、`launch.log`、`runner.log`、各端点 diagnostics、`*.inspection-live.json` 和同次 `execution.bag` 为准。诊断文件必须匹配当前 request／work／Goal，旧 AIR 文件不能冒充新的 WATER Action 状态。结果已生成、动作已完成、母船已实收要分别核对。

## 9. 已迁移的项目和 Codex 上下文

项目源码、配置、模型、许可证、文献及两个项目 PDF／Word 已进 GitHub 并在服务器单独目录管理。实验／bag、构建缓存、凭据、私有聊天不进 GitHub；r67 bag 和证据留在服务器 `experiments/`。

服务器 Codex 配置在 `/home/server715/.codex/config.toml`：保留原模型／账号环境，已设置 `model_context_window=1000000`、`model_auto_compact_token_limit=900000`。本地 Codex 配置没有修改，本地认证没有复制。

本项目可见聊天与附件的私有归档：`/data/lhj/codex-migrations/Heformation/`。原生归档线程名 **Heformation 项目迁移与 B/C 验收**，ID `01a0b951-d37d-7052-afc3-ce5447a2caa0`，工作目录为服务器项目。

原生解析核对：171回合、1885条可见消息（184用户＋1701助手）、45图像引用。原工具记录留在私有 JSONL，没有全部重建为 UI 工具卡片；部分历史临时附件不存在，清单在 manifest。最新增量截断为2026-10-03 11:50:31.168117 UTC，位于 `increments/20261003T115030Z/`。这是注明时刻的快照，之后的最新工程事实以本文件、context02／15和 WORKLOG 为准，不假装无限实时同步。

实际 Desktop Codex 二进制 `/usr/lib/chatgpt/resources/codex` 为0.159.0-alpha.12.1；`/snap/bin/codex` 为较旧0.114，不要混淆两个版本。直接在服务器 Codex 打开项目并阅读入口即可；若在本地控制服务器，才使用原 SSH skill。后续源码和计算以服务器为准，避免本地旧工作树与服务器同时改代码。

## 10. 给服务器 Codex 的接手提示词

可以直接粘贴以下文字：

> 请在 `/home/server715/data/lhj/codes/Heformation` 接手 Heformation。先读 `AGENT/AGENT.md`、`docs/requirements/server715-project-handoff-20261003.md`、`context/02_current_status.md` 和 `context/15_handoff.md` 的最新置顶，以及 WORKLOG 最新条目。我们的部门只做任务规划、资源调度、路径／编队与跨介质控制及执行报告，不开发专业识别或精细重建。总体架构冻结，A/B/C 共用原 Plan、Action 和唯一任务权威。r67 的 B 监测闭环和管件2动态复查已完成，但新完整 C 因 WATER AAV 与 USV 近距失败，完整 B/C 动态返航总验收尚未完成；旧世界已退出、bag闭合，失败锁和未验证停止保留。优先用 `aav-usv-encounter-window.json` 和原源代码修复 WATER／PVS 本地导航缺动态成员输入的问题，核对支援进场与巡视／退出运动，然后完成完整 C 和同会话排队／替换／驻留／人工 Swarm 编队返航、返航中换任务及最终 HOME5/5零锁/end_session。保留原物理模型和已经声明的实际安全／GoalID／进度／实收规则，不靠降低阈值、固定身份、全队串行或伪造运动完成。只处理阻塞业务链的真实问题，不开展无关测试或新框架。每次源码和运行都在 WORKLOG 顶部追加真实事实并同步交接；只推 main，实验数据不推送。

