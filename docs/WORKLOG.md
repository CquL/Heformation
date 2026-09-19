# 工作日志（计划 / 实际 / 效果）

这个文件是项目的持续记忆：**每次改动仓库或跑实验，都在最上面追加一条**。目的是让下一次（人或 agent）
不看对话历史也能知道：当时计划做什么、实际做了什么、效果如何、哪里与计划不一致、下一步是什么。

维护约定：

- 只追加，不重写历史条目；旧的更正用新条目说明。
- 每条包含五个字段：计划 / 实际 / 效果 / 证据 / 未完成与下一步。
- 「实际」与「计划」不一致时，写在「实际」里并说明原因——这是最有价值的部分。
- 证据只写可复查的东西：提交号、实验目录、命令、测试数、指标。
- 不写没跑过的结果，不把"代码写了"写成"验证通过"。

---

## 2026-09-19 · 实时仪表盘（提交 `f044383`）

- **计划**：用户追问"没有实时可视化仿真吗 / 为什么不做实时可视化"。核对后发现：实时三维视图
  **已有**（`./scripts/docker_run_qn_demo.sh mission` 起 RViz），但**任务层**（分配、参考采用、
  判定、时间）没有任何实时视图——上一轮做的两张图都是事后离线渲染。本轮补实时任务层。
- **实际**：
  - 新增只读订阅节点 `mission_dashboard.py`：四面板（场景+槽位布局 / 状态与判定 / 槽位误差 /
    高度与最近接近），输出 matplotlib 窗口（`~window`）和 `~image/compressed` 的 JPEG，
    可给 RViz Image 显示或 `rqt_image_view`。只订阅，不参与控制链。
  - `formation_air.launch` 增加 `dashboard`（默认 false，**opt-in**）；`docker_run_qn_demo.sh`
    支持 `DASHBOARD=true`。
  - `Dockerfile.qn` 增加 `python3-tk`：基础镜像有 matplotlib 但没有 Tk 绑定，否则只能发图不能开窗。
  - `docker_run_qn_demo.sh` 原本只把 `$1` 传进容器，`DASHBOARD` 需要额外传参，一并改了。
  - **发现并修掉的两个真问题**：
    1. `live` 槽位误差一开始是"用当前中心回算整段历史"，于是每次派发都会让历史曲线整体跳变，
       看起来像误差突然出现。改成按每个样本自己的时刻查中心时间线——与离线图同语义。
       这个问题在真实运行里同样会出现，不只是测试环境的假象。
    2. 缺 `/formation_action_server/relative_slots` 时会静默退化成"所有成员槽位都是原点"，
       图上表现为一条平直的 4.0 m 误差线。现在会显式告警。
  - 另一个自己引入的 bug：场景标题插在了 `centre` 赋值之前 → `UnboundLocalError`，节点渲染一帧后
    就死。用 `rosbag play` 复放录制的 bag 才发现（首帧 78489 字节三连相同暴露了"只发了一帧"）。
- **验证方式**：不跑 8 分钟仿真，而是 `roscore + rosparam load formation_air.yaml +
  rosbag play --clock -r 3 <录制的 bag>` + 仪表盘，按消息时间戳抓帧。抓到 mission t=5.3 s 与
  t=20.1 s 两帧：场景/槽位六边形/`qn trajectory_id 4 (all adopted)`/槽位误差在驻留段收敛到
  `epsilon_p` 以下/最近接近在编队切换时收到 1.7 m，均与离线图一致。
- **效果**：实时视图与离线渲染在语义上对齐；167 项测试通过；离线两张图重跑无回归。
- **证据**：`docs/QN_INTEGRATION.md` 的 "Live views" 一节；
  `roslaunch qn_aav_simulator formation_air.launch run_mission:=true dashboard:=true`。
- **未完成与下一步**：仪表盘尚未在**真实实时任务**里跑过（只做了 bag 复放验证）——因为它与
  模型时钟门槛争 CPU，需要一次带 `dashboard:=true` 的真实运行来确认门槛不受影响；
  M2 第 3 步（原盒子场景定位）与第 4 步（局部扫描中断）仍未做。

---

## 2026-09-18 · 全链路可视化与动画回放（提交 `9fcb40f`）

- **计划**：用户要求"可视化仿真整个实验，从任务分配到编队控制"。已有的
  `plot_formation_experiment.py` 是按主题分三张图（轨迹 / 槽位 / 时间），没有一条把
  **分配 → 派发 → 飞行 → 控制 → 安全 → 时间**放在同一个时间轴上的视图，也没有运动回放。
- **实际**：新增两个只读渲染器 + 一个命令封装，全部只读 bag + `metrics.json` + `config.json`：
  - `plot_mission_overview.py`：一张 7 面板总览（任务甘特 / 分配路由 / XY 轨迹与槽位 /
    槽位误差 / 轨迹跟踪误差 / 高度与 AIR 域 / 净距 / 时间漂移）。
  - `animate_mission_replay.py`：GIF 回放，场景 + 槽位布局 + 障碍，右侧联动槽位误差与高度曲线。
  - `scripts/docker_render_figures.sh <实验目录> [fps]`：一条命令出两张图。
  - 两者加入 `CMakeLists.txt` 的 `catkin_install_python`，镜像已重建。
  - 写第一版时踩到的坑（都是"看图才发现"的语义错误，值得记下来）：
    1. `set_aspect("equal", adjustable="datalim")` 把 x 轴撑到 −44…−5，轨迹被压成细线；
       改用 `adjustable="box"` 并加高该行。
    2. **无障碍场景仍然画了盒子净距曲线**——`metrics["obstacle_center"]` 在 `obstacle off`
       时照样存在，必须用 `obstacle_scenario` 门控。否则图上会出现一条并不存在的负净距，
       正是计划里"盒子不存在要记为不适用，不是距离为零"的图形版本。
    3. 槽位误差在运输段天然很大（派发瞬间目标中心跳到新目标，误差瞬间到 ~4 m）。第一版把它
       当误差信号直接画会误导；改为标出驻留窗口并注明 `epsilon_p` 只在驻留段把关，另外单独画
       "对 qn 实际采用参考的跟踪误差"。
    4. `action_task_outcome` 是枚举整数，字符串判定必须取每任务 diagnostics 的 `verdict`。
    5. **PIL 7.0.0 的 GIF `optimize=True` 不做帧间差分**：实测 60 帧对照中 plain 与 optimize
       字节数完全相同，`disposal=2` 反而大 15 倍。第一版 71 MB。改成"共享调色板 + 关抖动 +
       快进 3× + scale 0.52"后 8 MB。帧数/分辨率/调色板是唯一有效的三个杠杆。
- **效果**：
  - `experiments/20260918-m2-baseline-shm` 出图成功，图面确认的事实：T1 晚 9.1 s、T2 晚 10.5 s、
    T3 晚 4.3 s；三次派发 `release_lag` 0.000/0.069/0.065 s；`plan_revision=3`、
    `updated_plan_used=true`、`dispatch_changed=false`（串行单资源下是预期结果）；
    三任务 `PASS/PASS/VALID`；最大模型-ROS 漂移 0.00064 s、跨机 0.00069 s。
  - 面板语义与 plan 的判定口径对齐：实际净距用实际状态判、参考余量单独评估；盒子不存在记
    `NOT_APPLICABLE`；`epsilon_p` 只在驻留段把关。
  - 167 项单元测试仍全部通过。
- **证据**：`experiments/20260918-m2-baseline-shm/{mission_overview.png,mission_replay.gif}`；
  `docs/figures/mission_overview.png`；命令 `./scripts/docker_render_figures.sh experiments/<run>`；
  `docs/QN_INTEGRATION.md` 的 "Whole-mission figure and animated replay" 一节。
- **未完成与下一步**：M2 第 3 步（原盒子场景定位，盒子保持 `(-23,0,0.5)` / `(1,1,1.2)` 不动）
  与第 4 步（局部扫描中断）仍未做；GIF 未入库（`experiments/*` 被 gitignore），需要时按上面的命令重出。

---

## 2026-09-18 · 定位累计时间偏差的根因：bag 写盘（提交 `9e64648`）

- **计划**：M2 第 2 步的停止条件是"无障碍基线必须通过时间、AIR 域、平面包络、机间净距与共同驻留"。
  基线三任务都过，但**累计模型-ROS 偏差 57 ms > 50 ms 门槛**，卡在这里；计划本身没有给出偏差的归因方法。
- **实际**：
  - 先量化：外层循环中位 10.00 ms、p99 10.07 ms、7308 步 / 73.1 s = 100.0 Hz——**速率没问题**；
    偏差是两次离散跳变（+6.4 s 处 −21.8 ms、+38.2 s 处 −6.1 ms），都发生在任务进行中，
    对应单步间隔 31.8 ms / 16.1 ms 的停顿。固定步长设计不补算，所以每次停顿永久留在偏差里。
  - 排除主机 CPU：32 核、load 1.4、无其他容器在跑。
  - 隔离实验：同一任务、同一参数，`record:=false` 不录 bag → 偏差 **0.0035 s**、速率 0.99999991、门槛通过。
  - 验证修法：照常录制但把实验目录放到 tmpfs（`/dev/shm`）→ 偏差 **0.000636 s**、速率 0.99999996、
    门槛通过，三任务 `PASS/PASS/VALID`。
- **效果**：找到了根因——不是仿真、控制器或宿主 CPU，而是 bag 写 home 盘时的间歇写回停顿。
  录制改到内存盘后第 2 步的停止条件满足，第 3、4 步解锁；代价是暂存目录不持久，运行结束需要把文件
  搬到实验目录（启动脚本已改为自动暂存 + 搬移，不改变实验目录布局）。
- **证据**：`experiments/20260918-m2-baseline-clean`（写盘，57 ms）、
  `experiments/20260918-m2-baseline-clean-shm`（tmpfs，0.64 ms；bag 已随实验目录保留）、
  不录 bag 的隔离运行 `/tmp/norecord/metrics.json`；脚本改动见 `scripts/docker_test_qn_formation_action.sh`。

---

## 2026-09-18 · M2 第 1–2 步：单一地图来源、几何判据分离（提交 `31c1aab`）

- **计划**：按冻结版《M2 定位与修复实施基线》第 1 步修地图与几何语义，第 2 步按新规则重评无障碍基线；
  基础条件不过就停，不进入盒子避障归因。
- **实际**：
  - 场景发布器独占 `/scene/global_cloud`，launch 一处配置展开给渲染器 / Action / readiness / 验证器；
    `obstacle:=off` 只移除盒子。
  - 排查发现上游 CPU 渲染器**在全局地图为空时崩溃**（对空点云做体素化再建 k-d 树），此前所有运行的
    "本地点云零消息"就是它，不是"环境为空"。加构建期补丁：空地图=已初始化且障碍集为空；
    一次有效扫描无返回点时发布带时间戳的空点云，帧改 `world`。
  - 盒子点云改为与实体几何同源，精确覆盖 `[-size/2, +size/2]`（原来 `range(round(size/res))`
    漏掉 +x/+y/+z 面）。
  - 判据分离：实际 `d_actual ≥ d_required`；参考 `d_ref ≥ d_required + e_budget`（预算运行前声明 0.30 m）；
    半径只减一次；无盒子记 `NOT_APPLICABLE`；平面按全程最小高度判定；违规即时锁存。
  - 与计划不一致的地方：计划写"先压参考、不够再提高度"，但实测 1.5 m/s 与 1.0 m/s 的最低高度都是
    0.237 m，证明**不是参考强度问题**，于是直接走了提高度的路径，并加了第三个构建期补丁
    （上游 replan FSM 原本硬编码编队高度 0.5，现读取目标 z），场景巡航高度声明为 0.8 m。
- **效果**：`experiments/20260918-m2-baseline-clean` 三任务全部 `PASS / PASS / VALID`，最低高度
  0.380/0.502/0.426 m，`obstacle_check=NOT_APPLICABLE`，基线 30.20 s；但**累计时间门槛不过**：
  `model/ROS rate` 0.99928 → 72.8 s 累计偏差 57 ms > 50 ms。按停止规则不进入盒子场景，未调盒子或权重。
- **证据**：提交 `31c1aab`；`evidence/m2-baseline-clean`、`evidence/m2-baseline-plane-violation`、
  `evidence/m2-baseline-slow-reference`；167 项单元测试通过。

## 2026-09-18 · RViz 演示 master 修补（提交 `5961317`）

- **计划**：用户报告关掉 RViz 后终端一直刷 `XmlRpcClient::writeRequest: Connection refused`，要求修补。
- **实际**：定位为演示脚本先启动 `rviz.launch`，而该 launch 拥有 rosmaster，且 rviz 节点 `required="true"`：
  关窗即结束 launch、master 随之消失，仍在跑的七机节点刷错。改为脚本自己启动并持有 roscore，RViz 作为
  旁路进程，退出时一起收掉。
- **效果**：实测杀掉 rviz 进程后 master 与仿真继续（39 节点、`/drone_0_qn/odometry` 99.9 Hz），
  日志 0 条连接错误。
- **证据**：提交 `5961317`；容器内实测记录见提交信息。

## 2026-09-18 · 测试命令收敛与历史文档归档（提交 `436600e`、`2ed2765`）

- **计划**：把散在四份文档里的测试命令集中到权威文档；历史审查文档入 `docs/reviews/` 并标注对应提交，
  个人笔记不入库。
- **实际**：`docs/QN_INTEGRATION.md` 增加 Tests 一节（单元 / 冒烟 / 端到端三层 + 分析工具 + 失联注入）；
  审查文档移到 `docs/reviews/source-review-85a5463.md` 并加历史声明横幅。
- **效果**：命令可直接复制执行；历史结论与当前状态不再混淆。
- **证据**：提交 `436600e`、`2ed2765`。

## 2026-09-18 · M1 正确性修复 + M3 路由与离线算例 + M2 首版场景（提交 `8362db8`）

- **计划**：按冻结的《Heformation 框架验证实施基线》修坐标 / 时间 / AIR 域 / 样本对齐 / 执行单元状态。
- **实际**：
  - 坐标：实测表明模型**姿态与角速度是物理量**（前飞加速低头、机体角速度匹配 `R(q)ᵀdR/dt`），
    不自洽的是机体线速度；因此实现为"保留姿态与角速度、发布 `R(q)ᵀv_world`"，而非计划字面写的
    反射四元数方案。
  - 时间：恢复固定外层步长，模型时钟自驱；命令步边界采用 + 有界缓存。
  - AIR 域由 `hg_m/2` 派生并在线锁存；净距改表面净距；采用门槛先进驻留；未被接受的任务不释放资源。
  - 样本台账期望网格独立生成、按状态时间戳匹配。
  - M3：静态执行单元路由 + 每单元独立位置/可用时刻；离线五平台三类算例。
- **效果**：`experiments/20260918-mission-m1` 242 项检查 0 失败，三任务 `PASS/PASS/VALID`；
  用新评估器复评三个旧 bag，全部 `air_domain_ok=false`（旧结论失效被正确判出）。
- **证据**：提交 `8362db8`；`evidence/m1-seven-aav-online`、`evidence/reevaluation-*`。

## 2026-09-18 · 运行可视化与坐标问题暴露（提交 `bd04c1b`）

- **计划**：给出可复查的运行图（轨迹、槽位误差、时间对齐）。
- **实际**：新增 `plot_formation_experiment.py`，每个运行出三张图；把时间对齐图改成"门槛视图 + 去偏置视图"，
  否则亚毫秒信号在门槛尺度上不可见。
- **效果**：图直接暴露 `mission-g-fault` 中 drone_3 曾降到 z=−1.276 m（穿地）而任务仍判 PASS——
  由此引出 AIR 域检查缺失，成为后续 M1 的输入。
- **证据**：提交 `bd04c1b`；`docs/figures/`；`context/15_handoff.md` 的未解决清单。

## 2026-09-18 · P0–P3 主链落地（提交 `d8b0fc2`）

- **计划**：按 `plan.md` 的 P0–P3 实施七机 AIR 任务闭环。
- **实际**：双 Odometry 语义、参考遥测、trajectory_id 归属、模型时间门槛、资源状态机、三个独立判定、
  样本台账、A→B→Return + 幂等 DelayEvent、离线执行单元层；一次回归（窗口样本当单样本用）导致执行崩溃，
  修好后重跑。
- **效果**：三次 ROS 运行（repair on / repair off / 失联注入）各有记录；128 项单元测试通过。
- **证据**：提交 `d8b0fc2`；`experiments/20260918-mission-e|f|g-fault`。

## 计划演进（为什么现在是这个方案）

- 起点是 `plan.md` 的 P0–P3（接口/时间语义修复 + 一项感知避障场景 + 离线多资源）。
- 两轮外部审阅把范围收敛为：先修"已接通链路是否在表达同一个物理过程"，再谈感知避障与资源调度；
  明确不做参数扫描、不换算法、不为通过而缩小障碍。
- M2 从"跑一个障碍场景"进一步收敛为"先修地图输入语义，再用时间线定位失败层"，
  并规定：定位完成 ≠ 避障通过。
- 每次范围变化都记在 `context/14_decisions_and_unknowns.md`，结果记在本文件与 `context/15_handoff.md`。
