# Heformation

空中两栖无人机（AAV）、无人船（USV）与潜航器（UUV）的协同任务研究工程。当前固定场景使用 Swarm-Formation 空中规划、三台 AAV 的 qn 跨介质模型、一台固定 WATER 的 qn 水下 UUV 代理，以及 Otter/PVS 水面艇；运行环境为 ROS 1 Noetic、Docker 和 RViz。UUV 代理不是 REMUS100 水动力学验证。任务层在 `integration/mrta_python`，执行与场景在 `integration/qn_aav_simulator`。

## 当前远域场景

**2026-10-01：同一环境已加入三业务场景。** 母船附近仍部署3台AAV、1艘USV、1台UUV；现有调查区之外，加入两座风机及水下基础、作业平台及海底管路（三处阀件）、港口建筑、道路、灯塔、码头和树木。实际[全景](docs/images/three-business-scene-overview.png)、[风机](docs/images/three-business-scene-wind.png)、[平台/管路](docs/images/three-business-scene-platform.png)、[部署区](docs/images/three-business-scene-deployment.png)、[同次UI](docs/images/three-business-scene-ui.png)来自同一ROS运行或读取其实际状态的原Qt组件。

主命令沿用下方入口。资产准备根据场景的 `world_models` 生成COLLADA资源及 `visual-assets/expanded-scene.yaml`，UI、感知和净距共用此展开几何；有/无界面时一致。新增主要结构使用同组件尺寸生成的保守盒体，精细显示网格不是精确曲面碰撞模型。原Swarm/qn/PVS、任务权威、GoalID及安全门槛保留；设施 AIR 工作域已接入声明的三维高度，地图顶界为13m。

RViz“Views”提供全景、风机、平台/管路与部署近景。实际点云显示默认隐藏，可手动开启，感知/导航仍持续接收。UI显示同一区域及设施示意。**2026-10-02：B/C 请求、联合分配、设施 AIR/WATER 作业与实际区间进度已接入原链路，完整 B/C 及同会话动态验收仍未通过。** 当前可加载[风机模板](integration/qn_aav_simulator/config/monitoring_request_wind.yaml)和[平台／管路模板](integration/qn_aav_simulator/config/monitoring_request_platform_pipeline.yaml)。实际失败与剩余工作记录于[WORKLOG](docs/WORKLOG.md)，不能把模板预览或部分进度当成完整业务完成。

[场景配置](integration/qn_aav_simulator/config/five_scene_offshore.yaml)把三台 AAV、USV、UUV 和母船放在同一岸边部署区；[任务请求](integration/qn_aav_simulator/config/monitoring_request_offshore.yaml)声明障碍通道另一侧的空中与水下观测、必要结果接收及返回。RViz 显示同一 ROS 会话中的实际平台状态。坐标以米计，是现有模型的缩比任务场景，不能解释为真实数公里航程或设备通信性能。

实时入口采用**独立任务控制台＋独立RViz**。控制台按设计图保留顶部任务阶段、左侧任务/分工、右侧作业/支援/收件/事件、底部并行时间线；中央为二维区域与方案切换，不嵌入三维画面。在中央地图按下鼠标确定圆心、拖动确定半径，再点击“生成协同方案”；RViz随后在独立窗口显示同一场景。方案生成后，在UI核对实际分工并点击“确认并执行”，无需到终端输入yes。关闭控制台不结束已确认任务；“停止任务”须在UI确认，交给原runner取消处置，不将点击停止当作平台已停稳或资源已释放。

运行中点击“圈选新任务”，可选择“排队执行”或“立即替换”，生成预览后再确认。排队保留当前监测，必要结果收齐后从实际位置接续；替换先取消匹配的旧 Goal 并验证安全停止，再按实际状态采用新 Plan。监测结束后平台在目标区驻留，等待下一任务或“共同返航”命令；返航途中仍可圈选和替换。返回部署区后会话继续，点击“结束会话”才退出。

共同返航使用已有三机 Swarm 编队端点：三台 AAV 在共同高度集结到原队形槽位，然后共同转场、在部署区下降；USV/UUV 同时按各自执行端返回。没有按成员设置不同返航高度。组级预订占用三台真实 AAV，途中替换须三成员实际停止，不能把编队当成额外机器人。

用户圈选目标区域，任务层快速选择成员、跨介质方法、共享支援与同步关系。`ONLINE_MAPPING` 只把区域作业和导航意图交给原 Action；执行端用实际位姿生成的局部测距更新自由／占据／未知地图，在线选择下一观察短段。初始不知道的障碍不会被用于提前删除目标或算好全程绕行路线。Swarm 的目标引导和局部轨迹优化保留；qn WATER 与 Otter 在原节点中更新参考。

**2026-09-30：动态协同链已完成同次实时实跑。** r42 在一个持续仿真会话中完成运动中替换、排队区域接续、三轮监测成果实收、目标区驻留15秒、人工共同返航、实际 Swarm 编队转场途中换区、最后区域扫描与再次共同返航。三台同型 AAV 分别完成一次入水／扫描／出水；每轮33份成果实收。最终五平台均实际返回、零资源锁，任务权威 `PASS_SAMPLED_MAPPING`，明确结束会话后为 `SESSION_ENDED/HOME`；驱动记录 `PASS_SESSION_SEQUENCE`。各次方案预览约0.44–0.49秒，完整会话约31分49秒。见[简要运行事实](docs/reviews/dynamic-session-result-20260930.json)、[实际HOME面板](docs/images/dynamic-session-home.png)和[实际编队返航RViz](docs/images/dynamic-swarm-return.png)。完整原始记录本地保留在 `experiments/20260930-formation-session-live-r42`；驱动使用UI同一命令接口，两窗口显示实际ROS状态，非轨迹回放。

本次独立核对的可对齐样本中，五平台最小净距0.746672m≥0.5m、USV/pier最小余量0.286077m≥0.2m，无无效原生轨迹或暂停期间普通参考。**严格时间审计仍FAIL**：最大累计模型/ROS漂移46.451202秒（UUV），14个位置对齐缺样。当前完成结论限用户已授权的可视化政策与本组有限请求，不是严格时钟、连续安全或任意环境保证。

**历史单批基线（2026-09-28）**：r17 圆心 `(2,4)`、半径 `2m`，约0.398秒输出联合方案，33份成果实收、四返回、零锁。它只证明当时单批请求，不代替上述动态会话验收；[终态区域视图](docs/images/online-mapping-terminal.png)是当时完成记录的Qt重绘。

当前量化成果为**声明深度切片的采样占据建图＋实际三维命中点云**：每个报告格内全部 0.25 m 细格实测才完成，按采集模式区分 AIR/WATER；不是完整未知三维表面重建证明。全向 5 m、6°角采样是明确的几何传感实验模型，不是设备参数或真实声呐模型。障碍命中是建图成果，遮挡后保持未知；先验部署自由空间不计为测量。Qt 只显示已知政策与实际命中，RViz 的完整环境模型明确标为仿真真值参照。

USV 局部指令在完整实际 Otter 状态副本上校核短命令与原生减速尾段，保留原控制器／执行器；无路时不再朝未知业务目标转向。查询到实际生效时刻之间的 coast 也纳入预测，并核对完整状态后才采用。依据与实际失败修复见[专项调研](docs/research/area-sensing-coverage-review-20260928.md)、[协同设计依据](docs/requirements/cooperation-source-design-20260920.md)和[WORKLOG](docs/WORKLOG.md)。

通信仍采用任务级共享支援：USV 到位并与 UUV 实际进入 8 m 声明范围后接收；AIR 使用 30 m 声明范围直达或共享 USV。必要结果收到且各方实际作业终态成立后，才共同释放返航。控制继续使用 ROS，未增加逐包协议、第二调度器或任务状态服务。

历史r17独立bag核对的最小成员净距为0.535626m，其严格时间审计也未通过。当前建图链的明确缺测触发后自动补测、完整三维表面完成及任意选区保证仍未验收；运行中改变区域的排队／替换已由r42实际验证。

在桌面终端打开实时场景与任务入口：

```bash
cd /home/lhj/Swarm-Formation
JOINT_GPU_RENDER=true JOINT_VISUAL_TIMING_RELAX=true \
  bash scripts/docker_run_three_class_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-offshore-live"
```

主入口默认 `JOINT_SIM_SPEED=2.0`，可显式设置1～4，例如在上述命令前加 `JOINT_SIM_SPEED=3`。2倍时一秒现实时间对应两秒 ROS 仿真时间，Qt 显示倍速与仿真用时。qn、Otter/PVS、Swarm、感知与任务使用同一个时钟；模型方程、积分步长和各平台物理速度比例保持不变。实际模型推进可能受计算负载限制，倍速设置不是模型持续达到该速率的证明，运行记录保留模型时间和时钟偏差。

本机桌面入口的仿真核组默认扩大为 `0-23`，求解使用 `16-19`，界面使用 `24-31`，避免把主要运行过程限制在六个物理核的兄弟线程上。可分别用 `JOINT_SIM_CPUSET`、`JOINT_PLANNER_CPUSET` 和 `JOINT_VIEW_CPUSET` 覆盖；其他硬件使用下述通用入口并按自身CPU设置。

默认共享规划预算为10秒。命令中的 `JOINT_VISUAL_TIMING_RELAX=true` 按用户已授权的可视化政策，仅允许时间审计不合格时在真实终态／采用／安全条件通过后继续；严格时间报告保留原结论，不能据此宣称时间资格通过。UI显示的是原runner同一份Plan与实际结果；预计时间不驱动完成判定，实际模式/支援状态在未收到或过期时显示未知。可选 `JOINT_GPU_RENDER=true` 只影响RViz渲染。这个入口使用本机CPU核组、Docker和Noto CJK字体，本机时钟处理需要非交互式sudo。其他机器可使用 `scripts/docker_run_joint_request.sh` 并按自身环境设置核组。

默认使用圈区和连续会话主入口（`JOINT_CONTINUOUS_SESSION=true`）。关闭圈区的非图形复现必须通过 `JOINT_REQUEST_FILE` 和 `JOINT_SCENE_FILE` 显式提供同次已生成的请求与场景（包含部署先验），不能直接把基础 BOX 配置当作本次圆区建图验收。UI 初始圆是选择建议，不代表所有区域都已验证。可在一次运行中继续选择区域，任务事实、成果和 GoalID 按请求隔离，已感知的导航知识保留。

缺少当前镜像时构建：

```bash
docker build -t swarm-formation-upstream:noetic upstream/Swarm-Formation
docker build -f docker/Dockerfile.qn -t swarm-formation-qn:joint-wip .
```

旧港口实验的大型本地产物和独立 VRX/Gazebo 试接已按用户要求删除。现有 Swarm、qn、PVS、任务求解与七机控制回归源码仍保留；旧港口结论不作为新远域任务验收。

## 目录

```text
AGENT/       项目规则
context/     当前状态、架构与交接
upstream/    仍在使用或研究参考的上游源码
integration/ 任务层、运动后端与 ROS 接入
models/      qn 原始模型
scripts/     Docker 构建及运行入口
docs/        实施记录与说明
experiments/ 当前场景的本地运行结果（不提交大型 bag）
```
