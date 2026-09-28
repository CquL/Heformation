# Heformation

空中两栖无人机（AAV）、无人船（USV）与潜航器（UUV）的协同任务研究工程。当前固定场景使用 Swarm-Formation 空中规划、三台 AAV 的 qn 跨介质模型、一台固定 WATER 的 qn 水下 UUV 代理，以及 Otter/PVS 水面艇；运行环境为 ROS 1 Noetic、Docker 和 RViz。UUV 代理不是 REMUS100 水动力学验证。任务层在 `integration/mrta_python`，执行与场景在 `integration/qn_aav_simulator`。

## 当前远域场景

[场景配置](integration/qn_aav_simulator/config/five_scene_offshore.yaml)把三台 AAV、USV、UUV 和母船放在同一岸边部署区；[任务请求](integration/qn_aav_simulator/config/monitoring_request_offshore.yaml)声明障碍通道另一侧的空中与水下观测、必要结果接收及返回。RViz 显示同一 ROS 会话中的实际平台状态。坐标以米计，是现有模型的缩比任务场景，不能解释为真实数公里航程或设备通信性能。

实时入口采用**独立任务控制台＋独立RViz**。控制台按设计图保留顶部任务阶段、左侧任务/分工、右侧作业/支援/收件/事件、底部并行时间线；中央为二维区域与方案切换，不嵌入三维画面。点击“编辑区域”或直接在中央地图按下鼠标、拖动半径，再点击“生成协同方案”；RViz随后在独立窗口显示同一场景。方案生成后，在UI核对实际分工并点击“确认并执行”，无需到终端输入yes。关闭控制台不结束已确认任务；“停止任务”须在UI确认，交给原runner取消处置，不将点击停止当作平台已停稳或资源已释放。

用户圈选目标区域，任务层快速选择成员、跨介质方法、共享支援与同步关系。`ONLINE_MAPPING` 只把区域作业和导航意图交给原 Action；执行端用实际位姿生成的局部测距更新自由／占据／未知地图，在线选择下一观察短段。初始不知道的障碍不会被用于提前删除目标或算好全程绕行路线。Swarm 的目标引导和局部轨迹优化保留；qn WATER 与 Otter 在原节点中更新参考。

**2026-09-28：当前有限采样建图请求已完成同次实时闭环。** r17 圆心 `(2,4)`、半径 `2m`，约 **0.398 秒**输出联合方案；AIR 区域观测、AAV 入水／水下观测／出水、UUV 区域观测与 USV 共享支援并行，**33 份成果实收后共同放行返航，四返回、零资源锁**。原任务权威 `PASS_SAMPLED_MAPPING`，16 个执行步骤验证通过，实际请求约12分13秒。见[任务记录](experiments/20260928-online-mapping-live-r17/metrics.json)、[事件核对](experiments/20260928-online-mapping-live-r17/task-event-check.json)和[终态区域视图](docs/images/online-mapping-terminal.png)（同次已完成记录的 Qt 重绘，原实时截图在该实验目录）。

当前量化成果为**声明深度切片的采样占据建图＋实际三维命中点云**：每个报告格内全部 0.25 m 细格实测才完成，按采集模式区分 AIR/WATER；不是完整未知三维表面重建证明。全向 5 m、6°角采样是明确的几何传感实验模型，不是设备参数或真实声呐模型。障碍命中是建图成果，遮挡后保持未知；先验部署自由空间不计为测量。Qt 只显示已知政策与实际命中，RViz 的完整环境模型明确标为仿真真值参照。

USV 局部指令在完整实际 Otter 状态副本上校核短命令与原生减速尾段，保留原控制器／执行器；无路时不再朝未知业务目标转向。查询到实际生效时刻之间的 coast 也纳入预测，并核对完整状态后才采用。依据与实际失败修复见[专项调研](docs/research/area-sensing-coverage-review-20260928.md)、[协同设计依据](docs/requirements/cooperation-source-design-20260920.md)和[WORKLOG](docs/WORKLOG.md)。

通信仍采用任务级共享支援：USV 到位并与 UUV 实际进入 8 m 声明范围后接收；AIR 使用 30 m 声明范围直达或共享 USV。必要结果收到且各方实际作业终态成立后，才共同释放返航。控制继续使用 ROS，未增加逐包协议、第二调度器或任务状态服务。

独立 bag 核对在可对齐样本中未发现净距越限，最小成员净距 **0.535626m ≥ 0.5m**。严格综合审计仍为 **FAIL**：模型／ROS 最大漂移0.210107秒，另有1个位置对齐缺样；原报告保留，按用户授权视觉政策运行，不能称严格时间／连续安全证明。当前新建图链的业务缺测再分配、完整三维表面完成和任意选区均未验收。

在桌面终端打开实时场景与任务入口：

```bash
cd /home/lhj/Swarm-Formation
JOINT_GPU_RENDER=true JOINT_VISUAL_TIMING_RELAX=true \
  bash scripts/docker_run_three_class_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-offshore-live"
```

默认共享规划预算为10秒。命令中的 `JOINT_VISUAL_TIMING_RELAX=true` 按用户已授权的可视化政策，仅允许时间审计不合格时在真实终态／采用／安全条件通过后继续；严格时间报告保留原结论，不能据此宣称时间资格通过。UI显示的是原runner同一份Plan与实际结果；预计时间不驱动完成判定，实际模式/支援状态在未收到或过期时显示未知。可选 `JOINT_GPU_RENDER=true` 只影响RViz渲染。这个入口使用本机CPU核组、Docker和Noto CJK字体，本机时钟处理需要非交互式sudo。其他机器可使用 `scripts/docker_run_joint_request.sh` 并按自身环境设置核组。

默认使用圈区主入口。关闭圈区的非图形复现必须通过 `JOINT_REQUEST_FILE` 和 `JOINT_SCENE_FILE` 显式提供同次已生成的请求与场景（包含部署先验），不能直接把基础 BOX 配置当作本次圆区建图验收。当前实跑选区为圆心 `(2, 4)`、半径 `2 m`；UI 初始圆是选择建议，不代表所有区域都已验证。一次运行只提交一个批次，已提交后若需更换区域，请退出该运行并使用新目录启动。

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
