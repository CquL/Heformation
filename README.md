# Heformation

海上异构无人集群项目的上游复现与平台集成工作区。

当前目录采用以下边界：

```text
upstream/Swarm-Formation
    官方 Swarm-Formation 源码与原始 ROS 1 catkin 包

integration/qn_aav_simulator
    qn.slx 对应的 Python 控制器、六自由度动力学和 ROS 适配器

integration/swarm_qn_bridge
    将官方 fake drone 替换为 qn 节点的 launch overlay

models/qn/qn.slx
    原始 qn Simulink 模型
```

当前上游运动基线来自 [ZJU-FAST-Lab/Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation)，
当前集成 Swarm AIR 规划、qn AAV 动力学、PVS 水面/水下执行端与任务调度。
五平台完整协同任务仍在实施，当前事实与未完成项见 [当前状态](context/02_current_status.md)。

## 五平台实时可视化仿真

### 两区域联合请求：AIR、USV支援与AAV跨介质（资格诊断）

在有桌面 `DISPLAY` 的终端运行，查看具体计划后输入 `yes` 才派发：

```bash
cd /home/lhj/Swarm-Formation
JOINT_PLANNING_BUDGET_S=360 JOINT_PLANNER_CPUSET=12-15 \
  JOINT_VIEW_CPUSET=24-31 JOINT_VISUALIZE=true \
  bash scripts/docker_run_joint_request.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-joint-live"
```

这条命令在同一个现有 MissionRunner 中加载[两区域请求](integration/qn_aav_simulator/config/monitoring_request_joint.yaml)，自动选择AIR成员、USV支援和合格跨介质方法；RViz显示同源港口障碍与实际平台，中文任务面板显示所选活动、实际Action、命令送达、母船收件和资源占用。**规划和等待确认期间平台只按本地待命参考运行，不开始作业运动。**关闭显示窗口不作为任务取消。输出目录保存任务结果、单条真实静态云`scene-once.bag`和五平台动态`execution.bag`，供独立审计。`360`秒是当前原生完整状态查询的**隔离诊断预算**，正式默认仍为`10`秒；`12-15`和`24-31`分别是本机规划与界面/录包进程的CPU核组，其他机器应按实际核组调整或省略，不能当算法参数。`JOINT_SIM_CPUSET`可在隔离诊断中另设动力学/Action核组，但它不是算法参数或时间保证。未找到完整候选时不派发。新增有限下行Goal门后的`joint-command-gate-live-r2`任务层完成两区域、10条命令送达和返回，**同次独立审计因模型时间偏差超原门槛而FAIL**；先前`joint-runner-live-audited-r1`是无该下行门版本的全项审计PASS。REMUS在这些普通请求中待命，异常复查和最终三类平台验收尚未通过。项目原生模型内部状态目前未从运行节点重建，此入口明确为声明初态资格仿真；实跑证据边界见[当前状态](context/02_current_status.md)和WORKLOG。

### 水下协作历史入口：潜航器观测、无人船支援

```bash
cd /home/lhj/Swarm-Formation
VISUALIZE=true bash scripts/docker_probe_five_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-water-live" cooperative
```

沿用现有 `swarm-formation-qn:cooperation` 消息镜像，挂载当前 Python 实现。
场景使用港口障碍配置；启动后自动比较有限候选，终端和中文面板展示所选计划。
**看到具体计划后，在启动终端输入 `yes` 才派发；其他输入不执行。**
本轮潜航器执行水下样点观测，无人船按选定路线支援，三台 AAV 待命。
母船是固定接收端：结果受距离、遮挡和有限链路容量影响，实际接收后才增加任务交付覆盖。
母船没有自主航行动力学，也未参与位置优化。

中文面板分别显示任务权威状态和独立传输过程（无人船／母船接收 KiB），RViz 母船标签随接收事件更新。
该入口验证当前**水下协作阶段**，不是完整三类平台任务；空中任务、跨介质选择、复查及规定返回仍未全部接通。
关闭 RViz 或面板不会结束执行；启动终端 `Ctrl+C` 结束整链并归档。
运行结束后画面保留，不会自动开始新一批任务。录包只覆盖确认后的执行区间，
待命和结果展示期间不持续录入静态点云。实跑结果与边界见
[水下协作可视化记录](docs/reviews/water-cooperation-live-20260921.md)。


### 新增：VRX官方海面环境试接

本机NVIDIA GPU环境下，首次构建后运行官方场景：

```bash
cd /home/lhj/Swarm-Formation
bash scripts/docker_build_vrx.sh
bash scripts/docker_run_vrx.sh native
```

尝试将现有五平台实际状态显示到VRX：

```bash
bash scripts/docker_run_vrx.sh five-view
```

`five-view`需要终端输入`yes`才运行固定动作，Ctrl+C结束。该试接保持qn/PVS动力学，
GPU负责Gazebo渲染；**VRX地形/海况尚未接入现有任务安全判定，也不是完整协同任务**。
依赖与实跑边界见[VRX试接记录](docs/reviews/vrx-integration-20260920.md)。

### 原RViz资格实验入口

在有桌面 `DISPLAY` 的终端运行（需要 Docker、`fonts-noto-cjk` 中文字体，以及
`swarm-formation-qn:cooperation` 镜像）：

```bash
cd /home/lhj/Swarm-Formation
VISUALIZE=true bash scripts/docker_probe_five_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-five-live"
```

1. RViz 和中文动作面板打开后，先查看终端打印的实验内容。
2. 在启动终端输入 `yes` 才派发本批动作；未确认时保持待命。
3. 在 RViz 用鼠标查看场景；Views 中可选“母船近景”或“跨介质作业区”。
4. 关闭显示窗口不会停止已启动的执行。在启动终端按 `Ctrl+C` 结束整链并归档记录。

默认显示三台 AAV、无人船和潜航器，以及岸壁、栈桥、礁石、母船模型和实际轨迹。
“安全几何（体素）”可显示实际使用的障碍地图。船模与岩石资源复用仓库内已有素材，启动时自动转换。

**当前运行的是固定动作资格实验**：空中转场、单台 AAV 垂直入水/水下短程/出水、USV/UUV 航行与终端验证。
它尚不包含完整协同监测、有限通信交付和条件复查。节拍修正后的港口无GUI运行
7项动作及独立全程时间审计通过（最大偏差约0.0257秒，门槛仍为0.05秒）；
此前带显示的时间失败记录保留，新的带显示全链验证尚待完成。
详见 [实时入口与实验记录](docs/reviews/five-live-view-20260920.md)。

首次使用、缺少上述镜像时，在仓库根目录构建：

```bash
docker build -t swarm-formation-upstream:noetic upstream/Swarm-Formation
docker build -f docker/Dockerfile.qn -t swarm-formation-qn:cooperation .
```

`PlatformTask` 已增加预装载、选定观测点ID字段与幂等启动服务；当前开发中的
AIR 有限交付还为 `Formation.action` Goal 增加本次观测点 ID。使用这些路径时，
客户端和 Action 服务端必须来自同一次 Noetic 消息构建，不要混用旧镜像。
已有水下协作命令仍是单阶段入口，不能将 AIR 探针或其新镜像当作完整联合任务验收。

## 七机兼容与回归入口

官方工程使用 ROS 1 Noetic。Ubuntu 24.04 主机通过 Docker 运行：

```bash
cd ~/Swarm-Formation
./scripts/docker_build_qn.sh
./scripts/docker_test_qn_single.sh
./scripts/docker_test_qn_swarm.sh
./scripts/docker_run_qn_demo.sh
```

七机入口保留为历史兼容与回归，不代表最终的三 AAV＋一 USV＋一 UUV 编成。

## 目录

```text
AGENT/       AI项目规则与协作入口
context/     项目背景、架构、接口、文献、计划和交接
prompt/      常用工作提示词
upstream/    按项目分开的上游源码
integration/ 我们的薄适配与平台接入
models/      qn.slx等模型文件
scenarios/   空中、水下、跨介质和任务场景
config/      平台、任务、编队、规划器和RViz配置
scripts/     固化环境、Docker、运行和测试脚本
tests/       可重复测试与实验入口
data/        原始数据、处理数据、轨迹和结果
experiments/ 实验配置与结果索引
docs/        技术接入与复现说明
```

完整背景和下一步工作见 [AGENT/AGENT.md](AGENT/AGENT.md)、[context/README.md](context/README.md)
和 [docs/QN_INTEGRATION.md](docs/QN_INTEGRATION.md)。
