# Heformation

空中两栖无人机（AAV）、无人船（USV）与潜航器（UUV）的协同任务研究工程。现有执行链使用 Swarm-Formation 空中规划、qn AAV 动力学、PVS 水面／水下后端、ROS 1 Noetic、Docker 和 RViz；任务层在 `integration/mrta_python`，执行与场景在 `integration/qn_aav_simulator`。

## 当前远域场景

[场景配置](integration/qn_aav_simulator/config/five_scene_offshore.yaml)把三台 AAV、USV、UUV 和母船放在同一岸边部署区；[任务请求](integration/qn_aav_simulator/config/monitoring_request_offshore.yaml)声明障碍通道另一侧的空中与水下观测、必要结果接收及返回。RViz 显示同一 ROS 会话中的实际平台状态。坐标以米计，是现有模型的缩比任务场景，不能解释为真实数公里航程或设备通信性能。

**当前状态：场景布局已实时显示，完整远域协同任务尚未通过。** 原生联合规划在 180 秒诊断预算内没有产生可派发的完整计划，暴露了运动净距、候选时域和轨迹采样连续性问题；因此新场景没有“三类平台一起完成任务”的实跑证据。规划期间平台待命，画面不动属于当前状态，不能把初始布局截图当作完成效果。详见[当前状态](context/02_current_status.md)和[工作日志](docs/WORKLOG.md)。

在桌面终端打开实时场景与任务入口：

```bash
cd /home/lhj/Swarm-Formation
JOINT_GPU_RENDER=true bash scripts/docker_run_three_class_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-offshore-live"
```

脚本打印完整可行计划后才会询问 `yes`；没有完整计划就不会派发 Goal。GPU 开关仅影响 RViz 渲染，不加速联合求解或动力学。该脚本使用本机 CPU 核组，并临时处理已知宿主时间回拨；需要 Docker、Noto CJK 字体和本机非交互式 `sudo`。其他机器可直接使用 `scripts/docker_run_joint_request.sh` 并按自身环境设置核组。

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
