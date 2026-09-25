# Heformation

空中两栖无人机（AAV）、无人船（USV）与潜航器（UUV）的协同任务研究工程。当前固定场景使用 Swarm-Formation 空中规划、三台 AAV 的 qn 跨介质模型、一台固定 WATER 的 qn 水下 UUV 代理，以及 Otter/PVS 水面艇；运行环境为 ROS 1 Noetic、Docker 和 RViz。UUV 代理不是 REMUS100 水动力学验证。任务层在 `integration/mrta_python`，执行与场景在 `integration/qn_aav_simulator`。

## 当前远域场景

[场景配置](integration/qn_aav_simulator/config/five_scene_offshore.yaml)把三台 AAV、USV、UUV 和母船放在同一岸边部署区；[任务请求](integration/qn_aav_simulator/config/monitoring_request_offshore.yaml)声明障碍通道另一侧的空中与水下观测、必要结果接收及返回。RViz 显示同一 ROS 会话中的实际平台状态。坐标以米计，是现有模型的缩比任务场景，不能解释为真实数公里航程或设备通信性能。

**当前状态：默认 10 秒预算下的实时三类协同联合监测区正常请求已通过。** [同次运行](experiments/20260925-joint-area-live-r20)约 1.15 秒选择空中 AAV、跨介质 AAV、qn 水下 UUV 和近岸 USV 的成员与时序，四项并行进入原 Action 链。AAV 完成空中四位置扫测，另一 AAV 完成 AIR→WATER→AIR 浅水点测，UUV 完成三位置深水通过段；八份结果实际到母船，参与成员均返回。同次 bag 的五平台时间、声明障碍和代理净距审计通过，最小代理净距约 1.37 m（门槛 0.5 m）；[接触核对](experiments/20260925-joint-area-live-r20/contact-audit.json)检查了本次 UUV–USV 和结果接收端距离。计划路线/时刻只是快速筛选估计；观测、接触范围分别是几何代理和声明式实验假设。[旧单点请求的一轮实际缺测复查](experiments/20260925-task-level-retest-r13)已通过，最终八结果模板的一轮复查正在验证。详见[当前状态](context/02_current_status.md)和[工作日志](docs/WORKLOG.md)。

USV 的运动由 Otter/PVS 航点及 LOS 引导执行，**不是 Swarm 空中规划器**。求解器从少量声明的接触机会按当前 USV、UUV 返回处和母船位置比较；本次选了近岸位置，前出南侧路线只是另一候选。UUV 产品在实际接近 USV 时由船接力，再由船进入母船范围后交付；AIR AAV 在母船范围内直接交付。实验接触阈值为 8 m/30 m，不代表设备指标。普通请求中平台可以先后返回；明确缺测时才追加一次备用 AAV 补测。

在桌面终端打开实时场景与任务入口：

```bash
cd /home/lhj/Swarm-Formation
bash scripts/docker_run_three_class_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-offshore-live"
```

脚本打印一份完整的任务级方案后才会询问 `yes`；没有方案就不会派发 Goal。默认共享规划预算为 10 秒；方案中的运动可行性由派发后的原生端点和实际反馈确认。可选 `JOINT_GPU_RENDER=true` 只影响 RViz 渲染，不加速联合求解或动力学。该入口使用本机 CPU 核组和中文实时面板；需要 Docker、Noto CJK 字体及本机非交互式 `sudo`。其他机器可直接使用 `scripts/docker_run_joint_request.sh` 并按自身环境设置核组。

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
