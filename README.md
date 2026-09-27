# Heformation

空中两栖无人机（AAV）、无人船（USV）与潜航器（UUV）的协同任务研究工程。当前固定场景使用 Swarm-Formation 空中规划、三台 AAV 的 qn 跨介质模型、一台固定 WATER 的 qn 水下 UUV 代理，以及 Otter/PVS 水面艇；运行环境为 ROS 1 Noetic、Docker 和 RViz。UUV 代理不是 REMUS100 水动力学验证。任务层在 `integration/mrta_python`，执行与场景在 `integration/qn_aav_simulator`。

## 当前远域场景

[场景配置](integration/qn_aav_simulator/config/five_scene_offshore.yaml)把三台 AAV、USV、UUV 和母船放在同一岸边部署区；[任务请求](integration/qn_aav_simulator/config/monitoring_request_offshore.yaml)声明障碍通道另一侧的空中与水下观测、必要结果接收及返回。RViz 显示同一 ROS 会话中的实际平台状态。坐标以米计，是现有模型的缩比任务场景，不能解释为真实数公里航程或设备通信性能。

**当前状态：从目标区共同返航的正常请求已实跑通过。** [同次运行](experiments/20260927-target-return-live-r3)在默认10秒预算内约1.34秒选择完整协同Plan：空中AAV完成四位置扫测，另一AAV完成AIR→WATER→AIR浅水点测，UUV完成三个深水点，USV在目标区外侧会合并接力。先完成的平台保持在目标区；八份必要结果实收、四方实际终态就绪后，统一释放全部返航后缀。四个首返航Goal发送时间差约0.025秒，四活动及规定返回全部成功，资源锁为空。[返航事件审计](experiments/20260927-target-return-live-r3/joint-return-audit.json)和[同次安全/时间审计](experiments/20260927-target-return-live-r3/safety-audit.json)均通过，最小代理净距约0.805m（门槛0.5m）。

你会看到两台作业AAV在目标区等UUV完成，再与USV一起开始返航。共同开始返航不等于同时到家；实际控制响应和各平台速度不同。本次AIR扫测AAV运动速度中位数约0.386m/s、UUV约0.130m/s，约3倍。空中返航走场景声明的2.5m高度以避开共同返程中的交叉；UUV慢速作业与返航分别使用230秒和185秒定时参考，各自本地终态由原误差/速度判据确认，有限观察上限350秒。

USV仍由Otter/PVS航点与LOS引导执行。求解器从有限接触位置中校核UUV目标区终端、USV和母船的距离；当前同步撤收政策下选择目标区外侧位置，近岸位置因不能在那里与UUV会合而被拒绝。UUV产品须实际接近USV才由船接力，AAV在AIR且进入母船范围后直接交付。8m/30m只是声明的接触几何，观测是几何足迹/驻留代理，均不是设备性能认证。计划时刻是估计；最终完成读取实际Result和接收事件。

本次验收覆盖正常任务的目标区等待和共同返航。旧缺测策略仍是首轮返岸后再派一次补测，未宣称实现“目标区内补测后全员一起撤收”。细节与失败记录见[WORKLOG](docs/WORKLOG.md)。

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
