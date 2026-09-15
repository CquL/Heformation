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
本项目自己的代码只接在 `PositionCommand -> Odometry` 平台边界上。任务调度、海面/水下平台
和受限通信仍按 `context/` 中的 V2 计划推进。

## 环境与运行

官方工程使用 ROS 1 Noetic。Ubuntu 24.04 主机通过 Docker 运行：

```bash
cd ~/Swarm-Formation
./scripts/docker_build_qn.sh
./scripts/docker_test_qn_single.sh
./scripts/docker_test_qn_swarm.sh
./scripts/docker_run_qn_demo.sh
```

RViz启动后使用 `2D Nav Goal` 发布目标。当前官方 `normal_hexagon.launch` 创建
`drone_0` 至 `drone_6`，共7台 qn AAV。

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
