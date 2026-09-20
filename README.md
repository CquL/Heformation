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

在有桌面 `DISPLAY` 的终端运行（需要 Docker、`fonts-noto-cjk` 中文字体，以及
`swarm-formation-qn:five-finite-wire` 镜像）：

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
它尚不包含完整协同监测、有限通信交付和条件复查。最新港口运行虽然各动作返回成功，
全程时间一致性审计仍未通过；不能将画面正常或单个动作成功当作整场验收成功。
详见 [实时入口与实验记录](docs/reviews/five-live-view-20260920.md)。

首次使用、缺少上述镜像时，在仓库根目录构建：

```bash
docker build -t swarm-formation-upstream:noetic upstream/Swarm-Formation
docker build -f docker/Dockerfile.qn -t swarm-formation-qn:five-finite-wire .
```

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
