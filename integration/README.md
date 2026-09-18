# 集成代码

这里保存本项目自己的薄适配和平台接入代码。

- `qn_aav_simulator/`：接收上游 `quadrotor_msgs/PositionCommand`，执行 qn 原RBF/PD、
  十路执行器和六自由度动力学，发布 `nav_msgs/Odometry`。
- `mrta_python/`：Calvo v9 restricted-domain Python port，负责第一阶段任务排序、
  固定七机联盟计划和正延迟时间修复；原始 MATLAB 版本保留在 `upstream/Calvo-MRTA/`。
- `qn_aav_simulator/action/Formation.action`：七机 AIR 编队低频任务边界；
  `scripts/formation_action_server.py` 只发布已有 Swarm 目标并读取 qn Odometry。

基础检查：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  integration/qn_aav_simulator/tests integration/mrta_python/tests
./scripts/docker_build_qn.sh
./scripts/docker_test_qn_formation_action.sh mission 1.5 <空的实验目录> on
```

`qn_aav_simulator/scripts/formation_mission_runner.py` 依次派发 `A -> B -> Return`，
把每次动作的 GoalID、trajectory_id 集合、计划时间与实际完成写进 `metrics.json`；
`verify_formation_experiment.py` 用 rosbag、逐动作诊断和指标做独立复核，产物为实验目录里的
`verification.json`。两者都不修改 Swarm 上游或 qn 控制器。
- `swarm_qn_bridge/`：构建 qn Docker 镜像时覆盖官方仿真入口，将`poscmd_2_odom`替换为
  qn 节点，并把 `~odometry_swarm_compat` 重映射到 Swarm 规划器实际订阅的话题。
  上游源码本身保持独立。
