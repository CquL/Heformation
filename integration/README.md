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
python -m pytest -q integration/mrta_python/tests
./scripts/docker_build_qn.sh
./scripts/docker_test_qn_formation_action.sh
```
- `swarm_qn_bridge/`：构建 qn Docker 镜像时覆盖官方仿真入口，将`poscmd_2_odom`替换为
  qn 节点。上游源码本身保持独立。
