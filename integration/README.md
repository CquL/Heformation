# 集成代码

这里保存本项目自己的薄适配和平台接入代码。

- `qn_aav_simulator/`：接收上游 `quadrotor_msgs/PositionCommand`，执行 qn 原RBF/PD、
  十路执行器和六自由度动力学，发布 `nav_msgs/Odometry`。
- `swarm_qn_bridge/`：构建 qn Docker 镜像时覆盖官方仿真入口，将`poscmd_2_odom`替换为
  qn 节点。上游源码本身保持独立。
