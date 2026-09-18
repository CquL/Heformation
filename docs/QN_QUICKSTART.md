# Swarm-Formation + qn 复现说明

本目录基于官方 Swarm-Formation 提交`967a4bdfae949e994691f8ffc87dbb0147cbebb7`。
规划器、编队代价、障碍代价、多机代价、轨迹优化、地图、感知、traj_server和消息定义均
保持官方源码。构建qn镜像时，`integration/swarm_qn_bridge/simulator.xml`覆盖容器内的
官方仿真入口，将直接复制状态的`poscmd_2_odom`替换为`qn_aav_simulator/qn_aav_node.py`。
官方源码目录`upstream/Swarm-Formation`保持原始版本。

当前链路：

```text
Swarm-Formation planner
-> fifth-order polynomial trajectory
-> PositionCommand(p, v, a, yaw, yaw_dot), 100 Hz
-> qn original RBF/PD
-> 10 first-order actuators
-> qn AIR/TRANSITION/WATER 6DOF
-> nav_msgs/Odometry
-> Swarm-Formation feedback
```

## 构建

宿主机是Ubuntu 24.04和ROS 2 Jazzy，官方工程是ROS 1 Noetic catkin，因此使用Docker：

```bash
cd ~/Swarm-Formation
./scripts/docker_build_qn.sh
```

构建得到：

```text
swarm-formation-upstream:noetic  官方18包对照镜像
swarm-formation-qn:noetic        增加qn后的19包镜像
```

## 测试

```bash
cd ~/Swarm-Formation
./scripts/docker_test_qn_single.sh
./scripts/docker_test_qn_swarm.sh
```

## 实时RViz

请从Ubuntu桌面终端执行，确保终端有`DISPLAY`：

```bash
cd ~/Swarm-Formation
./scripts/docker_run_qn_demo.sh
```

RViz打开后使用`2D Nav Goal`点击目标。官方`normal_hexagon.launch`实际创建`drone_0`至
`drone_6`，即中心节点加六边形节点，共7台qn AAV。

已验证点击中心目标`(20,0,0.5) m`后，7台实际qn Odometry全部到达各自中心加六边形
槽位，最终最大位置误差约`0.00062 m`。

## 当前边界

- 运行的是从`models/qn/qn.slx`解析并经过验证的Python公式，不需要MATLAB运行时。
- 正式配置为`ROUTE_POSITION + QN_ORIGINAL_POSITION + QN_ORIGINAL_RBF_PD`。
- `LOS_SURGE_YAW`没有启用。
- 当前先复现官方AIR场景；跨介质编队还需要把Swarm-Formation的速度、加速度可行性约束
  扩展为介质感知约束，不能只靠替换plant就宣称完成。
- 详细接口与验证见`docs/QN_INTEGRATION.md`。

## 七机编队动作实验（FormationAction）

```bash
cd ~/Swarm-Formation
./scripts/docker_test_qn_single.sh                 # 单机命令跟踪
./scripts/docker_test_qn_swarm.sh                  # 七机编队位置
./scripts/docker_test_qn_formation_action.sh mission 1.5 <空的实验目录> on
```

第三条命令运行七机 `A -> B -> Return` 任务链：启动隔离仿真、等待 Action server
进入 `READY_IDLE`、执行任务、再运行独立验证器，并把 `metrics.json`、`config.json`、
`execution.bag`、逐动作诊断和 `verification.json` 写进实验目录。第四个参数
`on|off` 只控制是否调用 `planRepair`，不会让 repair-off 跳过对真实动作完成的等待。

本次记录（`experiments/20260918-mission-e`，CPU 点云后端）：三个任务全部
`task_outcome=PASS`、`safety_outcome=PASS`、`experiment_validity=VALID`，模型时间驻留
5.02/5.01/5.03 s（要求 5.0 s），`verification.json` 220 项检查 0 失败；任务前基线为
30.1 s 连续七机对齐，`valid_sample_ratio=1.0`，模型-ROS 累计偏差 0.00054 s。
`experiments/20260918-mission-f` 是同一链路的 repair-off 对照。

边界未变：仍是 AIR 单介质、固定七机单一联盟，不代表跨介质或完整联合协调闭环。
