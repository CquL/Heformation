# Heformation P0–P2 实施基线修正版

## 总体原则

保留：

```text
Calvo v9 受限调度
→ FormationAction
→ Swarm-Formation
→ PositionCommand
→ qn 控制器与6DOF动力学
→ 实际 qn 状态
→ 组级完成判定
→ DelayEvent
→ planRepair
```

本轮不更换 Swarm，不修改 qn 控制器、动力学和消息定义，不增加新的控制层。重点修复三类语义问题：

```text
接口字段是否表达了正确坐标和物理意义
新任务参考是否真的被 qn 采用
模型时间是否与规划时间持续一致
```

## P0：状态、命令和时间语义

### 1. 分离标准 qn Odometry 和 Swarm 兼容输入

两种输出必须来自同一个 qn 状态快照，不建立两套状态计算。

**标准 qn Odometry：**

```text
header.frame_id = world
child_frame_id = drone_i/base_link
pose = 世界系位置和姿态
twist.linear = 机体系线速度
twist.angular = 机体系角速度
```

线速度由 qn 后端已有机体系结果提供；若某状态只有世界系表达，则在 ROS 发布边界完成坐标转换。不得修改控制器和动力学。

**Swarm 兼容输出：**

```text
保留当前 Swarm 实际依赖的世界系线速度数值
独立使用兼容话题
明确标注为 Swarm compatibility input
不作为通用 ROS Odometry
```

只调整 Swarm 的输入 remap，使其继续接收兼容话题。通用 ROS 消费者只读取标准 Odometry，避免用一个话题同时承载两种坐标约定。

### 2. 精确定义 qn 参考使用过程

当前外层参考速度为：

\[
v_{\mathrm{used},k}
=
\frac{p_{\mathrm{used},k}-p_{\mathrm{used},k-1}}
{\Delta t_{\mathrm{outer}}}
\]

其中：

```text
outer_dt_s = backend.step() 的外层步长
integration_step_s = qn 内部积分子步
outer_step_count = 外层控制调用次数
integration_step_count = 内部积分子步次数
model_time_s = 实际累计模型时间
```

外层步长不能与内部积分子步混用。

控制循环每次先冻结一个不可变的命令快照，再调用 `backend.step()`。`used_reference` 必须来自该快照，不能在积分结束后重新读取可能已被回调覆盖的 latest command。

使用现有标准消息发布：

```text
/drone_i/qn/used_reference_pose    geometry_msgs/PoseStamped
/drone_i/qn/used_reference_twist   geometry_msgs/TwistStamped
/drone_i/qn/diagnostics            diagnostic_msgs/DiagnosticArray
```

诊断中必须包含：

```text
source_trajectory_id
source_command_stamp
used_outer_step
model_interval_start
model_interval_end
velocity_directly_consumed=false/true
acceleration_directly_consumed=false/true
```

### 3. 新任务参考采用必须依靠轨迹归属

禁止使用：

```text
PositionCommand.stamp > goal_dispatch_time
```

作为新任务已采用的唯一证据，因为旧轨迹终点保持命令也会持续更新时间戳。

每次任务建立：

```text
execution_id
↔ Action GoalID
↔ goal_dispatch_time
↔ 每台成员派发前 trajectory_id
↔ 每台成员新的 trajectory_id
↔ qn source_trajectory_id
↔ used_outer_step
```

采用判定：

1. 保存派发前各成员轨迹编号。
2. 记录组级目标只由授权 publisher 发布一次。
3. 每台成员必须出现派发后的新轨迹编号。
4. qn 的 `source_trajectory_id` 必须与该轨迹归属一致。
5. 新轨迹编号不直接等同于新任务；若任务期间发生局部重规划，保留同一 execution 上下文。
6. 无法确认轨迹归属时标记 `REFERENCE_ADOPTION_UNCONFIRMED`，不得退化为“时间戳更新即通过”。

`execution_id` 由 runner 保存并映射到 Action GoalID，不假设 Action server 自动知道调度器内部编号。

### 4. 时间有效性增加累计偏差

在共同观测时刻 \(t_0\) 后计算：

\[
e_i(t)=
[\tau_i(t)-\tau_i(t_0)]
-
[t_{\mathrm{ROS}}(t)-t_{\mathrm{ROS}}(t_0)]
\]

持续记录：

```text
max_abs_model_ros_drift
max_cross_agent_drift
model/ROS rate
model/wall rate
```

七台数据按照共同 ROS 时间网格对齐；超出对齐窗口的样本记为缺失，不能直接比较不同接收时刻的诊断消息。

默认工程门槛：

```text
基线检查不少于30秒
model/ROS rate 在 0.95–1.05
累计模型-ROS偏差 <= 0.05 s
七机跨节点累计偏差 <= 0.05 s
时间偏差导致的参考位移 <= 0.5 * epsilon_p
```

30 秒只用于任务开始前的基线资格检查，不强制短任务本身运行 30 秒。任务执行期间继续检查，实验结束后汇总全区间结果。

驻留完成同时检查模型时间：

\[
\min_i
[
\tau_i(t_{\mathrm{hold,end}})
-
\tau_i(t_{\mathrm{hold,start}})
]
\geq hold\_duration
\]

因此 ROS 驻留时间不能自动替代 qn 模型驻留时间。

## P0：地图和运行健康

Action server 进程立即启动，但只有 `READY_IDLE` 状态接受 Goal。

就绪判断分为：

```text
话题存在
消息已经收到
输入对本次规划有效
```

CPU 模式只等待实际点云和规划接口；CUDA/depth 模式才等待 depth。

运行期持续检查：

```text
全局地图初始化
本地点云新鲜度
七路 Odometry 新鲜度
规划器健康
qn 状态更新时间
```

空点云不能单独证明“已知空环境”。只有显式配置的 `known_empty_map=true` 才能接受合法空地图。

输出：

```text
~ready
~readiness_reason
sensor_backend
missing_topics
stale_topics
planner_health
```

## P1：ActionServer 和资源占用

### 1. Goal 回调与执行循环分离

`goal_callback` 只执行：

```text
校验 Goal
→ 原子检查 READY_IDLE
→ 原子占用资源
→ 接受或拒绝
→ 交给单一执行工作循环
→ 立即返回
```

禁止在 goal callback 内运行完整任务循环。空闲检查和资源占用必须是同一个原子操作。

### 2. 固定资源状态机

```text
BOOTING
→ READY_IDLE
→ ACTIVE
→ HOLDING
→ SUCCEEDED
→ READY_IDLE
```

异常状态：

```text
UNKNOWN_LOCKED
```

规则：

- 运行中不接受新的 Goal。
- 运行中取消不调用 `set_preempted()`。
- actionlib 可能显示 `PREEMPTING`，但这不代表物理任务已取消。
- 不释放资源、不覆盖当前参考、不派发下一任务。
- Odometry 失联、执行超时、结果超时都进入 `UNKNOWN_LOCKED`。
- 只有完成驻留才释放当前资源。
- `UNKNOWN_LOCKED` 恢复必须重启完整仿真执行链，包括旧轨迹服务器、qn 节点和 Action server，不能只清理 Action 锁。

## P1：完成判定和实验有效性

分别输出：

```text
task_outcome = PASS | FAIL
safety_outcome = PASS | FAIL | NOT_VERIFIED
experiment_validity = VALID | INVALID | INCOMPLETE
```

指标区分：

```text
最终槽位误差：||p_i - p_i_goal||
轨迹跟踪误差：||p_i - p_i_used_ref||
速度跟踪误差：||v_i - v_i_used_ref||
驻留速度：||v_i||
编队形状误差：
max(i<j) ||(p_i-p_j)-s(r_i-r_j)||
```

缺失或时间无法对齐的样本必须记录：

```text
valid_sample_count
valid_sample_ratio
max_continuous_gap
alignment_failure_count
```

不能只剔除无效样本后统计剩余的良好误差。

碰撞或障碍净距检查独立于任务完成：

```text
到位但途中碰撞：
task_outcome=PASS
safety_outcome=FAIL
```

只检查离散轨迹点时，结果必须标记为离散采样安全检查，不扩大为连续时间安全保证。

## P2：连续任务和计划修复

A → B → Return 继续作为固定执行链测试，每个动作保存独立：

```text
execution_id
Action GoalID
trajectory_id 集合
planned_finish_at_dispatch
actual_finish
planner_nominal_finish
```

保留当前 `schedule.py`、`repair.py` 和 `process_completion()`，不建立第二套事件登记系统。

Test C 输出：

```text
plan_updated
updated_plan_used
dispatch_changed
```

串行单资源场景中：

```text
dispatch_changed=false
```

可以是正确结果，因为 repair-on/off 都必须等待上一动作真实完成。

验收重点是：

- 实际完成事件进入 planRepair。
- 修复前计划被保留。
- 修复后计划确实被 runner 读取。
- 同一 DelayEvent 只处理一次。
- 不通过让 repair-off 不等待真实完成来制造差异。
- 不把计划时间变化直接称为实际派发变化。

## P3：离线多资源扩展

P0–P2 通过后，单独增加离线执行单元模型：

```text
executor_id
physical_agent_ids
capabilities
available_from
nominal_speed
```

至少包含：

```text
一个任务只有执行单元 A 有资格
一个任务 A/B 都有资格但代价不同
两个任务竞争同一执行单元
```

这一阶段只验证资源选择和计划层竞争，不拆分当前七机物理编队，也不宣称完成多联盟物理闭环。

## 文献研究

继续下载开放版本到：

```text
research/literature/papers/
```

提交：

```text
research/literature/manifest.yaml
research/literature/notes/
```

PDF 默认不提交仓库。每篇笔记必须回答：

```text
该文献改变当前哪个接口定义？
该文献要求增加哪个反例测试？
```

优先级：

1. Calvo：延迟传播、等待吸收、repair 与全量重规划。[论文](https://arxiv.org/abs/2411.02062)
2. Swarm-Formation：编队槽位、重组、局部轨迹与上层联盟边界。[论文](https://arxiv.org/abs/2210.04048)
3. FaSTrack：规划模型、跟踪模型和误差界成立条件。[论文](https://arxiv.org/abs/1703.07373)
4. APEX-MR：预测时间与实际动作释放条件。[论文](https://arxiv.org/abs/2503.15836)
5. GRSTAPS：任务、分配、调度和运动可行性之间的信息交换。[资料](https://star-lab.cc.gatech.edu/papers/messing-grstaps/)
6. H-LTL/GCS：显式依赖和连续可行运动建模。[论文](https://www.roboticsproceedings.org/rss21/p099.html)
7. Robust MADER：通信延迟与异步轨迹发布，留到通信扩展。[论文](https://arxiv.org/abs/2303.06222)

## P0–P2 验收顺序

1. 标准 qn Odometry 与 Swarm 兼容输入语义分别正确。
2. qn 实际采用参考可追溯到 `trajectory_id`。
3. ActionServer 忙时不会自动抢占当前任务。
4. 取消、失联和超时不会错误释放资源。
5. 模型时间、ROS 时间和七机累计偏差通过门槛。
6. 单次七机任务完成真实到位和模型时间驻留。
7. A → B → Return 完成目标交接。
8. `task_outcome`、`safety_outcome`、`experiment_validity` 分开输出。
9. DelayEvent 幂等，repair 前后计划可复查。
10. repair-on/off 的作用是否影响实际派发能够被解释。
11. 无法证明新参考归属时，实验不得标记为完整有效。

最终冻结标准：

> 只有当本次任务的新轨迹能够通过 `trajectory_id` 归属到每台 qn，模型时间与规划时间持续兼容，并且七台动力学状态连续满足完成条件时，任务层才承认任务完成。

