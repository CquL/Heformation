# 当前必要输入输出

**更新：2026-09-19**

本项目不预设“大一统全能消息”。当前只定义真实连接所必需的输入、状态和结果。

## 1. 用户层输入

第一版使用结构化 Monitoring Request。

核心字段：

```text
request_id
regions
interest_points
weights
observation requirement
required_capabilities
service_time_s
deadline_s
delivery_required
requires_underwater
requires_relay_delivery
```

缺失必须字段时直接报错，不静默补默认 service time 或 deadline。

当前请求文件：

```text
integration/qn_aav_simulator/config/monitoring_request_coastal.yaml
```

## 2. 系统配置输入

由系统配置，不要求用户每次重填：

```text
平台清单
Executor 路由
physical_agent_ids
capabilities
Action endpoint
三机 formation slots
cruise altitude
地图 / scene
运动限制
安全阈值
```

最终目标平台：

```text
3 AAV + 1 USV + 1 UUV
```

当前在线 Action endpoint 只有 AAV。

## 3. 任务展开输出

请求经 `expand()` 产生 `ObservationTask`。

航点数量由覆盖需求决定，不等于兴趣点数量。

单机 ObservationTask 默认：

```text
required_agent_count = 1
allow_larger_unit = false
```

因此三机 Formation Executor 不会因为“人数更多且能力也满足”自动抢走普通单机任务。

## 4. Executor 计划

三机任务线使用：

```text
Executor
ExecutorPlan
ExecutorPlanItem
```

当前静态执行单元：

```text
aav_1         -> drone_0
aav_2         -> drone_1
aav_3         -> drone_2
aav_formation -> drone_0, drone_1, drone_2
```

静态成员重叠合法；同时占用共享成员非法。

任务线使用：

```text
build_executor_plan(..., serial=True)
```

串行起点：

```text
max(
  上一项在线串行任务结束时间,
  本执行单元各物理成员的预计可用时间
)
```

## 5. Action 输入语义

`Formation.action` 消息结构保持薄接口，不扩成任务总线。

### 单机

外部目标：

```text
成员自己的世界系目标 g_i
```

规划器：

```text
p_goal = g_i
```

不叠加 formation slot，不等待前序成员，不启用编队相似度项。

### 三机组级

外部目标：

```text
formation centre c
```

规划器：

```text
p_i_goal = c + scale * slot_i
```

三机配置使用声明式 3 节点期望编队图。

## 6. qn 状态与参考

权威实际状态：

```text
/drone_i_qn/odometry
```

参考采用证据包括：

```text
trajectory_id
source_trajectory_id
used_outer_step
used_reference
```

目标送达、Action 接受、规划成功和 qn 采用是不同事件。

## 7. 观测与交付结果

任务层至少区分：

```text
observed_fraction
delivered_fraction
uncovered_points
formation geometry diagnostics
formation Action results
retest decision
objective_outcome
```

定义：

```text
C_delivered =
Σ w_j * 1[observed_j AND received_j] / Σ w_j
```

Delivery 不能单靠 `delivered_point_ids` 越过 Observation。

## 8. 运动结果与任务目标结果分开

运动 Action 继续输出：

```text
task_outcome
safety_outcome
experiment_validity
```

任务层另外判断：

```text
objective_outcome
coverage / delivery
formation phase complete
mission complete
```

机器人到达目标并不自动表示监测任务完成。

## 9. 用户确认

当前交互：

```text
加载请求
→ 展示展开任务与分配
→ 用户确认
→ 才开始派发
```

默认未确认时不运动。

入口：`scripts/docker_run_monitoring_request.sh [request.yaml] [新实验目录]`。
runner 使用 `planning_mode=executor`；原七机路径使用默认 `fixed_coalition`。
不提供默认/隐藏自动确认；输入非 yes 或 EOF 都不派发。执行中不接收新批次。

几何代理只保留 footprint_radius_m、min_dwell_s、cruise_altitude_m 与垂直范围。
删除 exposure_s、blur_tolerance_m、nominal_standoff_m；旧字段不能被静默接受。

消息继续复用 `Formation.action`、原生 GoalID/Result、Odometry、PositionCommand，未增加消息层。
Result 的既有 evidence_file 指向既有采样记录，任务层据此计算覆盖；未在 ROS Action 中重复传输整条轨迹。
dashboard 从 `/formation_mission_runner/task_state` 读取任务层的计划、占用、当前动作、覆盖、接收和失败原因，
更新年龄同时显示；不从落点或收到任意状态消息自行推断整个任务已完成。


## 2026-09-19 本轮验收记录

完整请求 r3 已在同次运行贯通 CLI 确认、执行单元串行派发、真实 Result、几何观测/接收和实时显示，五段成功、六点观测/接收均 1.0。故意近距穿越的安全负例返回 SAFETY_FAIL，失败 Result 记录但保留资源并阻断重叠后继。证据见 `docs/reviews/taskline-implementation-20260919.md`。该完成口径仅为声明的几何代理，不能推广为真实图像或岸线载荷质量通过。
