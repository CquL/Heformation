# 必要输入输出：不再预先定义一套巨大数据包

**本页摘录V2已定位的原生接口；不是新schema或用户本地运行topic清单。**

## 1. 输入分三类

| 类别 | 必要内容 | 提供方式 |
|---|---|---|
| 任务请求 | 动作/任务类型、区域/位置、资格或人数、工作要求/时长、截止时间 | 任务文件、面板或外部任务系统 |
| 启动配置 | 平台列表、模型/工作域、运动限制、能源、地图、运行与通信条件 | 原生配置文件及最少补充 |
| 在线反馈 | 本机状态、实际收到的邻机轨迹、任务完成/失败、能源和必要故障信息 | 原生发布订阅或Action反馈 |

不要求用户每次重填全部配置。作业结果是执行输出，再作为上层反馈，不是预先填好的成功标记。日志内容也不是每层必须复制的通信载荷。

任务关系只采用所选上层实际支持的形式。第一版必须优化一般前置关系时，应选择相应模型；不能把不支持的输入默默忽略。

## 2. Calvo执行框架已存在的接口

下述内容来源于V2核查的`multirobot-use/mrta_execution_architecture`，适用于采用该执行框架的分支。[R10](13_references.md#r10)

### 规划请求：`action/HeuristicPlanning.action`

```text
# request
string scenario_id
string[] available_agents
string[] remaining_tasks
---
# result
bool success
TaskQueue[] planning_result
---
# feedback
string status
```

### 单平台任务队列：`action/NewTaskList.action`

```text
string agent_id
Task[] task_list
---
bool ack
---
string status
```

### 任务结束：`action/TaskResult.action`

原生结果区分FAILURE、SUCCESS、HALTED，携带Task和ack/status；原应用还有近距离检查参数。这里复用必要语义，不由字段名字推断本项目需要研发感知。

`msg/Task.msg`已有id、type及动作专用参数；`GenericTaskParams.msg`只有Td、Tw、Te，不能仅凭“Generic”一词推断它已经能描述任意海上作业几何。

## 3. Primitive的原生输入输出

V2定位到`src/planner/plan_manage/src/pp_replan_fsm.cpp`：[R03](13_references.md#r03)

```text
本机状态         odom_world
目标             /goal_with_id 或预置waypoints，取决于配置
接收邻机轨迹     planning/broadcast_primitive_recv
对外轨迹广播     planning/broadcast_primitive_send
运动输出         planning/selected_path_id、planning/polynomial_traj 等
```

消息类型、namespace、remap、触发条件和输出消费者以实际锁定版本为准。收到多项式或基元不是已经得到电机推力，后面要接原生轨迹服务器/控制器；这些无需再次包成一个自定义“全能指令”。

Swarm已经有本机里程计和轨迹收发；沿用已复现版本的入口，不要求改成与Primitive完全相同的内部结构。[R01](13_references.md#r01)

## 4. 第一轮只写两个方向的适配

```text
上层任务动作 → 所选后端的原生目标/协作请求
实际完成、失败或延迟 → 上层原生任务反馈
```

确实要增加的适配应说明：源字段/语义、目标字段/语义、差异、错误处理。已有字段能表达的就映射，不再新增同义对象。

例如区段跟随任务可能需要按原生航点输入或逐段目标执行；局部目标到达不等于整个作业完成，适配器只在全部要求满足时返回原生任务成功。若原后端不支持指定编队人数，先报告限制，不擅自把任何联盟当作合法编队。

## 5. 最少一致性

只要求实际连接所必需的一致性：机器人ID、坐标系与单位、时间基准、任务开始/结束/取消含义、状态与参考区分、控制接口层级、一次只有一个有效命令来源。

状态戳和任务ID优先使用原生字段。不能为了“未来可能需要”预设几十种Envelope、证书、缓存或权限包。必要的版本/重发语义若原生接口缺失且实验确实暴露问题，先记录缺口，再做最小改动。

## 6. 这不是最终消息规范

在线上层、执行器与运动后端尚未冻结之前，本页保持“已读接口＋待映射差异”性质。不存在本包已经实现的解析器、全平台统一控制消息或完整旅行时间服务。


---
整理依据：[V2实施方案](sources/implementation_plan_v2_2026-09-15.md)与[V2核查记录](sources/literature_audit_2026-09-15.md)。本页是资料重组，不表示新增代码或实验已完成。返回：[资料索引](README.md)。
