# 术语表

## Platform / Physical Agent

真实或仿真的物理平台实例，例如：

```text
drone_0
drone_1
drone_2
usv_1
uuv_1
```

## Executor

可被任务规划器预订的执行单元。

Executor 不等于物理机器人。

例如：

```text
aav_1         -> {drone_0}
aav_formation -> {drone_0, drone_1, drone_2}
```

两个 Executor 可以静态共享物理成员，但不能同时占用共享成员。

## Single-member Executor

只包含一个物理平台的执行单元。

当前单机 AAV 使用 member target。

## Formation Executor

包含多个物理成员并以组级语义执行任务的执行单元。

当前：

```text
aav_formation -> drone_0,1,2
```

## Offline-only Executor

存在于能力/资源模型中，但无真实 Action endpoint。

不能产生实际完成事件。

## Member Target

某个成员自己的世界系目标位置：

```text
p_goal = g_i
```

不加 formation slot。

## Formation Centre

组级目标中心：

```text
p_i_goal = centre + scale * slot_i
```

## Declared Formation

由配置声明 N 和 N 个 slot 生成的期望编队图。

当前三机使用该模式；七机继续使用原六边形类型。

## Observation

某兴趣点满足声明的可见性、遮挡、驻留和质量条件后形成的有效观测。

不是“机器人到了附近”。

## Delivery

观测结果被下一阶段/控制站接收的事件。

```text
Observation != Delivery
```

## C_observed

有效观测权重占总要求权重的比例。

## C_delivered

同时满足：

```text
observed AND received
```

的权重比例。

## Task Outcome

低层运动动作是否按 Action 语义完成。

## Objective Outcome

任务层作业目标是否满足，例如监测覆盖、交付、编队阶段完成。

两者不是同一个概念。

## Experiment Validity

本次实验是否具备足够证据支持结论，例如参考采用是否可归属、状态是否新鲜。

## Conditional Retest

初测结果被接收后，对未达标点最多释放一次的补测。

## Formation Shape Error

当前岸线阶段使用：

```text
E_form(t) =
max_{i<j} ||(p_i-p_j)-s(r_i-r_j)||
```

比较相对向量，不只比较成员间距离。

## Corridor Progress

实际编队中心沿 `path_start → path_end` 的投影进度。

回退相对历史最远进度判断，不只比较相邻样本。

## Authoritative State

对某一语义唯一可信的状态来源。

当前物理运动状态以 qn 实际状态为权威，不以规划轨迹或 RViz 显示代替。

## UNKNOWN_LOCKED

任务结果未知或安全/状态异常后资源不能安全释放的状态。

不应因为客户端超时就把资源重新分给下一任务。
