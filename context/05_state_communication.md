# 状态、信息与通信

**更新：2026-09-19**

## 1. 本机状态

控制器使用本平台实际状态。仿真中以 qn 积分后的状态为权威，不把期望轨迹当实际位置。

本机控制闭环不要求母船高频参与。

## 2. 邻机状态

Swarm 的邻机轨迹/状态沿用其原生消息交换。

原则：

```text
收到什么才知道什么
```

评测器和 RViz 可以读取全局真值，但不能把这些真值回灌给受通信约束的算法。

失联不等于物理平台消失，也不应从碰撞集合中删除。

## 3. Observation 与 Delivery 必须分离

当前已经在任务语义中区分：

```text
Observation generated
!=
Result received
```

下一任务若依赖某项结果，应满足：

```text
t_start >= max(
  resource_available,
  required_information_received
)
```

当前 `C_delivered` 只有在兴趣点已经有效观测且结果被记录为接收时才计分。

## 4. 当前零延迟交付的含义

第一版 AAV 任务线允许：

```text
delivery latency = 0
```

但仍保留“观测事件”和“接收事件”两个记录。

这只是理想通信代理，不代表：

- RF 链路已建模；
- 水声链路已建模；
- 多跳已实现；
- USV 中继已实现；
- 通信窗口已进入调度器。

## 5. 后续真实通信变量

目标架构后续可能加入：

```text
link availability
delay
bandwidth
packet loss
communication window
cache
relay
rendezvous
```

海上异构通信尤其要区分：

```text
母船 / USV / AAV 间 RF
USV / UUV 间声学或其他水下链路
```

不能把水下通信当作免费、零延迟、无限带宽。

## 6. USV 中继角色

USV 后续可以是移动资源，而不是固定“网络盒子”：

```text
RELAY
DATA_AGGREGATION
RENDEZVOUS
LOCALIZATION_SUPPORT
```

中继任务完成条件不能只写“USV 到达某点”，还应与真实链路质量、被支援对象和有效通信区间相关。

当前尚未实现，因此不产生在线完成事件。

## 7. 通信研究边界

CoCoPlan、Robust MADER、Xiroi II 等用于定义未来约束和实验，不表示当前已经接入它们的求解器或通信系统。

后续引入通信前，先保证所有远程通道都受到同一实验条件约束，避免留下共享内存、全局真值或未限速话题作为隐藏旁路。
