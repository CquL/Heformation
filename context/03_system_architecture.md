# 系统架构：目标功能架构与当前实现

**更新：2026-09-19**

必须区分“最终应该是什么”与“现在代码已经做到什么”。

本轮五平台增量的当前实现：qn节点内有实验性PlatformTask/TakeReference；Otter/REMUS有原生模型
资格端点；runner具备可选三机异步执行，Qt复用其请求/确认与状态。它们尚未组成五平台完整请求：
跨模式生产资格、预承诺会合和受限通信接收仍未接线。没有新增协调节点或第二在线调度器。
当前证据与未完成项统一见 `docs/reviews/five-platform-progress-20260919.md`。

## 1. 目标功能架构

```text
任务说明 + 五平台能力 + 地图 + 实际收到的状态与数据
                              ↓
                    任务模型与实例生成
                              ↓
             ┌────── 联合规划协调范围 ──────┐
             │                              │
             │ 任务分配、执行顺序、角色选择 │
             │              ↕               │
             │ 数据依赖、通信机会、中继缓存 │
             │              ↕               │
             │ 平台运动可行性与连续轨迹     │
             │  ├─ AAV AIR：Swarm-Formation │
             │  │          + 单机适配分支   │
             │  ├─ USV：水面运动后端        │
             │  ├─ UUV：水下运动后端        │
             │  └─ AAV水中/跨介质：后续验证 │
             │              ↕               │
             │ 时间、能量、通信与安全检查   │
             └──────────────┬───────────────┘
                            ↓
                    已接受的执行计划
                            ↓
                   本地任务执行器 + 控制器
                            ↓
                       动力学 / 实物
                            ↓
          实际完成、结果接收、误差、故障、能量
                            ↓
                   保持 / 修复 / 重新安排
```

这里的“联合规划协调范围”表示这些约束必须互相影响，不要求所有变量都在一个母船进程里一次性求解。

母船可以决定：

- 任务由谁执行；
- 何时执行；
- 单机还是编队；
- 后续需要什么数据；
- 是否需要中继/会合；
- 何时需要重新安排。

各平台仍保留本地规划与控制。

## 2. 当前实际实现

```text
MonitoringRequest YAML
        ↓
load_request / expand
        ↓
ObservationTask
        ↓
to_plan_tasks
        ↓
Executor Planner
  ├─ aav_1
  ├─ aav_2
  ├─ aav_3
  └─ aav_formation
        ↓
静态成员与 Action endpoint
        ↓
MEMBER_TARGET / FORMATION_CENTER
        ↓
Swarm-Formation / member branch
        ↓
PositionCommand
        ↓
qn controller + 6DOF
        ↓
actual Odometry
        ↓
Geometric observation / Delivery / Formation diagnostics
```

上面部分目前有三种状态：

### 已经实跑

- 单机与组级 Action endpoint；
- 3 节点声明式编队图；
- 单机→组级→单机模式切换；
- qn 实际落点；
- Action Result。

### 已经实现并单测

- 高层请求加载；
- 任务展开；
- Executor 计划；
- 全局串行模式；
- 共享成员预测位置；
- 观测与交付；
- 条件复测；
- 原生 Swarm 编队图指标的区间诊断（无任意业务阈值）。

### 已于 main@1840b08 完成完整请求实跑

- runner 的 Executor 规划模式；
- PlanItem → 选中 endpoint 的完整派发；
- 用户确认开始；
- 请求→执行→观测→结果接收的一次完整运行；条件复测已实现，本次正例无需触发；
- dashboard 的任务层权威状态。

本轮增量为现有本地 FSM 的固定参考锁存、Action 内的停止观察及共享成员不可调度判断。
需求基线见 `docs/requirements/layered-monitoring-and-safety-hold.md`；它不新增安全协调层、控制器或数据系统。

## 3. 七机链路的位置

七机链路继续存在：

```text
固定七机 Plan
→ FormationAction
→ Swarm
→ qn
→ Odometry
→ DelayEvent / planRepair
```

它是：

- 历史受限 Calvo 闭环基线；
- M1/M2；
- 安全/感知/运动回归。

它不再表示目标异构系统架构。

## 4. 当前异构边界

```text
AAV AIR：在线
USV：offline-only
UUV：offline-only
受限通信：未在线
能量：未进入联合规划
```

没有执行端点的平台不能产生“实际完成事件”。

## 5. 未来联合的核心关系

后续重点不是简单增加模块，而是让以下关系变成真实约束：

```text
任务能否开始
取决于资源是否可用 + 所需数据是否到达

数据何时到达
取决于平台位置 + 通信机会 + 中继角色

平台位置
又取决于任务分配、轨迹和动力学
```

即：

```text
Task
↔ Information / Communication
↔ Motion / Control
```

当前只完成了这套目标架构中的一部分。
