# 当前冻结决定与未知项

**更新：2026-09-19**

## 1. 已冻结项目决定

| 决定 | 当前状态 |
|---|---|
| 最终平台组成 = 3 AAV + 1 USV + 1 UUV | 已冻结 |
| 七机不再代表最终系统 | 仅 M1/M2、历史兼容和回归 |
| 当前先完成近岸监测 AAV 子系统 | 已冻结 |
| 首版在线只执行 AAV | USV/UUV offline-only |
| 三单机 Executor + 一三机 Formation Executor | 已冻结 |
| Executor 可静态共享成员，但不得同时 ACTIVE | 已冻结并已有计划层实现 |
| 单机目标 = member world target | 已实现 |
| 组级目标 = formation centre + slot | 已实现 |
| 三机使用声明式 3 节点期望编队图 | 已实现并实跑 |
| 单机/组级模式必须可反复切换 | 已实现并实跑 |
| 任务线全局串行 | 已实现并单测 |
| 成员预测状态按物理成员维护 | 已实现并单测 |
| Observation 与 Delivery 分离 | 已实现 |
| C_delivered 必须 observed AND received | 已实现 |
| 条件复测最多一次 | 已实现语义 |
| 岸线编队按全区间相对向量误差与走廊判定 | 已实现并单测 |
| 用户确认后才开始执行 | 已冻结，尚未接线 |
| Swarm-Formation 保留 AAV 编队后端 | 已冻结 |
| 只推 main，不自行扩大范围 | 已冻结 |
| 每次代码/实验更新 WORKLOG | 已冻结 |

## 2. 当前明确未实现

- runner 的 Executor 规划模式；
- PlanItem 真实驱动不同 Action endpoint；
- CLI 用户确认；
- 完整 MonitoringRequest 在线运行；
- dashboard 任务层覆盖/交付显示；
- USV 水面后端；
- UUV 水下后端；
- RF/水声通信；
- 通信窗口/缓存/中继联合安排；
- 能量联合约束；
- AAV 水中/跨介质模式。

## 3. 当前已知技术未知

### 同位置目标的 adoption 证据

若某成员已经位于下一组级任务槽位，规划器可能不产生新 trajectory_id。

当前 adoption 规则要求可归属的新轨迹，因此该任务可能无法确认采用。

需要决定的是如何在**不把旧参考误认为新任务**的前提下，为“无需移动但任务上下文确实改变”的动作建立可靠归属证据。

这是当前真实开放问题，不应静默退回时间戳通过。

### USV/UUV 后端

目标架构明确，但具体在线后端还未冻结。

Stonefish / VRX / DAVE 等是候选，不同时全部接入。

### 通信模型

真实设备参数未知。

第一版允许零延迟 delivery 代理，但进入通信研究前必须明确：

- RF / acoustic；
- delay；
- bandwidth；
- loss；
- connectivity；
- relay role。

### 新理论

任务—信息—运动联合、角色选择和通信 deadline 仍是候选问题，不是已定创新。

## 4. 已撤销或过时的决定

以下不再作为当前方案：

- “固定七机就是当前目标平台系统”；
- “多 Executor 只允许离线且不得成员重叠”；
- “最终异构平台组成后续再定”；
- “z 固定为 0.5 m”；
- “所有任务通过单一 formation_action endpoint”；
- “任务启动后自动读取固定 tasks 就开跑”；
- “观测完成等于到位+驻留”。

历史资料可以保留，但不能重新作为默认执行规则。
