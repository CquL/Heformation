# AGENT.md — 海上异构无人集群项目

> 当前背景状态：2026-09-19。  
> 当前工程事实以 `context/02_current_status.md`、`context/15_handoff.md` 和 `docs/WORKLOG.md` 为准。  
> 最新用户明确决定优先于旧 `plan.md`、V2 历史方案和旧聊天结论。

## 1. 项目做什么

建立一个**母船统筹、异构平台本地执行、任务结果反向修复计划**的海上无人集群系统。

最终目标平台固定为：

```text
3 × AAV
1 × USV
1 × UUV
```

系统输入包括：

```text
任务说明
+ 平台能力
+ 地图
+ 实际收到的状态与数据
```

目标链路为：

```text
任务模型与实例生成
↓
任务分配 / 执行顺序 / 角色选择
↕
数据依赖 / 通信机会 / 中继与缓存
↕
平台运动可行性与连续轨迹
↕
时间 / 能量 / 通信 / 安全检查
↓
已接受的执行计划
↓
本地任务执行器 + 平台控制器
↓
动力学 / 实物
↓
任务结果、消息到达、误差、故障、能量
↓
保持计划 / 修复 / 重新安排
```

“协同”不是所有平台始终同步或保持一个队形。并行作业、任务接替、数据依赖、中继、会合以及某一阶段的编队都属于协同。

## 2. 当前主推进，不再把七机当最终系统

### 当前任务线

当前主推进是**近岸监测 AAV 子系统验证**：

```text
MonitoringRequest
→ 请求校验与任务展开
→ Executor 计划
→ 三个单机 Executor + 一个三机 Formation Executor
→ 各自 Action endpoint
→ 单机 member target / 三机 formation centre
→ Swarm-Formation + qn
→ actual Odometry
→ Observation / Delivery / Task Result
→ 条件复测 / 后续计划
```

三机在线执行单元：

```text
aav_1         -> {drone_0}
aav_2         -> {drone_1}
aav_3         -> {drone_2}
aav_formation -> {drone_0, drone_1, drone_2}
```

### 七机配置的定位

原七机 Swarm/qn 配置继续保留，但只作为：

- 历史兼容；
- M1/M2 安全与运动闭环基线；
- 回归测试；
- 上游 Swarm-Formation 复现参考。

**七机不是最终目标平台组成。**

## 3. 当前已经完成到哪里

代码基线为远端 `main` 的 `79036d6`；以下本轮更新在工作树中，未提交/推送。

- 三机声明式期望图使用 3 节点；七机原七节点图及类型保留。
- 五段 Action 实跑通过：单机→连续单机→组级→再单机→同位置单机，每段均有正确路由、落点、SUCCEEDED Result 和释放依据。
- 同位置任务复用原生静止轨迹；三机适配会话连续目标保持原生 ID 递增，qn 仍须证明新参考采用。
- 同一 runner 已有 fixed_coalition / executor；请求加载、展开、串行计划、物理成员预测、所选端点、收到结果后修复、未知结果锁定已接线。模块测试 258 项通过。
- 走廊/回退/固定朝向门槛与未校准图像质量评分已撤销。记录原生 Swarm 图指标，不指定任意形状业务阈值。
- RViz/任务 dashboard 已同时实跑；完整请求 r3 五段 SUCCEEDED、六点几何观测/接收均 1.0、按期完成，最小转场实际机体净距 1.135775 m，资源全部释放。旧 r2 越限失败保留。
- M2 原盒子失败已定位到参考余量不足后的实际净距越限；扫描中断负例已验证锁定和阻断后继。

本轮修正范围已验收；后续范围仍未完成：真实图像/岸线载荷验证、USV/UUV 在线后端、真实中继与通信/能量联合安排。
详细证据见 `docs/reviews/taskline-implementation-20260919.md` 和 WORKLOG 最新记录。

## 4. 运动后端与平台边界

当前：

```text
AAV AIR：Swarm-Formation + 已适配的单机 member 分支 + qn
USV：尚无在线水面执行后端
UUV：尚无在线水下执行后端
AAV 水中/跨介质：后续单独验证
```

不要把降低 UAV 速度当作 UUV 动力学，不要让两个仿真器同时推进同一物理平台，也不要将 RViz/评测真值作为控制算法的隐藏输入。

## 5. 任务、观测和交付语义

第一版用户输入为结构化请求，不做自然语言理解。

必须区分：

```text
运动到位
有效观测
结果已接收
任务目标完成
安全通过
实验有效
```

兴趣点观测当前仅为请求声明的几何可见性（范围/LOS）与连续驻留代理。
未校准的曝光/模糊/分辨率评分已删除；不得将其写成 CARIC 原相机模型或有效图像证明。
三机转场仍需真实 Action 完成；未定义的岸线点不补造，不宣称岸线载荷任务通过。

`Observation generated != Result received`。

零延迟交付只是当前显式仿真假设，不代表真实海上通信已实现。

## 6. 通信与异构平台的目标位置

USV/UUV 保留在目标五平台体系和离线资源模型中。

后续 USV 可能承担：

```text
SURFACE_INSPECTION
RELAY
RENDEZVOUS
DATA_AGGREGATION
LOCALIZATION_SUPPORT
```

但在没有真实执行端点和链路模型之前，不产生实际完成事件。

如果用户请求明确要求水下作业或经 USV 中继交付，而当前无对应在线能力，系统必须明确报告不可完整在线执行，不能静默省略后仍给整个请求 PASS。

## 7. 稳定工程规则

- 一次任务只有一个权威上层计划状态源。
- 同一物理平台同一时刻只有一个生效的运动/控制参考源。
- Executor 可以静态共享物理成员，但共享成员的 Executor 不得同时 ACTIVE。
- 规划参考与实际 qn 状态必须分开。
- 任务完成与安全判定必须分开。
- 数据产生与数据接收必须分开。
- 未运行的功能写“未验证”，不得从代码存在推导为通过。
- 论文、源码、原例复现、项目集成和理论保证必须分别陈述。

## 8. 研究边界

不做：

- 珊瑚白化、风机缺陷等业务识别算法；
- 业务载荷选型；
- SLAM / 定位算法研发；
- 为了“显得统一”预建庞大消息协议；
- 未经实验就宣布“任务—信息—控制融合”已经成为新理论。

当前创新仍是候选研究问题，先由真实反例确定缺口，再对照已有工作。

## 9. 阅读顺序

每次先读：

1. 本文件；
2. `context/02_current_status.md`；
3. `context/15_handoff.md`；
4. 与本次任务相关的专题文件。

常用专题：

| 工作 | 文件 |
|---|---|
| 总体架构 | `context/03_system_architecture.md` |
| 输入输出 | `context/04_inputs_outputs.md` |
| 状态/通信 | `context/05_state_communication.md` |
| 场景与验收 | `context/08_first_scenario_benchmark.md` |
| 当前阶段路线 | `context/09_implementation_plan.md` |
| 研究问题 | `context/10_research_novelty.md` |
| 文献 | `context/12_literature_review.md`、`13_references.md` |
| 决策 | `context/14_decisions_and_unknowns.md` |

## 10. 仓库与日志规则

只推 `main`，未经明确要求不要创建或推送其他远端分支，也不要自行扩大动作范围。

每次改代码或跑实验，都必须在 `docs/WORKLOG.md` 顶部追加一条：

```text
计划
实际
效果
证据
未完成 / 下一步
```

只追加，不重写历史。

完成阶段工作后同步更新：

```text
context/02_current_status.md
context/15_handoff.md
```

若改变稳定选型或范围，再同步 `context/14_decisions_and_unknowns.md`。
