# 当前核心文献与工程作用

**更新：2026-09-19**

详细全文状态以：

```text
research/literature/manifest.yaml
research/literature/notes/
```

为准。

本页只保留当前真正改变接口、约束或实验的工作。

## 1. 任务分配与执行反馈

### Calvo / Capitán

**Heterogeneous Multi-robot Task Allocation for Long-Endurance Missions in Dynamic Scenarios**

当前作用：

- 受限 v9 调度依据；
- DelayEvent / planRepair；
- 等待与时序传播概念；
- 不把受限 Python port 写成完整 MATLAB 等价复现。

状态：全文已下载并阅读。

### GRSTAPS

**Graphically Recursive Simultaneous Task Allocation, Planning, and Scheduling**

当前作用：

- 任务分配、调度和运动可行性应双向交换；
- 单一欧氏标称 travel time 不能证明连续运动可执行。

状态：2026-09-19 本轮访问 SAGE 在线正文，核对 §4.4/4.5、§5 实验设置与 §6.1；本地仍无 PDF，未声称阅读全文。原实现使用二维 Lazy PRM、不含动力学，也不处理机器人间运动碰撞，不能据此继承三维集群安全保证。

### APEX-MR

**Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly**

当前作用：

- 实际执行完成而非计划时间决定后继释放；
- 延迟和偏序执行语义。

状态：全文已读。

## 2. AAV 编队与轨迹

### Swarm-Formation ICRA 2022

**Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments**

当前作用：

- 当前开源代码和 formation similarity term 的主要依据；
- `weight_formation` 是队形相似度代价权重，不是“编队模式成功”的自动证明；
- 任务层必须另外判断组级作业是否完成。

状态：全文已读，源码已接入。

### Swarm-Formation T-RO 2023

**Robust and Efficient Trajectory Planning for Formation Flight in Dense Environments**

当前作用：

- 编队槽位、重组、局部轨迹与上层 execution context 边界；
- 不能把论文后续能力自动等同于当前仓库源码全部已经实现。

状态：全文已读。

### MINCO / GCOPTER 相关

用于理解 Swarm 的连续轨迹表示和几何约束。

重要边界：

```text
规划轨迹约束满足
!=
真实 qn 轨迹自动满足
```

## 3. 规划—跟踪安全

### FaSTrack

当前作用：

- 规划参考与真实跟踪状态必须分开；
- 误差预算只有在对应模型和保证条件下才能当理论界。

当前项目的 `e_budget` 是工程预算，不能冒充 FaSTrack 的 HJ 保证。

状态：全文已读。

### Robust MADER

当前作用：

- optimized trajectory 与 committed trajectory 区分；
- 通信延迟安全依赖明确的延迟上界。

当前尚未把其通信安全机制接入主链。

状态：全文已读。

## 4. 监测任务与完成判据

### CARIC

**Cooperative Aerial Robot Inspection Challenge: A Benchmark for Heterogeneous Multi-UAV Planning and Lessons Learned**

直接影响：

- 有效观测不能只看半径；
- `q_seen * q_blur * q_res`；
- 每兴趣点取最佳观测，避免重复累计；
- 安全失败不能被 coverage=1 覆盖；
- 结果由控制站收到后才结算。

当前项目使用的是基于这些原则的解析代理，不是原 CARIC 相机模型。

状态：全文已读。

## 5. 通信与任务联合

### CoCoPlan

当前作用：

- precedence / mutual exclusion / concurrency；
- 运行期才能知道的通信条件；
- “数据到达”可成为任务释放条件；
- 通信质量与运动/任务可能需要联合协调。

当前只吸收了语义和反例，没有接入其求解器。

状态：全文已读。

## 6. 海上异构协同

### Xiroi II

**Xiroi II, an Evolved ASV Platform for Marine Multirobot Operations**

直接启发：

- ASV/USV 可以作为 AUV 与地面站之间的中继；
- 中继的目标不是单纯“到一个点”，而是维持有效链路/邻近关系；
- 中继任务需要链路性能证据。

状态：全文已读。

### USV/UUV cooperative surveys

海洋机器人综述和 USV-UUV 集成工作用于确认：

- 水下通信不是免费无线网络；
- 水下定位与通信往往依赖 USV 或外部基准；
- UUV 不能简单套用 AAV travel time 和运动模型；
- 中继、定位支援、数据汇聚是明确角色。

当前均为后续五平台设计依据。

### IEEE JOE 2022 heterogeneous marine collaboration

直接支持：

- UAV/USV/UUV 的跨域感知协同；
- 水面节点可承担集中协调/数据汇聚角色。

当前资料库为 abstract-only，不能据此声称阅读全文细节。

## 7. 文献使用规则

每篇核心论文只提取：

1. 它解决什么问题；
2. 它采用什么任务/资源/通信/运动假设；
3. 它改变我们哪个接口或约束；
4. 它要求增加什么反例或实验。

下载论文不是为了堆算法，而是为了减少拍脑袋定义任务接口和完成条件。
