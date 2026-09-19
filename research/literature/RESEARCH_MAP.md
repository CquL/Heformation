# 研究地图（RESEARCH_MAP）

按 `prompt/wenxianSKILL.md` 第 2 节的状态标签维护。**禁止**把"论文提出 / 框架具备"写成"本项目已实现"。

- **已采用**：已进入当前工程主链
- **已复现**：已在项目中复现或运行
- **已全文阅读**：已获得并阅读全文
- **已重点参考**：直接影响接口、约束或实验设计
- **候选**：未来可能采用，当前未进入主链
- **Benchmark / 工具**：用于验证、仿真、评分或工程支撑
- **历史参考**：曾调研，当前不是主要路线

## 1. 已采用 / 已复现（真正在跑的东西）

| 模块 | 来源 | 状态 |
|---|---|---|
| 任务分配 / 调度 | Calvo 受限 Python 移植（`integration/mrta_python`） | **已采用** |
| AAV 编队运动 | Swarm-Formation（`upstream/Swarm-Formation`） | **已复现**（七机 AIR 闭环） |
| 任务 → 运动接口 | 本项目 `FormationAction` | **已采用** |
| AAV 控制 / 动力学 | qn 6DOF（`integration/qn_aav_simulator`） | **已采用** |
| ROS 执行 | ROS1 Noetic / actionlib | **已采用** |
| 场景 | 解析 scene publisher | **已采用** |
| 可视化 | RViz + `mission_dashboard.py` | **已采用** |
| 单机指令隔离 | 本项目补丁 `plan_manage_member_goal_entry.patch` | **已复现**（探针三项通过） |
| USV / UUV | 仅目标资源模型 | **未接入**（无执行端点） |

## 2. 全文可读的文献（本地 `papers/<category>/`）

| key | 类别 | 状态 | 主要影响 |
|---|---|---|---|
| `calvo-2024-delay-propagation` | mrta | 已全文阅读 / 已采用（受限移植） | DelayEvent、planRepair、每机器人队列 |
| `apex-mr-2025` | mrta | 已全文阅读 / 已重点参考 | 释放条件 = 前驱真实完成，不是计划时间 |
| `cocoplan-2026` | task_motion | 已全文阅读 / 已重点参考 | 完成 = 驻留 ∧ 时间约束；三类时间关系（含互斥） |
| `hltl-gcs-2025` | task_motion | 已全文阅读 / 候选 | 时序规范与分配进入同一搜索；完备性条件 |
| `swarm-formation-icra2022` | formation | 已全文阅读 / **已复现** | `weight_formation` 的真实语义；单人时集群图退化 |
| `swarm-formation-2022` | formation | 已全文阅读 / **已复现** | 稠密环境编队飞行（T-RO 版） |
| `minco-2022` | formation | 已全文阅读 / **已复现**（作为规划器底层） | 约束"精确消去"vs 代价项；不含跟踪误差 |
| `fastrack-2017` | safety | 已全文阅读 / 已重点参考 | `e_budget` 思想；规划模型与跟踪模型误差界 |
| `robust-mader-2023` | safety | 已全文阅读 / 已重点参考 | 延迟上界与 committed/optimized 轨迹区分 |
| `caric-2025` | benchmark | 已全文阅读 / 已重点参考 | 观测评分 `q_seen·q_blur·q_res`；去重；未碰撞才计分 |
| `xiroi-ii-2023` | maritime | 已全文阅读 / 已重点参考 | USV 中继 = 邻近保持 + 排斥；链路指标 0.117→0.468 Hz |
| `usv-uuv-cooperation-2023` | maritime | 已全文阅读 / 已重点参考 | 水下通信硬约束；失联需声明式应急策略 |
| `usv-uuv-integrated-system-2020` | maritime | 已全文阅读 / 已重点参考 | 第三种耦合：有线缆绳（连续供电+大带宽） |
| `auv-collaborative-missions-2020` | maritime | 已全文阅读 / 已重点参考 | 水下协作前置条件是定位/导航；链路参数量级 |

## 3. 摘要级（不得引用正文结论）

| key | 状态 | 阻断原因 |
|---|---|---|
| `grstaps-2021` | **abstract-only** | IJRR 闭源；Unpaywall `is_oa=false`、无 OA 位置；作者仓库只有代码 |
| `joe-2022-marine-heterogeneous-collaboration` | **abstract-only** | JOE 闭源；Unpaywall `is_oa=false`、`has_repository_copy=false`；OpenAlex `oa_status=closed`；期刊不在 DOAJ |

## 4. 本地参考副本（`upstream/`，未提交，见 `.gitignore`）

| 目录 | 用途 |
|---|---|
| `GCOPTER/` | 轨迹优化器，内含 `minco.hpp`（我们规划器的轨迹表示来源） |
| `libMultiRobotPlanning/` | CBS/ECBS 等 MAPF 参考实现 |
| `stonefish_ros/` | Stonefish 的 ROS 桥（未来 USV/UUV 后端候选） |
| `CARIC/` | CARIC 官方站点/文档（评分规则入口；不是仿真栈） |

已在 `upstream/` 且已提交的 15 个副本见 `upstream/README.md`。

## 5. 尚未取得全文的 P0/P1 项（已知缺口）

| 项 | 状态 | 说明 |
|---|---|---|
| Swarm-Formation T-RO 后续（编队重组） | 部分 | 已有 T-RO 版 arXiv:2210.04048；"重组/扩展"的后续工作未单独定位 |
| USV–AUV TMC 2025（水面支援水下定位与任务执行） | **未定位** | 多轮 OpenAlex 检索未命中该篇；不据此写任何结论 |
| MIMRee / SeaClear | P1，未下载 | 进入相应功能实现时再取 |
| Stonefish / VRX / DAVE 技术论文与文档 | P1，未下载 | 同上 |
| CBF / GCBF+、CAT-ORA、Primitive-Swarm | P1，未下载 | 仓库已在 `upstream/`（CAT-ORA、Primitive-Planner） |

## 6. 这些文献把哪些口径钉死了（一句话版）

1. 观测完成 ≠ 到位：需要质量因子（CARIC）。
2. 一次派发只算一个目标；一个兴趣点只取全局最优一次（CARIC）。
3. 安全失败不予计分，不能由覆盖率覆盖（CARIC）。
4. 完成 = 驻留 ∧ 时间约束；重叠执行单元之间是显式互斥约束（CoCoPlan）。
5. 依赖闸门由运行期"数据已到达"驱动（CoCoPlan）。
6. 释放条件 = 前驱真实完成，不是计划时间（APEX-MR）。
7. 参考安全 ≠ 实际安全（MINCO + FaSTrack）。
8. 编队的"存在"只能由实际队形误差证明；单人时编队代价会把成员拉飞（Swarm-Formation ICRA）。
9. USV 中继 = 邻近保持 + 不碰撞，且必须有链路指标（Xiroi II）。
10. 水下交付不能默认零延迟；失联必须有声明式应急策略（USV-UUV 综述 / AUV 综述）。
