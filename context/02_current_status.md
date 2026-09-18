# 当前状态：不把资料、复现和集成混为一谈

**状态更新：2026-09-17。V2长期背景保留；当前阶段按[plan.md](../plan.md)的用户冻结方案实施。**

## 1. 事实状态

| 内容 | 当前可以说什么 | 尚不能说什么 |
|---|---|---|
| Swarm-Formation | 官方仓库已整理并在Docker Noetic中编译，qn AIR七机示例已运行 | 已完成全部后续T-RO功能或整个海上任务系统 |
| Primitive-Planner | 原始基元库已生成，五机 launch 已生成，Noetic 临时工作区 18 包编译通过 | 已完成完整 ROS 多机启动和运动验收，或可直接替代严格编队 |
| Calvo MRTA/执行框架 | 原始代码和执行入口已核查；MRTA 需要 MATLAB/Gurobi | 已集成 Swarm/Primitive，已支持任意任务前置关系 |
| D-ITAGS | 原始仓库已拉取，CMake 已定位缺失依赖 | 已本地完整复现、与另一执行框架无缝兼容 |
| Swarm + qn AIR闭环 | 单机、七机启动、目标运动和最终编队位置已有运行证据 | 已覆盖任务调度、海洋平台和跨介质 |
| 上游独立复现 | Fossen原生测试14项通过；Primitive和EGO原始Noetic工作区分别编译通过；DMPC进入原始8机循环；H-LTL全量语法检查通过；DANCERS基础消息/协调包已编译 | 尚未完成全部ROS/MATLAB/商业求解器项目的原始运行；详见[16](16_upstream_reproduction_status.md) |
| Calvo Python + FormationAction切片 | 已有受限计划模型与Action切片；本轮重跑旧4项计划/repair测试和Action服务冒烟通过；既有Action生成及qn镜像构建记录保留 | 新v9行为测试、Test A/B/C完整验收仍在实施，不能把旧测试通过写成全部完成 |
| 完整联合协调器 | V2明确作为第一版目标，保留母船—节点完整链路 | 当前尚无运行通过的任务/资源/运动反馈组合系统 |
| 海面/水下/AAV | 有候选模型和仿真来源 | 已验证全工作域、瞬时模式切换或任意悬停 |
| 新理论 | 有待测试的问题清单和近邻工作 | 三条融合概念已构成首创，或全系统保证已成立 |
| 旧HUC | 用户明确不再参考或继承 | 仍是可复用外壳、默认接口或必审计对象 |

## 2. 当前本地实现状态

当前工作区为`/home/lhj/Swarm-Formation`，远端为`CquL/Heformation`。上游源码放在
`upstream/Swarm-Formation`，qn平台适配放在`integration/qn_aav_simulator`，模型放在
`models/qn/qn.slx`。Docker镜像`swarm-formation-qn:noetic`包含官方18个catkin包和新增的
qn包，共19个包。

已验证：qn PositionCommand经过原RBF/PD、十路执行器和六自由度数值积分后发布Odometry；
官方`normal_hexagon.launch`的`drone_0`至`drone_6`共7个节点能够运动并到达目标槽位，最大
最终位置误差约0.00062 m。当前实验区域是AIR，尚未验证水下或跨介质。

## 3. 实施选择状态

唯一在线上层已冻结为 **Calvo v9 restricted-domain Python port**。保留原始MATLAB，移植版位于`integration/mrta_python/`；不称完整MATLAB等价复现，不实现v13剩余任务重新分配。

本阶段固定Swarm七机AIR和qn状态源，使用ROS1 Noetic/actionlib。`Plan.items`是唯一计划状态；本版`wait_time = 0`，正等待吸收和多执行单元资源竞争后续再验收。电池不形成约束，充电、fragmentation、relay、动态联盟人数关闭。

Primitive、D-ITAGS、OmniPlan ROS2、其他运动后端、水下/跨介质与完整网络联合仿真保留为后续对照或扩展，不构成本轮前置条件。原始复现继续按上游环境隔离。

## 4. 待由本地工作补齐的少量事实

当前待完成：受限v9自由任务小算例；七机真实Odometry连续驻留与异常路径；A→B→Return；实际小/大正延迟的幂等repair与后续派发。长期候选的原始复现事实见[16](16_upstream_reproduction_status.md)，不把那些阻塞误写为当前主链阻塞。

本页不填未经读取的路径、commit、耗时和指标。V2文献记录中的源码快照不是用户本地版本。

## 5. 背景维护与当前实施

此前已整理V2资料并完成Swarm/qn AIR运动基线。本轮把用户冻结方案写入plan.md，同步当前架构、接口、阶段决定与验收范围；受限v9和完整Action闭环正在实施。具体结果见[15](15_handoff.md)，文档同步不新增实验通过项。

## 6. 后续怎么维护

只在实际运行后更新：采用的上游/版本、原始例子是否通过、集成改了什么、通过何种场景。论文声称、代码存在、原例复现、组合测试、定理适用性分别描述即可，不强制引入分级数据库。

没有运行证据时保持“待复现”。修改设计文档不会自动把实施阶段标为完成。废止的旧HUC决定不得在下次交接中重新出现。

## 7. 通用 Skill 资料归档（2026-09-17）

已将9个来源的21个具名Skill按固定版本独立归档到`/home/lhj/skill-library/`，
用途、来源、许可和依赖见[中文索引](/home/lhj/skill-library/README.md)。已核对21个主入口、附带资源及31个本地索引链接；
未接入Codex/Claude、安装运行依赖或改动项目代码。本次仅完成资料归档，不新增仿真、算法或闭环运行证据。


---
整理依据：[V2实施方案](sources/implementation_plan_v2_2026-09-15.md)与[V2核查记录](sources/literature_audit_2026-09-15.md)。V2来源保留为历史依据；当前决定按用户最新冻结方案更新，实验完成情况仅以实际运行记录为准。返回：[资料索引](README.md)。

## Codex 配置补充（2026-09-17）

按用户要求，将`/home/lhj/.codex/config.toml`的`model_context_window`设为`1000000`，
`model_auto_compact_token_limit`设为`900000`。TOML解析及配置值校验通过；
尚未验证当前API服务实际接受1M上下文，不新增项目运行证据。
