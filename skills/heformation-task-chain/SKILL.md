---
name: heformation-task-chain
description: "在 Heformation 项目接手、规划或实施三业务协同任务链，明确当前任务、下一步方案和最终交付，继续 B/C 设施巡检及动态任务调整。适用于该项目的目标说明、服务器接手、相关阻塞修复与完整任务验收。"
---

# Heformation 三业务协同任务链

## 本次任务

在**现有 Heformation 单一任务系统**中完成 B/C 设施巡检接入，复用已完成的 A 业务和原控制模型，交付三业务共用的实时可视化协同任务系统。

固定平台为三台同型同能力 AAV、一台 USV、一台 UUV。成员身份由当前任务与实际状态选择；AIR/WATER 端点属于同一物理成员。我们部门负责**任务规划、分配与资源调度、路径自主规划、编队／跨介质控制、运动执行和控制侧报告接收**。专业识别、真实载荷质量评估和精细三维重建由外部模块承担。

| 业务 | 输入与控制作业 |
|---|---|
| A：区域调查／珊瑚礁监测 | UI 圈选区域，组织平台扫描运动和控制侧覆盖；作为用户已确认完成的基线保留复用 |
| B：海上风电装备巡检 | 选风机和作业部位，展开水上结构巡视／水下基础巡视，并安排共享支援 |
| C：海上作业平台／海底管路巡检 | 选设施平台、外侧／水下外部结构和连续管段，展开轮廓／沿线／局部作业 |

B/C 共用分层轮廓、沿线／条带、局部作业三类几何生成。设施定义表达作业意图，执行端从实际位置规划到入口、段间连接、绕障与退出。

## 最终要做出来的东西

一个可在 server715 运行、具有独立 Qt 控制台和 RViz 窗口的**三业务异构航行器协同控制仿真系统**，贯通：

```text
选择 A/B/C、区域／设施及范围
→ 从实际平台状态快速输出完整联合 Plan
→ 用户确认
→ 并行 AIR／qn WATER／Otter 作业与共享支援
→ 实际合格进度、匹配 Goal 的动作终态、母船报告实收
→ 目标区驻留
→ 下一任务／排队任务／用户确认共同返航
```

执行中新增任务由用户选择**排队**或**立即替换**；替换经过原安全终态交接。驻留和返航中也能接受新任务。运动不可继续、支援失效或明确复查要求反馈给原任务权威调整。规定作业和必要实收完成后驻留，人工确认才共同返航；AAV 返航使用原 Swarm 编队。

最终交付包括：可加载的 B/C 模板、同一套联合规划与执行链、UI 的分工／进度／支援／队列／替换／驻留／返航操作、独立 RViz 的实际运动和 AIR/WATER 模式，以及完整业务与动态调整的真实记录。规定返回的最终状态为**五平台实际 HOME、零资源锁、明确结束会话**。

## 使用时先核实当前事实

服务器项目根：`/home/server715/data/lhj/codes/Heformation`，实际路径 `/data/lhj/codes/Heformation`。直接在服务器工作的 Codex 操作该目录；本地控制方式才使用 SSH。本文代码路径均相对项目根。安装别名指向仓库技能目录，文档链接按技能文件实际所在目录解析。

先读项目 [AGENT 规则](../../AGENT/AGENT.md)、[当前状态](../../context/02_current_status.md)、[交接](../../context/15_handoff.md) 的最新置顶，以及 [WORKLOG](../../docs/WORKLOG.md) 最新条目；详细入口、运行方式和证据见 [服务器总接手文档](../../docs/requirements/server715-project-handoff-20261003.md)。业务与方法依据见 [B/C 模板设计](../../docs/requirements/inspection-business-templates-20261001.md)。

**2026-10-03 创建时快照：**

- A：用户确认已完成，保留其有效代码，本轮集中于 B/C 和共同任务链。
- B：r67 已完成四项规定巡视、4/4报告实收、匹配成功终态、零锁 HOLDING；最终会话返航仍待验收。
- 动态交接：B→C 排队已实际激活；C 执行中安全替换为管件2复查，完成 ENTER／LOCAL4s／EXIT、报告实收和零锁驻留。被打断的 C 保留为被替换，不能算完整 C 通过。
- 完整 C：新请求在 WATER AAV 与 USV 近距时失败；局部 WATER 导航和 PVS 指令＋coast 查询缺动态成员输入。相关修正尚未实施。
- AAV 后退：已证明一次原急停把参考跳到落后的实际位置并置零参考速度；连续减速、边界重定时和实际终点驻留修正已加载，整段实际回跳／折返审计仍未完成。
- 旧世界与读取进程已退出；旧失败、未知锁和未验证停止保留。完整 C 与最终动态返航总验收尚未完成。

此快照不能覆盖之后的新代码与实际记录。用户的新指令优先于本技能；状态判断必须区分源码已存在、实际运行、规定作业完成、动作终态、结果实收和整链验收。

## 接下来推进方案

1. **修复当前真实阻塞。**先用 r67 的 `aav-usv-encounter-window.json` 对照双方位姿、模式、目标和参考。在原 `LocalSurveyMap`、WATER 执行和 PVS 局部查询中接入新鲜动态成员状态／运动，核对支援进场与巡视／EXIT 的空间关系。当前故障需要执行输入修正，不能只靠上层事后近距报警。
2. **完成完整 C。**本轮范围为平台 AIR 三层／60段、水下外部结构三层／60段、连续管段0..3（3沿线段＋3个LOCAL4s）。真实区间、动作终态、共享支援与报告实收全部成立后驻留。演示覆盖求解器选择的 AAV 入水／作业／出水，不把某个成员身份或跨介质方法写成所有请求的永久要求。
3. **补齐同会话动态与返回证据。**保留 B→C 排队，核对执行中队列预览的具体拒因；完成替换／复查、驻留、人工 Swarm 编队返航、返航中换任务后继续执行及最终 HOME。恢复为新请求时独立计量，不能拼接旧 C 部分区间。
4. **核对同次实际运动与界面。**目标／pos_cmd／PolyTraj／采用参考／Odometry 对照后退与避碰；区分巡视转弯、必要应急和同一意图下的参考回跳。Qt/RViz 的角色、模式、进度、收件和等待原因来自同一任务权威。
5. **按实际调用关系清理并交付。**新链贯通后合并必要重复转换／校核，清理退出主链的硬编码和无消费者字段；保留 A 与 Swarm／qn／PVS 的有效实现、原模型和来源署名。

用户只问目标／架构／计划时，说明事实和方案；用户明确要求实施／继续／验收时，在其授权范围内推进相应步骤。调用本技能本身不授权启动实验、推送或新增外部操作。

## 实施边界和代码落点

- **同一任务权威、Plan 和原 Action。**不新增协调服务、能力注册中心、Action 包装层或第二套状态。`inspection_work.py` 只做几何与进度。
- **联合求解保持快速。**原10墙上秒预算，按当前状态和任务级估计排序，首个完整可派发方案即返回；不要求最优，不逐候选预演全任务动力学。固定候选展开顺序不等于串行执行。
- **任务级估计与执行分开。**航行耗时用于排序／预约／调整；规定作业位置、范围、深度／高度、偏距及朝向必须准确满足。运动可继续与安全由实际执行反馈判断。
- **保留原后端。**AIR Swarm→qn；AAV WATER 与固定 WATER UUV 使用 qn；USV 使用 Otter/PVS。WATER 航向主要沿路线切向。
- **真实计量和因果。**合格区间取并集，重复经过不重复计数；暂停／绕障／跳样不补缺段。账本绑定请求、work与版本；同请求有效修复可保留区间，新请求从零。旧报告不解除新 Goal 的锁，报告生成、动作完成与母船实收分别核对。
- **约束有来源。**使用当前任务定义、原模型／接口和声明包络；新增限制需说明依据和阻塞案例。0.5m成员／0.2m环境净距是当前工程配置，不是论文规定常数；当前缺口修复不能靠降低这些值或伪造位姿／速度过关。
- **围绕完整业务验证。**只处理阻塞本链的问题；必要小检查用于重复占用、进度跳增、旧结果解锁等直接错误。保留 A 已完成基线，避免过度封装、单机资格系列、无关回归和旧长序列。截图／回放不替代实跑。

关键修改位置：

| 位置 | 作用 |
|---|---|
| `integration/qn_aav_simulator/scripts/mission_console.py` | 选业务／范围、Plan确认、排队／替换与返航 UI |
| `integration/qn_aav_simulator/src/qn_aav_simulator/monitoring_request.py` | 模板展开和请求／work定义 |
| `integration/mrta_python/executors.py` | `build_inspection_executor_plan` 联合分配与共享支援 |
| `integration/qn_aav_simulator/scripts/formation_mission_runner.py` | 任务权威、派发／锁／收件／会话 |
| `integration/qn_aav_simulator/src/qn_aav_simulator/inspection_work.py` | 共享作业几何与实际进度 |
| `integration/qn_aav_simulator/scripts/formation_action_server.py` | AIR目标与实际完成 |
| `integration/qn_aav_simulator/src/qn_aav_simulator/platform_action.py` | qn WATER与跨介质执行 |
| `integration/qn_aav_simulator/scripts/pvs_node.py` | Otter本地查询、动作与支援 |
| `integration/qn_aav_simulator/src/qn_aav_simulator/observation_coverage.py` | `LocalSurveyMap`与局部导航 |
| `integration/swarm_qn_bridge/patches/swarm_peer_safety_contract.patch` | 原生Swarm参考交接、动态邻机与相关修正 |

当前原入口为 `scripts/docker_run_three_class_qualification.sh` → `docker_run_joint_request.sh`。镜像和命令读取总接手文档并核实实际标签／ID；native或消息修改后重建镜像，Python新进程使用当前挂载源。每次源码／运行在 WORKLOG 顶部追加真实结果并同步交接。只使用 `main`；沿用用户授权同步源码和文稿，实验数据／私有聊天／凭据留在相应服务器目录，不进 GitHub。
