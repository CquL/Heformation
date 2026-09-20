# Swarm / qn 双向交接依据（2026-09-20 UTC）

## 转换取消后的分阶段处置候选

已重新读取P17 §3.3、算法1及其解释：转换段以垂直运动穿过转换区，动力学传播可行后才接受；执行段在可行时采用停止/悬停。原文没有提供本项目的取消策略，不能把下面的工程推导称为论文原算法。

本项目实跑已证明转换区内立即固定参考可持续振荡；原qn.slx的chart_912确实按两个作用点水线上下切换浮力，移植并未写错。缺少真实浸没几何依据时，不为通过实验擅改浮力曲线或控制增益。用户冻结合同要求故障行为按模式验证，因此新增**仅资格实验显式选择**的`COMPLETE_ACCEPTED_VERTICAL_SEGMENT`候选：在实际TRANSITION收到取消或规划器确认丢失时，仅沿已接受当前垂直入/出水段继续到其原定介质端点，然后保持；不执行后续业务片段、不更改当前段时间/端点，原任务非成功、资源锁持续。未开始段或实际域违规不能选择此候选。

这复用原控制、原参考和原局部生命周期，不是新增路径算法或新的水线模型。它不是“取消瞬间停住”，必须报告完整退出运动、时间、空间占用和终端；观察超时也不能丢弃本地已经承诺的转换尾段。环境与当前段资格仍需独立验证，不能由文献引用推出安全性。原`FIXED_REFERENCE`仍为默认和对照，AIR固定保持行为不变。

## 原生优化结果的有限性与提交

五实例联调记录的原生轨迹ID6含NaN时长/系数，随后轨迹服务器输出原点参考并触发实际安全失败。源码`lbfgs.hpp`在线搜索失败时恢复变量x，但不会恢复成本回调的`jerkOpt_`副作用；Swarm的OptimizeTrajectory_lbfgs忽略返回码，直接碰撞检查并发布该缓存。这是实际数据与源码共同确定的断点。

参考本地GCOPTER `GCOPTER_PolytopeSFC::optimize`：求解成功后从最终变量重新设置MINCO参数、生成轨迹；失败不提交。七机回归表明Swarm原实时路径还使用未收敛的有限迭代：简单拒绝全部线搜索舍入停止会阻断首条轨迹。因此项目对明确恢复x的线搜索停止及原迭代上限，将返回变量重新计算成本/梯度和MINCO轨迹，有限性、正时长及原碰撞检查均合格才提交；取消或其他错误不提交。这是保留Swarm近似候选语义的工程适配，不称GCOPTER原接口或最优性保证。原生traj_server也在覆盖旧参考前检查维度、正时长和有限系数。未增加优化器、控制算法或任意物理阈值。

这是现有机制的执行接线，不提出新的控制律或控制权框架。

| 需求/约束 | 直接依据 | 实现位置 | 必须验证 |
|---|---|---|---|
| 模式与实际动力学分开 | P17 §3.3 对分域参考进行相应动力学传播，转换不是标签赋值 | platform_execution.DomainHistory，qn逐步记录 | 合法入水保留历史而不标AIR违规，真正越界跨模式仍锁存 |
| 下一个执行以真实完成为前提 | P04 APEX-MR §IV-C：前置事件实际到达/完成后才释放后继 | FormationAction Result→本地owner释放→下一Goal接管 | 仅服务接纳/预测结束不能放行 |
| 普通轨迹最终提交必须受保护 | Swarm ego_replan_fsm.cpp::callReboundReplan/callEmergencyStop、现有reference_mutex_ | plan_manage_reference_handover.patch | 暂停后不发布旧计算结果、过期Goal不触发恢复 |
| qp实际采用与请求来源不同 | 现有CommandSnapshot/ReferenceUsageTracker，冻结修订2.2 | snapshot来源/代次→backend.step→实际采用记录 | 接管请求成功但尚未采用不冒充新AIR参考 |
| 正常交接不消除故障 | 现有不可逆safety_hold与UNKNOWN_LOCKED语义、用户冻结要求 | 同一FSM中独立的普通参考暂停；故障锁仍优先 | 取消/感知故障后TakeReference不能解锁 |

本轮重新读取了本地P17 §3.3及P04异步执行段落。论文约束的是分域/事件执行思想，
不直接规定本项目的ROS消息、互斥锁或代次字段。后者从当前源码的两个可竞争参考流和
最终提交/采用边界推导；不得宣称是论文提供的现成跨介质实现。

复用已有诊断话题传递所请求来源/代次，并由Swarm在提交边界确认暂停或恢复。
CPP确认携带恢复前的原生轨迹ID下界；qp拒绝旧ID，Action仍要等真正新ID被采用。
物理状态、积分时钟、控制器状态和故障历史均不因正常交接重置。

来源：
- P17：research/literature/papers/maritime/hybrid-smooth-transition-2021.pdf，§3.3。
- P04：research/literature/papers/mrta/apex-mr-2025.pdf，§IV-C。
- 源码：upstream Swarm FSM；integration/swarm_qn_bridge/patches/plan_manage_safety_hold.patch。
- 实施规范：用户附件044043ac…/pasted-text-1.txt与docs/requirements/five-platform-implementation.md。

验收分层：原生往返成功、真实暂停/采用、安全与故障证据分别检查。
本轮单台往返的其他AAV保持待命；这不能替代五平台共同作业、非AIR邻机预测、受限通信或最终对照。
# 原生求根退化边界补充（2026-09-20 UTC）

Swarm上游`root_finder.hpp::isolateRealRoots`先计算Cauchy根界，再计算Kojima根界并取较小值。对`10^20*x^6-1`，归一化后只有首项大于现有DBL_EPSILON，Kojima比值向量为空，原实现仍取tail(1)，独立gdb已定位该越界。新增补丁只在比值不存在时沿用已计算的Cauchy根界`1+max|a_i/a_n|`，保留原多项式及后续Sturm求根，不把小系数删掉或宣称无根。没有新增阈值、优化代价或控制算法。

本地GCOPTER副本含相同结构，故本修正属于项目对退化输入的防护，并非声称直接取得作者补丁。解析反例的两个实根应为`±10^(-20/6)`；回归同时验证普通六实根多项式。证据在`experiments/20260920-g1-handover/debug/root-repro-stack.log`及原生编译回归。**这证明独立复现的缺陷，不足以直接证明normal-r2未取得调用栈的ROS断言与之同源。**
