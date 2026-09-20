# Swarm / qn 双向交接依据（2026-09-20 UTC）

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
