# 完整候选的名义运动与交付校核

本轮依据现有GRSTAPS/D-ITAGS运动反馈关系、APEX-MR实际执行依赖、Guo通信承诺及字节守恒，补上单方法可行不能推出完整计划可行的缺口。统一问题与来源边界见`docs/requirements/cooperation-source-design-20260920.md`。

## 直接接线

既有`_build_complete_candidate_plan`在保存完整叶节点为best之前调用同一provider的`_check_complete_plan`，继续使用本次单调截止时刻。没有新增调度器、节点或对外消息。`ExecutionCandidate`内部保留确有消费者的绝对时刻运动轨迹、机体半径和产品预测；密集轨迹和模型不进入Plan/Goal。Plan只标明`validation_scope`。

按每个已知物理成员拼接真实选定后继，检查时间/位置/介质连续、静态障碍和所有成员成对净距。最后真正空闲区间及未分配的已知PVS成员通过原生idle模型补足；缺轨迹、已承诺前段或相应idle资格直接UNKNOWN，不假定机器人静止。

所有通知与32KiB摘要用一个FiniteDelivery在0.1秒网格重放，只在规定活动区间允许传输，所有信道共用步首前缀。安全尾段预测不能偷偷延长通信支援承诺。接收需要在已检查生产者承诺结束前满足；更一般交付/返回动作尚需后续接线。

## 证据

- 两条合成路线各自无碰撞，合并在同一时刻交叉，整份计划拒绝。
- 两份摘要分别可在1.5秒RF窗口交付，合并共享32KiB/s容量则不足，整份计划拒绝。
- 正式provider缺运动证据返回UNKNOWN；旧抽象调度oracle保留SCHEDULE_ONLY，不能混称物理可行。
- 真实PVS完整查询正例通过、原模型pickle不变；所需USV第8秒可用的方法可延后，但完整计划缺0–8秒已承诺轨迹时正确UNKNOWN。
- 66项相关检查与主任务18项定向复核通过。Noetic水下正式入口`experiments/20260921-complete-plan/water-r1`实际两项SUCCEEDED、母船32KiB接收、覆盖1.0、资源释放。实际计划标记NOMINAL_COMPLETE_PLAN_MOTION_AND_CAPACITY。

## 不能扩大成什么

这是0.01秒运动采样／0.1秒通信的名义模型校核，不是连续时间安全证明。通知身份/时刻编码仍是候选名义值，不能保证与未来实际GoalID编码等长。校核只涵盖传入states的成员；水下入口仍只有UUV/USV，不证明三台AAV待命或整五平台安全。已有未知承诺前段不能凭预测available_from补造。

qn既有原生查询现在也提供实际模型位置推导的逐样本介质、初始边界与原机体半径，单成员水下原生片段通过完整校核；有观测ID时复用LocalObservationWindow产生内部产品证据。AAV待命/完整AIR跟踪与请求中的跨介质/编队/返回方法、有限控制/状态/Result通路和保持承诺的重查修复仍需实现。记录这些限制并继续补全，不将新反例测试数量当最终任务验收。
