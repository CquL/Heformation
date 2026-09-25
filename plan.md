# Heformation 当前实施顺序

唯一运行主链：远域区域扫测请求 → 原候选联合搜索快速选成员、方法、路线及支援时序 → 一份 Plan → 原 runner 并行派发 Action → Swarm AIR、qn 跨介质/qn WATER UUV、Otter/PVS USV → 实际 Result 与母船收到的结果 → 至多一轮缺测修复。固定五平台，允许冗余 AAV 待命，不强制编队。通信只按“USV 共享支援到位即可交付”这一声明的任务级条件评价。

## 在线求解与执行边界

当前任务层按实际收到的 Odometry、模式、成员占用和场景有限航路，在共享 10 秒预算内优先取得一份完整方案；名义时刻只用于候选排序和支援安排。不会把每个候选的 Swarm/qn/PVS 动力学完整预跑一遍。运动可行、终端和实际安全由已选 Action 及运行检查确认，故计划标记 `TASK_LEVEL_EXECUTION_PENDING`，不能称物理安全证书或全局最优。

场景的业务活动是：一台合格 AAV 穿过 6×4 m 空中扫测区的四个观测位置，另一台 AAV 做 AIR→WATER→AIR 点测，UUV 做水下通过段，USV 走南侧航路提供共享结果交付，参与成员返回。四份空中与两份水下结果均须实收。四点几何足迹覆盖该矩形是当前观测代理，不代表真实载荷影像质量。USV 使用 Otter 原生 LOS 航点控制，绿线的转弯和回转是实际动力学，不是 Swarm 路线。

## 当前证据

- `experiments/20260925-area-sweep-live-r15`：默认预算下初次规划约 1.01 秒，同次实时 RViz/中文面板四活动并行，四空中＋两水下产品到母船，跨介质、支援和规定返回完成，`PASS_GEOMETRIC_PROXY_QUALIFICATION`，同次五平台 bag 审计 `passed=true,failures=[]`，最小代理净距约 0.746 m。
- `experiments/20260925-task-level-retest-r13`：此前单空中点模板的一次明确缺测报告实际到达后，0.232 秒内从当前状态选择备用 AAV＋返回的 USV，新 Goal 真正执行、正产品实收、再次返航；同次 bag 安全/时间审计通过。这证明修复链在旧点测范围成立，不能替代新区域扫测的复查验收。
- `experiments/20260925-area-sweep-retest-r16`：新区域扫测的一次明确缺测复查正在实跑，完成前不记通过。

## 下一步

1. 完成 r16，核对负报告先于补测、新成员和共享支援实际派发、全部六结果收件、所有返回及同次独立 bag 审计。若失败只修该执行边界。
2. 同步 README、WORKLOG、当前状态与交接；明确缩比场景、几何观测和任务级通信假设，以及计划估计与实跑事实的区别。
3. 按实际调用清除旧 Calvo v9 专属生产排序、自动回退和退出主链的重复有限字节门控；保留共用模型、原 Action、安全代码与三机/七机控制工具。只删除已确认无消费者的内容，不增加协调层或新封装。

方法依据：[Calvo/Capitán T-RO 2025](https://arxiv.org/html/2411.02062v3) 的低成本估计与执行修复、[D-ITAGS RA-L/IROS 2023](https://arxiv.org/pdf/2209.13092) 的按需运动反馈、[GRSTAPS IJRR 2022](https://journals.sagepub.com/doi/10.1177/02783649211052066) 的交错任务/运动安排；实际空中由 [Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation) 执行，USV 采用 [Fossen LOS/PVS](https://www.fossen.biz/php/research/path_following.php)。这些来源支持职责划分，不提供本场景的严格物理保证。
