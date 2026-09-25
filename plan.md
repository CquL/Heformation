# Heformation 当前实施顺序

当前唯一主链：请求与固定远域任务模板 → 现有完整候选联合搜索 → 一份 Plan → 原 runner/Action → Swarm＋qn／Otter PVS → 实际 Result 与母船收到的观测事件 → 同请求修复。五个平台固定为三台同型 AAV、一台 USV、一台固定 WATER 的 qn UUV 代理；UUV 代理不称为 REMUS100 物理验证。通信在本轮只按“USV 共享支援到位后交付”评价，不做逐字节协议。普通请求可让第三台 AAV 待命，不强制编队。

## 当前验收进度

- **正常协同已实跑**：`experiments/20260924-qn-uuv-joint-live-r11` 在同一 ROS/RViz/中文面板会话中确认一份 Plan 后，两台 AAV 分别做 AIR 概览与 AIR→WATER→AIR 点测，qn UUV 代理巡测，Otter USV 共享支援。四项活动有实际并行段、三项必要结果被母船接收、四成员返回、零锁；任务状态 `PASS_GEOMETRIC_PROXY_QUALIFICATION`，同次独立五平台时间／静态障碍／代理净距审计通过。第三台 AAV 待命。
- **限制**：初次规划用了 300 秒明确诊断预算、搜索未穷尽；默认 10 秒预算仍无完整可派发 Plan。观测是几何代理，通信是任务级支援条件，均不是载荷／设备保证。已触发的一轮复查和同请求反馈修复尚未在这条三类任务中通过。

## 后续顺序

1. 使同一三类请求的一个明确缺测终结报告被母船收到后，原任务程序只修复未承诺部分，派发一次实际补测并完成结果接收与返回。保留其他平台的实际终态、占用和旧 GoalID 防护。失败就保留失败，不将另一次请求的复查拼接为本次成功。
2. 定位完整搜索中重复 Swarm/qn 查询的实际成本；在同一状态、场景、模型和参考条件下复用有限方法证据，保持每次规划共享 10 秒截止时刻。预算内没有完整解时如实报告，绝不以空计划覆盖承诺；不能用距离／速度替代运动资格。
3. 新主链以上两项稳定后，按实际调用关系删除 Calvo v9 专属排序、回退、配置与测试，以及退出生产链的有限字节通信门控；保留共用模型、执行、安全检查、三机／七机控制工具和来源署名。只合并真实重复代码，不增协调器、管理器或通用封装。
4. 用一次带 RViz 的完整请求和一次实际偏差复跑验收，更新 WORKLOG、当前状态与交接。安全、结果因果、成员占用及返回分别核对；默认 10 秒若仍未达成，明确标记未完成。

方法依据：Calvo／Capitán [异构协同调度与执行修复](https://arxiv.org/html/2411.02062v3)、D-ITAGS [分配与运动反馈交错](https://star-lab.cc.gatech.edu/papers/neville-ditags/)、Swarm-Formation [原生编队／轨迹优化](https://github.com/ZJU-FAST-Lab/Swarm-Formation)、Fossen [LOS 航路跟随](https://www.fossen.biz/php/research/path_following.php)及 [PVS 原生船艇模型](https://github.com/cybergalactic/PythonVehicleSimulator)。这些是采用机制的来源，不表示直接继承论文的最优性、稳定性或海上通信保证。
