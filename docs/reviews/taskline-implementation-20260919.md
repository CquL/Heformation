# 最新实施结果与验收边界（2026-09-19）

**最终状态：本轮按用户修订后的三机几何代理请求闭环已验收。** 完整请求 r3 五段原生成功，六点观测/接收均为 1.0；转场实际机体净距最小 1.135775 m，按期完成并释放资源。真实安全负例验证失败 Result、安全摘要、明确原因与双端锁定。下文保留原失败及修正过程。

本次按用户最新决定调整旧冻结计划：删除固定走廊、禁止回退、固定世界朝向以及未校准曝光/模糊/分辨率评分；没有通过放宽阈值换取“有效观测”。

## 保留与删除的依据

| 内容 | 处理 | 依据与边界 |
| --- | --- | --- |
| 固定走廊 ±1.5 m、回退 0.1 m、世界相对向量形状阈值 | 从任务输入及业务判定删除 | 未找到对应作业需求或上游 Swarm 要求；原 Swarm 图指标旋转不变 |
| exposure_s、blur_tolerance_m、nominal_standoff_m 和 q_blur/q_res 简化评分 | 删除，旧字段拒绝加载 | CARIC 原评分依赖真实相机投影/曝光/像素，当前参数无法代表该模型 |
| footprint、垂直范围、驻留、LOS | 仅保留声明的几何仿真代理 | 参数来自当前场景输入；不声称相机标定、图像有效或载荷验证 |
| observed AND received | 保留在评价函数本身 | 接收到未经观测的点不能增加已交付覆盖；零延迟本地结果交付是显式假设 |
| 三机图与运动 | 保留 | 使用原 Swarm 图机制；集结/转场分别以实际到位、参考采用、稳定驻留与成功 Result 完成 |
| 实际编队指标 | 全 Action 区间记录原生 normalized Laplacian 差的平方 Frobenius 范数 | 无量纲，记录最大/平均值；缺样拒绝，未任意指定业务合格阈值 |
| s1/s2/s3 | 删除悬空引用，不补造坐标 | 当前组级阶段是转场演示，不声称岸线载荷观测通过 |

来源： [Swarm ICRA 2022 正文](https://arxiv.org/html/2109.07682v2)、[CARIC 原评分](https://arxiv.org/html/2501.06566v2)。Swarm 源码为 `upstream/Swarm-Formation/src/planner/swarm_graph/src/swarm_graph.cpp` 的 calcMatrices/calcFNorm2。

## 已接线

- 原广播路由按实际话题去重发布，避免按成员重复发同一目标；三机三个目标话题仍各发一条。
- 同一 `formation_mission_runner.py` 增加显式 `planning_mode=executor`；默认保留 `fixed_coalition`。
- `load_request → expand → build_executor_plan(serial=True)`；无端点资源不进入在线时钟。
- 每个 PlanItem 通过实际成员选择单元，使用该单元的 Action 客户端。服务器节点名从端点发布者查询，未新增重复配置字段。
- 物理成员预测位置用于初始计划；收到成功结果后，用实际位置刷新剩余任务转场时间，修复包括不共享成员的串行后继。
- 完整请求显示任务、目标、所选端点、限制后等待 yes。未确认或 EOF 不发送；同批最多一次补测。
- 成功 Result、原生 GoalID、任务/安全/有效性与服务器释放证据全部确认后释放；超时/错误保留资源锁。重启 runner 不能清除已有共享成员锁。
- 单机安全检查包含其他待命成员；同位置目标复用原生静止多项式，连续目标不重置适配会话的原生 ID。
- dashboard 从任务层权威状态读取分配、当前动作、覆盖、接收、占用和失败，显示更新年龄；没有新建 ROS 消息/Action 或控制接口层。

## 三类证据

1. **模块测试**：当前 258 项通过；包括未确认不派发、选中端点派发、超时不释放、不共享成员串行修复、实际成员位置重算、观测/接收不变式等。
2. **Action 实跑**：`experiments/20260919-executor-action-regression-r4/` 五段分别通过路由、落点、SUCCEEDED+Result、释放。三机图节点数及代价路径有三台日志；连续第二个移动目标和同位置目标也确认 qn 消费新 ID。r2 原探针穿越待命成员被整队安全拦截，失败保留。
3. **完整请求当前证据**：r3 五段全部成功，独立 bag 验证通过，见本文最终正例与负例。**旧 r2 失败记录**：用户明确 yes 后，`experiments/20260919-monitoring-request-live-r2/` 已实际执行五段；A/B/C 与集结四段成功，六点几何观测及接收均为 1.0。最后转场实际机间表面净距 0.476381 m 低于既有 0.5 m 下限，ABORTED / UNKNOWN_LOCKED，三机资源未释放。**完整请求未通过**。五段实际时间无重叠，剩余计划修复 5 次；未启动补测。

## 实时可视化与本轮失败修正

此前入口只发布 dashboard 图像，没有打开观看窗口。现在同一请求入口在存在 DISPLAY 时自动打开三机 RViz 和独立 Tk 任务仪表盘；VISUALIZE=false 可无窗口运行。RViz 使用实际 qn 驱动的模型/路径与原规划轨迹，仪表盘只读取任务层状态；关闭窗口不产生任务成功事件。任务返回后保留窗口，关闭 RViz 才结束仿真。

r2 已实际看到 MOVING/HOLDING、覆盖 0→2/6→4/6→6/6、单机及编队占用和最终 UNKNOWN_LOCKED；保存实时阶段帧及桌面截图，见 `live-desktop-running.png`、`live-desktop-final.png`。这些截图是实时显示的存档，不能代替在线状态。旧待命容器的服务已失去就绪条件，保存旧记录后重启，未在旧环境派发。

失败还暴露两个报告问题：在线监测捕获的瞬时最小净距未进入较稀疏的最终 ledger，导致原始 Result 的 safety_outcome 错写 PASS；失败 Result 也未计入 runner 的“已接收”。代码已修为合并原有在线最小值、记录失败终态但保持占用；缺失 peer 数据仍不得冒充安全通过，也不被写成已观测碰撞。新增四个回归用例通过；当时修正仅有模块验证；现已完成本文最终实跑负例，仍不覆盖 r2 原始 Result/metrics。

另发现三机 launch 的 record 参数没有接 recorder。现已复用 rosbag 接入，后续运行记录现有话题；**r2 没有 bag**，证据是原生五段 diagnostics、成员 ledger、runner 日志、任务状态和实时帧。下轮需要用实际参考与状态定位转场越限原因，不能放宽安全阈值换取通过。

## 七机原入口回归

`experiments/20260919-seven-member-regression-r2/verification.json` 为 PASS，0 项失败；三个任务均成功，目标话题共三条消息。
首轮三个动作虽成功，独立验证抓到十三条重复目标；根因是旧子集适配对共享广播话题逐成员发送。该真实冗余已修，失败记录保留。

## 七机 M2 独立结果

- 原盒子没有移动或缩小。T1 的使用中参考净距最小约 0.25 m，对 0.20 m 安全下限只剩约 0.05 m 余量；所声明 0.30 m 跟踪预算未满足。实际首个低于 0.20 m 的样本为 0.196194 m，随后在线锁定。时间对齐、参考采用、AIR 域通过，T2/T3 未派发。**原盒子实际任务仍失败**。
- 局部扫描中断：只关闭 drone_0 渲染器，约 2.983 s 后以 local scan stopped delivering 中止并 UNKNOWN_LOCKED，资源未释放；T2/T3 未派发。故障注入负例通过，不是业务任务通过。
- 证据分别为 `experiments/20260919-m2-original-box-r2/box-timeline.json`、`experiments/20260919-m2-local-scan-loss-r2/scan-loss-check.json`，均保留 bag、原生结果、日志、镜像和工作树记录。

## 未声称完成

真实相机评分、岸线载荷业务、USV/UUV 在线后端、无线中继、通信与能量联合安排仍未验证。当前接口连通与几何代理成功不能替代这些目标。

## 转场净距后续定位与独立复验

r2 转场末端，实际 0/1 号机机心最小约 0.976381 m，即机体表面净距 0.476381 m；同时刻 20 Hz ledger 的最低值为 0.524099 m，证实在线极值不可被最终低频采样替代。使用中参考同期机体净距约 1.026 m，实际跟踪压缩了间距。上游 `swarmGradCostP()` 在 `1.5 * swarm_clearance` 内才激活避让，原三机 0.5 配置只覆盖机心 0.75 m；Action 的两机半径加净距要求机心 1.0 m。已把三机配置对齐到 `swarm_clearance=2/3`，未放宽 Action 0.5 m 下限。

在 `experiments/20260919-transfer-clearance-isolation/` 用相同三机初始队形、终点和 4 s hold 单独实跑：SUCCEEDED/PASS/VALID，最小实际净距 1.180131 m，Result 允许释放，38 MB bag 已保存。这验证该段可安全执行，但**不等于 A/B/C→集结→转场完整序列通过**。后者现已由用户确认后的 r3 完整请求验证；原 r2 失败与安全摘要误报证据保持原样。

## 本轮最终正例和负例

- **完整请求正例**：`experiments/20260919-monitoring-request-clearance-r3/metrics.json` 为 PASS_GEOMETRIC_PROXY；`independent-check.json` 所有检查为 true。录制 bag 独立确认 5 个 Goal 与 5 个 SUCCEEDED Result，GoalID/执行 ID 匹配，3 次真实 aav_2 端点、2 次组级端点，共 9 条成员目标消息；后段 Goal 在前段 Result 后，六点有效几何观测及结果接收均 1.0、deadline_met=true、formation_motion_complete=true、资源锁为空。实时帧随阶段保存，最终 RViz/任务仪表盘截图为 `live-desktop-final.png`。
- **实际安全失败负例**：`experiments/20260919-native-safety-failure-check-r2/check.json` 七项全部通过。故意安排 aav_1 穿越待命机的近距离路径，原生 ABORTED/SAFETY_FAIL；实际净距 0.459533 m，客户端记录失败 Result 和原因，服务端/客户端锁定，后续重叠成员组级任务在发送前被阻断。复用真实 runner dispatch，无人工 Odometry 注入。首轮测试脚本空权重错误发生在发送前，目录保留，不记作实跑证据。
- **故障记录根因**：此前循环在越限判定后先 break，触发中止的样本尚未写入 ledger/CSV。已先保存当次样本，再终止；最终结论还合并在线极值，避免 20 Hz 最终 ledger 丢掉窗口中更低的值。
- **待命修正**：服务首次 ready 后出现短暂 stale 数据，原代码仍与最初 startup deadline 比较而退出。现启动超时只作用于首次 ready 之前；随后不就绪仍拒绝派发、运行故障仍锁定。
- **验证边界**：258 项模块测试通过。完整正例运行后新增的终止样本保存和失败原因展示，在上述实际失败负例中验证。没有修改机体半径、Action 安全阈值、请求几何、服务时长、速度或 deadline 来换取通过。没有证明连续时间安全或通用三机轨迹可行性。

旧 r2 原始失败和错写 safety_outcome=PASS 的 Result 仍保留；不得用后续代码重写原实跑结果。真实图像评分/岸线载荷、USV/UUV 在线、水中跨介质、通信/能量联合安排仍属后续范围。


最终版本七机回归：`experiments/20260919-seven-final-regression/verification.json` 为 PASS、failure_count=0，T1/T2/T3 全部成功。最终镜像及构建日志位于 `experiments/20260919-final-build/`；本轮三机正例、实际安全负例、七机兼容和镜像构建均已完成。所有仿真已结束，工作树未提交/推送。
