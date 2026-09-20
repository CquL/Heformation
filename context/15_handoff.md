# 最新交接：分层监测需求基线与 AAV 本地停止保持

> **2026-09-20 UTC 新交接**：6011944已推送；后续参考交接增量本次按用户要求归档提交main。
> 先读`docs/reviews/reference-handover-progress-20260920.md`。debug-r1往返探针通过，normal-r2原生Eigen断言未定位。
> 当前无运行仿真容器；292项模块测试通过。下一步先复现定位断言、做取消/状态缺失负例与全程安全审计。
> 尚未开放新模式生产资格，未完成G1或五平台总验收；下方“下一关键缺口/未提交”为早期阶段记录。

> **继续工作先读** `docs/reviews/five-platform-progress-20260919.md` §4 与 WORKLOG 顶部。
> 总计划未完成；不要重新实现已通过的G0、原生片段、PVS边界、三机异步基础及Qt入口。
> 最新完整并行正例为g3/parallel-request-r6（实际0.35m/s，有序接近），r3/r4的1.5m/s失败保留。
> 下一关键缺口是Swarm双向参考交接及按授权模式区分健康/极值，之后才接五平台生产任务、预承诺断联与有限交付。
> 独立开发镜像标签five-platform-dev；noetic/safety-hold基线镜像不替换。源码未提交/推送。

> **五平台实施覆盖说明（2026-09-19）**：基线实际已为 `main@3f34df2`；本轮未提交修改正在实现冻结G0–G5。
> 已保存单台ROS原生跨介质片段正例 `experiments/20260919-five-platform-g1/ros-fragment-r1/`；
> 只证明该片段的控制/介质/终端/Action，不宣称完整请求或安全资格已齐备。
> 生产默认仍不启用 PlatformTask。工作日志记录文献、数值实验、构建和测试；
> 当前主线是补模式故障/Swarm交接，再连接PVS和模式调度，不能把新增接口存在当作完成。

RViz 视角问题已修正：原 Tools 列表为空，现添加标准相机／选择／聚焦工具和 Views 面板；已用真实 RViz 滚轮事件验证缩放。当前三机入口读取工作区配置，重新启动或重载即可；本次未重建镜像，七机纯镜像显示入口需重建后获得该配置更新。

> 2026-09-19：三机请求闭环已于 `1840b08` 提交推送。本轮停止保持正负例、完整正常请求、模式切换和七机回归已验证；最新报告为 `docs/reviews/safety-hold-implementation-20260919.md`。本轮代码尚未提交/推送。

> **2026-09-19 最新实施口径（覆盖下方旧阶段记录）**：固定走廊、禁止回退、固定朝向和未校准曝光/模糊/分辨率评分已撤销。
> 观测是请求声明的几何范围/LOS/驻留代理，不能宣称真实图像有效。原未定义 s1/s2/s3 已删除，保留三机集结与转场。
> runner 的 Executor 模式、逐端点客户端、确认门、成功结果后的串行修复、未知结果锁定和任务权威 dashboard 已接线；
> 当前新增实跑结果以 WORKLOG 最新记录为准。模块测试、Action 实跑、完整请求验收仍分开报告。


**2026-09-19**  
**远端基线：`main@79036d66178deacada2b4f4a30dc588899eeb094`**

## 1. 当前阶段目标

把已经分别成立的：

```text
高层请求语义
Executor 规划
多 Action endpoint
单机 / 三机规划
qn 实际运动
观测 / 交付 / 组级判据
```

真正接成一次用户可确认、可运行、可观察的近岸监测任务。

## 2. 已完成

### 交付一：编队与 Action 生命周期

提交：`dc4b0a0`

完成：

- 新增 `DECLARED_FORMATION`；
- 三机配置 `formation_type=2`、`formation_size=3`；
- 三机期望图从声明槽位生成；
- 七机原 `formation_type=1` 分支保留；
- 组级回调复位 `member_goal_active_ = false`；
- 探针改为：
  `单机 AAV2 → 三机组级 → 再单机 AAV1`；
- 每段分别验证：
  - routing；
  - landing；
  - SUCCEEDED + Result；
  - Result 可用于释放。

实跑结果：

```text
single-aav2       PASS
group-formation   PASS
single-aav1-again PASS
```

落点误差约 0.004–0.008 m。

编队参与证据：

```text
declared 3-node desired graph: 3
similarity cost applied: 3
```

不是用“代价值非零”判断。

### 交付二：串行计划、成员预测和交付不变式

提交：`be98c56`

完成：

- `build_executor_plan(serial=True)`；
- 在线串行时钟 `F_serial`；
- 本次不使用的晚可用资源不会阻塞任务；
- offline-only 单元可排除出在线串行时钟；
- 物理成员预计可用时刻；
- 物理成员预计位置；
- 单机先移动后，三机组级转场从更新后的成员位置计算；
- `delivered_fraction` 强制 observed AND received。

相关阶段测试：233 项通过。

### 历史交付三：全程队形与走廊判定（已按最新决定撤销业务门槛）

提交：`79036d6`

完成：

```text
E_form =
max ||(p_i-p_j)-s(r_i-r_j)||
```

整个观测区间汇总。

同时检查：

- 中心走廊管带；
- 相对历史最远进度的回退；
- 到达 path_end；
- 阶段兴趣点观测。

新增负例：

- 旋转；
- 中途散队后重排；
- 走出管带；
- 累计回退；
- 未到终点；
- 样本不足。

当前全套：

```text
240 tests passed
```

## 3. 本轮已解决的边界

同位置动作通过原生静止多项式生成真实新参考；适配会话保留递增原生 ID，避免连续移动任务重用 ID=1。
没有降级为时间戳证明。r4 五段 Action 每段四项均通过；r2 的旧探针穿越待命机安全失败保留。
单机安全现包含其他待命机；客户端未知结果保留物理成员锁，探针失败立即停止后继。

## 4. 当前接线与尚待验收

同一 runner 已实现：

```text
planning_mode=executor
load_request → expand → build_executor_plan(serial=True)
→ unit_for_coalition → 所选单元自己的 Action client
→ SUCCEEDED + Result + 安全/有效性/释放证据
→ 几何观测代理 / 本地结果接收
→ 剩余串行计划修复（实际成员位置）/ 至多一次补测
```

默认 fixed_coalition 保留七机原入口。完整请求 r3 已获用户 yes 并实际运行五段，全部成功；六点几何观测/接收均 1.0，转场实际净距最低 1.135775 m。旧 r2 失败保留。
RViz 与任务仪表盘已同时实时显示；真实相机有效性不在几何代理的完成定义内。

### 用户确认

第一版使用 CLI：

```text
加载请求
→ 打印展开任务、Executor、限制
→ 明确输入 yes
→ yes 才派发
```

runner 本身不要默认启动即自动跑。

### 实时显示

dashboard 读取任务层权威状态：

- 当前 Plan；
- Executor；
- 当前 Action；
- coverage；
- delivery；
- resource occupancy；
- failure reason。

不自己重新计算另一套 mission success。

### 七机 M2

原盒子时间线与局部 scan interruption 已实跑、诊断并存档；原盒子仍实际安全失败，扫描负例保持锁定。详见实施报告。

## 5. 下一步唯一建议顺序

```text
1. 本轮已验收完成，按证据使用当前三机请求入口
2. 后续真实载荷、USV/UUV 或通信能量工作另定范围
```

当前不要扩展：

```text
USV/UUV 动力学
真实通信
并行派发
动态拆队
自然语言
新仿真器
新算法
```

先把三机请求闭环变成一次真实运行证据。

## 6. 实施前审查快照（保留历史，解决状态见顶部）

[完整审查及源码定位](../docs/reviews/framework-interface-audit-79036d6.md)，[纯函数反例输出](../docs/reviews/framework-interface-audit-79036d6-counterexamples.json)。

- 模块测试本轮复跑 240 passed；Action 三段成功仍引用此前记录；完整请求仍未验收。
- Executor 完成反馈不能直接调用七机 `process_completion()`，实测会被七机成员校验拒绝。
- runner 的 `finally` 清除占用必须改为按真实可释放结果处理；单机安全评估还要覆盖其他物理待命机。
- 岸线 s1/s2/s3 缺坐标/权重定义，当前 expand 只产生三项区域单机任务；组级集结与沿岸动作尚未接入计划。
- 区间检查必须绑定起点、稳定/采用证据与已有样本完整性规则；当前中途起计、长缺样和倒序可假通过。
- phase loader 未读三个走廊容差字段；质量 score>0 即 observed，需明确有效质量要求，不能任意代填阈值。
- 同位置 adoption 尚未解决；探针失败后应立即阻止后继派发；零代价时的编队路径计数不能放在 similarity_error>0 内。

继续原冻结路线，不新增算法或接口层；上述项纳入 runner/任务判据接线。七机 M2 仍独立收尾。


## 7. 上一轮三机请求基线验收（已提交为1840b08）

用户已授权处理本轮所有剩余项。完整请求正例：`experiments/20260919-monitoring-request-clearance-r3/`，五段 native SUCCEEDED/PASS/VALID，六点几何观测/接收 1.0，deadline_met=true、formation_motion_complete=true、无资源锁。转场最小实际机体净距 1.135775 m。bag 独立检查确认五个 Goal/Result 身份匹配、实际所选端点、前段结果后才派发下段、九条真实成员目标，所有检查通过。

负例：`experiments/20260919-native-safety-failure-check-r2/`，故意使 aav_1 的真实路径穿越待命成员；返回 ABORTED/SAFETY_FAIL，错误结果进入接收记录，原因明确，服务端/客户端锁定，重叠组级后继未派发。首轮负例测试脚本因空权重在发送前退出，保留其目录，不算 Action 证据。

修正包括：三机 optimizer 避让触发机心距离从 0.75 m 对齐到现有 Action 的 1.0 m；异常终止前保存当次样本；最终安全结论合并在线极值；失败 Result 记录但不释放；首次就绪后短暂失去就绪不再误触发“启动超时”。258 项模块测试通过。此配置只解决已验证的不一致，并非连续时间安全或所有轨迹的理论保证。

仿真已结束，数据与实时帧已保存，不留未确认批次或旧活动目标。入口：`scripts/docker_run_monitoring_request.sh`，有 DISPLAY 自动打开 RViz/任务仪表盘，仍打印计划并等待 yes；VISUALIZE=false 可无窗口。图像质量仍 UNVERIFIED，零延迟本地接收仍为显式假设。

七机 M2 原盒子失败诊断与扫描中断锁定负例已收尾，原盒子不是安全飞行正例。该批修改后来已提交/推送为1840b08，不创建分支。详见实施报告和 WORKLOG。


该轮最终七机原入口回归已通过：`experiments/20260919-seven-final-regression/verification.json`，PASS、0 项失败；后来随1840b08提交。当前停止保持增量另见顶部和新报告，勿混淆两轮状态。

## 8. 本轮停止保持交接

三机默认启用，七机默认关闭新处置语义；恢复只支持整链重启和重新确认。取消返回取消终态／原任务失败，扫描失效返回失败终态；两者均保持成员锁。Trigger 接纳、原生参考发布、qn采用、实际保持和安全证据分别记录。
本轮已验证的整链重启入口是结束并重新启动独立仿真容器及其 ROS master、规划器、轨迹服务器、qn 和 Action；不通过手动删除资源参数恢复。
先读新需求表与实施报告，再用 `scripts/docker_test_safety_hold.sh <模式> <新目录>` 复跑；`VISUALIZE=true` 可显示 RViz，request_cancel/request_normal 还显示任务权威仪表盘。危险负例不等同业务成功，不改写旧原始结果。
最终镜像 `swarm-formation-qn:noetic` 与 `swarm-formation-qn:safety-hold` 指向本轮构建；上一轮镜像保留在 `swarm-formation-qn:baseline-1840b08`。所有仿真已结束。当前修改位于 main 工作树，未提交/推送。
