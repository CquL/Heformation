# 分层监测需求基线与 AAV 固定参考停止保持

基线为已提交推送的 `main@1840b08`。本轮只更新需求基线与现有 Swarm／Action 执行链；qn 控制器、三机正常作业要求和原安全间距没有改变。

## 需求与实现

[需求—约束—验收表](../requirements/layered-monitoring-and-safety-hold.md) 固定空中概览、水下样点、USV 转发和收到结果后的一次复查。数据关系只用于描述调度依赖，不新增业务识别、传输缓存系统或海洋后端。任务书编成与当前研究配置差异、位置／姿态指标统计口径仍需原始有效材料确认。

本地 FSM 通过补丁增加幂等 Trigger 服务与标准 safety_status。首次有效触发固定保持点；普通目标、FSM、碰撞重规划、fail-safe 和最终提交均检查同一锁存。停止参考同时发布本机和邻机通道，qn 与轨迹服务器持续运行。

CPU 点云看门狗首次任务后持续有效，WAIT_TARGET 不关闭；合法空扫描有效。三机显式使用 3 s 点云超时／0.25 s Odometry 新鲜度，七机默认不开启新语义。

Action 在同一 worker 内继续观察停止，组级先并发请求再汇总。取消与终态提交按 GoalID 串行化；仅取消且保持确认成功返回取消终态，故障／未验证返回失败终态。所有非成功都保留锁，包含该成员的全部单元不可派发；扫描恢复或单独重启 Action 不能解锁。

服务调用使用可终止的 ROS 标准命令客户端进程，三个请求先发出；同一单调截止时刻约束整个阻塞调用与观察过程。默认观察上限 180 s，重复取消／处置不重置。卡住 RPC 负例显式采用 15 s 软件测试预算。软件观察结束不会清除本地保持。

保持使用触发时的真实成员目标，而非原任务终点／固定高度槽位。全部成员确认 qn 采用停止轨迹后，重新累计连续 4 s 模型保持，位置≤0.5 m、速度≤0.25 m/s。原任务、保持、安全与证据充分性分别判定；状态缺失不能报告安全通过。

## 实跑证据

下列目录均位于 `experiments/20260919-safety-` 前缀下，保留 probe.json、原生 Action diagnostics、monitor CSV、日志及 rosbag。

| 用例目录后缀 | 已取得的结果 |
|---|---|
| `single-cancel-r3` | 首个单机运动中取消正例；停止采用、4.01 s 模型保持、取消终态、资源锁定成立 |
| `single_cancel_accel-r1` / `single_cancel_near-r1` | 加速／近终点取消通过，普通目标不能覆盖停止参考，共享端点拒绝派发 |
| `group-cancel-r1` | 三个保持请求并发发出，全组模型保持≥4.01 s，处置约7.179 s，取消终态且锁定 |
| `group_scan-r1` / `single_scan-r1` | 故障成员由 LOCAL_CLOUD_STALE 自主触发，固定参考保持通过，原任务失败 |
| `holding_scan-r1` | 原任务驻留期间中断扫描仍触发本地保持 |
| `idle_scan-r1` | 成功后待命发生扫描失效，旧成功 Result 不变；恢复扫描并重启 Action 后仍不可派发 |
| `group_rpc_hang-r1` | 一台服务存在但不返回，另外两台仍发布保持参考；15 s 预算结束后整体 UNVERIFIED、失败并锁定 |
| `state_missing-r2` | qn 状态缺失，保持 UNVERIFIED，安全 NOT_VERIFIED，失败且锁定 |
| `client_exit-r1` | 取消客户端在 Result 前退出，服务端仍完成保持确认；本地锁存及资源锁继续存在 |
| `request_cancel-r2` | 真实 runner 完成 A 后取消 B，记录非成功 Result 和已验证保持；C／组级后继未发送，实时仪表盘与 RViz正常运行 |
| `request_normal-r1` | 完整正常请求五段成功，几何观测／接收均1.0、资源正常释放 |

实际停止结果以每次 evidence 为准；负例 probe 的检查布尔 true 表示正确识别了预期失败，不表示保持通过。

## 独立时间线与解释边界

`probe_safety_hold.py --analyze <实验目录>` 从原始消息和 evidence 生成 `safety-timeline.json`，分别记录最后有效扫描、锁存、原生参考发布、qn 采用、首次进入保持条件和最终模型驻留；还检查采用后直到录制结束是否被其他参考替换。

组级扫描中断正例中，故障成员检测延迟约 **3.006 s**，锁存到 qn 采用约 **0.012 s**，锁存到首次满足位置／速度条件约 **2.562 s**。从保持触发点起最大位移约 **0.805 m**，从最后有效扫描位置起最大位移约 **4.504 m**。最终全组模型保持超过4 s，停止参考未被覆盖。

这些数值说明：0.5 m 是最终保持容差，不是全过程停止距离限制；3 s 感知超时也不是制动安全距离保证。本轮只是声明场景中的固定参考接管验证，没有新增预测避障，也不宣称任意状态或未知障碍环境安全。

## 修正与历史保留

- 初次探针在 ROS master 就绪前初始化参数失败，未派发；补就绪等待。
- 首次取消运动中发现原编队监测器拒绝实际保持点高度；在现有监测器增加内部显式成员目标，未更改控制器／容差。
- 处置最初误使用30 s启动基线要求，改为已有任务区间时间一致性检查，不能缓存较早的成功后继续等待。
- 运行中的 shell 实验脚本改写导致最后一项未启动，近终点用例已独立重跑。
- 首个 runner 取消控制结果通过但 RViz 用户目录权限失败，显示环境修复后 r2 通过。
- 状态缺失首轮仍给出历史几何安全 PASS，现已把处置期间证据缺口合入最终安全判定，r2 明确为 NOT_VERIFIED。

所有失败目录与原始结果保留，不用后续源码重写历史实跑。

## 最终验证与交接

- 264项模块测试通过；包括旧Goal取消、排队任务取消不派发普通目标、锁存后状态恢复不解锁、服务启动失败不阻断其他成员、卡住调用共享单调预算、采用前计时不继承和真实保持点高度。
- 模式切换五段全部通过：`experiments/20260919-safety-modes-regression/probe.log`。
- 七机代表性回归：`experiments/20260919-safety-seven-regression/verification.json` 为 PASS、failure_count=0。
- 最终构建：`experiments/20260919-safety-final-build/build-final.log`，镜像 `sha256:b3bbb9dc5b805e20756c0adb883daceffdc747b659044324c3823bb61ccfd3c0`。当前 noetic / safety-hold 两个标签均指向该镜像；前一版本保留 baseline-1840b08 标签。软件观察被提前中断只能记未验证，只有预算到期且可靠实际状态仍未满足条件才记保持失败。
- 实时处置图：`experiments/20260919-safety-request_cancel-r2/live-RUNNING-SAFETY_HOLD.jpg` 与 `live-UNKNOWN_LOCKED-UNKNOWN_LOCKED.jpg`；来自运行中的任务权威仪表盘，非回放代替实时验证。

本轮按声明场景验收完成。全部仿真已结束，工作树未提交/推送。恢复只支持重新启动独立仿真容器及其中的整套执行链／ROS master，然后重新确认任务；不提供单独清锁操作。

复跑示例：`VISUALIZE=true scripts/docker_test_safety_hold.sh request_cancel experiments/<新目录>`。正常请求使用 request_normal，其他模式见 probe_safety_hold.py；每次用新目录，原失败和原始数据不覆盖。

GRSTAPS 阅读范围仅更新本轮实际核对的在线章节，不宣称新下载 PDF 或阅读全文；局限性对应[期刊正文 §4.5、§6.1](https://journals.sagepub.com/doi/full/10.1177/02783649211052066)。
