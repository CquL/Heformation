# 五平台冻结方案：实施进度与未验收项

本轮基线 `main@3f34df2`，新增代码在 main 工作树，未提交/推送。
**总计划未完成，不能声明五平台完整请求已经通过。** 本报告只记录实际完成的增量。

## 1. 阶段状态

| 阶段 | 已完成的增量 | 未完成的出口 |
|---|---|---|
| G0 | 32篇查重；原11篇复用，新13篇有效PDF归档；8篇访问受限/无PDF状态记录；PVS源码复用；直接依据笔记 | 受限论文不能标全文阅读；不阻塞直接工程 |
| G1 | 水下引导入口修正；同qn实例ROS入水/水下直行/出水获得原生SUCCEEDED；水下重复取消得到PREEMPTED且锁定 | Swarm→本地模式→Swarm完整交接；所有模式故障/证据缺失与实际环境安全资格 |
| G2 | Otter/REMUS完整NED/ENU/FRD/FLU边界；同ROS环境两端原生Action通过；Otter原生静态配平终端 | 五个实例在共同场景中运行、原生模式故障与全平台安全、生产请求接入 |
| G3 | 计划快照内成员关系；联合无环；每次共享预算及可终止查询；原子预订；异步AIR派发/修复；受限三机并行请求通过 | 模式/转换/通信/能量成本、有限完整候选搜索；五平台统一派发与资格模型 |
| G4 | 有限字节、共享容量、两跳守恒、断链续传及去重的计算模型和反例 | runner真实接收边界、预承诺会合/有限断联片段、收到结果后复查同运行贯通 |
| G5 | Qt三机请求编辑/实际预览/确认/指定GoalID取消入口；实际Qt点击与关闭窗口后完成串行任务 | 五平台任务输入/显示、完整通信/UI验收、同条件科研对照与条件性论证 |

所有新水下/PVS Action 当前都是**资格实验端点**。生产资格未开放；现有请求仍明确拒绝
未支持的 UNDERWATER/中继要求。不可通过配置一个能力标签把这些未完成出口绕过去。

## 2. 实验记录

以下目录均相对于仓库 `experiments/`。模块测试、原生Action、完整请求分别报告。

| 记录 | 实际结果 | 边界 |
|---|---|---|
| `20260919-five-platform-g1/water-los-before` | 8s模型测量，引导进入次数0 | ROUTE_POSITION提前返回的原行为证据 |
| `20260919-five-platform-g1/water-los-after` | 引导进入2500步；短90°转向末端偏差大 | 不证明任意水下路径可执行 |
| `20260919-five-platform-g1/water-original-r1` | 原始分支测量；前800样本与原行为位置/速度/介质残差0 | 原控制未因接线修正被替换 |
| `20260919-five-platform-g1/transition-los-r1` | 单一实例数值AIR/WATER/AIR；入水有深度超调 | 数值测量，不当ROS结果 |
| `20260919-five-platform-g1/ros-fragment-r1` | 原生SUCCEEDED；实际最终AIR；52.48s模型时刻完成；模型/ROS最大漂移约0.000385s | 未经过Swarm往返或完整请求 |
| `20260919-five-platform-g1/ros-cancel-r1` | 原生PREEMPTED；实际WATER终端保持确认；原任务未完成；资源锁定 | 未证明未知环境安全/全部故障类型 |
| `20260919-five-platform-g2/ros-pvs-r1` | REMUS通过；Otter停推进后约0.0826m/s漂移，超时ABORTED/锁定 | 原失败保留，不放宽0.03m/s终端实验条件 |
| `20260919-five-platform-g2/ros-pvs-r2` | Otter TRIM_PROPULSION在32.71s、REMUS COAST_STOP在62.61s模型时刻均SUCCEEDED | 两个不同终端行为；零流参考模型；不等于船/艇位置保持算法 |
| `20260919-five-platform-g3/parallel-request-r1` | 规划子进程启动失败，无派发 | 修正catkin包装脚本与multiprocessing spawn冲突 |
| `20260919-five-platform-g3/parallel-request-r2` | 三单机实际开始相差约0.01s；请求PASS_GEOMETRIC_PROXY | 运行中编辑shell导致外层退出2；不把launcher退出也写通过 |
| `20260919-five-platform-g3/parallel-request-r3` | 三观测成功；组级集结净距0.491m，失败锁定 | 不能由r2覆盖这次失败 |
| `20260919-five-platform-g3/parallel-request-r4` | 有序单机到槽位和组级集结通过；转场净距0.459m，失败锁定 | 1.5m/s并行完整请求不能认定已合格 |
| `20260919-five-platform-g3/parallel-request-r5` | 配置核对发现要求0.35实际仍1.5，主动终止 | 不计作低速校准证据 |
| `20260919-five-platform-g3/parallel-request-r6` | 实际0.35配置；8段SUCCEEDED；观测/接收1.0、按期、资源清空；外层退出0 | 转场净距最低约0.917m；参考跟踪最大约0.401m，不能声称0.30m误差界 |
| `20260919-five-platform-g5/ui-request-r1` | 真实Qt点击预览/确认，确认前0Goal；关闭UI后runner继续并完成串行请求 | 离屏Qt事件与真实ROS执行；不是五平台UI或桌面五平台展示验收 |
| `20260919-five-platform-regression/seven-single` | 退出0；独立verification PASS，185项检查0失败 | 七机代表回归；原M2盒子失败不被改写 |

0.35m/s来自对已测较大跟踪偏差的**校准候选**，不是论文/设备指标，不构成全范围保证。
该配置改变了分配和耗时，不能与不同速度的串行运行直接比较“调度提速”。
有序集结枚举三成员6种顺序，依据现有机体与净距过滤直线接近其他成员的情况；
真正运动仍由Swarm与qn执行。两个组级Action保留，未删除编队交付。

## 3. 关键代码变化

- `qn_python_backend.py`：让ROUTE_POSITION进入已有水下引导；从最终位置参考差分，
  不把另一个速度字段作为独立输入。节点默认仍采用原始分支。
- `platform_execution.py/platform_action.py`：本机有限片段、参考代次、实际介质和终端观察。
  TakeReference只处理参考来源；未验证的Swarm恢复明确拒绝，不自动清锁。
- `pvs_backend.py/pvs_node.py`：复用原生模型/GNC，完整变换位姿与速度，初始化后不赋值位置。
  Otter静态配平直接解原生恢复力/载荷平衡，考虑原生反向推进映射，未修改PVS动力学或反馈控制律。
- `executors.py/repair.py`：无共享可变overlap表；联合先后关系校验；有界查询不重执行ROS主程序；
  并行修复不修改已接受的运动承诺；不向调用者泄漏试算预测位置。
- `formation_mission_runner.py`：单一任务状态提交边界、异步客户端、有序成员接近候选；
  仍只接AIR/理想交付，未将FiniteDelivery伪装为已经在线启用。
- `formation_aav3.launch`：速度覆盖移动到上游include之后；启动脚本在派发前校验6项实际参数。
- `mission_console.py`：使用同一runner与CLI确认门，状态从任务权威读取；取消使用所显示动作的GoalID。

## 4. 继续实施的顺序与不能绕过的缺口

1. **先补Swarm双向参考交接。** 当前qn端能隔离旧输入，但尚无Swarm普通规划提交的暂停/代次确认、
   恢复后的新目标证明，以及按授权工作域分开的AIR健康/历史极值。现有AIR全程锁存不能简单清零。
2. **补G1/G2故障与终端资格。** 本地状态丢失、转换中失败、实际参考采用缺失、时钟不一致
   必须阻止成功；当前原生Action的终端验证不能替代完整环境安全验证。
3. **建立五实例共同场景与模式注册。** 保持一个物理身份，按动作选择端点，避免仅按成员列表
   查找时把同一AAV的AIR/WATER入口混淆；不能让两个仿真器推进同一AAV。
4. **补模式相关完整候选计划。** 当前新增的是依赖/预算/并行基础，不是完整多模态搜索。
   需要转换后预测模式、经验证动作时间、支援/交付和必要返回成本，明确未知与不可行。
5. **再接G4接收与预承诺。** 本地有限片段目前在接受后开始，尚未实现全参与者已确认的断联许可。
   USV预定位不能依赖必须经自己转发的完成通知；母船必须停止从共享证据文件取得远端业务结果。
6. **扩展UI与完成对照。** 当前UI只能正确操作三机旧请求。五平台输入/数据年龄/通信承诺与
   科研对照必须在对应真实闭环成立后接入，不能只添加显示字段。

## 5. 可复现入口

保持当前工作树源码，普通串行请求与Qt入口：

```bash
cd /home/lhj/Swarm-Formation
TASK_UI=true VISUALIZE=true bash scripts/docker_run_monitoring_request.sh
```

受限并行实验（仍是三AAV、理想交付，需要终端yes确认）：

```bash
EXECUTOR_SERIAL=false PLANNER_SPEED=0.35 \
  bash scripts/docker_run_monitoring_request.sh
```

独立资格实验使用开发镜像，不注册为生产能力：

```bash
bash scripts/docker_probe_platform_fragment.sh experiments/new-qn-fragment
bash scripts/docker_probe_platform_fragment.sh experiments/new-qn-cancel --cancel
OTTER_INITIALIZATION=STATIC_TRIM \
  bash scripts/docker_probe_pvs.sh experiments/new-pvs
```

每次使用新的输出目录。关闭Qt不杀runner；当前恢复策略仍为停止整链并重新确认。
以上入口不能执行完整水下监测/中继请求，不能据此声称G0–G5全部完成。
