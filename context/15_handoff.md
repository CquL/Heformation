# 最新交接：分层监测需求基线与 AAV 本地停止保持

> **2026-09-23 继续点**：本轮直接修改原观测、PVS/qn Action、SceneTransport、runner、请求和完整候选校核。新增终结通知沿原有限链路抵达母船；受控缺测结果须有接收证据才允许任务层释放。正常水下协作港口实跑收到同 GoalID 的 32 KiB 产品和终结报告，状态 `PASS_WATER_GEOMETRIC_PROXY`；另一次真实缺测负例 UUV Action `OBSERVATION_NOT_SATISFIED`、本机物理终态可验证且不锁，母船只收到负报告、没有业务产品。新 `Formation.action` 增本次观测点 ID，已重建 `swarm-formation-qn:joint-wip`；单机 AIR r3 探针实际产生并经 USV RF 支援向母船送达 32 KiB 产品及终结通知。`monitoring_request_joint.yaml` 已声明必要返回；原路及多个转弯候选均被东栈桥安全几何拒绝，完整请求未通过。下一步必须把 AIR 产品/运动查询接入同一联合计划，解决 AAV 10 秒预算、动态完整状态和欠驱动 UUV 合法返回候选，才能进行复查修复与完整 UI 运行。相关文件和失败详情见 WORKLOG 顶部，旧实验失败不覆盖。旧 Calvo 分支、v1 文稿和重复背景文件按用户要求须等新联合业务入口真实接通并查无调用后清理，本轮尚未清除。

> **2026-09-21 UTC 用户正在查看实时系统**：独立桌面终端“最新系统 · 水下协作实时仿真”运行cooperative入口，容器`fb2e98608d6e`，输出`experiments/20260921T052700Z-operator-water-live`。RViz和中文面板已确认存在；最近核对RUNNING、USV/UUV占用，尚未核对最终结果。执行中记录位于容器`/experiments/current`，结束终端后归档；不要擅自重启或关闭，先查实际状态。

> **最新验证**：新增原Swarm硬几何/新peer未来复核后，三机原请求五动作成功、实际净距1.091589m，七机原185项通过；新只读查询二进制也已核对，旧失败保留。完整名义计划校核后的水下实跑通过，qn原生执行区间现有模式/边界证据可进入校核；仍缺完整AAV待命/AIR查询接线、模式选择、返回复查与受限状态消息。方法依据已补细节，当前增量未提交/推送，不声明最终目标完成。

> **本轮方法与约束增量**：完整候选接受前已联合检查所知成员名义运动与共享交付容量；碰撞/容量反例及Noetic水下正式执行通过，仍不称五平台整体或连续安全。qn纯Python同方程优化约24%，代表54760内步状态/命令摘要一致；受控同源编译单例9.70s满足10秒，但未进生产/不代表全搜索满足。新七机185项通过，三机转场却再次0.44164m越限，已定位新新参考组合与硬几何/未来检查缺口；修正编译中，尚无新实跑。方法依据与边界见`docs/requirements/cooperation-source-design-20260920.md`，完整目标继续。

> **本次归档继续点**：查询剖析已完成，session71330终态退出0；`experiments/20260921-qn-query-profile/query.prof`可用pstats读取，主要开销为controller_output_and_derivatives及_unflatten_closed_loop_state。gc冻结候选已撤回，七机回归仍FAIL，不能改成通过。实时operator-preview由用户桌面终端管理，后续先查运行状态，不擅自重启／关闭。

> **参考时间修正最新结果**：`experiments/20260921-reference-time/three-r1`原三机完整请求五动作SUCCEEDED，六点几何交付1.0，转场实际最小净距0.735091m>0.50m；旧0.404m失败保留。七机原生动作成功但兼容采样检查仍失败，GC冻结试验未解决且已撤回，保持原GC策略和时间／积分／阈值。下一步处理模型／发布开销与AIR完整查询，不能声明全回归通过。

> **当前可视化入口已接入水下协作阶段**：`VISUALIZE=true bash scripts/docker_probe_five_qualification.sh <新目录> cooperative`。复用区域候选选择与正式runner，具体计划确认后执行UUV作业／USV支援／母船有限接收。gui-r1及gui-r3带显示已完成两项SUCCEEDED和32KiB接收、资源释放；时间审计通过，但原跨介质审计因本轮没有AIR阶段仍FAIL。母船是固定接收端，不是自主航行平台。三台AAV待命，完整三类平台任务仍未完成。详见`docs/reviews/water-cooperation-live-20260921.md`；旧normal资格入口保留。

> **最新失败定位入口**：`docs/reviews/swarm-readonly-query-20260921.md`与`experiments/20260921-swarm-query/three-r1`。只读Swarm查询和A*边界修正已编译；七机r1及旧镜像对照有ROS样本间隔压缩造成的尾部残差，新七机r2通过。三机转场却有真实净距0.404m违规，不得忽略或清锁；需从该bag参考/实际轨迹定位。当前无运行进程，未晋级swarm-query实验镜像，main未提交/推送，完整目标继续。

> **最新继续点**：`docs/reviews/request-method-generation-20260920.md`。正式task_line已生成区域任务与有限Native方法，水下阶段请求实跑通过；不要再从手写单一方法开始。完整AIR/WATER任务仍保留并会因未实现能力拒绝。query_worker支持增量候选和有限清理，PVS同位置等待使用原积分。后续优先实际状态/信息边界、Swarm/AAV完整查询与正式全请求入口，再完成全局约束、返回/复查/UI及对照。无运行中进程，main未提交/推送，目标active。

> **最新入口：联合原生方法选择**。`docs/reviews/joint-native-selection-20260920.md`记录run-r2：原搜索在Noetic9.05秒内选6方法之一，正式runner执行并完成接收/释放。TravelTimeProvider.cooperative_routes已连接完整原生预测、参与者名义净距和有限接收；PVS允许同完整快照下续算终端等待，错误上下文拒绝复用。下一步将实验方法生成移至正式请求展开并覆盖真正的观测/模式选择；不要再退回只执行固定预设方法，也不要把当前六个名义方法当全请求/全平台最优。AAV预算、Swarm查询、实际状态/消息、全计划约束及完整UI任务仍未完成。

> **最新接线点**：runner现在先确认支援的启动接纳和实际程序推进，再释放作业；`formal-r2`正常实跑与挂起RPC反例通过。`predict_received_products()`可用同一有限链路逻辑评价给定完整预测轨迹，禁止延长已结束占用路径；还需将它和观测/支援候选真正连接，特别保留名义消息身份、全计划并发容量与实际状态快照的边界。复合链也已加入逐Goal接收要求。不要把固定方法运行或名义预测等同完整优化/主任务通过；后续仍按原目标继续。

> **正式worker已实跑**：先读`docs/reviews/cooperative-worker-20260920.md`。同一候选多活动按物理成员分别预测，native终端等待持续实际积分；runner原子预订、全部PREPARED后StartPreparedAction、原生结果和本Goal接收事件均已接入。formal-r1通过这些边界，严格往返audit仍因未执行AIR阶段而FAIL。接下来不要再写手动prepare探针替代任务，优先实际联合候选生成/选择、信息边界与完整请求；AAV查询预算和Swarm只读查询尚未解决。源码main未提交/推送，目标保持active。

> **本轮继续入口**：`docs/reviews/local-products-finite-transport-20260920.md`。原生端点新增选定observation_ids和本地产品，scene_publisher复用FiniteDelivery模拟链路；UUV/USV并发组件实跑和runner接收已通过，主请求并未自动产生这些路径。精确.1秒网格最后改动由r2实际bag重放验证。下一步把观测、支援、交付活动真正纳入候选计划/正式runner，并隔离远端状态/控制消息；AAV完整查询10秒缺口仍保留。没有运行中的容器，main工作树未提交/推送。

> **当前继续点：AAV完整查询预算**。`docs/reviews/qn-motion-query-20260920.md`记录原模型deepcopy查询及10秒UNKNOWN、离线完整54.85秒运动和编译不足的证据。候选枚举已改为先深入取得完整可行链，避免先查完兄弟耗尽预算。后续要解决查询成本、接实际模型快照与Swarm只读查询，再推进联合观测/支援/交付；不要将实验目录编译产物当新默认后端或擅自放宽预算。整个目标active，未完成、未提交/推送。

> **复合阶段结果**：composite-r2真实两段REMUS通过，composite-r1探针输入错误保留。当前代码已改，日志与证据在`docs/reviews/cooperation-implementation-20260920.md`。下一步从联合候选/覆盖与实际产品交付接线继续，勿再重复模型接入或把PREPARED探针当作正式runner预承诺已完成。整项任务未完成，main未提交/推送。

> **2026-09-20 新实施交接**：用户的三类协同实施计划已开始，不再处于暂停。新增deadline=None、step唯一来源、多活动/联合查环、固定步绝对墙钟节拍、有限交付步首快照及原生PREPARED/StartPreparedAction。镜像使用swarm-formation-qn:cooperation（新PlatformTask消息）；prepared-r1、pacing-r1、seven-r1与no-deadline-r1已有实跑成功证据。composite-r1验证同一worker两步原生执行；完整业务请求候选、协调启动、产品事件/有限通信及复查仍待接通。证据目录experiments/20260920-cooperation-implementation，首先读WORKLOG顶部，不从旧三机接线或模型引入重新开始。

> **最新研究交接**：先读`docs/requirements/cooperation-source-design-20260920.md`。用户输入作业目标，平台模式/编队/支援应由优化选择；无业务期限不人工强制。文档逐项给出来源、硬约束、实际协同关系、当前代码断点和减封装位置。仅新增一篇直接相关ICRA2023联盟收益论文，纠正Calvo正式T-RO出处。实施目标仍暂停，未删除编队交付、不引入新框架。

> **本次提交交接**：中文实时入口、港口场景、资源转换及README命令统一提交main；下方“未提交”保留历史口径。下一步仍从时间一致性及G3/G4接线开始，不扩大外观工作。

> **最新用户重点**：完整协同任务，而非继续美化固定动作探针。harbor-r2动作Result通过但独立全段时间审计未通过，见WORKLOG最上条。当前AAV资格为“空中稳定→垂直入水→短水下段→垂直出水”，不等同任意3D穿水轨迹；母船/中继/复查链仍未实现。继续工作从时间问题及G3/G4接线开始，勿把环境显示完成误作总目标完成。

> **中文入口最终结果**：`experiments/20260920-five-live/chinese-r3`固定资格动作实跑通过，7项成功Result，关闭/重开RViz后实际积分继续。短中文标识、原AAV模型及中文动作面板已实机核对；r2失败不删除。当前显示增量尚未提交；完整五平台业务链仍须继续，不从旧三机场景重新开始。

> **用户最新显示要求**：不要在场景堆诊断英文长串；使用短中文名称，AAV继续用原四旋翼。现有五实例脚本增加VISUALIZE=true与终端yes确认，既有dashboard增加中文资格视图。见`docs/reviews/five-live-view-20260920.md`，不是完整五平台请求UI。
> 中文r2有真实时间对齐失败/资源锁定；保留证据，不以物理到位改判。Marker每帧DELETEALL已改为增量更新，r3复验结果以WORKLOG顶部为准。完整G3–G5继续，不能将展示收尾作为总目标完成。

> **2026-09-20 提交交接**：本次归档92f2b1b之后的代码与报告，阶段仍为G2–G3接线，非五平台总验收完成；提交前319项模块测试通过。以下增量按时间保留，“未提交”属于当时状态。
> 下一步以最上方完整候选阶段报告为入口：请求模式展开、AAV完整实际候选及复合派发，随后平台间在线约束、G4有限交付和G5完整请求验收。原始实验包留本地，报告保留证据路径和失败。

> **最新继续入口**：`docs/reviews/complete-candidates-20260920.md`。完整候选计划与PVS提供者已实现，自动计划→现有runner→真实Result通过，319项模块测试通过。
> 下一步连接请求模式展开和复合步骤派发，补AAV完整实际候选；不要重新退回只算距离/速度。未实现复合链目前明确拒绝。正式业务请求/确认、在线平台间约束及G4–G5仍待完成。

> **最新继续入口**：`docs/reviews/runner-native-boundary-20260920.md`。现有runner原生Goal/客户端/状态源/Result提交已接线，run-r2真实UUV边界通过，覆盖未被补造；请求展开/确认尚未在该测试中执行。
> 下一步连接请求模式候选与完整运动查询到ExecutorPlanItem.native_action/native_prediction，不能把所选动作写死在业务Task上。平台间在线约束及G4–G5仍待完成，当前增量未提交。

> **下一步从这里继续**：`docs/reviews/native-motion-query-20260920.md`。PVS完整尾段查询、接纳前拒绝、PENDING取消/预算UNKNOWN及成本对账已完成实证；不再重新从模型接入/尾段常量估计开始。
> 优先让现有正式runner/模式候选消费query结果和预测终态（含等待残余滑行），并补平台间在线约束。不能把后端接纳拒绝视为高层已经修复了任务计划。
> 当前309项模块测试通过，本轮全部进程已结束；代码仍在main工作树，未提交/推送，完整G0–G5目标保持。

> **当前继续入口**：先读`docs/reviews/five-scene-safety-20260920.md`和WORKLOG顶部。五实例带实体/海底场景normal-r1通过；coast-obstacle-r2的UUV尾段安全失败与后续派发拒绝已有实跑，原负例保持FAIL。
> 下一步是平台间在线约束、包括滑行尾段的运动查询/占用及正式runner操作路由，不要再从PVS/共同坐标接入开始。生产资格保护未开放，完整G0–G5目标未完成。

> **继续工作优先读**`docs/reviews/five-common-integration-20260920.md`与WORKLOG顶部。五实例`finite-integration-r2`已真实接通7项动作，完整场景/海洋安全/正式runner仍未完成。
> 新默认开发镜像`five-finite-wire`包含NaN提交修正、float32检查与重新评估有限非收敛候选；七机`seven-wire-r2`通过185检查，五实例`wire-integration-r3`通过原生动作/时钟/域审计。旧失败离线验证已完成且仍FAIL；本轮全部进程已结束。
> 下一步推进G2完整实体场景/海洋包络及全平台安全，再将操作路由接入正式runner与模式计划。不要重新从PVS接入或Swarm往返开始；也不能把固定探针当作G3/G4/最终请求已完成。当前302模块测试通过，新改动未提交。
> `COMPLETE_ACCEPTED_VERTICAL_SEGMENT`明确为转换中的资格处置候选，入/出水取消均到原端点保持后非成功结束，原FIXED_REFERENCE失败不改写。旧镜像通过不能直接当最新镜像通过。

> **当前下一步（92f2b1b之后本地增量）**：先定位`rootfix-cancel-exit-r1`固定z=-0.080293m参考已采用后的持续垂向振荡；不能因正常往返/取消终态而将G1标通过。
> 原qn离线重放入口`scripts/replay_qn_handover.py`；输入`replay-input-with-reference.json`使用原始采用参考，步4886实际状态缺失保持缺失。重放已结束：同镜像`original-replay-container.json`在所有有实际记录的位置残差0，因缺一个比较样本退出1；主机重放另有数值环境差异，不混用。
> 下一步核对qn.slx中水线浮力分支及AIR/WATER控制门控，末20s浮力分支切换80次；尚未证明单独因果或选择物理修正。当前无运行仿真/重放进程。
> 入水取消Result后阶段保留已修正，r2额外4s模型观察与独立审计通过；七机185检查0失败；295项模块测试通过。
> 新root_finder补丁修复有确定反例的空Kojima向量，保留Cauchy界；旧ROS断言尚不能追溯为同一调用栈。新模式仍资格保护，G2–G5总目标未缩小。

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
