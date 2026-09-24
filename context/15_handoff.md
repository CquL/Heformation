# 最新交接：分层监测需求基线与 AAV 本地停止保持

> **2026-09-24 清理后交接**：用户要求停止，已停止本轮 Docker 仿真。旧港口实验 bag/截图等本地产物及 VRX 独立克隆、专用脚本/镜像已删除；下方历史条目中的实验路径不再可读取。当前新远域场景与请求在 `integration/qn_aav_simulator/config/`，初始岸边五平台布局在实时 RViz 可见，完整三类任务仍因原生计划查询失败未派发。新场景、联合搜索和清理改动都在**未提交**工作树。恢复时先读 README 和 WORKLOG 首条，不得把初始截图写成协同运行完成。

> **2026-09-24 可视化最新交接**：用户当前要看实时协同效果，优先运行README顶部`bash scripts/docker_run_three_class_qualification.sh <新目录>`并在具体Plan打印后输入`yes`。最近`experiments/20260924-three-class-live-ui-sync-r1`同一ROS/RViz/中文面板中真实AAV2 AIR＋REMUS WATER＋Otter支援、两结果母船收件、五Goal/Result、七有限命令、返回/零锁与独立时间/安全审计PASS；截图在该目录`rviz-live.png`/`dashboard-live.png`。规划前约180秒平台待命，不是离线回放；下一次启动原面板还显示规划已用时间、复查、状态回执与返回进度。该入口只为三类执行资格临时不提供AAV1水下备选；普通全方法四区域的区B通信/返程顺序仍FAIL，不能冒充最终验收。WORKLOG最新条目与README为准。

> **2026-09-24 四区域全方法继续点**：WORKLOG首条和`experiments/20260924-full-method-four-region-live-r{1,2}`为当前证据。原A/B/C＋水下样点同请求在保留AAV1水下备选时可名义选UUV＋USV；r1真实AAV1 AIR轨迹侵入待命AAV2原0.5m门，最终独立净距最低0.173709m，安全FAIL。排序只作为候选遍历提示后r2改选AAV2、zone_A和UUV/USV实际成功且平台代理安全，但zone_B正产品/终态在USV返程附近才晚约217秒抵母船，而原AIR复合worker在收件前不发返回Goal，zone_C未启动，请求UNKNOWN_LOCKED。仅延长等待没有解决未预订的保持/通信窗口；必须使整计划有限收件预测与执行片段顺序同一语义，或在既有本地Action端点实现真正已接纳返程。原合作组Result提前释放/错误标量刷新已修，审计已动态按实际AIR成员；r2旧bag缺drone1规划话题，整体独立审计不能说PASS。早先单AIR有限状态复查r4及限定配置三类实时正例仍有效，但不能拼成最终全方法同次任务。两项业务/初次预算异步用户答案仍待。

> **2026-09-24 最新有限状态修复交接**：先读WORKLOG首条和`experiments/20260924-air-retest-finite-state-r4/{metrics.json,nominal-plan.json,scene-once.bag,execution.bag,safety-audit.json,control-audit.json,state-claim-audit.json,rviz-finite-repair-running.png,dashboard-finite-repair-running.png}`。在原港口单AIR缺测请求、实时RViz/中文面板中，母船**实收**五成员有限状态回执后3.439秒内重验条件候选，AAV2＋USV复查Action、32KiB结果收件、返回/零锁和三类同场安全时间审计PASS。`r1`缺回执字段零Goal、`r2`30秒保持到期取消锁定、`r3`首次AIR旧参考`INITIAL_HOLD`漏认取消锁定均保留；r4修复不代表普通全方法双区域请求会选UUV，亦不代表三类与复查在同一个请求内已闭合。初次20秒隔离预算、条件备选11.023秒与在线3.439秒分开，用户关于最终业务UUV结果资格和初次规划预算的两项答复仍待。下方r9直读qn摘要说明是已被新有限状态链取代的历史口径。

> **2026-09-24 三类最新交接**：先看WORKLOG首条与`experiments/20260924-three-class-live-current-r1/{metrics.json,nominal-plan.json,execution.bag,scene-once.bag,safety-audit.json,control-audit.json,rviz-three-class-running.png,dashboard-three-class-running.png}`。原港口双区域正式runner、原五物理实例、限定执行单元表下，AIR AAV＋WATER REMUS＋移动Otter同次真实Action/有限收件/返回/零锁、实时GUI和独立整场审计均PASS（3396对齐样本零缺，代理净距最小0.593100m）。本次仍是180秒隔离初次规划，实验表排除AAV水下备选，不称普通全方法自主选择UUV；另一单区域`air-retest-live-r9`虽已真实完成10秒内复查，但不能拼接成同一三类请求。后续解决普通业务UUV结果资格与初次预算政策，并将真实复查扩展到同次三类任务；下方更旧条目保留为历史。

> **2026-09-24 AIR修复交接**：先看WORKLOG与`experiments/20260924-air-retest-live-r9/{metrics.json,nominal-plan.json,scene-once.bag,execution.bag,safety-audit.json,control-audit.json,visual-retest-running.png}`。同次真实AIR缺测负报告经有限链路到母船后，条件候选在线2.650秒重验、AAV2＋USV新Goal/Result、32KiB复查产品母船收件、返回/零锁，实时RViz＋中文面板与独立审计PASS。但**待命AAV完整状态摘要由runner直调同机qn服务**，还没经过有限链路；不能将此同机资格说成通信受限母船状态闭环。首轮20秒隔离诊断规划、事先只读备选11.407秒分开报告，默认10秒初次规划/全局最优未证。普通全方法两区域仍可选AAV水下、UUV待命；测试限定配置的三类实时正例见本文件最上条，两个不同请求不能拼成同次三类反馈修复。下方旧“复查尚未派发”为历史快照。

> **2026-09-24 复查候选交接（本地未提交）**：原`build_request_executor_plan`现能只搜索`retest-overview-0`，保留首轮AAV1为锁定、有限30s条件资格的“opaque hold”物理占用并用原0.5m成员净距校核。`experiments/20260924-opaque-air-retest-plan`同真实AIR负报告/原场景只读：10s共享预算无可提交Plan，20s诊断得AAV2 AIR+USV支援18.04s完整名义方案；首个方法约9.781s、整计划完成约10.958s。尚未在正式runner收到负报告后自动重搜/派发、也未连续监测AAV1保持条件。此本地增量不能推送为最终修复通过；下一步接任务权威与已收到状态，且解决严格10s的约1s缺口或取得用户明确政策变更，不把20s冒充生产。

> **2026-09-24 AIR复查下一步证据**：`experiments/20260924-air-retest-peer-margin/result.json`与`20260924-air-state-replay/result.json`说明当前AAV2空中方法与AAV1已声明0.5m保持球在采样轨迹下有约0.4998m条件余量，原qn固定参考离线续算30s最大偏离0.0411m/速度0.1222m/s。但这是离线/旧同运行证据，不是母船收到的完整执行后状态，更不是下一Plan已表示AAV1占用。下一步如做真实`retest-overview-0`，不得把AAV1从空间约束删除或用初始trim替换；必须在已验证有限时域保留其物理占用、在线监测并对AAV2/USV实际方法重校。超出范围保持UNKNOWN，不添加无依据阈值。

> **2026-09-24 新镜像交接**：当前`joint-wip`镜像`sha256:63e0434c361958e91ee7753ca771925abc6d8e681fddf5b3a8d7ad7075efd003`含0.5m成员净距计划修正及现有状态摘要，私有qn SHA/ABI核对True；受影响177项检查通过。它在之前正式ROS水下组件**零Goal预览**之后重建，尚无新镜像的最终三类/普通完整请求Action全程审计，不能用镜像构建或测试数量声称交付。旧GPU真GUI普通请求180s预算失败零Goal与AIR负报告后`pending_retest`无合格重选仍是当前关键继续点。

> **2026-09-24 成员净距继续点**：当前`executors.py`把原误用实体0.2m的两处成对检查改回仓库既有机体净距0.5m，实体/海底仍0.2m；合成0.3m表面间距反例在名义计划阶段拒绝，相关62项检查通过。正式ROS水下组件`20260924-water-fleet-clearance-preview-r1`20s诊断预算仍得并行完整计划，输入`no`零Goal。别为保留旧计划通过而降低它；三类旧实跑实际最小净距约0.593m仍支持那条实际执行正例，但普通完整能力请求/新计划结果应按新门重验。具体证据/边界见WORKLOG顶部。

> **2026-09-24 镜像/诊断RPC交接**：当前`joint-wip`镜像`sha256:3c7a3729283c375768adb4e4c338bae62ca43a7cf8e091d1cf7385d3ca7b7722`从本轮源码重建、同源qn扩展校验True。单机AIR终态读取本机摘要只在原Odometry0.25s预算内等待，受控挂起RPC不会阻断Action Result；守护线程永久挂起时可能残留，暂不称通用长期回收。镜像构建晚于旧实跑，完整三类同次新源码/新镜像物理验收仍需重跑；GPU真GUI普通请求已有一次180s预算失败零Goal，任务层/安全证据不能凭镜像更新改写。

> **2026-09-24 qn跨介质本机摘要交接**：`experiments/20260924-qn-native-digest-live-r1`原五实例固定资格在同一qn完成AIR/入水/水中/出水/AIR，`native-roundtrip`真实Action SUCCEEDED、终态有效且本机`ACTION_TERMINAL`已有对应GoalID状态摘要；不等于联合请求、母船收件或后继修复资格。普通完整能力GPU真GUI180s诊断规划另在`air-cross-gpu-live-r1`预算失败零Goal，GPU预览成功的旧证据不覆盖它。下步需选定跨介质完整方法在同次业务运行中对照名义/实际摘要；缺测修复、正式10s与用户业务资格仍未完成。

> **2026-09-24 GPU显示不解决正式规划期限**：`experiments/20260924-air-cross-gpu-live-r1`真RViz/中文面板+NVIDIA图形进程下，原普通两区域/完整执行单元/180s诊断规划到期无完整候选，0 Goal、0锁，NTP脚本已恢复active。上一GPU仅预览有Plan和无GUI预览有Plan不能替代这次失败；不能称新增qn跨介质终态摘要通过实跑。正式GPU开关已加入脚本及README，仅改变渲染；继续从受限初次预算与业务资格两项用户答复、缺测后实际方法重选和运动中本机查询推进。

> **2026-09-24 GPU渲染交接**：`scripts/docker_run_joint_request.sh`在可用NVIDIA容器环境增加`JOINT_GPU_RENDER=true`，只让RViz图形使用GPU；默认软件路径不变，qn/PVS和求解仍CPU。`experiments/20260924-gpu-rviz-plan-r1`同原普通两区域/180s诊断预算带真GUI得到完整名义Plan但输入`no`零派发；对应软件GUI一次预算失败、无GUI一次计划成功，三次结果均在WORKLOG顶部。当前只能说GPU图形入口可用且一次预览有解，不能称普通完整请求实时物理任务或10s规划通过。下一步实跑时需重核模型/ROS时间、净距、Action和有限收件，不拿预览替代。

> **2026-09-24 qn状态交接**：先看WORKLOG顶部、`experiments/20260924-air-state-digest-live-r1/metrics.json`、`20260924-qn-state-service/result.json`。原qn节点新增只读Trigger返回小量实际完整状态摘要，单机AIR Action有界获取并经现有有限终态通知传到母船；原名义候选也有摘要。真AIR+USV业务仍PASS，但两AIR步骤真实摘要均不等于查询预测，runner正确撤回旧名义完整计划资格，不能据Action到位直接重搜复查。静态qn状态跨进程同值已验证，第一AIR终态离线bag重放吻合；第二AIR段缺一参考样本且母船不能用bag真值。新跨介质本机终态摘要尚未ROS Action复验，普通完整能力/10s初次规划/已触发复查仍未完成。

> **2026-09-24 qn状态继续点**：先看WORKLOG顶部和`experiments/20260924-air-state-replay/result.json`。同源qn用已录采用参考离线完整重放正常AIR Action，记录位置/速度残差0；但高频参考只在bag/本机评测中，母船有限链路没有它。不要借此给真实AIR缺测后`retest-overview-0`分配一个由初始trim重置的AAV1状态。要么本机提供可传且可核的状态条件，要么保留该成员未知/锁定并验证其它成员与其空间占用；现有UUV复查路线仍碰quay，不能靠重派旧Goal完成总目标。

> **2026-09-24 水下收件顺序交接**：`experiments/20260924-water-report-regression-r1`在最终runner代码下，UUV/USV真实Action和32KiB母船产品接收完成；产品在1790238738.7先到，终结报告在1790238795.6后到，UUV活动在1790238795.987664才提交。两Result/四下行/两有限终态、返回/零锁、两个PVS终态摘要匹配与同bag控制因果均通过。这只证明正常水下组件收件不提前释放，不证明AIR缺测复查或三类最终同运行。继续从下方真实AIR缺测`pending_retest`和已知REMUS复查碰quay反例推进，不再从消息是否到达重新排查。

> **2026-09-24 复查继续点**：先读WORKLOG顶部的`20260924-air-missing-feedback-r1`真负例。测试专用本机传感失效使AIR两个原生Action/USV支援都完成，但母船只收缺测报告、不收产品，任务层生成一个`retest-overview-0`，并因缺合格重选Plan明确FAIL/零锁。消息因果与AIR本机负报告提交已接好；当前断点是真实终态与全部物理成员占用进入下一次联合求解。REMUS原水下终态按现有有限路线直接或低速重测碰quay；继续原零推进等待安全，但不等于可重测。下一步不能再把`retest_tasks`测试或旧三机重测当成五平台修复通过，要取得有效入口状态并派实际备选方法。普通业务资格与初次预算用户选择仍待。

> **2026-09-24 缺测收件修正交接**：WORKLOG顶部/来源审计新节说明`formation_mission_runner.py`现在等本机终结报告与其全部正向产品实际到母船、双向拒绝矛盾消息，并且AIR几何可见不再覆盖本机负报告；错误报告在提交PlanItem终态之前拦下。复合AIR可返回而报告`OBSERVATION_MISSING`，正常AIR+USV正式组件`20260924-air-report-regression-r2`在最终源码下实际PASS，相关61项检查通过。**还没有触发缺测后的同请求联合重搜和Action实跑**；REMUS原终态直接/原环路复查候选不安全，保持未资格。不要把本轮收件因果修正写成最终复查完成。

> **2026-09-24 终态摘要与复查反例交接**：先看WORKLOG顶部和`docs/reviews/source-and-assumption-audit-20260923.md`新节。原PVS按值状态字节与母船有限终态摘要已接到所选方法预测摘要；正式UUV/USV组件`experiments/20260924-water-component-digest-action-r1/metrics.json`实际母船两项`nominal_terminal_state_match=true`，双原生Result、有限收件、返回/零锁成立。旧三类长实时`three-class-terminal-digest-live-r1`任务/独立全场审计PASS，但启动于计划摘要字段接入前，只有同路线离线三段摘要对照成立；不能宣称三类新字段同次在线已验收。从匹配REMUS终态尝试三条现有路线衔接，均在quay实体余量0.196791m失败，原零推进尾段仍安全。下一步不能盲发同一水下任务作复查；要选择合格新方法或经验证AAV备选，并维持母船收件与实际入口资格。普通完整能力请求、10s初次规划与触发后一轮复查仍未通过。

> **2026-09-24 最新三类实跑与状态边界**：先看WORKLOG顶部、`experiments/20260924-three-class-state-guard-live-r1/{metrics.json,safety-audit.json,control-audit.json,dashboard-terminal.png,rviz-terminal.png}`及`docs/reviews/source-and-assumption-audit-20260923.md`新节。原PVS预检已加完整内部状态签名提交/预装启动核对；三种陈旧/缺资格受控负例正确拒绝，Otter普通和预装实际Action成功。新同次三类实时运行两业务收件、四Result、六有限指令与返回/零锁，独立全场时间/安全及控制因果均PASS。仍只是一条原港口**测试资格配置**正例，180s诊断预算、几何代理、AAV水下备选未参与；普通完整请求仍选AAV替代、复查和未来运动状态查询未完成。`systemd-timesyncd`已恢复active。最后补的无签名拒绝支路只有受控负例，长运行不冒认覆盖。

> **2026-09-24 面板交接**：从现有Plan的`coalition`计算短中文待命说明，不增第二状态源；普通预览录制状态由同面板函数静态重绘可读，`experiments/20260924-dashboard-standby-render/ordinary-preview-panel.png`。尚未随新ROS任务实时刷新核验。三类完整请求、复查及预算两项用户选择仍未由这项显示工作解决。

> **2026-09-24 普通请求方法选择复核**：`experiments/20260924-ordinary-full-preview-r1`用正式ROS、原两区域与**完整**执行单元配置，在180秒诊断预算下选AAV2空中/USV支援＋AAV1跨介质水下/返回，UUV待命；输入`no`所以零Goal，不是最终三类实跑。此前限定接线三类实时正例仍有效，但不能替代普通请求的业务资格和最优方法选择。WORKLOG最新条目位于文件顶部，上一轮日志排序已纠正。两项用户业务/初次预算选择仍待，复查/实际在线状态修复未完成。

> **2026-09-24 待命全状态诊断**：`experiments/20260924-native-idle-equivalence/result.json`从相同原生初态和180s待命后的真实模型副本查询原REMUS/Otter路线：内部对象不逐字相同，但此两条方法的名义轨迹相同/约`10^-16m`差异。不要将这个有限等价当作“Odometry位置足以重建所有后继方法”的通用合同；在线复查或换方法仍须本机真实入口资格与母船信息边界。


> **2026-09-24 最新继续点**：WORKLOG 顶部记录只读 qn/Otter 精确固定点、独立 PVS 查询并发、REMUS 同步重复快照消除及完整计划同输入谓词/收件重放去重；没有改变在线动力学或任务条件。相关 60 项检查通过，但正式原 10 秒 ROS UUV/USV 组件 r4–r8 仍预算耗尽、零派发；r8 增到六个独立物理规划核心也未解决。20 秒预算预览 r1 虽得并行完整计划（18.3091 秒），输入 `no`，不是实跑/默认预算通过；隔离10秒约10.0140秒返回亦超严格墙钟。原限定配置三类同请求实时 RViz/中文面板正例依旧是 `20260924-three-class-live-stable-clock-r3`，其 180 秒隔离规划、UUV 水下替代资格排除、几何代理与未触发复查边界不变。下一步先处理普通完整能力请求中真实 UUV 业务依据与初次规划预算政策（两项异步用户答复仍待），同时只沿现有 worker 补缺测复查与有效未承诺修复；不要从零重建场景或新增层。

> **2026-09-24 最新三类实跑交接**：优先看WORKLOG顶部与`experiments/20260924-three-class-live-stable-clock-r3/{dashboard-terminal.png,rviz-terminal.png,metrics.json,nominal-plan.json,safety-audit.json,control-audit.json,execution.bag,scene-once.bag,clock-service-before.txt,clock-service-after.txt}`。测试配置只暂不提供AAV水下备选，原港口两区域请求/原样点/回区不改；正式`joint_request`同次真实完成AAV AIR＋REMUS WATER＋移动Otter支援、两32KiB有限收件/5成功Result/三方返回/零锁，实时中文面板/RViz及独立整场五平台时间安全/控制因果均PASS。宿主NTP曾造成同刻ROS墙钟回拨负例，这次有界实验临停并恢复原active；同源Cython qn显式启用且源码哈希/ABI核对。**不要**把此接线正例说成普通完整能力优化器选择UUV、10s生产初次规划、真实载荷质量、已触发复查或完整在线修复。前两次时间和协作时序失败记录仍在，下一步取决于用户两项业务/预算选择以及缺测复查实现。

> **2026-09-24 三类同请求时间阻断交接**：先读WORKLOG顶部。原请求/原障碍、仅实验执行单元表不提供AAV水下候选时，正式联合求解器给出AAV AIR＋UUV水下＋移动USV完整候选，真RViz/中文面板实际打开；但360s诊断规划期间三qn模型/ROS漂移峰值约1.73s，AIR就绪原0.05s门拒绝，AAV零Goal、请求UNKNOWN_LOCKED，UUV/USV完成与水下32KiB收件不能代替AIR缺项。缩至180s仍给出完整计划，但规划期峰值1.719s，输入`no`零Goal。失败图`experiments/20260923-three-class-integration-r1/dashboard-failure.png`、bag审计和`three-class-plan-180-r1/safety-audit.json`保留。私有同源Cython独立20模型秒只从5.00s改善到3.44s，未证明可消除该ROS漂移，不直接切在线控制。初次离线规划政策及代表性UUV业务资格两项异步用户答案仍待，别以重置时间、预先手工派Goal或强制平台动作绕过。

> **2026-09-23 水下实时组件最新交接**：先看WORKLOG顶部和`experiments/20260923-uuv-usv-joint-live-r1/{dashboard-running.png,dashboard-delivered.png,rviz-running.png,rviz-meeting.png,metrics.json,safety-audit.json,control-audit.json}`。同一正式`joint_request`的原样点UUV工作＋预承诺移动USV支援、有限32KiB产品与双终态通知收件、双方返回和锁空任务PASS；RViz/中文面板确由同ROS会话实时驱动。独立五平台**全程**时间0.0149/0.0151s、执行期位置零缺、代理净距0.5930m和场景云合格；通用审计只因本组件未执行AIR而报两条AIR证据缺失，文件不改。普通请求仍因现有任务条件选择AAV替代而让UUV待命；最终同一请求三类协同/复查/在线10秒规划仍未完成，用户对UUV特有业务资格的异步选择待答，不能强迫普通请求多平台出动。

> **2026-09-23 原港口UUV方法新资格与同请求组件实跑**：先读WORKLOG顶部、`experiments/20260923-remus-segmented-action-r1/result.json`、`20260923-usv-precommitted-action-r1/result.json`及`20260923-uuv-usv-joint-component-r1/{metrics.json,safety-audit.json,execution-only-time.json,control-audit.json}`。用户方案2原样点/回区未改：REMUS一条本机原生分航段500/300rpm方法实际重进0.805621m区并安全低速终结；Otter本机预承诺200s配平等待后移动/返区实际SUCCEEDED。正式runner组件请求在同次ROS下UUV+USV两活动、有限产品/终态母船收件和返回PASS，资源锁空。**全程**bag时间审计因规划期0.7575s漂移FAIL，执行区间原判据PASS、代理/实体净距PASS；AIR缺证据是组件无AIR作业。普通原请求仍会选AAV跨介质、UUV待命，不得把组件当最终三类同请求/实时可视化验收。业务差异异步待用户答；初次规划预算另一项仍未答，默认10s继续失败。

> **2026-09-23 用户已选原港口几何方案2**：不要再等待“能否另建REMUS示范点”答复或改原业务点。只用原`water_sample=(0,8,-2)`和REMUS回区`(-5,8,-2)`继续找原生可行方法；原半径0.805621m及静态安全不放宽。`experiments/20260923-remus-original-return-search`有限PVS路径/转速/航向核查仍无观测＋重进＋安全全满足：最接近轨迹1.252803m且后撞quay。详见WORKLOG顶部。这不证明全空间无解，也不构成ROS UUV实跑。初次规划预算的**第二个**异步选择仍待用户回答；下方旧条目凡写“示范几何待答”均已过时。

> **2026-09-23 旧排序入口交接**：`MissionRunner`缺省模式曾静默进入Calvo v9固定联盟排序；现无模式即报错，`docker_test_qn_formation_action.sh`和七机旧launch显式指明`fixed_coalition`作为控制回归。正式`joint_request`完整候选入口和无回退语义不变。`schedule.py`仍有真实回归调用，不能因清理旧算法目标而连带删除；详见WORKLOG顶部与源/假设审计。

> **2026-09-23 并行修复交接**：`repair.py`非串行Result处理不再用任一事件实际结束时刻统一推迟全部待派项；只沿相同物理成员和已声明活动前置传时间。AAV迟到而独立UUV可从0执行的确定性反例及相关76项通过，见WORKLOG顶部。已接受活动、旧Result锁定和串行入口语义不变。此补丁不产生新的运动资格/在线完整状态，也未实跑缺测重选，不要把时间传播修正称最终反馈修复。

> **2026-09-23 原水下任务UUV方法交接**：本机`experiments/20260923-remus-heading-study`保持原港口业务样点和返回球，只用已有PVS初态航向＋少量场景预检路线查REMUS通过式观测/返回。−0.22/−0.30rad两条真实模型轨迹已满足几何观测和静态障碍/原生尾段，但实际未重进0.805621m部署球（最近4.865/5.430m）；其余有限路线在声明实体/禁区失败。`advance_path_target`投影式切航点使回区目标生效过晚。没有修改配置或派ROS Goal，不能称全空间无解或UUV已参与完整请求。等用户此前对另建示范几何的异步决定，再衔接已有开阔水域双会合只读候选；详情与负例见WORKLOG顶部。

> **2026-09-23 最新实时可视化交接**：先看WORKLOG顶部与`experiments/20260923-joint-result-uplink-live-r1/{dashboard-terminal.png,rviz-terminal.png,metrics.json,safety-audit.json,control-audit.json}`。正式MissionRunner`joint_request`同一Noetic/Docker港口场景中，RViz/中文面板实时显示选定AIR＋USV与AAV跨介质两区域任务；九子Action、10/10有限下行命令与10/10有限终态通知、两份32KiB收件、规定返回/零锁，任务权威PASS。同次单条静态云+动态bag的五平台时间/安全独立审计PASS（3262位置样本0缺，最小代理/实体余量1.46390/1.90108m，最大模型/ROS偏差0.02882s），Goal摘要/命令先送/Result通知独立对账PASS。UI终态真窗口显示完成与64KiB母船收件；“指令已达10/10”当时因精简任务状态漏传没显示，现只补两个计数，尚未下一次live截图复验。普通请求的UUV待命，几何代理/声明初态/360s诊断预算边界不变，**最终三类平台业务、实际复查及10s求解未完成**。继续依据用户待答的REMUS示范几何和预算策略推进，不重做已有AAV+USV正例。

> **2026-09-23 最新交接（无GUI结果上行全链）**：先读WORKLOG顶部，查`experiments/20260923-joint-result-uplink-nogui-r2/{metrics.json,safety-audit.json,control-audit.json,execution.bag,scene-once.bag}`。现有MissionRunner在原两区域请求下，用360秒**隔离诊断**预算取得10/10有限下行命令、九段原生Action、10/10有限Action终态母船通知、双32KiB几何产品收件、AAV1/AAV2/USV规定返回与零锁；同次独立五平台安全/时间审计及bag控制因果核对均PASS。返程第4段先前0.54s Action监测空档的r1 INVALID/锁定保留；只将例行CSV flush移出20Hz监测循环后r2该段缺样0、VALID，不据一次正例声称任意负载保证。下一步必须在**同一实时RViz/中文面板**下复验同范围证据，然后继续UUV实际参与、真实缺测复查、在线内部状态修复和10秒预算问题；无GUI正例不能冒充最终五平台协同任务。

> **2026-09-23 时间失效与早期失败交接**：r3新下行版首AIR Goal以0.07116s模型/ROS漂移INVALID/锁定，qn0/1当时固定逻辑0/1但它们是同物理核心超线程；r4把qn0/1/2改放不同物理核心0/2/4，仍在Goal前因AAV1基线0.0585s>0.05s拒绝。`joint_request`早期异常写状态因`weights`未初始化遮蔽原原因，已在代码把任务权重提前到就绪门前，针对性及相关49项检查通过；r4原现场不改写。纯qn模型热路径10模型秒约1.90s墙钟/P99外层步1.99ms，只证明孤立计算有余量，不证明ROS无调度抖动。下一步保留原时间判据找真实原因并复验全程审计；细节和证据路径见WORKLOG顶部。

> **2026-09-23 当前继续点**：受限下行Goal送达已接入现有`scene_publisher`/`FiniteDelivery`和正式runner；`20260923-command-gate-r1`组件实跑先收命令后发原生Action。`joint-command-gate-live-r2`在同一RViz/中文面板运行中10/10命令回执、9 Action验证、2摘要收件、AAV1/AAV2/USV返回、资源释放，**任务层PASS但独立整场审计FAIL**：模型/ROS峰值0.13555s、跨平台0.13567s>原0.05s，定位见`drift-probe.json`，其余场景/位置/净距/参考采用通过。r1因AIR动作0.08290s超时差门槛锁定。先查模型时间落后，不放宽阈值；然后才称同次完整运行有效。当前UUV待命、缺测复查和正式10秒规划仍未完成，用户示范几何及预算策略异步选择仍待答。细节见WORKLOG顶部和源/假设审计。

> **2026-09-23 长Goal实跑交接**：原港口Otter/PVS同位置静止配平Goal的`terminal_wait=185s`、`execution_timeout=230s`已实际运行约192.28秒并获SUCCEEDED/终态已验证/资源未锁，19,230条状态样本、部署误差0.037713m；证据`experiments/20260923-pvs-long-timeout-r1/{result.json,execution.bag}`及WORKLOG顶部。只证明本机PVS期限不再隐截180秒，不代表UUV长航线/五平台请求完成；原REMUS港内失败及异步几何选择保留。

> **2026-09-23 搜索补充**：`iter_execution_candidates()`已让AIR/跨介质原生候选通过现有有界子进程流逐个返回，不再等待同单元剩余方法列表；34项相关测试通过。`experiments/20260923-joint-stream-10s-r1`真实正式入口在原10秒预算下仍明确无完整计划、零Goal、零锁；不能宣称在线预算达标。预算选项的异步用户问题仍待答，见WORKLOG顶部。

> **2026-09-23 最新补充**：用户确认REMUS只需“观测后重进部署区→完成原生安全尾段”，不是定点悬停。现有港口原样点/部署区继续保留失败；另一组内存试验几何有原PVS/静态障碍/有限交付的双会合只读名义候选，具体数值与失败点见WORKLOG顶部和`experiments/20260923-remus-two-contact-study/result.json`。该实验曾误用0.2m障碍余量做平台间门槛，现用原0.5m门槛重算，最终采样富余1.45295m；不代表ROS Action或五平台验收。`pvs_node.py`外加180秒硬截断已去掉，总请求结项改用匹配GoalID本机Result判断UUV通过式返回；相关15项PVS和32项runner测试通过。长Goal实跑、第二会合下行接纳、复查与10秒规划仍待解决。示范几何及预算策略的异步用户问题未答，未改生产业务配置。

> **2026-09-23 当前最强交接与下一步**：先读WORKLOG顶部，打开`experiments/20260923-joint-runner-live-audited-r1/{dashboard-final.png,rviz-water.png,metrics.json,safety-audit.json}`。同一正式`joint_request`运行的AIR＋USV→AAV跨介质两区域任务在360秒明确隔离规划、具体确认后实际9子Action成功、双32KiB收件、AAV1/AAV2/USV返回、锁空；中文任务面板/RViz在同ROS会话实时显示，最终面板已正确表述完成及“本次未触发异常复查”。独立审计使用同次`scene-once.bag`与`execution.bag`全项PASS（3227五平台对齐样本、0缺失、最小代理/实体净距1.46354/1.90067m）。项目代码从节点声明配置构造初始模型并以Odometry核位置，但尚不能重建运行中全部控制器状态，所以仍称**固定场景几何代理资格仿真**；正式默认10秒规划未达（此轮360秒诊断），UUV普通请求待命。继续工作顺序：等待用户对“另建REMUS可执行的明确示范样点/部署回区并保留现有负例”的异步答复；同时独立推进真实缺测报告后的一轮合格重搜/Action和规划性能/在线状态查询，不用本轮普通请求替代最终三类平台验收。启动命令与回归边界见README和源/假设审计。

> **2026-09-23 正式联合入口交接**：优先读WORKLOG顶部和`experiments/20260923-joint-runner-entry-r4/metrics.json`、`20260923-joint-runner-live-r4/{metrics.json,dashboard-running.png,dashboard-water.png,dashboard-final.png,rviz-running.png}`。现在有可运行的`joint_request`模式，六端点配置在`integration/qn_aav_simulator/config/joint_request_executors.yaml`，可视化命令在README。正式runner无GUI的r4于330秒隔离规划后由原Action完成AAV2 AIR＋USV支援、AAV1七步跨介质及两区结果、返回/锁空；GUI核组隔离的r4在360秒诊断预算下也完整执行并有真窗口画面，r3 GUI因Action监测缺样INVALID/锁定需保留。当前GUI最终截屏展示2结果/100%/无锁，但旧面板文字错误，现已更正映射，未在修正代码下重复ROS实跑。规划器仍从声明初始化模型建候选，只用实际Odometry核起点，不具任意运行时内部状态只读查询；10秒正式求解、UUV合格回区参与、真实水下缺测复查均未完成。`retest_tasks`现可表达水下缺测复查需求，复合worker在母船收到匹配负报告时会继续已接受安全返程，但完整实际负例及重搜未验收。异步用户问题仍待答：是否允许为REMUS另建明确声明的示范样点/部署回收区并保留现有失败；未答前不移动业务点或扩大半径。

> **2026-09-23 最新交接：同请求实时仿真子目标达成，最终三类平台目标继续**：先读WORKLOG顶部和`experiments/20260923-full-joint-live/{metrics.json,joint-result.json,independent-audit.json,dashboard-running.png,dashboard-water.png,rviz-running.png,rviz-water.png}`。同一Noetic运行中AAV2 AIR＋USV支援、AAV1七步跨介质真实执行，9个子Action成功、两份32KiB结果到母船、三活动COMPLETED、交付1.0、AAV1/AAV2/USV返回合格、锁空；中文任务面板与RViz在同一会话实时更新。独立bag3229对齐样本0缺失、最小五平台代理净距1.46309m，原审计因未录静态云发布者而总FAIL，Goal前云hash匹配声明实体。该运行使用300秒隔离规划，UUV待命且无异常复查，**不能称最终三类平台系统完成**。Plan已新增AAV活动级实际Result先后边；同次运行后发现实际AIR延迟需将validation_scope降级，代码/定向测试已补，但新标记尚未在ROS实跑复验。REMUS现有(-5,8)→(0,8)样点回部署区受东栈桥与原生转弯包络阻断，用户已收到是否允许另建明确示范请求/部署区的文本选择题；答复到来前继续推进独立的预算、修复与可视化工作，不自行更改业务点或回收半径。

> **2026-09-23 最新继续点：七步AAV跨介质返航已实跑，整项任务与实时画面仍在推进**：先看WORKLOG顶部及`experiments/20260923-aav-return-detour-action-r2/{joint-result.json,metrics.json,independent-audit.json}`。r2七个Action均成功、母船收32KiB水下产品、返部署区0.09133m、锁空；实际AIR→AIR偏航连续消除了r1的海面下沉反例，五平台bag对齐2560样本0缺失、最小代理净距1.46429m；因bag没有静态云发布者信息，原审计总FAIL但Goal前云hash与声明场景相同。`executors.py`现从已声明南侧出程点逆序原生查询返程，不增算法。完整两区域r6用旧直返曾在西栈桥实际净距0.19576m<0.2m而正确失败；下一步用新返程再跑**同次AIR＋USV支援→AAV1水中→返回**，并打开现有中文RViz/任务面板记录真实状态。生产10秒预算、UUV回区/参与、缺测复查仍不通过。

> **2026-09-23 最新交接：AAV跨介质方法已真实执行，整项协作仍未完成**：先读WORKLOG顶部和`experiments/20260923-aav-cross-medium-action-r12/{posthoc-assessment.json,metrics.json,independent-audit.json}`。同一五平台港口实例中，AAV1的南侧两中转＋入水入口三段AIR、同一qn水中通过式观测/出水、AIR返部署区共五个Goal均SUCCEEDED且worker验证；母船收32KiB产品与终结通知，计划项COMPLETED，锁空，实际返航误差0.08069m<0.5m。独立bag 2143个五平台对齐位置样本0缺失、最小平台代理净距1.49856m；因bag不再重复录静态全局云，原审计缺发布者证据而总FAIL，同次Goal前云哈希匹配声明场景。诊断脚本在worker完成后要求待命USV/UUV同刻新鲜而写总状态FAILED，故只能按各自证据报告“跨介质方法组件执行完成”，不能把原诊断或完整请求标PASS。r7–r11真实失败均保留。当前最大缺口：两区域三类平台同请求、UUV回区资格、复查实跑和正式10秒预算；中文实时显示已有无返回联合诊断，不是最终请求验收。

> **2026-09-23 阶段记录（被上条推进覆盖）**：`experiments/20260923-aav-water-method/result.json`当时只有同一qn全状态的AIR转场74.55秒、垂直入水/短水下观测/出水54.91秒、AIR回部署区72.36秒三段只读FEASIBLE；此后已接入候选并取得r12实际五步证据，不能继续将本条“未接入Action”当当前状态。UUV返航仍失败，10秒预算仍UNKNOWN。

> **2026-09-23 最新返回合同**：用户进一步确认REMUS可在观测后通过式重进回收区，但必须接续原PVS验证的安全尾段、末端合法后才算返回完成并释放；不研发驻停控制。`pvs_node.py`现在记录实际“离区→观测→重进”，`executors.py`完整计划按产品产生后重进与全尾段安全检查；未重进Action锁定，合法重进且尾段终态在区外可成功。当前港口原路线在东栈桥/pier越限，开放水域原路线在观测后180秒也不重进小回收区，因此UUV回收仍未通过。USV西侧支援→原部署区真实Action已SUCCEEDED，终点距中心0.24761m。任务/计划/PVS相关30项检查、原水下协作回归均通过；详见WORKLOG顶部。不要按下面旧阶段的“REMUS须末态在回收球”继续实现。

> **2026-09-23 水下返航继续点**：先读WORKLOG顶部。±π航向薄接入修正有Fossen源码依据和15项相关检查，原水下协作实跑仍PASS；但原REMUS回(-5,8,-2)路线依旧碰东栈桥/西侧pier。`pvs_node.py`现对最后航点恰为本机已声明回收中心的Action，在预检和完成时核对回收半径；未到则拒绝或失败锁定，普通通过式观测保留。无障碍反例证明原路径越过终点平面时可能距中心42.55m；不能把PVS `terminal_verified`单独当返回成功。证据`experiments/20260923-pvs-return-contract`、`20260923-marine-return-qualification`和`20260923-pvs-heading-water-regression`。下一步找真正可达的海洋返回方法，不能降净距或放大回收区凑成功。

> **2026-09-23 返回与安全继续点**：用户已确认部署区返回，`five_scene_harbor.yaml`现有每成员显式回收中心/半径。`experiments/20260923-air-return-action-r1`单AIR两段实际Action均成功、返航终点距AAV1中心0.08383m<0.5m；这不证明UUV/USV返回。`experiments/20260923-joint-safety-r9/safety-audit.json`在同请求无返回诊断的831个对齐样本0缺失、最小五平台代理净距0.91359m、静态场景审计PASS；r7/r8证据缺失失败保留。下一步查海洋两平台回部署区的原生可达路线，再把返回接到同一完整请求；10秒生产预算和缺测复查仍未完成。

> **2026-09-23 可视化继续点**：`experiments/20260923-joint-live-r6`用同一个诊断请求实时打开RViz和中文任务面板，执行期面板可见AIR已完成、水下/USV正在执行、母船收件及占用；最终三Action成功、两产品收齐、锁空。已保存真窗口规划期RViz与执行期面板截图，但未保存执行期RViz或全程bag，安全全段不可据截图推断。当前窗口随实验结束关闭；不宣称一直后台运行。仍缺10秒生产预算、返回、复查及正式入口。

> **2026-09-23 最新继续点：同请求真实并行诊断首次通过**。直接读`experiments/20260923-joint-physical-r5/{joint-result.json,metrics.json,nominal-plan.json}`和WORKLOG顶部。现有runner在60秒诊断预算、暂不要求缺失回收区的返回时，真实执行AAV1 AIR、UUV水下、USV支援三项Action，两个几何代理产品实际到母船，三Result成功、两业务任务接收、资源全部释放。r4的Result先于水下产品导致旧worker误锁，已在现有Result提交前有界等收件并通过r5；旧失败保留。**尚未完成**原10秒生产规划、直接联合请求入口、回收/返回、收到缺测后复查、完整实时窗口和全程独立安全审计；别把`PASS_JOINT_NO_RETURN_DIAGNOSTIC`当最终验收。

> **2026-09-23 最新继续点**：完整请求10秒生产预算仍未形成可派发计划，首个AIR候选原生查询约6.2秒，后续UUV/USV候选在预算内变UNKNOWN。按文献会合原则试验的UUV先出发/USV第14秒接续在PVS+有限交付组件名义合格，60秒无返回全区域诊断得到工期80.58模型秒计划；但`pvs_node.goal()`不允许USV执行前一Goal时预接纳下一Goal，runner也不能承诺该未来片段，故临时生产错峰改动已撤回。先别把该诊断计划派发或画成完整实跑；下一步继续原10秒性能、真实未来片段接纳、声明回收区及同请求复查。证据与失败口径见WORKLOG顶部和`docs/reviews/source-and-assumption-audit-20260923.md`。

> **2026-09-23 当前水下入口**：固定0/30/40秒等待已撤销；同源PVS路线＋步首有限传输推导并核验的26.5秒等待在`experiments/20260923-water-derived-wait-r1`实际通过。别从下方旧交接把0/30/40误认为当前生产方法。全区域同请求、合格返回区、一次复查与完整可视化仍是目标，未因该单阶段通过而完成。

> **2026-09-23 预算证据**：全区域首次名义方案约38.212秒，10秒生产规划仍返回UNKNOWN/预算耗尽。drone_0、drone_2的110秒原qn idle计算各约26秒，虽位置未动，但控制器/执行器状态不可用初始trim替代。不能为了显示任务完成而删除待命成员安全、放大生产预算或假设它们永久悬停；若引入有条件的位置占用资格，须证明适用初态/期限及未来不再指派。最新日志见WORKLOG顶部。

> **2026-09-23 可视化继续点**：AIR＋USV组件已有同运行RViz/中文任务权威面板正例，`experiments/20260923-air-support-live-r1` 保存了运行状态、Action/收件结果和窗口图。`draw_water_cooperation`现从PlanItem识别AIR、水下或联合活动；`scene_publisher`不再把任何请求的静态图例都称作水下任务。未接同一全区域请求/复查/返回，故不可称最终可视化。待命AAV100秒原qn只读延续位置未漂移但内部状态变化，不能用初态trim代替后续完整状态凑10秒预算；证据`experiments/20260923-qn-idle-horizon`。

> **2026-09-23 依据审计继续点（阶段记录）**：先读 `docs/reviews/source-and-assumption-audit-20260923.md`。删除了无业务依据的默认回起点/以入口容差代替回收区域，场景或调用必须给出回收位置及半径；该时点联合请求缺这一输入，随后已按用户确认的部署区政策补入港口场景。AIR服务时间已由请求驱动，固定多一秒USV支援和重复AIR尾段Action已撤。旧水下0/30/40秒等待为实验候选，不是文献规律。全AIR/WATER请求在10秒预算内形成可行计划、同次复查与海洋平台返航仍未验收；见顶部最新结果。

> **2026-09-23 预算诊断**：组件 AIR＋USV 的实际 worker r4 已通过。全区域、五平台、不固定 AIR 成员的10秒规划仍预算耗尽，零 Goal；60秒隔离诊断在暂不要求原路返回时找出 AAV1 AIR＋USV RF、之后同 USV 与 UUV 水下协作的完整名义可行方案，工期107.44模型秒，但 `search_complete=False`。因此全任务不是“没有联合方法”，而是原预算下缺可提交结果，同时 UUV 原路返回单独不合格。不能将诊断预算改为生产默认或拿诊断计划派发；下一步从长时待命完整状态计算和返回终态解决。

> **2026-09-23 全区域继续点**：新 AIR＋USV 同计划组件已有 r4 原生 Action/母船接收/资源释放正例。对不固定 AIR 成员的完整 AIR＋WATER 两区域请求，保持五平台名义安全与共享通信，在仅为隔离原因而撤销返回要求的诊断条件下，原10秒规划仍没有完整可提交方案，返回 `PlanningBudgetExceeded`，未派发；原返回要求另被 REMUS 东栈桥实际几何阻断。下一步是减少完整方法查询及待命状态预测的重复时间并保持原硬约束，而不是延长生产预算或以组件结果代替全请求。见 WORKLOG 顶部。

> **2026-09-23 最新继续点**：不要再把 AIR 的产品/USV 接线列为“尚无组件实跑”。在限定 AIR 区域与指定AAV的受控实例中，原候选搜索给出 AIR＋USV 两活动完整名义可行方案；原 runner 先确认 USV 原生动作开始，再派 AIR，r4 两个 Action、母船收件及资源释放通过。10秒共享预算内找到可行方案但 `search_complete=False`，不能宣称有限候选最优。r3 的旧 PlanItem 完成检查错误已修，原失败保留。下一步是全区域请求的真正方法比较与同次执行、规定返回及收到缺测报告后的未承诺部分修复；水下原路返回东栈桥违规仍阻断原请求通过。工作日志及原始实跑目录见 WORKLOG 顶部。

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
