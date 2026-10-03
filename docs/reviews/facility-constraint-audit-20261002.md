# B/C 主链约束审查

本次审查依据用户冻结的 B/C 任务定义、现有控制与资源合同、实际失败记录。论文支持运动和协调方法，不为本工程的数值容差提供直接证明。B/C 尚未完整运行通过。

| 检查项 | 判断与处理 | 所在位置 |
|---|---|---|
| AIR 速度必须大于所有 WATER 速度 | 配置偏好，不是任务可行性条件。删除硬拒绝；当前默认仍保持 AIR 更快。 | `inspection_work.py:inspection_motion_profile` |
| 单份 AIR/WATER 作业检查所有平台的参考速度 | 跨域/跨未选成员耦合没有必要。定义校验自己的nominal包络；联合候选按实际成员速度筛选，端点校验已选速度。合法成员仍可执行。 | `inspection_work.py:validate_work`、原联合候选/端点 |
| 请求层数必须≤8、所有工作偏距必须≥0.95 m | 没有通用几何依据。请求端仅检查正整数层数与有限正偏距；实际轮廓有效偏距仍按既有导航包络计算。UI 当前的层数控件范围属于显示配置。 | `monitoring_request.py:facility_joint_mission_mappings` |
| 设施 AAV 必须先降至0.8 m再进入 native ENTER | 原 native 接受合法停稳 AIR 初态，未要求0.8 m。使用已有2.2 m共同转场层交接，删除这段重复空中下降。 | `executors.py:build_inspection_executor_plan`；`platform_action.py:_entry_reason` |
| 下降时水平误差0.1 m、速度0.03 m/s不满足就重新飞至转场层 | 额外门槛比原到位容差严格，且下降本身必然有垂直速度。改为按原入口容差接纳一次下降阶段，持续执行同一目标。 | `formation_action_server.py:_mapping_step` |
| 实际/参考差超过0.30 m就不提交任何后续局部目标 | 不能纠正正在执行的旧参考，可能把瞬时误差变成长期卡住。删除该局部调度否决及无消费者的Action成员/launch参数；独立原Swarm净距reserve仍有用途，轨迹校核、参考采用证据、实际净距监视继续生效。 | 同上及原launch |
| 残余补扫要求在0.1 m内停稳 | 补扫前后已按工作容差外扩，另加0.1 m没有必要。已恢复该作业原位置容差；原连续合格区间和终端停稳条件不变。 | `inspection_work.py:residual_target` |
| 轨迹/实际安全与工作有效偏距 | 保留。障碍实际净距0.2 m、成员净距0.5 m是既有工程合同；膨胀和偏距是相应规划裕量，不宣称论文常数或连续安全证明。 | 原场景、Swarm补丁、`effective_contour_offset` |
| 转换前姿态、航向、停稳 | 保留当前声明方法。yaw0依据原 qn 部分浸没力矩计算；0.1/0.05/0.03是工程容差，未证明为必要充分的最小界。实际转换曾失稳，不能仅为运行通过而删除。 | `transition_alignment_ready`；原 native入口/转换 |
| 真实区间并集、样本不跳补、匹配 GoalID/作业版本、实际收件 | 保留。这些决定报告对应哪份任务，防止重复计量或旧结果释放新任务；不是搜索额外运动方案的门槛。 | 原进度、Action、runner/收件通道 |
| 求解10秒与执行有限观察预算 | 两者是计算/运行策略，不是物理可行性证明。初次取得完整可派发方案即可返回；执行预算按作业量/速度及已有裕量计算，不逐候选预演。 | 原联合Plan、工作预算与端点 |
| USV查询耗时后观测过期 | r48证实同步查询阻塞扫描回调，自身制造过期；不是有效输入被外部中断。原PVS连续接收观测，有限运动查询读取实测地图快照；保留实际期限和full-state采用条件，不延长期限掩盖阻塞。 | `pvs_node.py:_survey_cloud/_mapping_target` |
| 各USV候选重复相同制动前缀、较长零推进优先 | 同次源/地图/dt相同，重复积分浪费有限预算；较长零推进先于可行短推进也没有任务进展依据。复用已校核privatefuture前缀，优先检查正推进候选，保留每个分支的原生制动/可行性检查。 | 原PVS局部query/候选循环 |
| 沿线作业每次部分合格后必须回整条线起点 | r51实际leg82已有首尾合格区间，只缺中间却反复整段重进；这个前置步骤没有必要。按实际union缺段做有padding的局部补扫，接近不计量，已有合格区间保留，完整作业与原质量谓词保持。 | `inspection_work.py:sample_progress`及原执行端 |
| 水平到位就冻结水下全部三轴参考 | r52证明UUV需要下降到下一层，而实际/参考Z一直-1.2m。水平对齐只控制SURGE；原3D容差才判断作业目标到位，Z继续交给原qn竖向控制。这修正维度错误，没有新增深度门槛。 | `platform_action.py:_mapping_reference` |
| 巡检已有有限wall看门狗，又按名义工期强制model截止 | r52最后样本合格且仍在推进，却在新Goal下发0.107s后被估计工期截断。仅删除重复model硬门，保留原固定wall观察上限与所有实际完成/安全/资源条件；名义时间继续估计，不当作精确截止。 | 原AIR/WATER巡检预算分支 |
| 不计量的补扫接近阶段也必须面向设施 | r53导航点离实际航向74.07°、超出原前向半FoV60°，新参考反复未生成。接近先朝导航意图；原位置容差内转向结构，达到已有朝向要求再SWEEP。不新增角度门槛，不把接近算作业。 | 原AIR heading发布/残余状态 |
| 补扫中间转接必须≤0.03m/s | 删除额外中间停速门；原位置/合格计量仍有效，最终native终态与LOCAL驻留的0.03/4model秒保持未改。 | `inspection_work.py:residual_target` |
| WATER补扫接近须先原地满足切向朝向 | 前进式控制应继续驶入扫测段，实际朝向条件在合格区间采样中判断。删除普通APP转换的额外朝向前置门，AIR面向结构及WATER工作采样条件保持。 | 同上 |
| 未到终态就清除所有WATER保持意图 | 原tick会撤掉_mapping_reference刚请求的local_wait/column_hold zeroSURGE，使旧落后参考重新产生推力。保留仍有效的等待/同列意图，Z是否推进独立判断，不改物理状态。 | `platform_action.py:tick` |
| PVS局部查询固定1wall秒 | 实测完整原生检查及约0.17秒解释器启动可超过1秒，预算不足不是物理不可行。当前局部计算2wall秒，futureprefix同步；顶层Plan仍10秒，实际几何/full-state采用/最终保持不改。 | scene profile、原有限query |
| WATER终点容差再加进避障机体半径 | 到位容差没有被验证为跟踪上界。删除该混用：局部导航使用原机体半径0.25m＋声明净距0.2m，终点仍按原0.2m到位判定；几何意图按该包络重新产生有效偏距/新版本，旧计量不继承。实际净距规则不降低。 | 原WATER局部query、`effective_contour_offset` |
| 扫描先到就视为没有对应位姿 | scan与Odometry是独立ROS连接；在既有输入窗口内等待精确stamp匹配，仍不接纳错误位姿。读当前map后再取now，避免新数据被较早时间误判为future。 | 原AIR survey callback/局部调度 |
| A*只检查邻接中心，简化阶段再拒绝同一不安全边 | r57连续找到4点路径又拒绝edge1/4，无法提交新参考。搜索阶段使用相同ESDF/1cm边谓词，拒绝该边后搜索其他入边；节点只有通过安全边后才标记为发现。没有新增几何/时间限制，是使原搜索与原验收一致。 | 原`dyn_a_star.cpp`补丁 |
| 每次等待完整局部预算、只提供极短推进候选 | 最大计算预算不等于实际延迟。futureprefix使用最近完整query实测耗时及原余量，迟到仍拒绝；当前局部目标距离与名义速度仅定义候选窗口，完整native pulse/coast才决定可行性。原观察期限未加长。 | 原PVS query/采用分支 |
| 每个局部查询冷启动整套模块 | r58完整query1.96wall秒，原生计算仅约0.27秒。单caller复用现有trusted worker；每份fresh snapshot独立deadline，timeout结束进程并清空响应，不管理物理Goal/资源。 | 原bounded query/query_worker |
| 瞬时fleet状态缺口恰逢hold成功就结束验收 | r58物理hold与采用成功，最后信息gap0.058秒仍在原≤1秒恢复窗口。延后终态提交并取得fresh真实bracket；真实安全与期限保持，严格时间报告/旧Result不改。 | 原FormationAction终态提交 |
| 完整WATER工作后zeroSURGE，同时仍强制追最后顶点 | r59完整区间已获得，实际静止而离名义末点约0.21m。先真正刹停，再采用实际停止reference验证原4秒；连接runout从实际末端生成，未完成区间与实际pose不改。 | 原WATER执行/接续 |
| 球形机体被扩成方体，在体素角误判相交 | r59 UUV第三层证据表明球形包络有余量而L∞角包含当前位置。WATER/PVS改为同半径的Euclidean swept-sphere/voxel计算，UNKNOWN仍拒绝、实际净距不降；AIR保留与原Swarm相同的box inflation。 | 原LocalSurveyMap |
| 声明支援区有效，但USV还必须停到精确中心0.2m | 服务完成使用声明区域，原实际0.03/4model秒期间仍须持续在区内；漂出就恢复同一导航，区外不释放。ordinary movement/返航条件不改，没有新设容差。 | 原PVS任务服务Goal |

当前直接失败证据：`experiments/20261002-facility-session-live-r47/air-entry-vertical-failure.json`。实际包络在AIR下降时越过水面，native ENTER未派发。修正后只通过完整B/C及同会话动态执行检验结果，不能用旧部分运行拼接为通过。

r49的状态缺失另有底层问题：`dyn_a_star.cpp`路径简化在相邻段校核失败时可能重复同一输入索引，存在无界vector增长缺陷；其增长特征与约25GB RSS的OOM一致。未采集live native调用栈，具体OOM分配现场归因仍是推断。修改为严格推进输入路径索引，相邻段不安全返回本次局部失败；没有增加新的次数/几何限制，也未延长缺样期限掩盖被杀进程。实际源码与内核记录位于r49/native-*及native-oom-diagnosis.json。

r50两机已采用参考中心距离1.070167m，小于原native1.6m，且实际/source时间均新鲜。源码确认共用`callEmergencyStop()`只向本机发布，普通停止参考缺少peer广播；修正为同一PolyTraj同时进入原local和peer通道，去掉stationary外层重复广播。不另加高度层或加大净距替代引用接线修正。证据为r50/air-peer-clearance-failure.json与native-peer-broadcast-fix.json。

方法边界见 [B/C模板设计](../requirements/inspection-business-templates-20261001.md)。[Swarm官方源码](https://github.com/ZJU-FAST-Lab/Swarm-Formation)用于原轨迹与避碰框架；[D-ITAGS官方论文页](https://star-lab.cc.gatech.edu/papers/neville-ditags/)用于执行反馈下的局部调整思路。两者均不直接提供上述工程门槛数值。
