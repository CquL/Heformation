# 三类平台协同实施记录：时间、活动与原生执行边界

用户于2026-09-20明确要求实施《Heformation 三类平台协同任务实施计划》。本次从main@80a0908开始修改；以下为工作树阶段结果，**完整协同任务尚未完成**。没有引入VRX、控制算法或新的协调节点。

## 已实施与证据

| 改动 | 实际证据 | 结论边界 |
|---|---|---|
| deadline=None贯通请求、任务、调度和结果 | no-deadline-r1五段Action SUCCEEDED，观测/接收各1.0，deadline_met和deadline_lateness为null | 仍是原三机几何代理和零延迟交付 |
| qn/PVS固定步的绝对单调节拍 | pacing-r1港口7项动作成功；独立全段audit PASS，模型/ROS最大偏差0.025656964s，跨平台0.025566624s | 保留0.05s原门槛；无GUI运行，不代表任意负载均可靠 |
| 原生片段预装载与幂等启动 | prepared-r1三方PREPARED后2秒未开始，错误ID/代次拒绝，重复启动幂等，三方原生Result成功 | 探针直接协调；正式runner的多方承诺/断联释放尚未接通 |
| 同业务多活动、实际依赖、合法等待表示 | 同业务水下作业与USV支援区间可重叠；共享成员重占及联合循环拒绝 | 查询证明的等待可以表示；等待派发仍拒绝，未静默省略 |
| execution_steps单一来源与复合worker | composite-r2经原生预测的两段REMUS运动分别收到Result，父活动最后释放；失败阻断逻辑检查通过 | 当前复合链按运动资格处理，不据此产生观测或交付 |
| 日志/显示与执行锁分离 | 写盘、ROS参数发布位于执行锁外；受控阻塞发布时执行锁仍可取得 | 写入仍同步且串行，不声称任意日志负载均无延迟 |
| 全信道步首因果 | FiniteDelivery.advance_all以同一步首持有量更新；两种调用顺序均禁止同一步瞬时多跳 | 尚未与业务产品和母船接收事件在线贯通 |
| 原七机兼容 | seven-r1代表运行，Action成功，verification PASS，failure_count=0 | 不替代所有七机场景，M2历史失败仍保留 |

所有证据均在`experiments/20260920-cooperation-implementation/`。原始bag和运行日志留本地，不因体积将其伪装成已提交文件。模块检查在复合改动时325项通过；最后日志锁/事件显示及修复元信息改动后42项相关检查通过。检查数量仅辅助确认回归，不能替代上表实跑。

节拍修正依据是当前镜像Noetic `rospy.Rate.sleep()`在慢循环超过两个周期后重设下一时刻，而旧循环每次仅积分一次固定dt。新循环每一步仍真实积分，模型时间只增加完成的dt；不更改消息时间戳、不清零累计时间、不放宽时间/安全阈值。当前不支持外部暂停型`/use_sim_time`，运行入口明确拒绝该配置。

## 代码落点及接口

- `integration/mrta_python/models.py`、`executors.py`、`repair.py`：可选期限、活动身份/先后关系、单一步骤来源及完成传播。未经证明的预测工期不用于部分搜索剪枝。
- `integration/qn_aav_simulator/scripts/formation_mission_runner.py`：同一worker连续派发steps，逐步核对Result、故障阻断、父成员占用、显示快照在锁外写入。
- `integration/qn_aav_simulator/src/qn_aav_simulator/platform_action.py`与`scripts/pvs_node.py`：现有端点的prepare_only/StartPreparedAction。接纳与启动分开；GoalID与reference_generation核对，不重置模型状态。
- `integration/qn_aav_simulator/src/qn_aav_simulator/observation_coverage.py`：有限交付同一步首状态更新。
- `PlatformTask.action`增加默认false的prepare_only，新增`StartPreparedAction.srv`，Formation.action未改变。**所有PlatformTask客户端/服务端必须重新生成同版本消息**。

构建镜像：`docker build -f docker/Dockerfile.qn -t swarm-formation-qn:cooperation .`。原RViz启动命令已更新到该镜像；显示仍是固定资格实验，不能宣传为新协同任务。

## 保留的失败与尚未完成项

首次期限测试错误地向旧七机planner传入一个成员，1失败/107通过；修正测试输入为七机，未改旧planner约束。composite-r1在运动派发前因探针传入空权重表失败，修正探针输入后composite-r2成功；没有修改覆盖函数的非空约束。过去harbor-r2时间失败及M2安全失败均保留。

仍需按用户方案继续完成：

1. 有限观测候选参与联合覆盖/方法选择，AAV完整AIR/转换/WATER/返回的同源只读查询，具有明确入口资格的模式候选。
2. 联合候选实际生成多平台活动，包含有界等待、会合、尾段和全平台运动冲突；正式runner协调预装载/启动，不能仅靠本次探针。
3. 本地产生业务产品，经有限容量边界传输；母船仅消费收到的事件；水下通知不可达时仍执行预承诺会合；至多一轮复查及规定返回。
4. 独立/编队完整成本比较与独立能力验收；具体计划版本确认、中文实时联合任务展示。
5. 带显示负载的完整请求与失败实跑，统一评价条件的方法对照，以及明确假设下的条件性论证。

这些未完成项不能用既有原生动作或模块检查抵扣。直接依据及限制仍以`docs/requirements/cooperation-source-design-20260920.md`为准；本轮未扩展文献队列，也未宣称重新精读全部论文。
