# FaSTrack 2017：规划模型、跟踪模型与误差界成立条件

## 文献记录

- 作者：Sylvia L. Herbert*, Mo Chen*, SooJean Han, Somil Bansal, Jaime F. Fisac, Claire J. Tomlin（* 同等贡献）
- 年份：2017
- 发表：IEEE Conference on Decision and Control (CDC) 2017, DOI 10.1109/CDC.2017.8263867；arXiv:1703.07373（v2, 2021）
- 链接：https://arxiv.org/abs/1703.07373
- 本地 PDF：`papers/safety/fastrack-2017.pdf`（2,363,148 B，sha256 `d070eb7…460de5`）
- 阅读范围：全文（pdftotext 正文）

## 该文献的保证与成立假设

FaSTrack 的保证是**相对规划模型的跟踪误差界**，不是“任务完成/避障安全”的现成定理：

- 跟踪系统动力学 `f` 一致连续、有界、对状态 Lipschitz；控制集 `Us` 与扰动集 `D` 紧致；因此 ODE 解唯一（§III）。
- 规划模型不含扰动输入；规划状态与跟踪状态的对应关系 `r = s - Qp` 必须已知、可定义（§III-B, §V-A）。
- 离线解 pursuit–evasion 博弈并用 HJ reachability 得到值函数 `V∞(r)`；要求值函数收敛。取最小不变水平集 `B = {r : V∞(r) ≤ V}`，其投影 `Bp(s)` 作为跟踪误差界（§V）。
- 在线保证成立需同时满足（§V-C 条件与 §VI）：
  1. 初始相对状态在所选不变水平集内；
  2. 跟踪系统在边界附近使用安全控制器（`∇V∞` 查表），远离边界可用任意性能控制器；
  3. 规划模型的行为在最坏情况下仍不逃出该水平集（即使规划器“对抗性”移动）。
- 规划器把感知障碍按 `Bp(0)` 膨胀后再规划（`Oaug = Osense + Bp(0)`），所以安全来自“障碍膨胀 + 跟踪误差界”，不是来自规划器本身。
- 论文声明：采样数据控制等实际问题可由 [33]–[35] 处理，不是该文重点；测量噪声/状态估计误差不在保证范围内。
- 实验（RRT + 四旋翼）用作演示；数值不是对本项目 qn 闭环的保证。

## 该文献改变当前哪个接口定义？

1. **`safety/validity verdicts`**：FaSTrack 的安全性等价于“离线算出的误差界 + 安全控制器 + 障碍膨胀”三者同时存在。本项目的 qn 是 RBF/PD + 6DOF 闭环，既没有 `V∞` 查表，也没有按界膨胀的障碍；因此 `safety_outcome` 不能因为“成员到达槽位/没有碰撞采样点”就写成 PASS 并声称有误差保证——在缺少离线误差界的当前版本中，最多只能给出离散采样的观察结果（`NOT_VERIFIED` 用于无界可证的情形）。
2. **`PositionCommand` / `used_reference`**：`used_reference` 是规划模型的期望状态，不是被控对象的真实状态；两者之差正是 FaSTrack 要约束的量。接口上必须持续发布/统计 `||p_i - p_i_used_ref||`（plan.md 已有 trajectory tracking error 指标），并且完成阈值 `epsilon_p` 不能同时充当“跟踪误差已证明有界”的声明。
3. **完成判据**：若未来要把 `epsilon_p` 与安全边距建立关系，需要先有一个（哪怕是很保守的）跟踪误差上界；在得到之前，`epsilon_p` 只是工程到位阈值，不是 FaSTrack 意义上的误差界。

## 该文献要求增加哪个反例测试？

**测试：`test_bounded_tracking_claim_requires_computed_error_bound`**（建议放 `integration/qn_aav_simulator/tests/test_formation_action_server.py` 或 `test_time_alignment.py` 同级的闭环测试）

- **setup**：在 qn 后端注入一个持续扰动或把执行器权限调低到控制器无法在最后 0.5 m 内完全消除偏差；规划一条 A→B 的 `FormationAction`，成员最终位置可以进入 `epsilon_p` 但 `||p_i - p_i_used_ref||` 在驻留窗口内持续大于一个给定的界 `B_test`；实验不预置任何 HJ 误差界或安全控制器。
- **stimulus**：执行 Goal → 驻留 `hold_duration` → 运行 `verify_formation_experiment`。
- **expected observation**：`task_outcome` 可以按到位窗口判定，但**不得**据此把 `safety_outcome` 置为 PASS 或声称“跟踪误差有界”；`safety_outcome` 必须为 `NOT_VERIFIED`，日志中必须同时给出 `trajectory_tracking_error_m`（最大值与分位数）而不是只给最终 slot error；若实验报告声称 FaSTrack 式保证，则 `experiment_validity` 必须为 `INVALID`/`INCOMPLETE`。
- **为什么是反例**：只要代码存在“最终位置达标 ⇒ 跟踪有界/安全”的捷径，这个测试就会失败；它把 FaSTrack 的前提（离线误差界 + 安全控制器）显式变成测试条件。

## 我们实际复用什么

- 规划参考与真实状态分离的思想：`used_reference_pose/twist` 只表示采用过的规划参考，Odometry 是真实状态，二者不能混为安全证据。
- “障碍膨胀/完成阈值需要计入跟踪误差余量”的思想，可用于后续把 `epsilon_p` 与实测跟踪误差联系起来（本版只记录，不宣称界）。
- 误差界是离线预计算、在线查表的架构，可作为后续“规划模型-跟踪模型误差界”研究模板；当前不引入 HJ 计算。
- 安全控制器只在接近边界时接管、其余时间用性能控制器的混合策略，可作为 qn 控制扩展的对照设计。
- 明确不适用：本项目的 qn RBF/PD 闭环、六自由度数值积分和 AIR 编队目标不满足该文误差界的构造前提，不能直接引用其保证。
