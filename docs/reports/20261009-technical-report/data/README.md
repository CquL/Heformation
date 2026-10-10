# 图表实验数据

本目录是三份既有完整运行记录的小型派生副本，不含原始bag，不代表新仿真。A、B、C各一次完整运行；各次独立，不是同一世界的业务切换，也不是重复试验统计。

| 业务 | 原记录（仓库根目录下） | 请求 |
|---|---|---|
| A | `experiments/20261005-abc-16x-videos/A` | `circle-2.00-4.00-r2.00-mapping` |
| B | `experiments/20261004-server715-b-complete-r75` | `b-09b8504be510` |
| C | `experiments/20261004-server715-c-complete-r78` | `c-45b0c96d0f75` |

## 数据文件与口径

| 文件 | 内容／报告位置 |
|---|---|
| `mission_summary.csv` | 首份完整Plan墙时、实收数、核实成功步骤、实际收件和返回事件、终态资源锁；表5、6、8 |
| `goal_events.csv` | 61个匹配成功Goal的物理成员、活动、端点、派发及终态核实时间；图4 |
| `report_events.csv` | 40份当前请求产品的产生和母船匹配实收事件，附作业版本和GoalID；图4 |
| `facility_work.csv` | B/C七项所选作业的成员、版本、要求段数、合格区间数、合格长度与驻留；表7 |
| `b_support_revisions.csv` | B同请求的服务0／1在Plan历史中的原安排、实际完成及更新后继；图5 |
| `work_geometry.json` | B/C已接纳的作业定义，包含轮廓、沿线、局部作业及要求；图2 |
| `provenance.json` | 原数据和实际场景图的路径、SHA256及提取／作图口径 |

### 时间

- `planning_wall_s`、`planning_budget_wall_s`使用墙上秒；其余事件使用仿真秒。
- 每次运行时间原点取匹配活动的`executions.result_received_at - plan.items.actual_finish`的中位数，全部匹配项的差异同时核对。时间原点保存在`model_epoch_ros_s`。
- `dispatch_s`取ROS GoalID末尾编码时间，约1ms分辨率，是派发而非机体开始运动。`verified_terminal_s`取成功且核实的`step_results.result_received_at`。
- `last_verified_terminal_s`是核实成功步骤接收时间的最大值；Plan的活动汇总完成可能晚一轮，两者不混用。
- `authorized_return_to_last_terminal_s`是最后核实终态减人工返回授权，包含集结、运动和核实等待，不是纯航行时间。
- `task_event_wait_s`是控制报告产生到软件匹配实收的等待。原运行使用任务级接触门控及ROS收件，B/C报告`required_bytes=0`，不能当作声学／射频实测时延、吞吐量或传输成功率实验。

### 作业量与安全

- `qualified_movement_length_m`取控制报告`measured_length_m`，是合格作业区间长度，不是全程里程；`qualified_local_hold_s`取报告连续合格驻留累计。
- `required_segments`包括CONTOUR／LINE的非零空间短段及LOCAL项；管路6项为3段沿线与3处LOCAL，驻留合计12s。`qualified_segment_keys`是已报告的区间索引数，须结合报告完成比例理解，不单凭索引数量宣布完成。
- B/C净距取原独立采样审计，C使用完整历史修正的`c-full-audit-summary.json`，不覆盖此前失败审计。净距是声明碰撞代理量，不是连续实机安全保证。
- A没有本次同口径完整独立审计，安全字段留空，不能补零或填入在线最小值冒充对比。
- 单次记录不产生均值、方差或误差条；不同任务规模和参与成员不同，不用三组时长宣称性能优势。

## 复绘

在仓库根目录、可使用Matplotlib及中文字体的Python环境中运行：

```bash
python docs/reports/20261009-technical-report/make_report_figures.py
```

默认只读本目录副本并复绘图2、4、5，不要求原始实验目录存在。图1为独立FigGenie矢量架构图，见figures/architecture/。需要本地重新提取时：

```bash
python docs/reports/20261009-technical-report/make_report_figures.py --refresh-data
```

本次使用Python3.12、Matplotlib3.11.2及Noto Sans CJK SC字体。脚本只处理报告数据，不启动ROS、Docker或物理仿真，不修改原实验。图3直接使用仓库已有`docs/images/three-business-scene-overview.png`，未重绘或增强原画面。
