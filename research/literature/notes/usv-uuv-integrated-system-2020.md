# 集成式 USV–UUV 平台 2020：第三种水面—水下耦合形态

## 文献记录

- 作者：Study on Control System of Integrated Unmanned Surface Vehicle and Underwater Vehicle（Sensors 20(9):2633）
- 年份：2020
- 发表：Sensors（MDPI，开放获取），DOI 10.3390/s20092633
- 链接：https://www.mdpi.com/1424-8220/20/9/2633
- 本地 PDF：`papers/maritime/usv-uuv-integrated-system-2020.pdf`（6,161,839 B，sha256 `f56e6f30074d9fe5…`）
- 阅读范围：全文（pdftotext 正文，摘要与系统设计部分）

## 1. 它解决什么问题

- Problem：已有 USV/AUV 协同系统难以**实时**获取水下数据并长时间作业。
- Inputs：GPS（装在 USV 上）、USBL、AHRS 的相对位置/艏向测量。
- Outputs：USV 航点跟踪 + UUV 相对艏向/距离控制，完成一体化平台的现场试验。
- Assumptions：USV 与 UUV 之间**有物理缆绳**（winch 收放）。

## 2. 它的任务、资源、通信和运动模型

- Task：水面航点跟踪 + 水下相对位姿保持（不是"各自去一个点"）。
- Agent / capability：双体船型 USV 作为**供电与通信枢纽**；鱼雷型 UUV 负责水下载荷数据处理与回传。
- Resource：**缆绳**提供连续供电与大带宽数据通道——这是与声学链路完全不同的资源模型。
- Communication：**有线**（缆绳）传输数据，GPS 在 USV 上，UUV 靠 USBL 得到相对位置。
- Motion：USV 航点跟踪；UUV 相对艏向与距离控制（跟随 USV）。
- Execution feedback：传感器实测（USBL/AHRS 性能验证）。

## 3. 对我们哪个接口或约束有影响

1. **USV–UUV 耦合不止一种**：至今我们看到三种——有线（本文，连续供电+大带宽）、
   声学跟踪（Xiroi II，间歇低带宽、靠保持邻近改善链路）、支援轨迹（JOE 2022，约束对方状态估计误差）。
   我们任务模型里**一种都没有**，只有"到位 + 驻留"。若将来要表达 UUV 任务，
   必须先声明采用哪一种耦合及其资源假设。
2. **UUV 的定位依赖水面平台**：本文的 UUV 位置来自 USV 上的 GPS + USBL 相对测量。
   这支持我们"AUV/UUV 无执行端点时不产生实际完成事件"的规则，也说明
   `TravelTimeProvider` 的欧氏距离模型对 UUV 完全不适用。
3. **"数据交付"的形态由耦合方式决定**：有线可以实时大带宽，声学只能间歇低带宽。
   我们计划里的"零延迟理想交付"必须标注为**假设**，并说明它对应哪一种耦合。

## 4. 它要求我们增加什么实验或反例

- **反例：`test_uuv_task_requires_declared_coupling_mode`**：任何水下任务必须在配置里声明耦合方式
  （有线 / 声学 / 支援）及其带宽与时延假设；缺失时拒绝加载，而不是默认零延迟。
- **反例：`test_relative_positioning_is_not_gps`**：水下平台不得使用水面/空中的绝对定位假设；
  若任务时间由欧氏距离/标称速度推出，必须标记为不适用于 UUV。
