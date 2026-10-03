# Castelar Wembers et al. (2024)：风机巡检运动模板依据

核查日期：2026-10-01。仅核查主源、下载开放资料和提炼控制模板；未修改或运行项目代码。

## 书目与取得状态

- 作者：Carlos Castelar Wembers、Jasper Pflughaupt、Ludmila Moshagen、Michael Kurenkov、Tim Lewejohann、Georg Schildbach。第一作者姓应保留为 **Castelar Wembers**。
- 题名：*LiDAR-based automated UAV inspection of wind turbine rotor blades*。
- *Journal of Field Robotics*, **41**(4), **1116–1132**, **2024**；DOI：**10.1002/rob.22309**。题名、作者、年卷页与所给DOI一致，无需纠正。
- [Wiley出版者正文](https://onlinelibrary.wiley.com/doi/10.1002/rob.22309)：页面标记Open Access，首发2024-03-13。
- [University of Lübeck机构记录](https://research.uni-luebeck.de/en/publications/lidar-based-automated-uav-inspection-of-wind-turbine-rotor-blades/)：核对作者/年卷页，期刊出版2024-06；授权标记CC BY-NC-ND。机构记录中另有“Article 4”字段，本记录采用出版社标准卷期页格式，不将其误当唯一文章编号。
- 正常请求[出版者PDF入口](https://onlinelibrary.wiley.com/doi/pdf/10.1002/rob.22309)和[pdfdirect入口](https://onlinelibrary.wiley.com/doi/pdfdirect/10.1002/rob.22309)均返回HTTP403，响应类型为HTML。ePDF的网页读取也失败；机构条目仅链接DOI，未提供另一个作者稿PDF。
- **取得状态：出版社开放HTML正文已读；PDF未取得。** 未绕过付费或访问控制，未把HTML、网页打印件或其他论文写为本论文PDF。预定文件`research/literature/papers/task_motion/castelar-wembers-wind-inspection-jfr2024.pdf`本轮未创建。
- 下列定位使用正文节号、段落和图号；没有本地PDF，因此**未核验印刷页位置**。

公开原始资料保存补记（2026-10-01 06:23:54 UTC前完成）：机构页普通HTTPS下载成功（HTTP200，`text/html;charset=UTF-8`，55728字节），原始响应未改写保存为[机构原始HTML](../sources/inspection-templates-20261001/castelar-wembers-wind-inspection-jfr2024-luebeck.html)。来源URL、时间、SHA-256及此前Wiley403状态见[获取记录](../sources/inspection-templates-20261001/castelar-wembers-wind-inspection-jfr2024-source.json)。机构HTML含元数据、摘要及引用，**不是论文完整正文或PDF**；Wiley正文的web工具读取结果没有冒充原始HTML保存。本次补记未重复PDF请求。

## 正文依据（归纳，非长段引用）

| 主题 | 正文定位与事实 |
|---|---|
| 静止叶片 | §2.1第1段：Mercedes停机位置，一片叶片竖直向上；检查后重定位转子，再检查下一片。此设定来自该热成像方法。 |
| 路径分阶段 | §3.1开头及图4：预检、起飞接近、定向定位、检查、返回降落。 |
| 偏置距离 | §2.3.4的PI距离/朝向控制段、§4.2步骤(iii)：结构距离7–12m；图10说明给10m示例。 |
| 表面与朝向 | §3.1.3末段、§3.1.4第1段、§3.2.3表面角度小节及图7：分别约束距离和相对表面角度；目标包括两面与前/后缘。§2.2.3第1段相机云台补偿机体俯仰/滚转。 |
| 最终检查运动 | §4.2步骤(i)–(x)、图10：环绕定向、沿缘上升；细尖端改为每4m一圈；沿面下降、换面上升、换缘下降。 |
| 观测限制 | §4.2开头3段：固定LiDAR受姿态影响，机舱方案在实地频繁中断；叶尖附近10–15m的范围内，细叶片难以持续检测。 |
| 识别scope | §1.2贡献节末尾明确把热成像损伤检测方法排除在论文范围。 |
| 实证边界 | §1.2贡献节的验证场景说明：检查在陆上风机验证，海上验证起降阶段。§4.3末两段：海上自主标记降落频繁中断，最终人工着陆。 |

## 对B控制模板的建议（本项目解释）

1. 将停机/叶片身份/当前姿态写入任务前置状态。AAV只执行当前已确认静止对象；转子转动或重定位是外部设施事件，应先退出作业包络并等待新状态，再重算目标相对路径。
2. 按“接近—定位—指定表面/边缘跟随—换面—退出—结果交付”表达阶段，逐段记录进入条件与实际终态。这里的交付阶段是本项目要求，不能归为该论文已实现的母船通信闭环。
3. 路径应由目标几何及表面法向生成，可声明`p_ref = p_surface + d*n`；`d`是作业偏置，与机体半径、实体净距、定位误差余量分别记录。7–12m属于原设备和原热成像方法，不能直接作为缩比场景净距阈值。
4. 将机体航向、载荷视轴、目标法向分开。原文云台事实不能证明本项目机体朝向满足载荷视野；若仅检查位姿/LOS/朝向代理，应明确这不是有效热图、实际缺陷检测或完整表面重建。
5. 尖端低可观测区、遮挡或状态失效须能暂停、退出或换候选阶段；不把环绕视为一定解决所有缺测。具体姿态来源可用本项目声明对象状态，不据此增加SVD/SLAM研发范围。

## 不能直接继承

本文并未验证本项目三AAV/USV/UUV协作、跨介质、任意风机/平台避障、完整海上自主着舰或业务识别精度。其M300设备、PI/Kalman实现、距离和运动节拍均不能替代原Swarm/qn控制器及实际安全验收。本文只能支持业务对象相对运动及阶段划分的设计动机；B模板当前仍需明确标为未实施、未实跑。
