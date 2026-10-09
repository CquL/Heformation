# 海上通信导航支援轨迹 RA-L 2019

论文：Sampling-Based Path Planning for Cooperative Autonomous Maritime Vehicles to Reduce Uncertainty in Range-Only Localization，DOI `10.1109/LRA.2019.2926947`，8页。已核查摘要、第III节及算法1—3。

水面艇根据水下成员已知位置、速度、意图和定位协方差，采样可达发送位置及TDMA时刻；优先扩展较好候选，剪除部分分支，形成未来支援路径。

Heformation借鉴有限可达候选和传输窗口，不迁入EKF或以定位协方差替代控制报告交付。本项目USV实际可达性继续由Otter有限运动与coast检查确认，不能继承论文的运动假设。

[作者全文](https://jonatansw.github.io/files/papers/jswiros2019.pdf)。当前未核实官方源码；本地文件与哈希见manifest。
