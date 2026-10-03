# Guo / Zavlanos：巡检作业与共享支援的参考边界

核查日期：2026-10-01。用于 B/C 模板设计；本轮无运行代码、测试或仿真。

- 发表记录：*Multirobot Data Gathering under Buffer Constraints and Intermittent Communication*，Meng Guo、Michael M. Zavlanos，IEEE T-RO 34(4):1082–1097，2018，DOI 10.1109/TRO.2018.2830370。[Duke 官方记录](https://scholars.duke.edu/publication/1322925)。
- 复用已有 [PDF](../papers/task_motion/guo-zavlanos-2018.pdf)，不是重新下载副本。PDF 首页为 arXiv:1706.02092v2（2017-10-30）作者预印本，对应上述 2018 发表记录；不把本地稿标为正式出版排版版。
- 本轮阅读：摘要、引言、§III 问题模型/Remarks 1–2及会合协调说明的重点内容；不把本轮重读标成新一次全文阅读。原取得信息保留在 manifest。

原文把机器人分为收集数据的 source 与收集转发的 relay，通过运行期安排会合协调各自任务；无需全队时时保持连通。§III-A 中 source 只有有限距离无线通信，relay 可长距离上传；Remark 1 将后者说明为假设，也可改为访问固定数据中心。Remark 2 的 relay 没有自己的局部采集任务，不能直接推出本项目可随意让忙碌 AAV 同时承担全部作业与中继。

任务模型的缓冲容量、传输事件和 LTL 约束是本文问题的重要组成，本项目并不移植这套模型。§III-A 明确二维区域与点质点机器人，且不考虑机器人间碰撞；论文结论不能替代 Swarm/qn/PVS 的实际避碰与控制资格。

本项目借鉴的是“对象作业成员＋移动共享支援成员＋必要会合事件”的组织：USV 服务位置由水下作业及任务级接收条件决定，同一服务可被多个作业引用；不能以 USV 到达就认定各产品已收到，也不要求所有成员同速同行。母船接收继续使用明确的任务级抽象，不宣称声学/RF 协议、网络链路或论文缓冲保证已经实现。

直接设计参见 [B/C 模板](../../../docs/requirements/inspection-business-templates-20261001.md)。
