# Gosrich等：联盟收益与任务前置关系

文献：Multi-Robot Coordination and Cooperation with Task Precedence Relationships，ICRA 2023，5800–5806，DOI 10.1109/ICRA48891.2023.10160998。

出处核对：[作者实验室出版列表](https://www.modlabupenn.org/publications/)。正文副本：[arXiv:2209.14417](https://arxiv.org/pdf/2209.14417)；本地`papers/mrta/gosrich-coordination-icra2023.pdf`，1,627,192字节，SHA256 `65ec417262d81d8183855848cf3dcb135506e6adeadd5944e489f8fca4e2bb8a`。本轮重点阅读§III–V，不标全篇精读。

该文用任务DAG、联盟收益及前驱收益影响函数表达多机协作；区分人数增加改善效率的任务和达到必要人数才可完成的任务。本文问题定义采用同类机器人、固定任务时长，收益函数由模型给定；不能把其函数直接当本项目编队观测收益或相机评分。

对当前设计的约束：用户不应手选哪个AAV入水/何时编队；分配器应比较有实际资格和作业收益依据的方法。仅增加formation标志并不能证明组合方法有收益；在没有明确共同几何/同时作业要求或同条件实测收益时，选择独立执行是允许结果。

反例：让编队的收益和独立运动相同，但增加集结等待。如果优化器仍无条件选编队，说明它仍是展示脚本而非决策。反向测试必须有真实声明的协同作业条件，不能人为给编队负成本使之获选。

代码落点：`formation_mission_runner.py::_executor_plan`当前末尾追加编队；`executors.py::build_executor_plan`及其完整候选入口应评估方法而不是固定插入；保留原Swarm运动后端，不移植本论文网络流系统。
