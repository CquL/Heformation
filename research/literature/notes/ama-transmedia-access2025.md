# AMA跨介质执行 IEEE Access 2025

论文：A Modular Execution Architecture for Robust Multi-Robot Planning and Acting in Trans-Media Environments，DOI `10.1109/ACCESS.2025.3625646`。已取得HAL v2作者稿47页，封页标注CC BY 4.0；出版社版页码不同。已核查摘要、规划执行接口与跨介质动作相关段落。

AMA以PDDL规划、STN时序监督和分布式BT执行协调跨介质任务，监测故障并修复受影响部分。其已有角色、sample／translate_data等动作是创新比较的重要先例。

本项目参考执行因果及局部修复，继续使用原ROS1请求、Plan和Action，不迁入PlanSys2。论文的符号动作语义不等于经过有限声学和无线链路的实际字节交付。

[HAL](https://laas.hal.science/hal-05244656v2)、[官方源码](https://github.com/ViDLR/Ros2PLAN_ws)。本地文件与哈希见manifest。
