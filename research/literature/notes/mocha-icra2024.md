# MOCHA ICRA 2024

论文：Enabling Large-scale Heterogeneous Collaboration with Opportunistic Communications，DOI `10.1109/ICRA57147.2024.10611469`。
作者预印本已取得，7页；已核查摘要和官方源码接口，不标记为全文逐段复核。

研究在间歇连接下使用gossip交换信息，并通过空中节点运动增加可交换的信息。输入是机器人本地信息、可用连接和任务，输出是信息交换及通信感知运动。

对Heformation的作用是允许暂时离线、保留缓存，并把信息可用性用于任务接续。本文无线电实机结果不作为本项目声学模型保证；不迁入PX4或gossip框架。

[原文](https://arxiv.org/abs/2309.15975)、[MOCHA源码](https://github.com/KumarRobotics/MOCHA)、[air_router源码](https://github.com/KumarRobotics/air_router)。本地文件与哈希见manifest。
