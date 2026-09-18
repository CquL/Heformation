# 当前集成与上游原始复现的边界

**2026-09-17：当前按[plan.md](../plan.md)实施Calvo v9 restricted-domain Python port与ROS1七机AIR闭环。本版wait_time为零，无v13重分配；正等待和多执行单元测试后移。**

## 1. V2原始复现基线记录

| 对象 | 原始小例子目标 | 当前状态 |
|---|---|---|
| Swarm-Formation [R01] | 保存用户已有编队复现的版本、原生入口和代表结果 | 已在`upstream/Swarm-Formation`用Docker Noetic编译，并完成qn AIR七机闭环 |
| Primitive-Planner [R03] | 运行官方多机导航例子，理解基元生成、目标与输出 | 基元库生成成功；Noetic 临时工作区 18 包编译通过；完整 ROS 启动待继续 |
| Calvo规划/执行 [R09,R10] | 先做最小分配与延迟修复，再查看原生执行入口 | 已拉取并核查入口；MATLAB/Gurobi 环境阻塞原始运行 |
| D-ITAGS [R08] | 运行任务—调度—运动交错小例，检查所需依赖/数据 | CMake 已执行；缺少 nlohmann_json 配置包，且完整实验需要 Gurobi/OMPL |

这些是原始复现记录，不要求先在一个环境安装四套系统。Swarm + qn AIR是当前运动基线；
受限Python移植已获用户授权，但仍须与原始MATLAB复现区分，不能声称完整等价。

本阶段的所有上游复现结果集中记录在[16_upstream_reproduction_status.md](16_upstream_reproduction_status.md)。
原始复现实验保持隔离；当前已进入单独授权的Python移植与集成工作。

## 2. 本阶段已冻结上层

本阶段选择Calvo v9 restricted-domain Python port，禁用电池约束、充电、fragmentation、relay与动态联盟人数，只移植启用奖励和已有协作关系下的时间修复。原MATLAB及D-ITAGS的运行条件不阻塞这项已授权工作。

最终只有一个在线主求解器。另一个可以离线比较或后续替换。Calvo模型不含一般前置关系，D-ITAGS也不自动提供完整电池/充电模型；不能用两个仓库的标题拼出不存在的共同能力。[R08–R10](13_references.md)

## 3. 如何连接执行器和规划器

本阶段采用用户冻结的Python模型与FormationAction，连接Swarm原生组级目标和七台qn Odometry。只补“动作映射到目标”和“实际运动结果映射到DelayEvent”的必要路径，不要求先引入Calvo原生执行框架。

D-ITAGS与Calvo执行器如需结合，属于新适配工作；不能说前者自带后者的Action。原执行器中的Generic任务时长不表示自动支持任意几何。

旅行时间估计沿用上游可用方法。运动后端只做在线规划时，先用实际耗时/延迟触发上层修复；需要提前查询且原方法没有时才补小接口，不预先设计多层能力报告。

## 4. 后端选项不是插件已完成

Primitive负责UAV一般转场和多机避让候选；Swarm负责已有编队基线。CAT-ORA只有满足原假设且确有开阔区重排动作才加入。RMADER用于延迟对照，DMPC-Swarm用于通信—控制对照，AMSwarmX用于复杂障碍对照。

开始时分实验比较，不在同一执行器上同时启用多套规划器。跨任务切换需要专门验证状态、时间与命令层级，不把配置字段写出来就当作支持热切换。

## 5. 环境策略

当前集成固定使用ROS1 Noetic Docker和actionlib；原始候选复现仍按上游测试环境隔离。长期部署环境可后续评估，不在本轮迁移ROS或系统。

MATLAB/Gurobi等许可或依赖要确认。不能把许可证写入仓库，不能把无法运行原版后自行替换求解器记作原始复现。后续移植必须在同一小例对照约束与结果。

## 6. 建议的新工作区

```text
maritime_mission_system/
  upstream/       # 选定、固定版本的原始成果
  integration/    # 必要的动作与结果薄适配
  scenarios/      # 第一场景任务、平台、地图与事件
  experiments/    # 原始复现和统一实验入口
  docs/           # 选型理由、少量接口对照与结果
```

这是职责建议，不是必须先空建所有目录。旧HUC不迁入、不参考。资料包自身的AGENT/prompt/content放在项目根目录即可，不当运行时框架。

## 7. 每个工作单元只保留必要证据

写清：上游与版本、实际命令和配置、原始或修改版、成功/失败、日志位置、下一步。保留失败例子与用户原有修改，不重命名全部topic、不重写可复用控制器、不同时启用两个模拟器。

可选使用[上游复现记录](templates/upstream_record.md)，不把填写模板变成实施阻塞。


---
整理依据：[V2实施方案](sources/implementation_plan_v2_2026-09-15.md)与[V2核查记录](sources/literature_audit_2026-09-15.md)。V2来源保留为历史依据；当前决定按用户最新冻结方案更新，实验完成情况仅以实际运行记录为准。返回：[资料索引](README.md)。
