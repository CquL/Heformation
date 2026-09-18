# 最新交接：受限v9与七机AIR闭环

**2026-09-17｜当前工作：按用户冻结方案实施Calvo v9 restricted-domain Python port与ROS1七机AIR闭环，实施中。**

## 既有基线与历史记录

此前读取本地V2实施方案/核查记录及其接续说明；整理上游选型、架构、最少输入输出、
通信范围、实施计划和研究状态。随后拉取并复现Swarm-Formation，接入qn AIR平台，结果已记录
在`docs/QN_INTEGRATION.md`和`docs/QN_QUICKSTART.md`。

没有把旧HUC作为新工程基础。Swarm-Formation官方源码已放到`upstream/Swarm-Formation`，
Docker Noetic官方镜像编译通过；qn接入镜像和单机/七机AIR闭环测试通过。当前整理和接入修改
尚未提交或推送。

## 当前工程状态

Swarm-Formation + qn AIR运动基线已经有本地运行证据。当前唯一上层冻结为受限Calvo v9 Python移植，
ROS1 Noetic/actionlib连接固定七机；原始MATLAB及其他候选的独立复现状态另见16。完整任务反馈闭环仍在实施，海洋分支与新理论未完成。

不再保存旧HUC commit为当前工程起点，不再要求旧P0审计。新工作区及所选上游的本地版本在实际工作中记录。

## 当前主实施切片

本轮已写入[plan.md](../plan.md)并同步当前背景决定；`integration/mrta_python/`正在完善受限v9任务排序、单一Plan状态和正延迟时间修复；
`Formation.action`与任务适配器提供七机AIR组级任务边界。本版`wait_time = 0`，正等待吸收、多执行单元及v13剩余任务重新分配均不在本轮验收。
本轮重跑既有4项Python计划/repair测试与Action服务冒烟通过；已有Action生成和qn镜像构建记录保留。
新v9行为测试、七机实际驻留、A→B→Return与真实延迟修复闭环尚待完整验收，不能据现有冒烟结果提前宣称Test A/B/C通过。

## 下一次最小入口

继续[当前计划](../plan.md)：先完成受限v9自由任务小算例Test B，再完成七机实际动作Test A，最后运行实际延迟驱动更新Plan的Test C。
所有结果保存为可复查实验记录。不要重复进行Calvo/D-ITAGS或ROS1/ROS2选型，也不要求先完成全部候选原例。

## 后续每次只需更新这些事实

本次目标；实际上游/版本；做了什么；真实命令、结果与日志位置；未完成/失败；选型是否变化；下一项最小工作。其他细节可留在原生日志或实验目录。

## 当前提醒

第一版目标是完整联合协调闭环，原始复现只是第一步。上层和运动后端是待组合，不是原作者已经共同验证。三条旧“创新”仍属研究候选，不能为了符合名称强行增加信息流、中继或误差证书。

## Skill 资料库补充交接（2026-09-17）

按用户确认清单，使用系统`skill-installer`并显式指定`--ref`、`--path`和`--dest`，
将9个来源的21个Skill下载到`/home/lhj/skill-library/`。原始目录、附带资源和适用许可已保留；
21个主入口及31个本地索引链接核对通过。来源版本、使用条件、仅收录链接的工具和待编写建议见
[中文索引](/home/lhj/skill-library/README.md)。没有最终下载失败项；未接入宿主、运行附带脚本或安装依赖。
当前交付为独立资料库，后续实际启用按具体任务另行选择，不改变海上集群工程现有实施状态。


---
整理依据：[V2实施方案](sources/implementation_plan_v2_2026-09-15.md)与[V2核查记录](sources/literature_audit_2026-09-15.md)。V2来源保留为历史依据；当前决定按用户最新冻结方案更新，实验完成情况仅以实际运行记录为准。返回：[资料索引](README.md)。

## Codex 配置补充交接（2026-09-17）

`/home/lhj/.codex/config.toml`已设置1,000,000-token上下文窗口和900,000-token自动压缩阈值，
保留现有模型及服务配置，使用Python `tomllib`完成解析与配置值校验。
字段依据：[OpenAI官方配置参考](https://developers.openai.com/codex/config-reference/)。
实际1M请求能力尚未经当前API服务验证；使用新配置需让客户端重新加载配置。
