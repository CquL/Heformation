# server715 迁移与 B/C 全链继续入口

> 本文件下方为迁移开始时的历史快照。Docker数据盘、服务器Codex与聊天迁移已完成；当前r67失败与接手入口以 [2026-10-03项目总接手文档](server715-project-handoff-20261003.md) 为准。


本文件记录迁移前实际事实。业务范围保持控制侧：三种任务的输入、联合分配与调度、运动规划、跨域控制、实际执行报告与会话调整；不承担专业损伤识别或精细重建。

## 工作位置

- GitHub：`https://github.com/CquL/Heformation`，只使用 `main`。
- SSH：`server715`；服务器目录 `/home/server715/data/lhj/codes/Heformation`。
- 本地控制目录 `/home/lhj/codex-remote/Heformation-server715`。后续文件修改和运行通过 SSH 在服务器进行。
- 项目源代码、配置、模型、许可证、文献及项目 PDF/Word 入库；实验数据、bag、构建缓存、凭据和私有聊天不入库。
- 聊天独立存放 `/home/server715/data/lhj/codex-migrations/Heformation`；仅迁移本项目可见内容与引用附件，不复制账号认证、其他项目或隐藏推理。

## 实际状态

B/C **没有完成全链验收**。最近实际 B 运行 r59：两台 AIR 完整86区间、跨介质 AAV 基础三层60区间均100%，母船实收三个 `INSPECTION_CONTROL_REPORT`，AIR 正常终态已释放。UUV 只完成两层66.67%；AAV 尚未出水；USV 第二共享服务 `OBSERVATION_TIMEOUT_UNVERIFIED`。C 已确认排队但未激活。失败 Result 与剩余三个锁不升级。

r60 仅生成完整 B 预览，规划0.008875墙上秒，迁移前明确 decline，最终 `NOT_CONFIRMED`，资源锁为空，未派发运动 Goal，bag闭合且本地世界退出。不能把它记作实际业务运行。

最新三项 Python 修正仍需新同次完整验证：

1. WATER 已完成真实合格区间且减速到原阈值后，终端参考绑定实际停止位置，避免要求已停住的平台继续到名义末顶点；实际4模型秒/速度0.03/原安全终态不变。
2. WATER/PVS 导航球体与占据 voxel 按欧氏距离校核，修正方盒膨胀造成的安全角隅误拒绝；AIR 仍与 Swarm 的盒状包络一致，未知空间仍拒绝。
3. USV 共享服务按已声明支援区域半径验收，保持原实际减速与连续停稳要求；普通航行与返回不变。

## 同一系统链路与代码位置

```text
Qt选择A区域/B风机/C平台和连续管段
→ monitoring_request.py展开任务及允许方法
→ mrta_python/executors.py 当前状态下首个完整联合Plan
→ 用户确认 / 执行中选择queue或replace
→ formation_mission_runner.py原Action与资源锁派发
→ AIR: formation_action_server.py + Swarm + qn
→ WATER: platform_action.py + qn（UUV固定WATER）
→ USV: pvs_node.py + 原Otter/PVS共享服务
→ inspection_work.py实际合格区间并集
→ 原产品/母船实收 + 匹配GoalID的实际终态
→ 目标区HOLDING / 下一任务 / 人工Swarm共同返航
```

`inspection_work.py` 是唯一新增共享运行文件，只做几何和实际进度。ROS、资源锁、Plan、会话继续在原位置。联合求解预算10墙上秒，取得首个完整任务级可派发方案即返回，不宣称最优或逐候选全动力学证明。当前位置到入口、连接和绕障由执行端规划。

速度配置：AIR planner0.6/nominal0.5m/s（用户要求降半）、AAV/UUV WATER0.225m/s、USV nominal0.6m/s/60N、PVS局部预算2墙上秒、时钟目标2×。scene净距0.2m/member净距0.5m、实际区间/终态/GoalID/旧结果拒绝不改。数值为本工程演示配置，不是文献规定的常数或普适保证。

## 服务器运行准备

服务器 Ubuntu22.04、64核、双RTX4090；SSH可达。检查时尚无Docker，用户负责安装 Docker 并将 server715 加入 docker 组；无需复制本地凭据。ROS Noetic 仍在原 Ubuntu20.04 镜像中，宿主不另建第二 ROS 栈。

原构建入口 `scripts/docker_build_qn.sh`，业务入口 `scripts/docker_run_three_class_qualification.sh`。当前设施 native 镜像 `swarm-formation-qn:facility-build`，SHA `ba3b2396841671d61e266e7703fc7c6f70d1a460c1590d1aee5459f9132e79b7`；可迁移此可复用镜像缓存，但项目文件必须从 GitHub 精确 commit 获取。Python 每次新进程从当前源码挂载，不热改进行中的旧世界。

```bash
JOINT_IMAGE=swarm-formation-qn:facility-build \
JOINT_SIM_SPEED=2 JOINT_GPU_RENDER=false JOINT_REGION_UI=false \
JOINT_REQUEST_FILE=/workspace/src/src/qn_aav_simulator/config/monitoring_request_wind.yaml \
bash scripts/docker_run_three_class_qualification.sh experiments/NEW-UNIQUE-RUN
```

运行需要可访问的 DISPLAY、中文字体、Xauthority。Qt 控制台与 RViz 独立，均展示当前实际任务状态。服务器运行时不得把轨迹回放或图片当协同实跑。

## 继续验收（只推进完整链）

1. 完整 B：两风机所选水上结构和三层水下基础均实际完成，重叠活动，共享支援、实际 Action 终态和规定报告实收，随后 HOLDING。
2. 排队完整 C：平台三层外侧/三层水下外部结构及完整连续管段0..3，含三个局部作业4秒。此次完整演示至少一次由求解器选择 AAV 入水/作业/出水；不固定其身份、不作为所有C的必选。
3. 同会话执行中 replace、B→C queue接续、目标区驻留、人工共同返航、返航中替换局部任务继续完成；AIR返回复用原 Swarm 三机编队。
4. 一次明确复查或支援延迟反馈，确认新安排确实执行。最终要求 HOME 五方实际到位、零锁，明确 end_session。

Operator 文件与 UI 使用同一任务权威：`operator-confirmation.json` 需匹配 request_id/plan_revision；`operator-command.json` 使用 command_id、当前request_id、preview_region/confirm_plan/return_home/end_session。每次先看 metrics.json 当前状态，不重启未结束世界或释放未验证锁。

只修阻塞此链的真实问题。不开展单机资格系列、旧长序列或无关回归，不用多个失败运行的局部100%拼成完成。`docs/WORKLOG.md` 顶部记录计划/实际/效果/证据/未完成，同步 context02/15。
