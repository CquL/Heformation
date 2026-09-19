# 工作日志（计划 / 实际 / 效果）

这个文件是项目的持续记忆：**每次改动仓库或跑实验，都在最上面追加一条**。目的是让下一次（人或 agent）
不看对话历史也能知道：当时计划做什么、实际做了什么、效果如何、哪里与计划不一致、下一步是什么。

维护约定：

- 只追加，不重写历史条目；旧的更正用新条目说明。
- 每条包含五个字段：计划 / 实际 / 效果 / 证据 / 未完成与下一步。
- 「实际」与「计划」不一致时，写在「实际」里并说明原因——这是最有价值的部分。
- 证据只写可复查的东西：提交号、实验目录、命令、测试数、指标。
- 不写没跑过的结果，不把"代码写了"写成"验证通过"。

---

## 2026-09-19 · 阶段 1 下半：三机在线配置与组级目标验证（提交 `ea02797`）

- **计划**：修正上一轮的过早结论——计划第五节的**阶段 1 交付包含两次探针**：
  （a）单独派发 AAV2；（b）**再验证三机组级目标**；并确认每台仅一个 qn、仅一条被接受参考。
  我只做了 (a) 就宣布阶段 1 完成，(b) 需要三机在线配置，而同一份配置也是第二节
  "受控扩展为 3 机规模"、第三节"岸线编队阶段（3 AAV）"与第六节"两套互斥静态配置"的前提。
- **实际**：
  - 新增第 5 个构建期补丁 `plan_manage_formation_config_arg.patch`：把编队配置文件从
    `advanced_param.xml` 里硬编码的 `normal_hexagon.yaml` 变成 launch 参数，**默认值不变**，
    因此七机配置行为不受影响。三机配置需要自己的槽位布局与队形形状，而该路径原本无法从外部覆盖。
  - 新增 `config/formation_aav3.yaml`：三机横向一字、间距 2 m、scale 1.0，
    中心 `(-30, 6, 0.8)`；并显式写明成员 3–6 在该配置下不存在，不是遗漏。
  - 新增 `launch/formation_aav3.launch`：三个 `run_in_sim` 实例 + swarm_bridge + 场景发布器，
    并为每台设置**各自的编队目标话题**（`/drone_i_formation_goal`）——这正是上游"全局单话题"做不到的事。
  - 新增 `probe_group_goal.py` + `scripts/docker_probe_group_goal.sh`。
- **效果（实测，`./scripts/docker_probe_group_goal.sh`）**：向三台各自的编队目标话题发布同一个编队中心
  `(-24, 6, 0.8)`：
  - `drone_0 (-24, 6, 0.8)` / `drone_1 (-24, 4, 0.8)` / `drone_2 (-24, 8, 0.8)`，
    三者对 `centre + scale·slot_i` 的误差均为 **0.000 m**；
  - 每台 `/drone_i_qn/odometry` 的发布者数 **= 1**（每台只有一个权威动力学源）。
  → 阶段 1 的两次探针现在都通过，**阶段 1 才算真正完成**。
- **过程中的自伤**：探针第一次报 `publishers=4 FAIL`，是我自己的解析 bug——`rostopic info` 的
  **发布者与订阅者都以 `*` 开头**，我把 1 个发布者 + 3 个订阅者一起数了。只统计 Publishers 段后通过。
  这类"检查器自己有 bug"的情况必须排除，否则会把解析错误记成系统缺陷。
- **证据**：`integration/swarm_qn_bridge/patches/plan_manage_formation_config_arg.patch`（已 dry-run 校验）；
  `integration/qn_aav_simulator/{config/formation_aav3.yaml,launch/formation_aav3.launch,scripts/probe_group_goal.py}`；
  182 项测试通过。
- **未完成**：阶段 2 下半（runner 侧"同一成员不得属于两个同时活跃单元"的运行时校验）；
  阶段 3（作业请求→展开→覆盖/交付→复测→实时展示，含岸线编队阶段）；
  阶段 4（M2 收尾、口径修复、文档）。三机配置目前只有"组级目标"这一条通路，
  **尚未接入 Action server 与 executor 路由**（阶段 3 的事）。

---

## 2026-09-19 · 阶段 2（上半）：共享成员的执行单元模型（提交 `c8a75f2`）

- **计划**：计划《近岸监测任务线 + M2 收尾》第二节——允许静态成员重叠、禁止重叠单元同时占用、
  实际状态与计划状态分离、单机任务默认不交给编队单元、组级转场代价按最晚成员计。
- **实际**（`integration/mrta_python/executors.py`、`models.py`）：
  - `validate_executor_inputs` **不再拒绝静态重叠**：`aav_1={0}` 与 `aav_formation={0,1,2}`
    是同一批物理平台的两种用法，不是两批平台。仍拒绝同一单元内重复成员与重复 executor_id。
    重叠关系记入模块级 `overlap_pairs`，供计划校验使用。
  - `validate_executor_plan` 增加**互斥校验**（对应 CoCoPlan 的 mutual exclusion）：
    共享物理成员的两个单元不得有相交的时间区间。
  - `build_executor_plan` 增加 `earliest_start()`：单元的可用起点取**自身队列**与**所有重叠单元队列**
    的最大值——否则规划器会给两个共享成员的单元都排零时刻，再由 runner 强行排队，把人为串行造成的
    等待解释成执行延迟。这一条是被测试逼出来的：先只加校验时，规划器自己产出了重叠计划。
  - **单机任务默认排除编队单元**：`eligible_executors` 改为"能力匹配 且 规模相等"，
    只有任务显式 `allow_larger_unit=True` 时才接受更大的单元。`Task` 新增该字段（默认 False）。
    依据：CARIC 明确指出把少量机器人分组不一定更快，且更大的单元会拖着任务没要求的成员走。
  - **组级代价按最晚成员计**：`ExecutorTravelTimeProvider` 支持 `member_slots` 与 `member_positions`；
    有成员明细时按"每个成员从**它实际所在**到目标槽位"的最大值算，而非中心点平均。
    `member_positions` 是**执行反馈写入的实际状态**，与规划器内部推演的 `queue_finish`/`current_target_ref`
    （计划状态）分开，不混成一份持续覆盖的变量。
- **效果**：182 项单测通过（新增 4 项：静态重叠合法、重叠单元不得同时占用、单机任务默认不给编队单元、
  组级代价由最晚成员决定）。其中"重叠单元不得同时占用"同时覆盖了**规划器不产出**与**校验器拒绝**
  两条路径（手写的重叠计划必须被拒）。
- **过程中的自伤**：第一次插入互斥校验时把 `validate_executor_plan` 的 per-item 循环切断了，
  队列顺序检查被孤立到错误的循环里 → 8 个原有测试失败。已在同一轮修回，不是遗留缺陷。
- **证据**：`integration/mrta_python/tests/test_executors.py` 新增用例；全量 182 passed。
- **未完成（阶段 2 下半）**：runner 侧的"同一成员不得属于两个**同时活跃**的单元"运行时校验尚未加；
  三机在线配置仍未建立；阶段 3（作业请求→展开→覆盖/交付→复测→实时展示）与阶段 4（M2 收尾、文档）未开始。

---

## 2026-09-19 · 按 wenxianSKILL 补齐文献与参考源码（提交 `c666af7`）

- **计划**：按 `prompt/wenxianSKILL.md` 把 P0/P1 里还缺的全文补齐、把有源码的项目拉到 `upstream/` 下参考，
  并按 Skill 推荐的 `papers/<category>/` 结构 + `RESEARCH_MAP.md` 整理。
- **实际**：
  - 补齐 P0 缺口：**Swarm-Formation ICRA 2022**（真名 `Distributed Swarm Trajectory Optimization for
    Formation Flight in Dense Environments`，arXiv **2109.07682**——与当前源码直接对应的就是这一篇；
    我们此前拿的是 T-RO 版 2210.04048）。另补 `minco-2022`（GCOPTER 的轨迹表示，Swarm-Formation README 自述建立其上）
    与 `usv-uuv-integrated-system-2020`（USV–UUV **有线缆绳**耦合，第三种形态）。
  - 按要求重组为 `papers/{mrta,task_motion,formation,safety,maritime,benchmark}/`，笔记里的本地 PDF 行同步更新。
  - 新增 `research/literature/RESEARCH_MAP.md`：按 Skill 的七类状态标签（已采用/已复现/已全文阅读/已重点参考/
    候选/Benchmark·工具/历史参考）维护，并列出"尚未取得全文的已知缺口"。
  - 拉取 4 个参考源码到 `upstream/`：`GCOPTER`（内含 `minco.hpp`）、`libMultiRobotPlanning`、
    `stonefish_ros`、`CARIC`（官方站点/文档仓库，**不是**评测仿真栈）。MINCO 独立仓库不存在（已在 GCOPTER 内）。
  - **踩到并修掉的两个坑**：
    1. 把 PDF 移进子目录后，原 `.gitignore` 的 `papers/*.pdf` **只匹配顶层**，PDF 一度被 `git add` 暂存；
       已改为 `papers/**/*.pdf` 并确认重新被忽略。
    2. 四个新克隆合计约 190 MB（既有 15 个 `upstream/` 副本是当作普通文件提交的）。
       本轮**没有**把它们入库，加入 `.gitignore` 并在 `upstream/README.md` 里写明是本地参考副本、
       删掉四行即可入库——这个体量决定应由用户拍板，不擅自把仓库再撑大。
  - `prompt/wenxianSKILL.md` 本身也纳入版本管理（`prompt/` 其余文件本就在库内）。
- **效果**：manifest 16 条（14 全文 + 2 摘要级），`pdf_file`/`notes` 路径脚本校验无断链；
  新增 3 篇 4 段式笔记（问题 / 任务·资源·通信·运动模型 / 影响的接口 / 要求的反例实验）。
  最有价值的一条：`weight_formation` 的语义被原论文钉死为"集群图与期望队形的可微相似度距离"，
  这**从原理上解释了**阶段 1 探针里"单独一台成员被拉向原点"的现象，而不是把它当成偶发 bug。
- **证据**：`research/literature/RESEARCH_MAP.md`、`manifest.yaml`、`notes/*.md`（16 篇）、
  `upstream/README.md`；`git check-ignore` 确认 PDF 与四个新副本均未入库。
- **未完成**：USV–AUV TMC 2025 多轮检索未命中，已记入 RESEARCH_MAP 的缺口表，不据此写任何结论；
  MIMRee / SeaClear / Stonefish·VRX·DAVE 文档等 P1 项留到对应功能实现时再取。

---

## 2026-09-19 · 文献全文补充：巡检完成判据、海上协同、任务—通信耦合（提交 `116fe5c`）

- **计划**：把与接口直接相关的文献补到全文级——CARIC（巡检任务定义与观测完成判据）、Xiroi II（USV 中继）、
  USV-UUV 协同、CoCoPlan（任务—通信联合）、Swarm-Formation / GRSTAPS / APEX-MR（已有）。
  每篇只提取三类内容：它解决什么问题、改变我们哪个接口或约束、要求增加什么反例或实验。
  目的是**防止我们自己拍脑袋定义任务接口、完成条件和协同关系**，不是再堆算法。
- **实际**：
  - 新增全文 5 篇（`research/literature/papers/`，PDF 不入库）：`caric-2025`、`cocoplan-2026`、
    `xiroi-ii-2023`、`usv-uuv-cooperation-2023`、`auv-collaborative-missions-2020`；
    每篇均以 `file` 与首页文本核对（标题自证），记录了字节数与 sha256。
  - CARIC 的真名是 **Cooperative Aerial Robot Inspection Challenge**（arXiv:2501.06566；比赛在 CDC 2023 /
    IROS 2024 Workshop）；用 OpenAlex/arXiv API 定位，MDPI 需走 `mdpi-res.com` 直链才能拿到 PDF
    （`www.mdpi.com/.../pdf` 返回 HTML，已记为抓取注意事项）。
  - `joe-2022-marine-heterogeneous-collaboration` 为**闭源**：Unpaywall `is_oa=false`、
    `has_repository_copy=false`、`best_oa_location=null`，OpenAlex `oa_status=closed`，期刊不在 DOAJ
    → 记为 `not-available` / `abstract-only`，笔记中明确声明只依据摘要、不引用正文结论。
    GRSTAPS 原有条目同样是摘要级，保持不变。
  - 新增 6 条笔记 + manifest 6 条记录（`read_scope` 与 `pdf_status` 一致，已脚本校验无矛盾、无断链）。
- **效果（这些文献直接改了我们的口径）**：
  1. **观测完成判据**：CARIC 的 `q = q_seen·q_blur·q_res` 说明"在半径内 + 驻留"只是 `q_seen`，
     还缺运动模糊与分辨率两维；`Q = Σ_i max_j q_ij` 给出**每兴趣点取全局最优一次**的去重规则；
     且"未碰撞者才计分"比我们"coverage=1 不能覆盖安全失败"更强（不是扣分，是**不予计分**）。
  2. **复测与依赖**：CoCoPlan 的任务完成是 (I) 到达 + 连续执行 η **且** (II) 满足全部时间约束；
     我们只有 (I)。其三类时间关系（precedence / **mutual exclusion** / concurrency）正好把
     "重叠执行单元不得同时占用"变成一条显式计划约束，而不是 runner 里的串行约定。
  3. **中继不是到位**：Xiroi II 的 ASV 按与 AUV 的距离分跟踪/漂移/排斥三区，"靠得够近"与"不碰撞"
     同时生效，效果用**声学定位频率 0.117→0.468 Hz** 量化；JOE 2022（摘要级）进一步说 USV 自身轨迹
     的目的是**约束 UUV 的状态估计误差**。→ 我们的任务模型缺"支援类"任务。
  4. **失联必须有声明式应急策略**（USV-UUV 综述：否则失控运行到耗尽/搁浅）；
     水下交付零延迟只能作为**显式声明**，不能是默认。
  5. **不要为拆队而拆队 / 单次运行不可当结论**：CARIC 明确"把少量机器人分成多组不一定更快"，
     且同场景不同运行分差可超 1000 分、出现过全队空闲的失败运行。
- **证据**：`research/literature/manifest.yaml`（13 条，含 sha256 与 `read_scope`）；
  `research/literature/notes/` 13 篇；上面每条结论都可在对应笔记里查到出处小节。
- **未完成与下一步**：`context/12_literature_review.md` 与 `13_references.md` 尚未并入这批新条目
  （属计划阶段 4 的文档工作）；JOE 2022 若要用作接口依据需先取得正文并升级笔记。

---

## 2026-09-19 · 阶段 1：单机指令隔离（提交 `a78a09a`）

- **计划**：《近岸监测任务线 + M2 收尾》阶段 1——把"单机运动 / 三机编队 / 待机保持"三种目标含义在全层统一，
  并证明"只派发一台"真的只动那一台。这是整条任务线的门，不通过后面都不能做。
- **实际**（本轮只做到阶段 1，阶段 2–4 未做）：
  - 核对后确认三处上游障碍，全部实测过：所有规划器订阅同一个全局 `/move_base_simple/goal`
    （`ego_replan_fsm.cpp:118`）；`formationWaypointCallback` 恒按 `end_pt_ = c + s·r_i` 加槽位偏移
    （`:650`）；`SEQUENTIAL_START` 让 `drone_id >= 1` 等前序成员轨迹（`:158-160`）。
  - 新增构建期补丁 `patches/plan_manage_member_goal_entry.patch`：目标话题参数化（默认值不变）、
    新增成员入口（位置**按原值使用**、不等前序、**不用编队代价**）、编队入口行为不变。
  - Action server：目标路由按单元规模分派——单成员单元 → `MEMBER_TARGET`；多成员单元 →
    `FORMATION_CENTRE`；readiness 改为逐个成员校验其目标话题订阅与连接；新增 `goal_semantics`
    与 `goal_topics` 诊断字段。默认模板下七机行为逐字节不变。
  - 采纳追踪：一次派发只计**一个** dispatch goal，即使多成员单元在多条话题上各发一条；
    同时记录话题与发布者，外来发布者仍会被 `group_goal_authorized` 抓到。
  - 新增 `probe_member_goal.launch` + `probe_member_goal.py` + `docker_probe_member_goal.sh`。
- **效果（实测，`./scripts/docker_probe_member_goal.sh`）**：向 drone_2 的成员入口发一个
  横向 1.5 m 的目标：
  - `drone_2 target = (-24.500, -4.000, 0.800)`，`actual = (-24.500, -4.000, 0.800)`，
    水平误差 **0.000 m**，y 误差 **0.000 m** → **没有叠加槽位偏移**（若叠加会是 4 m）。
  - 其余 6 台位移 **0.0** → 未收到该目标且保持在待机容差内。
  - 对照实验：同一环境发编队中心目标到 `/move_base_simple/goal`，七台整体平移且位移一致
    （4.272 m = 中心位移模长）→ 编队路径未被破坏。
- **过程中打掉的两个自己引入的缺陷（探针直接抓到，值得记）**：
  1. 成员回调里写了 `Eigen::Vector3d end_pt(...)` —— **局部变量遮蔽了成员 `end_pt_`**，
     成员保持默认 (0,0,0)，结果被寻址的成员一路飞到**原点**（轨迹就是一条从 (-26,-4) 到 (0,0) 的直线）。
     日志 `member goal received: -24.500 -4.000 0.800` 与 `end=(0.000 0.000 0.000)` 并置才定位到。
  2. 成员分支最初仍带编队代价，单独一台对着退化的 swarm 图优化会被拉向编队构型；
     已改为成员分支 `use_formation=false`。
  这两个都不是"接口没连上"，而是**语义接错了**——正是评审要求把指令隔离贯穿到参考层的原因。
- **证据**：`integration/swarm_qn_bridge/patches/plan_manage_member_goal_entry.patch`（构建期应用，
  已 `patch --dry-run` 校验、镜像内源码与二进制符号均已确认）；`scripts/docker_probe_member_goal.sh`
  的上面三项输出；178 项单测通过（新增 11 项：`test_goal_routing.py` 8 项 + 追踪语义 3 项）。
- **未完成与下一步**：阶段 2（共享成员执行单元：允许静态重叠、禁止同时占用、实际/计划状态分离、
  串行计划时间）、阶段 3（作业请求 → 展开 → 覆盖/交付 → 复测 → 实时展示）、阶段 4（M2 收尾）
  与文档更新全部未做。三机在线配置尚未建立（本轮探针用的是七机栈只开一个成员入口）。

---

## 2026-09-19 · 全量推送核查：发现本地克隆是 shallow（提交 `ee7da62`）

- **计划**：用户要求"把整个项目 push 到 GitHub"。先核查再推，不假设已经推过了。
- **实际**：
  - `main` 与 `origin/main` 完全一致（`4c4611ba`），逐个确认本轮新增文件都在远端
    （两个渲染器、仪表盘、`experiment.rviz`、`docker_render_figures.sh`、`docs/figures/mission_overview.png`、
    `docs/WORKLOG.md`、`docs/QN_INTEGRATION.md`）。`git lfs push --dry-run origin main` 空输出 → LFS 齐全。
  - 发现两个本地分支**从未推送**，推送被 GitHub 拒绝：
    `remote: fatal: did not receive expected object 9219fcab...`
  - 排查结论：
    1. `backup/independent-reproduction-799dcde` 用 **`git push --no-thin`** 推送成功——默认的 thin pack
       会以 shallow 边界作为 delta 基点，而远端没有那个对象。
    2. **本仓库是 shallow 克隆**：`.git/shallow` 里只有 1 个边界提交 `967a4bdf`
       （`Merge pull request #24 from avcuenes/main`）。`pre-heformation-upload` 本地**只有 2 个提交**，
       且 `967a4bd` 不在 `origin/main` 的历史里 → 被截断的祖先在 origin 上根本不存在，
       无法用 `--unshallow` 从 origin 恢复，因此这个分支推不上去。
  - 顺带确认：仓库用 **git-lfs**（`git lfs ls-files` 在 main 上有 62 个文件），
    两个已推分支的 LFS 对象在远端都齐全。
- **效果**：GitHub 上现有 `main`（4c4611ba）与 `backup/independent-reproduction-799dcde`（799dcde2），
  均含完整 LFS；`pre-heformation-upload` 只能留在本地。
- **证据**：`git ls-remote --heads origin`；`git lfs push --dry-run origin <branch>` 空输出；
  `.git/shallow` 内容。
- **未完成与下一步**：
  - 若确实要 `pre-heformation-upload`，需要从**原始远程/上游仓库**取回被截断的历史
    （origin 上不存在），或改为"内容快照"新分支推送——但那不是原历史，不应伪装成原分支。
  - `local-codex-remote-ssh-workspace-SKILL.md` 未提交：它是 Codex 远程 SSH 工作方式的**个人笔记**，
    不是本项目内容。要入库需要用户明确同意。
  - `experiments/`（含 192 MB 的 `execution.bag`）、`data/`、`research/literature/papers/*.pdf`
    按 `.gitignore` 不入库，这是既定设计。

---

## 2026-09-19 · 核对上游随机森林环境：还在发，但没人用（提交 `3efdeb2`）

- **计划**：用户问"Swarm-Formation 当时的仿真环境很好啊，我们不用了吗？是随机生成的吗？"
  这不是改代码的请求，但答案只能靠实测，不能靠读 launch 猜。
- **实际**：
  - 起一次 `formation_air.launch run_mission:=false` 实测两个地图话题：
    - `/map_generator/global_cloud`：发布者 `/random_forest`，**1.0 Hz**，**Subscribers: None**
    - `/scene/global_cloud`：发布者 `/scene_publisher`，**10 Hz**，订阅者是 7 个渲染器
  - 抓了一帧森林点云：**46237 点**，x −15.3…17.1、y −7.6…7.2、z −0.9…3.9。
    按 0.5 m 栅格统计占用：**在我们 0.8 m 巡航高度那一层是 10.7%**（199/1856 格），
    1.5 m 层 11.5%，2.1 m 层 11.4%——整根柱状都占，不是只有高处。
  - 来源确认：`normal_hexagon.launch` 里的 `map_generator/random_forest`，`ObstacleShape/seed=1`
    → **随机撒点但固定种子，每次布局相同**；60 个方形障碍 + 20 个圆柱，34×15×3 m，分辨率 0.1。
    （`simulator.xml` 里那段 random_forest 写在 `<![CDATA[…]]>` 里，是文本不是节点，不生效；
    launch.log 里打印的参数值与 `normal_hexagon.launch` 一致，可确认来源。）
- **结论**：上游那套环境**确实是个更复杂、更真实的障碍场**，而且现在**仍在生成、仍在发布**，
  只是**一个消费者都没有**——纯粹的 CPU 浪费，也是 RViz 显示错地图的来源。
  不用它是有意的：M2 要求"单一地图来源 + 有/无障碍只差一个东西 + 解析几何与点云同源"，
  随机森林下失败无法归因、净距没有解析真值、对照也没法只差一个变量。
- **效果**：把一个"看起来像配置问题"的现象查成了事实：森林是死数据。是否改用森林是一个
  需要先定义净距语义/对照方式/通行走廊的决策，不在本轮擅自更换。
- **证据**：`experiments/20260918-m2-baseline-shm/upstream_forest.npy`（抓到的点云）与
  `upstream_forest.png`（俯视 + 侧视）；`rostopic info` 输出见上。
- **未完成与下一步**：若要用森林，需作为**新场景版本**引入，并先定三件事——净距用点云还是解析、
  有/无对照怎么只差一个变量、编队（scale-2 六边形约 6.9 m 宽）在选定高度是否有可通行走廊。
  顺序上仍应先把 M2 第 3、4 步做完（原盒子场景定位 + 局部扫描中断），否则会同时改动地图、
  几何检查与故障归因，破坏 M2 要的"可解释失败"。

---

## 2026-09-19 · RViz 之前显示的是实验没用的地图（提交 `5641784`）

- **计划**：回答"RViz 的全局点云显示指向 `/map_generator/global_cloud` 是什么意思"。核对源码与
  实际进程后确认这是一个真实的展示缺陷，顺手修掉。
- **实际**：
  - 事实核对：`normal_hexagon.launch` 里上游的 `random_forest` 节点**确实在跑**
    （launch.log 里 `process[random_forest-3]`），发布 `/map_generator/global_cloud`；
    而渲染器的 `~global_map` 被重映射到 `/scene/global_cloud`（`simulator.xml:153`），
    Action server 的 `~global_map_topic` 也来自配置 = `/scene/global_cloud`。
    **仿真用的是我们的场景，RViz 显示的是那棵树**——即"显示了一个任务根本不使用的地图"。
  - 新增 `config/experiment.rviz`：把全局点云显示指向 `/scene/global_cloud`，相机对准作业区
    （焦点 (-24,0,0.8)、近俯视），只保留 `drone0..drone6`，删掉上游的深度 Image 面板与 ESDF。
    `docker_run_qn_demo.sh` 改为用这份配置，不再用 `ego_planner` 的默认 rviz。
  - 另一个实际挡视线的问题：rviz 会弹"ROS Noetic goes end-of-life"模态框盖住画面。
    demo 脚本加 `DISABLE_ROS1_EOL_WARNINGS=1`（对话框自己给的关法）。
  - 顺带核对一个容易被误传的说法：上游 `simulator.xml` 里 `so3_quadrotor_simulator` 那段是
    注释掉的，但我们的植物**不是** `poscmd_2_odom` 理想运动学环，而是 `drone_i_qn_aav`
    （qn 6DOF 模型）——实测进程列表里只有 7 个 `drone_i_qn_aav`。
- **效果**：RViz 现在显示实验真正使用的场景（截图确认橙色障碍盒出现、显示树为
  `Grid / scene / drone0..6`，无 EOL 弹窗、无 "No Image" 空面板）。
- **证据**：`integration/qn_aav_simulator/config/experiment.rviz`；`scripts/docker_run_qn_demo.sh`；
  167 项测试通过。
- **未完成与下一步**：**没有交互式"发布任务"入口**——任务集来自
  `config/formation_air.yaml` 的 `tasks`，跑起来就按序自动执行；要临时发任务只能自己写 action
  client 往 `/formation_action` 发 goal。如果确实需要运行时下发/改任务，这是一个待补的入口。
  M2 第 3、4 步仍未做。

---

## 2026-09-19 · 实时仪表盘（提交 `f044383`）

- **计划**：用户追问"没有实时可视化仿真吗 / 为什么不做实时可视化"。核对后发现：实时三维视图
  **已有**（`./scripts/docker_run_qn_demo.sh mission` 起 RViz），但**任务层**（分配、参考采用、
  判定、时间）没有任何实时视图——上一轮做的两张图都是事后离线渲染。本轮补实时任务层。
- **实际**：
  - 新增只读订阅节点 `mission_dashboard.py`：四面板（场景+槽位布局 / 状态与判定 / 槽位误差 /
    高度与最近接近），输出 matplotlib 窗口（`~window`）和 `~image/compressed` 的 JPEG，
    可给 RViz Image 显示或 `rqt_image_view`。只订阅，不参与控制链。
  - `formation_air.launch` 增加 `dashboard`（默认 false，**opt-in**）；`docker_run_qn_demo.sh`
    支持 `DASHBOARD=true`。
  - `Dockerfile.qn` 增加 `python3-tk`：基础镜像有 matplotlib 但没有 Tk 绑定，否则只能发图不能开窗。
  - `docker_run_qn_demo.sh` 原本只把 `$1` 传进容器，`DASHBOARD` 需要额外传参，一并改了。
  - **发现并修掉的两个真问题**：
    1. `live` 槽位误差一开始是"用当前中心回算整段历史"，于是每次派发都会让历史曲线整体跳变，
       看起来像误差突然出现。改成按每个样本自己的时刻查中心时间线——与离线图同语义。
       这个问题在真实运行里同样会出现，不只是测试环境的假象。
    2. 缺 `/formation_action_server/relative_slots` 时会静默退化成"所有成员槽位都是原点"，
       图上表现为一条平直的 4.0 m 误差线。现在会显式告警。
  - 另一个自己引入的 bug：场景标题插在了 `centre` 赋值之前 → `UnboundLocalError`，节点渲染一帧后
    就死。用 `rosbag play` 复放录制的 bag 才发现（首帧 78489 字节三连相同暴露了"只发了一帧"）。
- **验证方式**：不跑 8 分钟仿真，而是 `roscore + rosparam load formation_air.yaml +
  rosbag play --clock -r 3 <录制的 bag>` + 仪表盘，按消息时间戳抓帧。抓到 mission t=5.3 s 与
  t=20.1 s 两帧：场景/槽位六边形/`qn trajectory_id 4 (all adopted)`/槽位误差在驻留段收敛到
  `epsilon_p` 以下/最近接近在编队切换时收到 1.7 m，均与离线图一致。
- **效果**：实时视图与离线渲染在语义上对齐；167 项测试通过；离线两张图重跑无回归。
- **证据**：`docs/QN_INTEGRATION.md` 的 "Live views" 一节；
  `roslaunch qn_aav_simulator formation_air.launch run_mission:=true dashboard:=true`。
- **未完成与下一步**：仪表盘尚未在**真实实时任务**里跑过（只做了 bag 复放验证）——因为它与
  模型时钟门槛争 CPU，需要一次带 `dashboard:=true` 的真实运行来确认门槛不受影响；
  M2 第 3 步（原盒子场景定位）与第 4 步（局部扫描中断）仍未做。

---

## 2026-09-18 · 全链路可视化与动画回放（提交 `9fcb40f`）

- **计划**：用户要求"可视化仿真整个实验，从任务分配到编队控制"。已有的
  `plot_formation_experiment.py` 是按主题分三张图（轨迹 / 槽位 / 时间），没有一条把
  **分配 → 派发 → 飞行 → 控制 → 安全 → 时间**放在同一个时间轴上的视图，也没有运动回放。
- **实际**：新增两个只读渲染器 + 一个命令封装，全部只读 bag + `metrics.json` + `config.json`：
  - `plot_mission_overview.py`：一张 7 面板总览（任务甘特 / 分配路由 / XY 轨迹与槽位 /
    槽位误差 / 轨迹跟踪误差 / 高度与 AIR 域 / 净距 / 时间漂移）。
  - `animate_mission_replay.py`：GIF 回放，场景 + 槽位布局 + 障碍，右侧联动槽位误差与高度曲线。
  - `scripts/docker_render_figures.sh <实验目录> [fps]`：一条命令出两张图。
  - 两者加入 `CMakeLists.txt` 的 `catkin_install_python`，镜像已重建。
  - 写第一版时踩到的坑（都是"看图才发现"的语义错误，值得记下来）：
    1. `set_aspect("equal", adjustable="datalim")` 把 x 轴撑到 −44…−5，轨迹被压成细线；
       改用 `adjustable="box"` 并加高该行。
    2. **无障碍场景仍然画了盒子净距曲线**——`metrics["obstacle_center"]` 在 `obstacle off`
       时照样存在，必须用 `obstacle_scenario` 门控。否则图上会出现一条并不存在的负净距，
       正是计划里"盒子不存在要记为不适用，不是距离为零"的图形版本。
    3. 槽位误差在运输段天然很大（派发瞬间目标中心跳到新目标，误差瞬间到 ~4 m）。第一版把它
       当误差信号直接画会误导；改为标出驻留窗口并注明 `epsilon_p` 只在驻留段把关，另外单独画
       "对 qn 实际采用参考的跟踪误差"。
    4. `action_task_outcome` 是枚举整数，字符串判定必须取每任务 diagnostics 的 `verdict`。
    5. **PIL 7.0.0 的 GIF `optimize=True` 不做帧间差分**：实测 60 帧对照中 plain 与 optimize
       字节数完全相同，`disposal=2` 反而大 15 倍。第一版 71 MB。改成"共享调色板 + 关抖动 +
       快进 3× + scale 0.52"后 8 MB。帧数/分辨率/调色板是唯一有效的三个杠杆。
- **效果**：
  - `experiments/20260918-m2-baseline-shm` 出图成功，图面确认的事实：T1 晚 9.1 s、T2 晚 10.5 s、
    T3 晚 4.3 s；三次派发 `release_lag` 0.000/0.069/0.065 s；`plan_revision=3`、
    `updated_plan_used=true`、`dispatch_changed=false`（串行单资源下是预期结果）；
    三任务 `PASS/PASS/VALID`；最大模型-ROS 漂移 0.00064 s、跨机 0.00069 s。
  - 面板语义与 plan 的判定口径对齐：实际净距用实际状态判、参考余量单独评估；盒子不存在记
    `NOT_APPLICABLE`；`epsilon_p` 只在驻留段把关。
  - 167 项单元测试仍全部通过。
- **证据**：`experiments/20260918-m2-baseline-shm/{mission_overview.png,mission_replay.gif}`；
  `docs/figures/mission_overview.png`；命令 `./scripts/docker_render_figures.sh experiments/<run>`；
  `docs/QN_INTEGRATION.md` 的 "Whole-mission figure and animated replay" 一节。
- **未完成与下一步**：M2 第 3 步（原盒子场景定位，盒子保持 `(-23,0,0.5)` / `(1,1,1.2)` 不动）
  与第 4 步（局部扫描中断）仍未做；GIF 未入库（`experiments/*` 被 gitignore），需要时按上面的命令重出。

---

## 2026-09-18 · 定位累计时间偏差的根因：bag 写盘（提交 `9e64648`）

- **计划**：M2 第 2 步的停止条件是"无障碍基线必须通过时间、AIR 域、平面包络、机间净距与共同驻留"。
  基线三任务都过，但**累计模型-ROS 偏差 57 ms > 50 ms 门槛**，卡在这里；计划本身没有给出偏差的归因方法。
- **实际**：
  - 先量化：外层循环中位 10.00 ms、p99 10.07 ms、7308 步 / 73.1 s = 100.0 Hz——**速率没问题**；
    偏差是两次离散跳变（+6.4 s 处 −21.8 ms、+38.2 s 处 −6.1 ms），都发生在任务进行中，
    对应单步间隔 31.8 ms / 16.1 ms 的停顿。固定步长设计不补算，所以每次停顿永久留在偏差里。
  - 排除主机 CPU：32 核、load 1.4、无其他容器在跑。
  - 隔离实验：同一任务、同一参数，`record:=false` 不录 bag → 偏差 **0.0035 s**、速率 0.99999991、门槛通过。
  - 验证修法：照常录制但把实验目录放到 tmpfs（`/dev/shm`）→ 偏差 **0.000636 s**、速率 0.99999996、
    门槛通过，三任务 `PASS/PASS/VALID`。
- **效果**：找到了根因——不是仿真、控制器或宿主 CPU，而是 bag 写 home 盘时的间歇写回停顿。
  录制改到内存盘后第 2 步的停止条件满足，第 3、4 步解锁；代价是暂存目录不持久，运行结束需要把文件
  搬到实验目录（启动脚本已改为自动暂存 + 搬移，不改变实验目录布局）。
- **证据**：`experiments/20260918-m2-baseline-clean`（写盘，57 ms）、
  `experiments/20260918-m2-baseline-clean-shm`（tmpfs，0.64 ms；bag 已随实验目录保留）、
  不录 bag 的隔离运行 `/tmp/norecord/metrics.json`；脚本改动见 `scripts/docker_test_qn_formation_action.sh`。

---

## 2026-09-18 · M2 第 1–2 步：单一地图来源、几何判据分离（提交 `31c1aab`）

- **计划**：按冻结版《M2 定位与修复实施基线》第 1 步修地图与几何语义，第 2 步按新规则重评无障碍基线；
  基础条件不过就停，不进入盒子避障归因。
- **实际**：
  - 场景发布器独占 `/scene/global_cloud`，launch 一处配置展开给渲染器 / Action / readiness / 验证器；
    `obstacle:=off` 只移除盒子。
  - 排查发现上游 CPU 渲染器**在全局地图为空时崩溃**（对空点云做体素化再建 k-d 树），此前所有运行的
    "本地点云零消息"就是它，不是"环境为空"。加构建期补丁：空地图=已初始化且障碍集为空；
    一次有效扫描无返回点时发布带时间戳的空点云，帧改 `world`。
  - 盒子点云改为与实体几何同源，精确覆盖 `[-size/2, +size/2]`（原来 `range(round(size/res))`
    漏掉 +x/+y/+z 面）。
  - 判据分离：实际 `d_actual ≥ d_required`；参考 `d_ref ≥ d_required + e_budget`（预算运行前声明 0.30 m）；
    半径只减一次；无盒子记 `NOT_APPLICABLE`；平面按全程最小高度判定；违规即时锁存。
  - 与计划不一致的地方：计划写"先压参考、不够再提高度"，但实测 1.5 m/s 与 1.0 m/s 的最低高度都是
    0.237 m，证明**不是参考强度问题**，于是直接走了提高度的路径，并加了第三个构建期补丁
    （上游 replan FSM 原本硬编码编队高度 0.5，现读取目标 z），场景巡航高度声明为 0.8 m。
- **效果**：`experiments/20260918-m2-baseline-clean` 三任务全部 `PASS / PASS / VALID`，最低高度
  0.380/0.502/0.426 m，`obstacle_check=NOT_APPLICABLE`，基线 30.20 s；但**累计时间门槛不过**：
  `model/ROS rate` 0.99928 → 72.8 s 累计偏差 57 ms > 50 ms。按停止规则不进入盒子场景，未调盒子或权重。
- **证据**：提交 `31c1aab`；`evidence/m2-baseline-clean`、`evidence/m2-baseline-plane-violation`、
  `evidence/m2-baseline-slow-reference`；167 项单元测试通过。

## 2026-09-18 · RViz 演示 master 修补（提交 `5961317`）

- **计划**：用户报告关掉 RViz 后终端一直刷 `XmlRpcClient::writeRequest: Connection refused`，要求修补。
- **实际**：定位为演示脚本先启动 `rviz.launch`，而该 launch 拥有 rosmaster，且 rviz 节点 `required="true"`：
  关窗即结束 launch、master 随之消失，仍在跑的七机节点刷错。改为脚本自己启动并持有 roscore，RViz 作为
  旁路进程，退出时一起收掉。
- **效果**：实测杀掉 rviz 进程后 master 与仿真继续（39 节点、`/drone_0_qn/odometry` 99.9 Hz），
  日志 0 条连接错误。
- **证据**：提交 `5961317`；容器内实测记录见提交信息。

## 2026-09-18 · 测试命令收敛与历史文档归档（提交 `436600e`、`2ed2765`）

- **计划**：把散在四份文档里的测试命令集中到权威文档；历史审查文档入 `docs/reviews/` 并标注对应提交，
  个人笔记不入库。
- **实际**：`docs/QN_INTEGRATION.md` 增加 Tests 一节（单元 / 冒烟 / 端到端三层 + 分析工具 + 失联注入）；
  审查文档移到 `docs/reviews/source-review-85a5463.md` 并加历史声明横幅。
- **效果**：命令可直接复制执行；历史结论与当前状态不再混淆。
- **证据**：提交 `436600e`、`2ed2765`。

## 2026-09-18 · M1 正确性修复 + M3 路由与离线算例 + M2 首版场景（提交 `8362db8`）

- **计划**：按冻结的《Heformation 框架验证实施基线》修坐标 / 时间 / AIR 域 / 样本对齐 / 执行单元状态。
- **实际**：
  - 坐标：实测表明模型**姿态与角速度是物理量**（前飞加速低头、机体角速度匹配 `R(q)ᵀdR/dt`），
    不自洽的是机体线速度；因此实现为"保留姿态与角速度、发布 `R(q)ᵀv_world`"，而非计划字面写的
    反射四元数方案。
  - 时间：恢复固定外层步长，模型时钟自驱；命令步边界采用 + 有界缓存。
  - AIR 域由 `hg_m/2` 派生并在线锁存；净距改表面净距；采用门槛先进驻留；未被接受的任务不释放资源。
  - 样本台账期望网格独立生成、按状态时间戳匹配。
  - M3：静态执行单元路由 + 每单元独立位置/可用时刻；离线五平台三类算例。
- **效果**：`experiments/20260918-mission-m1` 242 项检查 0 失败，三任务 `PASS/PASS/VALID`；
  用新评估器复评三个旧 bag，全部 `air_domain_ok=false`（旧结论失效被正确判出）。
- **证据**：提交 `8362db8`；`evidence/m1-seven-aav-online`、`evidence/reevaluation-*`。

## 2026-09-18 · 运行可视化与坐标问题暴露（提交 `bd04c1b`）

- **计划**：给出可复查的运行图（轨迹、槽位误差、时间对齐）。
- **实际**：新增 `plot_formation_experiment.py`，每个运行出三张图；把时间对齐图改成"门槛视图 + 去偏置视图"，
  否则亚毫秒信号在门槛尺度上不可见。
- **效果**：图直接暴露 `mission-g-fault` 中 drone_3 曾降到 z=−1.276 m（穿地）而任务仍判 PASS——
  由此引出 AIR 域检查缺失，成为后续 M1 的输入。
- **证据**：提交 `bd04c1b`；`docs/figures/`；`context/15_handoff.md` 的未解决清单。

## 2026-09-18 · P0–P3 主链落地（提交 `d8b0fc2`）

- **计划**：按 `plan.md` 的 P0–P3 实施七机 AIR 任务闭环。
- **实际**：双 Odometry 语义、参考遥测、trajectory_id 归属、模型时间门槛、资源状态机、三个独立判定、
  样本台账、A→B→Return + 幂等 DelayEvent、离线执行单元层；一次回归（窗口样本当单样本用）导致执行崩溃，
  修好后重跑。
- **效果**：三次 ROS 运行（repair on / repair off / 失联注入）各有记录；128 项单元测试通过。
- **证据**：提交 `d8b0fc2`；`experiments/20260918-mission-e|f|g-fault`。

## 计划演进（为什么现在是这个方案）

- 起点是 `plan.md` 的 P0–P3（接口/时间语义修复 + 一项感知避障场景 + 离线多资源）。
- 两轮外部审阅把范围收敛为：先修"已接通链路是否在表达同一个物理过程"，再谈感知避障与资源调度；
  明确不做参数扫描、不换算法、不为通过而缩小障碍。
- M2 从"跑一个障碍场景"进一步收敛为"先修地图输入语义，再用时间线定位失败层"，
  并规定：定位完成 ≠ 避障通过。
- 每次范围变化都记在 `context/14_decisions_and_unknowns.md`，结果记在本文件与 `context/15_handoff.md`。
