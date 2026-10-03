# DNV 官方巡检对象与控制任务模板边界

核查日期：2026-10-01。用途：B 风机、C 平台/海底管路控制任务模板的业务对象依据；本记录未实施或运行 B/C 任务。

本次读取的是 DNV 官方公开网页。**未获得或阅读 DNV-RP-F116 完整标准**；以下“支持”仅指公开网页能支持的设施类别、管理目的和访问状态，不是条款级合规结论。

## 1. 风机检查公开说明

官方来源：[Wind turbine inspections](https://www.dnv.com/services/wind-turbine-inspections-3845/)。本地原始页面：[dnv-wind-turbine-inspections.html](../sources/inspection-templates-20261001/dnv-wind-turbine-inspections.html)。

正文说明服务涵盖陆上和海上风机及其组件，在制造、调试和运行阶段可以检查设施状态以及维护/维修表现。对象明确列出转子、机舱、塔架、基础、电气系统，并明确包括海上变电站平台。页面还介绍寿命延长检查中的近距目视检查与针对性无损检测。

对本项目模板的可用结论：B 可以按区域内风机及上述主要结构拆分控制作业对象；基础可作为设施对象单列。将可见结构交给 AIR、将浸水结构交给 WATER，是本项目根据场景几何和平台能力作出的控制任务映射，**不是该网页规定的无人机/无人潜器分工**。转子是公开列名；具体叶片表面、轮毂或内部机械的可达性须由场景和外部业务模块另行声明。

该页明确的平台是海上变电站平台。不能凭这一个来源宣称一般油气作业平台的甲板、吊机、直升机坪、支柱全部具有相同的 DNV 检查合同，也不能宣称无人平台已能替代页面中的专业工程检查或无损检测。

## 2. F116 当前公开目录、范围与取得方式

官方来源：[DNV-RP-F116 Integrity management of submarine pipeline systems](https://www.dnv.com/energy/standards-guidelines/dnv-rp-f116-integrity-management-of-submarine-pipeline-systems/)。本地原始页面：[dnv-rp-f116-public-description.html](../sources/inspection-templates-20261001/dnv-rp-f116-public-description.html)。

当前公开条目给出的正式标题是 **Integrity management of submarine pipeline systems**，条目版本显示 **Edition 2021-12**，类别为 Recommended practice。这里记录的是核查当日官方目录展示值，没有读取完整标准去核对内部修订页或全部适用条件。用户所用“subsea pipeline integrity management”可用于描述该业务方向，引用本条目时应保留当前正式标题。

公开摘要仅确认：该推荐实践提供建立、实施和维护完整性管理体系的指导。它支持把 C 的海底管路作为管路完整性检查业务对象；公开摘要没有列出所有管型、材质、介质、附件或系统边界，不能从这一行摘要确认平台甲板、吊机、直升机坪、立管或每种阀件的条款适用性。

官方页面将全文获取指向 [Rules and Standards Explorer+ 订阅](https://store.veracity.com/rules-and-standards-explorer-plus)，已订阅入口为 [F116 文档](https://standards.dnv.com/explorer/document/14B7904CF4E74E239CAA26F66D6B9969)。本次没有登录、付费或访问订阅全文。

官方[订阅文档列表](https://www.dnv.com/rules-standards/rules-and-standards-explorer-plus/list-of-documents-included-in-the-subscription/)列有 F116，并说明文档预览需 Veracity 登录。本地原始页面：[dnv-explorer-plus-document-list.html](../sources/inspection-templates-20261001/dnv-explorer-plus-document-list.html)。本次公开页面未提供直接 PDF 链接，因此没有把第三方标准副本放入 `papers/maritime/`，也没有以 HTML 冒充标准 PDF。

补充官方来源：[Offshore pipeline integrity management](https://www.dnv.com/training/offshore-pipeline-integrity-management/)，本地原始页面：[dnv-offshore-pipeline-integrity-management-course.html](../sources/inspection-templates-20261001/dnv-offshore-pipeline-integrity-management-course.html)。该课程公开说明以 F116 为基础，面向运行管路的完整性管理，介绍风险评估与计划、检查/监测/测试、完整性评估、减缓/干预/维修这些管理环节。这是官方课程内容说明，仍不等于标准全文。

## 3. B/C 模板中可以采用的引用口径

| 模板对象或内容 | 官方公开材料能支持到的程度 | 本项目还需单独声明的部分 |
|---|---|---|
| B：风机转子、机舱、塔架、基础、电气系统 | 风机服务页明确列出这些组件 | 哪些外表面可达、所需载荷、AIR/WATER 映射及控制作业段 |
| B：海上变电站平台 | 风机服务页明确列出该设施类别 | 具体场景结构、作业区域与运动约束 |
| C：一般海上作业平台的甲板、支柱、吊机、直升机坪 | 上述两个指定来源不能形成该对象清单的条款依据 | 来源于原项目业务定义与现有场景的几何任务对象，不能标作 F116 强制要求 |
| C：海底管路沿线检查 | F116 目录标题和官方课程可支持管路完整性管理业务范围 | 沿管段控制作业、入口/出口、路径可行性和收件要求 |
| C：管接口与阀件局部检查 | 本次未读全文，无法确认各附件的详细条款适用性 | 可以作为本项目选定的几何任务对象；须明确并非已核实的 F116 对象条款清单 |

两类官方公开材料都**没有提供**本项目可以直接照搬的无人平台轨迹参数规范：固定绕塔/绕基础半径、螺旋层距、管侧偏移、贴近距离、飞行或航行速度、驻留时间、航向/俯仰序列、跟踪误差界、传感器覆盖率门槛、机间净距与跨介质控制条件均不能据此定为 DNV 要求。模板可以提出可配置的控制作业结构，但数值须有现有控制器/安全规则、明确场景或后续专项实验的独立依据。

任务设计仍应区分：控制到位/完成作业段、外部载荷业务结果生成、结果实际收到、任务结束与安全通过。页面中出现“inspection”不能把这些事件合并为“已完成真实风机/管路缺陷检测”。

## 4. 归档可核查性

四份 HTML 均从 `www.dnv.com` 公开地址直接取得，HTTP 200、`text/html`，下载未使用登录或付款。每份页面旁的 `.source.json` 保存原始/最终 URL、UTC 获取时间、字节数、SHA-256、页面上的 PDF 链接检查结果和访问边界。F116 的获取状态是 **official_public_description_only / full_standard_not_read**。

本记录和公开页面归档不改变运行源码、不增设管理器或模拟器、不构成 B/C 新业务实跑通过证据。
