# Swarm / qn 双向参考交接阶段记录

## 92f2b1b之后的继续实施（本地工作树）

正常Swarm→同qn跨介质→新Swarm往返在非调试`handover-rootfix`镜像取得正例；AIR取消、水下重复取消、规划器确认丢失均产生所需非成功终态与持续资源锁。独立bag复核复用原时间对齐规则，正常任务区间无缺样、最小采样净距约1.50m、暂停确认后无普通轨迹发布。七机代表回归185检查0失败，当前295项模块测试通过。

**G1仍未通过：出水转换中取消后的固定参考保持不成立。** `rootfix-cancel-exit-r1`采用固定z=−0.080293m参考后，末20s实际高度约−0.795至0.008m、垂向速度约−0.959至0.687m/s，最终ABORTED、terminal_verified=false、资源锁定。没有放宽保持条件或观察期限。后续按授权边界核对原qn方程，再决定有依据的隔离修正。

入水取消还有一项已经修复的生命周期问题：原r1在Result后按瞬时AIR重新分类，但保持参考仍在转换边界，产生后续域违规。现在本地故障保持继续保留原授权处置阶段，Result不改参考、不清锁或历史。r2追加Result后4s模型观察及独立审计均通过，最终实际TRANSITION，未宣称入水任务完成。原r1失败保留。

原生求根防护有独立确定反例：归一化只留下一个大于机器精度的系数，Kojima比值向量为空；补丁退回原代码已有Cauchy根界，不改变正常分支。解析两根与普通六根原生回归通过。**不能由此追溯断言normal-r2没有调用栈的ROS失败必然同源。**

新增静态路由可按同一物理成员的操作/已选单元区分FormationAction与PlatformTaskAction，歧义明确拒绝；成员资源互斥保持。此项只有合同测试，尚不是五平台runner接线完成。

出水失败的离线定位使用记录的实际采用参考重新推进原qn后端，只在开始初始化一次。原记录模型步4886的诊断和Odometry缺失，初次重放拒绝继续；bag仍有该间隔唯一的used_reference_pose，后续输入提取原始参考并将实际状态明确记缺失，不插值实际状态或宣称完整逐步等价。结果以WORKLOG和对应重放输出为准。

同实验镜像重放最终在所有有实际记录的位置残差为0；主机Python重放存在数值环境差异，另存失败。原浮力分支在末20s切换80次、浮力z从−78.4N到0N，是后续检查对象；未据相关性宣称单独原因，未修改qn物理或控制参数。缺失的一个实际比较样本继续计缺失。

新增证据目录统一位于本机`experiments/20260920-g1-handover/`：`rootfix-normal-r1`、`debug-cancel-air-r1`、`debug-cancel-native-r1`、`debug-missing-planner-r1`、`rootfix-cancel-entry-r1/r2`、`rootfix-cancel-exit-r1`、`seven-single-rootfix`。记录是分项证据，不替代五平台同场景、模式相关计划、预承诺通信、UI与科研对照总验收。

本次为main@6011944之后的实现增量，**不是G1资格或G0–G5总验收完成报告**。

## 实现与依据

Swarm在已有FSM与最终提交边界加入可选普通参考暂停、代次确认和旧目标阻断；qn分别记录请求来源、实际采用来源与实际介质。正常返回AIR不清故障锁、不重置qn，且要求新Swarm轨迹高于规划器确认的退休编号。AIR Action使用实际完成与采用证据，水下/转换保留全局介质历史并另记授权域违规。

直接来源与项目工程推导边界见[参考交接依据](../../research/literature/notes/reference-handover-basis.md)。无新增控制律或协调节点，生产资格保护仍保留。

## 已有证据与失败

原始文件位于本机`experiments/20260920-g1-handover/`，遵循仓库既有忽略规则，不随源码上传bag与完整日志。

| 运行 | 结果 | 解释 |
|---|---|---|
| normal-r1 | 失败，首段缺Result | 实际AIR到位，但跨来源过滤丢失预派发模型步，采用计时未成立；已补独立交接边界记录与反例测试 |
| normal-r2 | 失败，原生节点exit=-6 | 首次AIR重规划出现Eigen动态向量Block断言，未进入水下；原因尚未定位 |
| debug-r1 | 探针passed=true | 调试镜像下首段AIR、原生入水/水下/出水片段、返回AIR均SUCCEEDED；水下活动期间新AIR任务REJECTED；末尾故障锁拒绝接管 |

debug-r1两个AIR结果分别记录4.03 s和4.05 s模型驻留；原生片段返回`terminal_verified=true`、实际AIR。最终全历史最低高度约−1.129 m、最大介质标志1.0，AIR范围最低约0.799 m，域违规标志false。最低高度是实测瞬态，不能将−0.6 m目标深度写成全程实际深度或已验证包络。

debug-r1使用`swarm-formation-qn:handover-debug`，镜像ID为`sha256:c8b6711f1bec9624f36bf32a69b4f2f23d6ee832b518a793092df5c834972012`。关键代码哈希与该次记录一致：

```text
platform_action.py                 ce98a962aec716fe4f446d81c0ba18c7795aa3bda935138540ae5829152bc781
qn_aav_node.py                     fa563b5a6f1ca3abb8690b052eb84aa9df2a40020620d6e38b5a1930d7c681a7
formation_action_server.py        1b3090768687f5e30619ac19a724b1404a00f1171b86b902871c3cd6a08b4003
plan_manage_reference_handover.patch e9d2e65ed911c845fd4c61cc7604bfb891fc22a9c09d0d160240b28337c60f88
```

推送前模块验证：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest integration/qn_aav_simulator/tests integration/mrta_python/tests -q`，292 passed；两个修改的Docker探针入口通过`bash -n`，`git diff --check`通过。handover-dev此前编译通过，调试正例没有复现断言，不能据此宣称断言已修复。

暂存检查补充：新`.patch`文件的上游空白上下文行触发Git尾空格提示，保留原始补丁内容与构建时哈希；其余暂存文件的diff检查通过。

## 未完成

继续取得原生断言调用栈并定位；运行AIR/水下取消、规划器确认缺失等负例；独立审计参考提交、状态连续性与全程多机安全。当前单成员资格探针中另两台AAV待命，未证明五实例共同场景、安全资格或有限交付闭环。G2–G5按冻结计划继续，不能把源码存在、模块测试或一次调试正例当作完整交付。
