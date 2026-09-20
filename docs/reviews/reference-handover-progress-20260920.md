# Swarm / qn 双向参考交接阶段记录

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
