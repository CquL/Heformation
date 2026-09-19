# 五平台实施直接依据与阅读边界

本轮依照 prompt/wenxianSKILL.md；不把下载等同阅读全文。
来源 P 编号与原 key 的对应关系见 manifest.yaml.final_plan_id。

| 来源 | 本轮实际阅读范围 | 落实到项目 | 不能继承 |
|---|---|---|---|
| P03 D-ITAGS | §III–V 模型、转场估计与定点修复正文 | executors 的资格/先后/未执行修复 | 不引入其 Gurobi 在线求解链 |
| P06 Dai MRTA | §III 问题模型、合作成员全时到齐 | 集结需等最慢成员，等待计入代价 | 二维静态能力不等于跨介质能力 |
| P07 DHRTA | 实际 PDF 开篇/摘要；尚未精读方法 | 保留海洋任务分配候选对照 | 不声称复现分布式调度 |
| P13 Guo–Zavlanos | §III 缓存/转移守恒、§V 会合执行、§VI 固定接收端 | 预承诺首个会合；生产者→USV→母船接收守恒 | 基本方案即时上传/可静止等待不能套给 REMUS；硬截止非原结论 |
| P14 ACHORD | §III-B 消息类别/确认、§IV 状态和剩余字节 | 通知与业务结果区分，母船只消费已接收信息 | 不搬 ROS2/DDS/JPL 网络栈与实测射频参数 |
| P17 Hybrid smooth transition | §3 分域控制/CL-RRT 传播与转换区域 | WATER/转换资格、实际介质与参考来源分开 | HyDrone 专用入口、垂直转换及控制参数不直接用于 qn |
| P18 TJ-FlyingFish | 推力分配、原型及水下操纵限制正文 | 先检查执行器与原生控制能力 | 不把可倾转推进器带来的全向能力赋予 qn |
| P11 CAT-ORA | §III A1–A6 假设 | 编队重排相关边界参考 | 直线路径/无障碍凸包假设不适用于任意场景 |
| P32 平均驻留时间 | 定理2、非线性假设3/定理4正文 | 条件性论证明确要求的稳定性/比较界 | 多等几秒不自动证明 qn 转换稳定 |
| T01 PVS | 本地 otter/remus100/mainLoop 模型及原生控制源码 | G2 原生动力学薄接入、完整 NED 转换 | 原生航向/深度控制并非路径规划或避障 |

已有 Calvo、CoCoPlan、FaSTrack 本地正文重点段落另行复核了应急动作、通信事件、
误差界依赖条件；不更新其历史 full-text 标签到“本轮全篇重读”。
GRSTAPS 本轮访问期刊正文入口，原已记录章节范围保留；PDF仍需单独记录访问结果。
Nezha-IV、WuKong、转换综述及 SeaClear 本轮核对官方记录/摘要；
JOE ADSC、Surfing、JOE 海上异构协作未取得方法正文，不能据摘要填补控制律。

归档原始访问记录在 experiments/20260919-five-platform-g0/literature-access.json。
标题校验阈值是下载身份筛查，不是观测/动力学验收标准。
