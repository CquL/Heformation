# 当前水下协作的实时可视化接入

## 范围

用户要求先把已有进展接到实时入口。复用港口场景、原MissionRunner、区域候选生成与有限传输；没有增加协调节点。此入口仅验证水下阶段，三台AAV待命。完整AIR/WATER自主选择、跨介质、复查及规定返回仍未完成。

## 可重跑入口

```bash
cd /home/lhj/Swarm-Formation
VISUALIZE=true bash scripts/docker_probe_five_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-water-live" cooperative
```

当前挂载Python源码及已有`swarm-formation-qn:cooperation`镜像。终端先打印所选成员、角色、活动时段、终点和终端等待；保存具体计划后输入yes才派发。不输入确认不执行，不循环接新批次。UI关闭不停止动力学，启动终端Ctrl+C才结束整链。

`run_water_cooperation.py`直接使用已有请求／规划器／runner。初始模型为已声明PVS trim快照，并核对真实初始位置与既有状态新鲜度；不是任意状态重规划入口。每次候选调用仍共享10s预算，程序外围序列化／返回开销另记在planning_wall_s。

## 母船作用与显示边界

母船是固定的业务接收端，没有自主航行动力学或位置优化。港口船体是实体障碍，接收位置是船体外的声明附件点。有限链路受实际位置、介质、遮挡和共享容量影响。

任务完成和占用来自runner的现有task_state；母船标签只按/mother/received_products事件计数。/scene/delivery_progress只给被动显示提供独立仿真账本的中继与接收字节数，runner不消费该真值。不将消息接收百分比自行改判请求成功。

面板明确：观测摘要产生、USV收到、母船收到、原生Result、资源释放是不同事实。32KiB、水下8m/2KiB/s、RF30m/32KiB/s仍是实验假设，不是设备参数。控制和状态并非全部经过有限链路，未宣称实现完整母船知识隔离。

## 已验证结果与保留失败

- headless-r1：入口未先发现Action端点，读位置时server_nodes为空。零派发，失败保存；按既有端点就绪流程修正。
- headless-r2：港口实际运行PASS_WATER_GEOMETRIC_PROXY；UUV／USV各自SUCCEEDED，母船接收32KiB，业务结果登记一次，资源释放。
- gui-r1：RViz＋中文面板的同链实跑通过上述水下业务条件；具体计划确认前零派发有快照。全程时间审计最大偏差0.029871s、缺样0、采样全平台代理净距1.5m。原跨介质审计整体仍FAIL，因为这次没有执行AIR轨迹／跨介质；不改判。
- gui-r2：启动模型构造后新鲜度单次检查失败，未派发；改为沿原0.25s新鲜度要求有界等待新样本。
- 后续记录生命周期调整：确认前不录包，确认后原生rosbag LZ4记录，结果提交后结束记录；显示待命不会无限积累重复点云。模型和执行器不因结束记录停止。

证据：`experiments/20260921-cooperative-live/`，包括具体计划、metrics、bag、截图、失败栈。整个三类平台任务和三机转场安全修正仍未完成。

- gui-r3：记录生命周期修改后再次带显示实跑通过；确认前无录包，执行结束bag关闭、两项Result及32KiB接收完成、资源释放。整链结束时RViz出现退出阶段segmentation fault，保留记录；不影响此前已保存的原生终态，显示退出问题仍需后续核查。operator-preview在独立桌面终端等待用户确认。
