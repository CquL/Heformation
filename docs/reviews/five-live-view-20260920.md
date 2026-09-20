# 五平台公共场景与实时显示

**最新结论：显示接线已实现，五平台协同任务未贯通。** 港口harbor-r2所需7项Action成功，但独立全段时间审计FAIL：模型/ROS最大偏差0.0795s、跨平台0.0796s，原规则为0.05s；未通过整场有效性验收。当前固定探针AAV采用空中稳定后垂直入水/出水，中间水下约0.7m，不是最终水下监测路线或一般斜向跨水面资格。后续优先G3/G4和时间一致性，停止继续扩充外观。

本轮把已有五平台资格实验接入实时RViz；不将固定开发动作冒充完整监测请求。最新运动结果见WORKLOG顶部，历史失败保留。

## 启动

```bash
cd /home/lhj/Swarm-Formation
VISUALIZE=true bash scripts/docker_probe_five_qualification.sh \
  "experiments/$(date -u +%Y%m%dT%H%M%SZ)-five-live"
```

从桌面终端运行。窗口出现后先待命，终端打印实验范围；输入`yes`才发送本批固定动作，其他输入不执行。中文字体使用本机`fonts-noto-cjk`，只挂载到本次容器，不修改原镜像。RViz为实际状态视图，侧边中文dashboard读取原生Action状态和Result。关闭显示窗口不停止执行；终端Ctrl+C结束整链并将记录归档到指定目录。

无`VISUALIZE=true`时保持原无窗口自动资格测试入口。它不是请求编辑/完整计划预览入口，不宣称已实现五平台任务UI。

可视化默认场景现为`config/five_scene_harbor.yaml`，增加岸壁、栈桥、礁石及母船实体包络；原`five_scene.yaml`保留为小场景基线，可用`FIVE_SCENE_FILE`显式选择。画面默认隐藏体素包络，可勾选“安全几何（体素）”查看实际地图。Views保留“母船近景”和“跨介质作业区”。

## 画面表示什么

- 三AAV使用原`odom_visualization/robot`四旋翼网格，位置和姿态来自原状态话题。
- Otter使用双体船示意，REMUS使用潜航器示意；尺寸仅用于显示，不修改动力学或碰撞代理。
- 码头、岩石来自公共SOLID几何，同时进入原实际点云；禁入区与海底沿用解析安全检查，任务标记不进入障碍点云。
- 水面、海底为有限显示面片，其边界不是新增的航行限制。
- 母船已用本地Stonefish `aquadelmo.obj`与原纹理替换占位盒；岩石复用其`icosphere.obj`。只做离线格式转换，不启动Stonefish仿真器。原99,130个船体顶点经坐标翻转均在声明母船包络内；栈桥空隙、岩石外观采用内部细节，安全检查仍用保守实体盒。未声明网格级碰撞能力。
- 预设空中观测点、水下样点、母船位置用于解释目标场景；当前位置资格实验没有生成业务观测/有限交付/复查。
- 场景只保留短中文名称，数据缺失/过期时明确提示。长诊断与GoalID保留原日志。动作“完成”只来自对应Result，平台实际位置不推导业务完成。
- 观察者可能错过非锁存Goal，必须从Result中的任务身份补齐；不同Goal分别保存，迟到结果不覆盖较新的动作。仅收到状态而无名称时显示名称待同步，不显示待命。

## 实测与边界

`experiments/20260920-five-live/normal-r1/`：确认前六个Action端点状态列表为空，五平台状态及场景已显示。确认后所需7项动作成功，码头/岩石等可见；岩石、禁入区入口负例正确拒绝。此运行尚为英文初版。

`chinese-r2/`：15条短中文标签、三份原四旋翼mesh消息、中文dashboard均已实机核对。dashboard初次因旧matplotlib缺少`addfont`失败，改为直接FontProperties后恢复，原失败日志保留。运动中的跨介质Action因时间对齐未满足而超时，资源保持锁定；实际回到AIR并不替代保持验收。原生片段首末诊断的模型/ROS相对时间偏差约0.143s，高于既有0.05s规则。未放宽阈值或延长超时。

显示更新改为复用已有Marker，仅删除过期对象，避免每帧DELETEALL重建文字与材质。该修正减少已知不必要工作，但没有据此宣称已证明r2时间偏差的唯一原因。最终中文版本继续以`chinese-r3/`验证。

`chinese-r3/`最终中文实跑通过：所需7项动作均收到SUCCEEDED Result，原生片段相对首个诊断的模型/ROS最大时间偏差约0.0329s。关闭RViz后实际积分从17027步推进到17047步，随后仅重开RViz，未重启任何动力学实例；中文面板五行均显示对应原生动作完成。该运行完成前，用户原来的三机场景已退出，本次结果不构成与r2相同负载下的性能因果对照。

消息显示受控检查：漏Goal但收到Result、旧Result晚于新Goal、仅收到活动status，均得到正确区分；证据`chinese-r2/display-message-check.json`。该检查只调用显示回调，没有向物理端点注入伪造Result。

字体依据：RViz [MovableText源码](https://docs.ros.org/en/noetic/api/rviz/html/c++/movable__text_8cpp_source.html)默认增加0..999码点。本次通过原OGRE字体资源补入短中文标签所需字形，未另建文字渲染节点。

G3请求模式展开/复合派发、平台间在线约束、G4有限交付/复查及G5完整业务验收仍未完成。本入口用于让已有物理能力和障碍检查可见，不是对这些未完成部分的替代。

## 复杂场景来源与港口边界

[Swarm原仓库](https://github.com/ZJU-FAST-Lab/Swarm-Formation)的`src/planner/plan_manage/launch/normal_hexagon.launch`配置60个柱状障碍及20个环；本地还包含mockamap的二维/三维迷宫。该项目的复杂性主要来自障碍密度及编队绕行，不包含母船或港口资产。本次近岸布局是明确声明的工程实验，不称为复制了原森林实验，也不宣称五实例固定动作已经证明密集障碍自主避障。

母船、岩石资源来自本地[`Stonefish`](https://github.com/patrykcieslak/Stonefish)副本，船模贴图用法对照`Tests/CameraTest/CameraTestManager.cpp`，仓库许可为GPLv3或后续版本。`prepare_five_scene_assets.py`输出转换后的COLLADA、原纹理、COPYING和源/产物哈希到每次实验的`visual-assets/`；不重复下载，不将生成的大文件加入源码仓库。

港口r1确实出现原生片段时间对齐失败（约0.0907s相对偏差），资源锁定，原结果保留。原静态点云包含244,248个采样点，每帧重复Python打包属于冗余；现改为启动时一次打包，保持原点集、10Hz心跳和更新后的消息时间戳。Marker改用稳定ID，不发布单点折线，删除消息带明确frame，避免启动缺失标识残留。未改变安全/保持阈值，继续用harbor-r2实跑验证。
