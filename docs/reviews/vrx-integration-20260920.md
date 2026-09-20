# VRX海面环境试接

用户授权拉取并试接现成海面环境。本轮使用官方VRX `gazebo_classic`分支，固定提交
`c9b9388308f8976c724da4af6685f69c9c378983`。源码位于`upstream/VRX`，保留原许可证；
本地副本约513MB，未重复提交进本项目，通过构建脚本下载/核对。

## 当前入口

```bash
cd /home/lhj/Swarm-Formation
bash scripts/docker_build_vrx.sh
bash scripts/docker_run_vrx.sh native
```

这是官方Sydney场景和原生WAM-V模型，使用Gazebo自己的动力学与仿真时间。
它不调用qn/PVS执行器。原生场景有岸线、建筑、植被、码头、海面和竞赛浮标。
首次启动需要加载资源；终端Ctrl+C退出，记录保存在自动生成的实验目录。

现有平台的单向状态视图：

```bash
bash scripts/docker_run_vrx.sh five-view
```

窗口打开后，终端输入`yes`才运行既有固定资格动作。ROS保持`use_sim_time=false`，
原qn/PVS照常产生状态；Gazebo仅放置没有碰撞、重力、推进器、传感器的静态显示代理。
代理用`heformation_`前缀，原生平台仍是唯一物理状态源。

明确声明ENU平移`(-500,156,0)m`，将本地状态放在官方场地附近；没有坐标旋转或高度补偿。
船的显示代理使用缩小的WAM-V网格，实际执行仍是Otter，不能将外观称为Otter物理模型。
水下代理暂为简化潜航器，透过不透明水面看不见时不能推断未执行。

**当前不是VRX任务环境完整接入**：VRX地形/波浪/风力未进入qn/PVS的运动与安全约束，
不能据画面声称在该地形中已经避障或承受海况。五平台动作仍使用已有小场景约束；
完整协同、有限交付、复查及共同安全地图尚未完成。

## GPU核对

本机为RTX5070 Laptop，驱动595.84。初次虽然指定`--gpus all`，Gazebo仍落到llvmpipe。
定位为容器NVIDIA GLX依赖`libnvidia-gpucomp.so.595.84`缺失；补同主机版本库及GLX选择后，
`glxinfo`与Gazebo `ogre.log`都显示NVIDIA RTX5070，`nvidia-smi`可看到gzclient。
脚本自动只读挂载主机存在的该库，不将主机驱动二进制提交到项目。
当前入口要求NVIDIA Container Toolkit；未声称验证其他显卡/驱动。
渲染在GPU，当前qn/PVS动力学与规划仍在CPU。

## 证据与已知问题

- `experiments/20260920-vrx/build.log`：独立`heformation-vrx:noetic`镜像构建完成。
- `native-r1/native-check.json`：原生WAM-V受短时推力后位移约0.214m，仿真时间推进约2.947s。
- `native-r1/ogre-gpu.log`：实际GPU渲染器；原生场景截图`scene.png`。
- `five-r1`：首次两个launch自动启动master竞争，失败保留；随后显式先启动唯一master。
- `five-r2/vrx-state-view.json`：五成员持续更新与Gazebo位置回读误差、接收年龄；
  `proxy-audit.json`核对五个代理均static且无collision，ROS未切成Gazebo时钟。
- **five-r2失败边界**：后续用户反馈GUI卡住，定位启动路径指向源码中的不存在world，实际退回空场景；因此该轮代理回读只证明状态同步，不证明海面接入。
- `five-r3`使用devel下真实生成world，并核对海面/地形/五代理名称；显示端经历退出与加载等待，OOM计数为0。子进程gdb观测到NVIDIA/OpenGL相机渲染，随后真实场景显示约28FPS，截图`rendered.png`。尚未证明全部启动等待的唯一原因，不将GPU已启用等同每次GUI稳定。
- 上游Classic有SDF重复插件名与渲染服务兼容性告警，原日志保留；场景显示不构成全部VRX模型/插件无错误的结论。

下一步要把场景选型与统一几何/时钟的真实接线完成，不能把单向状态显示当成五平台业务验收。
