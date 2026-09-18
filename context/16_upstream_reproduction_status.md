# 上游项目独立复现台账

**原始复现台账更新时间：2026-09-15；阶段范围说明更新：2026-09-17。**

本台账只记录各上游项目在自己的目录、依赖和入口下复现原始示例的历史工作。
不把它们拼进 Heformation 的在线链，不把 qn 控制器或 qn 动力学注入这些上游，
也不把“源码已拉取”写成“算法已复现”。当前另有已授权的Calvo受限Python移植与七机AIR集成，见[plan.md](../plan.md)；本台账不构成其前置门槛。

## 1. 当前边界

本台账记录的独立运动基线是：

```text
Swarm-Formation official source + qn AIR adapter
```

它位于 `upstream/Swarm-Formation/` 与 `integration/qn_aav_simulator/`，本台账中的其余项目
都是隔离的候选参考。后续若选择一个项目集成，必须另开适配工作并在同一场景做对照，
不能因为本表有通过项就自动改变在线架构。

状态含义：

| 状态 | 含义 |
|---|---|
| `PASS` | 上游入口或原生测试已在当前机器实际运行完成 |
| `PARTIAL` | 已启动或完成部分原生步骤，但完整示例还未结束 |
| `BLOCKED` | 已确认原始依赖、许可或运行环境阻塞，未伪造替代结果 |
| `INSPECTED` | 已拉取、固定版本并核对入口，尚未运行原始算法 |

## 2. 结果总表

| 项目 | 固定版本 | 原始复现状态 | 实际证据 / 阻塞 |
|---|---|---|---|
| Swarm-Formation | `967a4bdfae94...` | `PASS` | Noetic Docker 编译；官方七机入口和 qn 基线另有记录。它是当前独立运动基线。 |
| Fossen | `695c673ea6ae...` | `PASS` | `python -m pip install -e . --no-deps` 后，仓库原生 `pytest -q`：`14 passed`。 |
| DMPC-Swarm | `9c380b9b0b73...` | `PARTIAL` | 原始 Python 入口完成依赖加载并进入控制循环；短时运行生成了 8 机结果文件，但 `num_targets_reached=[0]`、`num_optimizer_runs=[0]`，因此不是任务完成 PASS。Python 3.12 还需要运行时 `collections.Mapping` 兼容处理，QP 需要 `quadprog`。未改源码。 |
| H-LTL-GCS | `1fc9e213a08a...` | `INSPECTED` | 原始 Python 全量 `py_compile` 通过；正式示例需要 Drake、MONA、MOSEK 许可证，尚未运行求解示例。 |
| Primitive-Planner | `cab5aa2f8294...` | `PASS` | 官方基元库生成成功；五机 launch 生成成功；独立 Noetic 临时工作区 18 个包编译通过。尚未做 GUI/飞行全过程验收。 |
| Calvo-MRTA | `c69509e8d237...` | `BLOCKED` | 原始代码为 MATLAB；本机无 `matlab`/`octave`，Gurobi 也未安装。不能用 Python 重写冒充原版。 |
| Calvo-Execution | `e20c41f7b812...` | `BLOCKED` | 独立 Noetic catkin 配置在 `behaviortree_cpp_v3` 处停止；完整入口还需要 UAL/MAVROS 和 MATLAB ROS connector。 |
| D-ITAGS | `e779ca8b6c98...` | `BLOCKED` | 普通 C++ 依赖已补齐；CMake 现在在 Gurobi library 处停止。 |
| GRSTAPS | `20efd0656da6...` | `BLOCKED` | OMPL 等系统依赖已补齐；官方 `.gitmodules` 中多个 `amessing/*` 地址返回 404，子模块无法完整初始化。 |
| CAT-ORA | `4bc41f593eeb...` | `BLOCKED` | 独立 Noetic catkin 配置在 `mrs_lib` 处停止；尚未启动其原生 ROS 节点。 |
| AMSwarmX | `522d340fc422...` | `BLOCKED` | thirdparty 组件独立编译通过；主包因缺失 `dynamicEDT3D` 配置包停止。 |
| RMADER | `c8d01dbeff22...` | `BLOCKED` | 原始工程依赖 Gurobi license、CGAL、GLPK、NLOPT；当前无 Gurobi license，未伪造构建结果。 |
| DANCERS | `bcc3390de94e...` | `PARTIAL` | Protobuf/em 已补齐；ROS 2 Jazzy 中 `protobuf_msgs`、`dancers_msgs`、`connector_core`、`coordinator`、`empty_connector` 已构建，剩余 `px4_msgs`、Gazebo `gz-sim8`、ns-3 阻塞。 |
| Stonefish | `b21eb8e194c5...` | `PARTIAL` | GLM 已补齐；库和全部 Tests 目标编译通过，但官方 `ConsoleTest` 场景报告接触定义错误，未记为运行 PASS。 |
| EGO-Planner-v2 | `9d85475ea7b9...` | `PASS` | 官方 `swarm-playground/main_ws` 在独立 Noetic 临时工作区 23 个包编译通过；尚未做 GUI/飞行启动验收。 |

版本短哈希仅用于台账可读性；每个副本的完整哈希由其内部 `.git` 保存。

## 3. 已实际执行的命令

### Fossen

```bash
cd /home/lhj/Swarm-Formation/upstream/Fossen
python -m pip install -e . --no-deps
python -m pytest -q
# 结果：14 passed
```

### DMPC-Swarm

原 README 入口：

```bash
cd /home/lhj/Swarm-Formation/upstream/DMPC-Swarm/cps_testbed_python
python dmpc_simulation_caller.py
```

当前 Python 3.12 机器上，使用已安装的依赖和一次性兼容注入启动原入口：

```bash
python -c 'import collections, collections.abc; collections.Mapping=collections.abc.Mapping; import runpy; runpy.run_path("dmpc_simulation_caller.py", run_name="__main__")' --gui false --plot false --obstacles false
```

这次实际进入原始 8 机仿真循环。默认配置为 `duration_sec: 200`，短时运行结果保存在
`data/results/dmpc/simulation_result-8_drones_simnr_1.pkl`，其中目标数为 0、优化器运行数为 0，
所以当前只能记为部分启动证据，不记为算法完成。下一步要定位它的原始提前停止条件，再复制配置为
独立实验配置并只缩短仿真时长，不改算法。

### H-LTL-GCS

```bash
cd /home/lhj/Swarm-Formation/upstream/H-LTL-GCS
python -m py_compile $(find . -name '*.py' -not -path './.git/*')
```

结果为语法检查通过；它不等于 Drake/MOSEK 规划器求解通过。

### D-ITAGS / GRSTAPS

```bash
cd /home/lhj/Swarm-Formation/upstream/D-ITAGS
cmake -S . -B /tmp/heformation-ditags-build-1 -DCMAKE_BUILD_TYPE=Release

cd /home/lhj/Swarm-Formation/upstream/GRSTAPS
cmake -S . -B /tmp/heformation-grstaps-build-1 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
```

两者现在都已执行 CMake 配置：D-ITAGS 在 Gurobi 处停止；GRSTAPS 在失效的上游子模块之后仍不能进入完整构建。

### Stonefish

```bash
cd /home/lhj/Swarm-Formation/upstream/Stonefish
cmake -S . -B /tmp/heformation-stonefish-config-1 -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/heformation-stonefish-config-1 -j2
```

配置和编译均已通过；库及全部测试目标已生成。运行 `Tests/ConsoleTest` 时，Stonefish 报告
`EEPipeContact` 接触定义缺失，但仍初始化并推进了物理仿真，因此当前记为部分通过。

### ROS 项目补充

本轮还用临时容器分别执行了原始 catkin/colcon 配置：

- `Calvo-Execution`：缺少 `behaviortree_cpp_v3`。
- `CAT-ORA`：缺少 `mrs_lib`。
- `AMSwarmX`：thirdparty（DecompUtil、消息、RViz 插件、JPS3D）通过；主包缺少 `dynamicEDT3D`。
- `DANCERS`：Protobuf/em 已补齐并有 5 个基础包通过；剩余 `px4_msgs`、`gz-sim8` 和 ns-3 未补齐。
- `EGO-Planner-v2`：官方 `main_ws` 的 23 个包已经通过 Noetic 编译。

## 4. 复现顺序

1. 先完成 Fossen、DMPC、H-LTL 的独立原始示例，并保留 stdout/结果文件。
2. 用 Noetic Docker 分别复现 Primitive-Planner、CAT-ORA、AMSwarmX、Calvo-Execution、EGO 的原始入口；每个项目单独工作区。
3. 补齐 D-ITAGS/GRSTAPS 的第三方依赖后，只运行各自最小测试或单个实验，不跑全量论文 sweep。
4. 有 MATLAB/Gurobi/MOSEK 或硬件/GUI 条件后再复现对应项目；许可文件不进入仓库。
5. 以上顺序仅用于独立原始复现。当前Calvo受限Python移植已单独获授权，可修改自己的`integration/`并进行集成，无需等全部候选原例完成；原始复现与移植结果分开记录。

## 5. 当前未完成事项

- DMPC 的短时完整原始仿真及结果文件尚未形成。
- ROS Noetic 项目的容器化原始构建/启动证据尚未逐项目完成。
- MATLAB、Gurobi、MOSEK 依赖项目只能等待对应环境或许可证，不能用替代实现填充。
- Stonefish 的无图形测试和 EGO-Planner-v2 的原始工作区启动尚未完成。

这些是独立复现工作项，不是 qn 动力学接入或 Heformation 在线链路的集成项。
