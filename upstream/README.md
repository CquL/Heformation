# 上游源码目录

每个外部参考项目单独放在本目录下，保留其原始目录结构、版本和许可证。

当前已拉取：

| 目录 | 项目 | 来源 |
|---|---|---|
| `Swarm-Formation/` | Swarm-Formation | https://github.com/ZJU-FAST-Lab/Swarm-Formation |
| `Primitive-Planner/` | Primitive-Swarm / Primitive-Planner | https://github.com/ZJU-FAST-Lab/Primitive-Planner |
| `Calvo-MRTA/` | Heterogeneous MRTA planner | https://github.com/multirobot-use/mrta_heuristic_planner |
| `Calvo-Execution/` | MRTA execution architecture | https://github.com/multirobot-use/mrta_execution_architecture |
| `D-ITAGS/` | D-ITAGS / GRSTAPS family | https://github.com/amessing/ditags |
| `GRSTAPS/` | GRSTAPS | https://github.com/amessing/grstaps |
| `H-LTL-GCS/` | H-LTL task and motion planning | https://github.com/intelligent-control-lab/Task_Motion_Planning_with_HLTL_and_GCS |
| `CAT-ORA/` | CAT-ORA formation reshaping | https://github.com/ctu-mrs/mrs_formation_reshaping |
| `AMSwarmX/` | AMSwarmX | https://github.com/ZJU-FAST-Lab/AMSwarmX |
| `RMADER/` | RMADER | https://github.com/mit-acl/rmader |
| `DMPC-Swarm/` | DMPC-Swarm | https://github.com/IMRCLab/DMPC-Swarm |
| `DANCERS/` | DANCERS co-simulator | https://github.com/Chroma-CITI/DANCERS |
| `Stonefish/` | Stonefish marine simulator | https://github.com/patrykcieslak/Stonefish |
| `Fossen/` | Python Vehicle Simulator | https://github.com/cybergalactic/PythonVehicleSimulator |
| `EGO-Planner-v2/` | Swarm Playground / EGO-related source | https://github.com/ZJU-FAST-Lab/EGO-Planner-v2 |

每个目录都是独立上游副本，复现时使用其自己的依赖、入口和配置。当前 `Swarm-Formation`
基于提交 `967a4bdfae949e994691f8ffc87dbb0147cb7`。这些副本暂不直接混入当前在线运行链；
`integration/` 下的 qn 适配仍只属于 Heformation 自己的 Swarm+qn 基线。

独立复现结果见 [`context/16_upstream_reproduction_status.md`](../context/16_upstream_reproduction_status.md)。
