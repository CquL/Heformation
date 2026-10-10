# 技术报告插图

| 图号 | 文件／来源 | 含义 |
|---|---|---|
| 1 | `fig01_task_loop`；`architecture/`原生矢量源稿 | 统一模型、联合方案、执行反馈与任务闭合 |
| 2 | `fig02_work_geometry` | 已接纳B/C作业几何的投影；不是实际运动轨迹 |
| 3 | [既有实际RViz全景](../../../images/three-business-scene-overview.png) | 共用缩比场景，原图不改 |
| 4 | `fig04_actual_timeline` | A/B/C各次Goal派发至核实终态窗口、报告产生／实收及返回授权 |
| 5 | `fig05_support_feedback` | B同请求预计服务接续与实际事件放行的差异 |

自行绘制的四张图各提供300dpi PNG、矢量PDF和SVG，Word使用PNG。数据／几何图2、4、5的颜色、简洁坐标轴、无框图例、独立图例区域和矢量输出参考[figures4papers的scientific-figure-making skill](https://github.com/ChenLiu-1996/figures4papers)，读取版本为`0ba898c8024ed4ce5bd06c8e5500d189fcd7a160`；中文使用Noto Sans CJK SC。

脚本为本项目新编写，未复制该仓库的示例脚本或资产。该仓库许可为CC BY-NC 4.0；这里对作图规范注明来源，不将其当作任务规划算法依据。

## 架构图与流程图

图1按用户追加要求改用[FigGenie](https://github.com/Sleepy-Avacado/FigGenie-paper-diagram-skill)的control-loop／control-data表达，读取版本`82be26d6f2954c8bfc4089d70d8dcf82fe395864`。采用白底分区、单一强强调、薄线正交主链和独立边缘反馈，区分项目联合任务组织与复用运动后端；参考其布局与核查机制，不复制示例论文的模块或资产。

`architecture/`保留brief、spec、plan和原生可编辑SVG。根目录`fig01_task_loop.svg`为字体子集嵌入版，132种字符均有覆盖；PDF的字体嵌入核对通过。字体为Arimo与Noto Sans CJK SC，采用对应开放字体许可。

在本插图目录内使用FigGenie时可运行：

```bash
python <FigGenie目录>/skills/figgenie-paper-diagram/scripts/check.py architecture/fig.svg --spec architecture/spec.json --scale 4.167
python <FigGenie目录>/skills/figgenie-paper-diagram/scripts/embed_fonts.py architecture/fig.svg -o fig01_task_loop.svg --strict
python <FigGenie目录>/skills/figgenie-paper-diagram/scripts/export_pdf.py fig01_task_loop.svg -o fig01_task_loop.pdf
```

本次临时工具接入服务器已有Chrome和Noto SC字形，不安装全局浏览器／字体。几何核查零错误；保留三条同源Action扇出边共用尾点的提示，见`architecture/plan.md`，不是标签或线路越界。此处核查只针对交付图形，不新增物理仿真或控制验收。

数据、口径和原文件哈希见[数据说明](../data/README.md)。没有用概念效果图充当实际场景，没有从视频长度反推执行时长，没有为单次记录绘制统计误差条。
