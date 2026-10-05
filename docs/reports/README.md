# 项目报告

项目报告的模板、可编辑源码和编译版统一保存在本目录，与运行代码、仿真视频和原始实验数据分开管理。

## 中期重要进展及成果

对应中期执行情况报告的“二、取得的重要进展及成果／1.课题中期重要进展及成果”。

| 文件 | 用途 |
|---|---|
| [统一理论报告 PDF](midterm-unified-theory-20261005.pdf) | A4、12页，适合直接阅读和审核 |
| [可编辑 LaTeX 源码](midterm-unified-theory-20261005.tex) | UTF-8、XeLaTeX，使用 TeXstudio 编辑 |
| [原 Word 模板](“海洋科技”重大专项旗舰项目中期执行情况报告（参照附件2修改补充）.docx) | 原文件从仓库根目录移入，内容保持不变 |

报告归纳三项进展：

1. 异构跨域任务、资源与运动统一模型。
2. 快速完整候选联合求解及事件反馈修复。
3. 自主运动、合格进度、支援实收及规定返回的闭环整体可行性。

理论稿包含48个编号公式、4个命题、1个整体可行性定理与10项参考资料。整体证明由跟踪误差界推导真实进度正增长、有限作业完成，再沿任务／报告事件图推导必要实收与规定返回；定理的适用条件在正文中明确给出。

当前为指定章节的独立理论稿，尚未回填 Word 或完成项目正式审核。项目申报书和子课题03实施方案继续作为共用背景保存在 [`context/`](../../context/)，不重复复制。

## 编辑与编译

用 TeXstudio 打开 `.tex` 文件；文件头已指定 XeLaTeX。也可以从仓库根目录执行：

```bash
cd docs/reports
xelatex -interaction=nonstopmode -halt-on-error midterm-unified-theory-20261005.tex
xelatex -interaction=nonstopmode -halt-on-error midterm-unified-theory-20261005.tex
```

文献内嵌于源码，两次编译解析引用，无需单独的 BibTeX 文件。源码使用 TeX Live 自带 Fandol 中文字体；编译辅助文件由本目录的 `.gitignore` 排除，保留最终 `.tex` 和 `.pdf`。
