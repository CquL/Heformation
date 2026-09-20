# Heformation 背景资料更新包（2026-09-19）

本包按远端 `CquL/Heformation` 的 `main` 提交
`79036d66178deacada2b4f4a30dc588899eeb094` 整理。

用途：覆盖 `/home/lhj/Swarm-Formation/AGENT/` 与
`/home/lhj/Swarm-Formation/context/` 中已经明显过时的背景文件。

## 本包只更新背景资料，不修改代码

包含：

- `AGENT/AGENT.md`
- `AGENT/PROJECT_CONTEXT.md`
- `context/README.md`
- `context/01_project_background.md`
- `context/02_current_status.md`
- `context/03_system_architecture.md`
- `context/04_inputs_outputs.md`
- `context/05_state_communication.md`
- `context/08_first_scenario_benchmark.md`
- `context/09_implementation_plan.md`
- `context/10_research_novelty.md`
- `context/11_glossary.md`
- `context/12_literature_review.md`
- `context/13_references.md`
- `context/14_decisions_and_unknowns.md`
- `context/15_handoff.md`

没有改动 `06/07/16`、`sources/`、历史 V2 文件、代码、实验或上游源码。

## 建议覆盖方式

先备份现有背景文件，再覆盖同名文件。不要删除整个目录，因为目录内还有本包没有修改的资料。

如果项目继续依赖 `AGENT/CONTENT_MANIFEST.json` 做文件哈希校验，应在覆盖后使用项目原有生成方式重新生成。
本包没有伪造新的 manifest。
