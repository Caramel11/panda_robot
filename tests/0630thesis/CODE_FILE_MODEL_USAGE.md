# 0630thesis 代码文件建模方法与使用说明

`tests/0630thesis` 是当前硕士论文工程。它包含 USTC thesis 模板、正文章节、参考文献、论文图、实验绘图代码和章节修改方案。本文按可执行/可复现文件说明其建模角色和使用方法；普通图片、PDF 结果和参考资料不逐一展开。

## 论文构建入口

| 文件 | 建模或组织方法 | 使用方法 |
|---|---|---|
| `main_ustc.tex` | 当前主论文入口。加载 USTC 模板、章节、图路径和参考文献，是最终编译目标。 | 推荐：`latexmk -xelatex -outdir=compiled_pdfs main_ustc.tex`。 |
| `main.tex` | 模板示例/备用入口，保留 ustcthesis 原始结构。 | 需要对照模板行为时编译。正式论文优先使用 `main_ustc.tex`。 |
| `ustcsetup.tex` | 学位类型、题名、作者、导师、摘要等模板元信息配置。 | 修改论文基本信息时编辑。 |
| `latexmkrc` | LaTeX 自动编译规则，指定 XeLaTeX、BibTeX/Biber 和输出清理策略。 | VS Code LaTeX Workshop 或命令行会读取。 |
| `Makefile` | 封装编译、清理和输出命令。 | `make` 编译，`make clean` 清理中间文件。 |
| `.vscode/settings.json` | 0630thesis 局部 VS Code 编译/预览设置。 | 用于减少 workspace 级设置干扰。 |
| `.gitignore` | 排除 LaTeX 中间文件、压缩包、参考 PDF 原文和本地临时输出。 | 提交前无需手动添加中间文件。 |
| `.gitattributes` | 将 PDF/PNG/JPG 作为二进制处理，避免 `diff --check` 扫描图像内容。 | 无需直接运行。 |

## 正文章节源文件

| 文件 | 建模内容 | 使用方法 |
|---|---|---|
| `chapters/abstract.tex` | 中文/英文摘要。概括柔性接触、人机协同、RCM 约束和双仲裁贡献。 | 修改摘要和关键词。 |
| `chapters/chapter1.tex` | 绪论。建立研究背景、应用场景、国内外研究现状和论文结构。 | 引用综述文献和章节组织图。 |
| `chapters/chapter2.tex` | 理论基础。描述机器人运动学、任务空间误差、柔性接触 Kelvin-Voigt 模型、力位协调和 Pareto 仲裁。 | 修改基础符号时同步检查后续章节。 |
| `chapters/chapter3.tex` | 当前第三章。围绕柔性接触力位协同控制和 continuous force margin 仲裁展开，包含 0603/第四章迁移来的仿真结果。 | 修改 0603 实验图或参数后同步更新。 |
| `chapters/chapter4.tex` | 当前第四章。围绕人机共享控制、模糊/KF 意图估计、GT/MPC 对照和 0213 实验展开。 | 修改第三章共享控制实验结果时同步更新。 |
| `chapters/chapter5.tex` | 第五章。描述参考层人机仲裁 + 执行层力位仲裁的双层方法，并使用 0608controller 仿真结果。 | 修改 `run_ch5_dual_arbitration.py` 后同步图和指标。 |
| `chapters/chapter6.tex` | 总结与展望。 | 完成全文修改后最后统一调整。 |
| `chapters/notation.tex` | 符号表。 | 新增变量时同步维护。 |
| `chapters/achievements.tex` | 成果列表。 | 填写论文、专利、项目等。 |
| `chapters/acknowledgements.tex`、`chapters/acknowledgments.tex` | 致谢文件，两种命名保留兼容。 | 正式提交前确认主入口使用哪一个。 |
| `chapters/citations.tex`、`chapters/floats.tex`、`chapters/math.tex`、`chapters/intro.tex`、`chapters/complementary.tex`、`chapters/innovations.tex` | 模板示例章节或补充材料。 | 主要用于保留模板结构和参考写法，正式正文以 chapter1-6 为主。 |

## 绘图代码

| 文件 | 建模方法 | 使用方法 |
|---|---|---|
| `attachments/plotting_code/fuzzy_logic.py` | 第三章共享控制模糊系统。绝对模式输出仲裁参数，增量模式输出仲裁增量。 | 被多个绘图脚本 import。 |
| `attachments/plotting_code/kalman_filter.py` | 卡尔曼融合和增量仲裁累积器。 | 用于展示模糊/KF 平滑机制。 |
| `attachments/plotting_code/ieee_style.py` | 统一 IEEE/论文图样式、尺寸和保存函数。 | 绘图脚本调用 `apply_ieee_style()` 和 `save_fig()`。 |
| `attachments/plotting_code/plot_fuzzy.py` | 绘制绝对模糊系统隶属函数、规则表和曲面。 | `python3 attachments/plotting_code/plot_fuzzy.py`。 |
| `attachments/plotting_code/plot_delta_fuzzy.py` | 绘制增量模糊系统隶属函数、规则表和曲面。 | `python3 attachments/plotting_code/plot_delta_fuzzy.py`。 |
| `attachments/plotting_code/plot_fig3_ieee.py` | 一键生成共享控制模糊图的 IEEE 版本。 | 用于投稿或论文图重绘。 |
| `attachments/plotting_code/plot_gt_analysis.py` | 单次 GT 实验参数同步图。检测交互力事件并标注时间区间。 | 修改 `base_path` 或命令参数后运行。 |
| `attachments/plotting_code/plot_traj_gt.py` | 绘制代表性轨迹、目标点和误差曲线。 | 从 0213 日志生成第三/第四章轨迹图。 |
| `attachments/plotting_code/data_analysis_4method.py` | 四方法统计图和显著性检验。 | 读取 0213 日志，生成箱线图和统计表。 |
| `attachments/plotting_code/data_analysis_GTvsMPC.py` | GT 与 MPC 对照统计。 | 用于分析控制器类别差异。 |
| `attachments/plotting_code/data_analysis_OtherTarget.py` | 目标切换/其他目标统计。 | 用于第四章目标切换实验图。 |
| `attachments/plotting_code/plot_ch3_force_margin_fuzzy.py` | 第三章 force-margin 模糊仲裁图。显式绘制力误差、力安全裕度、位置裕度到 $\alpha$ 的映射、趋势和曲面。 | `python3 attachments/plotting_code/plot_ch3_force_margin_fuzzy.py`；输出到对应 figures 子目录。 |
| `attachments/plotting_code/plot_ch5_fuzzy_arbitration.py` | 第五章双层仲裁图。绘制人机仲裁和力位仲裁的隶属函数、响应曲线与方向性保护。 | `python3 attachments/plotting_code/plot_ch5_fuzzy_arbitration.py`。 |
| `scripts/plot_ch2_theory_figures.py` | 第二章理论图生成脚本。生成任务空间映射、接触模型、力位协调和 Pareto 仲裁示意图。 | `python3 scripts/plot_ch2_theory_figures.py`。 |

## 参考文献和模板文件

| 文件 | 作用 | 使用方法 |
|---|---|---|
| `refs/references.bib` | 当前全文合并参考文献。 | 正文新增引用时优先维护该文件。 |
| `refs/references_ch1.bib`、`references_ch234.bib`、`references_ch5.bib` | 分章来源参考文献。 | 用于追溯引用来源。 |
| `bib/ustc.bib` | USTC 模板参考文献示例。 | 一般不改。 |
| `ustcthesis.cls`、`ustcthesis-*.bbx/cbx/bst` | USTC thesis 类文件和参考文献样式。 | 除非模板兼容性问题，否则不修改。 |
| `ustcthesis-doc.tex`、`ustcthesis-doc.pdf` | 模板说明文档。 | 查询模板命令用法。 |

## 章节方案和辅助文档

| 文件 | 作用 |
|---|---|
| `WRITING_STYLE_GUIDE_0630.md` | 当前论文行文风格约束。 |
| `0630thesis_全文框架与章节修改方案.md`、`0630thesis_全文框架与章节修改方案_深化版.md` | 全文结构和章节迁移方案。 |
| `0630thesis_第三章基于原第四章修改方案与实验设置.md` | 第三章由原第四章迁移后的方法和实验设计。 |
| `0630thesis_第四章基于原第三章修改方案与实验设置.md` | 第四章由原第三章迁移后的方法和实验设计。 |
| `0630thesis_第五章基于原第五章修改方案与实验设置.md` | 第五章双层仲裁方法和实验设计。 |
| `0630thesis_通用力位协同对比控制器实现方案.md` | 通用力位协同对比控制器设计说明。 |
| `0630thesis_仿真与实物实验方案.md`、`0630thesis_仿真与实物实验方案_深化版.md` | 仿真与实物实验总体安排。 |
| `人机协同柔性物体操作典型任务与RCM约束场景.md`、`柔性接触力位耦合文献配图整理.md`、`论文写作对话记忆.md` | 论文写作和文献配图辅助材料。 |
| `CHANGELOG.md`、`KNOWLEDGE_MAP.md`、`REFERENCE_REVIEW_MAP.md`、`NEXT_DEEP_REVISION_PROMPT.md` | 修订记录、知识地图、参考文献核对和后续修改提示。 |

## 推荐使用流程

1. 修改正文时优先编辑 `main_ustc.tex`、`ustcsetup.tex` 和 `chapters/chapter*.tex`。
2. 修改实验图时优先回到对应实验目录生成原始图，再同步到 `figures/ch3_ch4` 或 `figures/ch5`。
3. 修改第二章理论示意图时运行 `scripts/plot_ch2_theory_figures.py`。
4. 修改第三/五章模糊仲裁图时运行 `plot_ch3_force_margin_fuzzy.py` 或 `plot_ch5_fuzzy_arbitration.py`。
5. 编译前执行 `latexmk -xelatex -outdir=compiled_pdfs main_ustc.tex`，确认 `compiled_pdfs/main_ustc.pdf` 更新。
