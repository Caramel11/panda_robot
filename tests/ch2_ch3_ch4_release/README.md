# 第二、三、四章 LaTeX 打包文件

本包包含第二章、第三章、第四章的完整 LaTeX 源文件，支持单章编译与合并编译。

## 文件结构

- `chapters/chapter2.tex`：第 2 章预备知识
- `chapters/chapter3.tex`：第 3 章刚性约束共享控制
- `chapters/chapter4.tex`：第 4 章柔性接触力--位协同控制
- `setup/preamble.tex`：通用宏包、定理环境与命令定义
- `refs/references.bib`：参考文献库
- `main_all.tex`：第 2--4 章合并编译入口
- `standalone/*.tex`：各章单独编译入口
- `compiled_pdfs/`：已编译 PDF 预览

## 编译方式

VSCode 一键编译：

1. 用 VSCode 打开 `ch2_ch3_ch4_release` 文件夹。
2. 安装 LaTeX Workshop 插件。
3. 打开 `main_all.tex` 后保存文件，或运行默认构建任务 `LaTeX: build main_all.tex`。
4. 输出文件位于 `compiled_pdfs/main_all.pdf`。保存触发编译后，PDF 预览会自动跳转到当前源码光标附近。

合并编译：

```bash
latexmk -xelatex main_all.tex
biber main_all
latexmk -xelatex main_all.tex
```

单章编译示例：

```bash
cd standalone
latexmk -xelatex chapter2_standalone.tex
biber chapter2_standalone
latexmk -xelatex chapter2_standalone.tex
```

## 说明

第二章采用通识预备知识写法，二级标题压缩为机器人运动学与动力学、约束运动与接触建模、最优控制与微分博弈、稳定性与鲁棒性分析四个主干内容，并使用定义、假设、引理、证明、注等模块组织关键基础结论。第三章与第四章已根据第二章符号体系进行衔接调整。
