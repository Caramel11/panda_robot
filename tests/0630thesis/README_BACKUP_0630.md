# 0630thesis 备份说明

本目录是 `0609thesis` 的 2026-06-30 备份版本。论文正文、图表、参考文献、附件和 0609 打包文件来自 `0609thesis`，LaTeX 模板类文件、模板说明文件和模板示例资源来自 `tests/ustcthesis`。

## 论文入口

本备份的论文入口为：

```bash
main_ustc.tex
```

目录中的 `main.tex` 是 `ustcthesis` 模板自带示例文件，保留用于查阅模板结构，不作为本论文的默认编译入口。

## VS Code 编译

已在 `.vscode/settings.json` 中配置 LaTeX Workshop，默认 recipe 为：

```bash
latexmk (TeX Live 2026 xelatex)
```

该 recipe 调用工作空间内的 TeX Live 2026：

```bash
/home/liu/franka_ws_1101/.texlive/2026/bin/x86_64-linux/latexmk
```

在 VS Code 中打开 `main_ustc.tex` 后，使用 LaTeX Workshop 的 Build LaTeX project 命令即可编译。编译产物输出到：

```bash
compiled_pdfs/main_ustc.pdf
```

## 命令行验证

本备份已通过以下命令完成编译验证：

```bash
/home/liu/franka_ws_1101/.texlive/2026/bin/x86_64-linux/latexmk -g -xelatex -interaction=nonstopmode -halt-on-error main_ustc.tex
```

当前验证产物为 113 页 PDF。
