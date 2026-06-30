# 0609thesis package

This folder is a self-contained thesis package.

- Main TeX entry: `main_ustc.tex`
- Thesis setup: `ustcsetup.tex`
- Chapter files: `chapters/`
- Bibliography: `refs/references.bib`
- Template files: `ustcthesis.cls`, `ustcthesis-*.bst`, `ustcthesis-*.bbx`, `ustcthesis-*.cbx`
- Local thesis figures: `figures/`
- Third-chapter generated figures copied from the project figure set: `figures/panda_robot/`
- Reference PDFs and supporting material: `attachments/reference_pdfs/`
- Plotting and data-analysis scripts used to generate supporting figures: `attachments/plotting_code/`
- Compiled PDF output: `compiled_pdfs/main_ustc.pdf`

To rebuild:

```bash
latexmk -xelatex -interaction=nonstopmode -halt-on-error main_ustc.tex
```
