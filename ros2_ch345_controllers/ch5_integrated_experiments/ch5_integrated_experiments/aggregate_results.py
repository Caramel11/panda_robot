#!/usr/bin/env python3
"""第5章多次实验结果聚合与论文表格导出。

`run_ch5_gazebo_suite` 负责生成单个 suite 的 `ch5_gazebo_metrics.csv`。
正式论文通常需要多个 trial，因此本模块负责把多个 suite 的 CSV 汇总为：

- `ch5_aggregate_metrics.csv`：每个方法的均值、标准差、最小值、最大值；
- `ch5_paper_table.tex`：可直接粘贴到论文中的 LaTeX 表格；
- `CH5_AGGREGATE_REPORT.md`：方法可行性、缺失实验和写作建议。

统计方法当前使用总体标准差 `pstdev`。如果正式论文需要显著性检验，可在
本模块继续添加 t 检验、Mann-Whitney U 检验或 bootstrap 置信区间。
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean, pstdev


DEFAULT_NUMERIC_FIELDS = [
    "tracking_last_rms_m",
    "tracking_last_max_m",
    "R_acc",
    "R_sup",
    "T_vio_s",
    "F_peak_N",
    "alpha_HR_mean",
    "alpha_FP_mean",
    "rcm_last_rms_m",
    "score",
]


def _truthy(value):
    """把 CSV 中的成功标记解析为布尔值。"""

    return str(value).strip().lower() in ("1", "true", "yes", "y")


def _float(row, key):
    """安全读取浮点字段；缺失或非法时返回 None。"""

    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return None


def _csv_paths(inputs):
    """从文件或目录参数中收集所有 `ch5_gazebo_metrics.csv`。"""

    paths = []
    for item in inputs:
        p = Path(item)
        if p.is_file():
            paths.append(p)
        elif p.is_dir():
            paths.extend(sorted(p.rglob("ch5_gazebo_metrics.csv")))
        else:
            raise FileNotFoundError(item)
    return sorted(set(paths))


def read_rows(inputs):
    """读取多个 CSV 的所有原始行，并记录来源文件路径。"""

    rows = []
    for path in _csv_paths(inputs):
        with open(path, "r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                row = dict(row)
                row["source_csv"] = str(path)
                rows.append(row)
    return rows


def aggregate(rows, numeric_fields=None):
    """按 method 聚合多次实验。

    对每个方法统计：
    - `n`：实验次数；
    - `success_count/success_rate`：满足第5章成功判据的次数/比例；
    - 每个数值指标的 mean/std/min/max。
    """

    numeric_fields = numeric_fields or DEFAULT_NUMERIC_FIELDS
    grouped = defaultdict(list)
    for row in rows:
        grouped[row.get("method", "unknown")].append(row)

    out = []
    for method, items in sorted(grouped.items()):
        summary = {
            "method": method,
            "n": len(items),
            "success_count": sum(1 for r in items if _truthy(r.get("success", ""))),
            "success_rate": sum(1 for r in items if _truthy(r.get("success", ""))) / max(len(items), 1),
        }
        for field in numeric_fields:
            values = [_float(r, field) for r in items]
            values = [v for v in values if v is not None]
            if values:
                summary[f"{field}_mean"] = mean(values)
                summary[f"{field}_std"] = pstdev(values) if len(values) > 1 else 0.0
                summary[f"{field}_min"] = min(values)
                summary[f"{field}_max"] = max(values)
        out.append(summary)
    return out


def write_csv(rows, path):
    """写出机器可读的聚合 CSV。"""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({k for row in rows for k in row})
    preferred = ["method", "n", "success_count", "success_rate"]
    fields = preferred + [f for f in fields if f not in preferred]
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _fmt_mm(row, key):
    """把米单位指标格式化为论文常用的 mm 均值±标准差。"""

    m = row.get(f"{key}_mean")
    s = row.get(f"{key}_std", 0.0)
    if m is None:
        return "--"
    return f"{1000.0 * float(m):.2f} $\\pm$ {1000.0 * float(s):.2f}"


def _fmt(row, key, nd=3):
    """把无量纲或牛顿指标格式化为均值±标准差。"""

    m = row.get(f"{key}_mean")
    s = row.get(f"{key}_std", 0.0)
    if m is None:
        return "--"
    return f"{float(m):.{nd}f} $\\pm$ {float(s):.{nd}f}"


def write_latex(rows, path):
    """生成论文用 LaTeX 表格片段。

    表格列选择保持克制，只放最能支撑第5章结论的指标。更详细的指标仍保留
    在 `ch5_aggregate_metrics.csv` 中，避免正文表格过宽。
    """

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "\\begin{table}[htbp]",
        "\\centering",
        "\\caption{第5章综合实验主要指标统计}",
        "\\label{tab:ch5_integrated_metrics}",
        "\\begin{tabular}{lcccccc}",
        "\\hline",
        "方法 & 成功率 & 末段RMS/mm & $F_{peak}$/N & $R_{acc}$ & $R_{sup}$ & $\\bar{\\alpha}_{FP}$ \\\\",
        "\\hline",
    ]
    for row in rows:
        success = f"{100.0 * float(row.get('success_rate', 0.0)):.0f}\\%"
        lines.append(
            f"{row['method']} & {success} & {_fmt_mm(row, 'tracking_last_rms_m')} & "
            f"{_fmt(row, 'F_peak_N', 3)} & {_fmt(row, 'R_acc', 3)} & "
            f"{_fmt(row, 'R_sup', 3)} & {_fmt(row, 'alpha_FP_mean', 3)} \\\\"
        )
    lines.extend([
        "\\hline",
        "\\end{tabular}",
        "\\end{table}",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def write_markdown(rows, raw_rows, path, expected_methods=None):
    """生成查漏补缺报告。

    该报告用于回答“哪些对比方法已验证、哪些还缺数据、哪些可作为消融劣化”
    这类实验管理问题。
    """

    expected_methods = expected_methods or []
    by_method = {r["method"]: r for r in rows}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 第五章实验统计与查漏补缺报告",
        "",
        f"- 原始记录数：{len(raw_rows)}",
        f"- 方法数：{len(rows)}",
        "",
        "## 方法可行性",
        "",
        "| method | n | success_rate | tracking_rms_mm | tracking_max_mm | T_vio_s | alpha_HR_mean | alpha_FP_mean | 结论 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for method in expected_methods or [r["method"] for r in rows]:
        row = by_method.get(method)
        if row is None:
            lines.append(f"| {method} | 0 | 0% | -- | -- | -- | -- | -- | 缺少实验数据 |")
            continue
        success_rate = float(row.get("success_rate", 0.0))
        rms = row.get("tracking_last_rms_m_mean")
        maxe = row.get("tracking_last_max_m_mean")
        tvio = row.get("T_vio_s_mean")
        alpha_hr = row.get("alpha_HR_mean_mean")
        alpha_fp = row.get("alpha_FP_mean_mean")
        if success_rate >= 0.999:
            conclusion = "满足成功判据"
        elif success_rate > 0.0:
            conclusion = "部分成功，需增加重复次数"
        else:
            conclusion = "可运行但未满足成功判据或作为消融劣化"
        lines.append(
            f"| {method} | {row['n']} | {100.0*success_rate:.0f}% | "
            f"{1000.0*float(rms):.2f} | {1000.0*float(maxe):.2f} | "
            f"{float(tvio):.2f} | {float(alpha_hr):.3f} | {float(alpha_fp):.3f} | {conclusion} |"
        )

    missing = [m for m in expected_methods if m not in by_method]
    lines.extend([
        "",
        "## 论文使用建议",
        "",
        "- `full_method` 是主方法，应展示时序图和综合指标。",
        "- `balanced_fixed` 用于说明固定权重基线。",
        "- `reference_only` 用于隔离第4章参考层贡献。",
        "- `execution_only` 用于隔离第3章执行层贡献；若误差略超阈值，应作为消融劣化解释，而不是删除。",
    ])
    if missing:
        lines.append(f"- 以下预期方法缺少数据：{', '.join(missing)}。正式论文中不要声称这些消融已经完成。")
    else:
        lines.append("- 当前预期对比方法均已有数据；正式论文还应把每个方法重复到至少 3 次。")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def order_rows(rows, expected_methods):
    """按照论文方法顺序排序，而不是按字母排序。"""

    rank = {name: i for i, name in enumerate(expected_methods)}
    return sorted(rows, key=lambda row: (rank.get(row["method"], len(rank)), row["method"]))


def main(argv=None):
    """命令行入口。"""

    ap = argparse.ArgumentParser()
    ap.add_argument("--inputs", nargs="+", required=True, help="Metric CSV files or suite directories.")
    ap.add_argument("--output-dir", required=True)
    ap.add_argument(
        "--expected-methods",
        default="balanced_fixed,reference_only,execution_only,full_method",
    )
    args = ap.parse_args(argv)

    raw_rows = read_rows(args.inputs)
    if not raw_rows:
        raise RuntimeError("No ch5 metric rows found.")
    rows = aggregate(raw_rows)
    out_dir = Path(args.output_dir)
    expected = [x.strip() for x in args.expected_methods.split(",") if x.strip()]
    rows = order_rows(rows, expected)
    write_csv(rows, out_dir / "ch5_aggregate_metrics.csv")
    write_latex(rows, out_dir / "ch5_paper_table.tex")
    write_markdown(rows, raw_rows, out_dir / "CH5_AGGREGATE_REPORT.md", expected_methods=expected)
    print(json.dumps({
        "raw_rows": len(raw_rows),
        "methods": len(rows),
        "aggregate_csv": str(out_dir / "ch5_aggregate_metrics.csv"),
        "latex_table": str(out_dir / "ch5_paper_table.tex"),
        "report": str(out_dir / "CH5_AGGREGATE_REPORT.md"),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
