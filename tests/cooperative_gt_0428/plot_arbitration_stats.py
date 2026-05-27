#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
多仲裁策略多组实验统计比较。

用法:
  python plot_arbitration_stats.py
  python plot_arbitration_stats.py results/rcm_arbitration_compare_YYYYMMDD_HHMMSS

默认读取 results/ 下最近更新的 rcm_arbitration_compare_* 目录。每个 .npz
视为一组 trial，按 arbitration_strategy 字段或文件名分组。
"""
import argparse
import csv
import math
import os
import re
from itertools import combinations
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager


METRICS = [
    ("force_rmse", "Force RMSE (N)"),
    ("force_mae", "Force MAE (N)"),
    ("force_peak_abs_err", "Peak |Force Error| (N)"),
    ("pos_rmse_mm", "Tool Position RMSE (mm)"),
    ("pos_peak_mm", "Peak Tool Position Error (mm)"),
    ("rcm_rmse_mm", "RCM RMSE (mm)"),
    ("rcm_peak_mm", "Peak RCM Error (mm)"),
    ("force_dF_p95", "95% |dF/dt| (N/s)"),
    ("alpha_mean", "Mean Alpha"),
]


def setup_font():
    font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
    if os.path.exists(font_path):
        try:
            font_prop = font_manager.FontProperties(fname=font_path)
            plt.rcParams["font.sans-serif"] = [font_prop.get_name(), "DejaVu Sans"]
        except Exception:
            plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    else:
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def latest_compare_dir(results_root):
    root = Path(results_root)
    candidates = [
        p for p in root.glob("rcm_arbitration_compare_*")
        if p.is_dir() and list(p.glob("*.npz"))
    ]
    if not candidates:
        candidates = [
            p.parent for p in root.rglob("*.npz")
            if "arbitration" in p.parent.name or "compare" in p.parent.name
        ]
    if not candidates:
        raise FileNotFoundError(
            f"{results_root} 下没有 rcm_arbitration_compare_* 数据目录"
        )
    return max(set(candidates), key=lambda p: p.stat().st_mtime)


def fit_length(arr, n, fill=0.0):
    arr = np.asarray(arr).flatten()
    if len(arr) == n:
        return arr
    if len(arr) == 0:
        return np.full(n, fill)
    if len(arr) == 1:
        return np.full(n, arr[0])
    out = np.full(n, fill, dtype=arr.dtype)
    m = min(n, len(arr))
    out[:m] = arr[:m]
    if m < n:
        out[m:] = arr[m - 1]
    return out


def numeric(data, key, n=None, fill=0.0):
    if key not in data:
        if n is None:
            return np.array([], dtype=float)
        return np.full(n, fill, dtype=float)
    arr = np.asarray(data[key], dtype=float).flatten()
    if n is not None:
        arr = fit_length(arr, n, fill=fill).astype(float)
    return arr


def infer_strategy(path, data):
    if "arbitration_strategy" in data:
        arr = np.asarray(data["arbitration_strategy"]).flatten()
        if len(arr):
            value = str(arr[0])
            if value and value != "nan":
                return value
    name = path.stem
    if "continuous_force_margin" in name:
        return "continuous_force_margin_alpha"
    if "force_margin" in name:
        return "force_margin_alpha"
    if "s_curve" in name:
        return "s_curve_alpha"
    if "fixed" in name:
        match = re.search(r"fixed[_-]?(\d+)", name)
        if match:
            return "fixed_" + match.group(1)
        return "fixed"
    return name.split("_t")[0]


def trial_metrics(path):
    data = dict(np.load(path, allow_pickle=True))
    if "t" not in data:
        raise ValueError(f"{path} 缺少 t 字段")
    t = numeric(data, "t")
    n = len(t)
    if n < 2:
        raise ValueError(f"{path} 数据点过少")

    F = numeric(data, "F_measured", n)
    F_des = numeric(data, "F_desired", n, fill=np.nanmean(F) if len(F) else 0.0)
    F_err = numeric(data, "F_err", n) if "F_err" in data else F - F_des

    if "pos_err_norm" in data:
        pos_err = numeric(data, "pos_err_norm", n)
    elif all(k in data for k in ("pos_err_x", "pos_err_y", "pos_err_z")):
        ex = numeric(data, "pos_err_x", n)
        ey = numeric(data, "pos_err_y", n)
        ez = numeric(data, "pos_err_z", n)
        pos_err = np.sqrt(ex * ex + ey * ey + ez * ez)
    else:
        pos_err = np.zeros(n)

    rcm_err = np.abs(numeric(data, "error_rcm", n))
    alpha = numeric(data, "alpha", n, fill=np.nan)
    u_norm = numeric(data, "u_norm", n, fill=np.nan)

    dt = np.diff(t)
    valid_dt = np.maximum(dt, 1e-6)
    dF = np.diff(F) / valid_dt if n > 1 else np.array([0.0])

    strategy = infer_strategy(path, data)
    return {
        "file": str(path),
        "strategy": strategy,
        "duration": float(t[-1] - t[0]),
        "samples": int(n),
        "force_rmse": float(np.sqrt(np.nanmean(F_err ** 2))),
        "force_mae": float(np.nanmean(np.abs(F_err))),
        "force_mean_err": float(np.nanmean(F_err)),
        "force_peak_abs_err": float(np.nanmax(np.abs(F_err))),
        "force_std": float(np.nanstd(F)),
        "force_dF_p95": float(np.nanpercentile(np.abs(dF), 95)),
        "pos_rmse_mm": float(np.sqrt(np.nanmean(pos_err ** 2)) * 1000.0),
        "pos_peak_mm": float(np.nanmax(pos_err) * 1000.0),
        "rcm_rmse_mm": float(np.sqrt(np.nanmean(rcm_err ** 2)) * 1000.0),
        "rcm_peak_mm": float(np.nanmax(rcm_err) * 1000.0),
        "alpha_mean": float(np.nanmean(alpha)),
        "alpha_std": float(np.nanstd(alpha)),
        "u_rms": float(np.sqrt(np.nanmean(u_norm ** 2))),
    }


def collect_trials(input_path, results_root):
    if input_path is None:
        root = latest_compare_dir(results_root)
    else:
        root = Path(input_path)
    files = sorted(root.glob("*.npz")) if root.is_dir() else [root]
    rows = []
    for path in files:
        try:
            rows.append(trial_metrics(path))
        except Exception as exc:
            print(f"跳过 {path}: {exc}")
    if not rows:
        raise RuntimeError(f"{root} 中没有可用 .npz 数据")
    return root, rows


def group_values(rows, metric):
    groups = {}
    for row in rows:
        groups.setdefault(row["strategy"], []).append(row[metric])
    return {k: np.asarray(v, dtype=float) for k, v in groups.items()}


def normal_cdf(x):
    return 0.5 * (1.0 + math.erf(x / np.sqrt(2.0)))


def permutation_pvalue(a, b, n_perm=20000, seed=7):
    rng = np.random.default_rng(seed)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    observed = abs(np.nanmean(a) - np.nanmean(b))
    pooled = np.concatenate([a, b])
    n_a = len(a)
    count = 0
    for _ in range(n_perm):
        rng.shuffle(pooled)
        diff = abs(np.nanmean(pooled[:n_a]) - np.nanmean(pooled[n_a:]))
        if diff >= observed:
            count += 1
    return (count + 1.0) / (n_perm + 1.0)


def pairwise_tests(rows, metric):
    groups = group_values(rows, metric)
    out = []
    try:
        from scipy import stats
        scipy_ok = True
    except Exception:
        stats = None
        scipy_ok = False

    for a_name, b_name in combinations(sorted(groups), 2):
        a = groups[a_name]
        b = groups[b_name]
        if scipy_ok:
            p_t = float(stats.ttest_ind(a, b, equal_var=False, nan_policy="omit").pvalue)
            p_u = float(stats.mannwhitneyu(a, b, alternative="two-sided").pvalue)
            method = "Welch t / Mann-Whitney U"
        else:
            p_t = permutation_pvalue(a, b)
            p_u = p_t
            method = "permutation"
        out.append({
            "metric": metric,
            "a": a_name,
            "b": b_name,
            "mean_a": float(np.nanmean(a)),
            "mean_b": float(np.nanmean(b)),
            "delta_a_minus_b": float(np.nanmean(a) - np.nanmean(b)),
            "p_t": p_t,
            "p_nonparam": p_u,
            "method": method,
        })
    return out


def significance_label(p):
    if p < 0.001:
        return "***"
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return "n.s."


def write_csv(path, rows, tests):
    metrics_path = path / "arbitration_trial_metrics.csv"
    tests_path = path / "arbitration_pairwise_tests.csv"
    metric_keys = list(rows[0].keys())
    with metrics_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=metric_keys)
        writer.writeheader()
        writer.writerows(rows)
    with tests_path.open("w", newline="") as f:
        fieldnames = ["metric", "a", "b", "mean_a", "mean_b",
                      "delta_a_minus_b", "p_t", "p_nonparam", "method"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(tests)
    print(f"统计表已保存: {metrics_path}")
    print(f"显著性检验已保存: {tests_path}")


def print_summary(rows, tests):
    print("\n=== 每种仲裁策略统计 ===")
    for strategy in sorted({r["strategy"] for r in rows}):
        subset = [r for r in rows if r["strategy"] == strategy]
        print(f"\n{strategy} (n={len(subset)})")
        for metric, label in METRICS[:7]:
            vals = np.asarray([r[metric] for r in subset], dtype=float)
            print(f"  {label}: mean={np.nanmean(vals):.4g}, std={np.nanstd(vals, ddof=1):.4g}")

    print("\n=== 主要指标显著性 p 值 ===")
    for metric in ("force_rmse", "pos_rmse_mm", "rcm_rmse_mm", "force_dF_p95"):
        print(f"\n{metric}")
        for item in tests:
            if item["metric"] == metric:
                p = item["p_nonparam"]
                print(
                    f"  {item['a']} vs {item['b']}: "
                    f"p={p:.4g} {significance_label(p)}, "
                    f"delta={item['delta_a_minus_b']:.4g}"
                )


def plot_metric_grid(rows, tests, output_dir):
    strategies = sorted({r["strategy"] for r in rows})
    fig, axes = plt.subplots(3, 3, figsize=(14, 11))
    axes = axes.flatten()

    for ax, (metric, label) in zip(axes, METRICS):
        data = [np.asarray([r[metric] for r in rows if r["strategy"] == s], dtype=float)
                for s in strategies]
        means = [np.nanmean(v) for v in data]
        sems = [
            np.nanstd(v, ddof=1) / np.sqrt(len(v)) if len(v) > 1 else 0.0
            for v in data
        ]
        x = np.arange(len(strategies))
        ax.bar(x, means, yerr=sems, color=["#4C78A8", "#F58518", "#54A24B"][:len(x)],
               alpha=0.72, capsize=4, edgecolor="black", linewidth=0.6)
        ax.boxplot(data, positions=x, widths=0.36, patch_artist=False,
                   showfliers=True, medianprops={"color": "black"})
        for i, vals in enumerate(data):
            jitter = np.linspace(-0.08, 0.08, len(vals)) if len(vals) else []
            ax.scatter(np.full(len(vals), i) + jitter, vals, s=28,
                       color="black", alpha=0.75, zorder=3)
        ax.set_title(label)
        ax.set_xticks(x)
        ax.set_xticklabels(strategies, rotation=20, ha="right")
        ax.grid(True, axis="y", alpha=0.25)

        metric_tests = [item for item in tests if item["metric"] == metric]
        significant = [
            item for item in metric_tests
            if item["p_nonparam"] < 0.05
        ]
        if significant:
            text = "\n".join(
                f"{item['a']} vs {item['b']}: {significance_label(item['p_nonparam'])}"
                for item in significant[:3]
            )
        else:
            best = min(metric_tests, key=lambda item: item["p_nonparam"], default=None)
            text = (
                f"min p={best['p_nonparam']:.3g} {significance_label(best['p_nonparam'])}"
                if best else ""
            )
        if text:
            ax.text(0.02, 0.98, text, transform=ax.transAxes,
                    va="top", ha="left", fontsize=8,
                    bbox=dict(boxstyle="round", facecolor="white", alpha=0.75))

    fig.suptitle("不同仲裁策略实验结果统计比较", fontsize=15)
    fig.tight_layout()
    out = output_dir / "arbitration_stats_summary.png"
    fig.savefig(out, dpi=180)
    print(f"统计图已保存: {out}")
    return fig


def plot_time_series_examples(rows, output_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    chosen = {}
    for row in rows:
        chosen.setdefault(row["strategy"], row["file"])

    for strategy, file_path in sorted(chosen.items()):
        data = dict(np.load(file_path, allow_pickle=True))
        t = numeric(data, "t")
        n = len(t)
        axes[0].plot(t, numeric(data, "F_measured", n), label=strategy, linewidth=1.4)
        axes[1].plot(t, numeric(data, "pos_err_norm", n) * 1000.0,
                     label=strategy, linewidth=1.4)
        axes[2].plot(t, numeric(data, "alpha", n), label=strategy, linewidth=1.4)

    axes[0].set_ylabel("Force (N)")
    axes[1].set_ylabel("Tool error (mm)")
    axes[2].set_ylabel("Alpha")
    axes[2].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.suptitle("各仲裁策略代表 trial 时序对比", fontsize=14)
    fig.tight_layout()
    out = output_dir / "arbitration_timeseries_examples.png"
    fig.savefig(out, dpi=180)
    print(f"时序图已保存: {out}")
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", nargs="?", default=None,
                        help="结果目录或单个 .npz；默认读取最近的 rcm_arbitration_compare_*")
    parser.add_argument("--results-root", default="results")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    setup_font()
    root, rows = collect_trials(args.input, args.results_root)
    print(f"读取数据目录: {root}")

    tests = []
    for metric, _ in METRICS:
        tests.extend(pairwise_tests(rows, metric))

    output_dir = root if root.is_dir() else root.parent
    write_csv(output_dir, rows, tests)
    print_summary(rows, tests)
    plot_metric_grid(rows, tests, output_dir)
    plot_time_series_examples(rows, output_dir)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
