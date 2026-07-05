#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analyze Chapter 3 RCM-constrained Gazebo/ROS2 experiment results."""

import argparse
import csv
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-ch3-rcm")

import numpy as np
import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt


def flatten_npz(path):
    """把 NPZ 原始字段转换为统一长度和类型，便于指标计算。"""
    raw = np.load(path, allow_pickle=True)
    data = {}
    for key in raw.files:
        value = raw[key]
        if value.dtype.kind in ("U", "S", "O"):
            data[key] = np.asarray(value).flatten()
        else:
            data[key] = np.asarray(value, dtype=float).flatten()
    return data


def fit_length(value, n, fill=0.0):
    """把 NPZ 原始字段转换为统一长度和类型，便于指标计算。"""
    arr = np.asarray(value).flatten()
    if len(arr) == n:
        return arr
    if len(arr) == 0:
        return np.full(n, fill, dtype=float)
    if len(arr) == 1:
        return np.full(n, arr[0], dtype=float)
    out = np.full(n, fill, dtype=float)
    m = min(n, len(arr))
    out[:m] = arr[:m]
    if m < n:
        out[m:] = arr[m - 1]
    return out


def numeric(data, key, n=None, fill=np.nan):
    """把 NPZ 原始字段转换为统一长度和类型，便于指标计算。"""
    if key not in data:
        if n is None:
            return np.array([], dtype=float)
        return np.full(n, fill, dtype=float)
    arr = np.asarray(data[key], dtype=float).flatten()
    if n is not None:
        arr = fit_length(arr, n, fill=fill)
    return arr


def text_values(data, key, n):
    """把 NPZ 原始字段转换为统一长度和类型，便于指标计算。"""
    if key not in data:
        return np.full(n, "", dtype=object)
    arr = np.asarray(data[key]).flatten().astype(object)
    if len(arr) == n:
        return arr
    return fit_length(arr, n, fill="")


def safe_mean(x):
    """在空数组或异常数据情况下仍能稳定返回结果的安全计算函数。"""
    x = np.asarray(x, dtype=float)
    return float(np.nanmean(x)) if len(x) and np.isfinite(x).any() else np.nan


def safe_std(x):
    """在空数组或异常数据情况下仍能稳定返回结果的安全计算函数。"""
    x = np.asarray(x, dtype=float)
    return float(np.nanstd(x)) if len(x) and np.isfinite(x).any() else np.nan


def safe_max(x):
    """在空数组或异常数据情况下仍能稳定返回结果的安全计算函数。"""
    x = np.asarray(x, dtype=float)
    return float(np.nanmax(x)) if len(x) and np.isfinite(x).any() else np.nan


def safe_rmse(x):
    """在空数组或异常数据情况下仍能稳定返回结果的安全计算函数。"""
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.nanmean(x * x))) if len(x) and np.isfinite(x).any() else np.nan


def safe_corr(a, b):
    """在空数组或异常数据情况下仍能稳定返回结果的安全计算函数。"""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    mask = np.isfinite(a) & np.isfinite(b)
    if np.count_nonzero(mask) < 3:
        return np.nan
    aa = a[mask]
    bb = b[mask]
    if np.std(aa) < 1e-12 or np.std(bb) < 1e-12:
        return np.nan
    return float(np.corrcoef(aa, bb)[0, 1])


def infer_strategy(path, data):
    """根据文件名或时序字段推断实验标签。"""
    if "arbitration_strategy" in data:
        values = np.asarray(data["arbitration_strategy"]).flatten()
        if len(values):
            text = str(values[0])
            if text and text != "nan":
                return text
    return path.stem.split("_t")[0]


def infer_zone_from_k(data, n):
    """根据文件名或时序字段推断实验标签。"""
    k = numeric(data, "K_env_true", n, fill=np.nan)
    if not np.isfinite(k).any():
        return np.full(n, -1.0)
    unique = np.unique(np.round(k[np.isfinite(k)], 6))
    if len(unique) <= 1:
        return np.zeros(n)
    mapping = {value: idx for idx, value in enumerate(sorted(unique))}
    return np.asarray([mapping.get(round(v, 6), -1) for v in k], dtype=float)


def add_derived_fields(data):
    """执行本模块中的辅助计算或数据转换步骤。"""
    if "t" not in data:
        raise KeyError("missing t")
    t = numeric(data, "t")
    n = len(t)
    if n == 0:
        raise ValueError("empty result")
    data["t"] = t

    keys = (
        "pos_x", "pos_y", "pos_z",
        "pos_des_x", "pos_des_y", "pos_des_z",
        "pos_err_x", "pos_err_y", "pos_err_z",
        "F_measured", "F_desired", "F_err",
        "alpha", "K_hat", "K_hat_raw", "K_env_true",
        "error_rcm", "error_track", "u_norm",
        "ori_err_deg", "front_axis_err_deg",
        "phase", "delta", "delta_dot",
    )
    for key in keys:
        if key in data:
            data[key] = numeric(data, key, n, fill=np.nan)

    if "F_err" not in data and "F_measured" in data and "F_desired" in data:
        data["F_err"] = numeric(data, "F_measured", n) - numeric(data, "F_desired", n)
    else:
        data["F_err"] = numeric(data, "F_err", n, fill=np.nan)

    for axis in ("x", "y", "z"):
        pos_key = f"pos_{axis}"
        des_key = f"pos_des_{axis}"
        err_key = f"pos_err_{axis}"
        if err_key not in data:
            if pos_key in data and des_key in data:
                data[err_key] = numeric(data, pos_key, n) - numeric(data, des_key, n)
            else:
                data[err_key] = np.zeros(n)

    data["pos_err_norm"] = np.sqrt(
        data["pos_err_x"] ** 2 + data["pos_err_y"] ** 2 + data["pos_err_z"] ** 2
    )
    data["tangent_err_norm"] = np.sqrt(data["pos_err_x"] ** 2 + data["pos_err_y"] ** 2)
    if "error_track" not in data:
        data["error_track"] = data["pos_err_norm"].copy()
    if "error_rcm" not in data:
        data["error_rcm"] = np.full(n, np.nan)
    if "alpha" not in data:
        data["alpha"] = np.full(n, np.nan)
    if "K_hat" not in data:
        data["K_hat"] = np.full(n, np.nan)
    if "K_env_true" not in data:
        data["K_env_true"] = np.full(n, np.nan)
    if "phase" not in data:
        data["phase"] = np.full(n, np.nan)

    data["stiffness_zone_index"] = infer_zone_from_k(data, n)
    data["stiffness_zone_label"] = np.asarray(
        [f"zone_{int(z)}" for z in data["stiffness_zone_index"]],
        dtype=object,
    )
    data["force_source"] = text_values(data, "force_source", n)
    return data


def result_files(input_path):
    """执行本模块中的辅助计算或数据转换步骤。"""
    path = Path(input_path)
    if path.is_file():
        return [path]
    files = sorted(
        p for p in path.rglob("*.npz")
        if p.is_file() and "approach_debug" not in str(p)
    )
    if not files:
        raise FileNotFoundError(f"{path} contains no scan npz files")
    return files


def compute_metrics(path, data, mask=None, zone_name="all"):
    """计算本模块对应的控制量、派生变量或实验评价指标。"""
    t = data["t"]
    if mask is None:
        mask = np.ones(len(t), dtype=bool)
    mask = np.asarray(mask, dtype=bool)
    if not np.any(mask):
        return None

    dt = float(np.nanmedian(np.diff(t))) if len(t) > 1 else 0.01
    force = data["F_measured"][mask]
    ferr = data["F_err"][mask]
    alpha = data["alpha"][mask]
    khat = data["K_hat"][mask]
    rcm = data["error_rcm"][mask]
    track = data["error_track"][mask]
    force_jitter = np.diff(force) if len(force) > 1 else np.array([np.nan])
    rcm_violation = np.isfinite(rcm) & (rcm > 0.0035)
    rcm_abort_margin = np.isfinite(rcm) & (rcm > 0.0065)

    return {
        "source_file": str(path),
        "strategy": infer_strategy(path, data),
        "zone": zone_name,
        "samples": int(np.count_nonzero(mask)),
        "duration_s": float(t[mask][-1] - t[mask][0]) if np.count_nonzero(mask) > 1 else 0.0,
        "force_rmse_N": safe_rmse(ferr),
        "force_peak_abs_err_N": safe_max(np.abs(ferr)),
        "force_jitter_N": safe_std(force_jitter),
        "force_mean_N": safe_mean(force),
        "rcm_rmse_mm": safe_rmse(rcm) * 1000.0,
        "rcm_peak_mm": safe_max(rcm) * 1000.0,
        "rcm_soft_violation_time_s": float(dt * np.count_nonzero(rcm_violation)),
        "rcm_pause_margin_time_s": float(dt * np.count_nonzero(rcm_abort_margin)),
        "track_rmse_mm": safe_rmse(track) * 1000.0,
        "track_peak_mm": safe_max(track) * 1000.0,
        "tangent_pos_rmse_mm": safe_rmse(data["tangent_err_norm"][mask]) * 1000.0,
        "position_rmse_mm": safe_rmse(data["pos_err_norm"][mask]) * 1000.0,
        "alpha_mean": safe_mean(alpha),
        "alpha_min": float(np.nanmin(alpha)) if np.isfinite(alpha).any() else np.nan,
        "alpha_max": float(np.nanmax(alpha)) if np.isfinite(alpha).any() else np.nan,
        "alpha_khat_corr": safe_corr(alpha, khat),
        "k_hat_mean_Npm": safe_mean(khat),
        "k_env_mean_Npm": safe_mean(data["K_env_true"][mask]),
        "u_rms": safe_rmse(data["u_norm"][mask]) if "u_norm" in data else np.nan,
        "ori_err_mean_deg": safe_mean(data["ori_err_deg"][mask]) if "ori_err_deg" in data else np.nan,
        "front_axis_err_mean_deg": safe_mean(data["front_axis_err_deg"][mask]) if "front_axis_err_deg" in data else np.nan,
    }


def write_csv(path, rows):
    """把分析结果、图表索引或时序数据写入磁盘文件。"""
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def aggregate_by_strategy(rows):
    """按策略或方法聚合多次实验指标。"""
    grouped = {}
    for row in rows:
        if row["zone"] == "all":
            grouped.setdefault(row["strategy"], []).append(row)
    out = []
    for strategy, items in grouped.items():
        summary = {"strategy": strategy, "trials": len(items)}
        numeric_keys = [
            key for key, value in items[0].items()
            if key not in ("source_file", "strategy", "zone") and isinstance(value, (int, float))
        ]
        for key in numeric_keys:
            vals = np.asarray([item[key] for item in items], dtype=float)
            summary[f"{key}_mean"] = safe_mean(vals)
            summary[f"{key}_std"] = safe_std(vals)
        out.append(summary)
    return sorted(out, key=lambda row: row["strategy"])


def plot_rcm_metric_bars(summary_rows, output_dir):
    """根据实验数据生成论文或调试用图表。"""
    if not summary_rows:
        return None
    strategies = [row["strategy"] for row in summary_rows]
    metrics = [
        ("rcm_rmse_mm_mean", "RCM RMSE (mm)"),
        ("rcm_peak_mm_mean", "RCM peak (mm)"),
        ("track_rmse_mm_mean", "Tool tracking RMSE (mm)"),
        ("force_rmse_N_mean", "Force RMSE (N)"),
        ("force_jitter_N_mean", "Force jitter (N)"),
        ("rcm_soft_violation_time_s_mean", "RCM soft violation (s)"),
    ]
    fig, axes = plt.subplots(len(metrics), 1, figsize=(11, 15), constrained_layout=True)
    x = np.arange(len(strategies))
    for ax, (key, label) in zip(axes, metrics):
        values = [row.get(key, np.nan) for row in summary_rows]
        ax.bar(x, values, color="#4C78A8", alpha=0.82)
        ax.set_ylabel(label)
        ax.set_xticks(x)
        ax.set_xticklabels(strategies, rotation=20, ha="right")
        ax.grid(True, axis="y", alpha=0.3)
    fig.suptitle("Chapter 3 RCM-constrained controller metrics")
    path = output_dir / "ch3_rcm_metric_bars.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def plot_representative(data_by_file, output_dir):
    """根据实验数据生成论文或调试用图表。"""
    if not data_by_file:
        return None
    path, data = max(data_by_file, key=lambda item: len(item[1]["t"]))
    t = data["t"]
    fig, axes = plt.subplots(5, 1, figsize=(12, 14), constrained_layout=True)

    ax = axes[0]
    ax.plot(t, data["F_measured"], label="measured")
    ax.plot(t, data["F_desired"], "--", label="desired")
    ax.set_ylabel("Force (N)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, data["error_rcm"] * 1000.0, label="RCM error")
    ax.axhline(3.5, color="tab:orange", linestyle=":", label="soft limit")
    ax.axhline(6.5, color="tab:red", linestyle=":", label="pause limit")
    ax.set_ylabel("RCM error (mm)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(t, data["error_track"] * 1000.0, label="tool track")
    ax.plot(t, data["tangent_err_norm"] * 1000.0, label="tangent")
    ax.set_ylabel("Position error (mm)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax.plot(t, data["alpha"], label="alpha_FP")
    ax2 = ax.twinx()
    ax2.plot(t, data["K_hat"], color="tab:orange", alpha=0.8, label="K_hat")
    if np.isfinite(data["K_env_true"]).any():
        ax2.plot(t, data["K_env_true"], color="tab:green", alpha=0.55, label="K_env")
    ax.set_ylabel("alpha")
    ax2.set_ylabel("stiffness (N/m)")
    ax.set_xlabel("Time (s)")
    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[4]
    if "ori_err_deg" in data and np.isfinite(data["ori_err_deg"]).any():
        ax.plot(t, data["ori_err_deg"], label="SO(3) orientation")
    if "front_axis_err_deg" in data and np.isfinite(data["front_axis_err_deg"]).any():
        ax.plot(t, data["front_axis_err_deg"], label="front axis")
    ax.axhline(3.0, color="tab:orange", linestyle=":", label="3 deg reference")
    ax.set_ylabel("Orientation error (deg)")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"Representative Chapter 3 RCM run: {path.name}")
    out = output_dir / "ch3_rcm_representative_timeseries.png"
    fig.savefig(out, dpi=200)
    plt.close(fig)
    return out


def write_report(path, summary_rows, all_rows, figure_paths):
    """把分析结果、图表索引或时序数据写入磁盘文件。"""
    valid = [r for r in summary_rows if math.isfinite(r.get("rcm_rmse_mm_mean", np.nan))]
    best_rcm = min(valid, key=lambda r: r["rcm_rmse_mm_mean"], default=None)
    best_force = min(
        (r for r in summary_rows if math.isfinite(r.get("force_rmse_N_mean", np.nan))),
        key=lambda r: r["force_rmse_N_mean"],
        default=None,
    )
    lines = [
        "# 第三章 RCM 约束 Gazebo/ROS2 实验分析报告",
        "",
        "本报告统计 RCM 约束扫描实验的穿刺点误差、工具跟踪误差、接触力误差和仲裁参数变化。"
        "数据来自 `run_with_rcm` 的 ROS2/Gazebo 在线控制日志。",
        "",
        "## 核心结论",
        "",
        f"- 分析试次数：{len([r for r in all_rows if r['zone'] == 'all'])}",
        f"- 策略：{', '.join(r['strategy'] for r in summary_rows) if summary_rows else '无'}",
    ]
    if best_rcm:
        lines.append(
            f"- RCM 均方误差最低：`{best_rcm['strategy']}`，"
            f"{best_rcm['rcm_rmse_mm_mean']:.3f} mm。"
        )
    if best_force:
        lines.append(
            f"- 力跟踪均方误差最低：`{best_force['strategy']}`，"
            f"{best_force['force_rmse_N_mean']:.4f} N。"
        )
    lines += [
        "",
        "## 图表",
        "",
    ]
    for fig_path in figure_paths:
        if fig_path:
            lines.append(f"![{fig_path.name}]({fig_path.name})")
            lines.append("")

    headers = [
        "strategy", "trials", "rcm_rmse_mm_mean", "rcm_peak_mm_mean",
        "track_rmse_mm_mean", "force_rmse_N_mean", "force_jitter_N_mean",
        "alpha_mean_mean", "alpha_min_mean", "alpha_max_mean",
        "ori_err_mean_deg_mean", "front_axis_err_mean_deg_mean",
    ]
    lines += ["## 汇总指标", ""]
    if summary_rows:
        lines.append("| " + " | ".join(headers) + " |")
        lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
        for row in summary_rows:
            values = []
            for key in headers:
                value = row.get(key, "")
                values.append(f"{value:.6g}" if isinstance(value, float) else str(value))
            lines.append("| " + " | ".join(values) + " |")

    lines += [
        "",
        "## 判据说明",
        "",
        "- RCM 软约束阈值按当前控制器配置取 `3.5 mm`；暂停/恢复边界按 `6.5 mm` 统计。",
        "- `track_rmse_mm` 来自 `compute_torque_with_rcm` 返回的工具跟踪误差。",
        "- `alpha_FP` 越大越偏位置/RCM 几何保持，越小越偏法向力跟踪。",
        "- 姿态误差和 front-axis 误差用于检查末端是否保持竖直向下、正面朝前，以及是否存在绕工具轴自转抖动。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    """命令行入口，解析参数并执行本模块对应的实验、绘图或分析流程。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="RCM result npz file or directory")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    del args.no_show

    files = result_files(args.input)
    output_dir = Path(args.output_dir) if args.output_dir else Path(args.input)
    if output_dir.is_file():
        output_dir = output_dir.parent / "analysis"
    else:
        output_dir = output_dir / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)

    all_rows = []
    data_by_file = []
    for file_path in files:
        data = add_derived_fields(flatten_npz(file_path))
        data_by_file.append((file_path, data))
        row = compute_metrics(file_path, data)
        if row:
            all_rows.append(row)
        for zone in np.unique(data["stiffness_zone_index"][np.isfinite(data["stiffness_zone_index"])]):
            mask = data["stiffness_zone_index"] == zone
            label = str(data["stiffness_zone_label"][mask][0]) if np.any(mask) else f"zone_{int(zone)}"
            zone_row = compute_metrics(file_path, data, mask=mask, zone_name=label)
            if zone_row:
                all_rows.append(zone_row)

    summary_rows = aggregate_by_strategy(all_rows)
    write_csv(output_dir / "ch3_rcm_trial_metrics.csv", all_rows)
    write_csv(output_dir / "ch3_rcm_strategy_summary.csv", summary_rows)
    figures = [
        plot_rcm_metric_bars(summary_rows, output_dir),
        plot_representative(data_by_file, output_dir),
    ]
    report_path = output_dir / "ch3_rcm_analysis_report.md"
    write_report(report_path, summary_rows, all_rows, figures)

    print(f"RCM 分析结果目录: {output_dir}")
    print(f"试次指标: {output_dir / 'ch3_rcm_trial_metrics.csv'}")
    print(f"策略汇总: {output_dir / 'ch3_rcm_strategy_summary.csv'}")
    print(f"报告: {report_path}")


if __name__ == "__main__":
    main()
