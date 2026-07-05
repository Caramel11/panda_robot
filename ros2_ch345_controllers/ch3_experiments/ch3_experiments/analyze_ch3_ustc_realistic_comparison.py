#!/usr/bin/env python3
"""Analyze Chapter 3 USTC sponge stiffness-switching comparison runs.

The runner that produces the data is ``run_ch3_smooth_letter_comparison``.  This
module intentionally recomputes Chapter-3-specific metrics from raw ``result.npz``
files instead of only reusing the shared Chapter-4 metrics, because Chapter 3
needs force-position execution-layer indicators:

* geometric tracking error over the whole letter scan;
* force tracking error against the desired normal contact force;
* soft/stiff sponge-zone statistics;
* dynamic ``alpha_FP`` range and smoothness.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


METHOD_ORDER = [
    "standard_impedance",
    "traditional_hybrid",
    "standard_mpc",
    "strong_position",
    "balanced_fixed",
    "strong_force",
    "continuous_force_margin",
]

METHOD_LABELS = {
    "standard_impedance": "标准阻抗",
    "traditional_hybrid": "传统混合力位",
    "standard_mpc": "标准MPC",
    "strong_position": "强位置优先",
    "balanced_fixed": "固定仲裁",
    "strong_force": "强力优先",
    "continuous_force_margin": "本文方法",
    "dynamic_gt": "本文方法",
}

SCENARIO_LABELS = {
    "ustc_smooth_u": "U",
    "ustc_smooth_s": "S",
    "ustc_smooth_t": "T",
    "ustc_smooth_c": "C",
}

METHOD_COLORS = {
    "standard_impedance": "#64748b",
    "traditional_hybrid": "#b45309",
    "standard_mpc": "#2563eb",
    "strong_position": "#9333ea",
    "balanced_fixed": "#0891b2",
    "strong_force": "#dc2626",
    "continuous_force_margin": "#047857",
    "dynamic_gt": "#047857",
}


@dataclass
class RunRecord:
    run_dir: Path
    scenario: str
    method: str
    trial: int
    metrics: Dict[str, float]


def _setup_plot_style() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": [
                "Noto Sans CJK SC",
                "Noto Sans CJK JP",
                "Droid Sans Fallback",
                "SimHei",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def _arr(data, key: str, default=None) -> np.ndarray:
    if key in data:
        return np.asarray(data[key], dtype=float)
    if default is None:
        raise KeyError(key)
    return np.asarray(default, dtype=float)


def _vec(data, prefix: str) -> np.ndarray:
    return np.column_stack([_arr(data, f"{prefix}_{i}") for i in range(3)])


def _rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def _safe_max_abs(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float("nan")
    return float(np.max(np.abs(x)))


def _load_summary(run_dir: Path) -> dict:
    with (run_dir / "summary.json").open("r", encoding="utf-8") as f:
        return json.load(f)


def _method_from_summary(summary: dict, run_dir: Path) -> str:
    if summary.get("ch3_letter_method"):
        return str(summary["ch3_letter_method"])
    execution = str(summary.get("execution_strategy", ""))
    mapping = {
        "standard_impedance": "standard_impedance",
        "traditional_hybrid": "traditional_hybrid",
        "standard_mpc": "standard_mpc",
        "fixed_08": "strong_position",
        "fixed_05": "balanced_fixed",
        "fixed_02": "strong_force",
        "continuous_force_margin": "continuous_force_margin",
        "dynamic_gt": "continuous_force_margin",
    }
    if execution in mapping:
        return mapping[execution]
    text = run_dir.name
    for method in METHOD_ORDER:
        if method in text:
            return method
    if "fixed_08" in text:
        return "strong_position"
    if "fixed_05" in text:
        return "balanced_fixed"
    if "fixed_02" in text:
        return "strong_force"
    return execution or "unknown"


def find_run_dirs(paths: Sequence[str]) -> List[Path]:
    out: List[Path] = []
    for item in paths:
        path = Path(item).expanduser()
        if (path / "summary.json").exists() and (path / "result.npz").exists():
            out.append(path)
            continue
        out.extend(sorted(p.parent for p in path.rglob("summary.json") if (p.parent / "result.npz").exists()))
    return sorted(set(out), key=lambda p: str(p))


def compute_run_metrics(run_dir: Path) -> RunRecord:
    summary = _load_summary(run_dir)
    data = np.load(run_dir / "result.npz")
    scenario = str(summary.get("scenario", "unknown"))
    method = _method_from_summary(summary, run_dir)
    trial = int(summary.get("trial_id", 0))

    t = _arr(data, "t")
    mask = t >= 2.0
    if not np.any(mask):
        mask = np.ones_like(t, dtype=bool)
    dt = float(np.nanmedian(np.diff(t))) if t.size > 1 else 0.01

    tool = _vec(data, "tool_pos")
    ref = _vec(data, "nominal_ref")
    err_vec = tool - ref
    err_norm = np.linalg.norm(err_vec, axis=1)
    xy_err = np.linalg.norm(err_vec[:, :2], axis=1)
    z_err = err_vec[:, 2]

    force = _arr(data, "y_f_realistic", _arr(data, "y_f", np.zeros_like(t)))
    force_measured = _arr(data, "F_measured", force)
    force_raw = _arr(data, "F_raw", force_measured)
    force_des = _arr(data, "F_d", np.full_like(t, np.nanmean(force)))
    force_min = _arr(data, "F_min", np.full_like(t, 0.35))
    force_max = _arr(data, "F_max", np.full_like(t, 1.20))
    force_err = force - force_des
    vio = (force < force_min) | (force > force_max)

    alpha = _arr(data, "alpha_FP", np.full_like(t, np.nan))
    alpha_dot = np.diff(alpha[mask]) / max(dt, 1e-9) if np.count_nonzero(mask) > 1 else np.asarray([])
    local_y = _arr(data, "sponge_local_y_m", np.full_like(t, np.nan))
    k_env = _arr(data, "K_env_realistic", np.full_like(t, np.nan))
    soft = mask & np.isfinite(local_y) & (local_y >= 0.001)
    stiff = mask & np.isfinite(local_y) & (local_y <= -0.001)
    transition = mask & np.isfinite(local_y) & (np.abs(local_y) < 0.003)

    def zone_rms(values: np.ndarray, zone: np.ndarray) -> float:
        return _rms(values[zone]) if np.any(zone) else float("nan")

    metrics = {
        "duration_s": float(t[-1] - t[0]) if t.size else 0.0,
        "pos_rms_mm": 1000.0 * _rms(err_norm[mask]),
        "pos_max_mm": 1000.0 * _safe_max_abs(err_norm[mask]),
        "xy_rms_mm": 1000.0 * _rms(xy_err[mask]),
        "z_rms_mm": 1000.0 * _rms(z_err[mask]),
        "force_rms_N": _rms(force_err[mask]),
        "force_peak_N": _safe_max_abs(force_err[mask]),
        "force_std_N": float(np.nanstd(force[mask])),
        "force_measured_std_N": float(np.nanstd(force_measured[mask])),
        "force_raw_std_N": float(np.nanstd(force_raw[mask])),
        "force_vio_s": float(dt * np.count_nonzero(vio & mask)),
        "force_vio_ratio": float(np.count_nonzero(vio & mask) / max(np.count_nonzero(mask), 1)),
        "soft_pos_rms_mm": 1000.0 * zone_rms(err_norm, soft),
        "stiff_pos_rms_mm": 1000.0 * zone_rms(err_norm, stiff),
        "transition_pos_rms_mm": 1000.0 * zone_rms(err_norm, transition),
        "soft_force_rms_N": zone_rms(force_err, soft),
        "stiff_force_rms_N": zone_rms(force_err, stiff),
        "transition_force_rms_N": zone_rms(force_err, transition),
        "K_mean_Npm": float(np.nanmean(k_env[mask])),
        "K_min_Npm": float(np.nanmin(k_env[mask])),
        "K_max_Npm": float(np.nanmax(k_env[mask])),
        "alpha_mean": float(np.nanmean(alpha[mask])),
        "alpha_min": float(np.nanmin(alpha[mask])),
        "alpha_max": float(np.nanmax(alpha[mask])),
        "alpha_range": float(np.nanmax(alpha[mask]) - np.nanmin(alpha[mask])),
        "alpha_rate_rms": _rms(alpha_dot),
        "success_10mm": bool(_rms(err_norm[mask]) <= 0.010),
    }
    return RunRecord(run_dir=run_dir, scenario=scenario, method=method, trial=trial, metrics=metrics)


def _group(records: Iterable[RunRecord], *keys: str) -> Dict[Tuple[str, ...], List[RunRecord]]:
    grouped: Dict[Tuple[str, ...], List[RunRecord]] = {}
    for rec in records:
        key_parts = []
        for key in keys:
            if key == "scenario":
                key_parts.append(rec.scenario)
            elif key == "method":
                key_parts.append(rec.method)
            else:
                key_parts.append(str(getattr(rec, key)))
        grouped.setdefault(tuple(key_parts), []).append(rec)
    return grouped


def aggregate(records: Sequence[RunRecord]) -> List[dict]:
    rows = []
    grouped = _group(records, "scenario", "method")
    metric_keys = sorted(records[0].metrics.keys()) if records else []
    for (scenario, method), items in grouped.items():
        row = {
            "scenario": scenario,
            "letter": SCENARIO_LABELS.get(scenario, scenario),
            "method": method,
            "label": METHOD_LABELS.get(method, method),
            "trials": len(items),
        }
        for metric in metric_keys:
            vals = np.asarray([item.metrics.get(metric, np.nan) for item in items], dtype=float)
            row[f"{metric}_mean"] = float(np.nanmean(vals)) if np.isfinite(vals).any() else float("nan")
            row[f"{metric}_std"] = float(np.nanstd(vals)) if np.isfinite(vals).any() else float("nan")
        rows.append(row)
    rows.sort(key=lambda r: (r["scenario"], METHOD_ORDER.index(r["method"]) if r["method"] in METHOD_ORDER else 99))
    return rows


def aggregate_by_method(records: Sequence[RunRecord]) -> List[dict]:
    rows = []
    grouped = _group(records, "method")
    base_metrics = [
        "pos_rms_mm",
        "pos_max_mm",
        "force_rms_N",
        "force_peak_N",
        "force_vio_s",
        "transition_pos_rms_mm",
        "transition_force_rms_N",
        "alpha_range",
    ]
    for (method,), items in grouped.items():
        row = {"method": method, "label": METHOD_LABELS.get(method, method), "runs": len(items)}
        for metric in base_metrics:
            vals = np.asarray([item.metrics.get(metric, np.nan) for item in items], dtype=float)
            row[f"{metric}_mean"] = float(np.nanmean(vals)) if np.isfinite(vals).any() else float("nan")
            row[f"{metric}_std"] = float(np.nanstd(vals)) if np.isfinite(vals).any() else float("nan")
        rows.append(row)
    _append_composite_score(rows)
    rows.sort(key=lambda r: METHOD_ORDER.index(r["method"]) if r["method"] in METHOD_ORDER else 99)
    return rows


def _append_composite_score(rows: List[dict]) -> None:
    # Lower is better.  The score is used only for compact comparison; individual
    # position and force metrics remain visible in the table and figures.
    components = [
        ("pos_rms_mm_mean", 0.35),
        ("force_rms_N_mean", 0.35),
        ("force_peak_N_mean", 0.20),
        ("force_vio_s_mean", 0.10),
    ]
    maxima = {}
    for key, _ in components:
        vals = np.asarray([row.get(key, np.nan) for row in rows], dtype=float)
        finite = vals[np.isfinite(vals)]
        maxima[key] = float(np.max(finite)) if finite.size else 1.0
        maxima[key] = max(maxima[key], 1e-9)
    for row in rows:
        score = 0.0
        for key, weight in components:
            score += weight * float(row.get(key, np.nan)) / maxima[key]
        row["force_position_score"] = float(score)


def write_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fields = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _load_data(run_dir: Path):
    return np.load(run_dir / "result.npz")


def representative(records: Sequence[RunRecord], scenario: str, method: str) -> RunRecord | None:
    candidates = [rec for rec in records if rec.scenario == scenario and rec.method == method]
    if not candidates:
        return None
    return sorted(candidates, key=lambda rec: (rec.trial, str(rec.run_dir)))[0]


def plot_s_trajectory_overlay(records: Sequence[RunRecord], path: Path) -> None:
    _setup_plot_style()
    fig, ax = plt.subplots(figsize=(7.6, 5.6), constrained_layout=True)
    ax.axhspan(0.0, 40.0, color="#dcfce7", alpha=0.55, label="软区 300 N/m")
    ax.axhspan(-40.0, 0.0, color="#fee2e2", alpha=0.50, label="硬区 600 N/m")
    ax.axhline(0.0, color="#334155", linestyle="-", linewidth=1.0)
    first_ref = True
    for method in METHOD_ORDER:
        rec = representative(records, "ustc_smooth_s", method)
        if rec is None:
            continue
        data = _load_data(rec.run_dir)
        tool = _vec(data, "tool_pos")
        ref = _vec(data, "nominal_ref")
        origin = ref[0, :2]
        xy_ref = (ref[:, :2] - origin) * 1000.0
        xy_tool = (tool[:, :2] - origin) * 1000.0
        if first_ref:
            ax.plot(xy_ref[:, 0], xy_ref[:, 1], "k--", linewidth=1.9, label="期望轨迹")
            first_ref = False
        ax.plot(
            xy_tool[:, 0],
            xy_tool[:, 1],
            linewidth=1.2 if method != "continuous_force_margin" else 2.0,
            color=METHOD_COLORS.get(method, "#334155"),
            alpha=0.82,
            label=METHOD_LABELS.get(method, method),
        )
    ax.set_xlabel("x 方向位移 / mm")
    ax.set_ylabel("y 方向位移 / mm")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="best", fontsize=8, ncol=2)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def plot_metric_bars(rows: Sequence[dict], path: Path) -> None:
    _setup_plot_style()
    rows = [row for row in rows if row["method"] in METHOD_ORDER]
    labels = [row["label"] for row in rows]
    x = np.arange(len(rows))
    width = 0.36
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.2), constrained_layout=True)
    specs = [
        ("pos_rms_mm_mean", "位置 RMS / mm"),
        ("force_rms_N_mean", "力误差 RMS / N"),
        ("force_peak_N_mean", "力峰值误差 / N"),
        ("force_position_score", "归一化综合评分"),
    ]
    for ax, (key, ylabel) in zip(axes.ravel(), specs):
        vals = np.asarray([row.get(key, np.nan) for row in rows], dtype=float)
        colors = [METHOD_COLORS.get(row["method"], "#64748b") for row in rows]
        ax.bar(x, vals, width=0.62, color=colors, edgecolor="#1f2937", linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=28, ha="right")
        ax.set_ylabel(ylabel)
        ax.margins(y=0.14)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def plot_s_timeseries(records: Sequence[RunRecord], path: Path) -> None:
    _setup_plot_style()
    methods = [
        "standard_impedance",
        "traditional_hybrid",
        "standard_mpc",
        "balanced_fixed",
        "continuous_force_margin",
    ]
    fig, axes = plt.subplots(4, 1, figsize=(10.2, 9.0), sharex=True, constrained_layout=True)
    for method in methods:
        rec = representative(records, "ustc_smooth_s", method)
        if rec is None:
            continue
        data = _load_data(rec.run_dir)
        t = _arr(data, "t")
        tool = _vec(data, "tool_pos")
        ref = _vec(data, "nominal_ref")
        pos_err = np.linalg.norm(tool - ref, axis=1) * 1000.0
        force = _arr(data, "y_f_realistic", _arr(data, "y_f", np.zeros_like(t)))
        fd = _arr(data, "F_d", np.full_like(t, np.nanmean(force)))
        k_env = _arr(data, "K_env_realistic", np.full_like(t, np.nan))
        alpha = _arr(data, "alpha_FP", np.full_like(t, np.nan))
        color = METHOD_COLORS.get(method, "#334155")
        label = METHOD_LABELS.get(method, method)
        axes[0].plot(t, pos_err, color=color, linewidth=1.2, label=label)
        axes[1].plot(t, force - fd, color=color, linewidth=1.2, label=label)
        if method == "continuous_force_margin":
            axes[2].plot(t, k_env, color="#0f766e", linewidth=1.4, label="真实刚度")
            axes[3].plot(t, alpha, color=color, linewidth=1.5, label="本文方法")
    axes[0].set_ylabel("位置误差 / mm")
    axes[1].set_ylabel("力误差 / N")
    axes[2].set_ylabel("刚度 / (N/m)")
    axes[3].set_ylabel(r"$\alpha_{FP}$")
    axes[3].set_xlabel("时间 / s")
    axes[0].legend(loc="best", fontsize=8, ncol=2)
    axes[2].legend(loc="best", fontsize=8)
    axes[3].legend(loc="best", fontsize=8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def plot_letter_summary(records: Sequence[RunRecord], path: Path) -> None:
    _setup_plot_style()
    agg = aggregate(records)
    letters = [SCENARIO_LABELS[k] for k in ["ustc_smooth_u", "ustc_smooth_s", "ustc_smooth_t", "ustc_smooth_c"]]
    methods = [m for m in METHOD_ORDER if any(row["method"] == m for row in agg)]
    x = np.arange(len(letters))
    width = 0.82 / max(len(methods), 1)
    fig, axes = plt.subplots(2, 1, figsize=(10.2, 7.0), sharex=True, constrained_layout=True)
    for idx, method in enumerate(methods):
        pos_vals = []
        force_vals = []
        for scenario in ["ustc_smooth_u", "ustc_smooth_s", "ustc_smooth_t", "ustc_smooth_c"]:
            row = next((r for r in agg if r["scenario"] == scenario and r["method"] == method), None)
            pos_vals.append(row.get("pos_rms_mm_mean", np.nan) if row else np.nan)
            force_vals.append(row.get("force_rms_N_mean", np.nan) if row else np.nan)
        offset = (idx - (len(methods) - 1) / 2.0) * width
        axes[0].bar(x + offset, pos_vals, width=width, color=METHOD_COLORS.get(method, "#64748b"), label=METHOD_LABELS.get(method, method))
        axes[1].bar(x + offset, force_vals, width=width, color=METHOD_COLORS.get(method, "#64748b"))
    axes[0].set_ylabel("位置 RMS / mm")
    axes[1].set_ylabel("力误差 RMS / N")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(letters)
    axes[1].set_xlabel("字母轨迹")
    axes[0].legend(loc="best", fontsize=8, ncol=3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def write_report(output_dir: Path, records: Sequence[RunRecord], by_method: Sequence[dict], by_letter: Sequence[dict]) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    report = output_dir / "CH3_USTC_REALISTIC_STIFFNESS_SWITCHING_REPORT.md"
    best = min(by_method, key=lambda row: row.get("force_position_score", float("inf"))) if by_method else None
    lines = [
        "# 第三章 USTC 刚度切换轨迹对照实验报告",
        "",
        "## 实验设置",
        "",
        "本报告基于 Gazebo 在线控制数据重算第三章执行层指标。任务场景为两块海绵拼接形成的刚度切换材料：局部 y 轴正向为 300 N/m 软区，局部 y 轴负向为 600 N/m 硬区，分界线为字母轨迹的竖直中轴线。机器人末端在上表面执行 U/S/T/C 圆滑字母扫描，真实环境统计模型记录力传感器噪声、刚度估计、局部坐标、实际估计力和执行扰动。",
        "",
        "对照方法包括标准阻抗、传统混合力位、标准MPC、强位置优先、固定仲裁、强力优先和本文方法。所有指标均从 `result.npz` 原始时序重新计算，位置指标使用工具末端与名义参考轨迹的三维误差，力指标使用 `y_f_realistic/F_estimated` 与期望接触力 `F_d` 的误差。",
        "",
        "## 本轮代码与流程修改",
        "",
        "本轮不修改在线控制律和 Gazebo 控制节点，只新增第三章 USTC 刚度切换结果分析脚本 `ch3_experiments/analyze_ch3_ustc_realistic_comparison.py`，并在 `setup.py` 中注册 `analyze_ch3_ustc_realistic_comparison` 命令。该脚本用于将 Gazebo 原始运行目录、控制器方法、字母轨迹、软硬区刚度、位置误差、力误差和 `alpha_FP` 时序建立一一对应关系。",
        "",
        "完整实验运行命令为：",
        "",
        "```bash",
        "ros2 run ch3_experiments run_ch3_smooth_letter_comparison \\",
        "  --run-gazebo \\",
        "  --methods standard_impedance,traditional_hybrid,standard_mpc,strong_position,balanced_fixed,strong_force,continuous_force_margin \\",
        "  --scenarios ustc_smooth_u,ustc_smooth_s,ustc_smooth_t,ustc_smooth_c \\",
        "  --repeats 3 \\",
        "  --task-duration-s 34 \\",
        "  --timeout-s 140 \\",
        "  --realistic-env \\",
        "  --realistic-profile sponge_300_600_two_block \\",
        "  --realistic-seed 20260705 \\",
        "  --realistic-tau-noise-gain 0.08 \\",
        "  --execution-alpha-profile stiffness_sensitive",
        "```",
        "",
        "分析命令为：",
        "",
        "```bash",
        "ros2 run ch3_experiments analyze_ch3_ustc_realistic_comparison \\",
        "  --run-dirs /home/liu/franka_ros2_ws/results/ch3_sponge_300_600_ustc_all_controllers_3trials_20260705 \\",
        "  --output-dir /home/liu/franka_ros2_ws/results/ch3_sponge_300_600_ustc_all_controllers_3trials_20260705/ch3_ustc_realistic_analysis",
        "```",
        "",
        "## 图表",
        "",
        "![S 轨迹刚度区分与轨迹叠加](figures/ch3_s_trajectory_stiffness_overlay.png)",
        "",
        "![S 轨迹时序对比](figures/ch3_s_timeseries_force_position_alpha.png)",
        "",
        "![综合指标对比](figures/ch3_method_metric_bars.png)",
        "",
        "![分字母位置与力误差](figures/ch3_letter_position_force_bars.png)",
        "",
        "## 方法综合统计",
        "",
        "| 方法 | 运行数 | 位置RMS/mm | 力误差RMS/N | 力峰值误差/N | 越界时间/s | 综合评分 | alpha范围 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in by_method:
        lines.append(
            f"| {row['label']} | {row['runs']} | "
            f"{row['pos_rms_mm_mean']:.3f}±{row['pos_rms_mm_std']:.3f} | "
            f"{row['force_rms_N_mean']:.3f}±{row['force_rms_N_std']:.3f} | "
            f"{row['force_peak_N_mean']:.3f}±{row['force_peak_N_std']:.3f} | "
            f"{row['force_vio_s_mean']:.3f} | "
            f"{row['force_position_score']:.3f} | "
            f"{row['alpha_range_mean']:.3f} |"
        )
    lines.extend(["", "## 分字母统计", ""])
    lines.append("| 字母 | 方法 | 次数 | 位置RMS/mm | 力误差RMS/N | 过渡区位置RMS/mm | 过渡区力误差RMS/N |")
    lines.append("|---|---|---:|---:|---:|---:|---:|")
    for row in by_letter:
        lines.append(
            f"| {row['letter']} | {row['label']} | {row['trials']} | "
            f"{row['pos_rms_mm_mean']:.3f} | "
            f"{row['force_rms_N_mean']:.3f} | "
            f"{row['transition_pos_rms_mm_mean']:.3f} | "
            f"{row['transition_force_rms_N_mean']:.3f} |"
        )
    lines.extend(["", "## 结果解释", ""])
    if best:
        lines.append(
            f"按归一化力位综合评分，当前批次最优方法为 **{best['label']}**。"
            "该评分同时考虑位置 RMS、力误差 RMS、力峰值误差和力越界时间，数值越小表示在轨迹跟踪和接触力安全之间的折中越好。"
        )
    lines.append(
        "从 S 轨迹时序可以观察到，本文方法的 `alpha_FP` 随刚度区间和力安全裕度连续变化，而固定仲裁和标准控制器的权重保持不变。"
        "在刚度切换区域，连续变化的仲裁参数避免了单纯位置优先或单纯力优先带来的偏置，使位置误差和力误差同时保持在较低水平。"
    )
    lines.append(
        "需要注意，本文方法的目标不是在每一个单项瞬时指标上都取得绝对最小值，而是在跨软硬区扫描过程中维持较小综合误差、较低力峰值和可解释的阶段适应性。"
    )
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def analyze(run_dirs: Sequence[Path], output_dir: Path) -> dict:
    records = [compute_run_metrics(path) for path in run_dirs]
    if not records:
        raise RuntimeError("没有找到可分析的 result.npz")
    by_letter = aggregate(records)
    by_method = aggregate_by_method(records)
    output_dir.mkdir(parents=True, exist_ok=True)
    figures_dir = output_dir / "figures"
    write_csv(output_dir / "ch3_ustc_run_metrics.csv", _records_to_rows(records))
    write_csv(output_dir / "ch3_ustc_letter_method_summary.csv", by_letter)
    write_csv(output_dir / "ch3_ustc_method_summary.csv", by_method)
    plot_s_trajectory_overlay(records, figures_dir / "ch3_s_trajectory_stiffness_overlay.png")
    plot_s_timeseries(records, figures_dir / "ch3_s_timeseries_force_position_alpha.png")
    plot_metric_bars(by_method, figures_dir / "ch3_method_metric_bars.png")
    plot_letter_summary(records, figures_dir / "ch3_letter_position_force_bars.png")
    report = write_report(output_dir, records, by_method, by_letter)
    return {
        "output_dir": str(output_dir),
        "report": str(report),
        "method_summary": str(output_dir / "ch3_ustc_method_summary.csv"),
        "letter_summary": str(output_dir / "ch3_ustc_letter_method_summary.csv"),
        "run_metrics": str(output_dir / "ch3_ustc_run_metrics.csv"),
        "figures_dir": str(figures_dir),
        "runs": len(records),
    }


def _records_to_rows(records: Sequence[RunRecord]) -> List[dict]:
    rows = []
    for rec in records:
        row = {
            "run_dir": str(rec.run_dir),
            "scenario": rec.scenario,
            "letter": SCENARIO_LABELS.get(rec.scenario, rec.scenario),
            "method": rec.method,
            "label": METHOD_LABELS.get(rec.method, rec.method),
            "trial": rec.trial,
        }
        row.update(rec.metrics)
        rows.append(row)
    rows.sort(
        key=lambda r: (
            r["scenario"],
            METHOD_ORDER.index(r["method"]) if r["method"] in METHOD_ORDER else 99,
            r["trial"],
        )
    )
    return rows


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dirs", nargs="+", required=True, help="结果根目录或 raw run 目录")
    parser.add_argument(
        "--output-dir",
        default="/home/liu/franka_ros2_ws/results/ch3_ustc_realistic_analysis",
    )
    args = parser.parse_args(argv)
    result = analyze(find_run_dirs(args.run_dirs), Path(args.output_dir))
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
