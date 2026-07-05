#!/usr/bin/env python3
"""Chapter 5 force-violation-time challenge.

This module fills the evidence gap around ``T_vio`` in Chapter 5.  The normal
mixed-sequence experiment can leave all methods with zero force-bound violation
time, which makes the metric uninformative.  This script creates a small
dedicated unsafe-normal-push challenge and evaluates whether the proposed
reference-layer safety projection reduces force-bound violation time while the
closed-loop tracking error still converges.

Two execution modes are provided:

``--run-gazebo``
    Calls the existing Chapter 5 Gazebo suite with scenario
    ``tvio_stress_push``.  The actual ROS2 controller remains the Chapter 4
    Gazebo controller with the Chapter 3 execution-layer switch.

``--dry-run``
    Generates deterministic result files with the same ``result.npz`` and
    ``summary.json`` structure as the Gazebo controller.  This is only for
    checking the metric, plotting, and report pipeline before launching Gazebo.
"""

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .ch5_metrics import load_run, write_metrics_csv
from .method_switches import resolve_methods
from .run_ch5_gazebo_suite import main as run_gazebo_suite_main


DEFAULT_METHODS = "direct_accept,balanced_fixed,no_projection,reference_only,full_method"
SCENARIO = "tvio_stress_push"


def _smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _method_properties(method_name):
    """Return reference-layer behavior for the deterministic dry-run model."""

    if method_name == "direct_accept":
        return {"alpha_hr": 1.0, "projection": False, "kappa": False, "alpha_fp_dynamic": False}
    if method_name == "standard_impedance":
        return {"alpha_hr": 0.72, "projection": True, "kappa": True, "alpha_fp_dynamic": False}
    if method_name == "traditional_hybrid":
        return {"alpha_hr": 0.72, "projection": True, "kappa": True, "alpha_fp_dynamic": False}
    if method_name == "standard_mpc":
        return {"alpha_hr": 0.72, "projection": True, "kappa": True, "alpha_fp_dynamic": False}
    if method_name == "balanced_fixed":
        return {"alpha_hr": 0.5, "projection": False, "kappa": False, "alpha_fp_dynamic": False}
    if method_name == "no_projection":
        return {"alpha_hr": 0.72, "projection": False, "kappa": False, "alpha_fp_dynamic": True}
    if method_name == "reference_only":
        return {"alpha_hr": 0.72, "projection": True, "kappa": True, "alpha_fp_dynamic": False}
    if method_name == "full_method":
        return {"alpha_hr": 0.78, "projection": True, "kappa": True, "alpha_fp_dynamic": True}
    return {"alpha_hr": 0.5, "projection": False, "kappa": False, "alpha_fp_dynamic": False}


def _human_delta(t):
    """Unsafe normal input used by the T_vio challenge."""

    delta = np.zeros((len(t), 3), dtype=float)
    mask = (t >= 6.0) & (t <= 14.0)
    rise = _smoothstep((t[mask] - 6.0) / 2.0)
    fall = np.ones_like(rise)
    late = t[mask] >= 12.5
    fall[late] = 1.0 - _smoothstep((t[mask][late] - 12.5) / 1.5)
    amp = 0.014 * np.minimum(rise, fall)
    delta[mask, 1] += 0.004 * amp / 0.014
    delta[mask, 2] -= amp
    rec = (t >= 14.5) & (t <= 16.0)
    delta[rec, 1] += 0.006 * _smoothstep((t[rec] - 14.5) / 1.5)
    return delta


def _write_vector_fields(out, prefix, value):
    out[f"{prefix}_0"] = value[:, 0]
    out[f"{prefix}_1"] = value[:, 1]
    out[f"{prefix}_2"] = value[:, 2]


def _generate_dry_run(
    method_name,
    run_dir,
    trial_id,
    duration_s=22.0,
    dt=0.01,
    force_min=0.38,
    force_desired=0.70,
    force_max=1.05,
    force_margin=0.06,
    contact_stiffness_hat=70.0,
):
    """Generate a deterministic result.npz compatible with ch5_metrics."""

    props = _method_properties(method_name)
    t = np.arange(0.0, duration_s + 0.5 * dt, dt)
    n = len(t)
    f_min, f_d, f_max = force_min, force_desired, force_max
    k_hat = contact_stiffness_hat
    n_down = np.array([0.0, 0.0, -1.0])

    nominal = np.zeros((n, 3), dtype=float)
    nominal[:, 0] = 0.10 * t / duration_s
    nominal[:, 1] = 0.008 * np.sin(2.0 * np.pi * t / duration_s)
    nominal[:, 2] = 0.0
    h_delta = _human_delta(t)
    x_h = nominal + h_delta

    normal_raw = h_delta @ n_down
    tangent = h_delta - normal_raw[:, None] * n_down
    max_down = max(0.0, (f_max - force_margin - f_d) / k_hat)
    max_up = max(0.0, (f_d - (f_min + force_margin)) / k_hat)
    normal_safe = np.clip(normal_raw, -max_up, max_down)
    x_h_safe = nominal + tangent + normal_safe[:, None] * n_down

    y_candidate = f_d + k_hat * normal_raw
    y_safe = f_d + k_hat * normal_safe
    rho_candidate = np.clip(np.minimum(y_candidate - f_min, f_max - y_candidate) / (0.5 * (f_max - f_min)), 0.0, 1.0)
    kappa = np.clip(rho_candidate, 0.05, 1.0)
    alpha = np.full(n, props["alpha_hr"], dtype=float)

    if props["projection"]:
        delta_safe = x_h_safe - nominal
        normal_delta = delta_safe @ n_down
        tangent_delta = delta_safe - normal_delta[:, None] * n_down
        normal_gain = kappa if props["kappa"] else np.ones_like(kappa)
        tool_ref = nominal + alpha[:, None] * tangent_delta + (alpha * normal_gain)[:, None] * normal_delta[:, None] * n_down
        x_h_safe_used = x_h_safe
    else:
        tool_ref = nominal + alpha[:, None] * h_delta
        x_h_safe_used = x_h

    y_f = f_d + k_hat * ((tool_ref - nominal) @ n_down)
    rho_final = np.clip(np.minimum(y_f - f_min, f_max - y_f) / (0.5 * (f_max - f_min)), 0.0, 1.0)
    force_err = np.abs(y_f - f_d) / max(f_max - f_min, 1e-9)
    if props["alpha_fp_dynamic"]:
        alpha_fp_target = np.clip(0.75 * rho_final + 0.25 * (1.0 - force_err), 0.15, 0.85)
    else:
        alpha_fp_target = np.full(n, 0.5, dtype=float)
    alpha_fp = np.empty(n, dtype=float)
    alpha_fp[0] = 0.5
    tau = 0.22
    for i in range(1, n):
        alpha_fp[i] = alpha_fp[i - 1] + dt * (alpha_fp_target[i] - alpha_fp[i - 1]) / tau

    # First-order execution lag.  The controller tracks the final reference;
    # unsafe baselines may still have low tracking error, but their force proxy
    # crosses the safety bound.
    tool_pos = np.empty_like(tool_ref)
    tool_pos[0] = nominal[0]
    lag = 0.10 if method_name in ("full_method", "reference_only") else 0.14
    for i in range(1, n):
        tool_pos[i] = tool_pos[i - 1] + dt * (tool_ref[i] - tool_pos[i - 1]) / lag
    tracking_error = np.linalg.norm(tool_pos - tool_ref, axis=1)

    out = {
        "t": t,
        "tracking_error": tracking_error,
        "rcm_error": np.zeros(n),
        "alpha": alpha,
        "alpha_FP": alpha_fp,
        "alpha_FP_target": alpha_fp_target,
        "rho_F": rho_final,
        "kappa_N": kappa,
        "beta": np.where(np.linalg.norm(h_delta, axis=1) > 1e-6, 1.0, 0.0),
        "y_f": y_f,
        "y_f_candidate": y_candidate,
        "y_f_safe": y_safe,
        "F_min": np.full(n, f_min),
        "F_d": np.full(n, f_d),
        "F_max": np.full(n, f_max),
        "normal_raw": normal_raw,
        "normal_safe": normal_safe,
        "F_h": np.linalg.norm(h_delta, axis=1) / 0.010,
        "D_r": 0.10 * rho_final,
        "D_c": 1.0 - rho_final,
        "I_h": np.linalg.norm(h_delta, axis=1) / 0.010,
        "T_h": np.cumsum(np.linalg.norm(h_delta, axis=1) > 1e-6) * dt,
        "C_h": np.zeros(n),
        "human_button": np.where(np.linalg.norm(h_delta, axis=1) > 1e-6, 1.0, 0.0),
        "human_force_cmd_N": np.zeros(n),
    }
    for prefix, value in [
        ("nominal_ref", nominal),
        ("x_h", x_h),
        ("x_h_safe", x_h_safe_used),
        ("tool_ref", tool_ref),
        ("tool_pos", tool_pos),
        ("human_ref", x_h_safe_used),
        ("x_tele", x_h),
    ]:
        _write_vector_fields(out, prefix, value)

    run_dir.mkdir(parents=True, exist_ok=True)
    np.savez(run_dir / "result.npz", **out)
    summary = {
        "task_mode": "no_rcm",
        "controller_variant": "dry_run_ch5_tvio",
        "arbitration_strategy": resolve_methods([method_name])[0].arbitration_strategy,
        "execution_strategy": resolve_methods([method_name])[0].execution_strategy,
        "scenario": SCENARIO,
        "ch5_method": method_name,
        "trial_id": trial_id,
        "target_tolerance_m": 0.003,
        "dry_run": True,
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return run_dir


def _read_metrics(path):
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _float(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def _aggregate_rows(rows):
    """Aggregate repeated T_vio challenge trials by method."""

    groups = defaultdict(list)
    order = []
    for row in rows:
        method = row["method"]
        if method not in groups:
            order.append(method)
        groups[method].append(row)

    out = []
    metric_keys = [
        "tracking_last_rms_m",
        "T_vio_s",
        "F_peak_N",
        "F_max_observed_N",
        "R_acc",
        "R_sup",
        "alpha_FP_min",
        "score",
    ]
    for method in order:
        items = groups[method]
        row = {"method": method, "n": len(items)}
        row["success_count"] = sum(str(item.get("success", "")).lower() == "true" for item in items)
        row["success_rate"] = row["success_count"] / max(len(items), 1)
        for key in metric_keys:
            values = np.asarray([_float(item, key) for item in items], dtype=float)
            row[f"{key}_mean"] = float(np.mean(values))
            row[f"{key}_std"] = float(np.std(values))
        out.append(row)
    return out


def _write_summary_csv(summary_rows, path):
    fieldnames = [
        "method",
        "n",
        "success_count",
        "success_rate",
        "tracking_last_rms_m_mean",
        "tracking_last_rms_m_std",
        "T_vio_s_mean",
        "T_vio_s_std",
        "F_peak_N_mean",
        "F_peak_N_std",
        "F_max_observed_N_mean",
        "F_max_observed_N_std",
        "R_acc_mean",
        "R_acc_std",
        "R_sup_mean",
        "R_sup_std",
        "alpha_FP_min_mean",
        "alpha_FP_min_std",
        "score_mean",
        "score_std",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)


def _plot_tvio_bars(rows, out):
    summary_rows = _aggregate_rows(rows)
    labels = [r["method"] for r in summary_rows]
    x = np.arange(len(summary_rows))
    tvio = [r["T_vio_s_mean"] for r in summary_rows]
    tvio_std = [r["T_vio_s_std"] for r in summary_rows]
    fmax = [r["F_max_observed_N_mean"] for r in summary_rows]
    fmax_std = [r["F_max_observed_N_std"] for r in summary_rows]
    track = [1000.0 * r["tracking_last_rms_m_mean"] for r in summary_rows]
    track_std = [1000.0 * r["tracking_last_rms_m_std"] for r in summary_rows]
    colors = ["#b91c1c" if v > 0.20 else "#0f766e" for v in tvio]

    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), constrained_layout=True)
    axes[0].bar(x, tvio, yerr=tvio_std, color=colors, capsize=3)
    axes[0].axhline(0.20, color="k", linestyle="--", linewidth=1)
    axes[0].set_ylabel("$T_{vio}$ (s)")
    axes[0].set_title("Force-bound violation time")

    axes[1].bar(x, fmax, yerr=fmax_std, color="#2563eb", capsize=3)
    axes[1].axhline(1.05, color="k", linestyle="--", linewidth=1, label="$F_{max}$")
    axes[1].set_ylabel("max force proxy (N)")
    axes[1].set_title("Peak force proxy")
    axes[1].legend()

    axes[2].bar(x, track, yerr=track_std, color="#7c3aed", capsize=3)
    axes[2].axhline(3.0, color="k", linestyle="--", linewidth=1)
    axes[2].set_ylabel("last RMS tracking (mm)")
    axes[2].set_title("Convergence check")

    for ax in axes:
        ax.set_xticks(x, labels, rotation=25, ha="right")
        ax.grid(True, axis="y", alpha=0.3)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _plot_tvio_timeseries(run_dirs, out):
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.8), sharex=True, constrained_layout=True)
    for run_dir in run_dirs:
        run_dir = Path(run_dir)
        with np.load(run_dir / "result.npz") as npz:
            data = {k: np.asarray(npz[k]) for k in npz.files}
        method = json.loads((run_dir / "summary.json").read_text(encoding="utf-8")).get("ch5_method", run_dir.name)
        t = data["t"]
        y_f = data.get("y_f_realistic", data["y_f"])
        axes[0].plot(t, y_f, label=method, linewidth=1.4)
        axes[1].plot(t, data["normal_raw"] * 1000.0, linestyle="--", alpha=0.55)
        axes[1].plot(t, data["normal_safe"] * 1000.0, label=method, linewidth=1.4)
    axes[0].axhline(1.05, color="k", linestyle="--", linewidth=1, label="$F_{max}$")
    axes[0].axhline(0.70, color="k", linestyle=":", linewidth=1, label="$F_d$")
    axes[0].set_ylabel("force proxy (N)")
    axes[0].set_title("T_vio challenge force response")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncol=3, fontsize=8)

    axes[1].set_ylabel("normal input / safe normal (mm)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(ncol=3, fontsize=8)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _write_report(suite_dir, rows, run_dirs, dry_run, args, summary_rows):
    best = min(summary_rows, key=lambda r: float(r["score_mean"]))
    report = suite_dir / "CH5_TVIO_CHALLENGE_REPORT.md"
    lines = [
        "# 第五章 T_vio 专项验证报告",
        "",
        f"- 模式：{'dry-run deterministic check' if dry_run else 'ROS2/Gazebo online run'}",
        f"- 场景：`{SCENARIO}`",
        f"- 力边界：`F_min={args.force_min:.2f} N, F_max={args.force_max:.2f} N, margin={args.force_margin:.2f} N`",
        f"- 最优方法：`{best['method']}`",
        "",
        "## 重复实验汇总",
        "",
        "| method | n | success | tracking_last_rms_mm | T_vio_s | F_peak_N | F_max_observed_N | R_sup | alpha_FP_min | score |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in summary_rows:
        lines.append(
            f"| {r['method']} | {r['n']} | {r['success_count']}/{r['n']} | "
            f"{1000.0 * r['tracking_last_rms_m_mean']:.2f}±{1000.0 * r['tracking_last_rms_m_std']:.2f} | "
            f"{r['T_vio_s_mean']:.3f}±{r['T_vio_s_std']:.3f} | "
            f"{r['F_peak_N_mean']:.3f}±{r['F_peak_N_std']:.3f} | "
            f"{r['F_max_observed_N_mean']:.3f}±{r['F_max_observed_N_std']:.3f} | "
            f"{r['R_sup_mean']:.3f}±{r['R_sup_std']:.3f} | "
            f"{r['alpha_FP_min_mean']:.3f}±{r['alpha_FP_min_std']:.3f} | "
            f"{r['score_mean']:.3f}±{r['score_std']:.3f} |"
        )
    lines.extend([
        "",
        "## 逐次指标",
        "",
        "| method | success | tracking_last_rms_mm | T_vio_s | F_peak_N | F_max_observed_N | R_sup | alpha_FP_min | score |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for r in rows:
        lines.append(
            f"| {r['method']} | {r['success']} | {1000.0 * float(r['tracking_last_rms_m']):.2f} | "
            f"{float(r['T_vio_s']):.3f} | {float(r['F_peak_N']):.3f} | "
            f"{float(r['F_max_observed_N']):.3f} | {float(r['R_sup']):.3f} | "
            f"{float(r['alpha_FP_min']):.3f} | {float(r['score']):.3f} |"
        )
    lines.extend([
        "",
        "## 判据",
        "",
        "- `T_vio_s <= 0.20 s` 视为满足力边界约束。",
        "- `tracking_last_rms_m <= 0.003 m` 视为理想仿真末段误差收敛。",
        "- 若直接接受、固定融合或无投影方法出现明显 `T_vio`，而完整方法保持低 `T_vio` 且误差收敛，则该专项实验可用于补充第五章安全约束违背时间的优势证据。",
        "",
        "## 输出文件",
        "",
        f"- 指标表：`{suite_dir / 'ch5_tvio_metrics.csv'}`",
        f"- 重复实验汇总表：`{suite_dir / 'ch5_tvio_summary.csv'}`",
        f"- 柱状图：`{suite_dir / 'figures' / 'ch5_tvio_bars.png'}`",
        f"- 时序图：`{suite_dir / 'figures' / 'ch5_tvio_timeseries.png'}`",
        "",
        "## 单次运行目录",
        "",
    ])
    for p in run_dirs:
        lines.append(f"- `{p}`")
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def _latest_suite(output_dir, before):
    suites = set(Path(output_dir).glob(f"ch5_gazebo_no_rcm_{SCENARIO}_*"))
    new = sorted(suites - before, key=lambda p: p.stat().st_mtime)
    return new[-1] if new else None


def _run_gazebo(args):
    before = set(Path(args.output_dir).glob(f"ch5_gazebo_no_rcm_{SCENARIO}_*"))
    gazebo_args = [
        "--task-mode", "no_rcm",
        "--scenario", SCENARIO,
        "--methods", args.methods,
        "--trials", str(args.trials),
        "--task-duration-s", str(args.task_duration_s),
        "--timeout-s", str(args.timeout_s),
        "--output-dir", args.output_dir,
        "--force-min", str(args.force_min),
        "--force-max", str(args.force_max),
        "--force-margin", str(args.force_margin),
        "--contact-stiffness-hat", str(args.contact_stiffness_hat),
    ]
    if args.realistic_env:
        gazebo_args.extend([
            "--realistic-env",
            "--realistic-profile", args.realistic_profile,
            "--realistic-tau-noise-gain", str(args.realistic_tau_noise_gain),
        ])
    if args.verbose:
        gazebo_args.append("--verbose")
    if args.no_cleanup_gazebo:
        gazebo_args.append("--no-cleanup-gazebo")
    rc = run_gazebo_suite_main(gazebo_args)
    if rc not in (0, None):
        raise RuntimeError(f"Gazebo suite failed with code {rc}")
    suite_dir = _latest_suite(args.output_dir, before)
    if suite_dir is None:
        raise RuntimeError("Gazebo suite finished but no new T_vio suite directory was found")
    metrics_csv = suite_dir / "ch5_gazebo_metrics.csv"
    rows = _read_metrics(metrics_csv)
    run_dirs = [Path(r["run_dir"]) for r in rows if r.get("run_dir")]
    return suite_dir, rows, run_dirs


def _run_dry(args):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    suite_dir = Path(args.output_dir) / f"ch5_tvio_dryrun_{stamp}"
    raw_dir = suite_dir / "raw"
    methods = [m.name for m in resolve_methods(args.methods.split(","))]
    run_dirs = []
    rows = []
    for trial in range(args.trials):
        for method in methods:
            run_dir = raw_dir / f"no_rcm_dry_{method}_{SCENARIO}_t{trial:02d}"
            run_dirs.append(
                _generate_dry_run(
                    method,
                    run_dir,
                    trial,
                    duration_s=args.task_duration_s,
                    force_min=args.force_min,
                    force_max=args.force_max,
                    force_margin=args.force_margin,
                    contact_stiffness_hat=args.contact_stiffness_hat,
                )
            )
            rows.append(load_run(run_dir, method=method, trial_id=trial))
    write_metrics_csv(rows, suite_dir / "ch5_tvio_metrics.csv")
    return suite_dir, rows, run_dirs


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-gazebo", action="store_true", help="Run the online ROS2/Gazebo challenge.")
    ap.add_argument("--dry-run", action="store_true", help="Generate deterministic synthetic logs for pipeline checks.")
    ap.add_argument("--methods", default=DEFAULT_METHODS)
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--task-duration-s", type=float, default=22.0)
    ap.add_argument("--timeout-s", type=float, default=90.0)
    ap.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch5_tvio_challenge")
    ap.add_argument("--realistic-env", action="store_true")
    ap.add_argument("--realistic-profile", default="real_env_0607_0213")
    ap.add_argument("--realistic-tau-noise-gain", type=float, default=0.12)
    ap.add_argument("--force-min", type=float, default=0.38)
    ap.add_argument("--force-max", type=float, default=1.05)
    ap.add_argument("--force-margin", type=float, default=0.06)
    ap.add_argument("--contact-stiffness-hat", type=float, default=70.0)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--no-cleanup-gazebo", action="store_true")
    args = ap.parse_args(argv)

    if not args.run_gazebo and not args.dry_run:
        args.dry_run = True
    if args.run_gazebo and args.dry_run:
        raise SystemExit("Choose only one of --run-gazebo or --dry-run")

    if args.run_gazebo:
        suite_dir, rows, run_dirs = _run_gazebo(args)
        # Normalize the Gazebo CSV name for this specialized report.
        write_metrics_csv(rows, suite_dir / "ch5_tvio_metrics.csv")
        dry = False
    else:
        suite_dir, rows, run_dirs = _run_dry(args)
        dry = True

    summary_rows = _aggregate_rows(rows)
    _write_summary_csv(summary_rows, suite_dir / "ch5_tvio_summary.csv")
    figures = suite_dir / "figures"
    _plot_tvio_bars(rows, figures / "ch5_tvio_bars.png")
    _plot_tvio_timeseries(run_dirs, figures / "ch5_tvio_timeseries.png")
    report = _write_report(suite_dir, rows, run_dirs, dry_run=dry, args=args, summary_rows=summary_rows)
    payload = {
        "suite_dir": str(suite_dir),
        "metrics_csv": str(suite_dir / "ch5_tvio_metrics.csv"),
        "summary_csv": str(suite_dir / "ch5_tvio_summary.csv"),
        "report": str(report),
        "figures": [
            str(figures / "ch5_tvio_bars.png"),
            str(figures / "ch5_tvio_timeseries.png"),
        ],
        "runs": [str(p) for p in run_dirs],
    }
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
