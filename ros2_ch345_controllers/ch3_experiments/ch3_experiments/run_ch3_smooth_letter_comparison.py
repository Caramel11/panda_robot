#!/usr/bin/env python3
"""第三章 U/S/T/C 圆滑字母轨迹执行层 Gazebo 对照实验。

第三章关注执行层力位协同控制。为避免把第四章人机参考仲裁混入本章结论，
本脚本固定 ``arbitration_strategy:=autonomous_only``，只改变
``execution_strategy``：强位置优先、均衡优先、强力优先和动态优先级。

底层 Gazebo 节点复用已经稳定验证的 ``ch4_shared_gt_controller``，因为该节点
提供了统一的 FR3、重力补偿、姿态保持、接触代理量和 U/S/T/C 圆滑轨迹场景。
"""

import argparse
import csv
import json
import os
import signal
import select
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ch4_shared_gt_controller.ch4_metrics import compute_metrics, load_result


LETTER_SCENARIOS = [
    ("ustc_smooth_u", "U 形轨迹"),
    ("ustc_smooth_s", "S 形轨迹"),
    ("ustc_smooth_t", "T 形轨迹"),
    ("ustc_smooth_c", "C 形轨迹"),
]


@dataclass(frozen=True)
class ExecutionMethod:
    """实验辅助类，保存本模块运行所需的配置、状态或接口。"""
    name: str
    label: str
    execution_strategy: str
    fixed_alpha_fp: float


METHODS = {
    "standard_impedance": ExecutionMethod("standard_impedance", "标准阻抗控制", "standard_impedance", 1.0),
    "traditional_hybrid": ExecutionMethod("traditional_hybrid", "传统混合力位", "traditional_hybrid", 0.0),
    "standard_mpc": ExecutionMethod("standard_mpc", "标准MPC", "standard_mpc", 0.5),
    "strong_position": ExecutionMethod("strong_position", "强位置优先", "fixed_08", 0.8),
    "balanced_fixed": ExecutionMethod("balanced_fixed", "均衡优先", "fixed_05", 0.5),
    "strong_force": ExecutionMethod("strong_force", "强力优先", "fixed_02", 0.2),
    "continuous_force_margin": ExecutionMethod("continuous_force_margin", "本文方法", "continuous_force_margin", 0.5),
    "dynamic_gt": ExecutionMethod("dynamic_gt", "本文动态优先级", "dynamic_gt", 0.5),
}


def _setup_plot_style():
    plt.rcParams.update({
        "font.sans-serif": [
            "Noto Sans CJK SC",
            "Noto Sans CJK JP",
            "Droid Sans Fallback",
            "SimHei",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
        "figure.dpi": 140,
        "savefig.dpi": 260,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })


def _existing_run_dirs(root):
    root = Path(root)
    if not root.exists():
        return set()
    return {p.parent for p in root.rglob("summary.json")}


def _new_run_dir(root, before):
    after = _existing_run_dirs(root)
    new_dirs = sorted(after - before, key=lambda p: p.stat().st_mtime)
    return new_dirs[-1] if new_dirs else None


def _terminate(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=8.0)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5.0)
        except Exception:
            os.killpg(proc.pid, signal.SIGKILL)


def _cleanup_gazebo():
    patterns = [
        "ign gazebo",
        "gz sim",
        "shared_gt_gazebo_node",
        "robot_state_publisher",
        "ros2 launch ch4_shared",
    ]
    for pattern in patterns:
        subprocess.run(["pkill", "-INT", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)
    for pattern in patterns:
        subprocess.run(["pkill", "-TERM", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _resolve_methods(names):
    out = []
    for name in names.split(","):
        key = name.strip()
        if not key:
            continue
        if key not in METHODS:
            raise ValueError(f"未知方法 {key}; 可选: {', '.join(METHODS)}")
        out.append(METHODS[key])
    return out


def _scenario_seed_offset(scenario):
    """Return a deterministic seed offset so different letters do not share noise."""

    return 1000 * (1 + sum(ord(ch) for ch in str(scenario)) % 997)


def _run_one(args, scenario, method):
    output_root = Path(args.output_dir) / "raw"
    output_root.mkdir(parents=True, exist_ok=True)
    before = _existing_run_dirs(output_root)
    cmd = [
        "ros2",
        "launch",
        "ch4_shared_gt_controller",
        "ch4_shared_gt_gazebo.launch.py",
        f"task_mode:={args.task_mode}",
        f"controller_variant:={args.controller_variant}",
        "arbitration_strategy:=autonomous_only",
        f"execution_strategy:={method.execution_strategy}",
        f"execution_alpha_profile:={args.execution_alpha_profile}",
        f"scenario:={scenario}",
        "fixed_alpha_hr:=0.0",
        f"fixed_alpha_fp:={method.fixed_alpha_fp}",
        f"task_duration_s:={args.task_duration_s}",
        f"output_dir:={output_root}",
        f"rviz:={str(args.rviz).lower()}",
    ]
    if args.trajectory_scale is not None:
        cmd.append(f"trajectory_scale:={args.trajectory_scale}")
    if args.realistic_env:
        realistic_seed = int(args.realistic_seed) + _scenario_seed_offset(scenario) + int(args.current_repeat_index)
        cmd.extend([
            "realistic_env_enabled:=true",
            f"realistic_profile:={args.realistic_profile}",
            f"realistic_seed:={realistic_seed}",
            f"realistic_tau_noise_gain:={args.realistic_tau_noise_gain}",
        ])
    if args.human_input_profile:
        cmd.append(f"human_input_profile:={args.human_input_profile}")
    if args.gz_args:
        cmd.append(f"gz_args:={args.gz_args}")
    print(f"[ch3-letter] scenario={scenario} method={method.name}: {' '.join(cmd)}", flush=True)
    proc = subprocess.Popen(
        cmd,
        cwd="/home/liu/franka_ros2_ws",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        bufsize=1,
    )
    deadline = time.time() + args.timeout_s
    tail = []
    run_dir = None
    try:
        while time.time() < deadline:
            line = ""
            if proc.stdout:
                ready, _, _ = select.select([proc.stdout], [], [], 0.2)
                if ready:
                    line = proc.stdout.readline()
            if line:
                tail.append(line.rstrip())
                tail = tail[-100:]
                if args.verbose:
                    print(line, end="")
            run_dir = _new_run_dir(output_root, before)
            if run_dir is not None and (run_dir / "summary.json").exists() and (run_dir / "result.npz").exists():
                break
            if proc.poll() is not None and not line:
                break
            time.sleep(0.2)
    finally:
        _terminate(proc)
        if args.cleanup_gazebo:
            _cleanup_gazebo()
    if run_dir is None or not (run_dir / "result.npz").exists():
        log_path = Path(args.output_dir) / f"failed_{scenario}_{method.name}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("\n".join(tail), encoding="utf-8")
        raise RuntimeError(f"{scenario}/{method.name} 未生成完整结果: {log_path}")
    with open(run_dir / "summary.json", "r", encoding="utf-8") as f:
        summary = json.load(f)
    summary.update({
        "ch3_letter_method": method.name,
        "ch3_letter_label": method.label,
        "trial_id": int(args.current_repeat_index),
        "realistic_env_enabled": bool(args.realistic_env),
        "human_input_profile": args.human_input_profile,
        "arbitration_strategy": "autonomous_only",
        "execution_strategy": method.execution_strategy,
    })
    with open(run_dir / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    return run_dir


def _find_run_dirs(paths):
    out = []
    for path in paths:
        p = Path(path)
        if (p / "summary.json").exists():
            out.append(p)
        else:
            out.extend(sorted(q.parent for q in p.rglob("summary.json")))
    return out


def _vec(data, prefix):
    return np.column_stack([np.asarray(data[f"{prefix}_{i}"], dtype=float) for i in range(3)])


def _relative_xy(data, prefix):
    pts = _vec(data, prefix)
    nominal = _vec(data, "nominal_ref")
    origin = nominal[0, :2]
    return (pts[:, :2] - origin[None, :]) * 1000.0


def _method_from_summary(summary):
    name = summary.get("ch3_letter_method")
    if name:
        return name
    strategy = summary.get("execution_strategy", "unknown")
    for key, method in METHODS.items():
        if method.execution_strategy == strategy:
            return key
    return strategy


def _collect_rows(run_dirs):
    rows = []
    clean_dirs = []
    for run_dir in run_dirs:
        run_dir = Path(run_dir)
        with open(run_dir / "summary.json", "r", encoding="utf-8") as f:
            summary = json.load(f)
        scenario = summary.get("scenario", "unknown")
        _, data = load_result(run_dir)
        metrics = compute_metrics(data, scenario=scenario)
        method = _method_from_summary(summary)
        label = METHODS[method].label if method in METHODS else method
        row = {
            "run_dir": str(run_dir),
            "scenario": scenario,
            "letter": dict(LETTER_SCENARIOS).get(scenario, scenario),
            "method": method,
            "label": label,
            **metrics,
            "alpha_FP_mean": float(summary.get("alpha_FP_mean", metrics.get("alpha_mean", np.nan))),
            "alpha_FP_min": float(summary.get("alpha_FP_min", np.nan)),
            "alpha_FP_max": float(summary.get("alpha_FP_max", np.nan)),
            "S_alpha_FP": float(summary.get("S_alpha_FP", np.nan)),
        }
        rows.append(row)
        clean_dirs.append(run_dir)
    return rows, clean_dirs


def _write_csv(rows, path):
    if not rows:
        return
    fields = [
        "scenario", "letter", "method", "success",
        "tracking_last_rms_m", "tracking_last_max_m",
        "F_peak_N", "T_vio_s", "alpha_FP_mean", "alpha_FP_min",
        "alpha_FP_max", "S_alpha_FP", "ori_last_rms_deg",
        "front_last_rms_deg", "run_dir",
    ]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fields})


def _aggregate(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row["method"], []).append(row)
    out = []
    keys = [
        "tracking_last_rms_m", "tracking_last_max_m", "F_peak_N",
        "T_vio_s", "alpha_FP_mean", "S_alpha_FP",
        "ori_last_rms_deg", "front_last_rms_deg",
    ]
    order = [
        "standard_impedance",
        "traditional_hybrid",
        "standard_mpc",
        "strong_position",
        "balanced_fixed",
        "strong_force",
        "continuous_force_margin",
        "dynamic_gt",
    ]
    for method, items in grouped.items():
        row = {
            "method": method,
            "label": METHODS[method].label if method in METHODS else method,
            "letters": len(items),
            "success_count": int(sum(1 for item in items if bool(item.get("success")))),
        }
        for key in keys:
            vals = np.asarray([float(item.get(key, np.nan)) for item in items], dtype=float)
            row[f"{key}_mean"] = float(np.nanmean(vals)) if np.isfinite(vals).any() else np.nan
            row[f"{key}_std"] = float(np.nanstd(vals)) if np.isfinite(vals).any() else np.nan
        out.append(row)
    return sorted(out, key=lambda r: order.index(r["method"]) if r["method"] in order else 99)


def _write_summary_csv(rows, path):
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _best_run(run_dirs, scenario, method="continuous_force_margin"):
    candidates = []
    fallback = []
    for run_dir in run_dirs:
        with open(run_dir / "summary.json", "r", encoding="utf-8") as f:
            summary = json.load(f)
        if summary.get("scenario") == scenario:
            fallback.append(run_dir)
            if _method_from_summary(summary) == method:
                candidates.append(run_dir)
    return (candidates or fallback or [None])[-1]


def plot_trajectory_grid(run_dirs, output_path):
    """根据实验数据生成论文或调试用图表。"""
    _setup_plot_style()
    fig, axes = plt.subplots(2, 2, figsize=(9.3, 7.3), constrained_layout=True)
    for ax, (scenario, title) in zip(axes.ravel(), LETTER_SCENARIOS):
        run_dir = _best_run(run_dirs, scenario)
        if run_dir is None:
            ax.set_title(f"{title}（未运行）")
            ax.axis("off")
            continue
        _, data = load_result(run_dir)
        xy_nom = _relative_xy(data, "nominal_ref")
        xy_pos = _relative_xy(data, "tool_pos")
        ax.plot(xy_nom[:, 0], xy_nom[:, 1], "k--", linewidth=1.8, label="期望轨迹")
        ax.plot(xy_pos[:, 0], xy_pos[:, 1], color="#047857", linewidth=1.5, label="实际轨迹")
        ax.scatter([xy_nom[0, 0]], [xy_nom[0, 1]], color="#2563eb", s=18, zorder=5, label="起点")
        ax.set_title(title)
        ax.set_xlabel("x 位移 / mm")
        ax.set_ylabel("y 位移 / mm")
        ax.set_aspect("equal", adjustable="box")
    handles, labels = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False)
    fig.suptitle("第三章圆滑字母轨迹执行层跟踪结果", fontsize=14)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def plot_metric_summary(summary_rows, output_path):
    """根据实验数据生成论文或调试用图表。"""
    _setup_plot_style()
    labels = [row["label"] for row in summary_rows]
    x = np.arange(len(summary_rows))
    colors = ["#047857" if row["method"] in ("continuous_force_margin", "dynamic_gt") else "#64748b" for row in summary_rows]
    specs = [
        ("tracking_last_rms_m_mean", 1000.0, "末段轨迹 RMS / mm"),
        ("F_peak_N_mean", 1.0, "接触力峰值误差 / N"),
        ("T_vio_s_mean", 1.0, "力越界时间 / s"),
        ("alpha_FP_mean_mean", 1.0, "平均执行层权重"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10.6, 7.0), constrained_layout=True)
    for ax, (key, scale, ylabel) in zip(axes.ravel(), specs):
        vals = [scale * row.get(key, np.nan) for row in summary_rows]
        ax.bar(x, vals, color=colors)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x, labels, rotation=18, ha="right")
        if "权重" in ylabel:
            ax.set_ylim(-0.05, 1.05)
        ax.grid(True, axis="y")
    fig.suptitle("第三章圆滑字母轨迹执行层方法对比", fontsize=14)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def plot_tracking_by_letter(rows, output_path):
    """根据实验数据生成论文或调试用图表。"""
    _setup_plot_style()
    methods = [
        "standard_impedance",
        "traditional_hybrid",
        "standard_mpc",
        "strong_position",
        "balanced_fixed",
        "strong_force",
        "continuous_force_margin",
        "dynamic_gt",
    ]
    methods = [m for m in methods if any(row["method"] == m for row in rows)]
    x = np.arange(len(LETTER_SCENARIOS))
    width = 0.8 / max(len(methods), 1)
    palette = {
        "standard_impedance": "#475569",
        "traditional_hybrid": "#b45309",
        "standard_mpc": "#2563eb",
        "strong_position": "#64748b",
        "balanced_fixed": "#94a3b8",
        "strong_force": "#7c3aed",
        "continuous_force_margin": "#047857",
        "dynamic_gt": "#047857",
    }
    fig, ax = plt.subplots(figsize=(9.4, 4.7), constrained_layout=True)
    for i, method in enumerate(methods):
        vals = []
        for scenario, _ in LETTER_SCENARIOS:
            arr = [1000.0 * float(row["tracking_last_rms_m"]) for row in rows if row["scenario"] == scenario and row["method"] == method]
            vals.append(float(np.nanmean(arr)) if arr else np.nan)
        ax.bar(x + (i - (len(methods) - 1) / 2.0) * width, vals, width=width,
               color=palette.get(method, "#64748b"), label=METHODS[method].label)
    ax.set_xticks(x, [title for _, title in LETTER_SCENARIOS])
    ax.set_ylabel("末段轨迹 RMS / mm")
    ax.set_title("不同圆滑字母轨迹下的执行层跟踪误差")
    ax.legend(ncol=2, frameon=False)
    ax.grid(True, axis="y")
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def write_report(output_dir, rows, summary_rows, run_dirs):
    """把分析结果、图表索引或时序数据写入磁盘文件。"""
    output_dir = Path(output_dir)
    report = output_dir / "CH3_SMOOTH_LETTER_GAZEBO_REPORT.md"
    method = next((row for row in summary_rows if row["method"] == "continuous_force_margin"), None)
    if method is None:
        method = next((row for row in summary_rows if row["method"] == "dynamic_gt"), None)
    lines = [
        "# 第三章 U/S/T/C 圆滑字母轨迹 Gazebo 对照实验报告",
        "",
        "## 实验目的",
        "",
        "本实验用于补充第三章执行层力位控制器在复杂切向轨迹下的验证。实验固定参考层为自主参考，不接受人类输入，只改变执行层控制律或力位优先级，从而观察标准阻抗控制、传统混合力位控制、标准 MPC、固定强位置、固定均衡、固定强力和动态优先级在圆滑字母轨迹中的轨迹保持与接触力安全表现。",
        "",
        "## 图表",
        "",
        "![圆滑字母轨迹跟踪](figures/ch3_smooth_letter_trajectory_grid.png)",
        "",
        "![平均指标对比](figures/ch3_smooth_letter_metric_summary.png)",
        "",
        "![分字母误差](figures/ch3_smooth_letter_tracking_by_letter.png)",
        "",
        "## 平均指标",
        "",
        "| 方法 | 成功次数 | 轨迹RMS/mm | 轨迹峰值/mm | 力峰值/N | 越界时间/s | 平均alpha_FP | 姿态误差/deg |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        lines.append(
            f"| {row['label']} | {row['success_count']}/{row['letters']} | "
            f"{1000.0 * row['tracking_last_rms_m_mean']:.2f} | "
            f"{1000.0 * row['tracking_last_max_m_mean']:.2f} | "
            f"{row['F_peak_N_mean']:.3f} | {row['T_vio_s_mean']:.3f} | "
            f"{row['alpha_FP_mean_mean']:.3f} | {row['front_last_rms_deg_mean']:.3f} |"
        )
    lines.extend(["", "## 结果分析", ""])
    if method:
        lines.append(
            f"本文动态优先级方法在四类字母轨迹中的成功次数为 {method['success_count']}/{method['letters']}，"
            f"平均末段轨迹 RMS 为 {1000.0 * method['tracking_last_rms_m_mean']:.2f} mm，"
            f"平均接触力峰值误差为 {method['F_peak_N_mean']:.3f} N，"
            f"平均力越界时间为 {method['T_vio_s_mean']:.3f} s。"
            "这说明在不引入人类参考修正的条件下，执行层动态优先级仍能在复杂切向曲线中保持稳定收敛。"
        )
    lines.extend([
        "标准阻抗控制可视为三轴弹簧阻尼系统，核心目标是几何轨迹收敛，不显式闭合法向力环；因此预期轨迹误差较小，但在硬接触或危险法向输入下更容易出现接触力峰值增大。",
        "传统混合力位控制在切向保持位置控制、法向采用力误差反馈，原理上更强调接触力恢复；其代价是法向位置跟踪和曲线任务中的几何一致性可能下降。",
        "标准 MPC 对照组使用短预测窗和力边界约束修正法向输入，预期能降低越界风险，但其调节主要发生在约束激活时，不像本文动态 GT 能连续解释 alpha_FP 随力裕度和刚度变化的趋势。",
        "固定强位置优先有利于几何轨迹保持，但在接触力变化时缺少法向释放；固定强力优先更关注接触力恢复，但可能牺牲几何跟踪。动态优先级方法根据接触状态连续调整 alpha_FP，使控制器不长期停留在某一固定折中点，因此更适合后续第四章和第五章中的人机协同任务。",
    ])
    lines.extend(["", "## 原始结果目录", ""])
    for run_dir in run_dirs:
        lines.append(f"- `{run_dir}`")
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def analyze(output_dir, run_dirs):
    """分析已有实验结果并生成指标、图表和报告。"""
    rows, run_dirs = _collect_rows(run_dirs)
    if not rows:
        raise RuntimeError("没有找到第三章圆滑字母实验数据")
    output_dir = Path(output_dir)
    figures_dir = output_dir / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = _aggregate(rows)
    _write_csv(rows, output_dir / "ch3_smooth_letter_metrics.csv")
    _write_summary_csv(summary_rows, output_dir / "ch3_smooth_letter_summary.csv")
    plot_trajectory_grid(run_dirs, figures_dir / "ch3_smooth_letter_trajectory_grid.png")
    plot_metric_summary(summary_rows, figures_dir / "ch3_smooth_letter_metric_summary.png")
    plot_tracking_by_letter(rows, figures_dir / "ch3_smooth_letter_tracking_by_letter.png")
    report = write_report(output_dir, rows, summary_rows, run_dirs)
    return {
        "output_dir": str(output_dir),
        "report": str(report),
        "metrics_csv": str(output_dir / "ch3_smooth_letter_metrics.csv"),
        "summary_csv": str(output_dir / "ch3_smooth_letter_summary.csv"),
        "figures_dir": str(figures_dir),
        "run_dirs": [str(p) for p in run_dirs],
    }


def main(argv=None):
    """命令行入口，解析参数并执行本模块对应的实验、绘图或分析流程。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-gazebo", action="store_true", help="先运行四个圆滑字母 Gazebo 场景")
    parser.add_argument("--run-dirs", nargs="*", default=[], help="已有 run 或结果根目录")
    parser.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch3_smooth_letter_shared_gazebo")
    parser.add_argument(
        "--methods",
        default="standard_impedance,traditional_hybrid,standard_mpc,strong_position,balanced_fixed,strong_force,continuous_force_margin",
    )
    parser.add_argument("--scenarios", default=",".join(name for name, _ in LETTER_SCENARIOS))
    parser.add_argument("--task-mode", default="no_rcm", choices=["no_rcm", "with_rcm"])
    parser.add_argument("--controller-variant", default="gt_kf")
    parser.add_argument("--task-duration-s", type=float, default=34.0)
    parser.add_argument(
        "--trajectory-scale",
        type=float,
        default=None,
        help="圆滑字母轨迹缩放系数；短快可行性筛查建议使用 0.20-0.35",
    )
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--realistic-env", action="store_true")
    parser.add_argument("--realistic-profile", default="real_env_0607_0213")
    parser.add_argument("--realistic-seed", type=int, default=20260703)
    parser.add_argument("--realistic-tau-noise-gain", type=float, default=0.12)
    parser.add_argument(
        "--execution-alpha-profile",
        default="stable",
        choices=["stable", "stiffness_sensitive", "sensitive", "alpha_stiffness"],
        help="continuous_force_margin 的 alpha 调节参数组；默认 stable 保持主实验稳定设置",
    )
    parser.add_argument("--human-input-profile", default="")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--gz-args", default="")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-cleanup-gazebo", dest="cleanup_gazebo", action="store_false")
    parser.set_defaults(cleanup_gazebo=True)
    args = parser.parse_args(argv)

    run_dirs = _find_run_dirs(args.run_dirs)
    methods = _resolve_methods(args.methods)
    scenario_names = {name.strip() for name in args.scenarios.split(",") if name.strip()}
    scenarios = [(name, label) for name, label in LETTER_SCENARIOS if name in scenario_names]
    if not scenarios:
        raise ValueError(f"没有合法场景: {args.scenarios}")
    if args.run_gazebo:
        for scenario, _ in scenarios:
            for method in methods:
                for repeat_index in range(args.repeats):
                    args.current_repeat_index = repeat_index
                    run_dirs.append(_run_one(args, scenario, method))
    if not run_dirs:
        raise ValueError("需要指定 --run-gazebo 或 --run-dirs")
    result = analyze(args.output_dir, run_dirs)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
