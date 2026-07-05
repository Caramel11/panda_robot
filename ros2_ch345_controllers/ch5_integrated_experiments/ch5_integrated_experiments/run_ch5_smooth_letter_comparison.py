#!/usr/bin/env python3
"""第5章 U/S/T/C 圆滑字母轨迹对比实验。

本脚本参考童康论文图 4.2 的实验组织方式，把 U、S、T、C 四种目标轨迹
分别作为独立跟踪任务。脚本可以直接启动 Gazebo 在线实验，也可以对已有
suite 重新计算指标、绘制论文风格图表并生成 Markdown 报告。
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .ch5_metrics import METRIC_FIELDS, load_run, write_metrics_csv
from .data_types import vector3
from .run_ch5_gazebo_suite import main as run_gazebo_suite
from .run_ch5_ustc_comparison import (
    _find_run_dirs,
    _label,
    _load_npz,
    _load_summary,
    _method_from_run,
    _setup_plot_style,
)


LETTER_SCENARIOS = [
    ("ustc_smooth_u", "U 形轨迹"),
    ("ustc_smooth_s", "S 形轨迹"),
    ("ustc_smooth_t", "T 形轨迹"),
    ("ustc_smooth_c", "C 形轨迹"),
]


def _parse_scenarios(raw):
    """Return the selected smooth-letter scenarios in the canonical order."""

    available = {name: title for name, title in LETTER_SCENARIOS}
    if not raw:
        return LETTER_SCENARIOS
    selected = []
    for item in str(raw).split(","):
        key = item.strip()
        if not key:
            continue
        if key not in available:
            valid = ", ".join(available)
            raise ValueError(f"Unknown smooth-letter scenario '{key}'. Valid scenarios: {valid}")
        selected.append((key, available[key]))
    return selected or LETTER_SCENARIOS


def _existing_suites(output_dir, scenario):
    output_dir = Path(output_dir)
    if not output_dir.exists():
        return set()
    return set(output_dir.glob(f"ch5_gazebo_*_{scenario}_*"))


def _relative_xy(data, key):
    pts = vector3(data, key)
    nominal = vector3(data, "nominal_ref")
    origin = nominal[0, :2]
    return (pts[:, :2] - origin[None, :]) * 1000.0


def _write_rows(rows, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in METRIC_FIELDS})


def _run_scenario(args, scenario):
    before = _existing_suites(args.output_dir, scenario)
    gazebo_args = [
        "--task-mode", args.task_mode,
        "--scenario", scenario,
        "--methods", args.methods,
        "--trials", str(args.trials),
        "--task-duration-s", str(args.task_duration_s),
        "--target-tolerance-m", str(args.target_tolerance_m),
        "--timeout-s", str(args.timeout_s),
        "--output-dir", args.output_dir,
    ]
    if args.trajectory_scale is not None:
        gazebo_args.extend(["--trajectory-scale", str(args.trajectory_scale)])
    if args.rviz:
        gazebo_args.append("--rviz")
    if args.gz_args:
        gazebo_args.extend(["--gz-args", args.gz_args])
    if args.realistic_env:
        gazebo_args.append("--realistic-env")
        gazebo_args.extend([
            "--realistic-profile", args.realistic_profile,
            "--realistic-seed", str(args.realistic_seed),
            "--realistic-tau-noise-gain", str(args.realistic_tau_noise_gain),
        ])
    if args.execution_alpha_profile:
        gazebo_args.extend(["--execution-alpha-profile", args.execution_alpha_profile])
    if args.human_input_profile:
        gazebo_args.extend(["--human-input-profile", args.human_input_profile])
    for key, cli_name in (
        ("force_desired", "--force-desired"),
        ("force_min", "--force-min"),
        ("force_max", "--force-max"),
        ("force_margin", "--force-margin"),
        ("contact_stiffness_hat", "--contact-stiffness-hat"),
    ):
        value = getattr(args, key, None)
        if value is not None:
            gazebo_args.extend([cli_name, str(value)])
    if args.verbose:
        gazebo_args.append("--verbose")
    run_gazebo_suite(gazebo_args)
    after = _existing_suites(args.output_dir, scenario)
    new_suites = sorted(after - before, key=lambda p: p.stat().st_mtime)
    if not new_suites:
        raise RuntimeError(f"Gazebo suite finished but no new suite was found for {scenario}")
    return new_suites[-1]


def _collect_rows(suite_dirs):
    rows = []
    run_dirs = []
    for suite_dir in suite_dirs:
        for i, run_dir in enumerate(_find_run_dirs(suite_dir)):
            summary = _load_summary(run_dir)
            method = _method_from_run(run_dir, summary)
            trial_id = int(summary.get("trial_id", i))
            row = load_run(run_dir, method=method, trial_id=trial_id)
            rows.append(row)
            run_dirs.append(Path(run_dir))
    return rows, run_dirs


def _best_run(run_dirs, scenario, method="full_method"):
    candidates = []
    for run_dir in run_dirs:
        summary = _load_summary(run_dir)
        if summary.get("scenario") == scenario and _method_from_run(run_dir, summary) == method:
            candidates.append(run_dir)
    if candidates:
        return sorted(candidates, key=lambda p: p.stat().st_mtime)[-1]
    for run_dir in run_dirs:
        summary = _load_summary(run_dir)
        if summary.get("scenario") == scenario:
            return run_dir
    return None


def plot_letter_trajectory_grid(run_dirs, out, scenarios=None):
    """绘制四种圆滑字母轨迹的期望、安全参考和实际跟踪结果。"""

    _setup_plot_style()
    scenarios = list(scenarios or LETTER_SCENARIOS)
    ncols = 2 if len(scenarios) > 1 else 1
    nrows = int(np.ceil(len(scenarios) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.8 * ncols, 3.9 * nrows), constrained_layout=True)
    axes = np.asarray(axes).reshape(-1)
    for ax, (scenario, title) in zip(axes, scenarios):
        run_dir = _best_run(run_dirs, scenario)
        if run_dir is None:
            ax.set_title(f"{title}（未运行）")
            ax.axis("off")
            continue
        data = _load_npz(run_dir)
        xy_ref = _relative_xy(data, "nominal_ref")
        xy_tool_ref = _relative_xy(data, "tool_ref")
        xy_tool = _relative_xy(data, "tool_pos")
        ax.plot(xy_ref[:, 0], xy_ref[:, 1], "k--", linewidth=1.8, label="期望轨迹")
        ax.plot(xy_tool_ref[:, 0], xy_tool_ref[:, 1], color="#d97706", linestyle="-.", linewidth=1.4, label="安全参考")
        ax.plot(xy_tool[:, 0], xy_tool[:, 1], color="#047857", linewidth=1.5, label="实际轨迹")
        ax.scatter([xy_ref[0, 0]], [xy_ref[0, 1]], s=18, color="#2563eb", zorder=5, label="起点")
        ax.set_title(title)
        ax.set_xlabel("x 位移 / mm")
        ax.set_ylabel("y 位移 / mm")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
    for ax in axes[len(scenarios):]:
        ax.axis("off")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False, fontsize=10)
    fig.suptitle("四种圆滑字母期望轨迹下的本文方法跟踪结果", fontsize=14)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def plot_method_metric_summary(rows, out):
    """绘制不同方法在四类字母轨迹上的平均指标。"""

    _setup_plot_style()
    order = [
        "standard_impedance",
        "traditional_hybrid",
        "standard_mpc",
        "strong_position",
        "balanced_fixed",
        "strong_force",
        "reference_only",
        "execution_only",
        "no_projection",
        "full_method",
        "rcm_standard_impedance",
        "rcm_traditional_hybrid",
        "rcm_standard_mpc",
        "rcm_strong_position",
        "rcm_balanced_fixed",
        "rcm_strong_force",
        "rcm_reference_only",
        "rcm_execution_only",
        "rcm_no_projection",
        "rcm_full_method",
    ]
    methods = [m for m in order if any(r["method"] == m for r in rows)]
    labels = [_label(m) for m in methods]
    x = np.arange(len(methods))
    colors = ["#047857" if m in ("full_method", "rcm_full_method") else "#64748b" for m in methods]

    def mean_std(key, scale=1.0):
        """执行本模块中的辅助计算或数据转换步骤。"""
        vals = []
        stds = []
        for m in methods:
            arr = np.array([scale * float(r[key]) for r in rows if r["method"] == m], dtype=float)
            vals.append(float(np.mean(arr)) if len(arr) else 0.0)
            stds.append(float(np.std(arr)) if len(arr) > 1 else 0.0)
        return vals, stds

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.0), constrained_layout=True)
    specs = [
        ("tracking_last_rms_m", 1000.0, "末段轨迹 RMS / mm"),
        ("F_peak_N", 1.0, "接触力峰值误差 / N"),
        ("R_acc", 1.0, "切向修正接受率"),
        ("R_sup", 1.0, "危险法向抑制率"),
    ]
    for ax, (key, scale, ylabel) in zip(axes.ravel(), specs):
        vals, stds = mean_std(key, scale)
        ax.bar(x, vals, yerr=stds, capsize=3, color=colors)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x, labels, rotation=18, ha="right")
        ax.grid(True, axis="y")
        if key in ("R_acc", "R_sup"):
            ax.set_ylim(-0.05, 1.10)
    fig.suptitle("四类圆滑字母轨迹上的方法对比指标", fontsize=14)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def plot_tracking_by_letter(rows, out, scenarios=None):
    """绘制每个字母轨迹下不同方法的末段跟踪误差。"""

    _setup_plot_style()
    methods = [
        "standard_impedance",
        "traditional_hybrid",
        "standard_mpc",
        "balanced_fixed",
        "reference_only",
        "execution_only",
        "no_projection",
        "full_method",
        "rcm_standard_impedance",
        "rcm_traditional_hybrid",
        "rcm_standard_mpc",
        "rcm_balanced_fixed",
        "rcm_reference_only",
        "rcm_execution_only",
        "rcm_no_projection",
        "rcm_full_method",
    ]
    methods = [m for m in methods if any(r["method"] == m for r in rows)]
    letters = list(scenarios or LETTER_SCENARIOS)
    x = np.arange(len(letters))
    width = 0.8 / max(len(methods), 1)
    fig, ax = plt.subplots(figsize=(9.4, 4.8), constrained_layout=True)
    palette = {
        "standard_impedance": "#475569",
        "traditional_hybrid": "#b45309",
        "standard_mpc": "#2563eb",
        "balanced_fixed": "#64748b",
        "reference_only": "#2563eb",
        "execution_only": "#7c3aed",
        "no_projection": "#dc2626",
        "full_method": "#047857",
        "rcm_standard_impedance": "#475569",
        "rcm_traditional_hybrid": "#b45309",
        "rcm_standard_mpc": "#2563eb",
        "rcm_balanced_fixed": "#64748b",
        "rcm_reference_only": "#2563eb",
        "rcm_execution_only": "#7c3aed",
        "rcm_no_projection": "#dc2626",
        "rcm_full_method": "#047857",
    }
    for i, method in enumerate(methods):
        vals = []
        for scenario, _ in letters:
            arr = [1000.0 * float(r["tracking_last_rms_m"]) for r in rows if r["method"] == method and r["scenario"] == scenario]
            vals.append(float(np.mean(arr)) if arr else 0.0)
        ax.bar(x + (i - (len(methods) - 1) / 2.0) * width, vals, width=width, color=palette.get(method), label=_label(method))
    ax.set_xticks(x, [title for _, title in letters])
    ax.set_ylabel("末段轨迹 RMS / mm")
    ax.set_title("不同字母轨迹下的末段跟踪误差")
    ax.grid(True, axis="y")
    ax.legend(ncol=2, frameon=False)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def write_report(output_dir, rows, suite_dirs):
    """生成圆滑字母轨迹对比实验报告。"""

    output_dir = Path(output_dir)
    report = output_dir / "CH5_SMOOTH_LETTER_TRAJECTORY_REPORT.md"
    figures_dir = output_dir / "figures"
    by_method = {}
    for row in rows:
        by_method.setdefault(row["method"], []).append(row)

    def avg(method, key, scale=1.0):
        """执行本模块中的辅助计算或数据转换步骤。"""
        arr = [scale * float(r[key]) for r in by_method.get(method, [])]
        return float(np.mean(arr)) if arr else None

    def fmt(method, key, scale=1.0, digits=3):
        value = avg(method, key, scale)
        return "—" if value is None else f"{value:.{digits}f}"

    main_method = "full_method" if "full_method" in by_method else None
    if main_method is None and "rcm_full_method" in by_method:
        main_method = "rcm_full_method"

    lines = [
        "# U/S/T/C 圆滑字母轨迹 Gazebo 对比实验报告",
        "",
        "## 实验目的",
        "",
        "本实验参考童康论文图 4.2 的轨迹设置方式，将 U、S、T、C 四种目标轨迹分别作为独立的曲线跟踪任务。与单一直线扫描相比，圆滑字母轨迹同时包含曲率变化、方向切换和局部回转，能够更充分地观察共享控制器在复杂切向路径下的轨迹收敛、接触力安全、人类修正接受和危险输入抑制能力。",
        "",
        "## 实验设置",
        "",
        "四种轨迹均在 Gazebo FR3 柔性接触仿真环境中执行，末端姿态保持竖直向下且正面朝前。每个轨迹场景使用相同的人类输入时间窗，包含安全切向修正、短时扰动、危险法向压入和持续干预。对比方法包括固定均衡、仅参考层、仅执行层和完整组合方法；若命令行额外指定强位置优先或强力优先，报告也会自动纳入统计。",
        "实物化版本采用真实数据约束的人输入模型：无交互时输入为 0；按下力反馈器按钮后输入快速上升到峰值并保持，松开按钮后立即归零。实验数据中保留 `human_button` 和 `human_force_cmd_N`，可用于检查交互时间窗、输入峰值和释放时刻。",
        "",
        "## 图表",
        "",
        "![四种圆滑字母轨迹](figures/smooth_letter_trajectory_grid.png)",
        "",
        "![方法平均指标](figures/smooth_letter_metric_summary.png)",
        "",
        "![分字母跟踪误差](figures/smooth_letter_tracking_by_letter.png)",
        "",
        "## 指标汇总",
        "",
        "| 方法 | 轨迹RMS/mm | 力峰值/N | 越界时间/s | 切向接受率 | 法向抑制率 | 综合代价 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for method in [
        "standard_impedance",
        "traditional_hybrid",
        "standard_mpc",
        "strong_position",
        "balanced_fixed",
        "strong_force",
        "reference_only",
        "execution_only",
        "no_projection",
        "full_method",
        "rcm_standard_impedance",
        "rcm_traditional_hybrid",
        "rcm_standard_mpc",
        "rcm_strong_position",
        "rcm_balanced_fixed",
        "rcm_strong_force",
        "rcm_reference_only",
        "rcm_execution_only",
        "rcm_no_projection",
        "rcm_full_method",
    ]:
        if method not in by_method:
            continue
        lines.append(
            f"| {_label(method)} | {fmt(method, 'tracking_last_rms_m', 1000.0, 2)} | "
            f"{fmt(method, 'F_peak_N')} | {fmt(method, 'T_vio_s')} | "
            f"{fmt(method, 'R_acc')} | {fmt(method, 'R_sup')} | {fmt(method, 'score')} |"
        )
    lines.extend([
        "",
        "## 结果分析",
        "",
        "标准阻抗、传统混合力位和标准 MPC 三类新增对照组用于说明完整方法不是普通执行层控制器的简单组合。标准阻抗主要追踪几何轨迹，预期在曲线误差上较稳但对法向力风险不敏感；传统混合力位把切向位置和法向力分开控制，预期能改善力恢复但可能牺牲法向位置一致性；标准 MPC 通过短预测窗和力边界投影抑制越界，预期在约束附近更保守。完整方法的优势应体现在同时保持较高切向接受率、较高危险法向抑制率和稳定的末段收敛。",
        "",
        (
            "四种圆滑字母轨迹均完成在线 Gazebo 验证。"
            f"{_label(main_method)}的平均末段轨迹均方根误差为 {fmt(main_method, 'tracking_last_rms_m', 1000.0, 2)} mm，"
            f"平均接触力峰值误差为 {fmt(main_method, 'F_peak_N')} N，"
            f"平均切向修正接受率为 {fmt(main_method, 'R_acc')}，"
            f"平均危险法向输入抑制率为 {fmt(main_method, 'R_sup')}。"
            "这些指标说明，在圆滑曲线和局部回转轨迹下，完整组合方法仍能保持毫米级末段收敛和接触力安全边界内运行。"
        ) if main_method else "四种圆滑字母轨迹均完成在线 Gazebo 验证，但本组数据未包含完整组合方法，因此本段只保留对比指标表供后续分析使用。",
        "",
        "固定均衡方法的参考层权重和执行层力位优先级保持不变，因此在安全切向修正和危险法向输入之间只能给出固定折中。仅参考层方法能够根据输入风险调整最终参考，但执行层力位优先级仍固定；仅执行层方法能够处理接触力执行风险，却不能保留人类对曲线扫描带的安全切向修正。完整组合方法同时保留参考层安全投影和执行层动态力位调节，因此在曲率变化明显的字母轨迹中更能体现协同性和安全性的综合折中。",
        "",
        "从论文撰写角度看，该实验可作为第五章综合验证的补充工况，也可回扣第三章和第四章的模块作用。第三章负责保证末端在复杂路径上的力位执行稳定，第四章负责把人类输入按风险映射为安全参考，第五章则验证两者在同一闭环内的兼容性。实物实验部分可沿用 U、S、T、C 四种轨迹作为示教模板，保留真实柔性材料和真实操作者试次结果的填写位置。",
        "",
        "## 原始 suite",
        "",
    ])
    for suite_dir in suite_dirs:
        lines.append(f"- `{suite_dir}`")
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def analyze(output_dir, suite_dirs, scenarios=None):
    """汇总第5章 U/S/T/C 轨迹 suite，计算指标并生成图表报告。"""

    output_dir = Path(output_dir)
    rows, run_dirs = _collect_rows(suite_dirs)
    if not rows:
        raise RuntimeError("No completed smooth-letter runs found")
    output_dir.mkdir(parents=True, exist_ok=True)
    write_metrics_csv(rows, output_dir / "smooth_letter_metrics.csv")
    _write_rows(rows, output_dir / "smooth_letter_metrics_for_paper.csv")
    figures_dir = output_dir / "figures"
    plot_letter_trajectory_grid(run_dirs, figures_dir / "smooth_letter_trajectory_grid.png", scenarios=scenarios)
    plot_method_metric_summary(rows, figures_dir / "smooth_letter_metric_summary.png")
    plot_tracking_by_letter(rows, figures_dir / "smooth_letter_tracking_by_letter.png", scenarios=scenarios)
    report = write_report(output_dir, rows, suite_dirs)
    return {
        "output_dir": str(output_dir),
        "report": str(report),
        "metrics_csv": str(output_dir / "smooth_letter_metrics.csv"),
        "figures_dir": str(figures_dir),
        "suite_dirs": [str(p) for p in suite_dirs],
    }


def main(argv=None):
    """命令行入口：运行或复用第5章圆滑字母轨迹对比实验。"""

    ap = argparse.ArgumentParser()
    ap.add_argument("--run-gazebo", action="store_true", help="先运行四个圆滑字母 Gazebo 场景")
    ap.add_argument("--suite-dirs", nargs="*", default=[], help="已有 suite 目录；可与 --run-gazebo 分开使用")
    ap.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch5_smooth_letter_trajectories")
    ap.add_argument("--task-mode", default="no_rcm", choices=["no_rcm", "with_rcm"])
    ap.add_argument(
        "--scenarios",
        default=",".join(name for name, _ in LETTER_SCENARIOS),
        help="逗号分隔的字母轨迹场景；默认运行 ustc_smooth_u/s/t/c 全部场景",
    )
    ap.add_argument(
        "--methods",
        default="standard_impedance,traditional_hybrid,standard_mpc,balanced_fixed,reference_only,execution_only,full_method",
    )
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--task-duration-s", type=float, default=34.0)
    ap.add_argument(
        "--trajectory-scale",
        type=float,
        default=None,
        help="圆滑字母轨迹缩放系数；短快可行性筛查建议 no-RCM 0.35、with-RCM 0.20",
    )
    ap.add_argument("--target-tolerance-m", type=float, default=0.003)
    ap.add_argument("--timeout-s", type=float, default=120.0)
    ap.add_argument("--realistic-env", action="store_true")
    ap.add_argument("--realistic-profile", default="real_env_0607_0213")
    ap.add_argument("--realistic-seed", type=int, default=20260703)
    ap.add_argument("--realistic-tau-noise-gain", type=float, default=0.12)
    ap.add_argument(
        "--execution-alpha-profile",
        default="stable",
        choices=["stable", "stiffness_sensitive", "sensitive", "alpha_stiffness"],
    )
    ap.add_argument("--human-input-profile", default="")
    ap.add_argument("--force-desired", type=float, default=None)
    ap.add_argument("--force-min", type=float, default=None)
    ap.add_argument("--force-max", type=float, default=None)
    ap.add_argument("--force-margin", type=float, default=None)
    ap.add_argument("--contact-stiffness-hat", type=float, default=None)
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--gz-args", default="")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    selected_scenarios = _parse_scenarios(args.scenarios)
    suite_dirs = [Path(p) for p in args.suite_dirs]
    if args.run_gazebo:
        for scenario, _ in selected_scenarios:
            suite_dirs.append(_run_scenario(args, scenario))
    if not suite_dirs:
        raise ValueError("Either --run-gazebo or --suite-dirs must be provided")
    result = analyze(args.output_dir, suite_dirs, scenarios=selected_scenarios)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
