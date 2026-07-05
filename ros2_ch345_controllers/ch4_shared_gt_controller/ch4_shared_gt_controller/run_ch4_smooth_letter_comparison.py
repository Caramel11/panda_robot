#!/usr/bin/env python3
"""第四章 U/S/T/C 圆滑字母轨迹参考层仲裁对比实验。

本脚本把第四章已有共享控制 Gazebo 节点中的 ``ustc_smooth_u/s/t/c``
场景组织成论文可复现实验矩阵。它可以启动在线 Gazebo 仿真，也可以对已有
run 目录重新计算指标、绘制中文图表并生成 Markdown 报告。
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ch4_shared_gt_controller.ch4_metrics import compute_metrics, load_result
from ch4_shared_gt_controller.run_ch4_gazebo_suite import main as run_ch4_suite


LETTER_SCENARIOS = [
    ("ustc_smooth_u", "U 形轨迹"),
    ("ustc_smooth_s", "S 形轨迹"),
    ("ustc_smooth_t", "T 形轨迹"),
    ("ustc_smooth_c", "C 形轨迹"),
]

STRATEGY_LABELS = {
    "autonomous_only": "仅自主",
    "direct_accept": "直接接受",
    "fixed_blend": "固定融合",
    "single_sigmoid": "单通道仲裁",
    "dynamic_no_projection": "无投影动态",
    "full_method": "本文方法",
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


def _label(strategy):
    return STRATEGY_LABELS.get(strategy, strategy)


def _existing_suites(output_dir, scenario):
    output_dir = Path(output_dir)
    if not output_dir.exists():
        return set()
    return set(output_dir.glob(f"suite_*"))


def _find_run_dirs(root):
    root = Path(root)
    if (root / "summary.json").exists():
        return [root]
    return sorted(p.parent for p in root.rglob("summary.json"))


def _load_summary(run_dir):
    with open(Path(run_dir) / "summary.json", "r", encoding="utf-8") as f:
        return json.load(f)


def _run_scenario(args, scenario):
    before = _existing_suites(args.output_dir, scenario)
    suite_args = [
        "--scenarios", scenario,
        "--strategies", args.strategies,
        "--repeats", str(args.repeats),
        "--task-mode", args.task_mode,
        "--controller-variant", args.controller_variant,
        "--task-duration-s", str(args.task_duration_s),
        "--timeout-s", str(args.timeout_s),
        "--settle-after-save-s", str(args.settle_after_save_s),
        "--output-dir", args.output_dir,
        "--group-outputs",
    ]
    if args.trajectory_scale is not None:
        suite_args.extend(["--trajectory-scale", str(args.trajectory_scale)])
    if args.rviz:
        suite_args.append("--rviz")
    if args.gz_args:
        suite_args.extend(["--gz-args", args.gz_args])
    if args.realistic_env:
        suite_args.append("--realistic-env")
        suite_args.extend([
            "--realistic-profile", args.realistic_profile,
            "--realistic-seed", str(args.realistic_seed),
            "--realistic-tau-noise-gain", str(args.realistic_tau_noise_gain),
        ])
    if args.human_input_profile:
        suite_args.extend(["--human-input-profile", args.human_input_profile])
    run_ch4_suite(suite_args)
    after = _existing_suites(args.output_dir, scenario)
    new_suites = sorted(after - before, key=lambda p: p.stat().st_mtime)
    if not new_suites:
        # run_ch4_gazebo_suite names suites by timestamp; if a timestamp collision
        # happens, fall back to the newest suite.
        new_suites = sorted(after, key=lambda p: p.stat().st_mtime)
    if not new_suites:
        raise RuntimeError(f"没有找到 {scenario} 的新增 suite")
    return new_suites[-1]


def _vec(data, prefix):
    return np.column_stack([
        np.asarray(data[f"{prefix}_{i}"], dtype=float)
        for i in range(3)
    ])


def _relative_xy(data, prefix):
    pts = _vec(data, prefix)
    nominal = _vec(data, "nominal_ref")
    origin = nominal[0, :2]
    return (pts[:, :2] - origin[None, :]) * 1000.0


def _collect_rows(suite_dirs):
    rows = []
    run_dirs = []
    for suite_dir in suite_dirs:
        for run_dir in _find_run_dirs(suite_dir):
            summary = _load_summary(run_dir)
            scenario = summary.get("scenario", "unknown")
            strategy = summary.get("arbitration_strategy", summary.get("strategy", run_dir.name))
            if (Path(run_dir) / "result.npz").exists():
                _, data = load_result(run_dir)
                metrics = compute_metrics(data, scenario=scenario, strategy=strategy)
            else:
                metrics = {}
            row = {
                "suite_dir": str(suite_dir),
                "run_dir": str(run_dir),
                "scenario": scenario,
                "letter": dict(LETTER_SCENARIOS).get(scenario, scenario),
                "strategy": strategy,
                "label": _label(strategy),
                **summary,
                **metrics,
            }
            rows.append(row)
            run_dirs.append(Path(run_dir))
    return rows, run_dirs


def _write_csv(rows, path):
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scenario", "letter", "strategy", "success",
        "tracking_last_rms_m", "tracking_last_max_m",
        "R_acc", "R_sup", "R_project", "T_vio_s", "T_vio_normal_s",
        "R_sup_request_m", "R_sup_kept_m", "R_sup_mask_samples", "R_sup_eval_source",
        "F_peak_N", "F_peak_normal_N", "S_alpha",
        "reference_jump_max_m", "safe_projection_delta_max_m",
        "ori_last_rms_deg", "front_last_rms_deg", "run_dir",
    ]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fields})


def _aggregate(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row["strategy"], []).append(row)
    keys = [
        "tracking_last_rms_m", "tracking_last_max_m", "R_acc", "R_sup",
        "R_project", "T_vio_s", "F_peak_N", "S_alpha",
        "reference_jump_max_m", "safe_projection_delta_max_m",
        "front_last_rms_deg",
    ]
    out = []
    for strategy, items in grouped.items():
        row = {"strategy": strategy, "label": _label(strategy), "letters": len(items)}
        row["success_count"] = int(sum(1 for item in items if bool(item.get("success"))))
        for key in keys:
            vals = np.asarray([float(item.get(key, np.nan)) for item in items], dtype=float)
            row[f"{key}_mean"] = float(np.nanmean(vals)) if np.isfinite(vals).any() else np.nan
            row[f"{key}_std"] = float(np.nanstd(vals)) if np.isfinite(vals).any() else np.nan
        out.append(row)
    order = ["autonomous_only", "direct_accept", "fixed_blend", "single_sigmoid", "dynamic_no_projection", "full_method"]
    return sorted(out, key=lambda r: order.index(r["strategy"]) if r["strategy"] in order else 99)


def _write_summary_csv(rows, path):
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _best_run(run_dirs, scenario, strategy="full_method"):
    candidates = []
    fallback = []
    for run_dir in run_dirs:
        summary = _load_summary(run_dir)
        if summary.get("scenario") == scenario:
            fallback.append(run_dir)
            if summary.get("arbitration_strategy") == strategy:
                candidates.append(run_dir)
    return (candidates or fallback or [None])[-1]


def plot_trajectory_grid(run_dirs, output_path):
    """绘制 U/S/T/C 四类轨迹的名义、安全参考和实际跟踪轨迹。"""

    _setup_plot_style()
    fig, axes = plt.subplots(2, 2, figsize=(9.4, 7.4), constrained_layout=True)
    for ax, (scenario, title) in zip(axes.ravel(), LETTER_SCENARIOS):
        run_dir = _best_run(run_dirs, scenario)
        if run_dir is None:
            ax.set_title(f"{title}（未运行）")
            ax.axis("off")
            continue
        _, data = load_result(run_dir)
        xy_nom = _relative_xy(data, "nominal_ref")
        xy_ref = _relative_xy(data, "tool_ref")
        xy_pos = _relative_xy(data, "tool_pos")
        ax.plot(xy_nom[:, 0], xy_nom[:, 1], "k--", linewidth=1.8, label="名义轨迹")
        ax.plot(xy_ref[:, 0], xy_ref[:, 1], color="#d97706", linestyle="-.", linewidth=1.4, label="安全参考")
        ax.plot(xy_pos[:, 0], xy_pos[:, 1], color="#047857", linewidth=1.5, label="实际轨迹")
        ax.scatter([xy_nom[0, 0]], [xy_nom[0, 1]], color="#2563eb", s=18, zorder=5, label="起点")
        ax.set_title(title)
        ax.set_xlabel("x 位移 / mm")
        ax.set_ylabel("y 位移 / mm")
        ax.set_aspect("equal", adjustable="box")
    handles, labels = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False)
    fig.suptitle("第四章圆滑字母轨迹参考层仲裁结果", fontsize=14)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def plot_metric_summary(summary_rows, output_path):
    """绘制不同参考层仲裁策略的平均指标对比图。"""

    _setup_plot_style()
    labels = [row["label"] for row in summary_rows]
    x = np.arange(len(summary_rows))
    colors = ["#047857" if row["strategy"] == "full_method" else "#64748b" for row in summary_rows]
    specs = [
        ("tracking_last_rms_m_mean", 1000.0, "末段轨迹 RMS / mm"),
        ("F_peak_N_mean", 1.0, "接触力峰值误差 / N"),
        ("R_acc_mean", 1.0, "切向修正接受率"),
        ("R_sup_mean", 1.0, "危险法向抑制率"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10.8, 7.0), constrained_layout=True)
    for ax, (key, scale, ylabel) in zip(axes.ravel(), specs):
        vals = [scale * row.get(key, np.nan) for row in summary_rows]
        ax.bar(x, vals, color=colors)
        ax.set_ylabel(ylabel)
        ax.set_xticks(x, labels, rotation=18, ha="right")
        if "率" in ylabel:
            ax.set_ylim(-0.05, 1.10)
        ax.grid(True, axis="y")
    fig.suptitle("第四章圆滑字母轨迹对照实验平均指标", fontsize=14)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def plot_tracking_by_letter(rows, output_path):
    """按字母轨迹分组绘制末段跟踪误差。"""

    _setup_plot_style()
    strategies = ["direct_accept", "fixed_blend", "single_sigmoid", "dynamic_no_projection", "full_method"]
    strategies = [s for s in strategies if any(row["strategy"] == s for row in rows)]
    x = np.arange(len(LETTER_SCENARIOS))
    width = 0.8 / max(len(strategies), 1)
    palette = {
        "direct_accept": "#ef4444",
        "fixed_blend": "#94a3b8",
        "single_sigmoid": "#2563eb",
        "dynamic_no_projection": "#7c3aed",
        "full_method": "#047857",
    }
    fig, ax = plt.subplots(figsize=(9.6, 4.8), constrained_layout=True)
    for i, strategy in enumerate(strategies):
        vals = []
        for scenario, _ in LETTER_SCENARIOS:
            arr = [1000.0 * float(row["tracking_last_rms_m"]) for row in rows if row["scenario"] == scenario and row["strategy"] == strategy]
            vals.append(float(np.nanmean(arr)) if arr else np.nan)
        ax.bar(x + (i - (len(strategies) - 1) / 2.0) * width, vals, width=width,
               color=palette.get(strategy, "#64748b"), label=_label(strategy))
    ax.set_xticks(x, [title for _, title in LETTER_SCENARIOS])
    ax.set_ylabel("末段轨迹 RMS / mm")
    ax.set_title("不同圆滑字母轨迹下的参考层仲裁跟踪误差")
    ax.grid(True, axis="y")
    ax.legend(ncol=2, frameon=False)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)


def write_report(output_dir, rows, summary_rows, suite_dirs):
    """生成第四章圆滑字母轨迹实验 Markdown 报告。"""

    output_dir = Path(output_dir)
    report = output_dir / "CH4_SMOOTH_LETTER_GAZEBO_REPORT.md"
    method = next((row for row in summary_rows if row["strategy"] == "full_method"), None)
    lines = [
        "# 第四章 U/S/T/C 圆滑字母轨迹 Gazebo 对照实验报告",
        "",
        "## 实验目的",
        "",
        "本实验将 U、S、T、C 四类圆滑目标轨迹作为参考层仲裁任务，检验第四章方法在复杂切向路径、人类安全修正、危险法向压入和短时扰动同时存在时的参考生成能力。执行层控制器和接触环境保持固定，不同对照组只改变人类输入进入最终期望轨迹的方式。",
        "实物化版本采用真实数据约束的人输入模型：无交互时输入为 0；按下主端按钮后输入在约 0.18 s 内迅速升至峰值，随后保持；松开按钮后立即归零。控制器日志记录 `human_button` 和 `human_force_cmd_N`，用于核对交互时间窗。",
        "",
        "## 图表",
        "",
        "![圆滑字母轨迹仲裁结果](figures/ch4_smooth_letter_trajectory_grid.png)",
        "",
        "![平均指标对比](figures/ch4_smooth_letter_metric_summary.png)",
        "",
        "![分字母误差](figures/ch4_smooth_letter_tracking_by_letter.png)",
        "",
        "## 平均指标",
        "",
        "| 方法 | 成功次数 | 轨迹RMS/mm | 力峰值/N | 切向接受率 | 法向抑制率 | 权重平滑性 | 参考跳变/mm |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        lines.append(
            f"| {row['label']} | {row['success_count']}/{row['letters']} | "
            f"{1000.0 * row['tracking_last_rms_m_mean']:.2f} | "
            f"{row['F_peak_N_mean']:.3f} | {row['R_acc_mean']:.3f} | "
            f"{row['R_sup_mean']:.3f} | {row['S_alpha_mean']:.3f} | "
            f"{1000.0 * row['reference_jump_max_m_mean']:.2f} |"
        )
    lines.extend(["", "## 结果分析", ""])
    if method:
        lines.append(
            f"本文方法在四类圆滑字母轨迹中的成功次数为 {method['success_count']}/{method['letters']}，"
            f"平均末段轨迹 RMS 为 {1000.0 * method['tracking_last_rms_m_mean']:.2f} mm，"
            f"平均切向修正接受率为 {method['R_acc_mean']:.3f}，"
            f"平均危险法向输入抑制率为 {method['R_sup_mean']:.3f}。"
            "这些指标说明，动态仲裁和安全投影可以在保留有效人类切向修正的同时，削弱可能导致过压的法向输入。"
        )
    lines.append(
        "直接接受策略通常能够较快响应人类输入，但缺少安全投影，危险法向分量容易进入最终参考；"
        "固定融合策略给出固定折中，在不同字母曲率和输入阶段下缺少自适应性；"
        "无投影动态策略虽然具备权重变化，但无法从几何上约束危险法向输入。"
        "完整方法把安全投影、绝对通道、增量通道和滤波平滑结合起来，因此更适合包含方向切换和局部回转的复杂扫描任务。"
    )
    lines.extend(["", "## 原始 suite", ""])
    for suite_dir in suite_dirs:
        lines.append(f"- `{suite_dir}`")
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def analyze(output_dir, suite_dirs):
    """对已有或刚生成的 suite 目录重新计算指标并绘图。"""

    rows, run_dirs = _collect_rows(suite_dirs)
    if not rows:
        raise RuntimeError("没有找到第四章圆滑字母实验数据")
    output_dir = Path(output_dir)
    figures_dir = output_dir / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = _aggregate(rows)
    _write_csv(rows, output_dir / "ch4_smooth_letter_metrics.csv")
    _write_summary_csv(summary_rows, output_dir / "ch4_smooth_letter_summary.csv")
    plot_trajectory_grid(run_dirs, figures_dir / "ch4_smooth_letter_trajectory_grid.png")
    plot_metric_summary(summary_rows, figures_dir / "ch4_smooth_letter_metric_summary.png")
    plot_tracking_by_letter(rows, figures_dir / "ch4_smooth_letter_tracking_by_letter.png")
    report = write_report(output_dir, rows, summary_rows, suite_dirs)
    return {
        "output_dir": str(output_dir),
        "report": str(report),
        "metrics_csv": str(output_dir / "ch4_smooth_letter_metrics.csv"),
        "summary_csv": str(output_dir / "ch4_smooth_letter_summary.csv"),
        "figures_dir": str(figures_dir),
    }


def main(argv=None):
    """命令行入口：可在线运行 Gazebo，也可仅分析已有结果。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--run-gazebo", action="store_true", help="先运行四个字母 Gazebo 场景")
    parser.add_argument("--suite-dirs", nargs="*", default=[], help="已有 suite 或 run 目录")
    parser.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch4_smooth_letter_gazebo")
    parser.add_argument("--strategies", default="direct_accept,fixed_blend,single_sigmoid,dynamic_no_projection,full_method")
    parser.add_argument("--scenarios", default=",".join(name for name, _ in LETTER_SCENARIOS))
    parser.add_argument("--repeats", type=int, default=1)
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
    parser.add_argument("--settle-after-save-s", type=float, default=2.0)
    parser.add_argument("--realistic-env", action="store_true")
    parser.add_argument("--realistic-profile", default="real_env_0607_0213")
    parser.add_argument("--realistic-seed", type=int, default=20260703)
    parser.add_argument("--realistic-tau-noise-gain", type=float, default=0.12)
    parser.add_argument("--human-input-profile", default="")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--gz-args", default=None)
    args = parser.parse_args(argv)

    suite_dirs = [Path(p) for p in args.suite_dirs]
    scenario_names = {name.strip() for name in args.scenarios.split(",") if name.strip()}
    scenarios = [(name, label) for name, label in LETTER_SCENARIOS if name in scenario_names]
    if not scenarios:
        raise ValueError(f"没有合法场景: {args.scenarios}")
    if args.run_gazebo:
        for scenario, _ in scenarios:
            suite_dirs.append(_run_scenario(args, scenario))
    if not suite_dirs:
        raise ValueError("需要指定 --run-gazebo 或 --suite-dirs")
    result = analyze(args.output_dir, suite_dirs)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
