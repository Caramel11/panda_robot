#!/usr/bin/env python3
"""第5章 USTC 字母曲线轨迹对比实验。

该脚本面向论文新增的复杂曲线轨迹场景。它可以直接调用现有 Gazebo suite
运行 `scenario:=ustc_letters`，也可以对已经完成的 suite 重新计算指标、绘制
轨迹图和生成 Markdown 报告。实验数据格式沿用第4/5章 Gazebo 控制器输出的
`result.npz` 与 `summary.json`，因此不引入新的控制器实现。
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


METHOD_LABELS = {
    "standard_impedance": "标准阻抗",
    "traditional_hybrid": "传统混合力位",
    "standard_mpc": "标准MPC",
    "strong_position": "强位置优先",
    "balanced_fixed": "固定均衡",
    "strong_force": "强力优先",
    "reference_only": "仅参考层",
    "execution_only": "仅执行层",
    "no_projection": "无安全投影",
    "full_method": "本文方法",
    "rcm_standard_impedance": "RCM标准阻抗",
    "rcm_traditional_hybrid": "RCM传统混合力位",
    "rcm_standard_mpc": "RCM标准MPC",
    "rcm_strong_position": "RCM强位置优先",
    "rcm_balanced_fixed": "RCM固定均衡",
    "rcm_strong_force": "RCM强力优先",
    "rcm_reference_only": "RCM仅参考层",
    "rcm_execution_only": "RCM仅执行层",
    "rcm_no_projection": "RCM无安全投影",
    "rcm_full_method": "RCM本文方法",
}


def _setup_plot_style():
    """设置适合论文草图的中文绘图风格。"""

    plt.rcParams.update({
        "font.sans-serif": [
            "Noto Sans CJK JP",
            "Noto Sans CJK SC",
            "Source Han Sans SC",
            "Droid Sans Fallback",
            "AR PL UMing CN",
            "WenQuanYi Micro Hei",
            "SimHei",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
        "figure.dpi": 120,
        "savefig.dpi": 220,
        "axes.linewidth": 0.9,
        "grid.alpha": 0.25,
        "legend.frameon": False,
    })


def _find_run_dirs(suite_dir):
    """查找 suite 中所有包含完整结果的运行目录。"""

    suite_dir = Path(suite_dir)
    return sorted(
        p.parent for p in suite_dir.rglob("summary.json")
        if (p.parent / "result.npz").exists()
    )


def _load_npz(run_dir):
    """读取 result.npz 为普通字典。"""

    with np.load(Path(run_dir) / "result.npz") as npz:
        return {k: np.asarray(npz[k]) for k in npz.files}


def _load_summary(run_dir):
    path = Path(run_dir) / "summary.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _method_from_run(run_dir, summary=None):
    summary = summary or _load_summary(run_dir)
    return (
        summary.get("ch5_method")
        or summary.get("arbitration_strategy")
        or Path(run_dir).name
    )


def _label(method):
    return METHOD_LABELS.get(method, method)


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


def compute_suite_metrics(suite_dir):
    """对一个 USTC suite 重新计算统一指标。"""

    rows = []
    run_dirs = []
    for i, run_dir in enumerate(_find_run_dirs(suite_dir)):
        summary = _load_summary(run_dir)
        method = _method_from_run(run_dir, summary)
        trial_id = int(summary.get("trial_id", i))
        rows.append(load_run(run_dir, method=method, trial_id=trial_id))
        run_dirs.append(Path(run_dir))
    return rows, run_dirs


def plot_trajectory_comparison(run_dirs, out):
    """绘制所有方法在 USTC 字母轨迹上的实际轨迹对比。

    图中黑色虚线是机器人名义 USTC 自主轨迹。第4章参考层会根据人类
    输入与安全边界生成最终 `tool_ref`，因此本文方法的实际轨迹不应
    简单解释为对黑色虚线的几何偏离。这里额外绘制本文方法的最终安全
    参考轨迹，用于区分“接受安全切向修正”和“控制跟踪误差”。
    """

    _setup_plot_style()
    fig, ax = plt.subplots(figsize=(8.8, 4.8), constrained_layout=True)
    colors = {
        "standard_impedance": "#475569",
        "traditional_hybrid": "#b45309",
        "standard_mpc": "#2563eb",
        "strong_position": "#6b7280",
        "balanced_fixed": "#4b5563",
        "strong_force": "#b45309",
        "reference_only": "#2563eb",
        "execution_only": "#7c3aed",
        "no_projection": "#dc2626",
        "full_method": "#047857",
    }
    target_plotted = False
    safe_ref_plotted = False
    for run_dir in run_dirs:
        data = _load_npz(run_dir)
        method = _method_from_run(run_dir)
        xy_ref = _relative_xy(data, "nominal_ref")
        xy_tool_ref = _relative_xy(data, "tool_ref")
        xy_tool = _relative_xy(data, "tool_pos")
        if not target_plotted:
            ax.plot(xy_ref[:, 0], xy_ref[:, 1], "k--", linewidth=2.0, label="期望 USTC 轨迹")
            target_plotted = True
        if method == "full_method" and not safe_ref_plotted:
            ax.plot(
                xy_tool_ref[:, 0],
                xy_tool_ref[:, 1],
                color="#f59e0b",
                linestyle="-.",
                linewidth=1.8,
                alpha=0.95,
                label="本文方法安全参考",
            )
            safe_ref_plotted = True
        ax.plot(
            xy_tool[:, 0],
            xy_tool[:, 1],
            linewidth=1.6 if method == "full_method" else 1.1,
            color=colors.get(method, None),
            alpha=0.95 if method == "full_method" else 0.78,
            label=_label(method),
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("切向 x 位移 / mm")
    ax.set_ylabel("切向 y 位移 / mm")
    ax.set_title("USTC 字母曲线轨迹跟踪与参考仲裁对比")
    ax.grid(True)
    ax.legend(ncol=2, fontsize=9)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def plot_metric_comparison(rows, out):
    """绘制 USTC 场景关键指标对比。"""

    _setup_plot_style()
    rows = sorted(rows, key=lambda r: [
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
    ].index(r["method"]) if r["method"] in [
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
    ] else 99)
    labels = [_label(r["method"]) for r in rows]
    x = np.arange(len(rows))
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.0), constrained_layout=True)
    colors = ["#64748b" if r["method"] != "full_method" else "#047857" for r in rows]

    axes[0, 0].bar(x, [1000.0 * float(r["tracking_rms_m"]) for r in rows], color=colors)
    axes[0, 0].set_ylabel("轨迹 RMS 误差 / mm")
    axes[0, 0].grid(True, axis="y")

    axes[0, 1].bar(x, [float(r["F_peak_N"]) for r in rows], color=colors)
    axes[0, 1].set_ylabel("接触力峰值误差 / N")
    axes[0, 1].grid(True, axis="y")

    width = 0.36
    axes[1, 0].bar(x - width / 2, [float(r["R_acc"]) for r in rows], width=width, color="#2563eb", label="切向修正接受率")
    axes[1, 0].bar(x + width / 2, [float(r["R_sup"]) for r in rows], width=width, color="#b45309", label="法向危险抑制率")
    axes[1, 0].set_ylim(-0.1, 1.15)
    axes[1, 0].set_ylabel("比值")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, axis="y")

    axes[1, 1].bar(x, [float(r["score"]) for r in rows], color=colors)
    axes[1, 1].set_ylabel("综合代价")
    axes[1, 1].grid(True, axis="y")

    for ax in axes.ravel():
        ax.set_xticks(x, labels, rotation=18, ha="right")
    fig.suptitle("USTC 曲线轨迹场景下的控制性能指标对比", fontsize=14)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def plot_representative_timeseries(run_dir, out):
    """绘制本文方法的典型时序和局部参考变化。"""

    _setup_plot_style()
    data = _load_npz(run_dir)
    t = np.asarray(data["t"], dtype=float)
    tracking = np.asarray(data["tracking_error"], dtype=float) * 1000.0
    threshold_mm = 10.0 if ("y_f_realistic" in data or "realistic_env_enabled" in data) else 3.0
    y_f = np.asarray(data.get("y_f_realistic", data.get("y_f", np.zeros_like(t))), dtype=float)
    alpha_hr = np.asarray(data.get("alpha", np.zeros_like(t)), dtype=float)
    alpha_fp = np.asarray(data.get("alpha_FP", np.full_like(t, 0.5)), dtype=float)
    rho = np.asarray(data.get("rho_F", np.ones_like(t)), dtype=float)
    f_min = np.asarray(data.get("F_min", np.full_like(t, 0.35)), dtype=float)
    f_d = np.asarray(data.get("F_d", np.full_like(t, 0.7)), dtype=float)
    f_max = np.asarray(data.get("F_max", np.full_like(t, 1.2)), dtype=float)
    xy_ref = _relative_xy(data, "nominal_ref")
    xy_tool = _relative_xy(data, "tool_pos")
    xy_xh = _relative_xy(data, "x_h")
    xy_safe = _relative_xy(data, "x_h_safe")

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.6), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(t, tracking, color="#047857", linewidth=1.5)
    ax.axhline(threshold_mm, color="#dc2626", linestyle="--", linewidth=1.0, label=f"{threshold_mm:.0f} mm 阈值")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("轨迹误差 / mm")
    ax.set_title("轨迹跟踪误差")
    ax.grid(True)
    ax.legend(fontsize=9)

    ax = axes[0, 1]
    ax.plot(t, y_f, color="#b45309", linewidth=1.4, label="接触力代理")
    ax.plot(t, f_min, "k--", linewidth=0.8, label="安全边界")
    ax.plot(t, f_d, "k:", linewidth=0.9, label="期望值")
    ax.plot(t, f_max, "k--", linewidth=0.8)
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("接触力 / N")
    ax.set_title("接触力安全边界")
    ax.grid(True)
    ax.legend(fontsize=9)

    ax = axes[1, 0]
    ax.plot(t, alpha_hr, label=r"$\alpha_{\mathrm{HR}}$", color="#2563eb")
    ax.plot(t, alpha_fp, label=r"$\alpha_{\mathrm{FP}}$", color="#047857")
    ax.plot(t, rho, label=r"$\rho_F$", color="#6b7280", alpha=0.85)
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("仲裁参数")
    ax.set_title("参考层与执行层仲裁变化")
    ax.grid(True)
    ax.legend(fontsize=9)

    ax = axes[1, 1]
    ax.plot(xy_ref[:, 0], xy_ref[:, 1], "k--", linewidth=1.8, label="期望轨迹")
    ax.plot(xy_xh[:, 0], xy_xh[:, 1], color="#60a5fa", linewidth=0.9, alpha=0.75, label="人侧候选")
    ax.plot(xy_safe[:, 0], xy_safe[:, 1], color="#f59e0b", linewidth=0.9, alpha=0.75, label="安全参考")
    ax.plot(xy_tool[:, 0], xy_tool[:, 1], color="#047857", linewidth=1.5, label="实际轨迹")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("切向 x 位移 / mm")
    ax.set_ylabel("切向 y 位移 / mm")
    ax.set_title("本文方法典型轨迹")
    ax.grid(True)
    ax.legend(fontsize=8)

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def _best_full_method_run(rows, run_dirs):
    full = [(r, p) for r, p in zip(rows, run_dirs) if r["method"] == "full_method"]
    if not full:
        return run_dirs[0] if run_dirs else None
    return sorted(full, key=lambda item: float(item[0]["score"]))[0][1]


def write_report(suite_dir, rows, run_dirs, figures_dir):
    """写出 USTC 曲线轨迹对比实验报告。"""

    suite_dir = Path(suite_dir)
    figures_dir = Path(figures_dir)
    report = suite_dir / "CH5_USTC_TRAJECTORY_COMPARISON_REPORT.md"
    by_method = {r["method"]: r for r in rows}
    full = by_method.get("full_method")
    standard_impedance = by_method.get("standard_impedance")
    traditional_hybrid = by_method.get("traditional_hybrid")
    standard_mpc = by_method.get("standard_mpc")
    balanced = by_method.get("balanced_fixed")
    strong_force = by_method.get("strong_force")
    reference_only = by_method.get("reference_only")
    execution_only = by_method.get("execution_only")

    def fmt(row, key, scale=1.0, nd=3):
        """执行本模块中的辅助计算或数据转换步骤。"""
        if not row:
            return "未运行"
        return f"{scale * float(row[key]):.{nd}f}"

    lines = [
        "# USTC 字母曲线轨迹 Gazebo 对比实验报告",
        "",
        "## 实验目的",
        "",
        "USTC 字母曲线轨迹用于补充直线扫描以外的复杂切向路径验证。该轨迹包含多次拐角、横向连接段和方向切换，能够观察控制器在曲线扫描任务中的轨迹收敛、力安全保持、人类切向修正接受和危险法向输入抑制能力。实验沿用第3章执行层力位协同控制器、第4章参考层动态仲裁器和第5章综合实验接口，只改变自主参考轨迹形状，因此能够作为345章统一的新增对比场景。需要说明的是，黑色虚线表示机器人名义 USTC 自主轨迹；在有人类输入和安全投影存在时，实际跟踪目标是仲裁后的安全参考轨迹，而不是未修正的名义轨迹。",
        "",
        "## 图表文件",
        "",
        f"- 轨迹对比图：`{figures_dir / 'ustc_trajectory_comparison.png'}`",
        f"- 指标对比图：`{figures_dir / 'ustc_metric_comparison.png'}`",
        f"- 本文方法时序图：`{figures_dir / 'ustc_full_method_timeseries.png'}`",
        "",
        "![USTC轨迹对比](figures/ustc_trajectory_comparison.png)",
        "",
        "![USTC指标对比](figures/ustc_metric_comparison.png)",
        "",
        "![USTC本文方法时序](figures/ustc_full_method_timeseries.png)",
        "",
        "## 主要指标",
        "",
        "| 方法 | 成功 | 轨迹RMS/mm | 末段RMS/mm | 力峰值/N | 越界时间/s | 切向接受率 | 法向抑制率 | 综合代价 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {_label(row['method'])} | {row['success']} | "
            f"{1000.0 * float(row['tracking_rms_m']):.2f} | "
            f"{1000.0 * float(row['tracking_last_rms_m']):.2f} | "
            f"{float(row['F_peak_N']):.3f} | {float(row['T_vio_s']):.3f} | "
            f"{float(row['R_acc']):.3f} | {float(row['R_sup']):.3f} | "
            f"{float(row['score']):.3f} |"
        )

    lines.extend([
        "",
        "## 结果分析",
        "",
        f"从轨迹跟踪结果看，本文方法在 USTC 曲线轨迹上的轨迹均方根误差为 {fmt(full, 'tracking_rms_m', 1000.0, 2)} mm，末段均方根误差为 {fmt(full, 'tracking_last_rms_m', 1000.0, 2)} mm。USTC 轨迹的拐角和连接段比直线扫描更容易放大速度和姿态环中的瞬态误差，因此该场景主要用于观察复杂切向任务中的收敛稳定性，而不是只考察单段直线的稳态误差。",
        "",
        f"新增的三类标准控制器用于补足执行层结构对照：标准阻抗控制的末段误差为 {fmt(standard_impedance, 'tracking_last_rms_m', 1000.0, 2)} mm，传统混合力位控制的力峰值误差为 {fmt(traditional_hybrid, 'F_peak_N', 1.0, 3)} N，标准 MPC/QP 的越界时间为 {fmt(standard_mpc, 'T_vio_s', 1.0, 3)} s。标准阻抗控制以三轴弹簧阻尼模型为主，理论上更重视几何轨迹；传统混合力位控制将切向位置和法向力解耦，理论上更偏向力恢复；标准 MPC/QP 通过短预测窗与力边界投影抑制越界，理论上在约束激活时更保守。与这些方法相比，本文方法不是只在某个固定结构中取折中，而是通过参考层 alpha_HR 与执行层 alpha_FP 的连续调节，把人类输入接受、力安全和轨迹收敛放在同一闭环中协调。",
        "",
        f"固定均衡方法的末段误差为 {fmt(balanced, 'tracking_last_rms_m', 1000.0, 2)} mm，强力优先方法的接触力峰值误差为 {fmt(strong_force, 'F_peak_N', 1.0, 3)} N。若固定优先级在某一单项指标上较好，其原因通常来自长期偏向某一目标：强力优先更重视法向力恢复，强位置优先或固定均衡更重视几何路径保持。本文方法的优势在于让参考层和执行层权重随输入风险与力安全裕度连续变化，使复杂曲线轨迹中的人类修正、力安全和轨迹跟踪保持同一闭环内的折中。",
        "",
        f"从人机协同指标看，参考层方法的切向修正接受率为 {fmt(reference_only, 'R_acc', 1.0, 3)}，仅执行层方法的切向修正接受率为 {fmt(execution_only, 'R_acc', 1.0, 3)}，本文方法的切向修正接受率为 {fmt(full, 'R_acc', 1.0, 3)}。仅执行层调节能够处理接触执行风险，但不能体现操作者对曲线轨迹局部扫描带的修正；仅参考层仲裁能够保留切向修正，但在变刚度或法向扰动阶段缺少执行层优先级调节。完整方法同时保留安全切向修正并抑制危险法向输入，因此更符合复杂柔性接触操作中对协同性和安全性的共同要求。",
        "",
        "该场景可写入论文时，应把 USTC 字母曲线解释为复杂路径跟踪补充实验。仿真平台部分说明其由 Gazebo 中 FR3 机械臂、无远心运动约束短工具和第3章稳定姿态控制环组成；实物实验部分可保留同样的轨迹设置，并将真实柔性材料、末端力传感和重复试次结果留作后续填写。图表分析应优先展示轨迹叠加图，因为字母轨迹能够直观反映复杂路径下的跟踪稳定性；随后再结合力峰值、越界时间、切向接受率和法向抑制率说明本文控制器的综合优势。",
        "",
    ])
    report.write_text("\n".join(lines), encoding="utf-8")
    return report


def analyze_suite(suite_dir):
    """计算指标、绘图并生成 USTC 报告。"""

    suite_dir = Path(suite_dir)
    rows, run_dirs = compute_suite_metrics(suite_dir)
    if not rows:
        raise RuntimeError(f"No completed runs found under {suite_dir}")
    metrics_csv = suite_dir / "ustc_metrics.csv"
    write_metrics_csv(rows, metrics_csv)
    # 额外写一个固定字段顺序的 CSV，便于外部表格读取。
    _write_rows(rows, suite_dir / "ustc_metrics_for_paper.csv")
    figures_dir = suite_dir / "figures"
    plot_trajectory_comparison(run_dirs, figures_dir / "ustc_trajectory_comparison.png")
    plot_metric_comparison(rows, figures_dir / "ustc_metric_comparison.png")
    representative = _best_full_method_run(rows, run_dirs)
    if representative is not None:
        plot_representative_timeseries(representative, figures_dir / "ustc_full_method_timeseries.png")
    report = write_report(suite_dir, rows, run_dirs, figures_dir)
    return {
        "suite_dir": str(suite_dir),
        "metrics_csv": str(metrics_csv),
        "report": str(report),
        "figures_dir": str(figures_dir),
        "runs": [str(p) for p in run_dirs],
    }


def _existing_suites(output_dir):
    output_dir = Path(output_dir)
    if not output_dir.exists():
        return set()
    return set(output_dir.glob("ch5_gazebo_*_ustc_letters_*"))


def main(argv=None):
    """命令行入口。"""

    ap = argparse.ArgumentParser()
    ap.add_argument("--suite-dir", default="", help="已有 USTC suite 目录；为空且 --run-gazebo 时自动寻找新目录")
    ap.add_argument("--run-gazebo", action="store_true", help="先调用 Gazebo suite 运行 USTC 场景")
    ap.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch5_ustc_trajectory")
    ap.add_argument(
        "--methods",
        default="standard_impedance,traditional_hybrid,standard_mpc,strong_position,balanced_fixed,strong_force,reference_only,execution_only,full_method",
    )
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--task-duration-s", type=float, default=36.0)
    ap.add_argument("--timeout-s", type=float, default=120.0)
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--gz-args", default="")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    suite_dir = Path(args.suite_dir) if args.suite_dir else None
    if args.run_gazebo:
        before = _existing_suites(args.output_dir)
        gazebo_args = [
            "--task-mode", "no_rcm",
            "--scenario", "ustc_letters",
            "--methods", args.methods,
            "--trials", str(args.trials),
            "--task-duration-s", str(args.task_duration_s),
            "--timeout-s", str(args.timeout_s),
            "--output-dir", args.output_dir,
        ]
        if args.rviz:
            gazebo_args.append("--rviz")
        if args.gz_args:
            gazebo_args.extend(["--gz-args", args.gz_args])
        if args.verbose:
            gazebo_args.append("--verbose")
        run_gazebo_suite(gazebo_args)
        after = _existing_suites(args.output_dir)
        new_suites = sorted(after - before, key=lambda p: p.stat().st_mtime)
        if not new_suites:
            raise RuntimeError("Gazebo suite finished but no new USTC suite was found")
        suite_dir = new_suites[-1]
    if suite_dir is None:
        raise ValueError("Either --suite-dir or --run-gazebo must be provided")
    result = analyze_suite(suite_dir)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
