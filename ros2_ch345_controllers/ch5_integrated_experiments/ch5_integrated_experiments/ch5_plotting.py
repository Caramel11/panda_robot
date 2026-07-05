#!/usr/bin/env python3
"""第5章实验结果绘图模块。

本模块把 `ch5_gazebo_metrics.csv` 和典型 `result.npz` 转换为论文分析图。
它包含两类图：

1. `ch5_metrics_bars.png`：跨方法的指标柱状图，用于比较固定权重、
   仅参考层、仅执行层、安全投影消融和完整方法。
2. `ch5_representative_timeseries.png`：单次典型实验的时序图，用于说明
   `alpha_HR`、`alpha_FP`、力裕度、参考投影和末端轨迹之间的关系。

绘图后端使用 `Agg`，因此可以在没有显示器的 SSH/服务器环境中运行。
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


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
    "direct_accept": "直接接受",
    "rcm_direct_accept": "RCM直接接受",
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


def _setup_style():
    """设置中文论文图表风格。"""

    plt.rcParams.update({
        "font.sans-serif": [
            "Noto Sans CJK JP",
            "Noto Sans CJK SC",
            "Droid Sans Fallback",
            "AR PL UMing CN",
            "SimHei",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
        "figure.dpi": 140,
        "savefig.dpi": 240,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "grid.alpha": 0.28,
    })


def _label(method):
    return METHOD_LABELS.get(method, method)


def _read_csv(path):
    """读取指标 CSV，返回字典列表。"""

    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _load_npz(run_dir):
    """读取单次实验目录中的 `result.npz`。"""

    path = Path(run_dir) / "result.npz"
    data = np.load(path)
    return {k: np.asarray(data[k]) for k in data.files}


def _v3(data, prefix):
    """把 `prefix_0/1/2` 三个分量拼接成 `(N, 3)` 数组。"""

    return np.column_stack([data[f"{prefix}_{i}"] for i in range(3)])


def plot_metrics_bars(rows, out):
    """绘制跨方法综合指标柱状图。

    四个子图分别回答论文中的四类问题：
    - 综合 score：哪个方法整体代价更小；
    - tracking/force：收敛误差和力峰值是否满足安全要求；
    - R_acc/R_sup：是否既接受有益人类输入又抑制危险法向输入；
    - alpha_HR/alpha_FP：参考层和执行层权重是否真正参与调节。
    """

    _setup_style()
    labels = [_label(r["method"]) for r in rows]
    x = np.arange(len(rows))
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)

    axes[0, 0].bar(x, [float(r["score"]) for r in rows], color="tab:blue")
    axes[0, 0].set_ylabel("综合代价（越小越好）")
    axes[0, 0].grid(True, axis="y", alpha=0.3)

    threshold_mm = max([1000.0 * float(r.get("target_tolerance_m") or 0.003) for r in rows] or [3.0])
    axes[0, 1].bar(x - 0.18, [1000.0 * float(r["tracking_last_rms_m"]) for r in rows], width=0.35, label="轨迹RMS/mm")
    axes[0, 1].bar(x + 0.18, [float(r["F_peak_N"]) for r in rows], width=0.35, label="力峰值/N")
    axes[0, 1].axhline(threshold_mm, color="r", linestyle="--", linewidth=1, label=f"{threshold_mm:.0f} mm")
    axes[0, 1].set_ylabel("mm / N")
    axes[0, 1].legend()
    axes[0, 1].grid(True, axis="y", alpha=0.3)

    axes[1, 0].bar(x - 0.18, [float(r["R_acc"]) for r in rows], width=0.35, label="R_acc")
    axes[1, 0].bar(x + 0.18, [float(r["R_sup"]) for r in rows], width=0.35, label="R_sup")
    axes[1, 0].set_ylim(-0.2, 1.6)
    axes[1, 0].set_ylabel("比例")
    axes[1, 0].legend()
    axes[1, 0].grid(True, axis="y", alpha=0.3)

    axes[1, 1].bar(x - 0.18, [float(r["alpha_HR_mean"]) for r in rows], width=0.35, label="平均alpha_HR")
    axes[1, 1].bar(x + 0.18, [float(r["alpha_FP_mean"]) for r in rows], width=0.35, label="平均alpha_FP")
    axes[1, 1].set_ylim(0.0, 1.0)
    axes[1, 1].set_ylabel("权重")
    axes[1, 1].legend()
    axes[1, 1].grid(True, axis="y", alpha=0.3)

    for ax in axes.ravel():
        ax.set_xticks(x, labels, rotation=20, ha="right")
    fig.suptitle("第五章 Gazebo 综合实验指标")
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_representative(run_dir, out):
    """绘制一个典型实验 run 的时序图。

    典型图用于第5章正文解释完整方法的运行机理，而不只是展示最终数字。
    图中同时放入参考层变量和执行层变量，便于说明“第4章参考层 + 第3章
    执行层”在同一条控制链里如何协同。
    """

    _setup_style()
    data = _load_npz(run_dir)
    t = data["t"]
    tracking = data["tracking_error"] * 1000.0
    threshold_mm = 10.0 if ("y_f_realistic" in data or "realistic_env_enabled" in data) else 3.0
    alpha_hr = data.get("alpha", np.zeros_like(t))
    alpha_fp = data.get("alpha_FP", np.full_like(t, 0.5))
    rho = data.get("rho_F", np.ones_like(t))
    y_f = data.get("y_f_realistic", data.get("y_f", np.zeros_like(t)))
    f_min = data.get("F_min", np.full_like(t, 0.35))
    f_d = data.get("F_d", np.full_like(t, 0.7))
    f_max = data.get("F_max", np.full_like(t, 1.2))
    nominal = _v3(data, "nominal_ref")
    x_h = _v3(data, "x_h")
    x_safe = _v3(data, "x_h_safe")
    x_d = _v3(data, "tool_ref")
    tool = _v3(data, "tool_pos")
    normal_raw = data.get("normal_raw", np.zeros_like(t))
    normal_safe = data.get("normal_safe", np.zeros_like(t))

    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    # 轨迹误差图：红色虚线是收敛判据；实物化仿真按真实扰动放宽到 10 mm。
    ax = axes[0, 0]
    ax.plot(t, tracking, label="轨迹误差")
    ax.axhline(threshold_mm, color="r", linestyle="--", linewidth=1, label=f"{threshold_mm:.0f} mm")
    ax.set_ylabel("轨迹误差 / mm")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # 仲裁权重图：alpha_HR 是第4章参考层，alpha_FP 是第3章执行层。
    # rho_F 是力安全裕度，通常会影响两层策略的风险判断。
    ax = axes[0, 1]
    ax.plot(t, alpha_hr, label="alpha_HR")
    ax.plot(t, alpha_fp, label="alpha_FP")
    ax.plot(t, rho, label="rho_F", alpha=0.75)
    ax.set_ylim(-0.05, 1.05)
    ax.set_ylabel("权重")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # 力代理量图：f_min/f_d/f_max 分别是下界、期望值和上界。
    ax = axes[1, 0]
    ax.plot(t, y_f, label="接触力代理")
    ax.plot(t, f_min, "k--", linewidth=0.8)
    ax.plot(t, f_d, "k:", linewidth=0.8)
    ax.plot(t, f_max, "k--", linewidth=0.8)
    ax.set_ylabel("接触力 / N")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # 参考偏移图：比较人类候选参考、安全投影后参考和最终参考。
    ax = axes[1, 1]
    ax.plot(t, (x_h[:, 1] - nominal[:, 1]) * 1000.0, label="x_h tangent")
    ax.plot(t, (x_safe[:, 1] - nominal[:, 1]) * 1000.0, label="x_h_safe tangent")
    ax.plot(t, (x_d[:, 1] - nominal[:, 1]) * 1000.0, label="x_d tangent")
    ax.plot(t, normal_raw * 1000.0, "--", label="normal raw")
    ax.plot(t, normal_safe * 1000.0, "--", label="normal safe")
    ax.set_ylabel("参考偏移 / mm")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # XY 轨迹图：检查 nominal、人类参考、安全参考、最终参考和真实末端是否一致。
    ax = axes[2, 0]
    ax.plot(nominal[:, 0], nominal[:, 1], ":", label="x_r")
    ax.plot(x_h[:, 0], x_h[:, 1], "--", label="x_h")
    ax.plot(x_safe[:, 0], x_safe[:, 1], "-.", label="x_h_safe")
    ax.plot(x_d[:, 0], x_d[:, 1], label="x_d")
    ax.plot(tool[:, 0], tool[:, 1], color="k", linewidth=1, label="tool")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # 执行层图：比较 alpha_FP 目标值和滤波后的实际值，以及法向抑制系数 kappa_N。
    ax = axes[2, 1]
    ax.plot(t, data.get("alpha_FP_target", alpha_fp), label="alpha_FP target")
    ax.plot(t, alpha_fp, label="alpha_FP filtered")
    ax.plot(t, data.get("kappa_N", np.ones_like(t)), label="kappa_N")
    ax.set_xlabel("时间 / s")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.suptitle(Path(run_dir).name)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180)
    plt.close(fig)


def main(argv=None):
    """命令行入口：从 CSV 和可选代表性 run 目录生成图表。"""

    ap = argparse.ArgumentParser()
    ap.add_argument("--metrics-csv", required=True)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--representative-run", default=None)
    args = ap.parse_args(argv)

    rows = _read_csv(args.metrics_csv)
    out_dir = Path(args.output_dir)
    plot_metrics_bars(rows, out_dir / "ch5_metrics_bars.png")
    if args.representative_run:
        plot_representative(args.representative_run, out_dir / "ch5_representative_timeseries.png")
    print({
        "metrics": str(out_dir / "ch5_metrics_bars.png"),
        "representative": str(out_dir / "ch5_representative_timeseries.png") if args.representative_run else None,
    })


if __name__ == "__main__":
    main()
