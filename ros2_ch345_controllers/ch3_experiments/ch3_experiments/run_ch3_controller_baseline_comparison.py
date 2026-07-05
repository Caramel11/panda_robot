"""第三章控制器结构对照实验入口。

该脚本把本地文档和历史 ROS1/ROS2 代码中已经存在的第三章对照控制器
整理成一组可复现实验：阻抗控制、传统混合力位控制、固定 alpha 合作博弈、
动态 alpha 合作博弈和 MPC/QP 风格约束基线。脚本运行纯 Python 柔性接触仿真，
复用 no_rcm 的指标分析口径，并额外生成面向论文写作的中文图表和 Markdown 报告。
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path

import numpy as np

from .ch3_contact_sim import run_contact_simulation
from .ch3_metrics import analyze_with_no_rcm
from .config import load_config


DEFAULT_CONTROLLERS = (
    "standard_impedance",
    "traditional_hybrid",
    "fixed_02",
    "fixed_05",
    "fixed_08",
    "continuous_force_margin",
    "standard_mpc",
)

DISPLAY_NAMES = {
    "impedance": "阻抗/位置优先",
    "standard_impedance": "标准阻抗",
    "hybrid": "传统混合力位",
    "traditional_hybrid": "传统混合力位",
    "fixed_0.2": "固定GT(alpha=0.2)",
    "fixed_0.5": "固定GT(alpha=0.5)",
    "fixed_0.8": "固定GT(alpha=0.8)",
    "dynamic_gt": "本文动态GT",
    "continuous_force_margin": "本文方法",
    "mpc_qp": "MPC/QP约束基线",
    "standard_mpc": "标准MPC/QP",
}

CONTROLLER_EVIDENCE = [
    (
        "阻抗/位置优先",
        "src/ch3_experiments/ch3_experiments/controllers/impedance.py",
        "固定阻抗或位置优先基线，用于观察不显式闭合力环时的力安全代价。",
    ),
    (
        "传统混合力位",
        "src/ch3_experiments/ch3_experiments/controllers/hybrid_force_position.py",
        "切向位置控制、法向力控制，对应经典 hybrid force/position control 基线。",
    ),
    (
        "固定GT(alpha=0.2/0.5/0.8)",
        "src/ch3_experiments/ch3_experiments/controllers/fixed_gt.py",
        "固定力位优先级合作博弈，用于和动态 alpha_FP 调节律形成消融对照。",
    ),
    (
        "本文方法",
        "src/ch3_experiments/ch3_experiments/controllers/dynamic_gt.py",
        "复用 no-RCM 力位混合控制器和 continuous_force_margin 策略，根据力安全裕度与刚度估计在线调节 alpha_FP，是第三章主方法。",
    ),
    (
        "MPC/QP约束基线",
        "src/ch3_experiments/ch3_experiments/controllers/mpc_qp.py",
        "轻量预测/约束修正基线，对应 ROS1 0213 中 GT 与 MPC 的历史对照思想。",
    ),
    (
        "ROS1历史对照",
        "panda_robot_gt_controller_dev/tests/0213/real_GT_KF.py, real_GT_Sigmoid.py, real_MPC_KF.py, real_MPC_Sigmoid.py",
        "历史第三章包含 GT+KF、GT+Sigmoid、MPC+KF、MPC+Sigmoid 四方法对照，可作为论文对照来源说明。",
    ),
]


def _try_setup_matplotlib():
    """延迟导入 matplotlib，并尽量配置中文字体。"""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    font_candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/home/liu/.TinyTeX/texmf-dist/fonts/opentype/public/fandol/FandolHei-Regular.otf",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    ]
    selected_font = None
    for font_path in font_candidates:
        path = Path(font_path)
        if path.exists():
            try:
                font_manager.fontManager.addfont(str(path))
                if selected_font is None:
                    selected_font = font_manager.FontProperties(fname=str(path)).get_name()
            except Exception:
                pass

    plt.rcParams["font.family"] = selected_font or "sans-serif"
    plt.rcParams["font.sans-serif"] = [
        selected_font or "Noto Sans CJK JP",
        "Noto Sans CJK JP",
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
        "AR PL UMing CN",
        "FandolHei",
        "Droid Sans Fallback",
        "SimHei",
        "Microsoft YaHei",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 150
    return plt


def _read_csv_rows(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _float(row, key, default=np.nan):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def _display(strategy):
    return DISPLAY_NAMES.get(strategy, strategy)


def _strategy_order(rows):
    desired = [
        "impedance",
        "standard_impedance",
        "hybrid",
        "traditional_hybrid",
        "fixed_0.2",
        "fixed_0.5",
        "fixed_0.8",
        "mpc_qp",
        "standard_mpc",
        "continuous_force_margin",
        "dynamic_gt",
    ]
    present = {row["strategy"] for row in rows}
    return [name for name in desired if name in present] + sorted(present - set(desired))


def _plot_metric_bars(summary_rows, figure_dir):
    """绘制论文用多指标柱状图。"""

    plt = _try_setup_matplotlib()
    rows_by_strategy = {row["strategy"]: row for row in summary_rows}
    strategies = _strategy_order(summary_rows)
    labels = [_display(s) for s in strategies]

    metrics = [
        ("force_rmse_N_mean", "接触力RMSE/N", "越小越好"),
        ("force_violation_time_s_mean", "力越界时间/s", "越小越好"),
        ("position_rmse_mm_mean", "位置RMSE/mm", "越小越好"),
        ("force_jitter_N_mean", "接触力抖动/N", "越小越好"),
    ]
    colors = ["#5B8DB8", "#D08A48", "#6DA06F", "#8A6FB0"]
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.2), constrained_layout=True)
    for ax, (key, ylabel, note), color in zip(axes.ravel(), metrics, colors):
        values = [_float(rows_by_strategy[s], key) for s in strategies]
        bars = ax.bar(np.arange(len(values)), values, color=color, alpha=0.88, width=0.72)
        ax.set_ylabel(ylabel)
        ax.set_title(note, fontsize=11)
        ax.set_xticks(np.arange(len(values)))
        ax.set_xticklabels(labels, rotation=25, ha="right")
        ax.grid(axis="y", linestyle="--", alpha=0.28)
        for bar, val in zip(bars, values):
            if np.isfinite(val):
                ax.text(
                    bar.get_x() + bar.get_width() / 2.0,
                    val,
                    f"{val:.3g}",
                    ha="center",
                    va="bottom",
                    fontsize=8,
                )
    fig.suptitle("第三章不同控制器结构对照指标", fontsize=14)
    out = Path(figure_dir) / "ch3_controller_baseline_metric_bars_cn.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def _plot_zone_bars(trial_rows, figure_dir):
    """绘制软/硬刚度区内的控制器误差对比。"""

    plt = _try_setup_matplotlib()
    zones = ("soft_K300", "stiff_K500")
    strategies = _strategy_order([row for row in trial_rows if row.get("zone") == "all"])
    labels = [_display(s) for s in strategies]

    grouped = {}
    for row in trial_rows:
        zone = row.get("zone")
        strategy = row.get("strategy")
        if zone not in zones or strategy not in strategies:
            continue
        grouped.setdefault((strategy, zone), []).append(row)

    def mean_metric(strategy, zone, key):
        vals = [_float(row, key) for row in grouped.get((strategy, zone), [])]
        vals = [v for v in vals if np.isfinite(v)]
        return float(np.mean(vals)) if vals else np.nan

    x = np.arange(len(strategies))
    width = 0.38
    fig, axes = plt.subplots(2, 1, figsize=(12.0, 7.2), sharex=True, constrained_layout=True)
    for ax, key, ylabel in [
        (axes[0], "force_rmse_N", "接触力RMSE/N"),
        (axes[1], "normal_pos_rmse_mm", "法向位置RMSE/mm"),
    ]:
        soft = [mean_metric(s, zones[0], key) for s in strategies]
        stiff = [mean_metric(s, zones[1], key) for s in strategies]
        ax.bar(x - width / 2.0, soft, width, label="软区 K=300 N/m", color="#6DA06F", alpha=0.88)
        ax.bar(x + width / 2.0, stiff, width, label="硬区 K=500 N/m", color="#B45D5D", alpha=0.88)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", linestyle="--", alpha=0.28)
        ax.legend(frameon=False, ncol=2)
    axes[-1].set_xticks(x)
    axes[-1].set_xticklabels(labels, rotation=25, ha="right")
    fig.suptitle("不同刚度区域内的控制器误差对比", fontsize=14)
    out = Path(figure_dir) / "ch3_controller_baseline_zone_bars_cn.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def _plot_representative_timeseries(result_dir, figure_dir):
    """绘制代表性试次的时序对比图。"""

    plt = _try_setup_matplotlib()
    representative = [
        ("standard_impedance", "标准阻抗"),
        ("traditional_hybrid", "传统混合力位"),
        ("fixed_05", "固定GT(alpha=0.5)"),
        ("standard_mpc", "标准MPC/QP"),
        ("continuous_force_margin", "本文方法"),
        ("dynamic_gt", "本文动态GT"),
    ]
    data = []
    for controller, label in representative:
        path = Path(result_dir) / f"{controller}_t00.npz"
        if path.exists():
            data.append((label, np.load(path, allow_pickle=True)))
    if not data:
        return None

    fig, axes = plt.subplots(4, 1, figsize=(12.0, 9.2), sharex=True, constrained_layout=True)
    for label, arr in data:
        t = arr["t"]
        axes[0].plot(t, arr["F_measured"], label=label, linewidth=1.2)
        normal_err_mm = 1000.0 * arr["pos_err_z"]
        axes[1].plot(t, normal_err_mm, label=label, linewidth=1.2)
        axes[2].plot(t, arr["alpha"], label=label, linewidth=1.2)
    t0 = data[0][1]["t"]
    axes[0].plot(t0, data[0][1]["F_desired"], "k--", linewidth=1.0, label="期望力")
    axes[0].fill_between(t0, data[0][1]["F_min"], data[0][1]["F_max"], color="#999999", alpha=0.12, label="力安全边界")
    axes[3].plot(t0, data[0][1]["K_env_true"], color="#333333", linewidth=1.4)

    axes[0].set_ylabel("接触力/N")
    axes[1].set_ylabel("法向误差/mm")
    axes[2].set_ylabel("alpha_FP")
    axes[3].set_ylabel("真实刚度/(N/m)")
    axes[3].set_xlabel("时间/s")
    for ax in axes:
        ax.grid(True, linestyle="--", alpha=0.28)
    axes[0].legend(frameon=False, ncol=3, fontsize=8)
    fig.suptitle("代表性试次时序对比", fontsize=14)
    out = Path(figure_dir) / "ch3_controller_baseline_timeseries_cn.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    return out


def _relative(path, base):
    try:
        return Path(path).resolve().relative_to(Path(base).resolve()).as_posix()
    except ValueError:
        return Path(path).as_posix()


def _format_table_value(value):
    if not np.isfinite(value):
        return "-"
    if abs(value) >= 100:
        return f"{value:.2f}"
    if abs(value) >= 10:
        return f"{value:.3f}"
    return f"{value:.4f}"


def _write_report(result_dir, analysis, figures, cfg, command, summary_rows):
    """生成第三章控制器结构对照补充报告。"""

    report = Path(result_dir) / "CH3_CONTROLLER_BASELINE_COMPARISON_REPORT.md"
    rows_by_strategy = {row["strategy"]: row for row in summary_rows}
    strategies = _strategy_order(summary_rows)
    force_sorted = sorted(
        strategies, key=lambda s: _float(rows_by_strategy[s], "force_rmse_N_mean", np.inf)
    )
    pos_sorted = sorted(
        strategies, key=lambda s: _float(rows_by_strategy[s], "position_rmse_mm_mean", np.inf)
    )
    vio_sorted = sorted(
        strategies,
        key=lambda s: _float(rows_by_strategy[s], "force_violation_time_s_mean", np.inf),
    )

    lines = [
        "# 第三章控制器结构对照实验补充报告",
        "",
        f"生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "## 1. 文档与代码检索结论",
        "",
        "本次检索确认：第三章已有可作为对照组的控制器代码，并非只能比较本文动态合作博弈方法。"
        "这些代码分布在当前 ROS2 `ch3_experiments` 包和 ROS1 历史 `tests/0213` 目录中。"
        "其中 ROS2 包已经实现了统一数据字段、统一柔性接触环境和统一指标分析流程，适合作为当前论文第三章的主要对照实验入口。",
        "",
        "| 对照组 | 本地依据 | 用途 |",
        "| --- | --- | --- |",
    ]
    for name, path, usage in CONTROLLER_EVIDENCE:
        lines.append(f"| {name} | `{path}` | {usage} |")

    lines += [
        "",
        "由此，第三章建议新增“控制器结构对照实验”：在相同扫描轨迹、相同期望接触力、相同分段柔性表面下，"
        "比较阻抗控制、传统混合力位控制、固定优先级合作博弈、MPC/QP 风格约束基线和本文动态优先级合作博弈。",
        "",
        "## 2. 实验设置",
        "",
        f"- 扫描时间：{cfg.scan_duration:.1f} s，采样周期：{cfg.dt:.3f} s。",
        f"- 扫描起点：`{np.array2string(cfg.scan_start, precision=4)}`，扫描终点：`{np.array2string(cfg.scan_end, precision=4)}`。",
        f"- 期望接触力：{cfg.force_desired:.3f} N，安全边界：[{cfg.force_min:.3f}, {cfg.force_max:.3f}] N。",
        "- 柔性表面：两段 Kelvin-Voigt 环境，软区 `K=300 N/m`，硬区 `K=500 N/m`。",
        f"- 每种控制器重复次数：{cfg.repeats}。",
        "",
        "运行命令：",
        "",
        "```bash",
        command,
        "```",
        "",
        "## 3. 对照结果汇总",
        "",
        "| 控制器 | 力RMSE/N | 力越界时间/s | 位置RMSE/mm | 力抖动/N | alpha均值 | 平均计算时间/ms |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for strategy in strategies:
        row = rows_by_strategy[strategy]
        lines.append(
            "| "
            + _display(strategy)
            + " | "
            + " | ".join(
                [
                    _format_table_value(_float(row, "force_rmse_N_mean")),
                    _format_table_value(_float(row, "force_violation_time_s_mean")),
                    _format_table_value(_float(row, "position_rmse_mm_mean")),
                    _format_table_value(_float(row, "force_jitter_N_mean")),
                    _format_table_value(_float(row, "alpha_mean_mean")),
                    _format_table_value(_float(row, "compute_time_avg_ms_mean")),
                ]
            )
            + " |"
        )

    lines += [
        "",
        "从单项指标看，力跟踪最小值不一定由本文动态方法取得：偏力控制的固定低 alpha 和传统混合力位控制在法向力 RMSE 上天然更占优，"
        "但代价是位置误差和任务执行柔顺性变差；位置优先阻抗控制在位置误差上占优，但力误差和越界时间显著增加。"
        "第三章需要强调的是综合折中和刚度切换适应性，而不是单个指标绝对最小。",
        "",
        f"- 力 RMSE 最小的前三种方法：{', '.join(_display(s) for s in force_sorted[:3])}。",
        f"- 位置 RMSE 最小的前三种方法：{', '.join(_display(s) for s in pos_sorted[:3])}。",
        f"- 力越界时间最小的前三种方法：{', '.join(_display(s) for s in vio_sorted[:3])}。",
        "",
        "## 4. 图表结果",
        "",
    ]
    for fig in figures:
        if fig:
            rel = _relative(fig, result_dir)
            lines += [f"![{Path(fig).stem}]({rel})", ""]

    lines += [
        "## 5. 结果含义与论文补充建议",
        "",
        "第三章目前可以补充如下对比逻辑：",
        "",
        "1. `impedance` 作为位置优先控制器，说明仅依靠阻抗或位置误差闭环时，柔性接触力在硬区更容易接近安全边界，适合作为“无显式力位仲裁”的下限基线。",
        "2. `hybrid` 作为传统混合力位控制器，法向力误差较小，但缺少连续优先级调节，对位置误差和刚度切换的综合折中不足。",
        "3. `fixed_02/fixed_05/fixed_08` 形成固定 alpha 消融实验，能够展示 alpha_FP 取值对力安全和位置跟踪的权衡方向。",
        "4. `mpc_qp` 用作约束控制参考，体现预测/约束思想可以抑制部分风险，但在当前轻量实现中不具备本文动态 GT 的解释性 alpha 变化。",
        "5. `dynamic_gt` 是本文方法，应重点分析其 alpha_FP 随刚度和力安全裕度变化的趋势：进入硬区或力边界附近时降低位置优先级，平稳区域提高位置优先级，从而获得比单一固定策略更可解释的折中。",
        "",
        "建议在第三章实验小节中增加“多控制器结构对照实验”，并把本报告中的指标表和图作为论文图表来源。"
        "如果后续需要进一步增强优势显著性，应优先在 Gazebo/实物化仿真中增加刚度突变、测力噪声和安全边界收紧场景，而不是只在标称纯 Python 模型中调大差距。",
        "",
        "## 6. 输出文件",
        "",
        f"- 原始数据目录：`{Path(result_dir)}`",
        f"- no_rcm 指标表：`{analysis['summary']}`",
        f"- no_rcm 试次表：`{analysis['trial_metrics']}`",
        f"- no_rcm 自动报告：`{analysis['report']}`",
    ]
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def _parse_controllers(text):
    if not text:
        return DEFAULT_CONTROLLERS
    return tuple(item.strip() for item in text.split(",") if item.strip())


def main():
    parser = argparse.ArgumentParser(description="Run Chapter 3 controller baseline comparison.")
    parser.add_argument("--config", default=None, help="Optional ch3_experiments YAML config.")
    parser.add_argument(
        "--controllers",
        default=",".join(DEFAULT_CONTROLLERS),
        help="Comma separated controller names.",
    )
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--output-dir",
        default="/home/liu/franka_ros2_ws/results/ch3_controller_baseline_comparison",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg.controllers = _parse_controllers(args.controllers)
    cfg.repeats = int(args.repeats)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = Path(args.output_dir) / f"ch3_controller_baseline_{stamp}"
    result_dir.mkdir(parents=True, exist_ok=True)

    for controller in cfg.controllers:
        for trial in range(cfg.repeats):
            out = result_dir / f"{controller}_t{trial:02d}.npz"
            run_contact_simulation(cfg, controller, trial_index=trial, output_file=out)
            print(f"saved {out}")

    analysis = analyze_with_no_rcm(result_dir, result_dir / "ch3_analysis")
    summary_rows = _read_csv_rows(analysis["summary"])
    trial_rows = _read_csv_rows(analysis["trial_metrics"])
    figure_dir = result_dir / "figures"
    figure_dir.mkdir(exist_ok=True)
    figures = [
        _plot_metric_bars(summary_rows, figure_dir),
        _plot_zone_bars(trial_rows, figure_dir),
        _plot_representative_timeseries(result_dir, figure_dir),
    ]
    command = (
        "source /opt/ros/humble/setup.bash\n"
        "source /home/liu/franka_ros2_ws/install/setup.bash\n"
        "ros2 run ch3_experiments run_ch3_controller_baseline_comparison "
        f"--controllers {','.join(cfg.controllers)} --repeats {cfg.repeats} "
        f"--output-dir {Path(args.output_dir)}"
    )
    report = _write_report(result_dir, analysis, figures, cfg, command, summary_rows)
    print(f"report: {report}")
    print(f"result_dir: {result_dir}")


if __name__ == "__main__":
    main()
