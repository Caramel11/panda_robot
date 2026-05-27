#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
0526 no-RCM 控制器实验数据分析与绘图脚本
========================================

默认行为:
  1. 在 /home/liu/franka_ws_1101/results/no_rcm_*/*.npz 中自动寻找最近一次实验结果；
  2. 读取 DataLogger 保存的时序数据；
  3. 计算力控制、位置跟踪、alpha 仲裁、增益、阶段等统计指标；
  4. 保存多张 PNG/PDF 图；
  5. 将原始时序数据和统计结果导出为 Excel；
  6. 默认尝试展示图像窗口，服务器/无 DISPLAY 环境下自动跳过展示。

典型用法:
  cd /home/liu/franka_ws_1101/src/panda_robot/tests/0526_controller
  python3 plot_experiment_analysis.py

只保存不弹窗:
  python3 plot_experiment_analysis.py --no-show

指定某个结果:
  python3 plot_experiment_analysis.py \
    --input /home/liu/franka_ws_1101/results/no_rcm_20260526_140949/continuous_force_margin_alpha_t00.npz

指定输出目录:
  python3 plot_experiment_analysis.py \
    --output-dir /home/liu/franka_ws_1101/results/no_rcm_20260526_140949/analysis_custom

输出内容:
  - 5 组 PNG/PDF 图像；
  - 1 个 Excel 文件，包含 summary_metrics、phase_statistics、
    force_band_statistics、time_series 四个 sheet；
  - 终端打印关键指标，便于快速判断实验质量。
"""
import argparse
import os
from pathlib import Path
import textwrap

# matplotlib 第一次运行会写缓存。若用户环境中的 ~/.config/matplotlib 不可写，
# 指定到 /tmp 可避免反复出现 cache warning。
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-0526-controller")

import numpy as np
import pandas as pd
import matplotlib

# 没有图形桌面时使用 Agg 后端，仍可保存图片，但不弹窗。
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib import font_manager


BASE_DIR = Path(__file__).resolve().parent
# 默认结果目录改为工作区根目录下的 results。
# run_no_rcm.py 可以通过 --output-dir 指向这个目录；本脚本会自动在其中寻找
# 最近修改的 no_rcm_*/*.npz 文件。这样即使控制器脚本和结果目录分离，也能
# 直接运行 `python3 plot_experiment_analysis.py` 完成最近一次实验分析。
DEFAULT_RESULTS_DIR = Path("/home/liu/franka_ws_1101/results")


def latest_npz(results_dir):
    """寻找最近修改的实验 npz 文件。

    搜索模式固定为:
      <results_dir>/no_rcm_*/*.npz

    这样可以兼容不同策略名的输出文件，例如:
      online_priority_alpha_t00.npz
      continuous_force_margin_alpha_t00.npz
      fixed_0.5_t00.npz
    """
    candidates = sorted(
        Path(results_dir).glob("no_rcm_*/*.npz"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(
            f"未找到实验结果: {results_dir}/no_rcm_*/*.npz\n"
            "请先运行 run_no_rcm.py 生成数据，或使用 --input 指定 npz 文件。"
        )
    return candidates[0]


def load_npz(path):
    """读取 npz 并转换为普通 dict，便于后续统一处理。"""
    data = np.load(path, allow_pickle=True)
    return {k: data[k] for k in data.files}


def arr(data, key, default=None, dtype=float):
    """从结果字典中取数组；缺失字段时返回默认值数组。"""
    if key in data:
        return np.asarray(data[key], dtype=dtype)
    if default is None:
        raise KeyError(f"结果文件缺少字段: {key}")
    return np.asarray(default, dtype=dtype)


def safe_stat(x, fn, default=np.nan):
    """对可能为空的数组做统计，避免空数据报错。"""
    x = np.asarray(x)
    if x.size == 0:
        return default
    return float(fn(x))


def build_dataframe(data):
    """构造时序 DataFrame。

    DataLogger 保存时已经把向量拆为标量字段；这里按常用分析字段重组。
    若某些兼容字段不存在，则用 NaN 填充，保证 Excel 和绘图列稳定。
    """
    t = arr(data, "t")
    n = len(t)

    def get(name, fill=np.nan):
        return arr(data, name, np.full(n, fill))

    df = pd.DataFrame({
        "t_s": t,
        "pos_x_m": get("pos_x"),
        "pos_y_m": get("pos_y"),
        "pos_z_m": get("pos_z"),
        "pos_des_x_m": get("pos_des_x"),
        "pos_des_y_m": get("pos_des_y"),
        "pos_des_z_m": get("pos_des_z"),
        "pos_err_x_m": get("pos_err_x"),
        "pos_err_y_m": get("pos_err_y"),
        "pos_err_z_m": get("pos_err_z"),
        "pos_err_norm_m": get("pos_err_norm"),
        "F_measured_N": get("F_measured"),
        "F_desired_N": get("F_desired"),
        "F_err_N": get("F_err"),
        "alpha": get("alpha"),
        "K_hat_Npm": get("K_hat"),
        "K_total": get("K_total"),
        "K_r2": get("K_r2"),
        "K_ef": get("K_ef"),
        "K_sf": get("K_sf"),
        "e_f_N": get("e_f"),
        "e_r_m": get("e_r"),
        "sigma_f_norm": get("sigma_f_norm"),
        "e_r1_norm": get("e_r1_norm"),
        "u_norm": get("u_norm"),
        "phase": get("phase"),
    })

    # 衍生列统一使用工程单位，绘图更直观。
    df["pos_err_norm_mm"] = 1000.0 * df["pos_err_norm_m"]
    df["pos_z_mm"] = 1000.0 * df["pos_z_m"]
    df["pos_des_z_mm"] = 1000.0 * df["pos_des_z_m"]
    df["pos_x_mm"] = 1000.0 * df["pos_x_m"]
    df["pos_des_x_mm"] = 1000.0 * df["pos_des_x_m"]
    df["force_abs_err_N"] = np.abs(df["F_err_N"])
    return df


def compute_metrics(df, source_path):
    """计算实验总体统计指标。"""
    t = df["t_s"].to_numpy()
    dt = np.diff(t) if len(t) > 1 else np.array([])
    force_err = df["F_err_N"].to_numpy()
    force = df["F_measured_N"].to_numpy()
    force_des = df["F_desired_N"].to_numpy()
    pos_err_mm = df["pos_err_norm_mm"].to_numpy()
    alpha = df["alpha"].to_numpy()
    k_hat = df["K_hat_Npm"].to_numpy()

    metrics = {
        "source_file": str(source_path),
        "samples": int(len(df)),
        "duration_s": safe_stat(t, np.max) - safe_stat(t, np.min) if len(t) else np.nan,
        "dt_mean_s": safe_stat(dt, np.mean),
        "dt_std_s": safe_stat(dt, np.std),
        "force_mean_N": safe_stat(force, np.mean),
        "force_desired_mean_N": safe_stat(force_des, np.mean),
        "force_rmse_N": float(np.sqrt(np.nanmean(force_err ** 2))),
        "force_mae_N": safe_stat(np.abs(force_err), np.mean),
        "force_std_N": safe_stat(force, np.std),
        "force_min_N": safe_stat(force, np.min),
        "force_max_N": safe_stat(force, np.max),
        "force_final_N": float(force[-1]) if len(force) else np.nan,
        "force_err_final_N": float(force_err[-1]) if len(force_err) else np.nan,
        "force_within_0p05_ratio": float(np.nanmean(np.abs(force_err) <= 0.05)),
        "force_within_0p10_ratio": float(np.nanmean(np.abs(force_err) <= 0.10)),
        "pos_err_mean_mm": safe_stat(pos_err_mm, np.mean),
        "pos_err_rmse_mm": float(np.sqrt(np.nanmean(pos_err_mm ** 2))),
        "pos_err_max_mm": safe_stat(pos_err_mm, np.max),
        "pos_err_final_mm": float(pos_err_mm[-1]) if len(pos_err_mm) else np.nan,
        "alpha_mean": safe_stat(alpha, np.mean),
        "alpha_min": safe_stat(alpha, np.min),
        "alpha_max": safe_stat(alpha, np.max),
        "alpha_final": float(alpha[-1]) if len(alpha) else np.nan,
        "K_hat_mean": safe_stat(k_hat, np.mean),
        "K_hat_min": safe_stat(k_hat, np.min),
        "K_hat_max": safe_stat(k_hat, np.max),
    }

    # 力误差进入 ±0.1N 并连续保持 1 秒的时间，用于评估扫描早期收敛速度。
    metrics["force_settle_time_0p10_s"] = force_settle_time(
        t, force_err, band=0.10, hold_s=1.0
    )
    return pd.DataFrame([metrics])


def force_settle_time(t, force_err, band=0.10, hold_s=1.0):
    """计算力误差首次连续 hold_s 秒保持在 band 内的时间。"""
    if len(t) < 2:
        return np.nan
    dt = float(np.nanmedian(np.diff(t)))
    window = max(1, int(round(hold_s / max(dt, 1e-9))))
    ok = np.abs(force_err) <= band
    for i in range(0, len(ok) - window + 1):
        if np.all(ok[i:i + window]):
            return float(t[i])
    return np.nan


def phase_statistics(df):
    """按阶段统计样本数和持续时间。"""
    if "phase" not in df:
        return pd.DataFrame()
    rows = []
    t = df["t_s"].to_numpy()
    dt = float(np.nanmedian(np.diff(t))) if len(t) > 1 else 0.0
    for phase, group in df.groupby("phase", dropna=False):
        rows.append({
            "phase": phase,
            "samples": int(len(group)),
            "duration_s_est": float(len(group) * dt),
            "force_rmse_N": float(np.sqrt(np.nanmean(group["F_err_N"].to_numpy() ** 2))),
            "pos_err_mean_mm": float(np.nanmean(group["pos_err_norm_mm"])),
            "alpha_mean": float(np.nanmean(group["alpha"])),
        })
    return pd.DataFrame(rows)


def band_statistics(df):
    """统计不同力误差带内的占比。"""
    rows = []
    err = df["F_err_N"].to_numpy()
    for band in [0.02, 0.05, 0.10, 0.20, 0.50]:
        rows.append({
            "force_error_band_N": band,
            "inside_ratio": float(np.nanmean(np.abs(err) <= band)),
            "inside_percent": 100.0 * float(np.nanmean(np.abs(err) <= band)),
        })
    return pd.DataFrame(rows)


def save_excel(output_xlsx, df, metrics_df, phase_df, band_df):
    """导出 Excel，多 sheet 保存原始数据和统计结果。"""
    output_xlsx.parent.mkdir(parents=True, exist_ok=True)
    with pd.ExcelWriter(output_xlsx, engine="openpyxl") as writer:
        metrics_df.to_excel(writer, sheet_name="summary_metrics", index=False)
        phase_df.to_excel(writer, sheet_name="phase_statistics", index=False)
        band_df.to_excel(writer, sheet_name="force_band_statistics", index=False)
        df.to_excel(writer, sheet_name="time_series", index=False)


def choose_chinese_font():
    """自动选择当前系统可用的中文字体。

    图表标题、坐标轴和图例均使用中文。matplotlib 默认字体通常缺少中文字形，
    会导致保存图片时出现大量 Glyph missing 警告。这里优先选择仿真电脑上
    常见的 Noto/文泉驿/思源字体；若都不存在，则回退到 DejaVu Sans，
    此时图片仍能生成，只是中文字体可能由系统继续做替换。
    """
    preferred_fonts = [
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
        "Source Han Sans SC",
        "SimHei",
        "Microsoft YaHei",
        "Droid Sans Fallback",
        "DejaVu Sans",
    ]
    available_fonts = {font.name for font in font_manager.fontManager.ttflist}
    for name in preferred_fonts:
        if name in available_fonts:
            return name
    return "DejaVu Sans"


def apply_plot_style():
    """设置统一绘图风格和中文字体。"""
    chinese_font = choose_chinese_font()
    plt.rcParams.update({
        "figure.dpi": 120,
        "savefig.dpi": 200,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.unicode_minus": False,
        "font.family": "sans-serif",
        "font.sans-serif": [chinese_font, "DejaVu Sans"],
        "font.size": 10,
    })


def save_figure(fig, out_dir, stem):
    """同时保存 PNG 和 PDF。"""
    png = out_dir / f"{stem}.png"
    pdf = out_dir / f"{stem}.pdf"
    fig.tight_layout()
    fig.savefig(png, bbox_inches="tight")
    fig.savefig(pdf, bbox_inches="tight")
    return png, pdf


def make_plots(df, out_dir, title_prefix):
    """生成充分分析用的多组图。"""
    apply_plot_style()
    out_dir.mkdir(parents=True, exist_ok=True)
    saved = []
    t = df["t_s"]

    # 图 1: 力跟踪、力误差、alpha。
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, df["F_measured_N"], label="实际接触力 F")
    axes[0].plot(t, df["F_desired_N"], "--", label="目标力 Fd")
    axes[0].set_ylabel("力 / N")
    axes[0].legend(loc="best")
    axes[0].set_title(f"{title_prefix} - 力跟踪与 alpha")

    axes[1].plot(t, df["F_err_N"], color="tab:red", label="力误差 F-Fd")
    axes[1].axhline(0.10, color="gray", ls="--", lw=1)
    axes[1].axhline(-0.10, color="gray", ls="--", lw=1)
    axes[1].set_ylabel("力误差 / N")
    axes[1].legend(loc="best")

    axes[2].plot(t, df["alpha"], color="tab:green", label="alpha")
    axes[2].set_ylabel("alpha")
    axes[2].set_xlabel("时间 / s")
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].legend(loc="best")
    saved.extend(save_figure(fig, out_dir, "01_force_alpha"))

    # 图 2: 位置跟踪和位置误差。
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, df["pos_x_mm"], label="x 实际")
    axes[0].plot(t, df["pos_des_x_mm"], "--", label="x 期望")
    axes[0].set_ylabel("x / mm")
    axes[0].legend(loc="best")
    axes[0].set_title(f"{title_prefix} - 位置跟踪")

    axes[1].plot(t, 1000.0 * df["pos_y_m"], label="y 实际")
    axes[1].plot(t, 1000.0 * df["pos_des_y_m"], "--", label="y 期望")
    axes[1].set_ylabel("y / mm")
    axes[1].legend(loc="best")

    axes[2].plot(t, df["pos_z_mm"], label="z 实际")
    axes[2].plot(t, df["pos_des_z_mm"], "--", label="z 期望")
    axes[2].set_ylabel("z / mm")
    axes[2].legend(loc="best")

    axes[3].plot(t, df["pos_err_norm_mm"], color="tab:purple", label="位置误差范数")
    axes[3].set_ylabel("误差 / mm")
    axes[3].set_xlabel("时间 / s")
    axes[3].legend(loc="best")
    saved.extend(save_figure(fig, out_dir, "02_position_tracking"))

    # 图 3: 环境估计与控制增益。
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, df["K_hat_Npm"], label="RLS 估计刚度 K_hat")
    axes[0].set_ylabel("K_hat / N/m")
    axes[0].legend(loc="best")
    axes[0].set_title(f"{title_prefix} - 环境估计与增益")

    axes[1].plot(t, df["K_total"], label="K_total")
    axes[1].plot(t, df["K_r2"], label="K_r2")
    axes[1].set_ylabel("位置/速度增益")
    axes[1].legend(loc="best")

    axes[2].plot(t, df["K_ef"], label="K_ef")
    axes[2].plot(t, df["K_sf"], label="K_sf")
    axes[2].set_ylabel("力相关增益")
    axes[2].set_xlabel("时间 / s")
    axes[2].legend(loc="best")
    saved.extend(save_figure(fig, out_dir, "03_gain_estimation"))

    # 图 4: 控制强度、误差状态与阶段。
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, df["u_norm"], label="关节力矩范数")
    axes[0].set_ylabel("u_norm")
    axes[0].legend(loc="best")
    axes[0].set_title(f"{title_prefix} - 控制量与状态")

    axes[1].plot(t, df["e_f_N"], label="e_f")
    axes[1].plot(t, df["sigma_f_norm"], label="||sigma_f||")
    axes[1].set_ylabel("力状态")
    axes[1].legend(loc="best")

    axes[2].plot(t, 1000.0 * df["e_r_m"], label="e_r")
    axes[2].plot(t, df["pos_err_norm_mm"], label="位置误差范数")
    axes[2].set_ylabel("mm")
    axes[2].legend(loc="best")

    axes[3].step(t, df["phase"], where="post", label="phase")
    axes[3].set_ylabel("阶段")
    axes[3].set_xlabel("时间 / s")
    axes[3].legend(loc="best")
    saved.extend(save_figure(fig, out_dir, "04_control_state_phase"))

    # 图 5: 分布和相关性，用于快速发现偏置/饱和/异常聚类。
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes[0, 0].hist(df["F_err_N"].dropna(), bins=40, color="tab:red", alpha=0.75)
    axes[0, 0].set_xlabel("力误差 / N")
    axes[0, 0].set_ylabel("样本数")
    axes[0, 0].set_title("力误差分布")

    axes[0, 1].hist(df["pos_err_norm_mm"].dropna(), bins=40, color="tab:purple", alpha=0.75)
    axes[0, 1].set_xlabel("位置误差 / mm")
    axes[0, 1].set_ylabel("样本数")
    axes[0, 1].set_title("位置误差分布")

    sc = axes[1, 0].scatter(df["alpha"], df["F_err_N"], c=t, s=10, cmap="viridis")
    axes[1, 0].set_xlabel("alpha")
    axes[1, 0].set_ylabel("力误差 / N")
    axes[1, 0].set_title("alpha-力误差关系")
    fig.colorbar(sc, ax=axes[1, 0], label="时间 / s")

    axes[1, 1].scatter(df["K_hat_Npm"], df["F_measured_N"], s=10, alpha=0.6)
    axes[1, 1].set_xlabel("K_hat / N/m")
    axes[1, 1].set_ylabel("实际力 / N")
    axes[1, 1].set_title("环境估计-接触力关系")
    saved.extend(save_figure(fig, out_dir, "05_distribution_scatter"))

    return saved


def parse_args():
    usage_text = """
示例:
  # 1. 默认分析最近一次实验
  python3 plot_experiment_analysis.py

  # 2. 无桌面环境，只保存图像和 Excel
  python3 plot_experiment_analysis.py --no-show

  # 3. 指定用户给出的实验目录中的 npz
  python3 plot_experiment_analysis.py \\
    --input /home/liu/franka_ws_1101/results/no_rcm_20260526_140949/continuous_force_margin_alpha_t00.npz

  # 4. 指定默认搜索根目录
  python3 plot_experiment_analysis.py --results-dir /home/liu/franka_ws_1101/results

  # 5. 指定输出目录
  python3 plot_experiment_analysis.py --output-dir /tmp/no_rcm_analysis

输出:
  analysis/01_force_alpha.png/.pdf
  analysis/02_position_tracking.png/.pdf
  analysis/03_gain_estimation.png/.pdf
  analysis/04_control_state_phase.png/.pdf
  analysis/05_distribution_scatter.png/.pdf
  analysis/<输入文件名>_analysis.xlsx
"""
    ap = argparse.ArgumentParser(
        description="分析 0526 no-RCM 控制器实验数据，自动保存并展示绘图结果。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(usage_text),
    )
    ap.add_argument(
        "--input",
        default=None,
        help=(
            "指定单个 npz 文件；不指定时自动寻找 "
            f"{DEFAULT_RESULTS_DIR}/no_rcm_*/*.npz 中最近修改的文件"
        ),
    )
    ap.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR),
                    help="默认搜索根目录，脚本会匹配 <results-dir>/no_rcm_*/*.npz")
    ap.add_argument("--output-dir", default=None,
                    help="图表和 Excel 输出目录；默认在 npz 同级目录下创建 analysis")
    ap.add_argument("--no-show", action="store_true", help="只保存图，不弹窗展示")
    return ap.parse_args()


def main():
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve() if args.input else latest_npz(args.results_dir)
    data = load_npz(input_path)
    df = build_dataframe(data)

    if len(df) == 0:
        raise RuntimeError(f"结果文件为空: {input_path}")

    out_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else input_path.parent / "analysis"
    title_prefix = input_path.parent.name + "/" + input_path.stem

    metrics_df = compute_metrics(df, input_path)
    phase_df = phase_statistics(df)
    band_df = band_statistics(df)

    saved_figs = make_plots(df, out_dir, title_prefix)
    xlsx_path = out_dir / f"{input_path.stem}_analysis.xlsx"
    save_excel(xlsx_path, df, metrics_df, phase_df, band_df)

    print("分析完成")
    print(f"输入文件: {input_path}")
    print(f"输出目录: {out_dir}")
    print(f"Excel: {xlsx_path}")
    print("关键指标:")
    for key in [
        "samples", "duration_s", "force_rmse_N", "force_mae_N",
        "force_within_0p10_ratio", "pos_err_mean_mm", "pos_err_max_mm",
        "alpha_min", "alpha_max", "alpha_final",
    ]:
        print(f"  {key}: {metrics_df.iloc[0][key]}")
    print("保存图片:")
    for p in saved_figs:
        print(f"  {p}")

    # 有 DISPLAY 且未指定 --no-show 时展示图像。Agg 后端下 show 不会弹窗。
    if not args.no_show and os.environ.get("DISPLAY"):
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
