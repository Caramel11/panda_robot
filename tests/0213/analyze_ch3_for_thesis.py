#!/usr/bin/env python3
"""Generate Chapter 3 thesis statistics and figures from log_0213.

The script reuses the metric definitions from the older data_analysis_*.py
scripts, but points them at the current raw-data folder:

    tests/0213/log_0213

Outputs are written directly to the USTC thesis figure folder so that
chapter3.tex can include them without copying files by hand.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
import pandas as pd
from scipy import stats

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import seaborn as sns


METHOD_DIRS = {
    "GT_KF": "0213_GT_KF",
    "GT_Sigmoid": "0213_GT_Sigmoid",
    "MPC_KF": "0213_MPC_KF",
    "MPC_Sigmoid": "0213_MPC_Sigmoid",
}

METHOD_LABELS = {
    "GT_KF": "GT + Fuzzy-AKF",
    "GT_Sigmoid": "GT + Sigmoid",
    "MPC_KF": "MPC + Fuzzy-AKF",
    "MPC_Sigmoid": "MPC + Sigmoid",
}

METHOD_ORDER = ["GT_KF", "GT_Sigmoid", "MPC_KF", "MPC_Sigmoid"]

METRICS = {
    "mean_error_rcm": ("Mean RCM error", "mm", 1000.0),
    "mean_error_ee": ("Mean EE error", "mm", 1000.0),
    "mean_force_norm": ("Interaction force", "N", 1.0),
    "Efs_force_smoothness": ("Force smoothness", r"N$^2$", 1.0),
    "traj_rms_jerk": ("Trajectory RMS jerk", "a.u.", 1.0),
    "mean_dist_to_object": ("Mean obstacle clearance", "mm", 1000.0),
}

CORE_OUTLIER_METRICS = [
    "mean_error_rcm",
    "mean_error_ee",
    "mean_force_norm",
    "Efs_force_smoothness",
    "traj_rms_jerk",
]

PAIRS = [
    ("GT_KF", "GT_Sigmoid"),
    ("GT_KF", "MPC_KF"),
    ("MPC_KF", "MPC_Sigmoid"),
    ("GT_Sigmoid", "MPC_Sigmoid"),
]

OBJECT_POSITION = np.array([0.3, -0.05, 0.05])
OBJECT_RADIUS = 0.03
DISTANCE_FILTER_M = 0.042


def read_numeric_xlsx(path: Path) -> pd.DataFrame:
    return pd.read_excel(path).select_dtypes(include=[np.number])


def safe_mean(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    return float(np.mean(values))


def collect_run(folder: Path, method: str) -> Dict[str, float | str]:
    row: Dict[str, float | str] = {
        "method": method,
        "method_label": METHOD_LABELS[method],
        "folder_name": folder.name,
        "folder": str(folder),
    }

    err_path = folder / "error.xlsx"
    if err_path.exists():
        err = pd.read_excel(err_path)
        row["mean_error_rcm"] = safe_mean(err.get("error_rcm", pd.Series(dtype=float)).dropna().to_numpy())
        row["max_error_rcm"] = float(err.get("error_rcm", pd.Series(dtype=float)).max())
        row["mean_error_ee"] = safe_mean(err.get("error_ee", pd.Series(dtype=float)).dropna().to_numpy())
        row["max_error_ee"] = float(err.get("error_ee", pd.Series(dtype=float)).max())

    force_path = folder / "force.xlsx"
    if force_path.exists():
        force = read_numeric_xlsx(force_path).to_numpy(dtype=float)
        if force.size > 0:
            f_norm = np.linalg.norm(force, axis=1)
            row["mean_force_norm"] = float(np.mean(f_norm))
            row["max_force_norm"] = float(np.max(f_norm))
            row["Efs_force_smoothness"] = float(np.mean(np.diff(f_norm) ** 2)) if len(f_norm) > 1 else float("nan")

    traj_path = folder / "file_traj_slave.xlsx"
    if traj_path.exists():
        traj = read_numeric_xlsx(traj_path).iloc[:, :3].to_numpy(dtype=float)
        if len(traj) >= 4:
            jerk = np.diff(traj, n=3, axis=0)
            row["traj_rms_jerk"] = float(np.sqrt(np.mean(np.sum(jerk * jerk, axis=1))))
        if len(traj) > 0:
            dist = np.linalg.norm(traj - OBJECT_POSITION, axis=1) - OBJECT_RADIUS
            row["mean_dist_to_object"] = float(np.mean(dist))
            row["min_dist_to_object"] = float(np.min(dist))

    return row


def collect_dataset(log_root: Path) -> pd.DataFrame:
    rows: List[Dict[str, float | str]] = []
    for method, dirname in METHOD_DIRS.items():
        method_root = log_root / dirname
        for folder in sorted(method_root.glob("*")):
            if folder.is_dir():
                rows.append(collect_run(folder, method))
    df = pd.DataFrame(rows)
    if "mean_dist_to_object" in df.columns:
        df.loc[df["mean_dist_to_object"] < DISTANCE_FILTER_M, "mean_dist_to_object"] = np.nan
    return df


def mark_iqr_outliers(df: pd.DataFrame, metrics: Iterable[str] = CORE_OUTLIER_METRICS, k: float = 1.5) -> pd.DataFrame:
    """Mark run-level outliers within each method using a uniform IQR rule."""
    marked = df.copy()
    marked["outlier_metric_count"] = 0
    marked["outlier_metrics"] = ""
    for method, sub in marked.groupby("method"):
        for metric in metrics:
            values = sub[metric].dropna()
            if len(values) < 4:
                continue
            q1 = values.quantile(0.25)
            q3 = values.quantile(0.75)
            iqr = q3 - q1
            if iqr <= 0 or not np.isfinite(iqr):
                continue
            lower = q1 - k * iqr
            upper = q3 + k * iqr
            idx = sub[(sub[metric] < lower) | (sub[metric] > upper)].index
            marked.loc[idx, "outlier_metric_count"] += 1
            for i in idx:
                old = marked.at[i, "outlier_metrics"]
                marked.at[i, "outlier_metrics"] = metric if not old else f"{old};{metric}"
    marked["excluded_by_iqr"] = marked["outlier_metric_count"] > 0
    return marked


def sig_label(p_value: float) -> str:
    if p_value < 0.001:
        return "***"
    if p_value < 0.01:
        return "**"
    if p_value < 0.05:
        return "*"
    return "n.s."


def pairwise_tests(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for a, b in PAIRS:
        for metric in METRICS:
            x = df[df["method"] == a][metric].dropna()
            y = df[df["method"] == b][metric].dropna()
            if len(x) < 2 or len(y) < 2:
                continue
            try:
                normal = stats.shapiro(x)[1] > 0.05 and stats.shapiro(y)[1] > 0.05
                if normal:
                    equal_var = stats.levene(x, y)[1] > 0.05
                    _, p_value = stats.ttest_ind(x, y, equal_var=equal_var)
                    test_name = "t-test"
                else:
                    _, p_value = stats.mannwhitneyu(x, y, alternative="two-sided")
                    test_name = "Mann-Whitney"
            except Exception:
                p_value = 1.0
                test_name = "error"
            rows.append(
                {
                    "comparison": f"{a} vs {b}",
                    "metric": metric,
                    "metric_label": METRICS[metric][0],
                    "test": test_name,
                    "p_value": float(p_value),
                    "sig": sig_label(float(p_value)),
                    f"mean_{a}": float(x.mean()),
                    f"mean_{b}": float(y.mean()),
                }
            )
    return pd.DataFrame(rows)


def summary_stats(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for method in METHOD_ORDER:
        sub = df[df["method"] == method]
        row = {"method": method, "method_label": METHOD_LABELS[method], "n": int(len(sub))}
        for metric in METRICS:
            values = sub[metric].dropna()
            row[f"{metric}_n"] = int(len(values))
            row[f"{metric}_mean"] = float(values.mean()) if len(values) else float("nan")
            row[f"{metric}_std"] = float(values.std(ddof=1)) if len(values) > 1 else float("nan")
        rows.append(row)
    return pd.DataFrame(rows)


def format_value(mean: float, std: float, scale: float, precision: int = 2) -> str:
    if math.isnan(mean):
        return "--"
    return f"{mean * scale:.{precision}f} $\\pm$ {std * scale:.{precision}f}"


def write_latex_table(summary: pd.DataFrame, path: Path) -> None:
    metric_rows = [
        ("平均几何约束误差/mm", "mean_error_rcm", 1000.0, 2),
        ("平均末端误差/mm", "mean_error_ee", 1000.0, 2),
        ("平均交互力范数/N", "mean_force_norm", 1.0, 3),
        ("力平滑性指标", "Efs_force_smoothness", 1.0, 4),
        ("轨迹加加速度均方根", "traj_rms_jerk", 1.0, 5),
        ("平均障碍距离/mm", "mean_dist_to_object", 1000.0, 2),
    ]
    lines = [
        "% Auto-generated by tests/0213/analyze_ch3_for_thesis.py",
        "\\begin{tabular}{lcccc}",
        "\\toprule",
        "指标 & 博弈+模糊滤波 & 博弈+S型映射 & 预测+模糊滤波 & 预测+S型映射 \\\\",
        "\\midrule",
    ]
    row_map = {row["method"]: row for _, row in summary.iterrows()}
    for label, metric, scale, precision in metric_rows:
        cells = []
        for method in METHOD_ORDER:
            row = row_map[method]
            cells.append(format_value(row[f"{metric}_mean"], row[f"{metric}_std"], scale, precision))
        lines.append(f"{label} & " + " & ".join(cells) + r" \\")
    lines.extend(["\\bottomrule", "\\end{tabular}", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def configure_style() -> None:
    sns.set_theme(style="whitegrid", context="paper")
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.unicode_minus": False,
            "axes.linewidth": 0.8,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "legend.frameon": False,
            "savefig.bbox": "tight",
        }
    )


def plot_metric_boxes(df: pd.DataFrame, tests: pd.DataFrame, out_path: Path) -> None:
    configure_style()
    fig, axes = plt.subplots(2, 3, figsize=(13.2, 7.5), constrained_layout=True)
    axes = axes.ravel()
    palette = {
        "GT_KF": "#4C78A8",
        "GT_Sigmoid": "#72B7B2",
        "MPC_KF": "#F58518",
        "MPC_Sigmoid": "#E45756",
    }

    for ax, (metric, (title, unit, scale)) in zip(axes, METRICS.items()):
        plot_df = df[["method", metric]].dropna().copy()
        plot_df["value"] = plot_df[metric] * scale
        sns.boxplot(
            data=plot_df,
            x="method",
            y="value",
            order=METHOD_ORDER,
            palette=palette,
            showfliers=False,
            linewidth=0.9,
            ax=ax,
        )
        sns.stripplot(
            data=plot_df,
            x="method",
            y="value",
            order=METHOD_ORDER,
            color="0.2",
            size=2.4,
            alpha=0.42,
            jitter=0.18,
            ax=ax,
        )
        ax.set_title(title)
        ax.set_xlabel("")
        ax.set_ylabel(unit)
        ax.set_xticklabels([METHOD_LABELS[m].replace(" + ", "\n+") for m in METHOD_ORDER], fontsize=7.5)
        ax.grid(True, axis="y", alpha=0.28)

        ymin, ymax = ax.get_ylim()
        yr = ymax - ymin if ymax > ymin else 1.0
        y = ymax + 0.04 * yr
        for a, b in [("GT_KF", "GT_Sigmoid"), ("GT_KF", "MPC_KF")]:
            match = tests[(tests["comparison"] == f"{a} vs {b}") & (tests["metric"] == metric)]
            if match.empty:
                continue
            label = match.iloc[0]["sig"]
            if label == "n.s.":
                continue
            i, j = METHOD_ORDER.index(a), METHOD_ORDER.index(b)
            h = 0.025 * yr
            ax.plot([i, i, j, j], [y, y + h, y + h, y], color="black", lw=0.75)
            ax.text((i + j) / 2, y + h, label, ha="center", va="bottom", fontsize=8)
            y += 0.12 * yr
        ax.set_ylim(ymin, y + 0.02 * yr)

    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def choose_representative_run(df: pd.DataFrame) -> Path:
    sub = df[df["method"] == "GT_KF"].copy()
    sub = sub.dropna(subset=["mean_error_rcm", "mean_error_ee", "traj_rms_jerk"])
    med = sub[["mean_error_rcm", "mean_error_ee", "traj_rms_jerk"]].median()
    z = (sub[["mean_error_rcm", "mean_error_ee", "traj_rms_jerk"]] - med).abs()
    sub["median_distance"] = z.sum(axis=1)
    folder = sub.sort_values("median_distance").iloc[0]["folder"]
    return Path(str(folder))


def read_xyz(folder: Path, filename: str) -> pd.DataFrame:
    data = read_numeric_xlsx(folder / filename)
    cols = list(data.columns[:3])
    return data[cols].rename(columns={cols[0]: "x", cols[1]: "y", cols[2]: "z"})


def force_norm(folder: Path) -> np.ndarray:
    force = read_numeric_xlsx(folder / "force.xlsx").to_numpy(dtype=float)
    return np.linalg.norm(force, axis=1)


def plot_representative_trajectory(folder: Path, out_path: Path) -> None:
    configure_style()
    slave = read_xyz(folder, "file_traj_slave.xlsx")
    master = read_xyz(folder, "file_traj_master.xlsx")
    robot = read_xyz(folder, "file_traj_robot.xlsx")
    human = read_xyz(folder, "file_traj_human.xlsx")
    reshape = read_xyz(folder, "file_traj_reshape.xlsx")
    f_norm = force_norm(folder)
    active = f_norm > 0.05
    idx = np.where(active)[0]

    fig = plt.figure(figsize=(12.5, 5.2), constrained_layout=True)
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2)

    ax3d.plot(master["x"], master["y"], master["z"], color="#4C78A8", lw=1.8, label="master reference")
    ax3d.plot(slave["x"], slave["y"], slave["z"], color="#E45756", lw=2.1, label="slave actual")
    ax3d.plot(reshape["x"], reshape["y"], reshape["z"], color="#54A24B", lw=1.2, alpha=0.8, label="deformed ref.")
    if len(idx):
        ax3d.scatter(slave["x"].iloc[idx], slave["y"].iloc[idx], slave["z"].iloc[idx], s=4, c=f_norm[idx], cmap="viridis", alpha=0.75)
    targets = np.array([[0.25, 0.06, 0.03], [0.30, 0.06, 0.03], [0.35, 0.06, 0.03]])
    ax3d.scatter(targets[:, 0], targets[:, 1], targets[:, 2], marker="*", s=90, c="black", label="candidate targets")
    ax3d.set_xlabel("x / m")
    ax3d.set_ylabel("y / m")
    ax3d.set_zlabel("z / m")
    ax3d.view_init(elev=23, azim=-65)
    ax3d.legend(fontsize=7, loc="upper left")
    ax3d.set_title("Representative GT + Fuzzy-AKF trajectory")

    l = min(len(slave), len(master), len(robot), len(human))
    time = np.arange(l) * 0.01
    ee_master = np.linalg.norm(slave.iloc[:l].to_numpy() - master.iloc[:l].to_numpy(), axis=1) * 1000.0
    ee_robot = np.linalg.norm(slave.iloc[:l].to_numpy() - robot.iloc[:l].to_numpy(), axis=1) * 1000.0
    ee_human = np.linalg.norm(slave.iloc[:l].to_numpy() - human.iloc[:l].to_numpy(), axis=1) * 1000.0
    ax2.plot(time, ee_master, color="#4C78A8", lw=1.0, label="slave-master")
    ax2.plot(time, ee_robot, color="#F58518", lw=1.0, label="slave-robot")
    ax2.plot(time, ee_human, color="#54A24B", lw=1.0, label="slave-human")
    ax2.set_xlabel("time / s")
    ax2.set_ylabel("tracking distance / mm")
    ax2.set_title("Reference tracking under human intervention")
    ax2.grid(True, alpha=0.25)
    ax2.legend(fontsize=7)

    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def plot_representative_arbitration(folder: Path, out_path: Path) -> None:
    configure_style()
    error = pd.read_excel(folder / "error.xlsx")
    fuzzy = pd.read_excel(folder / "fuzzy.xlsx")
    fvd = pd.read_excel(folder / "FVD.xlsx")
    d_fvd = pd.read_excel(folder / "dFVD.xlsx")
    f_norm = force_norm(folder)
    n = min(len(error), len(fuzzy), len(fvd), len(d_fvd), len(f_norm))
    t = np.arange(n) * 0.01

    fig, axes = plt.subplots(4, 1, figsize=(12.5, 8.2), sharex=True, constrained_layout=True)
    axes[0].plot(t, f_norm[:n], color="#4C78A8", lw=1.0, label=r"$\|F_h\|$")
    axes[0].set_ylabel("force / N")
    axes[0].legend(loc="upper right", fontsize=8)

    axes[1].plot(t, fuzzy["lambda"].iloc[:n], color="#E45756", lw=1.0, label=r"$\lambda$")
    axes[1].plot(t, fuzzy["delta_lambda"].iloc[:n], color="#72B7B2", lw=0.9, label=r"$\lambda_{\Delta}$")
    axes[1].set_ylabel("arbitration")
    axes[1].legend(loc="upper right", fontsize=8)

    axes[2].plot(t, fvd["F_h"].iloc[:n], color="#4C78A8", lw=0.9, label=r"$F_h$")
    axes[2].plot(t, fvd["T_h"].iloc[:n], color="#F58518", lw=0.9, label=r"$T_h$")
    axes[2].plot(t, fvd["D_r"].iloc[:n], color="#54A24B", lw=0.9, label=r"$D_r$")
    axes[2].set_ylabel("fuzzy inputs")
    axes[2].legend(loc="upper right", ncol=3, fontsize=8)

    axes[3].plot(t, error["error_rcm"].iloc[:n] * 1000.0, color="#B279A2", lw=0.9, label="RCM error")
    axes[3].plot(t, error["error_ee"].iloc[:n] * 1000.0, color="#E45756", lw=0.9, label="EE error")
    axes[3].set_ylabel("error / mm")
    axes[3].set_xlabel("time / s")
    axes[3].legend(loc="upper right", fontsize=8)

    active = f_norm[:n] > 0.05
    edges = np.diff(active.astype(int))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0]
    for ax in axes:
        for s in starts:
            ax.axvline(t[s], color="#4C78A8", ls="--", lw=0.6, alpha=0.25)
        for e in ends:
            ax.axvline(t[e], color="#F58518", ls="--", lw=0.6, alpha=0.25)
        ax.grid(True, alpha=0.25)

    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-root", type=Path, default=Path("src/panda_robot/tests/0213/log_0213"))
    parser.add_argument("--figure-dir", type=Path, default=Path("src/panda_robot/tests/0608thesis/figures/ch3_ch4"))
    parser.add_argument("--output-dir", type=Path, default=Path("src/panda_robot/tests/0213/ch3_thesis_outputs"))
    parser.add_argument("--no-filter-outliers", action="store_true", help="Keep all valid raw runs instead of applying the IQR outlier rule.")
    args = parser.parse_args()

    args.figure_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    raw_df = collect_dataset(args.log_root)
    marked_df = mark_iqr_outliers(raw_df)
    df = marked_df[~marked_df["excluded_by_iqr"]].copy() if not args.no_filter_outliers else marked_df.copy()
    tests = pairwise_tests(df)
    summary = summary_stats(df)

    raw_df.to_csv(args.output_dir / "ch3_run_metrics_raw.csv", index=False)
    marked_df.to_csv(args.output_dir / "ch3_run_metrics_with_outlier_flags.csv", index=False)
    df.to_csv(args.output_dir / "ch3_run_metrics.csv", index=False)
    marked_df[marked_df["excluded_by_iqr"]].to_csv(args.output_dir / "ch3_excluded_iqr_outliers.csv", index=False)
    tests.to_csv(args.output_dir / "ch3_pairwise_tests.csv", index=False)
    summary.to_csv(args.output_dir / "ch3_summary_stats.csv", index=False)
    write_latex_table(summary, args.output_dir / "ch3_summary_table.tex")

    plot_metric_boxes(df, tests, args.figure_dir / "ch3_metrics_4method_box.png")
    rep = choose_representative_run(df)
    plot_representative_trajectory(rep, args.figure_dir / "ch3_representative_trajectory.png")
    plot_representative_arbitration(rep, args.figure_dir / "ch3_representative_arbitration.png")

    print(f"Loaded {len(raw_df)} raw runs from {args.log_root}")
    if args.no_filter_outliers:
        print("Outlier filtering disabled.")
    else:
        print(f"Excluded {int(marked_df['excluded_by_iqr'].sum())} IQR outlier runs.")
        print("Kept counts:", df.groupby("method").size().to_dict())
    print(f"Representative GT_KF run: {rep}")
    print(summary.to_string(index=False))
    print(tests[tests["comparison"].isin(["GT_KF vs GT_Sigmoid", "GT_KF vs MPC_KF"])].to_string(index=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
