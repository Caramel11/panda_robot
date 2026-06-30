"""
data_analysis_GTvsMPC.py  (IEEE-style refactor for Figure 6)
=============================================================
Generates `significance_annotated_GTvsMPC.png` for the simulation comparison
between the cooperative game-theoretic (GT) backbone and the model-predictive-
control (MPC) baseline.

Layout: 2x3 grid, sized for `\\begin{figure*}` (double-column).  See
ieee_style.py for shared rcParams.
"""

import os
import glob
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from scipy import stats

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ieee_style import apply_ieee_style, IEEE_COLORS, save_fig, DBL_W


# ==========================================
# 1. Configuration
# ==========================================
BASE_LOGS_DIR = "logs/20260127/0127_*"
INPUT_PATH = "logs/20260127/summary_batch_results.xlsx"
OUTPUT_DIR = "logs/20260127"
OUTPUT_EXCEL = os.path.join(OUTPUT_DIR, "detailed_significance_analysis_GTvsMPC.xlsx")
OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "significance_annotated_GTvsMPC.png")

metrics = {
    "mean_error_rcm":      "Mean RCM Error (m)",
    "mean_error_ee":       "Mean EE Error (m)",
    "traj_rms_jerk":       "Trajectory RMS Jerk",
    "mean_dist_to_object": "Mean Dist. to Object (m)",
    "mean_control_force":  "Control Linear Force (N)",
    "mean_control_torque": r"Control Torque (N$\cdot$m)",
}

method_disp = {
    "GT_KF":  "GT (proposed)",
    "MPC_KF": "MPC baseline",
}

target_pairs = [("GT_KF", "MPC_KF")]


def get_sig_label(p):
    if p < 0.001: return "***"
    if p < 0.01:  return "**"
    if p < 0.05:  return "*"
    return None


# ==========================================
# 2. Summary table builder
# ==========================================
def generate_summary_table():
    print(f"[+] Building summary from {BASE_LOGS_DIR} ...")
    folders = [f for f in glob.glob(os.path.join(BASE_LOGS_DIR, "*_*_20*_*"))
               if os.path.isdir(f)]
    if not folders:
        raise FileNotFoundError(f"No experiment folders found in {BASE_LOGS_DIR}")

    rows = []
    object_position = np.array([0.3, -0.05, 0.05])
    object_radius = 0.03

    for folder in sorted(folders):
        name = os.path.basename(folder)
        r = {"folder_name": name}
        try:
            err_f = os.path.join(folder, "error.xlsx")
            if os.path.exists(err_f):
                df_e = pd.read_excel(err_f)
                r["mean_error_rcm"] = df_e["error_rcm"].mean() if "error_rcm" in df_e else np.nan
                r["mean_error_ee"]  = df_e["error_ee"].mean()  if "error_ee"  in df_e else np.nan
            else:
                r["mean_error_rcm"], r["mean_error_ee"] = np.nan, np.nan

            force_f = os.path.join(folder, "force.xlsx")
            if os.path.exists(force_f):
                df_f = pd.read_excel(force_f)
                vals = df_f.select_dtypes(include=[np.number]).values
                if len(vals) > 0:
                    fmag = np.linalg.norm(vals, axis=1)
                    r["mean_force_norm"]      = float(np.mean(fmag))
                    r["Efs_force_smoothness"] = float(np.mean(np.diff(fmag) ** 2)) if len(fmag) > 1 else np.nan
                else:
                    r["mean_force_norm"], r["Efs_force_smoothness"] = np.nan, np.nan
            else:
                r["mean_force_norm"], r["Efs_force_smoothness"] = np.nan, np.nan

            traj_f = os.path.join(folder, "file_traj_slave.xlsx")
            if os.path.exists(traj_f):
                df_t = pd.read_excel(traj_f)
                xyz = df_t.select_dtypes(include=[np.number]).iloc[:, :3].values
                r["traj_rms_jerk"] = (
                    float(np.sqrt(np.mean(np.sum(np.diff(xyz, n=3, axis=0) ** 2, axis=1))))
                    if len(xyz) >= 4 else np.nan
                )
                if len(xyz) > 0:
                    d = np.linalg.norm(xyz - object_position, axis=1) - object_radius
                    r["mean_dist_to_object"] = float(np.mean(d))
                else:
                    r["mean_dist_to_object"] = np.nan
            else:
                r["traj_rms_jerk"], r["mean_dist_to_object"] = np.nan, np.nan

            torque_f = os.path.join(folder, "end_torque.xlsx")
            if os.path.exists(torque_f):
                df_et = pd.read_excel(torque_f)
                r["mean_control_force"] = float(np.mean(np.linalg.norm(
                    df_et[["slave_x", "slave_y", "slave_z"]].values, axis=1)))
                r["mean_control_torque"] = float(np.mean(np.linalg.norm(
                    df_et[["slave_a", "slave_b", "slave_c"]].values, axis=1)))
            else:
                r["mean_control_force"], r["mean_control_torque"] = np.nan, np.nan

            rows.append(r)
        except Exception as e:
            print(f"  ! folder {name}: {e}")

    df = pd.DataFrame(rows)
    df.to_excel(INPUT_PATH, index=False)
    print(f"[+] summary saved to {INPUT_PATH}")
    return df


# ==========================================
# 3. Significance testing
# ==========================================
def run_significance(df):
    out = []
    for m1, m2 in target_pairs:
        for k, label in metrics.items():
            d1 = df.loc[df["method"] == m1, k].dropna()
            d2 = df.loc[df["method"] == m2, k].dropna()
            if len(d1) < 2 or len(d2) < 2:
                continue
            try:
                _, ps1 = stats.shapiro(d1)
                _, ps2 = stats.shapiro(d2)
                if ps1 > 0.05 and ps2 > 0.05:
                    eq = stats.levene(d1, d2)[1] > 0.05
                    _, p = stats.ttest_ind(d1, d2, equal_var=eq)
                    test = "t-test"
                else:
                    _, p = stats.mannwhitneyu(d1, d2, alternative="two-sided")
                    test = "Mann-Whitney"
            except Exception:
                p, test = 1.0, "Error"

            out.append({"Comparison": f"{m1} vs {m2}", "Metric": label,
                        "P-Value": p,
                        "Significant": "Yes" if p < 0.05 else "No",
                        "Test": test,
                        f"Mean_{m1}": float(d1.mean()),
                        f"Mean_{m2}": float(d2.mean())})
    return pd.DataFrame(out)


# ==========================================
# 4. Plotting (IEEE 2x3)
# ==========================================
def make_figure(df, results_df):
    apply_ieee_style()

    methods = ["GT_KF", "MPC_KF"]
    df_plot = df[df["method"].isin(methods)].copy()

    fig, axes = plt.subplots(2, 3, figsize=(DBL_W, 4.4))
    axes_flat = axes.flatten()

    box_colors = {
        "GT_KF":  IEEE_COLORS[0],  # blue
        "MPC_KF": IEEE_COLORS[1],  # vermilion
    }

    for i, (k, label) in enumerate(metrics.items()):
        ax = axes_flat[i]
        # Feed the FULL per-method distribution to boxplot so that quartiles
        # match the population used by the significance test and the means
        # reported in the saved xlsx; whis=1.5 + showfliers=False reproduces
        # the original seaborn behaviour without recomputing the box on a
        # truncated sample.
        data_per_method = [
            df_plot.loc[df_plot["method"] == m, k].dropna().values
            for m in methods
        ]

        bp = ax.boxplot(
            data_per_method,
            positions=range(len(methods)),
            widths=0.45,
            patch_artist=True,
            showfliers=False,
            whis=1.5,
            medianprops=dict(color="black", linewidth=1.0),
            whiskerprops=dict(color="black", linewidth=0.7),
            capprops=dict(color="black", linewidth=0.7),
        )
        for patch, m in zip(bp["boxes"], methods):
            patch.set_facecolor(box_colors[m])
            patch.set_alpha(0.65)
            patch.set_edgecolor("black")
            patch.set_linewidth(0.7)

        ax.set_xticks(range(len(methods)))
        ax.set_xticklabels([method_disp[m] for m in methods])
        ax.set_title(label, fontsize=9, pad=4)

        ymax_now = ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 1
        if abs(ymax_now) < 1e-2 or abs(ymax_now) > 1e3:
            fmt = ScalarFormatter(useMathText=True)
            fmt.set_powerlimits((-2, 3))
            ax.yaxis.set_major_formatter(fmt)
            ax.yaxis.get_offset_text().set_fontsize(7)

        # Significance bracket (single comparison only)
        ymin, ymax = ax.get_ylim()
        yr = ymax - ymin
        for m1, m2 in target_pairs:
            row = results_df[(results_df["Comparison"] == f"{m1} vs {m2}") &
                             (results_df["Metric"] == label)]
            if row.empty:
                continue
            p = float(row.iloc[0]["P-Value"])
            tag = get_sig_label(p)
            if not tag:
                continue
            i1, i2 = methods.index(m1), methods.index(m2)
            yl = ymax + 0.04 * yr
            h = 0.02 * yr
            ax.plot([i1, i1, i2, i2],
                    [yl, yl + h, yl + h, yl],
                    lw=0.7, c="black")
            ax.text((i1 + i2) / 2.0, yl + h, tag,
                    ha="center", va="bottom", fontsize=8)
            ax.set_ylim(ymin, yl + 0.10 * yr)

    fig.tight_layout(pad=0.4, h_pad=0.8, w_pad=0.6)
    save_fig(fig, OUTPUT_IMAGE, formats=("png", "pdf"))
    plt.close(fig)


# ==========================================
# 5. Main
# ==========================================
def main():
    if not os.path.exists(INPUT_PATH):
        df = generate_summary_table()
    else:
        df = pd.read_excel(INPUT_PATH)
        if "mean_dist_to_object" not in df.columns:
            df = generate_summary_table()

    if "mean_dist_to_object" in df.columns:
        df.loc[df["mean_dist_to_object"] < 0.042, "mean_dist_to_object"] = np.nan

    df["method"] = df["folder_name"].apply(lambda x: "_".join(x.split("_")[:-2]))

    results_df = run_significance(df)
    print(results_df[["Comparison", "Metric", "P-Value", "Significant"]].to_string(index=False))

    with pd.ExcelWriter(OUTPUT_EXCEL) as w:
        results_df.to_excel(w, sheet_name="Pairwise_Significance", index=False)
        df.groupby("method")[[m for m in metrics if m in df.columns]] \
          .agg(["mean", "std"]).to_excel(w, sheet_name="Raw_Stats_Summary")

    make_figure(df, results_df)


if __name__ == "__main__":
    main()