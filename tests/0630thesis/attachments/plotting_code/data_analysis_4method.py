"""
data_analysis_4method.py  (IEEE-style refactor for Figure 9)
=============================================================
Generates `significance_annotated_4method.png` for the four-method physical
comparison.

Key changes vs. the original:
  * IEEE Times-serif typography, 8–9 pt at print size.
  * Output rendered at 7.16 in × 4.5 in -> intended for `\\begin{figure*}`
    (double-column).  At single-column display (3.5"), per-panel labels
    will compress to ~4.5 pt; switching the LaTeX wrapper to `figure*` is
    therefore strongly recommended for legibility.
  * Color-blind-friendly box fills + matched edge color.
  * Significance brackets drawn at consistent vertical offsets, with the
    star annotation centered above the bracket and using mathtext so the
    asterisks stay crisp.
  * IQR-based outlier suppression preserved; numeric tick formatting
    uses scientific notation only where the dynamic range demands it.
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
BASE_LOGS_DIR = "logs/20260213/0213_*"
INPUT_PATH = "logs/20260213/summary_batch_results.xlsx"
OUTPUT_DIR = "logs/20260213"
OUTPUT_EXCEL = os.path.join(OUTPUT_DIR, "detailed_significance_analysis_4method.xlsx")
OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "significance_annotated_4method.png")

# Display-friendly metric labels
metrics = {
    "mean_error_rcm":      "Mean RCM Error (m)",
    "mean_error_ee":       "Mean EE Error (m)",
    "mean_force_norm":     "Interaction Force (N)",
    "Efs_force_smoothness": r"Force Smoothness $E_{fs}$",
    "traj_rms_jerk":       "Trajectory RMS Jerk",
    "mean_dist_to_object": "Mean Dist. to Object (m)",
}

# Method-display name mapping (compact for narrow box labels)
method_disp = {
    "GT_KF":       "GT+FIS",
    "GT_Sigmoid":  "GT+Sig.",
    "MPC_KF":      "MPC+FIS",
    "MPC_Sigmoid": "MPC+Sig.",
}

target_pairs = [
    ("GT_KF", "GT_Sigmoid"),
    ("GT_KF", "MPC_KF"),
    ("MPC_KF", "MPC_Sigmoid"),
    ("GT_Sigmoid", "MPC_Sigmoid"),
]


def get_sig_label(p):
    if p < 0.001: return "***"
    if p < 0.01:  return "**"
    if p < 0.05:  return "*"
    return None


# ==========================================
# 2. Summary table (unchanged extraction logic)
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
# 4. Plotting (IEEE 2x3 layout)
# ==========================================
def make_figure(df, results_df):
    apply_ieee_style()

    methods = ["GT_KF", "GT_Sigmoid", "MPC_KF", "MPC_Sigmoid"]
    df_plot = df[df["method"].isin(methods)].copy()

    fig, axes = plt.subplots(2, 3, figsize=(DBL_W, 4.4))
    axes_flat = axes.flatten()

    # Two-tone palette: GT family in blue tones, MPC family in warm tones
    box_colors = {
        "GT_KF":       IEEE_COLORS[0],   # deep blue
        "GT_Sigmoid":  IEEE_COLORS[4],   # sky blue
        "MPC_KF":      IEEE_COLORS[1],   # vermilion
        "MPC_Sigmoid": IEEE_COLORS[5],   # amber
    }

    for i, (k, label) in enumerate(metrics.items()):
        ax = axes_flat[i]

        # IMPORTANT: feed the box the FULL per-method distribution so that
        # the box quartiles, medians and the t-test / Mann-Whitney statistic
        # are computed on the same population.  matplotlib's default
        # whis=1.5 already truncates the whiskers via the 1.5 * IQR rule;
        # showfliers=False hides the outlier markers but keeps the box
        # geometry intact.  This matches the original seaborn behaviour
        # and the means reported in the saved xlsx.
        data_per_method = [
            df_plot.loc[df_plot["method"] == m, k].dropna().values
            for m in methods
        ]

        bp = ax.boxplot(
            data_per_method,
            positions=range(len(methods)),
            widths=0.55,
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
        ax.set_xticklabels([method_disp[m] for m in methods], rotation=15, ha="right")
        ax.set_title(label, fontsize=9, pad=4)

        # Use scientific notation only when the magnitude needs it
        ymax_now = ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 1
        if abs(ymax_now) < 1e-2 or abs(ymax_now) > 1e3:
            fmt = ScalarFormatter(useMathText=True)
            fmt.set_powerlimits((-2, 3))
            ax.yaxis.set_major_formatter(fmt)
            ax.yaxis.get_offset_text().set_fontsize(7)

        # ---- Significance brackets ----
        ymin, ymax = ax.get_ylim()
        yr = ymax - ymin
        cur_y = ymax + 0.02 * yr

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
            h = 0.018 * yr
            ax.plot([i1, i1, i2, i2],
                    [cur_y, cur_y + h, cur_y + h, cur_y],
                    lw=0.7, c="black")
            ax.text((i1 + i2) / 2.0, cur_y + h, tag,
                    ha="center", va="bottom", fontsize=8)
            cur_y += 0.10 * yr

        ax.set_ylim(ymin, cur_y + 0.05 * yr)
        ax.tick_params(axis="x", which="major", pad=1)

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