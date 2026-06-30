"""
plot_gt_analysis.py  (IEEE-style refactor for Figure 7 panels 2 & 5)
====================================================================
Generates the synchronized 4-panel time-series view (force + arbitration
coefficients, RCM/EE error, posterior probabilities, end-effector torque)
for two physical-experiment runs:

  panel 2 (top row):    20260214_015547  ->  *_OtherTarget_false.png
  panel 5 (bottom row): 20260214_064530  ->  *_true.png

Each output is sized for a third of a `figure*` row in the camera-ready PDF
(~2.3 in wide), rendered at 3.5 in × 4.6 in to keep axis labels and legends
legible after scaling.
"""

import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ieee_style import apply_ieee_style, IEEE_COLORS, save_fig


# ==========================================
# Datasets to render
# ==========================================
DATASETS = [
    ("logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_015547",
     "experiment_parameter_sync_analysis_OtherTarget_false.png"),
    ("logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_064530",
     "experiment_parameter_sync_analysis_true.png"),
]


# ==========================================
# Helpers
# ==========================================
def load(base_path, filename):
    p = os.path.join(base_path, filename)
    if not os.path.exists(p):
        print(f"  ! missing: {p}")
        return None
    return pd.read_excel(p)


def detect_force_events(df_force, threshold=0.05):
    f_norm = np.sqrt(df_force["master_x"]**2 +
                     df_force["master_y"]**2 +
                     df_force["master_z"]**2)
    active = (f_norm > threshold).astype(int)
    diff = active.diff()
    return f_norm, diff[diff == 1].index.tolist(), diff[diff == -1].index.tolist()


# ==========================================
# Main figure (4-panel synchronized view)
# ==========================================
def plot_parameter_sync(base_path, output_png):
    apply_ieee_style()

    df_force  = load(base_path, "force.xlsx")
    df_error  = load(base_path, "error.xlsx")
    df_arb    = load(base_path, "arbitrary.xlsx")
    df_torque = load(base_path, "end_torque.xlsx")
    df_prob   = load(base_path, "posterior_probabilities.xlsx")

    if df_force is None or df_error is None or df_arb is None \
            or df_torque is None or df_prob is None:
        print(f"  ! skipping {output_png}: missing required files")
        return

    f_norm, starts, ends = detect_force_events(df_force)
    n = min(len(df_force), len(df_error), len(df_arb), len(df_prob))
    t = np.arange(n)

    fig, axes = plt.subplots(4, 1, figsize=(3.5, 4.6), sharex=True)

    # --- (a) |F_h| + alpha/beta ---
    ax = axes[0]
    ax.plot(t, f_norm[:n], color="black", lw=0.9, label=r"$|F_h|$")
    axt = ax.twinx()
    axt.plot(t, df_arb["alpha"][:n], color=IEEE_COLORS[2], lw=0.9, label=r"$\alpha$")
    axt.plot(t, df_arb["beta"][:n],  color=IEEE_COLORS[1], lw=0.9, ls="--", label=r"$\beta$")
    ax.set_ylabel("Force (N)")
    axt.set_ylabel("Weight")
    axt.grid(False)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = axt.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper right", ncol=3,
              fontsize=7, handlelength=1.3, columnspacing=0.6)
    ax.set_title("(a) Force and arbitration weight", pad=2, fontsize=8)

    # --- (b) RCM and EE errors ---
    ax = axes[1]
    ax.plot(t, df_error["error_rcm"][:n], color=IEEE_COLORS[0], lw=0.9, label="RCM")
    ax.plot(t, df_error["error_ee"][:n],  color=IEEE_COLORS[1], lw=0.9, ls="--", label="EE")
    ax.set_ylabel("Error (m)")
    ax.legend(loc="upper right", ncol=2, fontsize=7, handlelength=1.3, columnspacing=0.6)
    ax.set_title("(b) Tracking error", pad=2, fontsize=8)

    # --- (c) posterior probabilities ---
    ax = axes[2]
    df_prob.columns = ["P_a", "P_b", "P_c"]
    for i, key in enumerate(["P_a", "P_b", "P_c"]):
        ax.plot(t, df_prob[key][:n], color=IEEE_COLORS[i], lw=0.9,
                label=f"Tgt {key.split('_')[1]}")
    ax.set_ylabel("Posterior")
    ax.set_ylim(-0.05, 1.05)
    ax.legend(loc="upper right", ncol=3, fontsize=7, handlelength=1.3, columnspacing=0.6)
    ax.set_title("(c) Intent posterior", pad=2, fontsize=8)

    # --- (d) end-effector torque magnitude ---
    ax = axes[3]
    tmag = np.sqrt(df_torque["slave_x"]**2 + df_torque["slave_y"]**2 + df_torque["slave_z"]**2)
    ax.plot(t, tmag[:n], color=IEEE_COLORS[3], lw=0.9, label=r"$\|T_{ee}\|$")
    ax.set_ylabel(r"Torque (N$\cdot$m)")
    ax.set_xlabel("Sample index")
    ax.legend(loc="upper right", fontsize=7, handlelength=1.3)
    ax.set_title("(d) End-effector torque", pad=2, fontsize=8)

    # vertical event markers on every panel
    def mark(ax_):
        for s in starts:
            if s < n:
                ax_.axvline(x=s, color=IEEE_COLORS[0], ls=":", lw=0.6, alpha=0.55)
        for e in ends:
            if e < n:
                ax_.axvline(x=e, color=IEEE_COLORS[1], ls=":", lw=0.6, alpha=0.55)
    for ax_ in axes:
        mark(ax_)

    fig.tight_layout(pad=0.25, h_pad=0.4)
    save_fig(fig, output_png, formats=("png", "pdf"))
    plt.close(fig)
    print(f"Done -> {output_png}")


if __name__ == "__main__":
    for base, out in DATASETS:
        print(f"[+] processing {base}")
        plot_parameter_sync(base, out)
