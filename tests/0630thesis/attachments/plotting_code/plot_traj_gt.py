"""
plot_traj_gt.py  (IEEE-style refactor for Figure 7 panels 1, 3, 4, 6)
======================================================================
Generates trajectory-level visualisations of two physical-experiment
runs from the GT_KF_WithOtherTarget condition:

  Top row    (Figure 7, panels 1 & 3):   20260214_015547  ->  *_OtherTarget_false.png
  Bottom row (Figure 7, panels 4 & 6):   20260214_064530  ->  *_true.png

For each run, two figures are produced:
  (i)  3-D trajectory comparison overlaid with detected interaction-force
       events (Slave / Master / supporting reference signals + target stars).
  (ii) Per-axis (X, Y, Z) time-series with mean absolute tracking error
       between Slave and Master annotated in millimetres.

Outputs are sized for a third of a `figure*` row in the camera-ready PDF
(~2.3 in wide), but rendered at a slightly larger natural size so axis
labels, legends, and event markers remain crisp after scaling.

Run from the repository root so that the relative `logs/` paths resolve.
"""

import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ieee_style import apply_ieee_style, IEEE_COLORS, save_fig


# ==========================================
# Datasets (base_path, suffix used for output filenames)
# ==========================================
DATASETS = [
    ("logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_015547",
     "OtherTarget_false"),
    ("logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_064530",
     "true"),
]

# Target points used by the intent-arbitration experiment
TARGETS = {
    "a": (0.25, 0.00, 0.03),
    "b": (0.30, 0.00, 0.03),
    "c": (0.35, 0.00, 0.03),
}

# Trajectory streams to overlay on the 3-D plot
TRAJ_FILES = {
    "Slave":   "file_traj_slave.xlsx",
    "Master":  "file_traj_master.xlsx",
    "Robot":   "file_traj_robot.xlsx",
    "Human":   "file_traj_human.xlsx",
    "Reshape": "file_traj_reshape.xlsx",
    "Direct":  "file_traj_direct.xlsx",
}


# ==========================================
# Helpers
# ==========================================
def load(base_path, filename):
    """Load an Excel file from `base_path` or return None with a warning."""
    p = os.path.join(base_path, filename)
    if not os.path.exists(p):
        print(f"  ! missing: {p}")
        return None
    return pd.read_excel(p)


def detect_force_events(df_force, threshold=0.05):
    """Return (force-norm series, start indices, end indices)."""
    f_norm = np.sqrt(df_force["master_x"] ** 2 +
                     df_force["master_y"] ** 2 +
                     df_force["master_z"] ** 2)
    active = (f_norm > threshold).astype(int)
    diff = active.diff()
    starts = diff[diff == 1].index.tolist()
    ends = diff[diff == -1].index.tolist()
    return f_norm, starts, ends


def normalise_xyz(df):
    """Pick the right (x,y,z) columns from either master_* or slave_* streams."""
    xc = "slave_x" if "slave_x" in df.columns else "master_x"
    yc = "slave_y" if "slave_y" in df.columns else "master_y"
    zc = "slave_z" if "slave_z" in df.columns else "master_z"
    return df[[xc, yc, zc]].rename(columns={xc: "x", yc: "y", zc: "z"})


# ==========================================
# Figure 1: 3-D trajectory comparison with force-event markers
# ==========================================
def plot_3d_trajectory(traj_data, starts, ends, output_png):
    apply_ieee_style()

    fig = plt.figure(figsize=(3.5, 3.0))
    ax = fig.add_subplot(111, projection="3d")

    # --- target stars ---
    for name, pos in TARGETS.items():
        ax.scatter(pos[0], pos[1], pos[2],
                   s=70, marker="*",
                   color=IEEE_COLORS[6],          # black
                   edgecolors="white", linewidths=0.4,
                   zorder=15)
        ax.text(pos[0], pos[1], pos[2] + 0.005, f"  {name}",
                fontsize=7, fontweight="bold")

    # --- supporting trajectories (light) ---
    support_streams = [k for k in traj_data if k not in ("Slave", "Master")]
    for i, label in enumerate(support_streams):
        d = traj_data[label]
        ax.plot(d["x"], d["y"], d["z"],
                lw=0.55, alpha=0.55,
                color=IEEE_COLORS[(i + 2) % 6],
                label=label)

    # --- emphasised streams: Master (reference) and Slave (actual) ---
    if "Master" in traj_data:
        d = traj_data["Master"]
        ax.plot(d["x"], d["y"], d["z"],
                lw=1.4, color=IEEE_COLORS[0], alpha=0.9,
                label="Master (ref.)", zorder=8)

    if "Slave" in traj_data:
        d = traj_data["Slave"]
        ax.plot(d["x"], d["y"], d["z"],
                lw=1.7, color=IEEE_COLORS[1], alpha=1.0,
                label="Slave (actual)", zorder=10)

        # --- force-event markers placed on the slave trajectory ---
        for i, s_idx in enumerate(starts):
            if s_idx < len(d):
                p = d.iloc[s_idx]
                ax.scatter(p["x"], p["y"], p["z"],
                           color=IEEE_COLORS[2], s=22, marker="o",
                           edgecolors="white", linewidths=0.4, zorder=20,
                           label="Force start" if i == 0 else None)
        for i, e_idx in enumerate(ends):
            if e_idx < len(d):
                p = d.iloc[e_idx]
                ax.scatter(p["x"], p["y"], p["z"],
                           color=IEEE_COLORS[3], s=22, marker="s",
                           edgecolors="white", linewidths=0.4, zorder=20,
                           label="Force end" if i == 0 else None)

    ax.set_xlabel("X (m)", labelpad=2)
    ax.set_ylabel("Y (m)", labelpad=2)
    ax.set_zlabel("Z (m)", labelpad=2)
    ax.tick_params(axis="both", labelsize=6, pad=1)
    ax.tick_params(axis="z",    labelsize=6, pad=1)

    # 3-D projections look better with a slightly elevated viewpoint
    ax.view_init(elev=24, azim=-52)

    # legend de-duplication (force-start/end markers loop)
    handles, labels = ax.get_legend_handles_labels()
    seen, h_uniq, l_uniq = set(), [], []
    for h, lab in zip(handles, labels):
        if lab not in seen:
            seen.add(lab)
            h_uniq.append(h)
            l_uniq.append(lab)
    ax.legend(h_uniq, l_uniq,
              loc="upper left", bbox_to_anchor=(-0.18, 1.02),
              fontsize=6, handlelength=1.3, columnspacing=0.6,
              ncol=1, framealpha=0.85)

    fig.tight_layout(pad=0.25)
    save_fig(fig, output_png, formats=("png", "pdf"))
    plt.close(fig)


# ==========================================
# Figure 2: XYZ time-series with MAE annotation
# ==========================================
def plot_xyz_with_mae(traj_data, starts, ends, output_png):
    apply_ieee_style()

    coords = ["x", "y", "z"]
    coord_titles = ["(a) X position", "(b) Y position", "(c) Z position"]

    # Compute mean absolute tracking error (Slave vs Master) per axis [mm]
    mae = {}
    if "Slave" in traj_data and "Master" in traj_data:
        s, m = traj_data["Slave"], traj_data["Master"]
        L = min(len(s), len(m))
        for c in coords:
            mae[c] = float(np.abs(s[c].iloc[:L].values -
                                  m[c].iloc[:L].values).mean() * 1000.0)

    fig, axes = plt.subplots(3, 1, figsize=(3.5, 4.6), sharex=True)

    for i, (c, title) in enumerate(zip(coords, coord_titles)):
        ax = axes[i]

        # supporting streams in muted colours
        support_streams = [k for k in traj_data if k not in ("Slave", "Master")]
        for j, label in enumerate(support_streams):
            d = traj_data[label]
            ax.plot(d[c].values,
                    color=IEEE_COLORS[(j + 2) % 6],
                    lw=0.55, alpha=0.55, label=label if i == 0 else None)

        # emphasised streams
        if "Master" in traj_data:
            ax.plot(traj_data["Master"][c].values,
                    color=IEEE_COLORS[0], lw=1.1, alpha=0.95,
                    label="Master" if i == 0 else None)
        if "Slave" in traj_data:
            ax.plot(traj_data["Slave"][c].values,
                    color=IEEE_COLORS[1], lw=1.3, alpha=1.0,
                    label="Slave" if i == 0 else None)

        # MAE annotation (top-left of each axis)
        if c in mae:
            ax.text(0.02, 0.94,
                    f"MAE = {mae[c]:.3f} mm",
                    transform=ax.transAxes,
                    fontsize=7, fontweight="bold",
                    va="top", ha="left",
                    bbox=dict(boxstyle="round,pad=0.18",
                              facecolor="#FFF8DC",
                              edgecolor="#000000",
                              linewidth=0.5,
                              alpha=0.9))

        # vertical force-event guides
        for s in starts:
            ax.axvline(x=s, color=IEEE_COLORS[2], ls=":", lw=0.5, alpha=0.55)
        for e in ends:
            ax.axvline(x=e, color=IEEE_COLORS[3], ls=":", lw=0.5, alpha=0.55)

        ax.set_ylabel(f"{c.upper()} (m)")
        ax.set_title(title, fontsize=8, pad=2)

    axes[-1].set_xlabel("Sample index")

    # one legend at the top, shared across panels
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels,
                   loc="upper center", bbox_to_anchor=(0.5, 1.02),
                   ncol=min(len(labels), 4),
                   fontsize=7, handlelength=1.3, columnspacing=0.8,
                   framealpha=0.9)

    fig.tight_layout(pad=0.3, h_pad=0.5, rect=(0, 0, 1, 0.97))
    save_fig(fig, output_png, formats=("png", "pdf"))
    plt.close(fig)


# ==========================================
# Per-dataset driver
# ==========================================
def render_dataset(base_path, suffix):
    print(f"[+] processing {base_path}")

    df_force = load(base_path, "force.xlsx")
    if df_force is None:
        print(f"  ! skipping (no force.xlsx)")
        return
    _, starts, ends = detect_force_events(df_force)

    traj_data = {}
    for label, fname in TRAJ_FILES.items():
        df = load(base_path, fname)
        if df is None:
            continue
        traj_data[label] = normalise_xyz(df)

    if not traj_data:
        print(f"  ! no trajectory files found, skipping")
        return

    out_3d  = f"experiment_3d_trajectories_final_{suffix}.png"
    out_xyz = f"trajectory_xyz_with_mae_{suffix}.png"

    plot_3d_trajectory(traj_data, starts, ends, out_3d)
    plot_xyz_with_mae(traj_data,  starts, ends, out_xyz)

    print(f"  ok -> {out_3d}\n  ok -> {out_xyz}")


if __name__ == "__main__":
    for base_path, suffix in DATASETS:
        render_dataset(base_path, suffix)
