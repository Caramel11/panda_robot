"""
plot_fuzzy.py  (IEEE-style refactor for Figure 3 panels 1 & 2)
==============================================================
Generates `fis_abs_mf.png` (membership functions of the absolute-channel
authority evaluator) and `fis_abs_surface.png` (absolute-channel control
surface).

Both outputs are sized for a single-column slot at 0.235*\\textwidth, i.e.
roughly 1.7 in × 1.7 in in the camera-ready PDF.  The source files are
rendered at 2.6 in (×1.3 effective scale-up) so 9 pt source fonts arrive at
the print page at ~6 pt — but we increase the rendered font size to 11 pt
so that the displayed effective size is ≈7.2 pt, matching IEEE practice.
"""

import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ieee_style import apply_ieee_style, IEEE_COLORS, save_fig

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "figures"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# ---- bootstrap missing dependency stubs the original code expected ----
if not os.path.exists("kalman_filter.py"):
    with open("kalman_filter.py", "w") as f:
        f.write(
            "class KalmanFilterFusion:\n"
            "    def __init__(self, **k): pass\n"
            "    def update(self, l, t): return l\n"
            "class DeltaLambdaUpdater:\n"
            "    def __init__(self, **k): pass\n"
        )

try:
    from fuzzy_logic import FuzzyLogicTool
except ImportError:
    print("Error: fuzzy_logic.py not found in current directory.")
    sys.exit(1)


# Small bump to source font sizes so panels stay legible after IEEE column
# scaling.  These override the global ieee_style defaults locally.
LOCAL_RC = {
    "font.size": 11,
    "axes.titlesize": 11,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9.5,
}


class FuzzyAcademicVisualizer:
    """Plots the absolute-channel (lambda_based) FIS membership functions
    and 3-D control surface in IEEE-conference style."""

    def __init__(self):
        self.flt = FuzzyLogicTool(type="lambda_based")
        apply_ieee_style()
        plt.rcParams.update(LOCAL_RC)
        self.colors = IEEE_COLORS

    # ---------- membership-function helpers ----------
    @staticmethod
    def _triangular_mf(x, p):
        return np.maximum(
            0,
            np.minimum(
                (x - p[0]) / (p[1] - p[0] + 1e-9),
                (p[2] - x) / (p[2] - p[1] + 1e-9),
            ),
        )

    @staticmethod
    def _trap_left(x, p):
        res = np.zeros_like(x)
        res[(x >= p[0]) & (x <= p[1])] = 1.0
        mask = (x > p[1]) & (x <= p[2])
        res[mask] = (p[2] - x[mask]) / (p[2] - p[1] + 1e-9)
        return res

    @staticmethod
    def _trap_right(x, p):
        res = np.zeros_like(x)
        mask = (x >= p[0]) & (x <= p[1])
        res[mask] = (x[mask] - p[0]) / (p[1] - p[0] + 1e-9)
        res[(x > p[1]) & (x <= p[2])] = 1.0
        return res

    # ---------- 1) membership functions: 2x2 panel ----------
    def plot_membership_functions(self):
        fig, axes = plt.subplots(2, 2, figsize=(5.6, 4.2))

        ranges = [
            np.linspace(0, 2.0, 500),   # F_h
            np.linspace(0, 6.0, 500),   # T_h
            np.linspace(0, 0.1, 500),   # D_r
            np.linspace(0, 1.0, 500),   # lambda
        ]
        titles = [
            r"(a) Interaction Force $F_h$ (N)",
            r"(b) Interaction Duration $T_h$ (s)",
            r"(c) Task Distance $D_r$ (m)",
            r"(d) Arbitration Coefficient $\lambda$",
        ]

        for i in range(3):
            ax = axes.flatten()[i]
            input_set = self.flt.input_sets[i]
            for idx, (name, params) in enumerate(input_set.items()):
                if "PS" in name:
                    y = self._trap_left(ranges[i], params)
                elif "PL" in name:
                    y = self._trap_right(ranges[i], params)
                else:
                    y = self._triangular_mf(ranges[i], params)
                ax.plot(ranges[i], y, label=name,
                        lw=1.4, color=self.colors[idx % len(self.colors)])

            ax.set_title(titles[i], pad=3)
            ax.set_ylabel(r"$\mu(\cdot)$")
            ax.set_ylim(-0.05, 1.12)
            ax.legend(loc="upper right", ncol=1)

        # output: lambda
        ax = axes[1, 1]
        for idx, (name, params) in enumerate(self.flt.output_sets.items()):
            if name == "Z":
                y = self._trap_left(ranges[3], params)
            elif name == "PL":
                y = self._trap_right(ranges[3], params)
            else:
                y = self._triangular_mf(ranges[3], params)
            ax.plot(ranges[3], y, label=name,
                    lw=1.4, color=self.colors[idx % len(self.colors)])
        ax.set_title(titles[3], pad=3)
        ax.set_ylabel(r"$\mu(\lambda)$")
        ax.set_ylim(-0.05, 1.12)
        ax.legend(loc="upper right", ncol=1)

        fig.tight_layout(pad=0.3, h_pad=0.6, w_pad=0.5)
        save_fig(fig, str(OUTPUT_DIR / "fis_abs_mf.png"), formats=("png", "pdf"))
        plt.close(fig)

    # ---------- 2) 3-D control surface ----------
    def plot_3d_surface(self):
        fig = plt.figure(figsize=(4.2, 3.3))
        ax = fig.add_subplot(111, projection="3d")

        F = np.linspace(0, 2, 40)
        T = np.linspace(0, 6, 40)
        F, T = np.meshgrid(F, T)
        L = np.zeros_like(F)
        fixed_dr = 0.03
        for r in range(F.shape[0]):
            for c in range(F.shape[1]):
                L[r, c] = self.flt.compute([F[r, c], T[r, c], fixed_dr])

        surf = ax.plot_surface(F, T, L, cmap="viridis", edgecolor="none",
                               alpha=0.92, antialiased=True, linewidth=0)

        ax.set_xlabel(r"$F_h$ (N)", labelpad=4)
        ax.set_ylabel(r"$T_h$ (s)", labelpad=4)
        ax.set_zlabel(r"$\lambda$", labelpad=2)
        ax.set_title(rf"$D_r = {fixed_dr}$ m", pad=4)

        # Tighten axis tick density for the small panel size
        ax.tick_params(axis="x", pad=0)
        ax.tick_params(axis="y", pad=0)
        ax.tick_params(axis="z", pad=0)

        ax.view_init(elev=24, azim=-130)
        cb = fig.colorbar(surf, shrink=0.55, aspect=12, pad=0.07)
        cb.ax.tick_params(labelsize=8)

        fig.tight_layout(pad=0.2)
        save_fig(fig, str(OUTPUT_DIR / "fis_abs_surface.png"), formats=("png", "pdf"))
        plt.close(fig)

    # ---------- (Optional) rules table image ----------
    def plot_rules_table(self):
        rules = self.flt.rule_dict
        fig, ax = plt.subplots(figsize=(5.6, 3.2))
        ax.axis("off")

        headers = ["ID", r"$F_h$", r"$T_h$", r"$D_r$", r"$\lambda$"]
        data = [[i + 1, k[0], k[1], k[2], v] for i, (k, v) in enumerate(rules.items())]

        table = ax.table(cellText=[headers] + data,
                         loc="center", cellLoc="center",
                         colWidths=[0.10, 0.21, 0.21, 0.21, 0.18])
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.25)
        for j in range(5):
            table[(0, j)].set_facecolor("#EEEEEE")
            table[(0, j)].get_text().set_weight("bold")

        save_fig(fig, str(OUTPUT_DIR / "fuzzy_rules_table.png"), formats=("png", "pdf"))
        plt.close(fig)


if __name__ == "__main__":
    viz = FuzzyAcademicVisualizer()
    viz.plot_membership_functions()
    viz.plot_3d_surface()
    viz.plot_rules_table()
    print("Done.")
