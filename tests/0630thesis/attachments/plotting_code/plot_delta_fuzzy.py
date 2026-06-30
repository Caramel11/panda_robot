"""
plot_delta_fuzzy.py  (IEEE-style refactor for Figure 3 panels 3 & 4)
====================================================================
Generates `fis_inc_mf.png` (membership functions of the incremental-channel
authority evaluator) and `fis_inc_surface.png` (incremental-channel control
surface).

This is the delta_lambda_based companion to plot_fuzzy.py.  Same sizing
philosophy: render slightly larger than the camera-ready slot and use 11 pt
local font sizes so that legends and axis labels remain legible after the
0.235*\\textwidth scaling in the LaTeX source.
"""

import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

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


LOCAL_RC = {
    "font.size": 11,
    "axes.titlesize": 11,
    "axes.labelsize": 11,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9.5,
}


class DeltaFuzzyAcademicVisualizer:
    """Plots the incremental-channel (delta_lambda_based) FIS membership
    functions and 3-D control surface in IEEE style."""

    def __init__(self):
        self.flt = FuzzyLogicTool(type="delta_lambda_based")
        apply_ieee_style()
        plt.rcParams.update(LOCAL_RC)
        self.colors = IEEE_COLORS

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
            np.linspace(-3.0, 3.0, 500),    # dF_h
            np.linspace(0,    2.0, 500),    # dT_h
            np.linspace(-0.08, 0.08, 500),  # dD_r
            np.linspace(-0.5,  0.5, 500),   # delta lambda
        ]
        titles = [
            r"(a) $\Delta F_h$ (N/s)",
            r"(b) $\Delta T_h$ (1)",
            r"(c) $\Delta D_r$ (cm/s)",
            r"(d) $\Delta\lambda$ output",
        ]

        for i in range(3):
            ax = axes.flatten()[i]
            input_set = self.flt.input_sets[i]
            for idx, (name, params) in enumerate(input_set.items()):
                if name == "N":
                    y = self._trap_left(ranges[i], params)
                elif name == "P":
                    y = self._trap_right(ranges[i], params)
                else:
                    y = self._triangular_mf(ranges[i], params)
                ax.plot(ranges[i], y, label=name,
                        lw=1.4, color=self.colors[idx % len(self.colors)])

            ax.set_title(titles[i], pad=3)
            ax.set_ylabel(r"$\mu(\cdot)$")
            ax.set_ylim(-0.05, 1.12)
            ax.legend(loc="upper right")

        ax = axes[1, 1]
        for idx, (name, params) in enumerate(self.flt.output_sets.items()):
            if name == "NL":
                y = self._trap_left(ranges[3], params)
            elif name == "PL":
                y = self._trap_right(ranges[3], params)
            else:
                y = self._triangular_mf(ranges[3], params)
            ax.plot(ranges[3], y, label=name,
                    lw=1.4, color=self.colors[idx % len(self.colors)])
        ax.set_title(titles[3], pad=3)
        ax.set_ylabel(r"$\mu(\Delta\lambda)$")
        ax.set_ylim(-0.05, 1.12)
        ax.legend(loc="upper right")

        fig.tight_layout(pad=0.3, h_pad=0.6, w_pad=0.5)
        save_fig(fig, str(OUTPUT_DIR / "fis_inc_mf.png"), formats=("png", "pdf"))
        plt.close(fig)

    # ---------- 2) 3-D control surface ----------
    def plot_3d_surface(self):
        fig = plt.figure(figsize=(4.2, 3.3))
        ax = fig.add_subplot(111, projection="3d")

        DF = np.linspace(-3.0, 3.0, 40)
        DT = np.linspace(-1.5, 1.5, 40)
        DF, DT = np.meshgrid(DF, DT)
        DL = np.zeros_like(DF)
        fixed_ddr = 0.0
        for r in range(DF.shape[0]):
            for c in range(DF.shape[1]):
                DL[r, c] = self.flt.compute([DF[r, c], DT[r, c], fixed_ddr],
                                            types="delta_lambda_based")

        surf = ax.plot_surface(DF, DT, DL, cmap="coolwarm", edgecolor="none",
                               alpha=0.92, antialiased=True, linewidth=0)

        ax.set_xlabel(r"$\Delta F_h$ (N/s)", labelpad=4)
        ax.set_ylabel(r"$\Delta T_h$ (1)", labelpad=4)
        ax.set_zlabel(r"$\Delta\lambda$", labelpad=2)
        ax.set_title(rf"$\Delta D_r={fixed_ddr}$", pad=4)

        ax.tick_params(axis="x", pad=0)
        ax.tick_params(axis="y", pad=0)
        ax.tick_params(axis="z", pad=0)

        ax.view_init(elev=24, azim=-130)
        cb = fig.colorbar(surf, shrink=0.55, aspect=12, pad=0.07)
        cb.ax.tick_params(labelsize=8)

        fig.tight_layout(pad=0.2)
        save_fig(fig, str(OUTPUT_DIR / "fis_inc_surface.png"), formats=("png", "pdf"))
        plt.close(fig)

    # ---------- (optional) rule table ----------
    def plot_rules_table(self):
        rules = self.flt.rule_dict
        fig, ax = plt.subplots(figsize=(5.6, 3.2))
        ax.axis("off")

        headers = ["ID", r"$\Delta F_h$", r"$\Delta T_h$", r"$\Delta D_r$", r"$\Delta\lambda$"]
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

        save_fig(fig, str(OUTPUT_DIR / "delta_fuzzy_rules_table.png"), formats=("png", "pdf"))
        plt.close(fig)


if __name__ == "__main__":
    viz = DeltaFuzzyAcademicVisualizer()
    viz.plot_membership_functions()
    viz.plot_3d_surface()
    viz.plot_rules_table()
    print("Done.")
