#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Regenerate Fig. 3 from the original fuzzy-logic library files.

This script is a faithful IEEE-style rewrite of the original:
  - plot_fuzzy.py       -> Fig. 3(a), Fig. 3(b)
  - plot_delta_fuzzy.py -> Fig. 3(c), Fig. 3(d)

Required files in the same directory or on PYTHONPATH:
  - fuzzy_logic.py
  - kalman_filter.py

Outputs:
  - fig3a_absolute_membership_ieee.pdf/png
  - fig3b_absolute_surface_ieee.pdf/png
  - fig3c_increment_membership_ieee.pdf/png
  - fig3d_increment_surface_ieee.pdf/png
  - fig3_combined_ieee.pdf/png

Important: Fig. 3(a) and Fig. 3(c) are NOT single output-membership plots.
They reproduce the original 2x2 membership-function panels:
  Fig. 3(a): F_h, T_h, D_r, lambda
  Fig. 3(c): Delta F_h, Delta T_h, Delta D_r, Delta lambda
"""

import argparse
import inspect
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

try:
    from fuzzy_logic import FuzzyLogicTool
except Exception as exc:
    print("ERROR: failed to import FuzzyLogicTool from fuzzy_logic.py")
    print(exc)
    sys.exit(1)


# -------------------------------
# IEEE-style plotting parameters
# -------------------------------
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "dejavuserif",
    "axes.unicode_minus": False,
    "axes.linewidth": 0.8,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.major.width": 0.8,
    "ytick.major.width": 0.8,
    "xtick.major.size": 3.0,
    "ytick.major.size": 3.0,
    "legend.frameon": True,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]


def triangular_mf_array(x, p):
    a, b, c = p
    return np.maximum(
        0.0,
        np.minimum((x - a) / (b - a + 1e-12), (c - x) / (c - b + 1e-12)),
    )


def trap_left_array(x, p):
    a, b, c = p
    y = np.zeros_like(x, dtype=float)
    y[(x >= a) & (x <= b)] = 1.0
    mask = (x > b) & (x <= c)
    y[mask] = (c - x[mask]) / (c - b + 1e-12)
    return y


def trap_right_array(x, p):
    a, b, c = p
    y = np.zeros_like(x, dtype=float)
    mask = (x >= a) & (x <= b)
    y[mask] = (x[mask] - a) / (b - a + 1e-12)
    y[(x > b) & (x <= c)] = 1.0
    return y


def mf_curve(x, params, label, mode, io="input"):
    """Vectorized MF evaluator matching the original fuzzy_logic.py / plotting scripts."""
    if mode == "lambda_based":
        # Original plot_fuzzy.py logic:
        # input: PS=left trapezoid, PL=right trapezoid, PM=triangle
        # output: Z=left trapezoid, PL=right trapezoid, others=triangle
        if io == "input":
            if label == "PS":
                return trap_left_array(x, params)
            if label == "PL":
                return trap_right_array(x, params)
            return triangular_mf_array(x, params)
        else:
            if label == "Z":
                return trap_left_array(x, params)
            if label == "PL":
                return trap_right_array(x, params)
            return triangular_mf_array(x, params)
    else:
        # Original plot_delta_fuzzy.py logic:
        # input: N=left trapezoid, P=right trapezoid, Z=triangle
        # output: NL=left trapezoid, PL=right trapezoid, others=triangle
        if io == "input":
            if label == "N":
                return trap_left_array(x, params)
            if label == "P":
                return trap_right_array(x, params)
            return triangular_mf_array(x, params)
        else:
            if label == "NL":
                return trap_left_array(x, params)
            if label == "PL":
                return trap_right_array(x, params)
            return triangular_mf_array(x, params)


def compute_fuzzy(tool, inputs, mode):
    """Compatible wrapper for different historical compute signatures.

    The original scripts use both:
      compute(inputs)
      compute(inputs, types="delta_lambda_based")
    This wrapper first tries the faithful positional mode call, then falls back.
    """
    attempts = [
        lambda: tool.compute(inputs, mode),
        lambda: tool.compute(inputs, types=mode),
        lambda: tool.compute(inputs),
        lambda: tool.compute(*inputs),
    ]
    last = None
    for fn in attempts:
        try:
            return float(fn())
        except TypeError as exc:
            last = exc
            continue
    raise TypeError(f"Could not call FuzzyLogicTool.compute with compatible signature: {last}")


def save_figure(fig, out_dir, stem):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / f"{stem}.pdf")
    fig.savefig(out_dir / f"{stem}.png", dpi=600)
    plt.close(fig)


def plot_absolute_membership(abs_tool, out_dir):
    """Fig. 3(a): faithful 2x2 absolute FIS MF panel from plot_fuzzy.py."""
    ranges = [
        np.linspace(0.0, 2.0, 600),   # F_h
        np.linspace(0.0, 6.0, 600),   # T_h
        np.linspace(0.0, 0.10, 600),  # D_r
        np.linspace(0.0, 1.0, 600),   # lambda
    ]
    titles = [
        r"$F_h$ membership",
        r"$T_h$ membership",
        r"$D_r$ membership",
        r"$\lambda$ membership",
    ]
    xlabels = [
        r"Interaction force $F_h$ (N)",
        r"Intervention duration $T_h$ (s)",
        r"Task distance $D_r$ (m)",
        r"Authority coefficient $\lambda$",
    ]

    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.2))
    axes = axes.ravel()

    for k in range(3):
        ax = axes[k]
        for idx, (name, params) in enumerate(abs_tool.input_sets[k].items()):
            ax.plot(ranges[k], mf_curve(ranges[k], params, name, "lambda_based", "input"),
                    lw=2.0, color=COLORS[idx % len(COLORS)], label=name)
        ax.set_title(titles[k], fontsize=9.5, fontweight="bold")
        ax.set_xlabel(xlabels[k], fontsize=8.5)
        ax.set_ylabel("Membership", fontsize=8.5)
        ax.set_ylim(-0.02, 1.06)
        ax.tick_params(labelsize=8)
        ax.grid(True, linestyle="--", linewidth=0.45, alpha=0.45)
        ax.legend(fontsize=7.5, loc="upper right", ncol=3 if k == 0 else 1)

    ax = axes[3]
    for idx, (name, params) in enumerate(abs_tool.output_sets.items()):
        ax.plot(ranges[3], mf_curve(ranges[3], params, name, "lambda_based", "output"),
                lw=2.0, color=COLORS[idx % len(COLORS)], label=name)
    ax.set_title(titles[3], fontsize=9.5, fontweight="bold")
    ax.set_xlabel(xlabels[3], fontsize=8.5)
    ax.set_ylabel("Membership", fontsize=8.5)
    ax.set_ylim(-0.02, 1.06)
    ax.tick_params(labelsize=8)
    ax.grid(True, linestyle="--", linewidth=0.45, alpha=0.45)
    ax.legend(fontsize=7.5, loc="upper center", ncol=5)

    fig.suptitle("Absolute FIS membership functions", fontsize=11.0, fontweight="bold", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    save_figure(fig, out_dir, "fig3a_absolute_membership_ieee")


def plot_increment_membership(delta_tool, out_dir):
    """Fig. 3(c): faithful 2x2 incremental FIS MF panel from plot_delta_fuzzy.py."""
    ranges = [
        np.linspace(-3.0, 3.0, 600),    # dF_h
        np.linspace(0.0, 2.0, 600),     # dT_h
        np.linspace(-0.08, 0.08, 600),  # dD_r
        np.linspace(-0.5, 0.5, 600),    # delta lambda
    ]
    titles = [
        r"$\dot F_h$ membership",
        r"$\dot T_h$ membership",
        r"$\dot D_r$ membership",
        r"$\Delta\lambda$ membership",
    ]
    xlabels = [
        r"Change in force $\dot F_h$ (N/s)",
        r"Change in duration $\dot T_h$",
        r"Change in distance $\dot D_r$ (m/s)",
        r"Authority increment $\Delta\lambda$",
    ]

    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.2))
    axes = axes.ravel()

    for k in range(3):
        ax = axes[k]
        for idx, (name, params) in enumerate(delta_tool.input_sets[k].items()):
            ax.plot(ranges[k], mf_curve(ranges[k], params, name, "delta_lambda_based", "input"),
                    lw=2.0, color=COLORS[idx % len(COLORS)], label=name)
        ax.set_title(titles[k], fontsize=9.5, fontweight="bold")
        ax.set_xlabel(xlabels[k], fontsize=8.5)
        ax.set_ylabel("Membership", fontsize=8.5)
        ax.set_ylim(-0.02, 1.06)
        ax.tick_params(labelsize=8)
        ax.grid(True, linestyle="--", linewidth=0.45, alpha=0.45)
        ax.legend(fontsize=7.5, loc="upper right", ncol=3 if k == 0 else 1)

    ax = axes[3]
    for idx, (name, params) in enumerate(delta_tool.output_sets.items()):
        ax.plot(ranges[3], mf_curve(ranges[3], params, name, "delta_lambda_based", "output"),
                lw=2.0, color=COLORS[idx % len(COLORS)], label=name)
    ax.set_title(titles[3], fontsize=9.5, fontweight="bold")
    ax.set_xlabel(xlabels[3], fontsize=8.5)
    ax.set_ylabel("Membership", fontsize=8.5)
    ax.set_ylim(-0.02, 1.06)
    ax.tick_params(labelsize=8)
    ax.grid(True, linestyle="--", linewidth=0.45, alpha=0.45)
    ax.legend(fontsize=7.5, loc="upper center", ncol=5)

    fig.suptitle("Incremental FIS membership functions", fontsize=11.0, fontweight="bold", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    save_figure(fig, out_dir, "fig3c_increment_membership_ieee")


def plot_absolute_surface(abs_tool, out_dir, n=45, fixed_dr=0.03):
    """Fig. 3(b): faithful absolute FIS 3D surface from plot_fuzzy.py."""
    F = np.linspace(0.0, 2.0, n)
    T = np.linspace(0.0, 6.0, n)
    X, Y = np.meshgrid(F, T)
    Z = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Z[i, j] = compute_fuzzy(abs_tool, [float(X[i, j]), float(Y[i, j]), float(fixed_dr)], "lambda_based")

    fig = plt.figure(figsize=(6.2, 4.8))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(X, Y, Z, cmap="viridis", edgecolor="none", antialiased=True, alpha=0.95)
    ax.set_xlabel(r"$F_h$ (N)", fontsize=9, labelpad=7)
    ax.set_ylabel(r"$T_h$ (s)", fontsize=9, labelpad=7)
    ax.set_zlabel(r"$\lambda$", fontsize=9, labelpad=7)
    ax.set_title(r"Absolute FIS surface at $D_r=0.03$ m", fontsize=10, fontweight="bold")
    ax.tick_params(labelsize=7.5)
    ax.view_init(elev=30, azim=-135)
    cbar = fig.colorbar(surf, shrink=0.58, aspect=14, pad=0.08)
    cbar.ax.tick_params(labelsize=7.5)
    fig.tight_layout()
    save_figure(fig, out_dir, "fig3b_absolute_surface_ieee")


def plot_increment_surface(delta_tool, out_dir, n=45, fixed_ddr=0.0):
    """Fig. 3(d): faithful incremental FIS 3D surface from plot_delta_fuzzy.py."""
    DF = np.linspace(-3.0, 3.0, n)
    DT = np.linspace(-1.5, 1.5, n)
    X, Y = np.meshgrid(DF, DT)
    Z = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Z[i, j] = compute_fuzzy(delta_tool, [float(X[i, j]), float(Y[i, j]), float(fixed_ddr)], "delta_lambda_based")

    fig = plt.figure(figsize=(6.2, 4.8))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(X, Y, Z, cmap="viridis", edgecolor="none", antialiased=True, alpha=0.95)
    ax.set_xlabel(r"$\dot F_h$ (N/s)", fontsize=9, labelpad=7)
    ax.set_ylabel(r"$\dot T_h$", fontsize=9, labelpad=7)
    ax.set_zlabel(r"$\Delta\lambda$", fontsize=9, labelpad=7)
    ax.set_title(r"Incremental FIS surface at $\dot D_r=0$", fontsize=10, fontweight="bold")
    ax.tick_params(labelsize=7.5)
    ax.view_init(elev=25, azim=-135)
    cbar = fig.colorbar(surf, shrink=0.58, aspect=14, pad=0.08)
    cbar.ax.tick_params(labelsize=7.5)
    fig.tight_layout()
    save_figure(fig, out_dir, "fig3d_increment_surface_ieee")


def make_combined_preview(out_dir):
    """Create a small combined preview from the four generated PNG panels."""
    from PIL import Image, ImageDraw, ImageFont

    paths = [
        Path(out_dir) / "fig3a_absolute_membership_ieee.png",
        Path(out_dir) / "fig3b_absolute_surface_ieee.png",
        Path(out_dir) / "fig3c_increment_membership_ieee.png",
        Path(out_dir) / "fig3d_increment_surface_ieee.png",
    ]
    imgs = [Image.open(p).convert("RGB") for p in paths]
    # Resize to common panel size for preview only. Individual PDFs should be used in the paper.
    panel_w, panel_h = 1500, 1050
    imgs = [im.resize((panel_w, panel_h), Image.LANCZOS) for im in imgs]
    margin = 60
    label_h = 55
    canvas = Image.new("RGB", (2 * panel_w + 3 * margin, 2 * panel_h + 3 * margin + label_h), "white")
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("DejaVuSerif-Bold.ttf", 44)
    except Exception:
        font = None
    labels = ["(a)", "(b)", "(c)", "(d)"]
    positions = [
        (margin, margin + label_h),
        (2 * margin + panel_w, margin + label_h),
        (margin, 2 * margin + panel_h + label_h),
        (2 * margin + panel_w, 2 * margin + panel_h + label_h),
    ]
    for im, lab, pos in zip(imgs, labels, positions):
        draw.text((pos[0], pos[1]-50), lab, fill="black", font=font)
        canvas.paste(im, pos)
    canvas.save(Path(out_dir) / "fig3_combined_ieee.png", dpi=(600, 600))

    # Also save PDF preview.
    canvas.save(Path(out_dir) / "fig3_combined_ieee.pdf", resolution=600.0)


def main():
    parser = argparse.ArgumentParser(description="Regenerate Fig. 3 from fuzzy_logic.py in IEEE style.")
    parser.add_argument("--out", default="fig3_ieee_output", help="Output directory")
    parser.add_argument("--surface-n", type=int, default=45, help="Surface grid density")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    abs_tool = FuzzyLogicTool(type="lambda_based")
    delta_tool = FuzzyLogicTool(type="delta_lambda_based")

    plot_absolute_membership(abs_tool, out_dir)
    plot_absolute_surface(abs_tool, out_dir, n=args.surface_n, fixed_dr=0.03)
    plot_increment_membership(delta_tool, out_dir)
    plot_increment_surface(delta_tool, out_dir, n=args.surface_n, fixed_ddr=0.0)
    make_combined_preview(out_dir)

    print(f"Saved Fig. 3 panels to: {out_dir.resolve()}")
    print("Use the four individual PDF panels in the paper for maximum readability.")


if __name__ == "__main__":
    main()
