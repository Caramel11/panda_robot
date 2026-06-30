#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate Chapter 3 force-margin fuzzy arbitration figures.

The figures are based on the 0603 continuous_force_margin implementation:
  - ContinuousForceMarginFuzzyAlphaScheduler
  - force-margin inputs F_h, S_h, D_r
  - Mamdani min-max inference and centroid defuzzification
  - directional safety correction with k_upper / k_lower
  - safe-tracking boost used by the continuous force-margin strategy

Outputs:
  figures/ch3/ch3_force_margin_membership_ieee.pdf/png
  figures/ch3/ch3_force_margin_surface_ieee.pdf/png
  figures/ch3/ch3_force_margin_trends_ieee.pdf/png
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from ieee_style import DBL_W, IEEE_COLORS, apply_ieee_style, save_fig


THESIS_DIR = Path(__file__).resolve().parents[2]
TESTS_DIR = THESIS_DIR.parent
SCHED_ROOT = TESTS_DIR / "0603"
SCHED_SRC = SCHED_ROOT / "src"

sys.path.insert(0, str(SCHED_ROOT))
sys.path.insert(0, str(SCHED_SRC))

from alpha_scheduler_gt import ContinuousForceMarginFuzzyAlphaScheduler  # noqa: E402


OUT_DIR = THESIS_DIR / "figures" / "ch3"


def apply_chinese_style() -> None:
    """Use a local CJK font for Chinese labels while keeping mathtext stable."""
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": [
            "WenQuanYi Micro Hei",
            "Noto Sans CJK SC",
            "Noto Serif CJK SC",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
    })


def left_shoulder(x: np.ndarray, params: tuple[float, float, float]) -> np.ndarray:
    a, b, c = params
    y = np.zeros_like(x, dtype=float)
    y[(x >= a) & (x <= b)] = 1.0
    mask = (x > b) & (x < c)
    y[mask] = (c - x[mask]) / max(c - b, 1e-12)
    return y


def right_shoulder(x: np.ndarray, params: tuple[float, float, float]) -> np.ndarray:
    a, b, c = params
    y = np.zeros_like(x, dtype=float)
    mask = (x > a) & (x < b)
    y[mask] = (x[mask] - a) / max(b - a, 1e-12)
    y[(x >= b) & (x <= c)] = 1.0
    return y


def triangular(x: np.ndarray, params: tuple[float, float, float]) -> np.ndarray:
    a, b, c = params
    return np.maximum(
        0.0,
        np.minimum((x - a) / max(b - a, 1e-12), (c - x) / max(c - b, 1e-12)),
    )


def membership_curve(
    x: np.ndarray,
    label: str,
    params: tuple[float, float, float],
) -> np.ndarray:
    if label in ("PS", "Z"):
        return left_shoulder(x, params)
    if label == "PL":
        return right_shoulder(x, params)
    return triangular(x, params)


def fuzzy_alpha0(
    scheduler: ContinuousForceMarginFuzzyAlphaScheduler,
    F_h: float,
    S_h: float,
    D_r: float,
) -> float:
    fuzzified = scheduler._fuzzify(F_h, S_h, D_r)
    inferred = scheduler._infer(fuzzified)
    return scheduler._defuzzify_alpha(inferred)


def alpha_force_margin_only(
    F_norm: float,
    e_r: float,
    *,
    k_upper: float = 0.45,
    k_lower: float = 0.25,
) -> float:
    scheduler = ContinuousForceMarginFuzzyAlphaScheduler(
        k_upper=k_upper,
        k_lower=k_lower,
    )
    F_h, S_h, D_r, rho_F, s_F, _F_err = scheduler._force_margin_inputs(
        F_norm,
        scheduler.F_desired,
        scheduler.F_min,
        scheduler.F_max,
        e_r,
    )
    alpha0 = fuzzy_alpha0(scheduler, F_h, S_h, D_r)
    r_F = 1.0 - rho_F
    alpha = (
        alpha0
        + scheduler.k_upper * r_F * max(s_F, 0.0)
        - scheduler.k_lower * r_F * max(-s_F, 0.0)
    )
    return float(np.clip(alpha, scheduler.alpha_min, scheduler.alpha_max))


def static_continuous_force_margin_alpha(
    scheduler: ContinuousForceMarginFuzzyAlphaScheduler,
    F_norm: float,
    e_r: float,
    *,
    K_hat: float | None = None,
    tracking_boost_enabled: bool = True,
) -> float:
    F_h, S_h, D_r, rho_F, s_F, F_err = scheduler._force_margin_inputs(
        F_norm,
        scheduler.F_desired,
        scheduler.F_min,
        scheduler.F_max,
        e_r,
    )
    alpha0 = fuzzy_alpha0(scheduler, F_h, S_h, D_r)
    r_F = 1.0 - rho_F
    alpha_safe = (
        alpha0
        + scheduler.k_upper * r_F * max(s_F, 0.0)
        - scheduler.k_lower * r_F * max(-s_F, 0.0)
    )
    if F_norm >= scheduler.F_max or F_norm <= scheduler.F_min:
        alpha_safe = scheduler._force_risk_rebalance(
            alpha_safe,
            rho_F=rho_F,
            F_err=F_err,
        )
    elif tracking_boost_enabled:
        alpha_safe = scheduler._safe_tracking_boost(
            alpha_safe,
            rho_F=rho_F,
            F_err=F_err,
            e_r=e_r,
        )
        alpha_safe = scheduler._force_risk_rebalance(
            alpha_safe,
            rho_F=rho_F,
            F_err=F_err,
        )
    alpha_safe = scheduler._stiffness_rebalance(alpha_safe, K_hat)
    return float(np.clip(alpha_safe, scheduler.alpha_min, scheduler.alpha_max))


def plot_membership() -> None:
    scheduler = ContinuousForceMarginFuzzyAlphaScheduler()
    domains = [
        np.linspace(0.0, 2.0, 600),
        np.linspace(0.0, 6.0, 600),
        np.linspace(0.0, 0.10, 600),
        np.linspace(0.0, 1.0, 600),
    ]
    sets = [
        scheduler.input_sets[0],
        scheduler.input_sets[1],
        scheduler.input_sets[2],
        scheduler.output_sets,
    ]
    titles = [
        r"(a) $F_h$",
        r"(b) $S_h$",
        r"(c) $D_p$",
        r"(d) $\alpha_0$",
    ]
    xlabels = [
        r"$F_h=2|F-F_d|/(F_{\max}-F_{\min})$",
        r"$S_h=6\rho_F$",
        r"$D_p=0.1-|e_p|\,0.1/0.005$",
        r"$\alpha_0$",
    ]

    fig, axes = plt.subplots(2, 2, figsize=(DBL_W, 4.75))
    axes = axes.ravel()
    for ax, x, mf_sets, title, xlabel in zip(axes, domains, sets, titles, xlabels):
        for idx, (label, params) in enumerate(mf_sets.items()):
            label_names = {
                "Z": "零(Z)",
                "PS": "小(PS)",
                "PM": "中(PM)",
                "P": "较大(P)",
                "PL": "大(PL)",
            }
            ax.plot(
                x,
                membership_curve(x, label, params),
                color=IEEE_COLORS[idx % len(IEEE_COLORS)],
                lw=1.45,
                label=label_names.get(label, label),
            )
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("隶属度")
        ax.set_ylim(-0.03, 1.06)
        ax.legend(loc="upper right", ncol=min(len(mf_sets), 3))
    fig.tight_layout()
    save_fig(fig, OUT_DIR / "ch3_force_margin_membership_ieee.pdf", formats=("pdf", "png"))
    plt.close(fig)


def plot_response() -> None:
    scheduler = ContinuousForceMarginFuzzyAlphaScheduler()

    force = np.linspace(0.0, 1.25, 260)
    fig = plt.figure(figsize=(DBL_W, 4.55))
    gs = fig.add_gridspec(
        2,
        2,
        height_ratios=(1.08, 1.0),
        hspace=0.36,
        wspace=0.25,
    )

    ax = fig.add_subplot(gs[0, :])
    e_cases = [
        (0.0, r"$|e_p|=0$ mm"),
        (0.0015, r"$|e_p|=1.5$ mm"),
        (0.0035, r"$|e_p|=3.5$ mm"),
        (0.0050, r"$|e_p|=5.0$ mm"),
    ]
    for idx, (e_r, label) in enumerate(e_cases):
        sched_case = ContinuousForceMarginFuzzyAlphaScheduler()
        alpha = [
            static_continuous_force_margin_alpha(sched_case, float(f), e_r)
            for f in force
        ]
        ax.plot(force, alpha, color=IEEE_COLORS[idx], lw=1.35, label=label)
    ax.axvspan(scheduler.F_min, scheduler.F_max, color="#B0B0B0", alpha=0.16, lw=0)
    ax.axvline(scheduler.F_desired, color="k", lw=0.8, ls=":", label=r"$F_d$")
    ax.set_title(r"(a) $|e_p|$")
    ax.set_xlabel(r"测量接触力 $F$ (N)")
    ax.set_ylabel(r"力位优先级 $\alpha_{\mathrm{FP}}$")
    ax.set_ylim(0.0, 1.0)
    ax.legend(loc="lower right", ncol=2)

    force_lower = np.linspace(0.16, scheduler.F_desired, 220)
    alpha_default_lower = np.array([
        alpha_force_margin_only(float(f), 0.0015)
        for f in force_lower
    ])
    alpha_k_lower = np.array([
        alpha_force_margin_only(float(f), 0.0015, k_lower=0.55)
        for f in force_lower
    ])

    ax = fig.add_subplot(gs[1, 0])
    ax.axvspan(scheduler.F_min, scheduler.F_max, color="#B0B0B0", alpha=0.16, lw=0)
    ax.axvline(scheduler.F_desired, color="k", lw=0.8, ls=":", label=r"$F_d$")
    ax.fill_between(
        force_lower,
        alpha_default_lower,
        alpha_k_lower,
        color=IEEE_COLORS[2],
        alpha=0.10,
        lw=0,
    )
    ax.plot(
        force_lower,
        alpha_default_lower,
        color=IEEE_COLORS[0],
        ls="--",
        lw=1.75,
        label="默认参数",
        zorder=4,
    )
    ax.plot(
        force_lower,
        alpha_k_lower,
        color=IEEE_COLORS[2],
        lw=1.45,
        label=r"增大 $k_{\mathrm{lower}}$",
        zorder=3,
    )
    ax.set_title(r"(b) $k_{\mathrm{lower}}$")
    ax.set_xlabel(r"测量接触力 $F$ (N)")
    ax.set_ylabel(r"修正优先级 $\alpha_{\mathrm{FM}}$")
    ax.set_xlim(0.16, 0.52)
    ax.set_ylim(0.0, 0.72)
    ax.legend(loc="upper left", ncol=1)

    force_upper = np.linspace(scheduler.F_desired, 1.25, 240)
    alpha_default_upper = np.array([
        alpha_force_margin_only(float(f), 0.0015)
        for f in force_upper
    ])
    alpha_k_upper = np.array([
        alpha_force_margin_only(float(f), 0.0015, k_upper=0.80)
        for f in force_upper
    ])

    ax = fig.add_subplot(gs[1, 1])
    ax.axvspan(scheduler.F_min, scheduler.F_max, color="#B0B0B0", alpha=0.16, lw=0)
    ax.axvline(scheduler.F_desired, color="k", lw=0.8, ls=":", label=r"$F_d$")
    ax.fill_between(
        force_upper,
        alpha_default_upper,
        alpha_k_upper,
        color=IEEE_COLORS[1],
        alpha=0.10,
        lw=0,
    )
    ax.plot(
        force_upper,
        alpha_default_upper,
        color=IEEE_COLORS[0],
        ls="--",
        lw=1.75,
        label="默认参数",
        zorder=4,
    )
    ax.plot(
        force_upper,
        alpha_k_upper,
        color=IEEE_COLORS[1],
        lw=1.45,
        label=r"增大 $k_{\mathrm{upper}}$",
        zorder=3,
    )
    ax.set_title(r"(c) $k_{\mathrm{upper}}$")
    ax.set_xlabel(r"测量接触力 $F$ (N)")
    ax.set_ylabel(r"修正优先级 $\alpha_{\mathrm{FM}}$")
    ax.set_xlim(0.48, 1.25)
    ax.set_ylim(0.0, 1.0)
    ax.legend(loc="lower right", ncol=1)

    save_fig(fig, OUT_DIR / "ch3_force_margin_trends_ieee.pdf", formats=("pdf", "png"))
    plt.close(fig)


def plot_surface() -> None:
    scheduler = ContinuousForceMarginFuzzyAlphaScheduler()
    n = 45
    F_grid = np.linspace(0.0, 2.0, n)
    D_grid = np.linspace(0.0, 0.10, n)
    F_mesh, D_mesh = np.meshgrid(F_grid, D_grid)
    alpha = np.zeros_like(F_mesh)
    for i in range(F_mesh.shape[0]):
        for j in range(F_mesh.shape[1]):
            alpha[i, j] = fuzzy_alpha0(
                scheduler,
                float(F_mesh[i, j]),
                6.0,
                float(D_mesh[i, j]),
            )

    fig = plt.figure(figsize=(6.2, 4.8))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(
        F_mesh,
        D_mesh,
        alpha,
        cmap="viridis",
        edgecolor="none",
        antialiased=True,
        alpha=0.95,
    )
    ax.set_xlabel(r"力误差输入 $F_h$", fontsize=9, labelpad=7)
    ax.set_ylabel(r"位置裕度输入 $D_p$", fontsize=9, labelpad=7)
    ax.set_zlabel(r"基础优先级 $\alpha_0$", fontsize=9, labelpad=7)
    ax.set_title(r"$S_h=6$", fontsize=10, fontweight="bold")
    ax.tick_params(labelsize=7.5)
    ax.view_init(elev=30, azim=-135)
    cbar = fig.colorbar(surf, shrink=0.58, aspect=14, pad=0.08)
    cbar.set_label(r"基础优先级 $\alpha_0$", fontsize=8.5)
    cbar.ax.tick_params(labelsize=7.5)
    fig.tight_layout()
    save_fig(fig, OUT_DIR / "ch3_force_margin_surface_ieee.pdf", formats=("pdf", "png"))
    plt.close(fig)


def main() -> None:
    apply_ieee_style()
    apply_chinese_style()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plot_membership()
    plot_surface()
    plot_response()
    print(f"Saved Chapter 3 fuzzy arbitration figures to: {OUT_DIR}")


if __name__ == "__main__":
    main()
