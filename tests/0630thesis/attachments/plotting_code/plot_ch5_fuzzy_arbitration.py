#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate Chapter 5 fuzzy-arbitration figures.

The script follows the plotting style of plot_fig3_ieee.py while building
figures tailored to the integrated Chapter 5 model:
  1) reference-layer human-reference arbitration, reusing fuzzy_logic.py;
  2) execution-layer force-position priority arbitration, matching Ch. 3.

Outputs:
  figures/ch5/fuzzy_logic/ch5_fuzzy_membership_ieee.pdf/png
  figures/ch5/fuzzy_logic/ch5_fuzzy_response_ieee.pdf/png
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

from fuzzy_logic import FuzzyLogicTool
from ieee_style import DBL_W, IEEE_COLORS, apply_ieee_style, save_fig


THESIS_DIR = Path(__file__).resolve().parents[2]
OUT_DIR = THESIS_DIR / "figures" / "ch5" / "fuzzy_logic"


def apply_chinese_style() -> None:
    """Use a CJK font for Chinese labels while keeping STIX math text."""
    candidates = [
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
        "Droid Sans Fallback",
        "SimHei",
        "Microsoft YaHei",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    selected = next((name for name in candidates if name in available), candidates[0])
    matplotlib.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [selected, *candidates],
            "axes.unicode_minus": False,
            "axes.titlesize": 8.2,
            "axes.labelsize": 8.0,
            "xtick.labelsize": 7.2,
            "ytick.labelsize": 7.2,
            "legend.fontsize": 6.2,
        }
    )


INPUT_LABEL_CN = {
    "N": "负",
    "Z": "零",
    "P": "正",
    "PS": "小",
    "PM": "中",
    "PL": "大",
}

OUTPUT_LABEL_CN = {
    "NL": "负大",
    "N": "负",
    "Z": "零",
    "PS": "小",
    "PM": "中",
    "P": "较大",
    "PL": "大",
}


def display_label(label: str, *, output: bool = False) -> str:
    label_map = OUTPUT_LABEL_CN if output else INPUT_LABEL_CN
    return f"{label_map.get(label, label)}({label})"


def triangular(x: np.ndarray, params: tuple[float, float, float]) -> np.ndarray:
    a, b, c = params
    return np.maximum(
        0.0,
        np.minimum((x - a) / max(b - a, 1e-12), (c - x) / max(c - b, 1e-12)),
    )


def left_shoulder(x: np.ndarray, params: tuple[float, float, float]) -> np.ndarray:
    a, b, c = params
    y = np.zeros_like(x, dtype=float)
    y[(x >= a) & (x <= b)] = 1.0
    mask = (x > b) & (x <= c)
    y[mask] = (c - x[mask]) / max(c - b, 1e-12)
    return y


def right_shoulder(x: np.ndarray, params: tuple[float, float, float]) -> np.ndarray:
    a, b, c = params
    y = np.zeros_like(x, dtype=float)
    mask = (x >= a) & (x <= b)
    y[mask] = (x[mask] - a) / max(b - a, 1e-12)
    y[(x > b) & (x <= c)] = 1.0
    return y


def membership_curve(
    x: np.ndarray,
    label: str,
    params: tuple[float, float, float],
    *,
    output: bool = False,
) -> np.ndarray:
    if output:
        if label in ("Z", "NL"):
            return left_shoulder(x, params)
        if label in ("PL",):
            return right_shoulder(x, params)
        return triangular(x, params)
    if label in ("PS", "N", "Z"):
        return left_shoulder(x, params)
    if label in ("PL", "P"):
        return right_shoulder(x, params)
    return triangular(x, params)


def scalar_membership(value: float, label: str, params: tuple[float, float, float], *, output: bool = False) -> float:
    x = np.array([value], dtype=float)
    return float(membership_curve(x, label, params, output=output)[0])


FP_INPUT_SETS = [
    {"PS": (0.0, 0.4, 0.8), "PM": (0.4, 1.0, 1.5), "PL": (1.2, 1.5, 2.0)},
    {"PS": (0.0, 1.0, 2.0), "PM": (1.0, 3.0, 5.0), "PL": (3.0, 5.0, 6.0)},
    {"PS": (0.0, 1.0, 2.0), "PM": (1.0, 3.0, 5.0), "PL": (3.0, 5.0, 6.0)},
]

FP_OUTPUT_SETS = {
    "Z": (0.0, 0.05, 0.1),
    "PS": (0.05, 0.25, 0.5),
    "PM": (0.25, 0.5, 0.7),
    "P": (0.5, 0.7, 0.95),
    "PL": (0.9, 0.95, 1.0),
}

FP_RULES = {
    ("PS", "PS", "PS"): "PL",
    ("PS", "PS", "PM"): "P",
    ("PS", "PS", "PL"): "PM",
    ("PS", "PM", "PS"): "PL",
    ("PS", "PM", "PM"): "P",
    ("PS", "PM", "PL"): "PM",
    ("PS", "PL", "PS"): "P",
    ("PS", "PL", "PM"): "PM",
    ("PS", "PL", "PL"): "PM",
    ("PM", "PS", "PS"): "PL",
    ("PM", "PS", "PM"): "P",
    ("PM", "PS", "PL"): "PS",
    ("PM", "PM", "PS"): "P",
    ("PM", "PM", "PM"): "PM",
    ("PM", "PM", "PL"): "PS",
    ("PM", "PL", "PS"): "P",
    ("PM", "PL", "PM"): "PM",
    ("PM", "PL", "PL"): "PS",
    ("PL", "PS", "PS"): "PL",
    ("PL", "PS", "PM"): "PM",
    ("PL", "PS", "PL"): "Z",
    ("PL", "PM", "PS"): "P",
    ("PL", "PM", "PM"): "PS",
    ("PL", "PM", "PL"): "Z",
    ("PL", "PL", "PS"): "P",
    ("PL", "PL", "PM"): "PS",
    ("PL", "PL", "PL"): "Z",
}


def fp_alpha0(force_error_input: float, safety_margin_input: float, position_margin_input: float) -> float:
    values = [
        float(np.clip(force_error_input, 0.0, 2.0)),
        float(np.clip(safety_margin_input, 0.0, 6.0)),
        float(np.clip(position_margin_input, 0.0, 6.0)),
    ]
    memberships = []
    for value, sets in zip(values, FP_INPUT_SETS):
        memberships.append({
            label: scalar_membership(value, label, params)
            for label, params in sets.items()
        })

    output_activation: dict[str, float] = {}
    for labels, out_label in FP_RULES.items():
        activation = min(memberships[i][labels[i]] for i in range(3))
        output_activation[out_label] = max(output_activation.get(out_label, 0.0), activation)

    z_grid = np.linspace(0.0, 1.0, 401)
    mu_total = np.zeros_like(z_grid)
    for out_label, activation in output_activation.items():
        out_curve = membership_curve(z_grid, out_label, FP_OUTPUT_SETS[out_label], output=True)
        mu_total = np.maximum(mu_total, np.minimum(activation, out_curve))
    denominator = float(np.sum(mu_total))
    if denominator < 1e-9:
        return 0.5
    return float(np.sum(z_grid * mu_total) / denominator)


def fp_directional_target(alpha0: np.ndarray, rho_f: np.ndarray, s_f: np.ndarray) -> np.ndarray:
    k_upper = 0.45
    k_lower = 0.25
    risk = 1.0 - rho_f
    corrected = alpha0 + k_upper * risk * np.maximum(s_f, 0.0) - k_lower * risk * np.maximum(-s_f, 0.0)
    return np.clip(corrected, 0.05, 0.95)


def compute_hr_abs(abs_tool: FuzzyLogicTool, x: float, y: float, d: float) -> float:
    return float(abs_tool.compute([x, y, d], types="lambda_based"))


def compute_hr_inc(delta_tool: FuzzyLogicTool, dx: float, dy: float, dd: float) -> float:
    return float(delta_tool.compute([dx, dy, dd], types="delta_lambda_based"))


def plot_membership() -> None:
    abs_tool = FuzzyLogicTool(type="lambda_based")
    fig, axes = plt.subplots(2, 4, figsize=(DBL_W, 4.95))

    hr_domains = [
        np.linspace(0.0, 2.0, 500),
        np.linspace(0.0, 6.0, 500),
        np.linspace(0.0, 0.1, 500),
        np.linspace(0.0, 1.0, 500),
    ]
    hr_sets = [*abs_tool.input_sets, abs_tool.output_sets]
    hr_titles = [
        r"(a) 参考层干预强度 $F_H^{\mathrm{fis}}$",
        r"(b) 参考层持续时间 $T_H^{\mathrm{fis}}$",
        r"(c) 综合风险特征 $D_c^{\mathrm{fis}}$",
        r"(d) 人侧权重输出 $\lambda_{\mathrm{abs}}$",
    ]

    fp_domains = [
        np.linspace(0.0, 2.0, 500),
        np.linspace(0.0, 6.0, 500),
        np.linspace(0.0, 6.0, 500),
        np.linspace(0.0, 1.0, 500),
    ]
    fp_sets = [*FP_INPUT_SETS, FP_OUTPUT_SETS]
    fp_titles = [
        r"(e) 力误差输入 $E_F^{\mathrm{fis}}$",
        r"(f) 力安全裕度 $S_F^{\mathrm{fis}}$",
        r"(g) 切向位置裕度 $D_T^{\mathrm{fis}}$",
        r"(h) 力位优先级 $\alpha_0$",
    ]

    for row, domains, sets_list, titles in [
        (0, hr_domains, hr_sets, hr_titles),
        (1, fp_domains, fp_sets, fp_titles),
    ]:
        for col, (domain, mf_sets, title) in enumerate(zip(domains, sets_list, titles)):
            ax = axes[row, col]
            for idx, (label, params) in enumerate(mf_sets.items()):
                is_output = col == 3
                ax.plot(
                    domain,
                    membership_curve(domain, label, params, output=is_output),
                    color=IEEE_COLORS[idx % len(IEEE_COLORS)],
                    lw=1.15,
                    label=display_label(label, output=is_output),
                )
            ax.set_title(title)
            ax.set_ylim(-0.03, 1.06)
            ax.set_xlabel("变量取值")
            ax.set_ylabel("隶属度")
            ax.legend(loc="upper right", ncol=1, fontsize=5.8)

    fig.tight_layout()
    save_fig(fig, OUT_DIR / "ch5_fuzzy_membership_ieee.pdf", formats=("pdf", "png"))
    plt.close(fig)


def plot_response() -> None:
    abs_tool = FuzzyLogicTool(type="lambda_based")
    delta_tool = FuzzyLogicTool(type="delta_lambda_based")

    fig, axes = plt.subplots(2, 2, figsize=(DBL_W, 5.28))
    levels01 = np.linspace(0.0, 1.0, 21)

    f_grid = np.linspace(0.0, 2.0, 90)
    t_grid = np.linspace(0.0, 6.0, 90)
    f_mesh, t_mesh = np.meshgrid(f_grid, t_grid)
    lambda_abs = np.zeros_like(f_mesh)
    for i in range(f_mesh.shape[0]):
        for j in range(f_mesh.shape[1]):
            lambda_abs[i, j] = compute_hr_abs(abs_tool, float(f_mesh[i, j]), float(t_mesh[i, j]), 0.05)

    ax = axes[0, 0]
    cf = ax.contourf(f_mesh, t_mesh, lambda_abs, levels=levels01, cmap="viridis")
    ax.contour(f_mesh, t_mesh, lambda_abs, levels=np.linspace(0.1, 0.9, 5), colors="k", linewidths=0.35)
    ax.set_title(r"(a) 参考层绝对权重 $\lambda_{\mathrm{abs}}$")
    ax.set_xlabel(r"干预强度 $F_H^{\mathrm{fis}}$")
    ax.set_ylabel(r"持续时间 $T_H^{\mathrm{fis}}$")
    fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.02, label=r"人侧权重 $\lambda_{\mathrm{abs}}$")

    df_grid = np.linspace(-3.0, 3.0, 90)
    dt_grid = np.linspace(0.0, 2.0, 90)
    df_mesh, dt_mesh = np.meshgrid(df_grid, dt_grid)
    lambda_inc = np.zeros_like(df_mesh)
    for i in range(df_mesh.shape[0]):
        for j in range(df_mesh.shape[1]):
            lambda_inc[i, j] = compute_hr_inc(delta_tool, float(df_mesh[i, j]), float(dt_mesh[i, j]), 0.0)

    ax = axes[0, 1]
    cf = ax.contourf(df_mesh, dt_mesh, lambda_inc, levels=np.linspace(-0.5, 0.5, 21), cmap="coolwarm")
    ax.contour(df_mesh, dt_mesh, lambda_inc, levels=np.linspace(-0.4, 0.4, 5), colors="k", linewidths=0.35)
    ax.set_title(r"(b) 参考层增量修正 $\Delta\lambda_{\mathrm{inc}}$")
    ax.set_xlabel(r"干预强度变化率 $\dot F_H^{\mathrm{fis}}$")
    ax.set_ylabel(r"持续时间变化率 $\dot T_H^{\mathrm{fis}}$")
    fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.02, label=r"权重增量 $\Delta\lambda_{\mathrm{inc}}$")

    ef_grid = np.linspace(0.0, 2.0, 90)
    dtp_grid = np.linspace(0.0, 6.0, 90)
    ef_mesh, dtp_mesh = np.meshgrid(ef_grid, dtp_grid)
    alpha0 = np.zeros_like(ef_mesh)
    for i in range(ef_mesh.shape[0]):
        for j in range(ef_mesh.shape[1]):
            alpha0[i, j] = fp_alpha0(float(ef_mesh[i, j]), 4.5, float(dtp_mesh[i, j]))

    ax = axes[1, 0]
    cf = ax.contourf(ef_mesh, dtp_mesh, alpha0, levels=levels01, cmap="viridis")
    ax.contour(ef_mesh, dtp_mesh, alpha0, levels=np.linspace(0.1, 0.9, 5), colors="k", linewidths=0.35)
    ax.set_title(r"(c) 执行层基础优先级 $\alpha_0$")
    ax.set_xlabel(r"力误差输入 $E_F^{\mathrm{fis}}$")
    ax.set_ylabel(r"切向位置裕度 $D_T^{\mathrm{fis}}$")
    fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.02, label=r"基础优先级 $\alpha_0$")

    sf_grid = np.linspace(-1.0, 1.0, 100)
    rho_grid = np.linspace(0.0, 1.0, 100)
    sf_mesh, rho_mesh = np.meshgrid(sf_grid, rho_grid)
    alpha_fp = fp_directional_target(0.5 * np.ones_like(sf_mesh), rho_mesh, sf_mesh)

    ax = axes[1, 1]
    cf = ax.contourf(sf_mesh, rho_mesh, alpha_fp, levels=levels01, cmap="viridis")
    ax.contour(sf_mesh, rho_mesh, alpha_fp, levels=np.linspace(0.1, 0.9, 5), colors="k", linewidths=0.35)
    ax.set_title(r"(d) 执行层方向修正 $\alpha_{\mathrm{FP},r}$")
    ax.set_xlabel(r"边界方向 $s_F$")
    ax.set_ylabel(r"力安全裕度 $\rho_F$")
    ax.text(-0.78, 0.09, "接触不足", color="white", fontsize=7.0, ha="center")
    ax.text(0.62, 0.09, "接近上界", color="white", fontsize=7.0, ha="center")
    fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.02, label=r"目标优先级 $\alpha_{\mathrm{FP},r}$")

    fig.tight_layout()
    save_fig(fig, OUT_DIR / "ch5_fuzzy_response_ieee.pdf", formats=("pdf", "png"))
    plt.close(fig)


def main() -> None:
    apply_ieee_style()
    apply_chinese_style()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plot_membership()
    plot_response()
    print(f"Saved Chapter 5 fuzzy arbitration figures to: {OUT_DIR}")


if __name__ == "__main__":
    main()
