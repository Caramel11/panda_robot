#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate vector figures for Chapter 2 theory explanations."""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "figures" / "ch2"


def setup_style() -> None:
    candidates = [
        "Microsoft YaHei",
        "Microsoft YaHei UI",
        "微软雅黑",
        "WenQuanYi Micro Hei",
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans SC",
        "WenQuanYi Zen Hei",
        "SimHei",
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    for name in candidates:
        if name in available:
            mpl.rcParams["font.family"] = "sans-serif"
            mpl.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    mpl.rcParams.update(
        {
            "axes.unicode_minus": False,
            "font.serif": ["Latin Modern Roman", "STIX", "Tinos", "Times New Roman"],
            "mathtext.fontset": "cm",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "font.size": 10,
            "axes.linewidth": 0.9,
            "figure.dpi": 180,
        }
    )


def arrow(ax, start, end, color="#2f6fbb", lw=2.0, mutation_scale=14, **kwargs):
    patch = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=mutation_scale,
        linewidth=lw,
        color=color,
        shrinkA=0,
        shrinkB=0,
        **kwargs,
    )
    ax.add_patch(patch)
    return patch


def save(fig: plt.Figure, name: str) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "svg", "png"):
        fig.savefig(OUT_DIR / f"{name}.{ext}", bbox_inches="tight")
    plt.close(fig)


def add_box(ax, xy, width, height, text, fc="#f7f9fb", ec="#9aa7b2", fontsize=10):
    box = Rectangle(xy, width, height, facecolor=fc, edgecolor=ec, linewidth=1.1)
    ax.add_patch(box)
    ax.text(
        xy[0] + width / 2,
        xy[1] + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        linespacing=1.35,
    )
    return box


def plot_task_space_mapping() -> None:
    fig, ax = plt.subplots(figsize=(7.2, 3.8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5.4)
    ax.axis("off")

    p0 = np.array([0.9, 1.0])
    p1 = np.array([2.45, 2.35])
    p2 = np.array([4.05, 2.85])
    p_tip = np.array([4.65, 2.55])
    links = [(p0, p1), (p1, p2), (p2, p_tip)]

    for a, b in links:
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#58636f", lw=9, solid_capstyle="round")
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#75808c", lw=5, solid_capstyle="round")
    for p, label in [(p0, r"$q_1$"), (p1, r"$q_2$"), (p2, r"$q_3$")]:
        ax.add_patch(Circle(p, 0.17, facecolor="#f3f6f8", edgecolor="#53606c", lw=1.2))
        ax.text(p[0] - 0.28, p[1] - 0.48, label, fontsize=10)
    ax.add_patch(Circle(p_tip, 0.13, facecolor="#d95f5f", edgecolor="#8b2f2f", lw=1.0))

    arrow(ax, p_tip, p_tip + np.array([1.1, 0.35]), color="#2878b5")
    ax.text(p_tip[0] + 0.65, p_tip[1] + 0.47, r"$\dot{x}=J(q)\dot{q}$", color="#2878b5")
    arrow(ax, p_tip + np.array([0.15, 0.75]), p_tip + np.array([0.15, 0.12]), color="#d9534f")
    ax.text(p_tip[0] - 0.42, p_tip[1] + 0.78, r"$w=f_{\rm ext}$", color="#d9534f")
    ax.text(p_tip[0] - 0.18, p_tip[1] - 0.48, r"$x=\Phi(q)$")

    add_box(
        ax,
        (6.0, 3.35),
        3.1,
        0.95,
        "关节空间\n$q,\\dot q,\\tau$",
        fc="#eef5ff",
        ec="#7aa0c4",
    )
    add_box(
        ax,
        (6.0, 1.15),
        3.1,
        1.1,
        "任务空间\n$x,\\dot x,u_x,f_{\\rm ext}$",
        fc="#fff4ed",
        ec="#c9916a",
    )
    arrow(ax, (7.55, 3.35), (7.55, 2.30), color="#2878b5")
    ax.text(7.73, 2.75, r"$J(q)$", color="#2878b5")
    arrow(ax, (6.55, 1.15), (6.55, 3.30), color="#d9534f", mutation_scale=12)
    ax.text(5.58, 2.18, r"$\tau_{\rm ext}=J^T(q)w$", color="#d9534f")

    ax.text(
        0.3,
        4.78,
        "任务空间模型：运动映射与力映射",
        fontsize=13,
        weight="bold",
    )
    ax.text(
        0.3,
        4.35,
        "雅可比矩阵把关节速度映射到末端速度，雅可比转置把末端外力映射到关节广义力。",
        fontsize=10,
        color="#3b4148",
    )
    save(fig, "ch2_task_space_mapping")


def plot_contact_model() -> None:
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(7.4, 3.8), gridspec_kw={"width_ratios": [1.0, 1.1]})
    ax0.set_xlim(0, 5.3)
    ax0.set_ylim(0, 5.2)
    ax0.axis("off")

    ax0.add_patch(Rectangle((0.45, 0.45), 4.35, 0.8, facecolor="#f4d7dc", edgecolor="#ba6c79", lw=1.2))
    ax0.plot([0.45, 1.2, 2.0, 2.8, 3.6, 4.8], [1.25, 1.32, 1.18, 1.28, 1.20, 1.25], color="#ba6c79", lw=2.0)
    ax0.add_patch(Rectangle((2.35, 3.65), 0.65, 0.8, facecolor="#647585", edgecolor="#33404c"))
    ax0.add_patch(Polygon([[2.25, 3.65], [3.10, 3.65], [2.82, 2.65], [2.48, 2.65]], closed=True, facecolor="#4a5866", edgecolor="#33404c"))
    ax0.add_patch(Circle((2.65, 2.55), 0.22, facecolor="#f2a93b", edgecolor="#a46718", lw=1.1))

    ys = np.linspace(1.25, 2.45, 9)
    xs = 2.1 + 0.18 * np.sin(np.linspace(0, 4 * np.pi, len(ys)))
    ax0.plot(xs, ys, color="#3b78b8", lw=2.0)
    ax0.plot([3.15, 3.15], [1.25, 2.45], color="#6a7785", lw=3.0)
    ax0.plot([2.95, 3.35], [2.45, 2.45], color="#6a7785", lw=2.0)
    ax0.plot([2.95, 3.35], [1.25, 1.25], color="#6a7785", lw=2.0)
    arrow(ax0, (1.45, 3.15), (1.45, 1.35), color="#d9534f")
    ax0.text(1.05, 2.25, r"$\delta,\dot{\delta}$", color="#d9534f", rotation=90, va="center")
    arrow(ax0, (3.85, 1.45), (3.85, 2.55), color="#2878b5")
    ax0.text(4.03, 2.0, r"$f_e$", color="#2878b5", va="center")
    ax0.text(0.55, 4.75, "局部柔性接触等效模型", fontsize=12, weight="bold")
    ax0.text(0.85, 0.75, "柔性环境", color="#7d3e49")
    ax0.text(1.82, 1.9, "弹簧", color="#3b78b8")
    ax0.text(3.35, 1.9, "阻尼", color="#5d6570")

    delta = np.linspace(0.0, 4.0, 150)
    K = 1.15
    for v, color, label in [
        (0.0, "#4c78a8", r"$\dot{\delta}=0$"),
        (0.7, "#f58518", r"$\dot{\delta}>0$"),
        (-0.45, "#54a24b", r"$\dot{\delta}<0$"),
    ]:
        force = K * delta + 0.85 * v
        ax1.plot(delta, force, lw=2.1, color=color, label=label)
    ax1.set_xlabel(r"压入量 $\delta$")
    ax1.set_ylabel(r"接触力 $f_e$")
    ax1.set_title(r"$f_e=K_e\delta+B_e\dot{\delta}+\Delta_f$", fontsize=12)
    ax1.grid(True, alpha=0.25)
    ax1.legend(frameon=False, loc="upper left")
    ax1.spines[["top", "right"]].set_visible(False)
    save(fig, "ch2_kelvin_voigt_contact")


def plot_force_position_coordination() -> None:
    fig, ax = plt.subplots(figsize=(7.2, 3.9))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5.2)
    ax.axis("off")

    xs = np.linspace(0.7, 5.3, 250)
    ys = 1.15 + 0.28 * np.sin(1.4 * xs)
    ax.plot(xs, ys, color="#6aa56f", lw=2.2)
    ax.fill_between(xs, 0.45, ys, color="#d8ead8", alpha=0.75)
    pc = np.array([3.1, 1.15 + 0.28 * np.sin(1.4 * 3.1)])
    tangent = np.array([1.0, 0.28 * 1.4 * np.cos(1.4 * 3.1)])
    tangent = tangent / np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    ax.add_patch(Circle(pc, 0.18, facecolor="#f2a93b", edgecolor="#a46718", lw=1.1))
    ax.add_patch(Rectangle((pc[0] - 0.22, pc[1] + 0.18), 0.44, 0.86, angle=-25, facecolor="#4d5e6f", edgecolor="#303a44"))
    arrow(ax, pc + 0.1 * tangent, pc + 1.35 * tangent, color="#2878b5")
    arrow(ax, pc + 0.1 * normal, pc + 1.25 * normal, color="#d9534f")
    ax.text(*(pc + 1.45 * tangent), "切向位置跟踪", color="#2878b5", ha="center")
    ax.text(*(pc + 1.45 * normal), "法向力调节", color="#d9534f", ha="center")
    ax.text(0.65, 0.65, "柔性接触面", color="#3f7f46")

    add_box(
        ax,
        (5.95, 3.18),
        3.3,
        0.75,
        r"位置目标：$S_p e_x\to0$",
        fc="#eef5ff",
        ec="#7aa0c4",
    )
    add_box(
        ax,
        (5.95, 2.08),
        3.3,
        0.75,
        r"力目标：$S_f e_f\to0$",
        fc="#fff2f0",
        ec="#cf8179",
    )
    add_box(
        ax,
        (5.95, 0.88),
        3.3,
        0.86,
        r"安全边界：$f_{\min}\leq f_e\leq f_{\max}$",
        fc="#f8fbf1",
        ec="#9ab46a",
    )
    arrow(ax, (5.55, 2.50), (5.95, 3.43), color="#2878b5", mutation_scale=10)
    arrow(ax, (5.55, 2.20), (5.95, 2.41), color="#d9534f", mutation_scale=10)
    arrow(ax, (5.55, 1.90), (5.95, 1.31), color="#6aa56f", mutation_scale=10)
    ax.text(0.45, 4.75, "力位协同：同一接触任务中的方向分解与目标分配", fontsize=12.5, weight="bold")
    ax.text(0.55, 4.32, "切向误差影响任务覆盖，法向误差影响接触可靠性和材料安全。", color="#3b4148")
    save(fig, "ch2_force_position_coordination")


def plot_pareto_arbitration() -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.set_xlim(0.35, 4.35)
    ax.set_ylim(0.35, 4.35)
    ax.set_xlabel(r"性能指标 $J_1$")
    ax.set_ylabel(r"性能指标 $J_2$")
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(True, alpha=0.22)

    def pareto_y(xv):
        return 0.52 + 2.85 / (xv + 0.18)

    def tangent_point(eta):
        # The tangent of J_2=a+b/(J_1+c) has slope -eta/(1-eta).
        return np.sqrt(2.85 * (1.0 - eta) / eta) - 0.18

    x = np.linspace(0.65, 3.75, 250)
    y = pareto_y(x)
    ax.fill_between(x, y, 4.35, color="#d9e7f5", alpha=0.72)
    ax.plot(x, y, color="#1f77b4", lw=2.6)
    ax.text(
        3.08,
        4.02,
        "可行性能集合",
        color="#496f95",
        ha="center",
        va="center",
        bbox=dict(boxstyle="round,pad=0.24", facecolor="#edf4fb", edgecolor="none", alpha=0.86),
    )
    ax.annotate(
        "帕累托边界",
        xy=(2.22, pareto_y(2.22)),
        xytext=(2.52, 1.92),
        color="#1f77b4",
        arrowprops=dict(arrowstyle="-|>", color="#1f77b4", lw=1.2),
    )
    ax.scatter([3.2], [3.2], s=60, color="#9aa0a6", zorder=4)
    ax.annotate(
        "被支配点",
        xy=(3.2, 3.2),
        xytext=(3.36, 3.42),
        color="#6b7075",
        ha="left",
        arrowprops=dict(arrowstyle="-|>", color="#6b7075", lw=1.1),
    )
    arrow(ax, (3.12, 3.03), (2.36, 2.02), color="#6b7075", lw=1.4, mutation_scale=11)

    etas = [0.2, 0.5, 0.8]
    colors = ["#54a24b", "#f58518", "#d9534f"]
    label_offsets = [(0.08, 0.20), (0.10, 0.22), (0.17, -0.18)]
    tangent_half_width = [0.86, 0.62, 0.34]
    for eta, color, offset, half_width in zip(etas, colors, label_offsets, tangent_half_width):
        xp = tangent_point(eta)
        yp = pareto_y(xp)
        ax.scatter([xp], [yp], s=70, color=color, edgecolor="white", linewidth=1.0, zorder=5)
        slope = -eta / (1 - eta)
        xx = np.linspace(max(0.42, xp - half_width), min(4.05, xp + half_width), 20)
        yy = yp + slope * (xx - xp)
        ax.plot(xx, yy, "--", color=color, lw=1.2, alpha=0.8)
        ax.text(xp + offset[0], yp + offset[1], rf"$\eta={eta:.1f}$", color=color)

    arrow(ax, (3.25, 0.62), (0.88, 0.62), color="#30343b", lw=1.4, mutation_scale=12)
    ax.text(2.05, 0.43, r"$\eta$ 增大：$J_1$ 权重增大", ha="center", color="#30343b")
    ax.text(3.28, 0.84, r"$J_2$ 优先", color="#54a24b", ha="center")
    ax.text(0.86, 0.84, r"$J_1$ 优先", color="#d9534f", ha="center")

    save(fig, "ch2_pareto_arbitration")


def main() -> None:
    setup_style()
    plot_task_space_mapping()
    plot_contact_model()
    plot_force_position_coordination()
    plot_pareto_arbitration()
    print(f"Generated Chapter 2 theory figures in {OUT_DIR}")


if __name__ == "__main__":
    main()
