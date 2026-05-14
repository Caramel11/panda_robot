import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# 自动处理依赖环境
if not os.path.exists("kalman_filter.py"):
    with open("kalman_filter.py", "w") as f:
        f.write(
            "class KalmanFilterFusion: def __init__(self, **k): pass\n    def update(self, l, t): return l\n"
            "class DeltaLambdaUpdater: def __init__(self, **k): pass"
        )

try:
    from fuzzy_logic import FuzzyLogicTool
except ImportError:
    print("错误：请确保 fuzzy_logic.py 在当前目录下。")
    sys.exit()


class FuzzyAcademicVisualizer:
    def __init__(self):
        # 初始化工具
        self.flt = FuzzyLogicTool(type="lambda_based")

        # 学术图表配置
        sns.set_theme(style="ticks")
        plt.rcParams["font.sans-serif"] = [
            "DejaVu Sans",
            "WenQuanYi Micro Hei",
            "sans-serif",
        ]
        plt.rcParams["axes.unicode_minus"] = False
        plt.rcParams["xtick.direction"] = "in"
        plt.rcParams["ytick.direction"] = "in"
        self.colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]

    def _triangular_mf(self, x, p):
        return np.maximum(
            0,
            np.minimum(
                (x - p[0]) / (p[1] - p[0] + 1e-9), (p[2] - x) / (p[2] - p[1] + 1e-9)
            ),
        )

    def _trap_left(self, x, p):
        res = np.zeros_like(x)
        res[(x >= p[0]) & (x <= p[1])] = 1.0
        mask = (x > p[1]) & (x <= p[2])
        res[mask] = (p[2] - x[mask]) / (p[2] - p[1] + 1e-9)
        return res

    def _trap_right(self, x, p):
        res = np.zeros_like(x)
        mask = (x >= p[0]) & (x <= p[1])
        res[mask] = (x[mask] - p[0]) / (p[1] - p[0] + 1e-9)
        res[(x > p[1]) & (x <= p[2])] = 1.0
        return res

    def plot_membership_functions(self):
        """1. 绘制隶属度函数图"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 定义显示范围 (根据 init_fuzzy_sets 定义)
        ranges = [
            np.linspace(0, 2.0, 500),  # F_h
            np.linspace(0, 6.0, 500),  # T_h (Duration)
            np.linspace(0, 0.1, 500),  # D_r (基于代码注释的 0.1 范围)
            np.linspace(0, 1.0, 500),  # Lambda
        ]

        titles = [
            "(a) Interaction Force $F_h$ (N)",
            "(b) Interaction Duration $T_h$ (s)",
            "(c) Task Distance $D_r$ (m)",
            "(d) Arbitration Coefficient $\lambda$",
        ]

        for i in range(3):
            ax = axes.flatten()[i]
            input_set = self.flt.input_sets[i]
            for idx, (name, params) in enumerate(input_set.items()):
                # 匹配 PS=左梯形, PL=右梯形 逻辑
                if "PS" in name:
                    y = self._trap_left(ranges[i], params)
                elif "PL" in name:
                    y = self._trap_right(ranges[i], params)
                else:
                    y = self._triangular_mf(ranges[i], params)
                ax.plot(ranges[i], y, label=name, lw=2.5, color=self.colors[idx % 5])

            ax.set_title(titles[i], fontsize=14, fontweight="bold")
            ax.set_ylim(0, 1.1)
            ax.legend(loc="upper right", frameon=True)
            ax.grid(True, linestyle="--", alpha=0.5)

        # 输出 λ
        ax = axes[1, 1]
        for idx, (name, params) in enumerate(self.flt.output_sets.items()):
            if name == "Z":
                y = self._trap_left(ranges[3], params)
            elif name == "PL":
                y = self._trap_right(ranges[3], params)
            else:
                y = self._triangular_mf(ranges[3], params)
            ax.plot(ranges[3], y, label=name, lw=2.5, color=self.colors[idx % 5])

        ax.set_title(titles[3], fontsize=14, fontweight="bold")
        ax.legend(loc="upper right", frameon=True)
        ax.grid(True, linestyle="--", alpha=0.5)

        plt.tight_layout()
        plt.savefig("academic_mfs.png", dpi=300)
        print("已成功生成: academic_mfs.png")

    def plot_rules_table(self):
        """2. 绘制模糊规则表"""
        # --- 修复：使用 self.flt.rule_dict ---
        rules = self.flt.rule_dict

        fig, ax = plt.subplots(figsize=(12, 8))
        ax.axis("off")

        # 构建表格数据
        headers = [
            "ID",
            "Force ($F_h$)",
            "Duration ($T_h$)",
            "Distance ($D_r$)",
            "$\lambda$ Output",
        ]
        data = []
        for i, (key, val) in enumerate(rules.items()):
            data.append([i + 1, key[0], key[1], key[2], val])

        # 为了美观，仅显示前 30 条规则（如果规则很多）
        display_data = data
        table = ax.table(
            cellText=[headers] + display_data,
            loc="center",
            cellLoc="center",
            colWidths=[0.1, 0.2, 0.2, 0.2, 0.2],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1.5)

        # 简单美化表头
        for j in range(5):
            table[(0, j)].set_facecolor("#eeeeee")
            table[(0, j)].get_text().set_weight("bold")

        plt.title(
            "Fuzzy Logic Rule Base (Selection)", fontsize=16, fontweight="bold", pad=20
        )
        plt.savefig("fuzzy_rules_table.png", dpi=300, bbox_inches="tight")
        print("已成功生成: fuzzy_rules_table.png")

    def plot_3d_surface(self):
        """3. 绘制三维控制表面"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")

        # 基于 compute 函数内部的裁剪区间进行采样
        F = np.linspace(0, 2, 40)
        T = np.linspace(0, 6, 40)
        F, T = np.meshgrid(F, T)
        L = np.zeros_like(F)

        # 固定 D_r = 0.4 (即规则中的“小”和“中”临界点)
        fixed_dr = 0.03

        for r in range(F.shape[0]):
            for c in range(F.shape[1]):
                L[r, c] = self.flt.compute([F[r, c], T[r, c], fixed_dr])

        surf = ax.plot_surface(
            F, T, L, cmap="coolwarm", edgecolor="none", alpha=0.9, antialiased=True
        )

        ax.set_xlabel("Force $F_h$ (N)", fontsize=12, labelpad=10)
        ax.set_ylabel("Duration $T_h$ (s)", fontsize=12, labelpad=10)
        ax.set_zlabel("Output $\lambda$", fontsize=12, labelpad=10)
        ax.set_title(
            f"Fuzzy Control Surface (at $D_r={fixed_dr}$ m)",
            fontsize=15,
            fontweight="bold",
            pad=20,
        )

        ax.view_init(elev=30, azim=-135)
        fig.colorbar(surf, shrink=0.5, aspect=10, pad=0.1)

        plt.savefig("fuzzy_3d_surface.png", dpi=300)
        print("已成功生成: fuzzy_3d_surface.png")


if __name__ == "__main__":
    viz = FuzzyAcademicVisualizer()
    viz.plot_membership_functions()
    viz.plot_rules_table()
    viz.plot_3d_surface()
    plt.show()
