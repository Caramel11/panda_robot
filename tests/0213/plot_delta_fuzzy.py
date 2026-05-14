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


class DeltaFuzzyAcademicVisualizer:
    def __init__(self):
        # 初始化为 delta_lambda_based 类型
        self.flt = FuzzyLogicTool(type="delta_lambda_based")

        # 学术图表配置 (参考 plot_fuzzy)
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
        """1. 绘制隶属度函数图 (Delta 模式)"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 根据 fuzzy_logic.py 定义范围
        ranges = [
            np.linspace(-3.0, 3.0, 500),  # dF_h
            np.linspace(0, 2, 500),  # dT_h
            np.linspace(-0.08, 0.08, 500),  # dD_r
            np.linspace(-0.5, 0.5, 500),  # Delta Lambda Output
        ]

        titles = [
            r"(a) Change in Force $\Delta F_h$ (N/s)",
            r"(b) Change in Duration $\Delta T_h$ (1)",
            r"(c) Change in Distance $\Delta D_r$ (cm/s)",
            r"(d) Change in Arbitration $\Delta \lambda$",
        ]

        # 绘制输入模糊集
        for i in range(3):
            ax = axes.flatten()[i]
            input_set = self.flt.input_sets[i]
            for idx, (name, params) in enumerate(input_set.items()):
                # 匹配代码逻辑：N=左梯形, P=右梯形
                if name == "N":
                    y = self._trap_left(ranges[i], params)
                elif name == "P":
                    y = self._trap_right(ranges[i], params)
                else:
                    y = self._triangular_mf(ranges[i], params)
                ax.plot(ranges[i], y, label=name, lw=2.5, color=self.colors[idx % 5])

            ax.set_title(titles[i], fontsize=14, fontweight="bold")
            ax.set_ylim(0, 1.1)
            ax.legend(loc="upper right", frameon=True)
            ax.grid(True, linestyle="--", alpha=0.5)

        # 绘制输出模糊集
        ax = axes[1, 1]
        for idx, (name, params) in enumerate(self.flt.output_sets.items()):
            # 匹配代码逻辑：NL=左梯形, PL=右梯形
            if name == "NL":
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
        plt.savefig("delta_lambda_mf.png", dpi=300)
        print("已成功生成: delta_lambda_mf.png")

    def plot_rules_table(self):
        """2. 绘制模糊规则表"""
        rules = self.flt.rule_dict

        fig, ax = plt.subplots(figsize=(10, 8))
        ax.axis("off")

        headers = [
            "ID",
            "$\Delta F_h$ Label",
            "$\Delta T_h$ Label",
            "$\Delta D_r$ Label",
            "$\Delta \lambda$ Output",
        ]

        data = []
        for i, (key, val) in enumerate(rules.items()):
            data.append([i + 1, key[0], key[1], key[2], val])

        table = ax.table(
            cellText=[headers] + data,
            loc="center",
            cellLoc="center",
            colWidths=[0.08, 0.2, 0.2, 0.2, 0.15],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1.3)

        # 表头着色
        for j in range(5):
            table[(0, j)].set_facecolor("#f2f2f2")
            table[(0, j)].get_text().set_weight("bold")

        plt.title(
            "Incremental Fuzzy Rule Base ($\Delta \lambda$)",
            fontsize=16,
            fontweight="bold",
            pad=20,
        )
        plt.savefig("delta_fuzzy_rules_table.png", dpi=300, bbox_inches="tight")
        print("已成功生成: delta_fuzzy_rules_table.png")

    def plot_3d_surface(self):
        """3. 绘制三维控制表面"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")

        # 采样范围根据输入集设定
        DF = np.linspace(-3.0, 3.0, 40)
        DT = np.linspace(-1.5, 1.5, 40)
        DF, DT = np.meshgrid(DF, DT)
        DL = np.zeros_like(DF)

        # 固定 dD_r = 0 (稳态)
        fixed_ddr = 0.0

        for r in range(DF.shape[0]):
            for c in range(DF.shape[1]):
                DL[r, c] = self.flt.compute(
                    [DF[r, c], DT[r, c], fixed_ddr], types="delta_lambda_based"
                )

        surf = ax.plot_surface(
            DF, DT, DL, cmap="coolwarm", edgecolor="none", alpha=0.9, antialiased=True
        )

        ax.set_xlabel(r"$\Delta F_h$ (N/s)", fontsize=12, labelpad=10)
        ax.set_ylabel(r"$\Delta T_h$ (1/s)", fontsize=12, labelpad=10)
        ax.set_zlabel(r"Output $\Delta \lambda$", fontsize=12, labelpad=10)
        ax.set_title(
            f"Fuzzy Control Surface (at $\Delta D_r={fixed_ddr}$)",
            fontsize=15,
            fontweight="bold",
            pad=20,
        )

        ax.view_init(elev=25, azim=-135)
        fig.colorbar(surf, shrink=0.5, aspect=10, pad=0.1)

        plt.savefig("delta_fuzzy_surface.png", dpi=300)
        print("已成功生成: delta_fuzzy_surface.png")


if __name__ == "__main__":
    viz = DeltaFuzzyAcademicVisualizer()
    viz.plot_membership_functions()
    viz.plot_rules_table()
    viz.plot_3d_surface()
    plt.show()
