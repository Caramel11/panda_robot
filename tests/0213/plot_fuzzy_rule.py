import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import os

# 注意：由于我无法访问你的本地环境，请确保 fuzzy_logic.py 在同一目录下
try:
    from fuzzy_logic import FuzzyLogicTool
except ImportError:
    # 仅作演示用的 Mock 类，如果你在本地运行请忽略这段
    class FuzzyLogicTool:
        def __init__(self, type="lambda_based"):
            self.rule_dict = {}


# --- 核心改进：定义美化版 3D 箭头类 ---
class Arrow3D(FancyArrowPatch):
    """
    基于 FancyArrowPatch 的 3D 箭头实现，支持实心箭头和复杂的线型设置。
    """

    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)
        # return -1e9


def plot_refined_fuzzy_rules_final():
    # 1. 初始化工具
    flt = FuzzyLogicTool(type="lambda_based")
    rules = getattr(flt, "rule_dict", {})

    # 2. 映射与参数
    labels_list = ["PS", "PM", "PL"]
    output_vals = {"Z": 0.05, "PS": 0.25, "PM": 0.5, "P": 0.75, "PL": 0.95}
    cmap = plt.cm.RdYlBu_r

    tile_size = 0.88
    z_gap = 1.0  # 层间距

    fig = plt.figure(figsize=(16, 14))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect((1, 1, 1.2))  # 视觉拉伸

    # 3. 绘制层
    for z_idx, d_label in enumerate(labels_list):
        z_pos = z_idx * z_gap
        for x_idx, f_label in enumerate(labels_list):
            for y_idx, t_label in enumerate(labels_list):
                out_label = rules.get((f_label, t_label, d_label), "Z")
                val = output_vals.get(out_label, 0.5)
                color = cmap(val)

                x0, y0 = x_idx - tile_size / 2, y_idx - tile_size / 2
                x1, y1 = x_idx + tile_size / 2, y_idx + tile_size / 2
                verts = [
                    [(x0, y0, z_pos), (x1, y0, z_pos), (x1, y1, z_pos), (x0, y1, z_pos)]
                ]
                poly = Poly3DCollection(
                    verts,
                    facecolor=color,
                    alpha=0.9,
                    edgecolor="#111111",
                    lw=1.2,
                    zorder=0,
                )
                ax.add_collection3d(poly)

                # 标签
                text_color = "white" if val < 0.35 or val > 0.65 else "black"
                ax.text(
                    x_idx,
                    y_idx,
                    z_pos,
                    out_label,
                    color=text_color,
                    ha="center",
                    va="center",
                    fontsize=20,
                    fontweight="black",
                    zorder=30,
                )

    # 4. 坐标轴、刻度与标题
    ax.set_zlim(0, 2 * z_gap)
    ax.set_zticks([0, z_gap, 2 * z_gap])
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels([])
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels([])
    ax.set_zticklabels([])

    # --- 刻度向外平移 ---
    x_tick_labels = ["Small (PS)", "Med (PM)", "Large (PL)"]
    for i, txt in enumerate(x_tick_labels):
        ax.text(
            i + 0.15,
            -0.2,
            -0.3,
            txt,
            fontsize=13,
            ha="center",
            fontweight="bold",
            color="#6E4700",
        )

    y_tick_labels = ["Short (PS)", "Med (PM)", "Long (PL)"]
    for i, txt in enumerate(y_tick_labels):
        ax.text(
            -0.9,
            i + 0.65,
            -0.3,
            txt,
            fontsize=13,
            ha="center",
            rotation=-20,
            fontweight="bold",
            color="#003077",
        )

    z_tick_labels = ["Near (PS)", "Mid (PM)", "Far (PL)"]
    for i, txt in enumerate(z_tick_labels):
        ax.text(
            -0.5,
            2.5,
            i * z_gap,
            txt,
            fontsize=13,
            ha="right",
            fontweight="bold",
            color="#690000",
        )

    # --- 轴标题 ---
    ax.text(
        1.0,
        -0.8,
        -0.3,
        "Interaction Force $F_h$",
        fontsize=17,
        fontweight="bold",
        ha="center",
        color="#6E4700",
    )
    ax.text(
        -1.3,
        0,
        -0.3,
        "Interaction Duration $T_h$",
        fontsize=17,
        fontweight="bold",
        ha="center",
        rotation=15,
        color="#003077",
    )
    ax.text(
        -1,
        3,
        z_gap * 2 + 0.2,
        "Object Distance $D_r$",
        fontsize=17,
        fontweight="bold",
        ha="center",
        va="center",
        rotation=90,
        color="#690000",
    )

    # --- 新增：使用 Arrow3D 绘制美化箭头 ---
    arrow_props = dict(
        mutation_scale=20,  # 箭头大小
        arrowstyle="-|>",  # 实心三角头
        lw=2,  # 线宽
        shrinkA=0,
        shrinkB=0,
        zorder=999,  # 软件层面的最高排序
        clip_on=False,  # 防止被轴边界裁剪
    )

    # X轴方向箭头
    ax.add_artist(
        Arrow3D([-0.6, 0.4], [-0.6, -0.6], [-0, -0], **arrow_props, color="#6E4700")
    )
    # Y轴方向箭头
    ax.add_artist(
        Arrow3D([-0.6, -0.6], [-0.6, 0.4], [-0, -0], **arrow_props, color="#003077")
    )
    # Z轴方向箭头
    ax.add_artist(
        Arrow3D([-0.6, -0.6], [-0.6, -0.6], [0, 1], **arrow_props, color="#690000")
    )

    # 视角与风格清理
    ax.view_init(elev=20, azim=-100)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.set_pane_color((1, 1, 1, 0))
        axis.line.set_color("black")

    # 5. 颜色条
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
    cbar = fig.colorbar(sm, ax=ax, location="left", shrink=0.7, aspect=15, pad=0.1)
    cbar.set_label(
        "Human Control Weight ($\lambda$)", fontsize=16, fontweight="bold", labelpad=25
    )
    cbar.set_ticks([0.1, 0.5, 0.9])
    cbar.set_ticklabels(["Robot-Led", "Shared Control", "Human-Led"], fontsize=13)

    plt.title(
        "Fuzzy Logic Rules Visualization: $\lambda$ rules",
        fontsize=22,
        fontweight="bold",
        pad=30,
    )
    plt.savefig("lambda_rules.png", dpi=300, bbox_inches="tight")
    plt.show()


if __name__ == "__main__":
    plot_refined_fuzzy_rules_final()
