import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

# 自动尝试从本地加载，失败则使用内置定义的规则（delta_lambda_based）
try:
    from fuzzy_logic import FuzzyLogicTool
except ImportError:

    class FuzzyLogicTool:
        def __init__(self, type="delta_lambda_based"):
            self.rule_dict = {
                ("N", "N", "N"): "NL",
                ("N", "N", "Z"): "NL",
                ("N", "N", "P"): "N",
                ("N", "Z", "N"): "NL",
                ("N", "Z", "Z"): "N",
                ("N", "Z", "P"): "Z",
                ("N", "P", "N"): "N",
                ("N", "P", "Z"): "Z",
                ("N", "P", "P"): "P",
                ("Z", "N", "N"): "NL",
                ("Z", "N", "Z"): "N",
                ("Z", "N", "P"): "Z",
                ("Z", "Z", "N"): "N",
                ("Z", "Z", "Z"): "Z",
                ("Z", "Z", "P"): "P",
                ("Z", "P", "N"): "Z",
                ("Z", "P", "Z"): "P",
                ("Z", "P", "P"): "PL",
                ("P", "N", "N"): "N",
                ("P", "N", "Z"): "Z",
                ("P", "N", "P"): "P",
                ("P", "Z", "N"): "Z",
                ("P", "Z", "Z"): "P",
                ("P", "Z", "P"): "PL",
                ("P", "P", "N"): "P",
                ("P", "P", "Z"): "PL",
                ("P", "P", "P"): "PL",
            }


# --- 置顶美化箭头类 ---
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return -1e9  # 强制置顶


def plot_delta_lambda_rules():
    # 1. 初始化数据
    flt = FuzzyLogicTool(type="delta_lambda_based")
    rules = getattr(flt, "rule_dict", {})

    # 输入标签
    labels_list = ["N", "Z", "P"]
    # 输出标签到数值的映射（用于颜色显示）
    output_vals = {"NL": 0.05, "N": 0.25, "Z": 0.5, "P": 0.75, "PL": 0.95}
    cmap = plt.cm.RdYlBu_r

    tile_size = 0.88
    z_gap = 1.0  # 层间距

    fig = plt.figure(figsize=(16, 14))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect((1, 1, 1.2))

    # 2. 绘制 27 条规则方块
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
                    zorder=1,
                )
                ax.add_collection3d(poly)

                # 标签文本
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
                    zorder=100,
                )

    # 3. 坐标轴与刻度设置
    ax.set_zlim(0, 2 * z_gap)
    ax.set_xticks([0, 1, 2])
    ax.set_xticklabels([])
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels([])
    ax.set_zticks([0, z_gap, 2 * z_gap])
    ax.set_zticklabels([])

    # 绘制辅助刻度文本
    tick_labels = ["Neg (N)", "Zero (Z)", "Pos (P)"]
    for i, txt in enumerate(tick_labels):
        ax.text(
            i + 0.15,
            -0.2,
            -0.3,
            txt,
            fontsize=13,
            ha="center",
            fontweight="bold",
            color="#6E4700",
        )  # X
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
        )  # Y
        ax.text(
            -0.5,
            2.5,
            i * z_gap,
            txt,
            fontsize=13,
            ha="right",
            fontweight="bold",
            color="#690000",
        )  # Z

    # 4. 轴标题 (使用 Delta 符号)
    ax.text(
        1.0,
        -0.8,
        -0.3,
        "$\Delta$ Interaction Force $\Delta F_h$",
        fontsize=17,
        fontweight="bold",
        ha="center",
        color="#6E4700",
    )
    ax.text(
        -0.7,
        -0.2,
        -0.3,
        "$\Delta$ Interaction Duration\n$\Delta T_h$",
        fontsize=17,
        fontweight="bold",
        ha="right",
        rotation=15,
        color="#003077",
    )
    ax.text(
        -0.4,
        3,
        z_gap * 2 + 0.2,
        "$\Delta$ Object Distance $\Delta D_r$",
        fontsize=17,
        fontweight="bold",
        ha="right",
        va="center",
        rotation=90,
        color="#690000",
    )

    # --- 5. 绘制置顶美化箭头 ---
    arrow_props = dict(
        mutation_scale=20,
        arrowstyle="-|>",
        lw=2,
        shrinkA=0,
        shrinkB=0,
        zorder=999,
        clip_on=False,
    )
    ax.add_artist(
        Arrow3D([-0.6, 0.4], [-0.6, -0.6], [-0.2, -0.2], **arrow_props, color="#6E4700")
    )  # X轴
    ax.add_artist(
        Arrow3D([-0.6, -0.6], [-0.6, 0.4], [-0.2, -0.2], **arrow_props, color="#003077")
    )  # Y轴
    ax.add_artist(
        Arrow3D([-0.6, -0.6], [-0.6, -0.6], [-0.2, 1.2], **arrow_props, color="#690000")
    )  # Z轴

    # 6. 视角与视觉清理
    ax.view_init(elev=20, azim=-100)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.set_pane_color((1, 1, 1, 0))
        axis.line.set_color("black")

    # 7. 颜色条
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, 1))
    cbar = fig.colorbar(sm, ax=ax, location="left", shrink=0.7, aspect=15, pad=0.1)
    cbar.set_label(
        "Change in Control Weight ($\Delta \lambda$)",
        fontsize=16,
        fontweight="bold",
        labelpad=25,
    )
    cbar.set_ticks([0.1, 0.5, 0.9])
    cbar.set_ticklabels(["Negative", "Zero", "Positive"], fontsize=13)

    plt.title(
        "Fuzzy Logic Rules Visualization: $\Delta \lambda$ rules",
        fontsize=22,
        fontweight="bold",
        pad=30,
    )
    plt.savefig("delta_lambda_rules.png", dpi=300, bbox_inches="tight")
    plt.show()


if __name__ == "__main__":
    plot_delta_lambda_rules()
