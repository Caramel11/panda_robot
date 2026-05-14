import matplotlib.pyplot as plt
import matplotlib.patches as patches


def draw_system_framework():
    # 设置画布
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 10)
    ax.axis("off")

    # 定义颜色和样式
    color_human = "#FFDDC1"  # 浅橙色
    color_robot = "#C1E1FF"  # 浅蓝色
    color_algo = "#D1FFC1"  # 浅绿色
    color_arb = "#E1C1FF"  # 浅紫色
    box_style = dict(boxstyle="round,pad=0.5", ec="black", lw=1.5)

    # ================= 绘制模块 (Nodes) =================

    # 1. Human Operator
    ax.add_patch(patches.FancyBboxPatch((0.5, 7), 2, 1.5, fc=color_human, **box_style))
    ax.text(
        1.5,
        7.75,
        "Human Operator\n(Surgeon)",
        ha="center",
        va="center",
        fontsize=12,
        fontweight="bold",
    )

    # 2. Master Device (Falcon)
    ax.add_patch(patches.FancyBboxPatch((0.5, 4), 2, 1.5, fc="#EEEEEE", **box_style))
    ax.text(
        1.5,
        4.75,
        "Master Device\n(Novint Falcon)",
        ha="center",
        va="center",
        fontsize=11,
    )

    # 3. Arbitrary Module (Intent & Trajectory)
    ax.add_patch(patches.FancyBboxPatch((3.5, 6), 3, 3, fc=color_algo, **box_style))
    ax.text(
        5,
        8.7,
        "Arbitrary Module\n(Intent Inference)",
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold",
    )

    # Inside Arbitrary: Bayesian Goal
    ax.add_patch(patches.Rectangle((3.8, 7.5), 2.4, 0.8, fc="white", ec="black"))
    ax.text(5, 7.9, "Bayesian Goal\nPrediction", ha="center", va="center", fontsize=9)

    # Inside Arbitrary: Traj Deform
    ax.add_patch(patches.Rectangle((3.8, 6.4), 2.4, 0.8, fc="white", ec="black"))
    ax.text(5, 6.8, "Trajectory\nDeformation", ha="center", va="center", fontsize=9)

    # 4. Arbitration Strategy (Fuzzy + KF)
    ax.add_patch(patches.FancyBboxPatch((4, 2.5), 2, 2, fc=color_arb, **box_style))
    ax.text(
        5, 4.2, "Arbitration", ha="center", va="center", fontsize=11, fontweight="bold"
    )
    ax.text(
        5,
        3.5,
        "Fuzzy Logic\n+\nKalman Filter",
        ha="center",
        va="center",
        fontsize=9,
        fontstyle="italic",
    )

    # 5. GT Controller (Differential Game)
    ax.add_patch(
        patches.FancyBboxPatch((7.5, 3.5), 2.5, 2.5, fc=color_robot, **box_style)
    )
    ax.text(
        8.75,
        5.7,
        "Cooperative GT\nController",
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold",
    )
    ax.text(
        8.75,
        4.5,
        "Solve ARE (LQR)\n$u_r = -K_{GT} x$",
        ha="center",
        va="center",
        fontsize=10,
    )

    # 6. Energy Tank (Stability)
    ax.add_patch(patches.FancyBboxPatch((11, 3.5), 2, 1.5, fc="#FFABAB", **box_style))
    ax.text(12, 4.25, "Energy Tank\n(Passivity)", ha="center", va="center", fontsize=11)

    # 7. Slave Robot (Franka)
    ax.add_patch(patches.FancyBboxPatch((11, 6.5), 2, 1.5, fc="#EEEEEE", **box_style))
    ax.text(
        12, 7.25, "Slave Robot\n(Franka Emika)", ha="center", va="center", fontsize=11
    )

    # ================= 绘制连接线 (Edges) =================

    # 【修复重点】：定义 arrowprops 字典，而不是直接作为 kwargs
    # 定义通用的箭头样式
    arrow_common = dict(arrowstyle="->", lw=1.5, color="black")

    # Human -> Master
    # 使用 arrowprops=arrow_common 替代 **props
    ax.annotate("", xy=(1.5, 5.5), xytext=(1.5, 7), arrowprops=arrow_common)
    ax.text(1.6, 6.2, "Force/Motion", fontsize=9)

    # Master -> Arbitrary (Human Input uh)
    ax.annotate("", xy=(3.5, 6.8), xytext=(2.5, 5.2), arrowprops=arrow_common)
    ax.text(3.0, 5.8, "$u_h, x_h$", fontsize=9, rotation=0)

    # Master -> Arbitration (Force input)
    ax.annotate("", xy=(4, 4), xytext=(2.5, 4.5), arrowprops=arrow_common)
    ax.text(3.2, 4.1, "$F_{ext}, v$", fontsize=9)

    # Arbitrary -> GT Controller (Reference Trajectory)
    ax.annotate("", xy=(7.5, 5.5), xytext=(6.5, 6.8), arrowprops=arrow_common)
    ax.text(7.0, 6.3, "$p_{ref}$ (Target)", fontsize=9)

    # Arbitration -> GT Controller (Alpha)
    # 这里需要单独定义紫色的箭头
    arrow_alpha = dict(arrowstyle="->", lw=1.5, color="purple")
    ax.annotate("", xy=(7.5, 4.0), xytext=(6.0, 3.5), arrowprops=arrow_alpha)
    ax.text(6.6, 3.8, "$\\alpha$ (Weight)", fontsize=9, color="purple")

    # GT Controller -> Energy Tank (Control Command)
    ax.annotate("", xy=(11, 4.25), xytext=(10, 4.25), arrowprops=arrow_common)
    ax.text(10.5, 4.4, "$u_r^*$", fontsize=9)

    # Energy Tank -> Slave Robot (Safe Command)
    ax.annotate("", xy=(12, 6.5), xytext=(12, 5), arrowprops=arrow_common)
    ax.text(12.1, 5.8, "$u_{safe}$", fontsize=9)

    # Slave Robot -> Feedback (to Human/Arbitrary)
    # 虚线箭头样式
    arrow_dashed = dict(arrowstyle="-", lw=1.5, linestyle="--", color="black")
    arrow_dashed_head = dict(arrowstyle="->", lw=1.5, linestyle="--", color="black")

    # 画反馈回路的三段线
    ax.annotate("", xy=(13, 8.5), xytext=(12, 8), arrowprops=arrow_dashed)
    ax.annotate("", xy=(0.5, 8.5), xytext=(13, 8.5), arrowprops=arrow_dashed)
    ax.annotate("", xy=(0.5, 7.5), xytext=(0.5, 8.5), arrowprops=arrow_dashed_head)
    ax.text(6.5, 8.6, "Visual/Haptic Feedback (State $x$)", ha="center", fontsize=9)

    # Robot State -> GT Controller (State Feedback)
    # 弧形箭头
    arrow_arc = dict(
        arrowstyle="->",
        lw=1.5,
        linestyle=":",
        color="black",
        connectionstyle="arc3,rad=-0.3",
    )
    ax.annotate("", xy=(8.75, 6), xytext=(11, 7.25), arrowprops=arrow_arc)
    ax.text(9.5, 7.5, "State Feedback $x$", fontsize=9)

    plt.title(
        "Framework of the Game-Theoretic Shared Control System", fontsize=16, pad=20
    )
    plt.tight_layout()

    # 如果你想保存图片，取消下面这行的注释
    # plt.savefig('system_framework.png', dpi=300)
    plt.show()


if __name__ == "__main__":
    draw_system_framework()
