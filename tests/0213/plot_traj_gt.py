import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import os

# ==========================================
# 1. 配置路径与参数
# ==========================================
# 更新后的数据路径
# BASE_PATH = "logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_015547"  #False
BASE_PATH = "logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_064530"


# 目标点定义
TARGETS = {"a": (0.25, 0.01, 0.03), "b": (0.30, 0.01, 0.03), "c": (0.35, 0.01, 0.03)}

# 轨迹文件映射
TRAJ_FILES = {
    "Slave": "file_traj_slave.xlsx",
    "Master": "file_traj_master.xlsx",
    "Robot": "file_traj_robot.xlsx",
    "Human": "file_traj_human.xlsx",
    "Reshape": "file_traj_reshape.xlsx",
    "Direct": "file_traj_direct.xlsx",
}

# 学术绘图风格
sns.set_theme(style="whitegrid")
plt.rcParams.update(
    {
        "font.sans-serif": ["DejaVu Sans"],
        "axes.unicode_minus": False,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "axes.titlesize": 14,
        "axes.labelsize": 12,
    }
)


def load_xlsx(filename):
    path = os.path.join(BASE_PATH, filename)
    if os.path.exists(path):
        return pd.read_excel(path)
    print(f"跳过未找到的文件: {path}")
    return None


# ==========================================
# 2. 交互力检测逻辑
# ==========================================
def get_interaction_events():
    df_force = load_xlsx("force.xlsx")
    if df_force is None:
        return None, [], []

    # 计算力模长 (Force Norm)
    f_norm = np.sqrt(
        df_force["master_x"] ** 2
        + df_force["master_y"] ** 2
        + df_force["master_z"] ** 2
    )

    # 交互检测 (阈值 0.05N)
    threshold = 0.05
    is_active = (f_norm > threshold).astype(int)
    diff = is_active.diff()

    starts = diff[diff == 1].index.tolist()
    ends = diff[diff == -1].index.tolist()
    return f_norm, starts, ends


# ==========================================
# 3. 绘图函数
# ==========================================
def plot_results_with_xyz_errors():
    # 获取力事件索引
    f_norm, start_indices, end_indices = get_interaction_events()

    # 存储标准化轨迹数据供计算和对比使用
    traj_data = {}
    for label, file in TRAJ_FILES.items():
        df = load_xlsx(file)
        if df is None:
            continue
        xc = "slave_x" if "slave_x" in df.columns else "master_x"
        yc = "slave_y" if "slave_y" in df.columns else "slave_y"  # 修正可能的 typo
        yc = "slave_y" if "slave_y" in df.columns else "master_y"
        zc = "slave_z" if "slave_z" in df.columns else "master_z"
        traj_data[label] = df[[xc, yc, zc]].rename(columns={xc: "x", yc: "y", zc: "z"})

    # 计算三轴平均绝对误差 (MAE) - 基于 Slave 和 Master 的对比
    mean_errors = {}
    if "Slave" in traj_data and "Master" in traj_data:
        s = traj_data["Slave"]
        m = traj_data["Master"]
        # 取最小长度进行对比
        l = min(len(s), len(m))
        mean_errors["x"] = np.abs(s["x"][:l] - m["x"][:l]).mean() * 1000  # 转为 mm
        mean_errors["y"] = np.abs(s["y"][:l] - m["y"][:l]).mean() * 1000
        mean_errors["z"] = np.abs(s["z"][:l] - m["z"][:l]).mean() * 1000

    # --- A. 3D 轨迹对比 ---
    fig_3d = plt.figure(figsize=(12, 10))
    ax_3d = fig_3d.add_subplot(111, projection="3d")

    # 标注目标点
    for name, pos in TARGETS.items():
        ax_3d.scatter(
            pos[0], pos[1], pos[2], s=250, marker="*", edgecolors="k", zorder=15
        )
        ax_3d.text(
            pos[0], pos[1], pos[2], f" Target {name}", fontsize=12, fontweight="bold"
        )

    for label, data in traj_data.items():
        if label == "Slave":
            ax_3d.plot(
                data["x"],
                data["y"],
                data["z"],
                label=f"{label} (Actual)",
                color="red",
                linewidth=3.5,
                alpha=1.0,
                zorder=10,
            )
            # 在红色曲线上标注交互力事件
            for i, s_idx in enumerate(start_indices):
                if s_idx < len(data):
                    p = data.iloc[s_idx]
                    ax_3d.scatter(
                        p["x"],
                        p["y"],
                        p["z"],
                        color="blue",
                        s=100,
                        marker="o",
                        edgecolors="w",
                        zorder=20,
                    )
                    ax_3d.text(
                        p["x"],
                        p["y"],
                        p["z"],
                        f" Start {i+1}",
                        color="blue",
                        fontweight="bold",
                    )
            for i, e_idx in enumerate(end_indices):
                if e_idx < len(data):
                    p = data.iloc[e_idx]
                    ax_3d.scatter(
                        p["x"],
                        p["y"],
                        p["z"],
                        color="darkorange",
                        s=100,
                        marker="s",
                        edgecolors="w",
                        zorder=20,
                    )
                    ax_3d.text(
                        p["x"],
                        p["y"],
                        p["z"],
                        f" End {i+1}",
                        color="darkorange",
                        fontweight="bold",
                    )
        elif label == "Master":
            ax_3d.plot(
                data["x"],
                data["y"],
                data["z"],
                label=f"{label} (Reference)",
                color="blue",
                linewidth=2,
                alpha=1.0,
                zorder=10,
            )
        else:
            ax_3d.plot(
                data["x"], data["y"], data["z"], label=label, linewidth=1.2, alpha=0.6
            )

    ax_3d.set_xlabel("X (m)", labelpad=10)
    ax_3d.set_ylabel("Y (m)", labelpad=10)
    ax_3d.set_zlabel("Z (m)", labelpad=10)
    ax_3d.set_title("3D Trajectory Comparison with Force Events", fontweight="bold")
    ax_3d.legend(loc="upper right", bbox_to_anchor=(1.2, 1))
    plt.tight_layout()
    plt.savefig("trajectory_3d_events.png", dpi=300)

    # --- B. XYZ 坐标时间序列对比 (标注 MAE) ---
    fig_xyz, axes = plt.subplots(3, 1, figsize=(14, 15), sharex=True)
    coords = ["x", "y", "z"]
    coord_labels = ["X Position (m)", "Y Position (m)", "Z Position (m)"]

    for i, (coord, title) in enumerate(zip(coords, coord_labels)):
        ax = axes[i]
        for label, data in traj_data.items():
            color = "red" if label == "Slave" else None
            color = "blue" if label == "Master" else color
            lw = 2.5 if label == "Slave" or label == "Master" else 1.2
            ax.plot(data[coord], label=label, color=color, linewidth=lw, alpha=0.8)

        # 标注平均误差 MAE
        if coord in mean_errors:
            error_text = f"Mean Error: {mean_errors[coord]:.3f} mm"
            ax.text(
                0.02,
                0.05,
                error_text,
                transform=ax.transAxes,
                fontsize=12,
                fontweight="bold",
                verticalalignment="bottom",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
            )

        # 标注力交互垂直线
        for s in start_indices:
            ax.axvline(x=s, color="blue", linestyle="--", alpha=0.4)
        for e in end_indices:
            ax.axvline(x=e, color="darkorange", linestyle="--", alpha=0.4)

        ax.set_ylabel(title, fontweight="bold")
        ax.grid(True, linestyle=":", alpha=0.6)
        if i == 0:
            ax.legend(loc="upper right", ncol=3)

    axes[2].set_xlabel("Sample Index (Time Step)", fontweight="bold")
    plt.tight_layout()
    plt.savefig("trajectory_xyz_with_mae.png", dpi=300)

    print("图像已保存：\n1. trajectory_3d_events.png\n2. trajectory_xyz_with_mae.png")
    if mean_errors:
        print(
            f"平均绝对误差: X={mean_errors['x']:.3f}mm, Y={mean_errors['y']:.3f}mm, Z={mean_errors['z']:.3f}mm"
        )


if __name__ == "__main__":
    plot_results_with_xyz_errors()
    plt.show()
