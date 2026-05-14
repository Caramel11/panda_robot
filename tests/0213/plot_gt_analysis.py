import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import os

# ==========================================
# 1. 配置参数与路径
# ==========================================
# 更新后的实验数据路径
# BASE_PATH = "logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_015547   #False
BASE_PATH = "logs/20260213/0213_GT_KF_WithOtherTarget/GT_KF_WithOtherTarget_20260214_064530"


# 目标点坐标定义
TARGET_PTS = {"a": (0.25, 0.0, 0.03), "b": (0.30, 0.0, 0.03), "c": (0.35, 0.0, 0.03)}

# 学术绘图风格
sns.set_theme(style="whitegrid")
plt.rcParams.update(
    {
        "font.sans-serif": ["DejaVu Sans"],
        "axes.unicode_minus": False,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "legend.fontsize": 10,
        "axes.labelsize": 12,
        "axes.titlesize": 14,
    }
)


def load_data(file_name):
    """读取指定目录下的 Excel 文件"""
    path = os.path.join(BASE_PATH, file_name)
    if not os.path.exists(path):
        print(f"警告: 文件未找到 -> {path}")
        return None
    return pd.read_excel(path)


# ==========================================
# 2. 交互力事件检测逻辑
# ==========================================
def detect_interaction_events(force_df):
    """检测交互力开始(从0跃升)和结束(瞬间降到0)的位置"""
    # 计算力模长 (Force Norm)
    f_norm = np.sqrt(
        force_df["master_x"] ** 2
        + force_df["master_y"] ** 2
        + force_df["master_z"] ** 2
    )

    # 阈值判定：识别显著的力交互行为 (0.05N作为噪声过滤阈值)
    threshold = 0.05
    is_interacting = (f_norm > threshold).astype(int)

    # 计算差分获取起始和结束的跳变点
    diff = is_interacting.diff()
    start_indices = diff[diff == 1].index.tolist()
    end_indices = diff[diff == -1].index.tolist()

    return f_norm, start_indices, end_indices


# ==========================================
# 3. 核心绘图程序
# ==========================================
def run_full_visualization():
    # --- A. 加载核心参数数据 ---
    df_force = load_data("force.xlsx")
    df_error = load_data("error.xlsx")
    df_arb = load_data("arbitrary.xlsx")
    df_torque = load_data("end_torque.xlsx")
    df_prob = load_data("posterior_probabilities.xlsx")

    if df_force is None:
        return

    # 执行事件检测
    f_norm, start_idxs, end_idxs = detect_interaction_events(df_force)

    # 统一长度对齐
    min_len = min(len(df_force), len(df_error), len(df_arb), len(df_prob))
    time_idx = np.arange(min_len)

    # --- B. 3D 轨迹对比图 (带事件标注) ---
    fig_3d = plt.figure(figsize=(12, 10))
    ax_3d = fig_3d.add_subplot(111, projection="3d")

    # 1. 绘制目标点 (Stars)
    colors_pts = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for (name, pos), color in zip(TARGET_PTS.items(), colors_pts):
        ax_3d.scatter(
            pos[0],
            pos[1],
            pos[2],
            s=300,
            marker="*",
            color=color,
            edgecolors="black",
            label=f"Target {name} {pos}",
            zorder=15,
        )
        ax_3d.text(pos[0], pos[1], pos[2], f"  {name}", fontsize=14, fontweight="bold")

    # 2. 绘制各轨迹线条
    traj_list = ["direct", "human", "master", "reshape", "robot", "slave"]
    df_slave_traj = None

    for t in traj_list:
        df_t = load_data(f"file_traj_{t}.xlsx")
        if df_t is None:
            continue

        # 兼容不同文件的坐标列名
        xc = "master_x" if "master_x" in df_t.columns else "slave_x"
        yc = "master_y" if "master_y" in df_t.columns else "slave_y"
        zc = "master_z" if "master_z" in df_t.columns else "slave_z"

        if t == "slave":
            df_slave_traj = df_t
            # Slave 轨迹加粗红色，最醒目
            ax_3d.plot(
                df_t[xc],
                df_t[yc],
                df_t[zc],
                label="Slave (Actual Traj)",
                linewidth=3.5,
                color="red",
                alpha=1.0,
                zorder=10,
            )
        elif t == "master":
            df_slave_traj = df_t
            # Slave 轨迹加粗红色，最醒目
            ax_3d.plot(
                df_t[xc],
                df_t[yc],
                df_t[zc],
                label="Master (Reference Traj)",
                linewidth=2,
                color="blue",
                alpha=1.0,
                zorder=10,
            )
        else:
            ax_3d.plot(
                df_t[xc],
                df_t[yc],
                df_t[zc],
                label=t.capitalize(),
                linewidth=1.2,
                alpha=0.5,
            )

    # 3. 在 Slave 轨迹上精确标注事件点
    if df_slave_traj is not None:
        for i, s_idx in enumerate(start_idxs):
            if s_idx < len(df_slave_traj):
                p = df_slave_traj.iloc[s_idx]
                ax_3d.scatter(
                    p[xc],
                    p[yc],
                    p[zc],
                    color="blue",
                    s=120,
                    marker="o",
                    edgecolors="white",
                    zorder=20,
                )
                ax_3d.text(
                    p[xc],
                    p[yc],
                    p[zc],
                    f" Force Start {i+1}",
                    color="blue",
                    fontweight="bold",
                    fontsize=10,
                )
        for i, e_idx in enumerate(end_idxs):
            if e_idx < len(df_slave_traj):
                p = df_slave_traj.iloc[e_idx]
                ax_3d.scatter(
                    p[xc],
                    p[yc],
                    p[zc],
                    color="orange",
                    s=120,
                    marker="s",
                    edgecolors="white",
                    zorder=20,
                )
                ax_3d.text(
                    p[xc],
                    p[yc],
                    p[zc],
                    f" Force End {i+1}",
                    color="orange",
                    fontweight="bold",
                    fontsize=10,
                )

    ax_3d.set_xlabel("X (m)", labelpad=10)
    ax_3d.set_ylabel("Y (m)", labelpad=10)
    ax_3d.set_zlabel("Z (m)", labelpad=10)
    # ax_3d.set_title(
    #     "3D Trajectory Analysis with Interaction Event Markers", fontweight="bold"
    # )
    ax_3d.legend(loc="upper right", bbox_to_anchor=(1.28, 1))
    plt.tight_layout()
    plt.savefig("experiment_3d_trajectories_final.png", dpi=300)

    # --- C. 多模态参数对比图 (4 Panels) ---
    fig_2d, axes = plt.subplots(4, 1, figsize=(15, 22), sharex=True)

    # 定义全局事件标注辅助函数
    def mark_event_lines(ax, current_min_len):
        for s in start_idxs:
            if s < current_min_len:
                ax.axvline(x=s, color="blue", ls="--", alpha=0.4, lw=1.5)
                if ax == axes[0]:
                    ax.text(
                        s,
                        ax.get_ylim()[1] * 0.85,
                        "Force Start",
                        color="blue",
                        rotation=90,
                        fontsize=10,
                        fontweight="bold",
                    )
        for e in end_idxs:
            if e < current_min_len:
                ax.axvline(x=e, color="orange", ls="--", alpha=0.4, lw=1.5)
                if ax == axes[0]:
                    ax.text(
                        e,
                        ax.get_ylim()[1] * 0.85,
                        "Force End",
                        color="orange",
                        rotation=90,
                        fontsize=10,
                        fontweight="bold",
                    )

    # Panel 1: 交互力与仲裁系数 (Force + Alpha + Beta)
    axes[0].plot(
        time_idx,
        f_norm[:min_len],
        color="black",
        label="Interaction Force Magnitude (N)",
        lw=1.5,
    )
    ax0_twin = axes[0].twinx()
    # 将 Alpha 和 Beta 绘制在同一个右 Y 轴
    ax0_twin.plot(
        time_idx,
        df_arb["alpha"][:min_len],
        color="green",
        label=r"Arbitration $\alpha$",
        lw=2,
    )
    ax0_twin.plot(
        time_idx,
        df_arb["beta"][:min_len],
        color="darkred",
        label=r"Arbitration $\beta$",
        lw=2,
        linestyle="--",
    )

    axes[0].set_ylabel("Force (N)", fontweight="bold")
    ax0_twin.set_ylabel("Weight Value", fontweight="bold")
    axes[0].set_title(
        "Panel 1: Interaction Force & Combined Arbitration Coefficients",
        fontweight="bold",
    )
    # 合并两个坐标轴的图例
    h1, l1 = axes[0].get_legend_handles_labels()
    h2, l2 = ax0_twin.get_legend_handles_labels()
    axes[0].legend(h1 + h2, l1 + l2, loc="upper right", frameon=True, shadow=True)

    # Panel 2: 跟踪误差 (RCM & EE Error)
    axes[1].plot(
        time_idx,
        df_error["error_rcm"][:min_len],
        label="RCM Tracking Error (m)",
        color="#1f77b4",
        lw=1.5,
    )
    axes[1].plot(
        time_idx,
        df_error["error_ee"][:min_len],
        label="End-Effector Error (m)",
        color="#d62728",
        lw=1.5,
        alpha=0.8,
    )
    axes[1].set_ylabel("Error (m)", fontweight="bold")
    axes[1].set_title("Panel 2: System Operational Errors", fontweight="bold")
    axes[1].legend(loc="upper right")

    # Panel 3: 意图后验概率 (Target-specific Probabilities)
    # 按要求映射 3 列数据到 a, b, c
    df_prob.columns = ["Prob_Target_a", "Prob_Target_b", "Prob_Target_c"]
    axes[2].plot(
        time_idx,
        df_prob["Prob_Target_a"][:min_len],
        label="Target a (0.25, 0.06, 0.03)",
        alpha=0.85,
        lw=2,
    )
    axes[2].plot(
        time_idx,
        df_prob["Prob_Target_b"][:min_len],
        label="Target b (0.30, 0.06, 0.03)",
        alpha=0.85,
        lw=2,
    )
    axes[2].plot(
        time_idx,
        df_prob["Prob_Target_c"][:min_len],
        label="Target c (0.35, 0.06, 0.03)",
        alpha=0.85,
        lw=2,
    )
    axes[2].set_ylabel("Posterior Probability", fontweight="bold")
    axes[2].set_title("Panel 3: Intent Identification Probabilities", fontweight="bold")
    axes[2].legend(loc="upper right")

    # Panel 4: 末端执行器扭矩模长
    torque_mag = np.sqrt(
        df_torque["slave_x"] ** 2
        + df_torque["slave_y"] ** 2
        + df_torque["slave_z"] ** 2
    )
    axes[3].plot(
        time_idx,
        torque_mag[:min_len],
        color="purple",
        label="End-Effector Torque (N·m)",
        lw=1.3,
    )
    axes[3].set_ylabel("Torque (N·m)", fontweight="bold")
    axes[3].set_xlabel("Sample Index (Time Step)", fontweight="bold")
    axes[3].set_title("Panel 4: Interaction Torque at End-Effector", fontweight="bold")
    axes[3].legend(loc="upper right")

    # 遍历所有子图，同步标注垂直事件虚线和网格线
    for ax in axes:
        mark_event_lines(ax, min_len)
        ax.grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    plt.savefig("experiment_parameter_sync_analysis.png", dpi=300)
    print(
        "绘制完成：\n1. 3D轨迹图 -> experiment_3d_trajectories_final.png\n2. 多面板分析图 -> experiment_parameter_sync_analysis.png"
    )


if __name__ == "__main__":
    run_full_visualization()
    plt.show()
