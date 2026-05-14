import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import os
import glob

# ==========================================
# 1. 路径与指标配置
# ==========================================
# 实验文件夹的根目录 (包含 GT_KF_... 等子文件夹)
BASE_LOGS_DIR = "logs/20260213/0213_*"
# 汇总文件存储路径
INPUT_PATH = "logs/20260213/summary_batch_results.xlsx"
# 输出结果目录
OUTPUT_DIR = "logs/20260213"

OUTPUT_EXCEL = os.path.join(OUTPUT_DIR, "detailed_significance_analysis.xlsx")
OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "significance_annotated_final.png")

# 评价指标映射 (已移除 traj_ldlj)
metrics = {
    "mean_error_rcm": "Mean RCM Error (m)",
    "mean_error_ee": "Mean EE Error (m)",
    "mean_force_norm": "Interaction Force (N)",
    "Efs_force_smoothness": "Force Smoothness (Efs)",
    "traj_rms_jerk": "Trajectory RMS Jerk",
    "mean_control_force": "Control Linear Force (N)",
    "mean_control_torque": "Control Torque (Nm)",
}

# 显著性对比组合
target_pairs = [
    ("GT_KF", "GT_Sigmoid"),
    ("GT_KF", "MPC_KF"),
    ("GT_KF", "GT_KF_WithOtherTarget"),
    ("MPC_KF", "MPC_Sigmoid"),
    ("GT_Sigmoid", "MPC_Sigmoid"),
]


def get_sig_label(p):
    """根据P值返回星号标注"""
    if p < 0.001:
        return "***"
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return None


# ==========================================
# 2. 自动汇总文件生成逻辑
# ==========================================
def generate_summary_table():
    """扫描子文件夹并生成汇总 Excel"""
    print(f"汇总文件缺失。正在从 {BASE_LOGS_DIR} 自动提取数据并生成...")

    # 获取目录下所有带时间戳的子文件夹
    folders = [
        f
        for f in glob.glob(os.path.join(BASE_LOGS_DIR, "*_*_20*_*"))
        if os.path.isdir(f)
    ]
    if not folders:
        raise FileNotFoundError(f"在 {BASE_LOGS_DIR} 下未找到有效的实验文件夹。")

    all_results = []
    for folder in sorted(folders):
        folder_name = os.path.basename(folder)
        res = {"folder_name": folder_name}

        try:
            # 1. 处理 Error (RCM & EE)
            df_err = pd.read_excel(os.path.join(folder, "error.xlsx"))
            res["mean_error_rcm"] = df_err["error_rcm"].mean()
            res["mean_error_ee"] = df_err["error_ee"].mean()

            # 2. 处理 Interaction Force
            df_f = pd.read_excel(os.path.join(folder, "force.xlsx"))
            f_vals = df_f.select_dtypes(include=[np.number]).values
            f_mag = np.linalg.norm(f_vals, axis=1)
            res["mean_force_norm"] = np.mean(f_mag)
            res["Efs_force_smoothness"] = (
                np.mean(np.diff(f_mag) ** 2) if len(f_mag) > 1 else 0
            )

            # 3. 处理 Trajectory Jerk
            df_t = pd.read_excel(os.path.join(folder, "file_traj_slave.xlsx"))
            coords = df_t.select_dtypes(include=[np.number]).iloc[:, :3].values
            res["traj_rms_jerk"] = np.sqrt(
                np.mean(np.sum(np.diff(coords, n=3, axis=0) ** 2, axis=1))
            )

            # 4. 处理 End Torque (控制力与扭矩)
            torque_file = os.path.join(folder, "end_torque.xlsx")
            if os.path.exists(torque_file):
                df_et = pd.read_excel(torque_file)
                # 平动力 xyz
                res["mean_control_force"] = np.mean(
                    np.linalg.norm(
                        df_et[["slave_x", "slave_y", "slave_z"]].values, axis=1
                    )
                )
                # 扭矩 abc
                res["mean_control_torque"] = np.mean(
                    np.linalg.norm(
                        df_et[["slave_a", "slave_b", "slave_c"]].values, axis=1
                    )
                )
            else:
                # MPC 或缺失文件填入 NaN
                res["mean_control_force"] = np.nan
                res["mean_control_torque"] = np.nan

            all_results.append(res)
        except Exception as e:
            print(f"处理文件夹 {folder_name} 时发生错误: {e}")

    summary_df = pd.DataFrame(all_results)
    summary_df.to_excel(INPUT_PATH, index=False)
    print(f"汇总表已成功生成并保存至: {INPUT_PATH}")
    return summary_df


# ==========================================
# 3. 主程序逻辑
# ==========================================
def main():
    # --- A. 数据检查与获取 ---
    if not os.path.exists(INPUT_PATH):
        df = generate_summary_table()
    else:
        print(f"正在读取现有汇总文件: {INPUT_PATH}")
        df = pd.read_excel(INPUT_PATH)

    # 提取方法名
    df["method"] = df["folder_name"].apply(lambda x: "_".join(x.split("_")[:-2]))

    # --- B. 显著性检验计算 ---
    print("正在进行统计分析...")
    pairwise_results = []
    for m1, m2 in target_pairs:
        for m_key, m_name in metrics.items():
            d1 = df[df["method"] == m1][m_key].dropna()
            d2 = df[df["method"] == m2][m_key].dropna()

            if len(d1) < 2 or len(d2) < 2:
                continue

            try:
                # 统计路径：正态性 -> 方差齐性 -> 检验
                _, p_sh1 = stats.shapiro(d1)
                _, p_sh2 = stats.shapiro(d2)
                if p_sh1 > 0.05 and p_sh2 > 0.05:
                    equal_var = stats.levene(d1, d2)[1] > 0.05
                    _, p = stats.ttest_ind(d1, d2, equal_var=equal_var)
                    test_name = "t-test"
                else:
                    _, p = stats.mannwhitneyu(d1, d2, alternative="two-sided")
                    test_name = "Mann-Whitney"
            except:
                p, test_name = 1.0, "Error"

            pairwise_results.append(
                {
                    "Comparison": f"{m1} vs {m2}",
                    "Metric": m_name,
                    "P-Value": p,
                    "Significant": "Yes" if p < 0.05 else "No",
                    "Test Type": test_name,
                    f"Mean_{m1}": d1.mean(),
                    f"Mean_{m2}": d2.mean(),
                }
            )

    results_df = pd.DataFrame(pairwise_results)

    # --- C. 控制台打印 ---
    print("\n" + "=" * 80)
    print(
        results_df[["Comparison", "Metric", "P-Value", "Significant"]].to_string(
            index=False
        )
    )
    print("=" * 80)

    # --- D. 保存 Excel ---
    with pd.ExcelWriter(OUTPUT_EXCEL) as writer:
        results_df.to_excel(writer, sheet_name="Pairwise_Significance", index=False)
        df.groupby("method")[[m for m in metrics.keys() if m in df.columns]].agg(
            ["mean", "std"]
        ).to_excel(writer, sheet_name="Raw_Stats_Summary")
    print(f"详细报告已保存至: {OUTPUT_EXCEL}")

    # --- E. 绘图与标注 ---
    sns.set_theme(style="whitegrid")
    plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Liberation Sans", "sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(3, 3, figsize=(24, 18))
    axes_flat = axes.flatten()

    relevant_methods = sorted(list(set([m for pair in target_pairs for m in pair])))
    df_plot = df[df["method"].isin(relevant_methods)]

    for i, (m_key, m_name) in enumerate(metrics.items()):
        ax = axes_flat[i]
        sns.boxplot(
            x="method",
            y=m_key,
            data=df_plot,
            order=relevant_methods,
            ax=ax,
            palette="Set3",
            hue="method",
            fliersize=0,
        )
        sns.stripplot(
            x="method",
            y=m_key,
            data=df_plot,
            order=relevant_methods,
            ax=ax,
            palette="dark:.3",
            hue="method",
            alpha=0.5,
            jitter=True,
        )

        if ax.get_legend():
            ax.get_legend().remove()
        ax.set_title(m_name, fontsize=16, fontweight="bold")
        ax.tick_params(axis="x", rotation=45)

        # 标注
        y_min, y_max = ax.get_ylim()
        y_range = y_max - y_min
        current_annot_y = y_max + 0.02 * y_range

        for m1, m2 in target_pairs:
            match = results_df[
                (results_df["Comparison"] == f"{m1} vs {m2}")
                & (results_df["Metric"] == m_name)
            ]
            if match.empty:
                continue
            p_val = match.iloc[0]["P-Value"]
            sig_label = get_sig_label(p_val)
            if sig_label:
                idx1, idx2 = relevant_methods.index(m1), relevant_methods.index(m2)
                h = 0.02 * y_range
                ax.plot(
                    [idx1, idx1, idx2, idx2],
                    [
                        current_annot_y,
                        current_annot_y + h,
                        current_annot_y + h,
                        current_annot_y,
                    ],
                    lw=1.2,
                    c="k",
                )
                ax.text(
                    (idx1 + idx2) * 0.5,
                    current_annot_y + h,
                    sig_label,
                    ha="center",
                    va="bottom",
                    color="k",
                    fontsize=12,
                )
                current_annot_y += 0.12 * y_range
        ax.set_ylim(y_min, current_annot_y + 0.05 * y_range)

    # 删除多余的子图
    for j in range(len(metrics), len(axes_flat)):
        fig.delaxes(axes_flat[j])

    plt.tight_layout()
    plt.savefig(OUTPUT_IMAGE, dpi=300)
    plt.show()


if __name__ == "__main__":
    main()
