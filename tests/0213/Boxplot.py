import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import os

# ==========================================
# 1. 路径与参数配置
# ==========================================
# 输入文件路径
INPUT_PATH = "logs/20260213/summary_batch_results.xlsx"
# 输出结果目录
OUTPUT_DIR = "logs/20260213"
# 输出文件名称
OUTPUT_EXCEL = os.path.join(OUTPUT_DIR, "detailed_significance_analysis.xlsx")
OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "significance_annotated_final.png")

# 评价指标映射
metrics = {
    "mean_error_rcm": "Mean RCM Error (m)",
    "mean_error_ee": "Mean EE Error (m)",
    "mean_force_norm": "Mean Force Magnitude (N)",
    "Efs_force_smoothness": "Force Smoothness (Efs)",
    "traj_rms_jerk": "Trajectory RMS Jerk",
}

# 显著性对比组合
target_pairs = [
    ("GT_KF", "0Force_GT_KF"),
    ("GT_KF", "GT_Sigmoid"),
    ("GT_KF", "MPC_KF"),
    ("GT_KF", "GT_KF_WithOtherTarget"),
    ("0Force_GT_KF", "0Force_MPC_KF"),
    ("MPC_KF", "MPC_Sigmoid"),
    ("MPC_KF", "0Force_MPC_KF"),
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


def main():
    if not os.path.exists(INPUT_PATH):
        print(f"错误：无法找到输入文件 {INPUT_PATH}")
        return

    # --- 2. 数据处理与清洗 ---
    print("正在加载数据并进行统计分析...")
    df = pd.read_excel(INPUT_PATH)
    # 提取方法名：去掉末尾的时间戳
    df["method"] = df["folder_name"].apply(lambda x: "_".join(x.split("_")[:-2]))

    # --- 3. 显著性检验计算 ---
    pairwise_results = []
    for m1, m2 in target_pairs:
        for m_key, m_name in metrics.items():
            d1 = df[df["method"] == m1][m_key].dropna()
            d2 = df[df["method"] == m2][m_key].dropna()

            if len(d1) < 2 or len(d2) < 2:
                continue

            try:
                # 统计学路径：正态性检验 -> 方差齐性检验 -> 对应检验
                if stats.shapiro(d1)[1] > 0.05 and stats.shapiro(d2)[1] > 0.05:
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
                    "Diff_Percent": ((d2.mean() - d1.mean()) / (d1.mean() + 1e-9))
                    * 100,
                }
            )

    results_df = pd.DataFrame(pairwise_results)

    # --- 4. 控制台打印表格 ---
    print("\n" + "=" * 100)
    print(f"{'Comparison':<40} | {'Metric':<25} | {'P-Value':<10} | {'Significant'}")
    print("-" * 100)
    # 排序输出：按对比组合排序
    sorted_df = results_df.sort_values(["Comparison", "Metric"])
    for _, row in sorted_df.iterrows():
        print(
            f"{row['Comparison']:<40} | {row['Metric']:<25} | {row['P-Value']:<10.6f} | {row['Significant']}"
        )
    print("=" * 100)

    # --- 5. 保存结果至 Excel ---
    summary_stats = df.groupby("method")[list(metrics.keys())].agg(["mean", "std"])
    with pd.ExcelWriter(OUTPUT_EXCEL) as writer:
        results_df.to_excel(writer, sheet_name="Pairwise_Significance", index=False)
        summary_stats.to_excel(writer, sheet_name="Raw_Stats_Summary")
    print(f"\n[1/2] 显著性分析表格已保存至: {OUTPUT_EXCEL}")

    # --- 6. 绘图与标注 ---
    sns.set_theme(style="whitegrid")
    plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Liberation Sans", "sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(2, 3, figsize=(22, 16))
    axes = axes.flatten()

    relevant_methods = sorted(list(set([m for pair in target_pairs for m in pair])))
    df_plot = df[df["method"].isin(relevant_methods)]

    for i, (m_key, m_name) in enumerate(metrics.items()):
        ax = axes[i]
        # 绘图
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
        ax.set_xlabel("")
        ax.set_ylabel("")
        ax.tick_params(axis="x", rotation=45)

        # 显著性标注
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

    plt.tight_layout()
    plt.savefig(OUTPUT_IMAGE, dpi=300)
    print(f"[2/2] 带有显著性标注的对比图已保存至: {OUTPUT_IMAGE}")
    plt.show()


if __name__ == "__main__":
    main()
