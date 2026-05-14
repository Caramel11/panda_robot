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

OUTPUT_EXCEL = os.path.join(OUTPUT_DIR, "detailed_significance_analysis_OtherTarget.xlsx")
OUTPUT_IMAGE = os.path.join(OUTPUT_DIR, "significance_annotated_OtherTarget.png")

# 评价指标映射 (6个指标)
metrics = {
    "mean_error_rcm": "Mean RCM Error (m)",
    "mean_error_ee": "Mean EE Error (m)",
    "mean_force_norm": "Interaction Force (N)",
    "Efs_force_smoothness": "Force Smoothness (Efs)",
    "traj_rms_jerk": "Trajectory RMS Jerk",
    "mean_dist_to_object": "Mean Dist to Object (m)"  
}

# 显著性对比组合
target_pairs = [
    ("GT_KF", "GT_KF_WithOtherTarget"),
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
    print(f"汇总文件缺失或需要更新。正在从 {BASE_LOGS_DIR} 自动提取数据...")

    folders = [
        f for f in glob.glob(os.path.join(BASE_LOGS_DIR, "*_*_20*_*")) if os.path.isdir(f)
    ]
    if not folders:
        raise FileNotFoundError(f"在 {BASE_LOGS_DIR} 下未找到有效的实验文件夹。")

    all_results = []
    
    # 定义 Object 参数
    object_position = np.array([0.3, -0.05, 0.05])
    object_radius = 0.03

    for folder in sorted(folders):
        folder_name = os.path.basename(folder)
        res = {"folder_name": folder_name}

        try:
            # 1. 处理 Error (RCM & EE)
            err_file = os.path.join(folder, "error.xlsx")
            if os.path.exists(err_file):
                df_err = pd.read_excel(err_file)
                res["mean_error_rcm"] = df_err["error_rcm"].mean() if not df_err.empty and "error_rcm" in df_err.columns else np.nan
                res["mean_error_ee"] = df_err["error_ee"].mean() if not df_err.empty and "error_ee" in df_err.columns else np.nan
            else:
                res["mean_error_rcm"], res["mean_error_ee"] = np.nan, np.nan

            # 2. 处理 Interaction Force
            force_file = os.path.join(folder, "force.xlsx")
            if os.path.exists(force_file):
                df_f = pd.read_excel(force_file)
                f_vals = df_f.select_dtypes(include=[np.number]).values
                if len(f_vals) > 0:
                    f_mag = np.linalg.norm(f_vals, axis=1)
                    res["mean_force_norm"] = np.mean(f_mag)
                    res["Efs_force_smoothness"] = np.mean(np.diff(f_mag) ** 2) if len(f_mag) > 1 else np.nan
                else:
                    res["mean_force_norm"], res["Efs_force_smoothness"] = np.nan, np.nan
            else:
                res["mean_force_norm"], res["Efs_force_smoothness"] = np.nan, np.nan

            # 3. 处理 Trajectory Jerk & Object Distance
            traj_file = os.path.join(folder, "file_traj_slave.xlsx")
            if os.path.exists(traj_file):
                df_t = pd.read_excel(traj_file)
                coords = df_t.select_dtypes(include=[np.number]).iloc[:, :3].values
                
                # 计算 Jerk
                if len(coords) >= 4:
                    res["traj_rms_jerk"] = np.sqrt(np.mean(np.sum(np.diff(coords, n=3, axis=0) ** 2, axis=1)))
                else:
                    res["traj_rms_jerk"] = np.nan
                    
                # 计算与 Object 的平均距离
                if len(coords) > 0:
                    dists_to_center = np.linalg.norm(coords - object_position, axis=1)
                    dists_to_surface = dists_to_center - object_radius
                    res["mean_dist_to_object"] = np.mean(dists_to_surface)
                else:
                    res["mean_dist_to_object"] = np.nan
            else:
                res["traj_rms_jerk"] = np.nan
                res["mean_dist_to_object"] = np.nan

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
        if "mean_dist_to_object" not in df.columns:
            print("检测到缺失新指标，正在重新生成汇总文件...")
            df = generate_summary_table()

    # ★ 过滤 mean_dist_to_object 小于 0.04 的数据点 ★
    if "mean_dist_to_object" in df.columns:
        original_valid_count = df["mean_dist_to_object"].notna().sum()
        df.loc[df["mean_dist_to_object"] < 0.042, "mean_dist_to_object"] = np.nan
        filtered_valid_count = df["mean_dist_to_object"].notna().sum()
        removed_count = original_valid_count - filtered_valid_count
        if removed_count > 0:
            print(f"[*] 数据过滤: 已舍弃 {removed_count} 个 mean_dist_to_object < 0.04 的数据点。")

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

            pairwise_results.append({
                "Comparison": f"{m1} vs {m2}", "Metric": m_name, "P-Value": p,
                "Significant": "Yes" if p < 0.05 else "No", "Test Type": test_name,
                f"Mean_{m1}": d1.mean(), f"Mean_{m2}": d2.mean(),
            })

    results_df = pd.DataFrame(pairwise_results)

    # --- C. & D. 打印与保存数据 ---
    print("\n" + "=" * 80)
    print(results_df[["Comparison", "Metric", "P-Value", "Significant"]].to_string(index=False))
    print("=" * 80)

    with pd.ExcelWriter(OUTPUT_EXCEL) as writer:
        results_df.to_excel(writer, sheet_name="Pairwise_Significance", index=False)
        df.groupby("method")[[m for m in metrics.keys() if m in df.columns]].agg(["mean", "std"]).to_excel(writer, sheet_name="Raw_Stats_Summary")
    print(f"详细报告已保存至: {OUTPUT_EXCEL}")

    # --- E. 绘图与标注 ---
    print("\n正在生成 2x3 布局的可视化图表...")
    sns.set_theme(style="whitegrid")
    plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Liberation Sans", "sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False

    relevant_methods = sorted(list(set([m for pair in target_pairs for m in pair])))
    df_plot = df[df["method"].isin(relevant_methods)]

    # 创建 2x3 画布
    fig, axes = plt.subplots(2, 3, figsize=(24, 14))
    axes_flat = axes.flatten()

    for i, (m_key, m_name) in enumerate(metrics.items()):
        ax = axes_flat[i]
        
        # --- 核心修改：剔除散点图中的异常点 ---
        # 1. 为散点图生成一个专属的 filtered DataFrame
        strip_data_list = []
        for method in relevant_methods:
            df_m = df_plot[df_plot["method"] == method].copy()
            s = df_m[m_key].dropna()
            if not s.empty:
                # 计算四分位数和 IQR边界
                q1 = s.quantile(0.25)
                q3 = s.quantile(0.75)
                iqr = q3 - q1
                lower_bound = q1 - 1.5 * iqr
                upper_bound = q3 + 1.5 * iqr
                # 仅保留处于正常边界内的数据点
                df_m = df_m[(df_m[m_key] >= lower_bound) & (df_m[m_key] <= upper_bound)]
            strip_data_list.append(df_m)
            
        df_strip = pd.concat(strip_data_list)

        # 2. 绘制箱线图 (用全局数据，保证箱线计算准确；showfliers=False 隐藏本身的点)
        sns.boxplot(
            x="method", y=m_key, data=df_plot, order=relevant_methods,
            ax=ax, palette="Set3", hue="method", showfliers=False
        )

        if ax.get_legend():
            ax.get_legend().remove()
        ax.set_title(m_name, fontsize=16, fontweight="bold")
        ax.tick_params(axis="x", rotation=0)

        # --- 显著性标注逻辑 ---
        y_min, y_max = ax.get_ylim()
        y_range = y_max - y_min
        current_annot_y = y_max + 0.02 * y_range

        for m1, m2 in target_pairs:
            match = results_df[(results_df["Comparison"] == f"{m1} vs {m2}") & (results_df["Metric"] == m_name)]
            if match.empty:
                continue
            p_val = match.iloc[0]["P-Value"]
            sig_label = get_sig_label(p_val)
            
            if sig_label:
                idx1, idx2 = relevant_methods.index(m1), relevant_methods.index(m2)
                h = 0.02 * y_range
                # 绘制显著性连线
                ax.plot([idx1, idx1, idx2, idx2], [current_annot_y, current_annot_y + h, current_annot_y + h, current_annot_y], lw=1.2, c="k")
                # 绘制星号标注
                ax.text((idx1 + idx2) * 0.5, current_annot_y + h, sig_label, ha="center", va="bottom", color="k", fontsize=12)
                
                # 更新下一次标注的起始高度
                current_annot_y += 0.12 * y_range
                
        # 调整 Y 轴上限，以防止标注被顶出边界
        ax.set_ylim(y_min, current_annot_y + 0.05 * y_range)

    plt.tight_layout()
    plt.savefig(OUTPUT_IMAGE, dpi=300, bbox_inches='tight')
    print(f"成功保存综合图表: {OUTPUT_IMAGE}")
    
    plt.show()

if __name__ == "__main__":
    main()