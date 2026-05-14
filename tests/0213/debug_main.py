import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager
import os
from fuzzy_logic import FuzzyLogicTool
import time


# -------------------------- Ubuntu中文配置（强制加载字体） --------------------------
def setup_chinese_font_ubuntu():
    font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
    if not os.path.exists(font_path):
        print(f"❌ 字体文件不存在：{font_path}")
        print("👉 请先执行：sudo apt-get install -y fonts-wqy-zenhei")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False
        return
    try:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams["font.sans-serif"] = [font_prop.get_name(), "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        print(f"✅ 中文配置成功，使用字体：{font_prop.get_name()}")
    except Exception as e:
        print(f"⚠️  中文配置警告：{e}")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False


setup_chinese_font_ubuntu()

# 1. 初始化优化后的λ-based模糊逻辑工具
lambda_fuzzy = FuzzyLogicTool(type="lambda_based")

# 2. 3组测试场景
test_cases = [
    {"name": "场景1（机器人主导）", "inputs": [5, 0.8, 0.05]},
    {"name": "场景2（人机协同）", "inputs": [0, 0, 0]},
    {"name": "场景3（人主导）", "inputs": [2.5, 0.2, 0.02]},
]

# 3. 绘图配置：3个场景 × 5个子图（F_h/V_h/D_r隶属度 + 隶属度和 + 解模糊结果）
fig, axes = plt.subplots(3, 5, figsize=(20, 12))
fig.suptitle(
    "λ-based模糊逻辑调试：三输入隶属度+隶属度和+解模糊结果",
    fontsize=16,
    fontweight="bold",
)

# 输入配置（名称+范围+模糊集标签）
input_configs = [
    (
        "F_h (N)",
        np.linspace(0.0, 2, 200),
        ["PS", "PM", "PL"],
        lambda_fuzzy.input_sets[0],
    ),
    (
        "V_h (cm/s)",
        np.linspace(0.0, 0.12, 200),
        ["PS", "PM", "PL"],
        lambda_fuzzy.input_sets[1],
    ),
    (
        "D_r (cm)",
        np.linspace(0.0, 0.1, 200),
        ["PS", "PM", "PL"],
        lambda_fuzzy.input_sets[2],
    ),
]

# 4. 逐场景绘制（核心逻辑）
for case_idx, case in enumerate(test_cases):
    # start_time = time.time()
    inputs = case["inputs"]  # [F_h, V_h, D_r]
    case_name = case["name"]

    # 4.1 计算模糊逻辑输出
    lambda_w = lambda_fuzzy.compute(inputs)
    print(
        f"{case_name}：输入(F_h={inputs[0]}, V_h={inputs[1]}, D_r={inputs[2]}) → 输出λ_w={lambda_w:.3f}"
    )
    # end_time = time.time()

    # 计算耗时（单位：秒）
    # elapsed_time = end_time - start_time
    # print(f"代码运行时间：{elapsed_time:.6f} 秒")

    # 4.2 绘制3个输入的隶属度曲线（前3个子图）
    for input_idx, (input_label, input_range, fuzzy_labels, fuzzy_sets) in enumerate(
        input_configs
    ):
        ax = axes[case_idx, input_idx]

        # 绘制每个模糊集的隶属度曲线
        memberships_sum = np.zeros_like(input_range)  # 初始化隶属度和
        for label in fuzzy_labels:
            params = fuzzy_sets[label]
            if label == "PS" or label == "N":
                memberships = [
                    lambda_fuzzy.trapezoidal_mf_type1(x, params) for x in input_range
                ]
            elif label == "PL" or label == "P":
                memberships = [
                    lambda_fuzzy.trapezoidal_mf_type2(x, params) for x in input_range
                ]
            else:
                memberships = [
                    lambda_fuzzy.triangular_mf(x, params) for x in input_range
                ]
            memberships_sum += np.array(memberships)  # 累加隶属度和
            ax.plot(input_range, memberships, linewidth=2, label=f"{label}（{params}）")

        # 标注测试输入
        test_input = inputs[input_idx]
        ax.axvline(
            x=test_input,
            color="red",
            linestyle="--",
            linewidth=2,
            label=f"测试输入={test_input}",
        )

        # 子图配置
        ax.set_xlabel(input_label)
        ax.set_ylabel("隶属度 μ")
        ax.set_title(f"{case_name}\n{input_label}隶属度")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim([-0.05, 1.05])

    # 4.3 绘制隶属度和曲线（第4个子图）
    ax_sum = axes[case_idx, 3]
    for input_idx, (input_label, input_range, fuzzy_labels, fuzzy_sets) in enumerate(
        input_configs
    ):
        # 计算当前输入的隶属度和
        memberships_sum = np.zeros_like(input_range)
        for label in fuzzy_labels:
            params = fuzzy_sets[label]
            if label == "PS" or label == "N":
                memberships = [
                    lambda_fuzzy.trapezoidal_mf_type1(x, params) for x in input_range
                ]
            elif label == "PL" or label == "P":
                memberships = [
                    lambda_fuzzy.trapezoidal_mf_type2(x, params) for x in input_range
                ]
            else:
                memberships = [
                    lambda_fuzzy.triangular_mf(x, params) for x in input_range
                ]
            memberships_sum += np.array(memberships)
        # 绘制
        ax_sum.plot(input_range, memberships_sum, linewidth=2, label=f"{input_label}")

    # 标注测试输入的隶属度和
    for input_idx, (_, _, _, fuzzy_sets) in enumerate(input_configs):
        test_input = inputs[input_idx]
        # 计算测试输入的隶属度和
        sum_val = 0.0
        for label in fuzzy_labels:
            params = fuzzy_sets[label]
            if label == "PS" or label == "N":
                sum_val += lambda_fuzzy.trapezoidal_mf_type1(test_input, params)
            elif label == "PL" or label == "P":
                sum_val += lambda_fuzzy.trapezoidal_mf_type2(test_input, params)
            else:
                sum_val += lambda_fuzzy.triangular_mf(test_input, params)
        ax_sum.scatter(
            test_input,
            sum_val,
            color=f"C{input_idx}",
            s=50,
            zorder=5,
            label=f"{input_configs[input_idx][0]}测试点和={sum_val:.3f}",
        )

    # 子图配置
    ax_sum.set_xlabel("输入值")
    ax_sum.set_ylabel("隶属度和 Σμ")
    ax_sum.set_title(f"{case_name}\n三输入隶属度和分布")
    ax_sum.axhline(
        y=1.0, color="black", linestyle="--", alpha=0.5, label="Σμ=1.0（参考线）"
    )
    ax_sum.legend(fontsize=8)
    ax_sum.grid(True, alpha=0.3)
    ax_sum.set_ylim([-0.05, 1.5])  # 允许略大于1（模糊集重叠正常）

    # 4.4 绘制解模糊结果（第5个子图）
    ax_defuzz = axes[case_idx, 4]
    lambda_range = np.linspace(0.0, 1.0, 100)
    for out_label, out_params in lambda_fuzzy.output_sets.items():
        if out_label == "PL":
            out_memberships = [
                lambda_fuzzy.trapezoidal_mf_type2(x, out_params) for x in lambda_range
            ]
        elif out_label == "Z":
            out_memberships = [
                lambda_fuzzy.trapezoidal_mf_type1(x, out_params) for x in lambda_range
            ]
        else:
            out_memberships = [
                lambda_fuzzy.triangular_mf(x, out_params) for x in lambda_range
            ]
        ax_defuzz.plot(
            lambda_range,
            out_memberships,
            linewidth=2,
            label=f"{out_label}（{out_params}）",
        )
    ax_defuzz.axvline(
        x=lambda_w,
        color="red",
        linestyle="--",
        linewidth=2,
        label=f"解模糊结果={lambda_w:.3f}",
    )
    ax_defuzz.set_xlabel("仲裁系数 λ")
    ax_defuzz.set_ylabel("隶属度 μ")
    ax_defuzz.set_title(
        f"{case_name}\n解模糊结果（预期≈{0.9 if case_idx==0 else 0.3 if case_idx==1 else 0.1}）"
    )
    ax_defuzz.legend(fontsize=8)
    ax_defuzz.grid(True, alpha=0.3)
    ax_defuzz.set_ylim([-0.05, 1.05])

# 调整子图间距
plt.tight_layout()
plt.savefig("step1_lambda_based_optimized.png", dpi=300, bbox_inches="tight")
plt.show()

# 额外验证：打印每个输入有效范围内的隶属度和统计
print("\n=== 隶属度和统计（有效输入范围）===")
for input_label, input_range, fuzzy_labels, fuzzy_sets in input_configs:
    memberships_sum = np.zeros_like(input_range)
    for label in fuzzy_labels:
        params = fuzzy_sets[label]
        if label == "PS" or label == "N":
            memberships = [
                lambda_fuzzy.trapezoidal_mf_type1(x, params) for x in input_range
            ]
        elif label == "PL" or label == "P":
            memberships = [
                lambda_fuzzy.trapezoidal_mf_type2(x, params) for x in input_range
            ]
        else:
            memberships = [lambda_fuzzy.triangular_mf(x, params) for x in input_range]
        memberships_sum += np.array(memberships)
    # memberships_sum += np.array(
    #     [lambda_fuzzy.triangular_mf(x, params) for x in input_range]
    # )
    print(f"{input_label}：")
    print(f"  隶属度和范围：[{memberships_sum.min():.3f}, {memberships_sum.max():.3f}]")
    print(f"  隶属度和平均值：{memberships_sum.mean():.3f}")
    print(
        f"  无隶属度（和=0）的点数：{np.sum(memberships_sum < 1e-6)}（总点数：{len(input_range)}）"
    )
