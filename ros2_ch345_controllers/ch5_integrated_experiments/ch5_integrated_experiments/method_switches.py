"""第5章综合实验的方法开关表。

本模块只做一件事：把论文中的“对比方法名称”转换为底层 ROS2 控制器
能够理解的参数组合。第5章不是重新实现控制律，而是把：

1. 第4章参考层仲裁参数 `arbitration_strategy`；
2. 第3章执行层力/位置优先级参数 `execution_strategy`；
3. 固定权重基线 `fixed_alpha_hr`、`fixed_alpha_fp`；

统一封装为一个 `MethodSwitch`。这样批量实验脚本只需要传入
`full_method`、`reference_only` 等论文方法名，就能稳定复现实验矩阵。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class MethodSwitch:
    """一个第5章方法对应的一组控制器参数。

    Attributes:
        name: 论文和结果表格中显示的方法名。
        arbitration_strategy: 第4章参考层策略，决定 `alpha_HR` 和安全投影是否启用。
        execution_strategy: 第3章执行层策略，决定 `alpha_FP` 是否动态调整。
        fixed_alpha_hr: 固定参考层权重，仅在 fixed_blend 等基线中使用。
        fixed_alpha_fp: 固定执行层权重，仅在 fixed_02/fixed_05/fixed_08 中使用。
    """

    name: str
    arbitration_strategy: str
    execution_strategy: str
    fixed_alpha_hr: float = 0.5
    fixed_alpha_fp: float = 0.5


# 方法矩阵说明：
# - balanced_fixed：第3/4章都退化为固定权重，是最基础基线。
# - reference_only：只启用第4章参考层动态仲裁，执行层保持固定 alpha_FP。
# - execution_only：只启用第3章 continuous_force_margin 执行层，参考层不接收人类修正。
# - full_method：第3章执行层和第4章参考层都启用，是第5章主方法。
# - no_projection：关闭第4章安全投影，用于证明安全投影不可删除。
# - standard_impedance / traditional_hybrid / standard_mpc：保持第4章完整参考层，
#   只替换第3章执行层控制律，用于证明完整方法的执行层不是普通阻抗、
#   传统混合力位或标准 MPC 可直接替代。
METHODS = {
    "direct_accept": MethodSwitch("direct_accept", "direct_accept", "fixed_05", 1.0, 0.5),
    "standard_impedance": MethodSwitch("standard_impedance", "full_method", "standard_impedance", 0.5, 1.0),
    "traditional_hybrid": MethodSwitch("traditional_hybrid", "full_method", "traditional_hybrid", 0.5, 0.0),
    "standard_mpc": MethodSwitch("standard_mpc", "full_method", "standard_mpc", 0.5, 0.5),
    "strong_position": MethodSwitch("strong_position", "fixed_blend", "fixed_08", 0.5, 0.8),
    "balanced_fixed": MethodSwitch("balanced_fixed", "fixed_blend", "fixed_05", 0.5, 0.5),
    "strong_force": MethodSwitch("strong_force", "fixed_blend", "fixed_02", 0.5, 0.2),
    "reference_only": MethodSwitch("reference_only", "full_method", "fixed_05", 0.5, 0.5),
    "execution_only": MethodSwitch("execution_only", "autonomous_only", "continuous_force_margin", 0.0, 0.5),
    "full_method": MethodSwitch("full_method", "full_method", "continuous_force_margin", 0.5, 0.5),
    "no_projection": MethodSwitch("no_projection", "dynamic_no_projection_open", "continuous_force_margin", 0.5, 0.5),
    "fixed_hr": MethodSwitch("fixed_hr", "fixed_blend", "continuous_force_margin", 0.5, 0.5),
    "fixed_fp": MethodSwitch("fixed_fp", "full_method", "fixed_05", 0.5, 0.5),
    # RCM-specific copied methods.  They preserve the same comparison semantics
    # as the base methods, but the controller node maps the *_rcm suffix to
    # RCM-tuned gains, force feedback and arbitration parameters.
    "rcm_direct_accept": MethodSwitch("rcm_direct_accept", "direct_accept_rcm", "fixed_05_rcm", 1.0, 0.5),
    "rcm_standard_impedance": MethodSwitch("rcm_standard_impedance", "full_method_rcm", "standard_impedance_rcm", 0.5, 1.0),
    "rcm_traditional_hybrid": MethodSwitch("rcm_traditional_hybrid", "full_method_rcm", "traditional_hybrid_rcm", 0.5, 0.0),
    "rcm_standard_mpc": MethodSwitch("rcm_standard_mpc", "full_method_rcm", "standard_mpc_rcm", 0.5, 0.5),
    "rcm_strong_position": MethodSwitch("rcm_strong_position", "fixed_blend_rcm", "fixed_08_rcm", 0.5, 0.8),
    "rcm_balanced_fixed": MethodSwitch("rcm_balanced_fixed", "fixed_blend_rcm", "fixed_05_rcm", 0.5, 0.5),
    "rcm_strong_force": MethodSwitch("rcm_strong_force", "fixed_blend_rcm", "fixed_02_rcm", 0.5, 0.2),
    "rcm_reference_only": MethodSwitch("rcm_reference_only", "full_method_rcm", "fixed_05_rcm", 0.5, 0.5),
    "rcm_execution_only": MethodSwitch("rcm_execution_only", "autonomous_only_rcm", "continuous_force_margin_rcm", 0.0, 0.5),
    "rcm_no_projection": MethodSwitch("rcm_no_projection", "dynamic_no_projection_open_rcm", "continuous_force_margin_rcm", 0.5, 0.5),
    "rcm_fixed_hr": MethodSwitch("rcm_fixed_hr", "fixed_blend_rcm", "continuous_force_margin_rcm", 0.5, 0.5),
    "rcm_fixed_fp": MethodSwitch("rcm_fixed_fp", "full_method_rcm", "fixed_05_rcm", 0.5, 0.5),
    "rcm_full_method": MethodSwitch("rcm_full_method", "full_method_rcm", "continuous_force_margin_rcm", 0.5, 0.5),
}


# 默认 Gazebo 在线实验方法。这里故意不把所有方法都放进默认列表，
# 是为了控制单次在线仿真的时间；安全投影消融 no_projection 可按需补跑。
DEFAULT_GAZEBO_METHODS = ["balanced_fixed", "reference_only", "execution_only", "full_method"]
DEFAULT_SIM_METHODS = [
    "standard_impedance",
    "traditional_hybrid",
    "standard_mpc",
    "strong_position",
    "balanced_fixed",
    "strong_force",
    "reference_only",
    "execution_only",
    "full_method",
]


def resolve_methods(names):
    """把命令行传入的方法名解析成 `MethodSwitch` 列表。

    Args:
        names: 字符串列表，通常来自 `--methods a,b,c` 的逗号切分结果。

    Returns:
        与输入顺序一致的 `MethodSwitch` 列表。

    Raises:
        ValueError: 方法名不在 `METHODS` 中时抛出，并列出全部合法方法。
    """

    out = []
    for name in names:
        key = name.strip()
        if not key:
            continue
        if key not in METHODS:
            valid = ", ".join(sorted(METHODS))
            raise ValueError(f"Unknown Chapter 5 method '{key}'. Valid methods: {valid}")
        out.append(METHODS[key])
    return out
