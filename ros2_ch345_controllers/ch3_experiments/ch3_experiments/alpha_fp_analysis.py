"""第三章执行层 `alpha_FP` 仲裁规律。

本文件是动态力位优先级的理论到代码映射：输入为接触力、期望力、安全边界
和刚度估计，输出为执行层位置优先级 `alpha_FP`。值越大越偏位置跟踪，
值越小越偏力安全和力跟踪。
"""

import numpy as np


def force_safety_terms(force, force_desired, force_min, force_max):
    """计算力安全裕度和有符号归一化误差。

    `rho` 越接近 1 表示越远离安全边界，越接近 0 表示越靠近上下限；
    `signed` 表示当前力相对期望力的方向，供 alpha 规律区分欠压和过压。
    """

    width = max(float(force_max) - float(force_min), 1e-9)
    half_width = max(0.5 * width, 1e-9)
    lower = float(force) - float(force_min)
    upper = float(force_max) - float(force)
    rho = float(np.clip(min(lower, upper) / half_width, 0.0, 1.0))
    signed = float(np.clip((float(force) - float(force_desired)) / half_width, -1.0, 1.0))
    return rho, signed, lower, upper


def dynamic_alpha_fp(force, force_desired, force_min, force_max, k_hat, k_low=300.0, k_high=500.0):
    """Continuous Chapter 3 force-position priority.

    alpha_FP is high when the contact is safe and stiffness is high, because the
    controller can afford stronger tangential/path tracking. It decreases near
    force safety boundaries or in softer material where force error grows more
    quickly for the same z motion.
    """
    rho, signed, _, _ = force_safety_terms(force, force_desired, force_min, force_max)
    k_blend = np.clip((float(k_hat) - k_low) / max(k_high - k_low, 1e-9), 0.0, 1.0)
    stiffness_alpha = 0.28 + 0.34 * k_blend
    boundary_guard = 0.30 * (1.0 - rho)
    force_bias = -0.10 * np.tanh(2.0 * signed)
    alpha = stiffness_alpha - boundary_guard + force_bias
    return float(np.clip(alpha, 0.14, 0.72)), rho, signed
