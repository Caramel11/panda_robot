"""第三章执行层控制器公共基类。

第三章的阻抗、混合力位、固定优先级、动态 GT 和 MPC/QP 基线都共享
位置 PD、接触刚度估计、力安全裕度和控制量限幅逻辑。本文件把这些公共逻辑
抽出，避免各控制器在核心评价指标上使用不同的基础实现。
"""

from dataclasses import dataclass

import numpy as np

from ..alpha_fp_analysis import force_safety_terms


@dataclass
class ControlOutput:
    """控制器单步输出。

    `u` 是任务空间加速度/等效输入，`alpha` 是力位仲裁系数，其他字段用于
    记录环境刚度估计、力安全裕度和积分状态。
    """

    u: np.ndarray
    alpha: float
    sigma_f: float
    k_hat: float
    rho_f: float
    s_f: float


class BaseCh3Controller:
    """第三章各类执行层控制器的最小接口。"""

    name = "base"

    def __init__(self, cfg):
        """初始化对象参数和运行状态。"""
        self.cfg = cfg
        self.sigma_f = 0.0
        self.k_hat = 350.0

    def reset(self):
        """重置或切换控制器/调度器的运行阶段状态。"""
        self.sigma_f = 0.0
        self.k_hat = 350.0

    def estimate_stiffness(self, contact):
        """基于接触力/压入量估计局部等效刚度。

        只在压入量足够大时更新，避免刚接触或离开表面时因分母过小导致估计跳变。
        """

        if contact.penetration > 5e-4:
            measured = contact.force / max(contact.penetration, 1e-6)
            self.k_hat = 0.98 * self.k_hat + 0.02 * measured
        return float(np.clip(self.k_hat, 50.0, 2000.0))

    def base_position_pd(self, pos, vel, pos_des, vel_des):
        """末端三维位置跟踪的公共 PD 项。"""

        kp = np.array([140.0, 100.0, 90.0])
        kd = np.array([38.0, 30.0, 24.0])
        return kp * (pos_des - pos) + kd * (vel_des - vel)

    def force_terms(self, contact):
        """计算力安全裕度 `rho_f` 和有符号力误差 `s_f`。"""

        rho, s_f, _, _ = force_safety_terms(
            contact.force,
            self.cfg.force_desired,
            self.cfg.force_min,
            self.cfg.force_max,
        )
        return rho, s_f

    def clamp(self, u):
        """限制任务空间输入，防止对照控制器在仿真中发散。"""

        return np.clip(np.asarray(u, dtype=float), [-6.0, -5.0, -5.0], [6.0, 5.0, 5.0])

    def compute(self, t, pos, vel, pos_des, vel_des, contact):
        """计算本模块对应的控制量、派生变量或实验评价指标。"""
        raise NotImplementedError
