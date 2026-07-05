"""第三章增广力位状态空间模型。

该模型用于文档、增益检查和论文方法说明，不直接替代在线 Gazebo 控制器。
状态通常写为 `[e_p, e_v, e_f, sigma_f]`，用于说明位置误差、速度误差、
力误差和力误差积分项之间的耦合关系。
"""

import numpy as np


class AugmentedForcePositionModel:
    """Small Chapter 3 augmented model used for documentation and gain checks."""

    def __init__(self, mass=10.0, damping=300.0, kv=100.0, eps_r=1.0, eps_f=2.0):
        """初始化对象参数和运行状态。"""
        self.mass = float(mass)
        self.damping = float(damping)
        self.kv = float(kv)
        self.eps_r = float(eps_r)
        self.eps_f = float(eps_f)

    def build_state(self, ep, ev, ef, sigma_f):
        """把标量误差整理成增广状态向量。"""

        return np.array([float(ep), float(ev), float(ef), float(sigma_f)], dtype=float)

    def update_sigma_f(self, sigma_f, ef, dt):
        """更新带泄漏的力误差积分状态。"""

        return float(sigma_f + dt * (ef - self.eps_f * sigma_f))

    def linear_matrices(self, k_hat, b_hat):
        """根据当前刚度/阻尼估计生成线性化 A、B 矩阵。"""

        m = self.mass
        c = self.damping
        beta = float(b_hat) / m
        kappa = float(k_hat) - float(b_hat) * c / m
        a = np.array(
            [
                [-self.eps_r, 1.0, 0.0, 0.0],
                [-self.kv / m, -c / m, 1.0 / m, 0.0],
                [0.0, -kappa, -beta, 0.0],
                [0.0, 0.0, 1.0, -self.eps_f],
            ],
            dtype=float,
        )
        b = np.array([[0.0], [1.0 / m], [-beta], [0.0]], dtype=float)
        return a, b
