"""第三章固定力位优先级基线控制器。"""

import numpy as np

from .base import BaseCh3Controller, ControlOutput


class FixedGTController(BaseCh3Controller):
    """固定 `alpha_FP` 的 GT 风格执行层控制器。"""

    def __init__(self, cfg, alpha=0.5):
        """初始化对象参数和运行状态。"""
        super().__init__(cfg)
        self.alpha = float(alpha)
        self.name = f"fixed_{self.alpha:.1f}"

    def compute(self, t, pos, vel, pos_des, vel_des, contact):
        """用固定 alpha 融合位置控制项和 z 向力控制项。"""

        k_hat = self.estimate_stiffness(contact)
        rho, s_f = self.force_terms(contact)
        ef = self.cfg.force_desired - contact.force
        self.sigma_f = float(np.clip(self.sigma_f + self.cfg.dt * ef, -0.8, 0.8))
        u_pos = self.base_position_pd(pos, vel, pos_des, vel_des)
        u_force_z = -1.40 * ef - 0.16 * self.sigma_f - 0.05 * vel[2]
        u = u_pos.copy()
        u[2] = self.alpha * u_pos[2] + (1.0 - self.alpha) * u_force_z
        return ControlOutput(self.clamp(u), self.alpha, self.sigma_f, k_hat, rho, s_f)
