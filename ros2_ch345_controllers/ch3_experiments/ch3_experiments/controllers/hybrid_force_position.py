"""第三章传统混合力/位置控制基线。"""

import numpy as np

from .base import BaseCh3Controller, ControlOutput


class HybridForcePositionController(BaseCh3Controller):
    """切向位置控制、法向力控制的混合控制器。"""

    name = "hybrid"

    def compute(self, t, pos, vel, pos_des, vel_des, contact):
        """计算混合力位控制输入。"""

        k_hat = self.estimate_stiffness(contact)
        rho, s_f = self.force_terms(contact)
        dt = self.cfg.dt
        ef = self.cfg.force_desired - contact.force
        self.sigma_f = float(np.clip(self.sigma_f + dt * ef, -0.8, 0.8))
        u = self.base_position_pd(pos, vel, pos_des, vel_des)
        # In z, negative u moves down and increases contact force.
        u[2] = -1.15 * ef - 0.18 * self.sigma_f - 0.05 * vel[2]
        return ControlOutput(self.clamp(u), 0.0, self.sigma_f, k_hat, rho, s_f)
