"""第三章 MPC/QP 风格约束基线控制器。"""

import numpy as np

from .base import BaseCh3Controller, ControlOutput


class MpcQpController(BaseCh3Controller):
    """Lightweight constrained baseline that mimics MPC/QP saturation behavior."""

    name = "mpc_qp"

    def compute(self, t, pos, vel, pos_des, vel_des, contact):
        """用预测接触力的简单约束逻辑修正 z 向控制量。"""

        k_hat = self.estimate_stiffness(contact)
        rho, s_f = self.force_terms(contact)
        ef = self.cfg.force_desired - contact.force
        self.sigma_f = float(np.clip(self.sigma_f + self.cfg.dt * ef, -0.5, 0.5))
        u = self.base_position_pd(pos, vel, pos_des, vel_des)
        predicted_force = contact.force + 0.18 * (-u[2])
        if predicted_force > self.cfg.force_max:
            u[2] = min(u[2], 0.0) + 0.8 * (predicted_force - self.cfg.force_max)
        elif predicted_force < self.cfg.force_min:
            u[2] = max(u[2], 0.0) - 0.8 * (self.cfg.force_min - predicted_force)
        else:
            u[2] += -0.75 * ef - 0.10 * self.sigma_f
        return ControlOutput(self.clamp(u), 0.5, self.sigma_f, k_hat, rho, s_f)
