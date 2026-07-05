"""第三章纯阻抗/位置优先基线控制器。"""

from .base import BaseCh3Controller, ControlOutput


class ImpedanceController(BaseCh3Controller):
    """主要跟踪位置，仅用轻微力反馈修正 z 向输入。"""

    name = "impedance"

    def compute(self, t, pos, vel, pos_des, vel_des, contact):
        """计算阻抗控制输入。"""

        k_hat = self.estimate_stiffness(contact)
        rho, s_f = self.force_terms(contact)
        u = self.base_position_pd(pos, vel, pos_des, vel_des)
        # Pure impedance does not directly close the force loop.
        u[2] += 0.20 * (contact.force - self.cfg.force_desired)
        return ControlOutput(self.clamp(u), 1.0, self.sigma_f, k_hat, rho, s_f)
