"""第三章本文动态力位优先级控制器。

该控制器是第三章快速对照和实物噪声模拟中的“本文方法”入口。为避免
Python 快速仿真与 Gazebo/no-RCM 在线控制链出现两套 alpha 规律，本文件
现在直接复用 no-RCM 的完整 ``continuous_force_margin`` 调度器和 Pareto
力位混合增益表，只保留第三章仿真器所需的轻量接口。
"""

from .base import ControlOutput
from .fixed_gt import FixedGTController
from ..no_rcm_force_position import NoRcmContinuousForceMarginExecution


class DynamicGTController(FixedGTController):
    """带低通滤波的动态 `alpha_FP` 执行层控制器。"""

    name = "dynamic_gt"

    def __init__(self, cfg):
        """初始化对象参数和运行状态。"""
        super().__init__(cfg, alpha=0.5)
        self.name = "dynamic_gt"
        self.execution = NoRcmContinuousForceMarginExecution(
            dt=cfg.dt,
            force_desired=cfg.force_desired,
            force_min=cfg.force_min,
            force_max=cfg.force_max,
            control_scale=1.0,
            output_limit=6.0,
        )

    def reset(self):
        """重置或切换控制器/调度器的运行阶段状态。"""
        super().reset()
        self.execution.reset()

    def compute(self, t, pos, vel, pos_des, vel_des, contact):
        """计算当前步任务空间控制量。"""

        k_hat = self.estimate_stiffness(contact)
        out = self.execution.compute(
            pos=pos,
            vel=vel,
            pos_ref=pos_des,
            vel_ref=vel_des,
            force_norm=contact.force,
            K_hat=k_hat,
            force_blend=1.0,
            tracking_boost_enabled=True,
        )
        self.sigma_f = float(out.sigma_f[2])
        return ControlOutput(
            self.clamp(out.u),
            out.alpha,
            self.sigma_f,
            k_hat,
            out.rho_f,
            out.s_f,
        )
