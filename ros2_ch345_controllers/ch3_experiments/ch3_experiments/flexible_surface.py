"""第三章分段柔性接触环境模型。"""

import numpy as np

from .data_types import Ch3Config, ContactState


class PiecewiseKelvinVoigtSurface:
    """Piecewise Kelvin-Voigt contact surface used by Chapter 3 simulations."""

    def __init__(self, cfg: Ch3Config, rng=None):
        """初始化对象参数和运行状态。"""
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(cfg.random_seed)

    def zone_at(self, x):
        """返回当前位置所在的刚度分区。"""

        x = float(x)
        for idx, zone in enumerate(self.cfg.zones):
            if zone.x_min <= x < zone.x_max or (
                idx == len(self.cfg.zones) - 1 and x <= zone.x_max
            ):
                return idx, zone
        if x < self.cfg.zones[0].x_min:
            return 0, self.cfg.zones[0]
        return len(self.cfg.zones) - 1, self.cfg.zones[-1]

    def surface_height(self, _x, _y):
        """返回表面高度；当前实验中使用平面柔性表面。"""

        return float(self.cfg.surface_height)

    def compute_force(self, position, velocity):
        """根据末端位置和速度计算法向接触力。"""

        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        idx, zone = self.zone_at(position[0])
        height = self.surface_height(position[0], position[1])
        penetration = max(0.0, height - position[2])
        normal_speed = max(0.0, -velocity[2])
        force = zone.stiffness * penetration + zone.damping * normal_speed
        if self.cfg.force_noise_std > 0.0 and force > 0.0:
            force += float(self.rng.normal(0.0, self.cfg.force_noise_std))
        force = max(0.0, force)
        label = zone.label or f"z{idx}_K{zone.stiffness:.0f}"
        return ContactState(
            x=position.copy(),
            dx=velocity.copy(),
            force=float(force),
            k_env=float(zone.stiffness),
            b_env=float(zone.damping),
            penetration=float(penetration),
            zone_index=idx,
            zone_label=label,
        )
