"""第三章名义扫描轨迹生成器。"""

import numpy as np

from .data_types import Ch3Config


class LinearScanReference:
    """从 `scan_start` 匀速运动到 `scan_end` 的线性扫描参考。"""

    """Fixed scan trajectory for Chapter 3 execution-layer experiments."""

    def __init__(self, cfg: Ch3Config):
        """初始化对象参数和运行状态。"""
        self.cfg = cfg
        self.start = cfg.scan_start.copy()
        self.end = cfg.scan_end.copy()
        self.delta = self.end - self.start

    def sample(self, t):
        """返回时刻 `t` 的期望位置和速度。"""

        s = float(np.clip(t / max(self.cfg.scan_duration, 1e-9), 0.0, 1.0))
        # Smooth endpoints reduce artificial acceleration at the first/last sample.
        h = s * s * (3.0 - 2.0 * s)
        dh = 6.0 * s * (1.0 - s) / max(self.cfg.scan_duration, 1e-9)
        pos = self.start + h * self.delta
        vel = dh * self.delta
        return pos, vel
