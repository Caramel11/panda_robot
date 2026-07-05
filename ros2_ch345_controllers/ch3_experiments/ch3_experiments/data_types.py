"""第三章实验的核心数据结构。

本文件集中定义第三章柔性接触扫描实验中会反复使用的配置、环境分区、
接触状态和控制器中间量。把这些对象放在一个文件中，可以保证纯数值仿真、
Gazebo 在线实验、指标分析和绘图脚本使用同一套参数含义。
"""

from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class SurfaceZone:
    """柔性表面的一段分区。"""

    x_min: float
    x_max: float
    stiffness: float
    damping: float
    label: str = ""


@dataclass
class Ch3Config:
    """第三章默认实验参数。

    默认值来自当前论文实验代码的稳定配置：扫描时间 32 s，期望接触力 1 N，
    两段刚度分别为 300 N/m 和 500 N/m。对照实验脚本会从 YAML 覆盖这些字段。
    """

    dt: float = 0.01
    scan_duration: float = 32.0
    settle_time: float = 1.5
    scan_start: np.ndarray = field(
        default_factory=lambda: np.array([0.40, 0.00, 0.298], dtype=float)
    )
    scan_end: np.ndarray = field(
        default_factory=lambda: np.array([0.48, 0.00, 0.298], dtype=float)
    )
    mass: float = 6.0
    damping: float = 18.0
    force_desired: float = 1.0
    force_min: float = 0.66
    force_max: float = 1.38
    surface_height: float = 0.3013
    force_noise_std: float = 0.004
    zones: List[SurfaceZone] = field(
        default_factory=lambda: [
            SurfaceZone(0.40, 0.44, 300.0, 12.0, "soft_K300"),
            SurfaceZone(0.44, 0.48, 500.0, 22.0, "stiff_K500"),
        ]
    )
    repeats: int = 3
    controllers: Tuple[str, ...] = (
        "impedance",
        "hybrid",
        "fixed_02",
        "fixed_05",
        "fixed_08",
        "dynamic_gt",
        "mpc_qp",
    )
    random_seed: int = 7

    @property
    def scan_speed(self) -> float:
        """根据起止点和扫描时长计算名义切向扫描速度。"""

        dist = float(np.linalg.norm(self.scan_end - self.scan_start))
        return dist / max(self.scan_duration, 1e-9)


@dataclass
class ContactState:
    """单个仿真时刻的接触环境状态。"""

    x: np.ndarray
    dx: np.ndarray
    force: float
    k_env: float
    b_env: float
    penetration: float
    zone_index: int
    zone_label: str


@dataclass
class ControllerState:
    """记录控制器内部变量，主要用于调参分析和论文绘图。"""

    ep: np.ndarray
    ev: np.ndarray
    ef: float
    sigma_f: float
    k_hat: float
    rho_f: float
    alpha_fp: float
    u_task: np.ndarray


def as_array3(values: Sequence[float]) -> np.ndarray:
    """把 YAML/命令行中的三维列表安全转换为 NumPy 三维向量。"""

    arr = np.asarray(values, dtype=float).reshape(3)
    return arr.copy()
