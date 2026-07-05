"""第三章 YAML 配置读取工具。

默认参数由 `Ch3Config` 给出；本文件负责把 `config/ch3_*.yaml` 中的覆盖项
转换为强类型字段，并处理 NumPy 向量和表面分区列表。
"""

from pathlib import Path

import numpy as np

from .data_types import Ch3Config, SurfaceZone


def _try_yaml_load(path):
    try:
        import yaml  # type: ignore
    except Exception:
        return None
    with Path(path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_config(path=None):
    """读取第三章配置文件并返回 `Ch3Config`。"""

    cfg = Ch3Config()
    if path is None:
        return cfg
    data = _try_yaml_load(path)
    if not data:
        return cfg

    sim = data.get("simulation", {})
    cfg.dt = float(sim.get("dt", cfg.dt))
    cfg.scan_duration = float(sim.get("scan_duration", cfg.scan_duration))
    cfg.settle_time = float(sim.get("settle_time", cfg.settle_time))
    cfg.mass = float(sim.get("mass", cfg.mass))
    cfg.damping = float(sim.get("damping", cfg.damping))
    if "scan_start" in sim:
        cfg.scan_start = np.asarray(sim["scan_start"], dtype=float)
    if "scan_end" in sim:
        cfg.scan_end = np.asarray(sim["scan_end"], dtype=float)

    force = data.get("force", {})
    cfg.force_desired = float(force.get("desired", cfg.force_desired))
    cfg.force_min = float(force.get("min", cfg.force_min))
    cfg.force_max = float(force.get("max", cfg.force_max))

    surface = data.get("surface", {})
    cfg.surface_height = float(surface.get("height", cfg.surface_height))
    cfg.force_noise_std = float(surface.get("force_noise_std", cfg.force_noise_std))
    if "zones" in surface:
        zones = []
        for idx, item in enumerate(surface["zones"]):
            label = item.get("label", f"z{idx}_K{float(item['stiffness']):.0f}")
            zones.append(
                SurfaceZone(
                    float(item["x_min"]),
                    float(item["x_max"]),
                    float(item["stiffness"]),
                    float(item["damping"]),
                    label,
                )
            )
        cfg.zones = zones

    controllers = data.get("controllers", {})
    if "list" in controllers:
        cfg.controllers = tuple(str(v) for v in controllers["list"])
    cfg.repeats = int(controllers.get("repeats", cfg.repeats))
    cfg.random_seed = int(controllers.get("random_seed", cfg.random_seed))
    return cfg
