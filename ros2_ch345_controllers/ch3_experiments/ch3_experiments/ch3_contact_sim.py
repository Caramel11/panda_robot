"""第三章纯 Python 柔性接触仿真入口。

该仿真不依赖 ROS/Gazebo，用质量-阻尼点模型和分段 Kelvin-Voigt 接触模型
快速比较各类执行层控制器。它主要用于参数预筛、论文对照数据生成和 CI 式
快速验证；最终在线效果仍以 Gazebo 结果为准。
"""

import time

import numpy as np

from .ch3_logger import Ch3Logger
from .controllers import (
    DynamicGTController,
    FixedGTController,
    HybridForcePositionController,
    ImpedanceController,
    MpcQpController,
)
from .flexible_surface import PiecewiseKelvinVoigtSurface
from .reference_trajectory import LinearScanReference


def make_controller(name, cfg):
    """根据论文方法名实例化对应控制器。"""

    name = str(name)
    if name in ("impedance", "standard_impedance"):
        controller = ImpedanceController(cfg)
        controller.name = "standard_impedance" if name == "standard_impedance" else controller.name
        return controller
    if name in ("hybrid", "traditional_hybrid", "hybrid_force_position"):
        controller = HybridForcePositionController(cfg)
        controller.name = "traditional_hybrid" if name == "traditional_hybrid" else controller.name
        return controller
    if name == "fixed_02":
        return FixedGTController(cfg, 0.2)
    if name == "fixed_05":
        return FixedGTController(cfg, 0.5)
    if name == "fixed_08":
        return FixedGTController(cfg, 0.8)
    if name in ("dynamic_gt", "continuous_force_margin", "no_rcm_continuous_force_margin", "full_method"):
        controller = DynamicGTController(cfg)
        controller.name = name
        return controller
    if name in ("mpc_qp", "standard_mpc"):
        controller = MpcQpController(cfg)
        controller.name = "standard_mpc" if name == "standard_mpc" else controller.name
        return controller
    raise ValueError(f"unknown Chapter 3 controller: {name}")


def run_contact_simulation(cfg, controller_name, trial_index=0, output_file=None):
    """运行一次第三章柔性接触扫描仿真。

    Args:
        cfg: `Ch3Config`，包含表面、扫描轨迹和控制器参数。
        controller_name: 对照方法名，如 `dynamic_gt` 或 `fixed_05`。
        trial_index: 重复试验编号，用于扰动随机种子。
        output_file: 若给出，则把时序数据保存为 NPZ。

    Returns:
        以字典形式返回的时序数组，可直接传给指标和绘图脚本。
    """

    rng = np.random.default_rng(cfg.random_seed + int(trial_index))
    surface = PiecewiseKelvinVoigtSurface(cfg, rng=rng)
    reference = LinearScanReference(cfg)
    controller = make_controller(controller_name, cfg)
    controller.reset()

    # Start close to the desired contact force to keep the comparison focused on
    # execution-layer tracking, not impact transients.
    k0 = cfg.zones[0].stiffness
    pos = cfg.scan_start.copy()
    pos[2] = cfg.surface_height - cfg.force_desired / k0
    vel = np.zeros(3)

    logger = Ch3Logger()
    n_steps = int(round(cfg.scan_duration / cfg.dt)) + 1
    for i in range(n_steps):
        t = i * cfg.dt
        pos_des, vel_des = reference.sample(t)
        contact = surface.compute_force(pos, vel)
        compute_t0 = time.perf_counter()
        out = controller.compute(t, pos, vel, pos_des, vel_des, contact)
        compute_ms = (time.perf_counter() - compute_t0) * 1000.0

        acc = (out.u - cfg.damping * vel) / max(cfg.mass, 1e-9)
        vel = vel + cfg.dt * acc
        pos = pos + cfg.dt * vel

        pos_err = pos - pos_des
        force_err = contact.force - cfg.force_desired
        logger.add(
            t=t,
            pos_x=pos[0],
            pos_y=pos[1],
            pos_z=pos[2],
            pos_des_x=pos_des[0],
            pos_des_y=pos_des[1],
            pos_des_z=pos_des[2],
            pos_err_x=pos_err[0],
            pos_err_y=pos_err[1],
            pos_err_z=pos_err[2],
            vel_x=vel[0],
            vel_y=vel[1],
            vel_z=vel[2],
            F_measured=contact.force,
            F_desired=cfg.force_desired,
            F_err=force_err,
            F_min=cfg.force_min,
            F_max=cfg.force_max,
            rho_F=out.rho_f,
            s_F=out.s_f,
            alpha=out.alpha,
            alpha_x=1.0,
            alpha_y=1.0,
            alpha_z=out.alpha,
            K_hat=out.k_hat,
            K_env_true=contact.k_env,
            B_env_true=contact.b_env,
            stiffness_zone_index=contact.zone_index,
            stiffness_zone_label=contact.zone_label,
            sigma_f=out.sigma_f,
            u_x=out.u[0],
            u_y=out.u[1],
            u_z=out.u[2],
            u_norm=float(np.linalg.norm(out.u)),
            scan_vx_eff=cfg.scan_speed,
            compute_time_ms=compute_ms,
            arbitration_strategy=controller.name,
            ori_err_deg=0.0,
            front_axis_err_deg=0.0,
            phase=2,
        )

    if output_file:
        logger.save_npz(output_file)
    return logger.as_arrays()
