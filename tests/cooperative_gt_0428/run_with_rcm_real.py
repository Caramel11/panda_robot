#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
主程序 A: RCM 约束下的合作博弈力-位扫描实验
=============================================

控制链路:
  泄漏积分更新 → 4D-ARE 查表 → u_tool → RCM 杠杆 → τ

数学模型:
  M·ẍ + C·ẋ + K_v·(x−x_r) = u + f_ext      (含 K_v 基线刚度)
  ė_r1 = e_r2 − ε_r · e_r1                  (泄漏积分器)
  ė_f  = -(K_e − B_e·C/M)·e_r2 − B_e/M·e_f − B_e/M·u
  σ̇_f  = e_f − ε_f · σ_f                    (泄漏积分器)

用法:
  Terminal 1: roslaunch panda_simulator simulation.launch
  Terminal 2: python run_with_rcm_real.py --strategy all --trials 3
"""
import argparse
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rospy
from panda_robot import PandaArm, PandaKinematics

from src.gt_controller import CooperativeGameController
from src.alpha_scheduler_gt import (
    PhaseAwareFuzzyAlphaScheduler, ForceMarginFuzzyAlphaScheduler,
    FixedAlphaScheduler
)
from src.env_estimator import EnvironmentEstimator
from src.utils import (
    VirtualStiffnessSurface, DataLogger,
    FirstOrderLowPass, VectorRateLimiter,
)
from src.force_sensor_direct import DirectForceSensorInput
from src.leaky_integrator import LeakyIntegrator
from src.robot_interface import (
    update_robot_state, safe_move_to_joint_position,
    compute_torque_with_rcm,
)


# ================================================================
# 实验配置
# ================================================================
class Config:
    # 由 generate_rcm_safe_init.py 基于 tool=[0.3, 0, 0.1] 和 RCM 约束生成
    init_joints = (
        -0.02452756,
        -0.77878299,
        0.01473703,
        -2.26095140,
        0.01039203,
        1.48222746,
        0.77227957,
    )

    # 扫描轨迹 (tool tip)
    scan_start_x = 0.31
    scan_end_x   = 0.34
    scan_y       = 0.0
    scan_z       = 0.095
    approach_z   = 0.09
    max_surface_penetration = 0.005
    scan_vx      = 0.001
    approach_vz  = 0.0001
    approach_surface_margin = 0.0005
    approach_timeout_s = 180.0

    # 力控
    F_desired    = 1.0
    F_min        = 0.1
    F_max        = 2.0
    force_axis   = 2

    # RCM
    trocar_position = np.array([0.3, 0, 0.235])
    tool_length = 0.525

    # 变刚度区
    # stiffness_zones = [
    #     (0.25, 0.29, 500,  5),    # 软
    #     (0.29, 0.33, 5000, 50),   # 硬
    #     (0.33, 0.37, 80,   2),    # 极软
    #     (0.37, 0.41, 500,  5),    # 软
    # ]

    stiffness_zones = [
        (0.30, 0.325, 500, 1),    # 0.5 N/mm
        (0.325, 0.35, 500, 1),    # 0.5 N/mm
    ]

    # 控制频率 (文档要求 1kHz; 仿真可用 100Hz)
    ctrl_rate = 100
    dt = 1.0 / ctrl_rate

    # 泄漏积分器 (标量)
    eps_r = 1.0
    eps_f = 2.0

    settle_time = 2.0

    # 实物实验安全约束
    contact_force_blend_time = 1.0
    force_filter_tau = 0.04
    stiffness_filter_tau = 0.12
    torque_rate_limit = 80.0
    torque_abs_limit = 35.0
    torque_norm_limit = 70.0
    force_abs_limit = 20.0
    oscillation_window_s = 0.5
    force_osc_std_limit = 2
    torque_osc_std_norm_limit = 12.0
    loop_warn_period = 0.08


def stop_for_safety(robot, reason: str) -> bool:
    rospy.logerr(f"  SAFETY STOP: {reason}")
    try:
        robot.exec_position_cmd(robot.angles())
    except Exception as exc:
        rospy.logerr(f"  Failed to hold current joint position after safety stop: {exc}")
    return False


def safety_check(cfg, robot, tau, F_actual, force_hist, tau_hist, phase: str):
    tau = np.asarray(tau, dtype=float)
    if not np.all(np.isfinite(tau)):
        return stop_for_safety(robot, f"{phase}: non-finite torque command {tau}")
    if not np.isfinite(F_actual):
        return stop_for_safety(robot, f"{phase}: non-finite contact force {F_actual}")

    tau_abs_max = float(np.max(np.abs(tau)))
    tau_norm = float(np.linalg.norm(tau))
    if tau_abs_max > cfg.torque_abs_limit:
        return stop_for_safety(
            robot,
            f"{phase}: joint torque abs {tau_abs_max:.2f}Nm > {cfg.torque_abs_limit:.2f}Nm",
        )
    if tau_norm > cfg.torque_norm_limit:
        return stop_for_safety(
            robot,
            f"{phase}: torque norm {tau_norm:.2f}Nm > {cfg.torque_norm_limit:.2f}Nm",
        )
    if abs(F_actual) > cfg.force_abs_limit:
        return stop_for_safety(
            robot,
            f"{phase}: contact force {abs(F_actual):.2f}N > {cfg.force_abs_limit:.2f}N",
        )

    force_hist.append(float(F_actual))
    tau_hist.append(tau.copy())
    if len(force_hist) == force_hist.maxlen:
        force_std = float(np.std(force_hist))
        tau_std_norm = float(np.linalg.norm(np.std(np.stack(tau_hist, axis=0), axis=0)))
        # if force_std > cfg.force_osc_std_limit:
        #     return stop_for_safety(
        #         robot,
        #         f"{phase}: force oscillation std {force_std:.2f}N "
        #         f"> {cfg.force_osc_std_limit:.2f}N",
        #     )
        if tau_std_norm > cfg.torque_osc_std_norm_limit:
            return stop_for_safety(
                robot,
                f"{phase}: torque oscillation std-norm {tau_std_norm:.2f}Nm "
                f"> {cfg.torque_osc_std_norm_limit:.2f}Nm",
            )
    return True


def read_contact_force(force_sensor, venv, x, delta, delta_dot):
    """
    获取当前接触力。

    优先使用新鲜的六维力传感器数据；若传感器未启动、串口无数据或超时，
    回退到原 Kelvin-Voigt 虚拟刚度环境。
    """
    if force_sensor is not None and force_sensor.available():
        return (
            force_sensor.contact_force(),
            force_sensor.wrench_vector(),
            "sensor",
            True,
        )

    F_z = venv.compute_force(x, delta, delta_dot) if venv is not None else 0.0
    return F_z, np.zeros(6), "virtual", False


# ================================================================
# 单次试验
# ================================================================
def run_trial(robot, kin_tool, kin_flange,
              cfg, ctrl, sched, est, venv, force_sensor, logger, trial_id):
    rate = rospy.Rate(cfg.ctrl_rate)
    dt = cfg.dt

    # 泄漏积分器状态 (仅 σ_f, e_r1 直接由 x−x_r 计算)
    sigma_f_int = LeakyIntegrator(eps=cfg.eps_f, dt=dt, dim=3)
    integ_euler = np.zeros(3)
    force_filter = FirstOrderLowPass(cfg.force_filter_tau, initial=0.0)
    stiffness_filter = FirstOrderLowPass(cfg.stiffness_filter_tau, initial=500.0)
    tau_limiter = VectorRateLimiter(cfg.torque_rate_limit)
    safety_window = max(3, int(round(cfg.oscillation_window_s / dt)))
    force_hist = deque(maxlen=safety_window)
    tau_hist = deque(maxlen=safety_window)
    last_wall_time = time.time()

    # 差分导数缓存
    prev_ef = 0.0
    prev_er = 0.0
    prev_K = 500.0

    est.reset()
    sched.reset()
    sigma_f_int.reset()

    rospy.loginfo(f"[Trial {trial_id}] {sched.name} (RCM)")

    # ---- Phase 0: 到起点 ----
    # safe_move_to_joint_position(robot, cfg.init_joints)
    robot.move_to_joint_position(cfg.init_joints)
    rospy.sleep(cfg.settle_time)
    rs = update_robot_state(kin_tool, kin_flange)
    rospy.loginfo(f"  Tool at: {rs['tool_position']}")

    # ---- Phase 1: 下降接近 ----
    z_min_safe = cfg.scan_z - cfg.max_surface_penetration
    if rs["tool_position"][2] < cfg.approach_z:
        return stop_for_safety(
            robot,
            f"preflight: tool starts below approach clearance "
            f"z={rs['tool_position'][2]:.4f}m < approach_z={cfg.approach_z:.4f}m. "
            f"Move the robot to a safe pose above the surface before starting "
            f"(surface={cfg.scan_z:.4f}m, hard lower limit={z_min_safe:.4f}m)."
        )

    rospy.loginfo("  Phase 1: Approaching...")
    rospy.loginfo(
        f"  Approach speed={cfg.approach_vz:.4f}m/s, timeout={cfg.approach_timeout_s:.1f}s, "
        f"surface={cfg.scan_z:.4f}m, approach target="
        f"{cfg.scan_z + cfg.approach_surface_margin:.4f}m, "
        f"hard z limit>={z_min_safe:.4f}m"
    )
    z_contact = None
    F_z = 0.0
    t0 = time.time()

    while not rospy.is_shutdown() and z_contact is None:
        now_wall = time.time()
        wall_dt = now_wall - last_wall_time
        last_wall_time = now_wall
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]

        delta = max(0, cfg.scan_z - tp[2])
        F_raw, wrench, force_source, sensor_available = read_contact_force(
            force_sensor, venv, tp[0], delta, -tv[2]
        )
        F_z = float(force_filter.update(F_raw, dt))

        approach_target_z = cfg.scan_z + cfg.approach_surface_margin
        if abs(F_z) > 0.3 or tp[2] <= approach_target_z:
            z_contact = tp[2]
            rospy.loginfo(
                f"  Approach target reached at z={z_contact:.4f}, F={F_z:.3f}N "
                f"(raw={F_raw:.3f}, {force_source}, "
                f"target={approach_target_z:.4f}m)"
            )
            break

        # 纯位控下降 (α=1) — flange-space 控制
        z_ref_next = max(approach_target_z, tp[2] - cfg.approach_vz * dt)
        x_tool_ref = np.array([cfg.scan_start_x, cfg.scan_y, z_ref_next])
        z_vel_ref = 0.0 if z_ref_next <= approach_target_z + 1e-9 else -cfg.approach_vz
        xdot_tool_ref = np.array([0.0, 0.0, z_vel_ref])

        # 自由空间: 力误差为 0, 力积分不更新
        e_f = np.zeros(3)
        sigma_f = sigma_f_int.get()

        tau, _, _, integ_euler, _, _ = compute_torque_with_rcm(
            ctrl, rs, kin_flange,
            x_tool_ref=x_tool_ref, xdot_tool_ref=xdot_tool_ref,
            e_f=e_f, sigma_f=sigma_f,
            alpha=1.0, K_e_hat=500,
            trocar_pos=cfg.trocar_position,
            length=cfg.tool_length,
            integ_euler=integ_euler, dt=dt,
        )
        tau = tau_limiter.update(tau, dt)
        if not safety_check(cfg, robot, tau, abs(F_z), force_hist, tau_hist, "approach"):
            return False
        robot.exec_torque_cmd(tau)
        if wall_dt > cfg.loop_warn_period:
            rospy.logwarn_throttle(
                1.0,
                f"  RCM wall-loop slow in approach: wall_dt={wall_dt*1000:.1f}ms "
                f"(target={dt*1000:.1f}ms)"
            )
        rate.sleep()

        if time.time() - t0 > cfg.approach_timeout_s:
            rospy.logwarn("  Approach timeout")
            return False

    if z_contact is None:
        z_contact = cfg.scan_z

    # 重置积分器状态 (进入 Phase 2)
    force_filter.reset(F_z)
    stiffness_filter.reset(500.0)
    tau_limiter.reset()
    force_hist.clear()
    tau_hist.clear()
    sigma_f_int.reset()
    integ_euler = np.zeros(3)
    last_wall_time = time.time()

    # ---- Phase 2: 恒力扫描 ----
    rospy.loginfo("  Phase 2: Scanning...")
    x_cur = cfg.scan_start_x
    t_scan = time.time()

    while not rospy.is_shutdown() and x_cur < cfg.scan_end_x:
        now_wall = time.time()
        wall_dt = now_wall - last_wall_time
        last_wall_time = now_wall
        t = time.time() - t_scan
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]

        # 接触力: 传感器优先, 不可用时回退到虚拟环境
        delta = max(0, z_contact - tp[2] + 0.003)
        F_raw, wrench, force_source, sensor_available = read_contact_force(
            force_sensor, venv, tp[0], delta, -tv[2]
        )
        F_z = float(force_filter.update(F_raw, dt))

        # 环境估计
        K_hat_raw, B_hat = est.update(
            abs(F_z),
            max(0, z_contact - tp[2] + 0.003),
            -tv[2]
        )
        K_hat = float(stiffness_filter.update(K_hat_raw, dt))

        # tool 期望轨迹: z 参考不得低于表面下 5mm
        z_ref_scan = max(z_min_safe, z_contact)
        x_tool_ref = np.array([x_cur, cfg.scan_y, z_ref_scan])
        xdot_tool_ref = np.array([cfg.scan_vx, 0.0, 0.0])

        # 力目标: 向上接触反力 (z+) 为正
        F_des = np.array([0.0, 0.0, cfg.F_desired])
        F_meas = np.array([0.0, 0.0, abs(F_z)])

        # 力误差 + 力误差泄漏积分
        e_f_vec = F_meas - F_des
        force_blend = min(1.0, max(0.0, t / cfg.contact_force_blend_time))
        e_f_vec_ctrl = force_blend * e_f_vec
        sigma_f = sigma_f_int.update(e_f_vec_ctrl)

        # 差分导数 (用于 α 调度)
        F_actual = abs(F_z)
        e_f_scalar = F_actual - cfg.F_desired
        e_f_dot = (e_f_scalar - prev_ef) / dt
        prev_ef = e_f_scalar

        e_r_scalar = np.linalg.norm(tp[:2] - np.array([x_cur, cfg.scan_y]))
        de_r = (e_r_scalar - prev_er) / dt
        prev_er = e_r_scalar

        dK = (K_hat - prev_K) / dt
        prev_K = K_hat

        # α 计算
        if isinstance(sched, FixedAlphaScheduler):
            alpha = sched.compute()
            phase_val = -1
        else:
            alpha = sched.compute(
                F_norm=abs(F_z), e_f=e_f_scalar, K_hat=K_hat,
                e_r=e_r_scalar, z_vel=tv[2],
                de_f=e_f_dot, dK=dK, de_r=de_r,
                F_desired=cfg.F_desired,
                F_min=cfg.F_min,
                F_max=cfg.F_max,
            )
            phase_val = sched.phase_detector.phase.value

        K_hat_ctrl = K_hat

        # 博弈控制 (flange-space, 内部计算 flange 期望与误差)
        tau, u_flange, K_eff, integ_euler, x_flange_ref, error = \
            compute_torque_with_rcm(
                ctrl, rs, kin_flange,
                x_tool_ref=x_tool_ref, xdot_tool_ref=xdot_tool_ref,
                e_f=e_f_vec_ctrl, sigma_f=sigma_f,
                alpha=alpha, K_e_hat=K_hat_ctrl,
                trocar_pos=cfg.trocar_position,
                length=cfg.tool_length,
                integ_euler=integ_euler, dt=dt,
            )
        tau_limiter.max_rate = cfg.torque_rate_limit
        tau = tau_limiter.update(tau, dt)
        if not safety_check(cfg, robot, tau, F_actual, force_hist, tau_hist, "scan"):
            return False
        robot.exec_torque_cmd(tau)
        if wall_dt > cfg.loop_warn_period:
            rospy.logwarn_throttle(
                1.0,
                f"  RCM wall-loop slow in scan: wall_dt={wall_dt*1000:.1f}ms "
                f"(target={dt*1000:.1f}ms)"
            )

        # 推进扫描参考
        x_cur += cfg.scan_vx * dt

        # 6 个核心物理量
        pos_tool = tp.copy()
        pos_tool_des = x_tool_ref.copy()
        pos_err = pos_tool - pos_tool_des           # tool 位置误差 (3D)
        pos_err_norm = np.linalg.norm(pos_err)
        F_desired = cfg.F_desired
        F_err = F_actual - F_desired
        rcm_err = error[0]

        # 实时打印 (10 Hz, 每 10 步打一次)
        if logger.count % 10 == 0:
            rospy.loginfo(
                f"  t={t:5.2f}s | "
                f"tool=[{pos_tool[0]*1000:6.2f},{pos_tool[1]*1000:6.2f},{pos_tool[2]*1000:6.2f}]mm | "
                f"des=[{pos_tool_des[0]*1000:6.2f},{pos_tool_des[1]*1000:6.2f},{pos_tool_des[2]*1000:6.2f}]mm | "
                f"err={pos_err_norm*1000:5.2f}mm | "
                f"rcm={rcm_err*1000:5.2f}mm | "
                f"F={F_actual:.3f}N (des={F_desired:.3f}, err={F_err:+.3f}) | "
                f"rawF={F_raw:.3f}N | K={K_hat_ctrl:7.1f} rawK={K_hat_raw:7.1f} | "
                f"α={alpha:.2f} blend={force_blend:.2f} | "
                f"source={force_source}"
            )

        # 日志中的 e_r1 取 flange 误差范数 (新方案的反馈量)
        e_r1_flange_norm = np.linalg.norm(rs["flange_position"] - x_flange_ref)

        # 记录
        logger.log(
            t=t,
            pos=pos_tool,
            pos_des=pos_tool_des,
            pos_err=pos_err,
            F_measured=F_actual, F_desired=F_desired, F_err=F_err,
            F_raw=F_raw,
            wrench=wrench,
            force_source=force_source,
            sensor_available=int(sensor_available),
            e_f=e_f_scalar, e_r=e_r_scalar,
            sigma_f_norm=np.linalg.norm(sigma_f),
            e_r1_norm=e_r1_flange_norm,
            alpha=alpha, K_hat=K_hat,
            K_hat_raw=K_hat_raw,
            K_hat_ctrl=K_hat_ctrl,
            K_eff=K_eff,
            x_desired=x_cur,
            error_rcm=rcm_err, error_track=error[1],
            arbitration_strategy=sched.name,
            u_norm=np.linalg.norm(tau),
            force_blend=force_blend,
            z_min_safe=z_min_safe,
            phase=phase_val,
        )
        rate.sleep()

    # ---- Phase 3: 撤退 ----
    rospy.loginfo("  Phase 3: Retreating...")
    if hasattr(sched, 'set_retreat'):
        sched.set_retreat(True)
    # safe_move_to_joint_position(robot, cfg.init_joints)
    # robot.move_to_joint_position(cfg.init_joints)
    rospy.loginfo(f"  Done. {logger.count} samples.")
    return True


# ================================================================
# 策略工厂
# ================================================================
STRATEGIES = {
    'fixed_08': lambda: FixedAlphaScheduler(0.8),
    'fixed_05': lambda: FixedAlphaScheduler(0.5),
    'fixed_02': lambda: FixedAlphaScheduler(0.2),
    'coop_fuzzy': lambda: PhaseAwareFuzzyAlphaScheduler(dt=0.01),
    'force_margin': lambda: ForceMarginFuzzyAlphaScheduler(
        dt=0.01,
        F_min=Config.F_min,
        F_max=Config.F_max,
        F_desired=Config.F_desired,
    ),
}


# ================================================================
# 主入口
# ================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', default='force_margin')
    ap.add_argument('--trials', type=int, default=1)
    ap.add_argument('--output-dir', default='results')
    ap.add_argument('--gains-file', default=None)
    ap.add_argument('--force-port', default='/dev/ttyUSB0')
    ap.add_argument('--force-baudrate', type=int, default=460800)
    ap.add_argument('--force-serial-timeout', type=float, default=0.05)
    ap.add_argument('--force-timeout', type=float, default=0.02,
                    help='direct sensor freshness window; 0.02s matches 1kHz streaming with margin')
    ap.add_argument('--force-axis', type=int, default=2)
    ap.add_argument('--force-sign', type=float, default=1.0)
    ap.add_argument('--force-wait-timeout', type=float, default=2.0)
    ap.add_argument('--force-expected-rate', type=float, default=1000.0)
    ap.add_argument('--force-command-format', default='both',
                    choices=('manual', 'vendor', 'both'))
    ap.add_argument('--force-data-source', default='0x33')
    ap.add_argument('--force-streaming', dest='force_streaming',
                    action='store_true', default=True)
    ap.add_argument('--no-force-streaming', dest='force_streaming',
                    action='store_false')
    ap.add_argument('--force-poll-hz', type=float, default=100.0)
    ap.add_argument('--force-output-units', default='N', choices=('N', 'kgf'))
    ap.add_argument('--force-tare-on-start', dest='force_tare_on_start',
                    action='store_true', default=True)
    ap.add_argument('--no-force-tare-on-start', dest='force_tare_on_start',
                    action='store_false')
    ap.add_argument('--force-tare-settle', type=float, default=1.0,
                    help='seconds to wait/drain zero[] echo after startup tare')
    ap.add_argument('--no-force-sensor', action='store_true')
    args = ap.parse_args()

    rospy.init_node("coop_gt_rcm")
    rospy.loginfo("=" * 60)
    rospy.loginfo("  Cooperative Game Force-Position Experiment (RCM)")
    rospy.loginfo("  Model: 4D per-axis ARE + K_v baseline + leaky integrators")
    rospy.loginfo("=" * 60)

    robot = PandaArm()
    kin_tool = PandaKinematics(robot, "panda_link10")
    kin_flange = PandaKinematics(robot, "panda_link8")
    rospy.sleep(1.0)

    cfg = Config()
    ctrl = CooperativeGameController()
    rospy.loginfo(
        f"Real safety: approach_vz={cfg.approach_vz:.4f}m/s, scan_vx={cfg.scan_vx:.4f}m/s, "
        f"force_limit={cfg.force_abs_limit:.1f}N, torque_abs_limit={cfg.torque_abs_limit:.1f}Nm, "
        f"torque_norm_limit={cfg.torque_norm_limit:.1f}Nm, torque_rate_limit={cfg.torque_rate_limit:.1f}Nm/s, "
        f"osc_window={cfg.oscillation_window_s:.2f}s, "
        f"force_std_limit={cfg.force_osc_std_limit:.2f}N, "
        f"tau_std_norm_limit={cfg.torque_osc_std_norm_limit:.1f}Nm"
    )

    # 增益表
    if args.gains_file and os.path.exists(args.gains_file):
        ctrl.load_gains(args.gains_file)
        rospy.loginfo(f"Loaded gains from {args.gains_file}")
    else:
        Ke_vals = sorted(set(
            [v[2] for v in cfg.stiffness_zones]
            + [50, 80, 100, 150, 200, 300, 500, 800, 1000, 1500,
               2000, 3000, 5000]
        ))
        ctrl.precompute_gains(
            alpha_grid=np.linspace(0.0, 1.0, 21),
            Ke_grid=Ke_vals,
        )
        os.makedirs(args.output_dir, exist_ok=True)
        gpath = os.path.join(args.output_dir, 'coop_gains_rcm.npy')
        ctrl.save_gains(gpath)
        rospy.loginfo(f"Saved gains to {gpath}")

    venv = VirtualStiffnessSurface(cfg.stiffness_zones)
    force_sensor = None
    if not args.no_force_sensor:
        data_source = int(args.force_data_source, 0)
        force_sensor = DirectForceSensorInput(
            port=args.force_port,
            baudrate=args.force_baudrate,
            serial_timeout=args.force_serial_timeout,
            freshness_timeout=args.force_timeout,
            force_axis=args.force_axis,
            force_sign=args.force_sign,
            command_format=args.force_command_format,
            data_source_cmd=data_source,
            use_streaming=args.force_streaming,
            output_units=args.force_output_units,
            tare_on_start=args.force_tare_on_start,
            tare_settle_s=args.force_tare_settle,
            poll_hz=args.force_poll_hz,
        )
        rospy.on_shutdown(force_sensor.close)
        got_first_frame = force_sensor.wait_for_data(args.force_wait_timeout)
        rospy.loginfo(
            f"Direct force sensor: {args.force_port} @ {args.force_baudrate}, "
            f"axis={args.force_axis}, sign={args.force_sign}, "
            f"timeout={args.force_timeout}s; expected_rate={args.force_expected_rate:g}Hz; "
            f"command_format={args.force_command_format}, "
            f"data_source=0x{data_source:02X}, streaming={args.force_streaming}, "
            f"tare_on_start={args.force_tare_on_start}, "
            f"detected_format={force_sensor.detected_format}; "
            f"first_frame={got_first_frame}, seq={force_sensor.seq()}, "
            f"age={force_sensor.age():.4f}s; virtual env fallback enabled"
        )
    else:
        rospy.loginfo("Force sensor disabled; using virtual env fallback only")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    odir = os.path.join(args.output_dir, f"rcm_{stamp}")
    os.makedirs(odir, exist_ok=True)

    names = list(STRATEGIES.keys()) if args.strategy == 'all' else [args.strategy]
    for sn in names:
        s = STRATEGIES[sn]()
        rospy.loginfo(f"\n{'='*50}\n  Strategy: {s.name}\n{'='*50}")
        for t in range(args.trials):
            if rospy.is_shutdown():
                break
            lg = DataLogger()
            est = EnvironmentEstimator()
            s.reset()
            ok = run_trial(
                robot, kin_tool, kin_flange,
                cfg, ctrl, s, est, venv, force_sensor, lg, t,
            )
            if ok:
                lg.save(os.path.join(odir, f"{s.name}_t{t:02d}.npz"))

    if force_sensor is not None:
        force_sensor.close()

    rospy.loginfo(f"\nResults → {odir}")


if __name__ == '__main__':
    main()
