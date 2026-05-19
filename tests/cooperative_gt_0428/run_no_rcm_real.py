#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
主程序 B: 无 RCM 约束的合作博弈力-位扫描实验
=============================================

控制链路:
  泄漏积分更新 → 4D-ARE 查表 → u_tool 直接作为笛卡尔力 → J_tool^T → τ

与 RCM 模式的区别:
  - 使用 panda_link10 的 Jacobian (不是 flange)
  - 无杠杆变换
  - 姿态保持: PD+I 维持初始 tool frame 姿态
  - 扫描范围更大, 力目标更大

用法:
  Terminal 1: roslaunch panda_simulator simulation.launch
  Terminal 2: python run_no_rcm_real.py --strategy all --trials 3
"""
import argparse
import os
import time
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
    compute_torque_no_rcm, fixed_downward_tool_euler, INIT_JOINTS,
)


class Config:
    scan_start_x = 0.25
    scan_end_x   = 0.40
    scan_y       = 0.0
    scan_z       = 0.007
    approach_z   = 0.05
    scan_vx      = 0.005
    F_desired    = 0.5
    F_min        = 0.3
    F_max        = 1.0
    force_axis   = 2

    stiffness_zones = [
        (0.25, 0.29, 500,  5),
        (0.29, 0.33, 1000, 10),
        (0.33, 0.37, 200,  2),
        (0.37, 0.41, 500,  5),
    ]

    ctrl_rate = 100
    dt = 1.0 / ctrl_rate
    eps_r = 1.0
    eps_f = 2.0
    settle_time = 2.0
    force_filter_tau = 0.04
    stiffness_filter_tau = 0.12
    torque_rate_limit = 120.0
    loop_warn_period = 0.08
    no_rcm_u_threshold = 20.0
    no_rcm_P_ori = 12.0
    no_rcm_D_ori = 2.0
    no_rcm_I_ori = 5.0
    no_rcm_alpha_floor = 0.65
    no_rcm_u_tool_limits = np.array([8.0, 8.0, 6.0])
    approach_speed = 0.002
    approach_timeout_min = 20.0
    approach_timeout_margin = 8.0


def read_contact_force(force_sensor, venv, x, delta, delta_dot):
    """
    获取当前接触力。

    传感器数据在 timeout 内新鲜时优先使用六维力信息；否则保持原有
    Kelvin-Voigt 虚拟刚度环境作为回退。
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


def run_trial(robot, kin_tool, kin_flange,
              cfg, ctrl, sched, est, venv, force_sensor, logger, trial_id):
    rate = rospy.Rate(cfg.ctrl_rate)
    dt = cfg.dt

    sigma_f_int = LeakyIntegrator(eps=cfg.eps_f, dt=dt, dim=3)
    integ_euler = np.zeros(3)
    force_filter = FirstOrderLowPass(cfg.force_filter_tau, initial=0.0)
    stiffness_filter = FirstOrderLowPass(cfg.stiffness_filter_tau, initial=500.0)
    tau_limiter = VectorRateLimiter(cfg.torque_rate_limit)
    last_wall_time = time.time()

    prev_ef = 0.0; prev_er = 0.0; prev_K = 500.0

    est.reset()
    sched.reset()
    sigma_f_int.reset()

    rospy.loginfo(f"[Trial {trial_id}] {sched.name} (NO-RCM)")

    safe_move_to_joint_position(robot, INIT_JOINTS)
    rospy.sleep(cfg.settle_time)
    rs = update_robot_state(kin_tool, kin_flange)
    ref_euler_fixed = fixed_downward_tool_euler()
    rospy.loginfo(f"  Tool at: {rs['tool_position']}, "
                  f"fixed downward euler ref: {ref_euler_fixed}")

    # ---- Phase 1: 接近 ----
    rospy.loginfo("  Phase 1: Approaching...")
    z_contact = None
    t0 = rospy.Time.now()
    contact_z_threshold = cfg.scan_z + 0.002
    approach_distance = max(0.0, rs["tool_position"][2] - contact_z_threshold)
    approach_timeout = max(
        cfg.approach_timeout_min,
        approach_distance / max(cfg.approach_speed, 1e-6) + cfg.approach_timeout_margin,
    )
    approach_xy_ref = rs["tool_position"][:2].copy()
    z_ref_approach = float(rs["tool_position"][2])
    rospy.loginfo(
        f"  Contact criteria: |Fz|>0.300N or tool_z<={contact_z_threshold:.4f}m "
        f"(scan_z={cfg.scan_z:.4f}m); "
        f"distance={approach_distance:.4f}m, speed={cfg.approach_speed:.4f}m/s, "
        f"timeout={approach_timeout:.1f}s; "
        f"holding approach xy=[{approach_xy_ref[0]:.4f}, {approach_xy_ref[1]:.4f}]"
    )

    while not rospy.is_shutdown() and z_contact is None:
        now_wall = time.time()
        wall_dt = now_wall - last_wall_time
        last_wall_time = now_wall
        dt_loop = dt
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]

        F_raw, wrench, force_source, sensor_available = read_contact_force(
            force_sensor, venv, tp[0], max(0, cfg.scan_z - tp[2]), -tv[2]
        )
        F_z = float(force_filter.update(F_raw, dt_loop))

        if abs(F_z) > 0.3 or tp[2] <= contact_z_threshold:
            z_contact = tp[2]
            rospy.loginfo(
                f"  Contact at z={z_contact:.4f}, F={F_z:.3f}N "
                f"(raw={F_raw:.3f}, {force_source})"
            )
            break

        xdot_ref = np.array([0.0, 0.0, -cfg.approach_speed])
        z_ref_approach = max(
            contact_z_threshold,
            z_ref_approach - cfg.approach_speed * dt_loop,
        )
        x_ref_p1 = np.array([approach_xy_ref[0], approach_xy_ref[1],
                              z_ref_approach])
        e_r1 = tp - x_ref_p1
        e_r2 = tv - xdot_ref
        e_f = np.zeros(3)
        sigma_f = sigma_f_int.get()

        tau, _, _, integ_euler, _ = compute_torque_no_rcm(
            ctrl, rs, kin_tool,
            e_r1, e_r2, e_f, sigma_f,
            alpha=1.0, K_e_hat=500,
            ref_euler_fixed=ref_euler_fixed,
            integ_euler=integ_euler, dt=dt_loop,
        )
        tau = tau_limiter.update(tau, dt_loop)
        robot.exec_torque_cmd(tau)
        if wall_dt > cfg.loop_warn_period:
            rospy.logwarn_throttle(
                1.0,
                f"  NO-RCM wall-loop slow in approach: wall_dt={wall_dt*1000:.1f}ms "
                f"(target={dt*1000:.1f}ms)"
            )
        rate.sleep()

        if (rospy.Time.now() - t0).to_sec() > approach_timeout:
            rospy.logwarn(
                f"  Approach timeout after {approach_timeout:.1f}s "
                f"(z={tp[2]:.4f}, z_ref={z_ref_approach:.4f}, "
                f"target<={contact_z_threshold:.4f}, "
                f"F={F_z:.3f}N, raw={F_raw:.3f}N, source={force_source}, "
                f"sensor_available={sensor_available})"
            )
            return False

    if z_contact is None:
        z_contact = cfg.scan_z

    force_filter.reset(F_z)
    stiffness_filter.reset(500.0)
    tau_limiter.reset()
    sigma_f_int.reset()
    integ_euler = np.zeros(3)
    last_wall_time = time.time()

    # ---- Phase 2: 恒力扫描 ----
    rospy.loginfo("  Phase 2: Scanning...")
    rs = update_robot_state(kin_tool, kin_flange)
    x_cur = max(cfg.scan_start_x, rs["tool_position"][0])
    if abs(x_cur - cfg.scan_start_x) > 1e-4:
        rospy.logwarn(
            f"  Scan start shifted from {cfg.scan_start_x:.4f}m to "
            f"current tool x={x_cur:.4f}m to avoid a lateral step"
        )
    t_scan = rospy.Time.now()

    while not rospy.is_shutdown() and x_cur < cfg.scan_end_x:
        now_wall = time.time()
        wall_dt = now_wall - last_wall_time
        last_wall_time = now_wall
        dt_loop = dt
        t = (rospy.Time.now() - t_scan).to_sec()
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]

        delta = max(0, z_contact - tp[2] + 0.003)
        F_raw, wrench, force_source, sensor_available = read_contact_force(
            force_sensor, venv, tp[0], delta, -tv[2]
        )
        F_z = float(force_filter.update(F_raw, dt_loop))

        K_hat_raw, B_hat = est.update(
            abs(F_z),
            max(0, z_contact - tp[2] + 0.003),
            -tv[2],
        )
        K_hat = float(stiffness_filter.update(K_hat_raw, dt_loop))

        x_ref = np.array([x_cur, cfg.scan_y, z_contact])
        xdot_ref = np.array([cfg.scan_vx, 0, 0])

        # 力目标: +z 方向为上 (接触反力方向)
        F_des = np.array([0.0, 0.0, cfg.F_desired])
        F_meas = np.array([0.0, 0.0, abs(F_z)])

        e_r1 = tp - x_ref
        e_r2 = tv - xdot_ref
        e_f_vec = F_meas - F_des
        sigma_f = sigma_f_int.update(e_f_vec)

        F_actual = abs(F_z)
        e_f_scalar = F_actual - cfg.F_desired
        e_f_dot = (e_f_scalar - prev_ef) / dt_loop; prev_ef = e_f_scalar
        e_r_scalar = np.linalg.norm(tp[:2] - np.array([x_cur, cfg.scan_y]))
        de_r = (e_r_scalar - prev_er) / dt_loop; prev_er = e_r_scalar
        dK = (K_hat - prev_K) / dt_loop; prev_K = K_hat

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
            alpha = max(alpha, cfg.no_rcm_alpha_floor)
            phase_val = sched.phase_detector.phase.value

        tau, u_tool, K_eff, integ_euler, error = compute_torque_no_rcm(
            ctrl, rs, kin_tool,
            e_r1, e_r2, e_f_vec, sigma_f,
            alpha=alpha, K_e_hat=K_hat,
            ref_euler_fixed=ref_euler_fixed,
            integ_euler=integ_euler, dt=dt_loop,
        )
        tau = tau_limiter.update(tau, dt_loop)
        robot.exec_torque_cmd(tau)
        if wall_dt > cfg.loop_warn_period:
            rospy.logwarn_throttle(
                1.0,
                f"  NO-RCM wall-loop slow in scan: wall_dt={wall_dt*1000:.1f}ms "
                f"(target={dt*1000:.1f}ms)"
            )
        x_cur += cfg.scan_vx * dt_loop

        pos_tool = tp.copy()
        pos_tool_des = x_ref.copy()
        pos_err = pos_tool - pos_tool_des
        pos_err_norm = np.linalg.norm(pos_err)
        F_desired = cfg.F_desired
        F_err = F_actual - F_desired

        if logger.count % 10 == 0:
            rospy.loginfo(
                f"  t={t:5.2f}s | "
                f"tool=[{pos_tool[0]*1000:6.2f},{pos_tool[1]*1000:6.2f},{pos_tool[2]*1000:6.2f}]mm | "
                f"des=[{pos_tool_des[0]*1000:6.2f},{pos_tool_des[1]*1000:6.2f},{pos_tool_des[2]*1000:6.2f}]mm | "
                f"err={pos_err_norm*1000:5.2f}mm | "
                f"F={F_actual:.3f}N raw={F_raw:.3f}N "
                f"(des={F_desired:.3f}, err={F_err:+.3f}) | "
                f"K={K_hat:7.1f} rawK={K_hat_raw:7.1f} B={B_hat:5.2f} | "
                f"e_f={e_f_scalar:+.3f} e_r={e_r_scalar*1000:5.2f}mm | "
                f"α={alpha:.2f} phase={phase_val} | "
                f"dt={dt_loop*1000:4.1f}ms | source={force_source}"
            )

        logger.log(
            t=t, pos=pos_tool,
            pos_des=pos_tool_des,
            pos_err=pos_err,
            F_measured=F_actual, F_desired=F_desired,
            F_err=F_err,
            wrench=wrench,
            force_source=force_source,
            sensor_available=int(sensor_available),
            e_f=e_f_scalar, e_r=e_r_scalar,
            sigma_f_norm=np.linalg.norm(sigma_f),
            e_r1_norm=np.linalg.norm(e_r1),
            alpha=alpha, K_hat=K_hat,
            K_eff=K_eff,
            x_desired=x_cur,
            error_rcm=0.0, error_track=error[1],
            u_norm=np.linalg.norm(tau),
            phase=phase_val,
        )
        rate.sleep()

    rospy.loginfo("  Phase 3: Retreating...")
    if hasattr(sched, 'set_retreat'):
        sched.set_retreat(True)
    safe_move_to_joint_position(robot, INIT_JOINTS)
    rospy.loginfo(f"  Done. {logger.count} samples.")
    return True


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', default='force_margin')
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--use-virtual-env', action='store_true')
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
    ap.add_argument('--force-tare-on-start', action='store_true')
    ap.add_argument('--no-force-sensor', action='store_true')
    args = ap.parse_args()

    rospy.init_node("coop_gt_no_rcm")
    rospy.loginfo("=" * 60)
    rospy.loginfo("  Cooperative Game Force-Position Experiment (NO RCM)")
    rospy.loginfo("=" * 60)

    robot = PandaArm()
    kin_tool = PandaKinematics(robot, "panda_link10")
    kin_flange = PandaKinematics(robot, "panda_link8")
    rospy.sleep(1.0)

    cfg = Config()
    ctrl = CooperativeGameController()
    ctrl.u_threshold = cfg.no_rcm_u_threshold
    ctrl.P_ori = cfg.no_rcm_P_ori
    ctrl.D_ori = cfg.no_rcm_D_ori
    ctrl.I_ori = cfg.no_rcm_I_ori
    ctrl.no_rcm_u_tool_limits = cfg.no_rcm_u_tool_limits
    rospy.loginfo(
        f"NO-RCM damping guards: ctrl_rate={cfg.ctrl_rate}Hz, "
        f"force_tau={cfg.force_filter_tau:.3f}s, "
        f"K_tau={cfg.stiffness_filter_tau:.3f}s, "
        f"tau_rate_limit={cfg.torque_rate_limit:.1f}Nm/s, "
        f"u_threshold={ctrl.u_threshold:.1f}, "
        f"ori_gains=({ctrl.P_ori:.1f},{ctrl.D_ori:.1f},{ctrl.I_ori:.1f}), "
        f"alpha_floor={cfg.no_rcm_alpha_floor:.2f}, "
        f"u_tool_limits={cfg.no_rcm_u_tool_limits}"
    )

    if args.gains_file and os.path.exists(args.gains_file):
        ctrl.load_gains(args.gains_file)
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
        ctrl.save_gains(os.path.join(args.output_dir, 'coop_gains_no_rcm.npy'))

    venv = VirtualStiffnessSurface(cfg.stiffness_zones)
    if not args.use_virtual_env:
        rospy.loginfo(
            "--use-virtual-env is no longer required for fallback; "
            "virtual env remains enabled when force sensor is unavailable"
        )

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
            f"detected_format={force_sensor.detected_format}; "
            f"first_frame={got_first_frame}, seq={force_sensor.seq()}, "
            f"age={force_sensor.age():.4f}s; virtual env fallback enabled"
        )
    else:
        rospy.loginfo("Force sensor disabled; using virtual env fallback only")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    odir = os.path.join(args.output_dir, f"no_rcm_{stamp}")
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
