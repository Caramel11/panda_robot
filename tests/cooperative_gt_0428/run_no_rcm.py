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
  Terminal 2: python run_no_rcm.py --strategy all --trials 3 --use-virtual-env
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
    PhaseAwareFuzzyAlphaScheduler, FixedAlphaScheduler
)
from src.env_estimator import EnvironmentEstimator
from src.utils import VirtualStiffnessSurface, DataLogger
from src.leaky_integrator import LeakyIntegrator
from src.robot_interface import (
    update_robot_state, safe_move_to_joint_position,
    compute_torque_no_rcm, INIT_JOINTS,
)


class Config:
    scan_start_x = 0.40
    scan_end_x   = 0.65
    scan_y       = 0.0
    scan_z       = 0.20
    approach_z   = 0.30
    scan_vx      = 0.01
    F_desired    = 1.0
    force_axis   = 2

    stiffness_zones = [
    (0.25, 0.29, 500,  5),    # 软
    (0.29, 0.33, 1000,  10),   # 硬
    (0.33, 0.37, 200,  2),    # 极软
    (0.37, 0.41, 500,  5),    # 软
    ]

    ctrl_rate = 100
    dt = 1.0 / ctrl_rate
    eps_r = 1.0
    eps_f = 2.0
    settle_time = 2.0


def run_trial(robot, kin_tool, kin_flange,
              cfg, ctrl, sched, est, venv, logger, trial_id):
    rate = rospy.Rate(cfg.ctrl_rate)
    dt = cfg.dt

    sigma_f_int = LeakyIntegrator(eps=cfg.eps_f, dt=dt, dim=3)
    integ_euler = np.zeros(3)

    prev_ef = 0.0; prev_er = 0.0; prev_K = 500.0

    est.reset()
    sched.reset()
    sigma_f_int.reset()

    rospy.loginfo(f"[Trial {trial_id}] {sched.name} (NO-RCM)")

    safe_move_to_joint_position(robot, INIT_JOINTS)
    rospy.sleep(cfg.settle_time)
    rs = update_robot_state(kin_tool, kin_flange)
    ref_euler_fixed = rs["tool_rotation_euler"].copy()
    rospy.loginfo(f"  Tool at: {rs['tool_position']}, "
                  f"euler: {ref_euler_fixed}")

    # ---- Phase 1: 接近 ----
    rospy.loginfo("  Phase 1: Approaching...")
    z_contact = None
    t0 = time.time()

    while not rospy.is_shutdown() and z_contact is None:
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]

        F_z = 0.0
        if venv:
            F_z = venv.compute_force(
                tp[0], max(0, cfg.scan_z - tp[2]), -tv[2]
            )

        if abs(F_z) > 0.3 or tp[2] <= cfg.scan_z + 0.005:
            z_contact = tp[2]
            rospy.loginfo(f"  Contact at z={z_contact:.4f}, F={F_z:.3f}N")
            break

        xdot_ref = np.array([0, 0, -0.005])
        x_ref_p1 = np.array([cfg.scan_start_x, cfg.scan_y,
                              tp[2] - 0.005 * dt])
        e_r1 = tp - x_ref_p1
        e_r2 = tv - xdot_ref
        e_f = np.zeros(3)
        sigma_f = sigma_f_int.get()

        tau, _, _, integ_euler, _ = compute_torque_no_rcm(
            ctrl, rs, kin_tool,
            e_r1, e_r2, e_f, sigma_f,
            alpha=1.0, K_e_hat=500,
            ref_euler_fixed=ref_euler_fixed,
            integ_euler=integ_euler, dt=dt,
        )
        robot.exec_torque_cmd(tau)
        rate.sleep()

        if time.time() - t0 > 20:
            rospy.logwarn("  Approach timeout")
            return False

    if z_contact is None:
        z_contact = cfg.scan_z

    sigma_f_int.reset()
    integ_euler = np.zeros(3)

    # ---- Phase 2: 恒力扫描 ----
    rospy.loginfo("  Phase 2: Scanning...")
    x_cur = cfg.scan_start_x
    t_scan = time.time()

    while not rospy.is_shutdown() and x_cur < cfg.scan_end_x:
        t = time.time() - t_scan
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]

        F_z = 0.0
        if venv:
            F_z = venv.compute_force(
                tp[0], max(0, z_contact - tp[2] + 0.003), -tv[2]
            )

        K_hat, B_hat = est.update(
            abs(F_z),
            max(0, z_contact - tp[2] + 0.003),
            -tv[2],
        )

        x_ref = np.array([x_cur, cfg.scan_y, z_contact])
        xdot_ref = np.array([cfg.scan_vx, 0, 0])

        # 力目标: +z 方向为上 (接触反力方向)
        F_des = np.array([0.0, 0.0, cfg.F_desired])
        F_meas = np.array([0.0, 0.0, abs(F_z)])

        e_r1 = tp - x_ref
        e_r2 = tv - xdot_ref
        e_f_vec = F_meas - F_des
        sigma_f = sigma_f_int.update(e_f_vec)

        e_f_scalar = cfg.F_desired - abs(F_z)
        e_f_dot = (e_f_scalar - prev_ef) / dt; prev_ef = e_f_scalar
        e_r_scalar = np.linalg.norm(tp[:2] - np.array([x_cur, cfg.scan_y]))
        de_r = (e_r_scalar - prev_er) / dt; prev_er = e_r_scalar
        dK = (K_hat - prev_K) / dt; prev_K = K_hat

        if isinstance(sched, FixedAlphaScheduler):
            alpha = sched.compute()
            phase_val = -1
        else:
            alpha = sched.compute(
                F_norm=abs(F_z), e_f=e_f_scalar, K_hat=K_hat,
                e_r=e_r_scalar, z_vel=tv[2],
                de_f=e_f_dot, dK=dK, de_r=de_r,
            )
            phase_val = sched.phase_detector.phase.value

        tau, u_tool, K_eff, integ_euler, error = compute_torque_no_rcm(
            ctrl, rs, kin_tool,
            e_r1, e_r2, e_f_vec, sigma_f,
            alpha=alpha, K_e_hat=K_hat,
            ref_euler_fixed=ref_euler_fixed,
            integ_euler=integ_euler, dt=dt,
        )
        robot.exec_torque_cmd(tau)
        x_cur += cfg.scan_vx * dt

        pos_tool = tp.copy()
        pos_tool_des = x_ref.copy()
        pos_err = pos_tool - pos_tool_des
        pos_err_norm = np.linalg.norm(pos_err)
        F_actual = abs(F_z)
        F_desired = cfg.F_desired
        F_err = F_actual - F_desired

        if logger.count % 10 == 0:
            rospy.loginfo(
                f"  t={t:5.2f}s | "
                f"tool=[{pos_tool[0]*1000:6.2f},{pos_tool[1]*1000:6.2f},{pos_tool[2]*1000:6.2f}]mm | "
                f"des=[{pos_tool_des[0]*1000:6.2f},{pos_tool_des[1]*1000:6.2f},{pos_tool_des[2]*1000:6.2f}]mm | "
                f"err={pos_err_norm*1000:5.2f}mm | "
                f"F={F_actual:.3f}N (des={F_desired:.3f}, err={F_err:+.3f}) | "
                f"K={K_hat:7.1f} B={B_hat:5.2f} | "
                f"e_f={e_f_scalar:+.3f} e_r={e_r_scalar*1000:5.2f}mm | "
                f"alpha={alpha:.2f} phase={phase_val}"
            )

        logger.log(
            t=t, pos=pos_tool,
            pos_des=pos_tool_des,
            pos_err=pos_err,
            F_measured=F_actual, F_desired=F_desired,
            F_err=F_err,
            wrench=np.zeros(6),
            force_source='virtual' if venv else 'none',
            sensor_available=0,
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
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', default='all')
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--use-virtual-env', action='store_true')
    ap.add_argument('--output-dir', default='results')
    ap.add_argument('--gains-file', default=None)
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

    venv = VirtualStiffnessSurface(cfg.stiffness_zones) \
        if args.use_virtual_env else None

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
                cfg, ctrl, s, est, venv, lg, t,
            )
            if ok:
                lg.save(os.path.join(odir, f"{s.name}_t{t:02d}.npz"))

    rospy.loginfo(f"\nResults → {odir}")


if __name__ == '__main__':
    main()
