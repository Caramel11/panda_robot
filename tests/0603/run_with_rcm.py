#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
主程序 A: RCM 约束下的合作博弈力-位扫描实验
=============================================

控制链路:
  泄漏积分更新 → ARE/Pareto 增益查表 → u_tool → RCM 杠杆 → τ

数学模型:
  M·ẍ + C·ẋ + K_v·(x−x_r) = u + f_ext      (含 K_v 基线刚度)
  ė_r1 = e_r2 − ε_r · e_r1                  (泄漏积分器)
  ė_f  = -(K_e − B_e·C/M)·e_r2 − B_e/M·e_f − B_e/M·u
  σ̇_f  = e_f − ε_f · σ_f                    (泄漏积分器)

用法:
  Terminal 1: roslaunch panda_simulator simulation.launch
  Terminal 2: python run_with_rcm.py --strategy continuous_force_margin --controller-mode pareto_iter
"""
import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
import rospy
from panda_robot import PandaArm, PandaKinematics

from src.gt_controller import CooperativeGameController
from src.alpha_scheduler_gt import (
    PhaseAwareFuzzyAlphaScheduler, ForceMarginFuzzyAlphaScheduler,
    ContinuousForceMarginFuzzyAlphaScheduler,
    OnlinePriorityAdaptationAlphaScheduler,
    FixedAlphaScheduler
)
from src.env_estimator import EnvironmentEstimator
from src.utils import VirtualStiffnessSurface, ForceSensorInput, DataLogger
from src.leaky_integrator import LeakyIntegrator
from src.robot_interface import (
    update_robot_state, safe_move_to_joint_position,
    compute_torque_with_rcm, INIT_JOINTS,
    compute_position_rcm,
)


# ================================================================
# 实验配置
# ================================================================
class Config:
    # 扫描轨迹 (tool tip)
    scan_start_x = 0.23
    scan_end_x   = 0.32
    scan_y       = 0.0
    scan_z       = 0.2
    approach_z   = 0.05
    scan_vx      = 0.0004

    # 力控
    F_desired    = 0.5
    F_min        = 0.3
    F_max        = 1.0
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
        (0.23, 0.275, 250, 4),    # 低刚度
        (0.275, 0.32, 500, 8),    # 高刚度
    ]

    # 控制频率 (文档要求 1kHz; 仿真可用 100Hz)
    ctrl_rate = 100
    dt = 1.0 / ctrl_rate

    # 泄漏积分器 (标量)
    eps_r = 1.0
    eps_f = 2.0

    settle_time = 2.0
    # RCM 接近阶段限速。先慢速对齐 x/y，再下降，避免参考点跳变导致
    # tool 速度过大和 RCM 误差突增。
    approach_xy_speed = 0.001      # m/s
    approach_z_speed = 0.0002      # m/s
    approach_xy_tolerance = 0.001  # m
    approach_force_threshold = 0.3 # N
    approach_z_margin = 0.002      # m
    approach_timeout = 90.0        # s, 会按起始距离自动放大
    approach_surface_margin = 0.0005      # m
    approach_follow_tolerance = 0.0008    # m
    approach_rcm_soft_limit = 0.0025      # m
    approach_rcm_warn = 0.004             # m
    approach_tau_norm_limit = 3.0
    approach_retarget_log_interval = 50
    approach_start_speed_limit = 0.003    # m/s
    approach_start_settle_hold = 0.5      # s
    approach_start_settle_timeout = 5.0   # s
    scan_contact_bias = 0.0010            # m, virtual contact preload
    scan_z_ref_rate = 0.00008             # m/s, adaptive contact-depth slew rate
    scan_z_ref_min_offset = -0.0012       # m, deepest desired tool z relative to contact z
    scan_z_ref_max_offset = 0.0004        # m, shallowest desired tool z relative to contact z
    scan_force_deadband = 0.05            # N, avoids integral chatter around F_desired
    scan_force_error_limit = 0.35         # N, cap force error fed to controller
    scan_sigma_force_limit = 0.08         # Ns, cap force integral fed to controller
    scan_min_alpha_when_rcm_soft = 0.85
    scan_rcm_recovery_enter = 0.0050      # m
    scan_rcm_recovery_exit = 0.0028       # m
    scan_rcm_recovery_alpha = 0.95
    scan_rcm_recovery_timeout = 12.0      # s
    scan_stop_after_rcm_recovery = True
    scan_timeout = 260.0                  # s
    scan_track_slow_error = 0.0040        # m
    scan_track_pause_error = 0.0070       # m
    scan_rcm_slow_error = 0.0035          # m
    scan_rcm_pause_error = 0.0065         # m
    scan_force_slow = 0.75                # N
    scan_force_pause = 1.10               # N
    scan_tau_norm_limit = 2.5
    scan_abort_pos_error = 0.0080         # m
    scan_abort_rcm_error = 0.0075         # m
    scan_abort_force = 1.20               # N


def read_contact_force(force_sensor, venv, x, delta, delta_dot):
    """
    获取当前接触力。

    优先使用新鲜的六维力传感器数据；若传感器未启动、话题无数据或超时，
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


def slew_toward(current, target, max_step):
    """按最大步长推进参考点，避免目标位置阶跃。"""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    delta = target - current
    dist = np.linalg.norm(delta)
    if dist <= max_step or dist < 1e-12:
        return target.copy()
    return current + delta * (max_step / dist)


def clamp_norm(vec, limit):
    """限制向量范数，保持方向不变。"""
    vec = np.asarray(vec, dtype=float)
    norm = np.linalg.norm(vec)
    if norm <= limit or norm < 1e-12:
        return vec
    return vec * (float(limit) / norm)


def wait_for_tool_settle(robot, kin_tool, kin_flange, cfg, rate):
    """进入 approach 前等待当前位置速度变小，避免带残余速度开始接近。"""
    start = time.time()
    stable_start = None
    hold_joints = None
    try:
        hold_joints = robot.angles()
    except Exception:
        hold_joints = None

    while not rospy.is_shutdown():
        rs = update_robot_state(kin_tool, kin_flange)
        speed = float(np.linalg.norm(rs["tool_position_velocity"]))
        now = time.time()
        if speed <= cfg.approach_start_speed_limit:
            if stable_start is None:
                stable_start = now
            if now - stable_start >= cfg.approach_start_settle_hold:
                rospy.loginfo(
                    f"  Tool settled before approach: |v|={speed*1000:.2f}mm/s"
                )
                return rs
        else:
            stable_start = None

        if hold_joints is not None:
            try:
                robot.exec_position_cmd(hold_joints)
            except Exception:
                pass

        if now - start >= cfg.approach_start_settle_timeout:
            rospy.logwarn(
                f"  Tool did not fully settle before approach "
                f"(|v|={speed*1000:.2f}mm/s); continuing with rate-limited reference."
            )
            return rs
        rate.sleep()


# ================================================================
# 单次试验
# ================================================================
def run_trial(robot, kin_tool, kin_flange,
              cfg, ctrl, sched, est, venv, force_sensor,
              logger, trial_id, approach_logger=None):
    rate = rospy.Rate(cfg.ctrl_rate)
    dt = cfg.dt

    # 泄漏积分器状态 (仅 σ_f, e_r1 直接由 x−x_r 计算)
    sigma_f_int = LeakyIntegrator(eps=cfg.eps_f, dt=dt, dim=3)
    integ_euler = np.zeros(3)

    # 差分导数缓存
    prev_ef = 0.0
    prev_er = 0.0
    prev_K = 500.0

    est.reset()
    sched.reset()
    sigma_f_int.reset()

    rospy.loginfo(f"[Trial {trial_id}] {sched.name} (RCM)")

    # ---- Phase 0: 到起点 ----
    safe_move_to_joint_position(robot, INIT_JOINTS)
    rospy.sleep(cfg.settle_time)
    rs = wait_for_tool_settle(robot, kin_tool, kin_flange, cfg, rate)
    rospy.loginfo(f"  Tool at: {rs['tool_position']}")

    # ---- Phase 1: 下降接近 ----
    rospy.loginfo("  Phase 1: Approaching...")
    z_contact = None
    t0 = time.time()
    last_approach_time = t0
    approach_ref = rs["tool_position"].copy()
    target_xy = np.array([cfg.scan_start_x, cfg.scan_y], dtype=float)
    if approach_ref[2] <= cfg.scan_z + cfg.approach_z_margin:
        contact_z_threshold = approach_ref[2] - cfg.approach_surface_margin
        rospy.logwarn(
            f"  Current tool z={approach_ref[2]:.4f}m is already below "
            f"configured scan_z={cfg.scan_z:.4f}m; using local approach "
            f"threshold z={contact_z_threshold:.4f}m to avoid immediate "
            "contact-triggered scan start."
        )
    else:
        contact_z_threshold = cfg.scan_z
    approach_count = 0
    approach_distance_xy = np.linalg.norm(approach_ref[:2] - target_xy)
    approach_height = max(0.0, approach_ref[2] - contact_z_threshold)
    approach_timeout = max(
        cfg.approach_timeout,
        1.5 * approach_distance_xy / max(cfg.approach_xy_speed, 1e-9)
        + 1.5 * approach_height / max(cfg.approach_z_speed, 1e-9)
        + 10.0,
    )
    rospy.loginfo(
        f"  Approach reference starts at tool={approach_ref}, "
        f"target_xy=[{target_xy[0]:.4f}, {target_xy[1]:.4f}], "
        f"contact_z={contact_z_threshold:.4f}, "
        f"xy_speed={cfg.approach_xy_speed:.4f}m/s, "
        f"z_speed={cfg.approach_z_speed:.4f}m/s, timeout={approach_timeout:.1f}s"
    )

    while not rospy.is_shutdown() and z_contact is None:
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]
        flange_pos = rs["flange_position"]
        rcm_now = np.linalg.norm(
            cfg.trocar_position
            - compute_position_rcm(tp, flange_pos, cfg.trocar_position)
        )

        delta = max(0, contact_z_threshold - tp[2])
        F_z, wrench, force_source, sensor_available = read_contact_force(
            force_sensor, venv, tp[0], delta, -tv[2]
        )

        now = time.time()
        loop_dt = now - last_approach_time
        if loop_dt <= 0.0 or not np.isfinite(loop_dt):
            loop_dt = dt
        loop_dt = min(max(loop_dt, dt), 0.1)
        last_approach_time = now

        prev_ref = approach_ref.copy()
        xy_ref_err = np.linalg.norm(approach_ref[:2] - target_xy)
        xy_actual_err = np.linalg.norm(tp[:2] - target_xy)
        xy_aligned = (
            xy_ref_err <= cfg.approach_xy_tolerance
            and xy_actual_err <= cfg.approach_xy_tolerance
        )

        height_margin = min(cfg.approach_z_margin, 0.0001)
        force_contact = abs(F_z) > cfg.approach_force_threshold
        height_contact = tp[2] <= contact_z_threshold + height_margin
        if force_contact and not xy_aligned:
            rospy.logwarn(
                f"  Force contact reached before xy alignment "
                f"(xy_err={xy_actual_err*1000:.2f}mm, "
                f"rcm={rcm_now*1000:.2f}mm, F={F_z:.3f}N). "
                "Stopping this trial instead of dragging laterally under contact."
            )
            return False
        if height_contact and not xy_aligned:
            contact_z_threshold = min(
                contact_z_threshold,
                tp[2] - cfg.approach_surface_margin,
            )
            if approach_count % cfg.approach_retarget_log_interval == 0:
                rospy.logwarn(
                    f"  Approach height threshold reached before xy alignment "
                    f"without force contact; retarget contact_z to "
                    f"{contact_z_threshold:.4f}m and continue alignment "
                    f"(xy_err={xy_actual_err*1000:.2f}mm, "
                    f"rcm={rcm_now*1000:.2f}mm)."
                )
            height_contact = False
        if (force_contact or height_contact) and xy_aligned:
            z_contact = tp[2]
            rospy.loginfo(
                f"  Contact at z={z_contact:.4f}, F={F_z:.3f}N "
                f"({force_source}), rcm={rcm_now*1000:.2f}mm"
            )
            break

        follow_err = np.linalg.norm(tp - approach_ref)
        can_advance = (
            follow_err <= cfg.approach_follow_tolerance
            and rcm_now <= cfg.approach_rcm_soft_limit
        )

        if not can_advance:
            approach_stage = "settle_rcm"
        elif xy_aligned:
            approach_stage = "descend"
            approach_ref[:2] = target_xy
            approach_ref[2] = max(
                contact_z_threshold,
                approach_ref[2] - cfg.approach_z_speed * loop_dt,
            )
        else:
            approach_stage = "align_xy"
            approach_ref[:2] = slew_toward(
                approach_ref[:2],
                target_xy,
                cfg.approach_xy_speed * loop_dt,
            )

        # 纯位控接近 (α=1) — RCM flange-space 控制。参考轨迹显式限速，
        # 避免从当前 tool 位姿瞬间跳到扫描起点。
        x_tool_ref = approach_ref.copy()
        xdot_tool_ref = (approach_ref - prev_ref) / loop_dt

        # 自由空间: 力误差为 0, 力积分不更新
        e_f = np.zeros(3)
        sigma_f = sigma_f_int.get()

        tau, _, K_eff, integ_euler, _, error = compute_torque_with_rcm(
            ctrl, rs, kin_flange,
            x_tool_ref=x_tool_ref, xdot_tool_ref=xdot_tool_ref,
            e_f=e_f, sigma_f=sigma_f,
            alpha=1.0, K_e_hat=500,
            trocar_pos=cfg.trocar_position,
            length=cfg.tool_length,
            integ_euler=integ_euler, dt=dt,
        )
        tau = clamp_norm(tau, cfg.approach_tau_norm_limit)
        robot.exec_torque_cmd(tau)
        rate.sleep()
        approach_count += 1
        if approach_logger is not None:
            pos_err = tp - x_tool_ref
            approach_logger.log(
                t=time.time() - t0,
                pos=tp.copy(),
                pos_des=x_tool_ref.copy(),
                pos_err=pos_err,
                F_measured=abs(F_z),
                F_desired=0.0,
                F_err=abs(F_z),
                wrench=wrench,
                force_source=force_source,
                sensor_available=int(sensor_available),
                e_f=0.0,
                e_r=np.linalg.norm(pos_err),
                sigma_f_norm=0.0,
                e_r1_norm=follow_err,
                alpha=1.0,
                K_hat=500.0,
                K_eff=K_eff,
                x_desired=x_tool_ref[0],
                error_rcm=error[0],
                error_track=error[1],
                arbitration_strategy=sched.name,
                u_norm=np.linalg.norm(tau),
                phase=-2,
            )

        if approach_count % 10 == 0:
            rospy.loginfo(
                f"  approaching({approach_stage}) | "
                f"tool=[{tp[0]*1000:6.2f},{tp[1]*1000:6.2f},{tp[2]*1000:6.2f}]mm | "
                f"ref=[{x_tool_ref[0]*1000:6.2f},{x_tool_ref[1]*1000:6.2f},{x_tool_ref[2]*1000:6.2f}]mm | "
                f"|v|={np.linalg.norm(tv)*1000:.2f}mm/s | "
                f"follow={follow_err*1000:.2f}mm xy_err={xy_actual_err*1000:.2f}mm | "
                f"rcm={error[0]*1000:.2f}mm | F={F_z:.3f}N"
            )
        if error[0] > cfg.approach_rcm_warn and approach_count % 50 == 0:
            rospy.logwarn(
                f"  Approach RCM error is high: {error[0]*1000:.2f}mm. "
                "Reference motion remains rate-limited; check the initial pose "
                "if this persists."
            )

        if time.time() - t0 > approach_timeout:
            rospy.logwarn(
                f"  Approach timeout "
                f"(stage={approach_stage}, tool={tp}, ref={x_tool_ref}, "
                f"F={F_z:.3f}N, source={force_source}, rcm={error[0]:.4f}m)"
            )
            return False

    if z_contact is None:
        z_contact = cfg.scan_z

    # 重置积分器状态 (进入 Phase 2)
    sigma_f_int.reset()
    integ_euler = np.zeros(3)

    # ---- Phase 2: 恒力扫描 ----
    rospy.loginfo("  Phase 2: Scanning...")
    x_cur = cfg.scan_start_x
    scan_z_ref = z_contact
    prev_scan_z_ref = scan_z_ref
    scan_rcm_recovery = False
    scan_rcm_recovery_t0 = None
    t_scan = time.time()
    last_scan_time = t_scan

    while not rospy.is_shutdown() and x_cur < cfg.scan_end_x:
        t = time.time() - t_scan
        if t > cfg.scan_timeout:
            rospy.logwarn(
                f"  Scan timeout at x={x_cur:.4f}m; saving partial data."
            )
            break
        rs = update_robot_state(kin_tool, kin_flange)
        tp = rs["tool_position"]
        tv = rs["tool_position_velocity"]
        flange_pos = rs["flange_position"]

        now = time.time()
        loop_dt = now - last_scan_time
        if loop_dt <= 0.0 or not np.isfinite(loop_dt):
            loop_dt = dt
        loop_dt = min(max(loop_dt, dt), 0.1)
        last_scan_time = now

        # 接触力: 传感器优先, 不可用时回退到虚拟环境
        delta = max(0, z_contact - tp[2] + cfg.scan_contact_bias)
        F_z, wrench, force_source, sensor_available = read_contact_force(
            force_sensor, venv, tp[0], delta, -tv[2]
        )

        # 环境估计
        K_hat, B_hat = est.update(
            abs(F_z),
            delta,
            -tv[2]
        )

        F_actual = abs(F_z)
        if force_source == "virtual" and venv is not None:
            K_for_z, _ = venv.get_stiffness(x_cur)
        else:
            K_for_z = K_hat
        K_for_z = float(np.clip(K_for_z, 50.0, 5000.0))
        scan_z_target = z_contact + cfg.scan_contact_bias - cfg.F_desired / K_for_z
        scan_z_target = float(np.clip(
            scan_z_target,
            z_contact + cfg.scan_z_ref_min_offset,
            z_contact + cfg.scan_z_ref_max_offset,
        ))
        z_step = cfg.scan_z_ref_rate * loop_dt
        prev_scan_z_ref = scan_z_ref
        scan_z_ref += float(np.clip(scan_z_target - scan_z_ref, -z_step, z_step))

        rcm_now = np.linalg.norm(
            cfg.trocar_position
            - compute_position_rcm(tp, flange_pos, cfg.trocar_position)
        )
        if scan_rcm_recovery:
            if (
                rcm_now <= cfg.scan_rcm_recovery_exit
                and cfg.F_min <= F_actual <= cfg.F_max
            ):
                scan_rcm_recovery = False
                scan_rcm_recovery_t0 = None
                rospy.loginfo(
                    f"  RCM recovery complete "
                    f"(rcm={rcm_now*1000:.2f}mm, F={F_actual:.3f}N)."
                )
                if cfg.scan_stop_after_rcm_recovery:
                    rospy.loginfo(
                        "  Ending scan after successful RCM recovery; "
                        "saving stable partial trial."
                    )
                    break
        elif rcm_now >= cfg.scan_rcm_recovery_enter:
            scan_rcm_recovery = True
            scan_rcm_recovery_t0 = t
            rospy.logwarn(
                f"  Enter RCM recovery: rcm={rcm_now*1000:.2f}mm, "
                f"F={F_actual:.3f}N. Pausing scan and unloading contact."
            )

        if scan_rcm_recovery:
            unload_z = z_contact + cfg.scan_z_ref_max_offset
            scan_z_ref += float(np.clip(unload_z - scan_z_ref, -z_step, z_step))
            if (
                scan_rcm_recovery_t0 is not None
                and t - scan_rcm_recovery_t0 > cfg.scan_rcm_recovery_timeout
                and rcm_now > cfg.scan_rcm_recovery_enter
            ):
                rospy.logwarn(
                    "  RCM recovery timeout; stopping scan before oscillation "
                    f"(rcm={rcm_now*1000:.2f}mm, F={F_actual:.3f}N)."
                )
                break

        scan_z_vel = (scan_z_ref - prev_scan_z_ref) / loop_dt
        xy_track_err = np.linalg.norm(tp[:2] - np.array([x_cur, cfg.scan_y]))
        pos_track_err = np.linalg.norm(tp - np.array([x_cur, cfg.scan_y, scan_z_ref]))

        def _slowdown(value, slow_at, pause_at):
            if value <= slow_at:
                return 1.0
            if value >= pause_at:
                return 0.0
            return float((pause_at - value) / max(pause_at - slow_at, 1e-9))

        scan_speed_scale = min(
            _slowdown(pos_track_err, cfg.scan_track_slow_error,
                      cfg.scan_track_pause_error),
            _slowdown(rcm_now, cfg.scan_rcm_slow_error,
                      cfg.scan_rcm_pause_error),
            _slowdown(F_actual, cfg.scan_force_slow,
                      cfg.scan_force_pause),
        )
        if scan_rcm_recovery:
            scan_speed_scale = 0.0
        scan_vx_cmd = cfg.scan_vx * scan_speed_scale

        # tool 期望轨迹
        x_tool_ref = np.array([x_cur, cfg.scan_y, scan_z_ref])
        xdot_tool_ref = np.array([scan_vx_cmd, 0.0, scan_z_vel])

        if (
            pos_track_err > cfg.scan_abort_pos_error
            or rcm_now > cfg.scan_abort_rcm_error
            or F_actual > cfg.scan_abort_force
        ):
            rospy.logwarn(
                "  Scan safety limit reached; stopping scan before "
                f"oscillation grows (pos_err={pos_track_err*1000:.2f}mm, "
                f"rcm={rcm_now*1000:.2f}mm, F={F_actual:.3f}N)."
            )
            break

        # 力目标: 向上接触反力 (z+) 为正
        F_des = np.array([0.0, 0.0, cfg.F_desired])
        F_meas = np.array([0.0, 0.0, abs(F_z)])

        # 力误差 + 力误差泄漏积分
        e_f_raw_vec = F_meas - F_des
        e_f_control = np.sign(e_f_raw_vec[2]) * max(
            abs(e_f_raw_vec[2]) - cfg.scan_force_deadband,
            0.0,
        )
        e_f_control = float(np.clip(
            e_f_control,
            -cfg.scan_force_error_limit,
            cfg.scan_force_error_limit,
        ))
        if scan_rcm_recovery:
            e_f_control = 0.0
            sigma_f_int.reset()
        e_f_vec = np.array([0.0, 0.0, e_f_control])
        sigma_f = np.clip(
            sigma_f_int.update(e_f_vec),
            -cfg.scan_sigma_force_limit,
            cfg.scan_sigma_force_limit,
        )

        # 差分导数 (用于 α 调度)
        e_f_scalar = F_actual - cfg.F_desired
        e_f_dot = (e_f_scalar - prev_ef) / dt
        prev_ef = e_f_scalar

        e_r_scalar = xy_track_err
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
            if rcm_now >= cfg.scan_rcm_slow_error:
                alpha = max(alpha, cfg.scan_min_alpha_when_rcm_soft)
            if scan_rcm_recovery:
                alpha = max(alpha, cfg.scan_rcm_recovery_alpha)

        # 博弈控制 (flange-space, 内部计算 flange 期望与误差)
        tau, u_flange, K_eff, integ_euler, x_flange_ref, error = \
            compute_torque_with_rcm(
                ctrl, rs, kin_flange,
                x_tool_ref=x_tool_ref, xdot_tool_ref=xdot_tool_ref,
                e_f=e_f_vec, sigma_f=sigma_f,
                alpha=alpha, K_e_hat=K_hat,
                trocar_pos=cfg.trocar_position,
                length=cfg.tool_length,
                integ_euler=integ_euler, dt=dt,
            )
        tau = clamp_norm(tau, cfg.scan_tau_norm_limit)
        robot.exec_torque_cmd(tau)

        # 推进扫描参考
        x_cur += scan_vx_cmd * dt

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
                f"α={alpha:.2f} | vx={scan_vx_cmd*1000:.2f}mm/s | "
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
            wrench=wrench,
            force_source=force_source,
            sensor_available=int(sensor_available),
            e_f=e_f_scalar, e_r=e_r_scalar,
            sigma_f_norm=np.linalg.norm(sigma_f),
            e_r1_norm=e_r1_flange_norm,
            alpha=alpha, K_hat=K_hat,
            K_eff=K_eff,
            x_desired=x_cur,
            error_rcm=rcm_err, error_track=error[1],
            arbitration_strategy=sched.name,
            u_norm=np.linalg.norm(tau),
            phase=phase_val,
        )
        rate.sleep()

    # ---- Phase 3: 撤退 ----
    rospy.loginfo("  Phase 3: Retreating...")
    if hasattr(sched, 'set_retreat'):
        sched.set_retreat(True)
    safe_move_to_joint_position(robot, INIT_JOINTS)
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
    'continuous_force_margin': lambda: ContinuousForceMarginFuzzyAlphaScheduler(
        dt=0.01,
        F_min=Config.F_min,
        F_max=Config.F_max,
        F_desired=Config.F_desired,
    ),
    'online_priority': lambda: OnlinePriorityAdaptationAlphaScheduler(
        dt=0.01,
        F_min=Config.F_min,
        F_max=Config.F_max,
        F_desired=Config.F_desired,
    ),
}


def auto_plot_results(result_dir, no_show=False):
    """实验结束后自动生成最近结果图；多文件时额外生成策略对照图。"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.abspath(result_dir)
    latest_script = os.path.join(script_dir, "plot_latest_result.py")
    compare_script = os.path.join(script_dir, "plot_arbitration_compare.py")
    common_args = ["--input", result_dir]
    if no_show:
        common_args.append("--no-show")

    npz_files = [
        name for name in os.listdir(result_dir)
        if name.endswith(".npz") and os.path.isfile(os.path.join(result_dir, name))
    ]
    if not npz_files:
        approach_dir = os.path.join(result_dir, "approach_debug")
        approach_npz = []
        if os.path.isdir(approach_dir):
            approach_npz = [
                name for name in os.listdir(approach_dir)
                if name.endswith(".npz") and os.path.isfile(os.path.join(approach_dir, name))
            ]
        if not approach_npz:
            rospy.logwarn(f"No npz result files found for plotting: {result_dir}")
            return
        rospy.logwarn(
            f"No scan npz files found; plotting approach debug data: {approach_dir}"
        )
        common_args = ["--input", approach_dir]
        if no_show:
            common_args.append("--no-show")
        try:
            subprocess.run([sys.executable, latest_script] + common_args, check=False)
        except Exception as exc:
            rospy.logwarn(f"Auto approach plot failed: {exc}")
        return

    try:
        subprocess.run([sys.executable, latest_script] + common_args, check=False)
    except Exception as exc:
        rospy.logwarn(f"Auto plot failed: {exc}")

    if len(npz_files) > 1:
        try:
            subprocess.run([sys.executable, compare_script] + common_args, check=False)
        except Exception as exc:
            rospy.logwarn(f"Auto arbitration comparison failed: {exc}")


# ================================================================
# 主入口
# ================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', default='continuous_force_margin')
    ap.add_argument('--trials', type=int, default=1)
    ap.add_argument('--output-dir', default='results')
    ap.add_argument('--gains-file', default=None)
    ap.add_argument('--controller-mode', default='are',
                    choices=['are', 'pareto_iter'],
                    help='are: 原 4D ARE 查表; pareto_iter: Algorithm 2 迭代 P1/P2')
    ap.add_argument('--force-topic', default='/force_sensor/wrench')
    ap.add_argument('--force-timeout', type=float, default=0.02,
                    help='force topic freshness window; 0.02s matches a 1kHz publisher with margin')
    ap.add_argument('--force-axis', type=int, default=2)
    ap.add_argument('--force-sign', type=float, default=1.0)
    ap.add_argument('--force-wait-timeout', type=float, default=2.0)
    ap.add_argument('--force-expected-rate', type=float, default=1000.0)
    ap.add_argument('--force-node-command-format', default='both')
    ap.add_argument('--force-node-data-source', default='0x33')
    ap.add_argument('--force-node-streaming', dest='force_node_streaming',
                    action='store_true', default=True)
    ap.add_argument('--no-force-node-streaming', dest='force_node_streaming',
                    action='store_false')
    ap.add_argument('--no-force-sensor', action='store_true')
    ap.add_argument('--no-auto-plot', action='store_true',
                    help='实验结束后不自动调用绘图脚本')
    ap.add_argument('--plot-no-show', action='store_true',
                    help='自动绘图时只保存图像，不弹出 matplotlib 窗口')
    args = ap.parse_args()

    rospy.init_node("coop_gt_rcm")
    rospy.loginfo("=" * 60)
    rospy.loginfo("  Cooperative Game Force-Position Experiment (RCM)")
    rospy.loginfo("  Model: ARE/Pareto per-axis gains + K_v baseline + leaky integrators")
    rospy.loginfo("=" * 60)

    robot = PandaArm()
    kin_tool = PandaKinematics(robot, "panda_link10")
    kin_flange = PandaKinematics(robot, "panda_link8")
    rospy.sleep(1.0)

    cfg = Config()
    ctrl = CooperativeGameController(control_mode=args.controller_mode)
    rospy.loginfo(f"Controller mode: {args.controller_mode}")

    # 增益表
    need_precompute = True
    if args.gains_file and os.path.exists(args.gains_file):
        ctrl.load_gains(args.gains_file)
        need_precompute = not ctrl.has_precomputed_gains()
        if need_precompute:
            rospy.logwarn(
                f"Gains file {args.gains_file} does not contain "
                f"{args.controller_mode} data; recomputing."
            )

    if need_precompute:
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
        gains_name = 'coop_gains_rcm.npy'
        if args.controller_mode == 'pareto_iter':
            gains_name = 'coop_gains_rcm_pareto_iter.npy'
        gpath = os.path.join(args.output_dir, gains_name)
        ctrl.save_gains(gpath)
        rospy.loginfo(f"Saved gains to {gpath}")

    venv = VirtualStiffnessSurface(cfg.stiffness_zones)
    force_sensor = None
    if not args.no_force_sensor:
        force_sensor = ForceSensorInput(
            topic=args.force_topic,
            timeout=args.force_timeout,
            force_axis=args.force_axis,
            force_sign=args.force_sign,
        )
        got_first_frame = force_sensor.wait_for_data(args.force_wait_timeout)
        rospy.loginfo(
            f"Force sensor topic: {args.force_topic}, "
            f"axis={args.force_axis}, sign={args.force_sign}, "
            f"timeout={args.force_timeout}s; expected_rate={args.force_expected_rate:g}Hz; "
            f"node_format={args.force_node_command_format}, "
            f"data_source={args.force_node_data_source}, streaming={args.force_node_streaming}; "
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
            approach_lg = DataLogger()
            est = EnvironmentEstimator()
            s.reset()
            ok = run_trial(
                robot, kin_tool, kin_flange,
                cfg, ctrl, s, est, venv, force_sensor,
                lg, t, approach_logger=approach_lg,
            )
            if approach_lg.count:
                approach_dir = os.path.join(odir, "approach_debug")
                os.makedirs(approach_dir, exist_ok=True)
                approach_lg.save(
                    os.path.join(approach_dir, f"approach_{s.name}_t{t:02d}.npz")
                )
            if ok:
                lg.save(os.path.join(odir, f"{s.name}_t{t:02d}.npz"))

    rospy.loginfo(f"\nResults → {odir}")
    if not args.no_auto_plot:
        auto_plot_results(odir, no_show=args.plot_no_show)


if __name__ == '__main__':
    main()
