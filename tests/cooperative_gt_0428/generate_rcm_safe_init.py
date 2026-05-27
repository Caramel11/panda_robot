#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Generate a real-experiment safe initial joint pose for RCM control.

Target:
  tool link panda_link10 at [0.3, 0.0, 0.1] m while satisfying the RCM
  geometric constraint through the same tool-to-flange mapping used by
  run_with_rcm_real.py.

Default mode is dry-run. Add --execute to move the robot slowly to the IK
solution and print/save the resulting joint angles.
"""
import argparse
import json
import os
import time

import numpy as np
import rospy
from panda_robot import PandaArm, PandaKinematics
from scipy.spatial.transform import Rotation

from src.robot_interface import (
    update_robot_state,
    compute_position_rcm,
    tool_to_flange_full_ref,
)


class Config:
    target_tool = np.array([0.30, 0.0, 0.10], dtype=float)
    trocar_position = np.array([0.30, 0.0, 0.235], dtype=float)
    tool_length = 0.525
    # tool_length = 0.6
    tool_link = "panda_link10"
    flange_link = "panda_link8"

    ctrl_rate = 100
    move_time = 20.0
    hold_time = 2.0
    tool_tol = 0.004
    rcm_tol = 0.004
    joint_tol = 0.015
    max_joint_step = 0.004
    min_tool_z = 0.060


def ordered_joints(robot):
    return np.asarray(robot.angles(), dtype=float)


def joint_dict(robot, q):
    return dict(zip(robot.joint_names(), np.asarray(q, dtype=float).tolist()))


def stop_robot(robot):
    try:
        robot.exec_torque_cmd(np.zeros(7))
    except Exception:
        pass


def rcm_target_flange_pose(cfg):
    x_tool_ref = cfg.target_tool.copy()
    xdot_tool_ref = np.zeros(3)
    x_flange_ref, _, euler_ref = tool_to_flange_full_ref(
        x_tool_ref,
        xdot_tool_ref,
        cfg.trocar_position,
        cfg.tool_length,
    )
    quat_xyzw = Rotation.from_euler("xyz", euler_ref).as_quat()
    return x_flange_ref, quat_xyzw, euler_ref


def solve_safe_init_ik(robot, kin_tool, kin_flange, cfg):
    seed = ordered_joints(robot).tolist()
    x_flange_ref, quat_xyzw, euler_ref = rcm_target_flange_pose(cfg)
    q_sol = kin_flange.inverse_kinematics(
        position=x_flange_ref,
        orientation=quat_xyzw,
        seed=seed,
    )
    if q_sol is None:
        return None, {
            "x_flange_ref": x_flange_ref,
            "euler_ref": euler_ref,
            "reason": "IK failed for RCM-consistent flange pose",
        }

    q_dict = joint_dict(robot, q_sol)
    tool_pose = np.asarray(kin_tool.forward_position_kinematics(q_dict), dtype=float)
    flange_pose = np.asarray(kin_flange.forward_position_kinematics(q_dict), dtype=float)
    tool_pos = tool_pose[:3]
    flange_pos = flange_pose[:3]
    p_rcm = compute_position_rcm(tool_pos, flange_pos, cfg.trocar_position)
    tool_err = float(np.linalg.norm(tool_pos - cfg.target_tool))
    rcm_err = float(np.linalg.norm(p_rcm - cfg.trocar_position))
    return np.asarray(q_sol, dtype=float), {
        "x_flange_ref": x_flange_ref,
        "euler_ref": euler_ref,
        "tool_pos_ik": tool_pos,
        "flange_pos_ik": flange_pos,
        "p_rcm_ik": p_rcm,
        "tool_err": tool_err,
        "rcm_err": rcm_err,
    }


def smooth_move_to_joints(robot, kin_tool, q_target, cfg):
    rate = rospy.Rate(cfg.ctrl_rate)
    q_start = ordered_joints(robot)
    total_steps = max(1, int(cfg.move_time * cfg.ctrl_rate))
    max_delta = float(np.max(np.abs(q_target - q_start)))
    min_steps_by_delta = int(np.ceil(max_delta / max(cfg.max_joint_step, 1e-6)))
    total_steps = max(total_steps, min_steps_by_delta)

    stop_robot(robot)
    rospy.sleep(0.3)
    for step in range(1, total_steps + 1):
        if rospy.is_shutdown():
            return False
        s = step / float(total_steps)
        s_smooth = s * s * (3.0 - 2.0 * s)
        q_cmd = q_start + s_smooth * (q_target - q_start)
        robot.exec_position_cmd(q_cmd.tolist())

        tool_z = float(np.asarray(kin_tool.forward_position_kinematics())[:3][2])
        if tool_z < cfg.min_tool_z:
            rospy.logerr(
                "Safety abort: tool z %.4fm below min_tool_z %.4fm during init move",
                tool_z,
                cfg.min_tool_z,
            )
            stop_robot(robot)
            return False
        rate.sleep()

    rospy.sleep(cfg.hold_time)
    return True


def print_solution(q, metrics):
    rospy.loginfo("IK target metrics:")
    rospy.loginfo("  tool_pos_ik=%s", np.array2string(metrics["tool_pos_ik"], precision=6))
    rospy.loginfo("  p_rcm_ik=%s", np.array2string(metrics["p_rcm_ik"], precision=6))
    rospy.loginfo("  tool_err=%.6fm, rcm_err=%.6fm", metrics["tool_err"], metrics["rcm_err"])
    rospy.loginfo("Candidate joints:")
    print("INIT_JOINTS_REAL_SAFE = (")
    for idx, val in enumerate(q):
        suffix = "," if idx < len(q) - 1 else ","
        print(f"    {val:.8f}{suffix}")
    print(")")


def save_result(path, q, metrics, cfg, final_metrics=None):
    if not path:
        return
    data = {
        "joint_angles": [float(v) for v in q],
        "target_tool": [float(v) for v in cfg.target_tool],
        "trocar_position": [float(v) for v in cfg.trocar_position],
        "tool_length": float(cfg.tool_length),
        "ik_metrics": {
            k: np.asarray(v).tolist() if isinstance(v, np.ndarray) else float(v)
            for k, v in metrics.items()
            if k != "reason"
        },
    }
    if final_metrics is not None:
        data["final_metrics"] = {
            k: np.asarray(v).tolist() if isinstance(v, np.ndarray) else float(v)
            for k, v in final_metrics.items()
        }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    rospy.loginfo("Saved result to %s", path)


def measure_current_metrics(kin_tool, kin_flange, cfg):
    tool_pos = np.asarray(kin_tool.forward_position_kinematics())[:3]
    flange_pos = np.asarray(kin_flange.forward_position_kinematics())[:3]
    p_rcm = compute_position_rcm(tool_pos, flange_pos, cfg.trocar_position)
    return {
        "tool_pos": tool_pos,
        "flange_pos": flange_pos,
        "p_rcm": p_rcm,
        "tool_err": float(np.linalg.norm(tool_pos - cfg.target_tool)),
        "rcm_err": float(np.linalg.norm(p_rcm - cfg.trocar_position)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--execute", action="store_true",
                    help="actually move the robot; without this, only solve/check IK")
    ap.add_argument("--target", nargs=3, type=float, default=Config.target_tool.tolist())
    ap.add_argument("--trocar", nargs=3, type=float, default=Config.trocar_position.tolist())
    ap.add_argument("--tool-length", type=float, default=Config.tool_length)
    ap.add_argument("--move-time", type=float, default=Config.move_time)
    ap.add_argument("--hold-time", type=float, default=Config.hold_time)
    ap.add_argument("--tool-tol", type=float, default=Config.tool_tol)
    ap.add_argument("--rcm-tol", type=float, default=Config.rcm_tol)
    ap.add_argument("--min-tool-z", type=float, default=Config.min_tool_z)
    ap.add_argument("--output", default="results/rcm_safe_init_joints.json")
    args = ap.parse_args()

    rospy.init_node("generate_rcm_safe_init", anonymous=False)

    cfg = Config()
    cfg.target_tool = np.asarray(args.target, dtype=float)
    cfg.trocar_position = np.asarray(args.trocar, dtype=float)
    cfg.tool_length = float(args.tool_length)
    cfg.move_time = float(args.move_time)
    cfg.hold_time = float(args.hold_time)
    cfg.tool_tol = float(args.tool_tol)
    cfg.rcm_tol = float(args.rcm_tol)
    cfg.min_tool_z = float(args.min_tool_z)

    robot = PandaArm()
    kin_tool = PandaKinematics(robot, cfg.tool_link)
    kin_flange = PandaKinematics(robot, cfg.flange_link)
    rospy.sleep(1.0)

    current = measure_current_metrics(kin_tool, kin_flange, cfg)
    rospy.loginfo("Current tool=%s", np.array2string(current["tool_pos"], precision=6))
    rospy.loginfo("Current RCM err=%.6fm", current["rcm_err"])

    q_sol, metrics = solve_safe_init_ik(robot, kin_tool, kin_flange, cfg)
    if q_sol is None:
        rospy.logerr(metrics["reason"])
        return

    print_solution(q_sol, metrics)
    if metrics["tool_err"] > cfg.tool_tol or metrics["rcm_err"] > cfg.rcm_tol:
        rospy.logerr(
            "IK solution rejected: tool_err=%.6fm (tol %.6fm), "
            "rcm_err=%.6fm (tol %.6fm)",
            metrics["tool_err"], cfg.tool_tol, metrics["rcm_err"], cfg.rcm_tol,
        )
        return

    if not args.execute:
        rospy.logwarn("Dry-run only. Re-run with --execute to move and record final joints.")
        save_result(args.output, q_sol, metrics, cfg)
        return

    rospy.logwarn("Executing slow joint-space move to RCM-safe initial pose.")
    ok = smooth_move_to_joints(robot, kin_tool, q_sol, cfg)
    if not ok:
        return

    q_final = ordered_joints(robot)
    final_metrics = measure_current_metrics(kin_tool, kin_flange, cfg)
    rospy.loginfo("Final tool=%s", np.array2string(final_metrics["tool_pos"], precision=6))
    rospy.loginfo(
        "Final tool_err=%.6fm, rcm_err=%.6fm",
        final_metrics["tool_err"], final_metrics["rcm_err"],
    )
    print_solution(q_final, {
        "tool_pos_ik": final_metrics["tool_pos"],
        "p_rcm_ik": final_metrics["p_rcm"],
        "tool_err": final_metrics["tool_err"],
        "rcm_err": final_metrics["rcm_err"],
    })
    save_result(args.output, q_final, metrics, cfg, final_metrics=final_metrics)
    stop_robot(robot)


if __name__ == "__main__":
    main()
