#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
RCM 约束下的仲裁策略对比实验。

默认依次运行:
  1. 固定仲裁 fixed_0.5
  2. S 形曲线仲裁 s_curve_alpha
  3. force_margin 仲裁 force_margin_alpha

每种策略默认 5 组实验，并保存到同一个结果目录，便于统计绘图脚本读取。
"""
import argparse
import os
from datetime import datetime

import numpy as np
import rospy
from panda_robot import PandaArm, PandaKinematics

from src.alpha_scheduler_gt import (
    FixedAlphaScheduler, ForceMarginFuzzyAlphaScheduler,
    PhaseDetector
)
from src.env_estimator import EnvironmentEstimator
from src.gt_controller import CooperativeGameController
from src.utils import VirtualStiffnessSurface, ForceSensorInput, DataLogger
from src.robot_interface import INIT_JOINTS

from run_with_rcm import Config, run_trial


class SCurveAlphaScheduler:
    """随扫描进程按 S 曲线从位置优先平滑过渡到力-位折中。"""

    def __init__(self, dt=0.01, total_time=45.0,
                 alpha_start=0.85, alpha_end=0.45):
        self.dt = float(dt)
        self.total_time = max(float(total_time), self.dt)
        self.alpha_start = float(alpha_start)
        self.alpha_end = float(alpha_end)
        self.name = "s_curve_alpha"
        self.phase_detector = PhaseDetector(dt=dt)
        self.alpha_history = []
        self.phase_history = []
        self._t = 0.0

    def compute(self, F_norm, e_f=0.0, K_hat=None, e_r=0.0, z_vel=0.0,
                de_f=0.0, dK=0.0, de_r=0.0, **unused):
        phase = self.phase_detector.update(F_norm, z_vel)
        s = np.clip(self._t / self.total_time, 0.0, 1.0)
        smooth = 3.0 * s ** 2 - 2.0 * s ** 3
        alpha = self.alpha_start + (self.alpha_end - self.alpha_start) * smooth
        alpha = float(np.clip(alpha, 0.01, 0.99))
        self._t += self.dt
        self.alpha_history.append(alpha)
        self.phase_history.append(phase.value)
        return alpha

    def set_retreat(self, val=True):
        self.phase_detector.set_retreat(val)

    def reset(self):
        self._t = 0.0
        self.alpha_history = []
        self.phase_history = []
        self.phase_detector.reset()


def make_strategies(cfg):
    scan_duration = max(
        cfg.dt,
        (cfg.scan_end_x - cfg.scan_start_x) / max(cfg.scan_vx, 1e-9),
    )
    return {
        "fixed_05": lambda: FixedAlphaScheduler(0.5),
        "s_curve": lambda: SCurveAlphaScheduler(
            dt=cfg.dt,
            total_time=scan_duration,
            alpha_start=0.85,
            alpha_end=0.45,
        ),
        "force_margin": lambda: ForceMarginFuzzyAlphaScheduler(
            dt=cfg.dt,
            F_min=cfg.F_min,
            F_max=cfg.F_max,
            F_desired=cfg.F_desired,
        ),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--strategies", nargs="+",
                    default=[
                        "fixed_05", "s_curve", "force_margin",
                    ])
    ap.add_argument("--output-dir", default="results")
    ap.add_argument("--gains-file", default=None)
    ap.add_argument("--force-topic", default="/force_sensor/wrench")
    ap.add_argument("--force-timeout", type=float, default=0.02)
    ap.add_argument("--force-axis", type=int, default=2)
    ap.add_argument("--force-sign", type=float, default=1.0)
    ap.add_argument("--force-wait-timeout", type=float, default=2.0)
    ap.add_argument("--no-force-sensor", action="store_true")
    args = ap.parse_args()

    rospy.init_node("coop_gt_rcm_arbitration_compare")
    rospy.loginfo("=" * 60)
    rospy.loginfo("  RCM Arbitration Comparison: fixed / S-curve / force-margin")
    rospy.loginfo("=" * 60)

    cfg = Config()
    strategies = make_strategies(cfg)
    unknown = [name for name in args.strategies if name not in strategies]
    if unknown:
        raise ValueError("Unknown strategies: " + ", ".join(unknown))

    robot = PandaArm()
    kin_tool = PandaKinematics(robot, "panda_link10")
    kin_flange = PandaKinematics(robot, "panda_link8")
    rospy.sleep(1.0)

    ctrl = CooperativeGameController()
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
        gpath = os.path.join(args.output_dir, "coop_gains_rcm_compare.npy")
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
            f"Force sensor: topic={args.force_topic}, first_frame={got_first_frame}, "
            f"seq={force_sensor.seq()}, age={force_sensor.age():.4f}s; "
            f"virtual fallback enabled"
        )
    else:
        rospy.loginfo("Force sensor disabled; using virtual env fallback only")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    odir = os.path.join(args.output_dir, f"rcm_arbitration_compare_{stamp}")
    os.makedirs(odir, exist_ok=True)

    rospy.loginfo(
        f"Experiment config: x=[{cfg.scan_start_x:.3f}, {cfg.scan_end_x:.3f}]m, "
        f"vx={cfg.scan_vx:.4f}m/s, trials={args.trials}, output={odir}"
    )

    for strategy_key in args.strategies:
        for trial_id in range(args.trials):
            if rospy.is_shutdown():
                break
            scheduler = strategies[strategy_key]()
            logger = DataLogger()
            est = EnvironmentEstimator()
            rospy.loginfo(
                f"\n{'=' * 50}\n"
                f"  Strategy: {scheduler.name} ({strategy_key}), trial {trial_id}\n"
                f"{'=' * 50}"
            )
            ok = run_trial(
                robot, kin_tool, kin_flange,
                cfg, ctrl, scheduler, est, venv, force_sensor, logger, trial_id,
            )
            if ok:
                filename = f"{strategy_key}_{scheduler.name}_t{trial_id:02d}.npz"
                logger.save(os.path.join(odir, filename))

    rospy.loginfo(f"\nResults -> {odir}")


if __name__ == "__main__":
    main()
