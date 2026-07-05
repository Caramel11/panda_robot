#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第三章 RCM 约束 Gazebo 在线实验入口。

该脚本复用稳定的 effort 控制后端，运行第三章带 RCM 约束的控制器，
并自动调用 RCM 结果分析脚本。它对应论文中“RCM 约束是否被满足”的对照实验。
"""

import argparse
from pathlib import Path

from .gazebo_rcm_adapter import (
    analyze_latest_result,
    run_rcm_strategy,
    start_gazebo_rcm,
    stop_process_group,
    wait_for_controller,
)


def main():
    """解析命令行、启动 RCM 仿真、运行策略并生成分析报告。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--strategies", default="fixed_05")
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--controller-mode", default="pareto_iter")
    parser.add_argument(
        "--output-dir",
        default="/home/liu/franka_ros2_ws/results/ch3_experiments_rcm_gazebo",
    )
    parser.add_argument("--skip-launch", action="store_true", help="use an already running Gazebo/RCM controller")
    parser.add_argument("--no-gui", action="store_true", help="kept for CLI symmetry; current RCM launch is visual")
    parser.add_argument("--use-force-sensor", action="store_true", help="use /force_sensor/wrench instead of virtual force")
    parser.add_argument("--timeout", type=float, default=360.0)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    gazebo_proc = None
    try:
        if not args.skip_launch:
            gazebo_log = output_dir / "gazebo_rcm_launch.log"
            gazebo_proc = start_gazebo_rcm(gui=not args.no_gui, log_file=gazebo_log)
            print(f"Gazebo RCM launch log: {gazebo_log}", flush=True)
            ok, controllers = wait_for_controller(process=gazebo_proc)
            print(controllers, flush=True)
            if not ok:
                raise RuntimeError("no_rcm_effort_controller did not become active")

        for strategy in [s.strip() for s in args.strategies.split(",") if s.strip()]:
            print(f"running Gazebo/RCM strategy: {strategy}", flush=True)
            run = run_rcm_strategy(
                strategy,
                output_dir,
                trials=args.trials,
                controller_mode=args.controller_mode,
                timeout=args.timeout,
                use_force_sensor=args.use_force_sensor,
            )
            print(run.stdout, flush=True)
            if run.returncode != 0:
                raise RuntimeError(f"RCM controller failed for {strategy} with code {run.returncode}")

        latest, analysis = analyze_latest_result(output_dir)
        print(analysis.stdout, flush=True)
        if analysis.returncode != 0:
            raise RuntimeError(f"RCM analysis failed with code {analysis.returncode}")
        print(f"rcm_gazebo_result_dir: {latest}", flush=True)
        print(
            f"rcm_gazebo_report: {latest / 'ch3_rcm_analysis/analysis/ch3_rcm_analysis_report.md'}",
            flush=True,
        )
    finally:
        if gazebo_proc is not None:
            stop_process_group(gazebo_proc)


if __name__ == "__main__":
    main()
