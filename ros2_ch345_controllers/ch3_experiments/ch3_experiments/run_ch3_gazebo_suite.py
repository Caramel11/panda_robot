"""第三章 no-RCM Gazebo 在线实验批量入口。

该入口用于在线验证第三章执行层控制器。它可以自动启动 Gazebo，也可以通过
`--skip-launch` 连接到已经启动好的仿真环境；每个策略运行后会调用分析脚本
生成 CSV、PNG 和 Markdown 报告。
"""

import argparse
from pathlib import Path

from .gazebo_no_rcm_adapter import (
    analyze_latest_result,
    run_no_rcm_strategy,
    start_gazebo_no_rcm,
    stop_process_group,
    wait_for_controller,
)


def main():
    """解析命令行，启动/复用 Gazebo，并逐个运行第三章策略。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--strategies", default="fixed_05")
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--controller-mode", default="pareto_iter")
    parser.add_argument("--benchmark", default="default")
    parser.add_argument(
        "--output-dir",
        default="/home/liu/franka_ros2_ws/results/ch3_experiments_gazebo",
    )
    parser.add_argument("--skip-launch", action="store_true", help="use an already running Gazebo/no_rcm controller")
    parser.add_argument("--gui", action="store_true", help="launch Gazebo GUI instead of server-only")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    gazebo_proc = None
    try:
        if not args.skip_launch:
            gazebo_log = output_dir / "gazebo_launch.log"
            gazebo_proc = start_gazebo_no_rcm(gui=args.gui, log_file=gazebo_log)
            print(f"Gazebo launch log: {gazebo_log}", flush=True)
            if not wait_for_controller(process=gazebo_proc):
                raise RuntimeError("no_rcm_effort_controller did not become active")

        for strategy in [s.strip() for s in args.strategies.split(",") if s.strip()]:
            print(f"running Gazebo/no_rcm strategy: {strategy}", flush=True)
            run = run_no_rcm_strategy(
                strategy,
                output_dir,
                trials=args.trials,
                controller_mode=args.controller_mode,
                benchmark=args.benchmark,
            )
            print(run.stdout)
            if run.returncode != 0:
                raise RuntimeError(f"no_rcm failed for {strategy} with code {run.returncode}")

        latest, analysis = analyze_latest_result(output_dir)
        print(analysis.stdout)
        if analysis.returncode != 0:
            raise RuntimeError(f"analysis failed with code {analysis.returncode}")
        print(f"gazebo_result_dir: {latest}")
        print(f"gazebo_report: {latest / 'ch3_analysis/analysis/ch3_force_position_analysis_report.md'}")
    finally:
        if gazebo_proc is not None:
            stop_process_group(gazebo_proc)


if __name__ == "__main__":
    main()
