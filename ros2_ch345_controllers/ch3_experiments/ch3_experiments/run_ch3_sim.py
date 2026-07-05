"""第三章纯数值仿真批量入口。

运行后会按控制器和重复次数生成 NPZ，再复用 no_rcm 分析脚本输出指标、
图表和 Markdown 报告。该入口适合快速检查方法趋势，不需要启动 Gazebo。
"""

import argparse
from datetime import datetime
from pathlib import Path

from .ch3_contact_sim import run_contact_simulation
from .ch3_metrics import analyze_with_no_rcm
from .config import load_config


def main():
    """解析命令行、执行仿真矩阵并触发分析。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    parser.add_argument("--controllers", default=None, help="comma separated controller names")
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument(
        "--output-dir",
        default="/home/liu/franka_ros2_ws/results/ch3_experiments_sim",
    )
    parser.add_argument("--no-analysis", action="store_true")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.controllers:
        cfg.controllers = tuple(item.strip() for item in args.controllers.split(",") if item.strip())
    if args.repeats is not None:
        cfg.repeats = int(args.repeats)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = Path(args.output_dir) / f"ch3_sim_{stamp}"
    result_dir.mkdir(parents=True, exist_ok=True)

    for controller in cfg.controllers:
        for trial in range(cfg.repeats):
            out = result_dir / f"{controller}_t{trial:02d}.npz"
            run_contact_simulation(cfg, controller, trial_index=trial, output_file=out)
            print(f"saved {out}")

    if not args.no_analysis:
        analysis = analyze_with_no_rcm(result_dir, result_dir / "ch3_analysis")
        print(f"report: {analysis['report']}")
    print(f"result_dir: {result_dir}")


if __name__ == "__main__":
    main()
