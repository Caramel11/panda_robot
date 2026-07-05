#!/usr/bin/env python3
"""第5章 Gazebo 在线综合实验编排脚本。

本脚本是第5章仿真实验的主入口。它不直接实现机器人控制律，而是批量
调用第4章 Gazebo launch，并通过 `method_switches.py` 为每个论文对比
方法注入不同的参考层/执行层参数。

工作流程：

1. 根据 `--methods` 解析第5章方法矩阵；
2. 对每个 trial 和 method 启动 `ch4_shared_gt_gazebo.launch.py`；
3. 监视输出目录，等待控制器写出 `summary.json` 与 `result.npz`；
4. 关闭本次 Gazebo/控制器进程，避免多个仿真互相干扰；
5. 用 `ch5_metrics.py` 重新计算统一指标；
6. 输出 CSV、PNG 图表和 Markdown 自动报告。

该脚本适合论文中的“在线 Gazebo 实验”和“消融对比实验”。正式论文建议
每个方法至少运行 3 次，然后再用 `ch5_aggregate_results` 做均值/标准差统计。
"""

import argparse
import json
import os
import signal
import select
import subprocess
import sys
import time
from pathlib import Path

from .ch5_metrics import load_run, write_metrics_csv
from .ch5_plotting import plot_metrics_bars, plot_representative
from .method_switches import DEFAULT_GAZEBO_METHODS, resolve_methods


def _scenario_seed_offset(scenario):
    """按场景生成确定性 seed 偏移，避免不同字母共享同一噪声模板。"""

    return 1000 * (1 + sum(ord(ch) for ch in str(scenario)) % 997)


def _existing_run_dirs(root):
    """扫描已有实验目录。

    第4章控制器每完成一次实验会在输出目录下写 `summary.json`。这里用
    `summary.json` 的父目录作为一次 run 的唯一标识，便于启动前后做差集。
    """

    root = Path(root)
    if not root.exists():
        return set()
    return {p.parent for p in root.rglob("summary.json")}


def _new_run_dir(root, before):
    """找出本次 launch 新生成的 run 目录。"""

    after = _existing_run_dirs(root)
    new_dirs = sorted(after - before, key=lambda p: p.stat().st_mtime)
    return new_dirs[-1] if new_dirs else None


def _terminate(proc):
    """优雅终止一个由本脚本启动的 launch 进程组。

    先发 SIGINT，给 ROS2/Gazebo 保存日志和退出的机会；若超时再逐级升级
    到 SIGTERM/SIGKILL。这样比直接 kill 更不容易损坏 `result.npz`。
    """

    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=8.0)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5.0)
        except Exception:
            os.killpg(proc.pid, signal.SIGKILL)


def _cleanup_gazebo():
    """清理 Gazebo 和控制器残留进程。

    第5章批量实验假设当前机器由本脚本独占 Gazebo。在线仿真偶尔会留下
    `ign gazebo` 或 `robot_state_publisher`，如果不清理，下一次 trial 可能
    连到旧 topic 或占用资源，导致结果不可复现。
    """

    patterns = [
        "ign gazebo",
        "gz sim",
        "shared_gt_gazebo_node",
        "robot_state_publisher",
        "ros2 launch ch4_shared",
    ]
    for pattern in patterns:
        subprocess.run(["pkill", "-INT", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    for pattern in patterns:
        subprocess.run(["pkill", "-TERM", "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _run_one(args, method, trial_id, suite_dir):
    """运行一个 method/trial 组合，并返回原始目录和指标。

    Args:
        args: 命令行参数。
        method: `MethodSwitch`，包含第3/4章策略组合。
        trial_id: 当前重复编号。
        suite_dir: 本轮 suite 的总输出目录。

    Returns:
        `(run_dir, metrics)`，其中 `run_dir` 是第4章控制器写出的原始目录，
        `metrics` 是第5章重新计算后的统一指标。
    """

    output_root = suite_dir / "raw"
    output_root.mkdir(parents=True, exist_ok=True)
    before = _existing_run_dirs(output_root)
    # 这里是第5章和第4章代码的关键连接点：第5章方法名不直接传给控制器，
    # 而是展开为 `arbitration_strategy` 和 `execution_strategy`。
    cmd = [
        "ros2",
        "launch",
        "ch4_shared_gt_controller",
        "ch4_shared_gt_gazebo.launch.py",
        f"task_mode:={args.task_mode}",
        f"controller_variant:={args.controller_variant}",
        f"arbitration_strategy:={method.arbitration_strategy}",
        f"execution_strategy:={method.execution_strategy}",
        f"execution_alpha_profile:={args.execution_alpha_profile}",
        f"scenario:={args.scenario}",
        f"fixed_alpha_hr:={method.fixed_alpha_hr}",
        f"fixed_alpha_fp:={method.fixed_alpha_fp}",
        f"task_duration_s:={args.task_duration_s}",
        f"target_tolerance_m:={args.target_tolerance_m}",
        f"output_dir:={output_root}",
        f"rviz:={str(args.rviz).lower()}",
    ]
    if args.trajectory_scale is not None:
        cmd.append(f"trajectory_scale:={args.trajectory_scale}")
    if args.realistic_env:
        realistic_seed = int(args.realistic_seed) + _scenario_seed_offset(args.scenario) + int(trial_id)
        cmd.extend([
            "realistic_env_enabled:=true",
            f"realistic_profile:={args.realistic_profile}",
            f"realistic_seed:={realistic_seed}",
            f"realistic_tau_noise_gain:={args.realistic_tau_noise_gain}",
        ])
    if args.human_input_profile:
        cmd.append(f"human_input_profile:={args.human_input_profile}")
    for key in ("force_desired", "force_min", "force_max", "force_margin", "contact_stiffness_hat"):
        value = getattr(args, key, None)
        if value is not None:
            cmd.append(f"{key}:={value}")
    if args.gz_args:
        cmd.append(f"gz_args:={args.gz_args}")
    print(f"[ch5] running method={method.name} trial={trial_id}: {' '.join(cmd)}", flush=True)

    env = os.environ.copy()
    proc = subprocess.Popen(
        cmd,
        cwd="/home/liu/franka_ros2_ws",
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        bufsize=1,
    )

    deadline = time.time() + args.timeout_s
    last_lines = []
    run_dir = None
    try:
        while time.time() < deadline:
            line = ""
            if proc.stdout:
                ready, _, _ = select.select([proc.stdout], [], [], 0.2)
                if ready:
                    line = proc.stdout.readline()
            if line:
                last_lines.append(line.rstrip())
                last_lines = last_lines[-80:]
                if args.verbose:
                    print(line, end="")
            # 控制器完成 `_finish()` 后会写出 `summary.json` 和 `result.npz`。
            # 两个文件同时存在才认为本次实验数据完整。
            run_dir = _new_run_dir(output_root, before)
            if run_dir is not None and (run_dir / "summary.json").exists() and (run_dir / "result.npz").exists():
                break
            if proc.poll() is not None and not line:
                break
            time.sleep(0.2)
    finally:
        _terminate(proc)
        if args.cleanup_gazebo:
            _cleanup_gazebo()

    if run_dir is None:
        log_path = suite_dir / f"failed_{method.name}_t{trial_id:02d}.log"
        log_path.write_text("\n".join(last_lines), encoding="utf-8")
        raise RuntimeError(f"No summary produced for {method.name} trial {trial_id}; log={log_path}")

    metrics = load_run(run_dir, method=method.name, trial_id=trial_id)
    # 第4章原始 summary 只知道底层策略。这里补写第5章方法名，方便后续
    # 聚合统计和论文表格直接按 `balanced_fixed/full_method` 分组。
    metadata = {
        "ch5_method": method.name,
        "trial_id": trial_id,
        "arbitration_strategy": method.arbitration_strategy,
        "execution_strategy": method.execution_strategy,
    }
    summary_path = run_dir / "summary.json"
    with open(summary_path, "r", encoding="utf-8") as f:
        summary = json.load(f)
    summary.update(metadata)
    summary.update({f"ch5_{k}": v for k, v in metrics.items() if isinstance(v, (float, int, bool))})
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(
        f"[ch5] done {method.name} trial={trial_id}: "
        f"tracking={metrics['tracking_last_rms_m']*1000:.2f}mm "
        f"Fpeak={metrics['F_peak_N']:.3f}N "
        f"Racc={metrics['R_acc']:.3f} Rsup={metrics['R_sup']:.3f} "
        f"success={metrics['success']}",
        flush=True,
    )
    return run_dir, metrics


def _write_report(suite_dir, rows, representative):
    """为单个 suite 生成轻量 Markdown 自动报告。"""

    report = suite_dir / "CH5_GAZEBO_INTEGRATED_REPORT.md"
    lines = [
        "# 第五章 Gazebo 综合实验报告",
        "",
        "本报告由 `ch5_integrated_experiments` 自动生成。",
        "",
        "## 指标汇总",
        "",
        "| method | success | tracking_last_rms_mm | F_peak_N | T_vio_s | R_acc | R_sup | alpha_FP_mean | score |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in rows:
        lines.append(
            f"| {r['method']} | {r['success']} | {1000*float(r['tracking_last_rms_m']):.2f} | "
            f"{float(r['F_peak_N']):.3f} | {float(r['T_vio_s']):.2f} | "
            f"{float(r['R_acc']):.3f} | {float(r['R_sup']):.3f} | "
            f"{float(r['alpha_FP_mean']):.3f} | {float(r['score']):.3f} |"
        )
    lines.extend([
        "",
        "## 图像",
        "",
        f"- 综合指标图：`{suite_dir / 'figures' / 'ch5_metrics_bars.png'}`",
    ])
    if representative:
        lines.append(f"- 典型时序图：`{suite_dir / 'figures' / 'ch5_representative_timeseries.png'}`")
    lines.extend([
        "",
        "## 判据",
        "",
        "- 末段 tracking RMS 与 RCM RMS 使用本次启动传入的 `target_tolerance_m`。",
        "- 末段 tracking max 使用 `1.5 * target_tolerance_m`。",
        "- 力越界时间 <= 0.20 s。",
        "- with-RCM 时末段 RCM RMS：理想仿真 <= 3 mm，实物化仿真 <= 10 mm。",
    ])
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main(argv=None):
    """命令行入口。

    常用示例：
    `ros2 run ch5_integrated_experiments run_ch5_gazebo_suite --methods balanced_fixed,reference_only,execution_only,no_projection,full_method --trials 3`
    """

    ap = argparse.ArgumentParser()
    ap.add_argument("--task-mode", default="no_rcm", choices=["no_rcm", "with_rcm"])
    ap.add_argument("--scenario", default="mixed_sequence")
    ap.add_argument("--controller-variant", default="gt_kf")
    ap.add_argument("--methods", default=",".join(DEFAULT_GAZEBO_METHODS))
    ap.add_argument("--trials", type=int, default=1)
    ap.add_argument("--task-duration-s", type=float, default=24.0)
    ap.add_argument("--target-tolerance-m", type=float, default=0.003)
    ap.add_argument(
        "--trajectory-scale",
        type=float,
        default=None,
        help="USTC smooth-letter trajectory scale; use 0.20 with 18 s for short, fast with-RCM feasibility checks",
    )
    ap.add_argument("--timeout-s", type=float, default=80.0)
    ap.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch5_integrated")
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--gz-args", default="")
    ap.add_argument("--realistic-env", action="store_true")
    ap.add_argument("--realistic-profile", default="real_env_0607_0213")
    ap.add_argument("--realistic-seed", type=int, default=20260703)
    ap.add_argument("--realistic-tau-noise-gain", type=float, default=0.12)
    ap.add_argument(
        "--execution-alpha-profile",
        default="stable",
        choices=["stable", "stiffness_sensitive", "sensitive", "alpha_stiffness"],
        help="continuous_force_margin 的 alpha 调节参数组；默认 stable 保持主实验稳定设置",
    )
    ap.add_argument("--human-input-profile", default="")
    ap.add_argument("--force-desired", type=float, default=None)
    ap.add_argument("--force-min", type=float, default=None)
    ap.add_argument("--force-max", type=float, default=None)
    ap.add_argument("--force-margin", type=float, default=None)
    ap.add_argument("--contact-stiffness-hat", type=float, default=None)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--no-cleanup-gazebo", dest="cleanup_gazebo", action="store_false")
    ap.set_defaults(cleanup_gazebo=True)
    args = ap.parse_args(argv)

    methods = resolve_methods(args.methods.split(","))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    suite_dir = Path(args.output_dir) / f"ch5_gazebo_{args.task_mode}_{args.scenario}_{stamp}"
    suite_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    run_dirs = []

    # 按方法分组运行所有重复。Gazebo effort 模式下，某些故意失败的基线
    # 会在清理窗口内留下较大的状态扰动；先完整采集一个方法的 3 次重复，
    # 可以避免失败基线紧挨着本文方法时污染下一次启动。
    for method in methods:
        for trial_id in range(args.trials):
            run_dir, metrics = _run_one(args, method, trial_id, suite_dir)
            run_dirs.append(run_dir)
            rows.append(metrics)

    metrics_csv = suite_dir / "ch5_gazebo_metrics.csv"
    write_metrics_csv(rows, metrics_csv)
    figures_dir = suite_dir / "figures"
    plot_metrics_bars(rows, figures_dir / "ch5_metrics_bars.png")
    representative = None
    for method_name in ("full_method", methods[-1].name):
        candidates = [r for r, p in zip(rows, run_dirs) if r["method"] == method_name and p.exists()]
        if candidates:
            idx = [r["method"] for r in rows].index(method_name)
            representative = run_dirs[idx]
            break
    if representative:
        plot_representative(representative, figures_dir / "ch5_representative_timeseries.png")
    report = _write_report(suite_dir, rows, representative)

    print(json.dumps({
        "suite_dir": str(suite_dir),
        "metrics_csv": str(metrics_csv),
        "report": str(report),
        "runs": [str(p) for p in run_dirs],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
