#!/usr/bin/env python3
"""第4章通用 Gazebo 批量运行脚本。

该脚本按给定的 ``scenario`` 和 ``arbitration_strategy`` 列表顺序启动 Gazebo
launch。每次运行会监听控制器日志中的 ``Saved summary: ...``，确认结果已落盘后
优雅结束当前 Gazebo 进程，再进入下一组实验。

它是较底层的通用 runner；论文推荐矩阵请使用
``run_ch4_comparison_experiments.py``，后者会在本脚本基础上增加实验计划、指标重算
和论文资产导出。
"""

import argparse
import datetime as dt
import json
import os
import re
import signal
import select
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_STRATEGIES = [
    "autonomous_only",
    "direct_accept",
    "fixed_blend",
    "single_sigmoid",
    "dynamic_no_projection",
    "full_method",
]

DEFAULT_SCENARIOS = [
    "mixed_sequence",
]

SUMMARY_RE = re.compile(r"Saved summary:\s*(?P<path>\S+summary\.json)")


def _stamp():
    """生成当前批量实验的时间戳。"""
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def _scenario_seed_offset(scenario):
    """按场景生成确定性 seed 偏移，避免不同字母共享同一噪声模板。"""

    return 1000 * (1 + sum(ord(ch) for ch in str(scenario)) % 997)


def _build_launch_cmd(args, strategy, scenario, repeat_index):
    """根据策略和场景拼出一次 Gazebo launch 命令。"""
    output_dir = Path(args.output_dir)
    if args.group_outputs:
        output_dir = output_dir / f"suite_{args.suite_id}"
    cmd = [
        "ros2",
        "launch",
        "ch4_shared_gt_controller",
        "ch4_shared_gt_gazebo.launch.py",
        f"task_mode:={args.task_mode}",
        f"controller_variant:={args.controller_variant}",
        f"arbitration_strategy:={strategy}",
        f"execution_strategy:={args.execution_strategy}",
        f"execution_alpha_profile:={args.execution_alpha_profile}",
        f"scenario:={scenario}",
        f"task_duration_s:={args.task_duration_s}",
        f"fixed_alpha_hr:={args.fixed_alpha_hr}",
        f"output_dir:={output_dir}",
    ]
    if args.trajectory_scale is not None:
        cmd.append(f"trajectory_scale:={args.trajectory_scale}")
    if args.realistic_env:
        realistic_seed = int(args.realistic_seed) + _scenario_seed_offset(scenario) + int(repeat_index)
        cmd.extend([
            "realistic_env_enabled:=true",
            f"realistic_profile:={args.realistic_profile}",
            f"realistic_seed:={realistic_seed}",
            f"realistic_tau_noise_gain:={args.realistic_tau_noise_gain}",
        ])
    if args.human_input_profile:
        cmd.append(f"human_input_profile:={args.human_input_profile}")
    if args.rviz:
        cmd.append("rviz:=true")
    if args.enable_control is not None:
        cmd.append(f"enable_control:={str(args.enable_control).lower()}")
    if args.gz_args:
        cmd.append(f"gz_args:={args.gz_args}")
    return cmd


def _terminate_process(proc, grace_s=8.0):
    """按 SIGINT -> SIGTERM -> SIGKILL 顺序结束一个 launch 进程组。"""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def _run_one(args, strategy, scenario, repeat_index):
    """运行单个策略/场景组合，并返回该次运行的记录。"""
    cmd = _build_launch_cmd(args, strategy, scenario, repeat_index)
    if args.dry_run:
        return {
            "strategy": strategy,
            "scenario": scenario,
            "repeat": repeat_index,
            "command": " ".join(cmd),
            "status": "dry_run",
        }

    start = time.monotonic()
    print(f"[suite] start strategy={strategy} scenario={scenario} repeat={repeat_index}", flush=True)
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )

    # 控制器保存 summary 后会在 stdout 打印路径；捕获该路径即可判断一次实验完成。
    summary_path = None
    lines_tail = []
    deadline = start + args.timeout_s
    try:
        while time.monotonic() < deadline:
            line = ""
            if proc.stdout is not None:
                ready, _, _ = select.select([proc.stdout], [], [], 0.2)
                if ready:
                    line = proc.stdout.readline()
            if not line:
                if proc.poll() is not None:
                    break
                continue
            sys.stdout.write(line)
            sys.stdout.flush()
            lines_tail.append(line.rstrip())
            lines_tail = lines_tail[-40:]
            match = SUMMARY_RE.search(line)
            if match:
                summary_path = Path(match.group("path"))
                time.sleep(args.settle_after_save_s)
                break
        status = "ok" if summary_path and summary_path.exists() else "timeout"
    finally:
        _terminate_process(proc)

    elapsed = time.monotonic() - start
    return {
        "strategy": strategy,
        "scenario": scenario,
        "repeat": repeat_index,
        "command": " ".join(cmd),
        "status": status,
        "summary": str(summary_path) if summary_path else "",
        "elapsed_s": elapsed,
        "returncode": proc.returncode,
        "tail": lines_tail,
    }


def _parse_list(values, defaults):
    """解析命令行列表，兼容空格分隔和逗号分隔两种写法。"""
    if not values:
        return list(defaults)
    items = []
    for value in values:
        items.extend([part.strip() for part in value.split(",") if part.strip()])
    return items


def main(argv=None):
    """命令行入口：顺序执行批量实验并写出 suite_runs.json。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--strategies", nargs="*", default=None)
    ap.add_argument("--scenarios", nargs="*", default=None)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--task-mode", default="no_rcm", choices=["no_rcm", "with_rcm"])
    ap.add_argument("--controller-variant", default="gt_kf")
    ap.add_argument("--execution-strategy", default="fixed_05")
    ap.add_argument(
        "--execution-alpha-profile",
        default="stable",
        choices=["stable", "stiffness_sensitive", "sensitive", "alpha_stiffness"],
    )
    ap.add_argument("--task-duration-s", type=float, default=24.0)
    ap.add_argument(
        "--trajectory-scale",
        type=float,
        default=None,
        help="圆滑字母轨迹缩放系数；短快可行性筛查建议使用 0.20-0.35",
    )
    ap.add_argument("--fixed-alpha-hr", type=float, default=0.5)
    ap.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo")
    ap.add_argument("--timeout-s", type=float, default=90.0)
    ap.add_argument("--settle-after-save-s", type=float, default=2.0)
    ap.add_argument("--suite-id", default=None)
    ap.add_argument("--group-outputs", action="store_true")
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--enable-control", default=None, type=lambda s: s.lower() in ("1", "true", "yes", "on"))
    ap.add_argument("--gz-args", default=None)
    ap.add_argument("--realistic-env", action="store_true")
    ap.add_argument("--realistic-profile", default="real_env_0607_0213")
    ap.add_argument("--realistic-seed", type=int, default=20260703)
    ap.add_argument("--realistic-tau-noise-gain", type=float, default=0.12)
    ap.add_argument("--human-input-profile", default="")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)

    args.suite_id = args.suite_id or _stamp()
    strategies = _parse_list(args.strategies, DEFAULT_STRATEGIES)
    scenarios = _parse_list(args.scenarios, DEFAULT_SCENARIOS)

    # 按场景 -> 策略 -> 重复次数的顺序运行，便于后续按场景汇总。
    records = []
    for scenario in scenarios:
        for strategy in strategies:
            for repeat_index in range(args.repeats):
                record = _run_one(args, strategy, scenario, repeat_index)
                records.append(record)
                if record["status"] not in ("ok", "dry_run"):
                    print(f"[suite] failed: {json.dumps(record, ensure_ascii=False)}", flush=True)

    # manifest 记录所有命令、状态、summary 路径和失败时的日志尾部，便于复现实验。
    suite_dir = Path(args.output_dir) / f"suite_{args.suite_id}"
    suite_dir.mkdir(parents=True, exist_ok=True)
    manifest = suite_dir / "suite_runs.json"
    with open(manifest, "w", encoding="utf-8") as f:
        json.dump(records, f, indent=2, ensure_ascii=False)

    ok_count = sum(1 for r in records if r["status"] in ("ok", "dry_run"))
    print(json.dumps({
        "suite_id": args.suite_id,
        "runs": len(records),
        "ok": ok_count,
        "manifest": str(manifest),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
