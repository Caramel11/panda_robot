#!/usr/bin/env python3
"""第4章策略/场景对比结果汇总脚本。

本脚本连接 Gazebo 实验输出目录和论文图表。它读取每次运行的
``summary.json``，也可以根据 ``result.npz`` 调用 ``ch4_metrics.py`` 重新计算
按论文时间窗定义的指标。输出包括：

1. 机器可读的 CSV 表格，便于写入论文实验表。
2. 2x2 对比图，便于快速检查不同策略在误差、安全性和成功判据上的差异。
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ch4_shared_gt_controller.ch4_metrics import compute_metrics, load_result


METRIC_KEYS = [
    "tracking_last_rms_m",
    "tracking_last_max_m",
    "R_acc",
    "R_sup",
    "R_project",
    "T_vio_s",
    "T_vio_normal_s",
    "F_peak_N",
    "F_peak_normal_N",
    "S_alpha",
    "reference_jump_max_m",
    "safe_projection_delta_max_m",
]


def _read_summary(run_dir, recompute_window_metrics=False):
    """读取单次实验结果，并按需从原始 NPZ 数据重算论文指标。"""
    run_dir = Path(run_dir)
    path = run_dir / "summary.json"
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if recompute_window_metrics and (run_dir / "result.npz").exists():
        _, npz = load_result(run_dir)
        metrics = compute_metrics(npz, scenario=data.get("scenario", "mixed_sequence"))
        data.update(metrics)
        metrics_path = run_dir / "window_metrics.json"
        with open(metrics_path, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2, ensure_ascii=False)
    data["run_dir"] = str(run_dir)
    data.setdefault("strategy", data.get("arbitration_strategy", run_dir.name))
    data.setdefault("scenario", data.get("scenario", "unknown"))
    return data


def _discover(root):
    """在结果根目录下递归查找所有包含 summary.json 的 Gazebo 运行目录。"""
    root = Path(root)
    return sorted(p.parent for p in root.rglob("summary.json"))


def _write_csv(rows, out_csv):
    """写出论文表格和资产导出脚本使用的指标 CSV。"""
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = ["run_dir", "arbitration_strategy", "scenario", "success"] + METRIC_KEYS
    with open(out_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fields})


def _plot(rows, out_png):
    """绘制四面板策略对比图。"""
    multi_scenario = len({r.get("scenario", "") for r in rows}) > 1
    labels = [
        f"{r.get('scenario', '')}\n{r.get('arbitration_strategy', Path(r['run_dir']).name)}"
        if multi_scenario else r.get("arbitration_strategy", Path(r["run_dir"]).name)
        for r in rows
    ]
    x = np.arange(len(rows), dtype=float)
    width = 0.18

    tracking = np.array([1000.0 * float(r.get("tracking_last_rms_m", 0.0)) for r in rows])
    f_peak = np.array([float(r.get("F_peak_N", 0.0)) for r in rows])
    t_vio = np.array([float(r.get("T_vio_s", 0.0)) for r in rows])
    r_sup = np.array([float(r.get("R_sup", 0.0)) for r in rows])
    r_acc = np.array([float(r.get("R_acc", 0.0)) for r in rows])

    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    ax = axes[0, 0]
    ax.bar(x, tracking, width=0.55)
    ax.axhline(3.0, color="r", linestyle="--", linewidth=1)
    ax.set_ylabel("tracking RMS (mm)")
    ax.set_xticks(x, labels, rotation=20, ha="right")
    ax.grid(True, axis="y", alpha=0.3)

    ax = axes[0, 1]
    ax.bar(x - width / 2, r_acc, width=width, label="R_acc")
    ax.bar(x + width / 2, r_sup, width=width, label="R_sup")
    ax.set_ylabel("ratio")
    ax.set_xticks(x, labels, rotation=20, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()

    ax = axes[1, 0]
    ax.bar(x - width / 2, f_peak, width=width, label="F_peak")
    ax.bar(x + width / 2, t_vio, width=width, label="T_vio")
    ax.set_ylabel("N / s")
    ax.set_xticks(x, labels, rotation=20, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()

    ax = axes[1, 1]
    success = [1 if r.get("success") else 0 for r in rows]
    colors = ["tab:green" if v else "tab:red" for v in success]
    ax.bar(x, success, color=colors, width=0.55)
    ax.set_ylim(0.0, 1.2)
    ax.set_ylabel("success")
    ax.set_xticks(x, labels, rotation=20, ha="right")
    ax.grid(True, axis="y", alpha=0.3)

    fig.suptitle("Chapter 4 arbitration strategy comparison")
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def main(argv=None):
    """命令行入口，解析参数并执行本模块对应的实验、绘图或分析流程。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo")
    ap.add_argument("--runs", nargs="*", default=None, help="Explicit run directories")
    ap.add_argument("--output-dir", default=None)
    ap.add_argument(
        "--recompute-window-metrics",
        action="store_true",
        help="Recompute Chapter 4 window metrics from result.npz before comparing.",
    )
    args = ap.parse_args(argv)

    run_dirs = [Path(p) for p in args.runs] if args.runs else _discover(args.root)
    rows = [_read_summary(p, args.recompute_window_metrics) for p in run_dirs]
    rows.sort(key=lambda r: (r.get("scenario", ""), r.get("arbitration_strategy", ""), r.get("run_dir", "")))

    out_dir = Path(args.output_dir) if args.output_dir else Path(args.root) / "comparison"
    out_csv = out_dir / "ch4_metrics_summary.csv"
    out_png = out_dir / "ch4_strategy_comparison.png"
    _write_csv(rows, out_csv)
    _plot(rows, out_png)

    print(json.dumps({
        "runs": len(rows),
        "csv": str(out_csv),
        "figure": str(out_png),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
