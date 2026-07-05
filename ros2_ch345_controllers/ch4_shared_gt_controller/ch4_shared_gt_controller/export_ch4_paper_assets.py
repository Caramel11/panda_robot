#!/usr/bin/env python3
"""第4章论文资产导出脚本。

Gazebo 实验输出目录通常分散在 ``results/ch4_shared_gt_gazebo`` 下。该脚本把论文
写作需要引用的文件复制到固定目录，形成稳定的资产结构：

* ``figures/``：单次实验时序图和策略对比图。
* ``tables/``：对比指标 CSV。
* ``metrics/``：summary、window_metrics、plot_summary 等 JSON。
* ``data/``：原始 ``result.npz``，用于后续重画图或重算指标。

脚本最后写出 ``asset_manifest.json``，记录每个导出文件的来源和去向。
"""

import argparse
import json
import re
import shutil
from pathlib import Path


def _slug(text):
    """把场景/策略名称转换为适合文件名的短标签。"""
    text = str(text).strip().lower()
    text = re.sub(r"[^a-z0-9_]+", "_", text)
    return re.sub(r"_+", "_", text).strip("_") or "run"


def _read_json(path):
    """读取 UTF-8 JSON 文件。"""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _copy(src, dst):
    """复制文件；源文件不存在时返回 None，便于兼容未生成的可选资产。"""
    if not src.exists():
        return None
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return str(dst)


def _export_run(run_dir, out_dir):
    """导出单次实验运行目录中的图、表和原始数据。"""
    run_dir = Path(run_dir)
    summary_path = run_dir / "summary.json"
    if summary_path.exists():
        summary = _read_json(summary_path)
    else:
        summary = {}
    scenario = summary.get("scenario", "unknown")
    strategy = summary.get("arbitration_strategy", run_dir.name)
    tag = f"{_slug(scenario)}_{_slug(strategy)}"

    exported = {
        "run_dir": str(run_dir),
        "scenario": scenario,
        "strategy": strategy,
        "summary": _copy(summary_path, out_dir / "metrics" / f"{tag}_summary.json"),
        "window_metrics": _copy(run_dir / "window_metrics.json", out_dir / "metrics" / f"{tag}_window_metrics.json"),
        "result_npz": _copy(run_dir / "result.npz", out_dir / "data" / f"{tag}_result.npz"),
        "timeseries_figure": _copy(run_dir / "ch4_shared_gt_result.png", out_dir / "figures" / f"ch4_{tag}_timeseries.png"),
        "plot_summary": _copy(run_dir / "plot_summary.json", out_dir / "metrics" / f"{tag}_plot_summary.json"),
    }
    return exported


def _discover_runs(root):
    """在结果根目录下递归发现所有 Gazebo 运行目录。"""
    root = Path(root)
    return sorted(p.parent for p in root.rglob("summary.json"))


def main(argv=None):
    """命令行入口：批量导出实验资产并写出 manifest。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo")
    ap.add_argument("--runs", nargs="*", default=None)
    ap.add_argument("--comparison-dir", default=None)
    ap.add_argument(
        "--paper-dir",
        default="/home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets",
    )
    args = ap.parse_args(argv)

    out_dir = Path(args.paper_dir)
    run_dirs = [Path(p) for p in args.runs] if args.runs else _discover_runs(args.root)
    exports = [_export_run(run_dir, out_dir) for run_dir in run_dirs]

    comparison_exports = {}
    if args.comparison_dir:
        comp = Path(args.comparison_dir)
        comparison_exports = {
            "comparison_figure": _copy(comp / "ch4_strategy_comparison.png", out_dir / "figures" / "ch4_strategy_comparison.png"),
            "comparison_csv": _copy(comp / "ch4_metrics_summary.csv", out_dir / "tables" / "ch4_metrics_summary.csv"),
        }

    manifest = {
        "paper_dir": str(out_dir),
        "runs": exports,
        "comparison": comparison_exports,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = out_dir / "asset_manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    print(json.dumps({
        "runs": len(exports),
        "paper_dir": str(out_dir),
        "manifest": str(manifest_path),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
