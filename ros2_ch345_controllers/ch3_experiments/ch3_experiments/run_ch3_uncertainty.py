"""第三章鲁棒性/不确定性扫描实验入口。

该脚本在纯 Python 仿真中系统改变环境刚度比例和力噪声标准差，
用于验证控制器对柔性表面参数偏差和传感噪声的鲁棒性。输出包括每个
不确定性组合的 NPZ、指标 CSV 和 Markdown 摘要。
"""

import argparse
import csv
from copy import deepcopy
from datetime import datetime
from pathlib import Path

from .ch3_contact_sim import run_contact_simulation
from .ch3_metrics import analyze_with_no_rcm
from .config import _try_yaml_load, load_config


def parse_float_list(text):
    """解析逗号分隔浮点列表。"""

    return [float(item.strip()) for item in str(text).split(",") if item.strip()]


def read_uncertainty_defaults(config_path):
    """从 YAML 中读取不确定性扫描默认网格。"""

    if not config_path:
        return {}, {}
    data = _try_yaml_load(config_path) or {}
    return data.get("uncertainty", {}), data.get("controllers", {})


def write_rows(path, rows):
    """把不确定性扫描的汇总行写入 CSV。"""

    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_summary(summary_path, variant):
    """读取单个 variant 的策略汇总并补上 variant 名。"""

    if not summary_path.exists():
        return []
    rows = []
    with summary_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out = {"variant": variant}
            out.update(row)
            rows.append(out)
    return rows


def write_uncertainty_report(path, manifest_rows):
    """生成鲁棒性扫描 Markdown 报告。"""

    lines = [
        "# Chapter 3 Uncertainty Sweep Report",
        "",
        "This report summarizes pure Python Chapter 3 uncertainty sweeps.",
        "It is intended to support the thesis robustness experiment and does not replace Gazebo validation.",
        "",
        "## Experiment Meaning",
        "",
        "- `stiffness_scale` changes all Kelvin-Voigt stiffness/damping values and tests model/environment mismatch.",
        "- `force_noise_std` changes force measurement noise and tests force loop robustness.",
        "- The expected robust controller should keep force violation time small and avoid large force jitter growth.",
        "",
        "## Manifest",
        "",
    ]
    if manifest_rows:
        headers = [
            "variant",
            "strategy",
            "force_rmse_N_mean",
            "force_violation_time_s_mean",
            "force_jitter_N_mean",
            "tangent_pos_rmse_mm_mean",
            "alpha_mean_mean",
        ]
        lines.append("| " + " | ".join(headers) + " |")
        lines.append("| " + " | ".join(["---"] * len(headers)) + " |")
        for row in manifest_rows:
            values = [str(row.get(key, "")) for key in headers]
            lines.append("| " + " | ".join(values) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    """执行刚度/噪声参数网格并汇总全部结果。"""

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="/home/liu/franka_ros2_ws/src/ch3_experiments/config/ch3_uncertainty.yaml",
    )
    parser.add_argument("--controllers", default=None)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--stiffness-scales", default=None)
    parser.add_argument("--force-noise-stds", default=None)
    parser.add_argument(
        "--output-dir",
        default="/home/liu/franka_ros2_ws/results/ch3_experiments_uncertainty",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    defaults, controller_defaults = read_uncertainty_defaults(args.config)
    if args.controllers:
        cfg.controllers = tuple(item.strip() for item in args.controllers.split(",") if item.strip())
    elif "list" in controller_defaults:
        cfg.controllers = tuple(str(item) for item in controller_defaults["list"])
    if args.repeats is not None:
        cfg.repeats = int(args.repeats)

    stiffness_scales = (
        parse_float_list(args.stiffness_scales)
        if args.stiffness_scales
        else [float(v) for v in defaults.get("stiffness_scales", [1.0])]
    )
    force_noise_stds = (
        parse_float_list(args.force_noise_stds)
        if args.force_noise_stds
        else [float(v) for v in defaults.get("force_noise_stds", [cfg.force_noise_std])]
    )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(args.output_dir) / f"ch3_uncertainty_{stamp}"
    root.mkdir(parents=True, exist_ok=True)

    manifest_rows = []
    for scale in stiffness_scales:
        for noise in force_noise_stds:
            variant = f"k{scale:.2f}_noise{noise:.3f}".replace(".", "p")
            variant_dir = root / variant
            variant_dir.mkdir(parents=True, exist_ok=True)

            variant_cfg = deepcopy(cfg)
            variant_cfg.force_noise_std = float(noise)
            for zone in variant_cfg.zones:
                zone.stiffness *= float(scale)
                zone.damping *= float(scale)
                zone.label = f"{zone.label}_s{scale:.2f}"

            for controller in variant_cfg.controllers:
                for trial in range(variant_cfg.repeats):
                    out = variant_dir / f"{controller}_t{trial:02d}.npz"
                    run_contact_simulation(variant_cfg, controller, trial_index=trial, output_file=out)
                    print(f"saved {out}")

            analysis = analyze_with_no_rcm(variant_dir, variant_dir / "ch3_analysis")
            manifest_rows.extend(read_summary(analysis["summary"], variant))

    write_rows(root / "ch3_uncertainty_manifest.csv", manifest_rows)
    write_uncertainty_report(root / "CH3_UNCERTAINTY_SWEEP_REPORT.md", manifest_rows)
    print(f"uncertainty_root: {root}")
    print(f"manifest: {root / 'ch3_uncertainty_manifest.csv'}")
    print(f"report: {root / 'CH3_UNCERTAINTY_SWEEP_REPORT.md'}")


if __name__ == "__main__":
    main()
