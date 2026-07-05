#!/usr/bin/env python3
"""第4章论文对比实验矩阵运行脚本。

本脚本把论文需要的场景和策略组合固化为可复现的实验计划。与
``run_ch4_gazebo_suite.py`` 相比，它不仅启动 Gazebo，还会：

1. 将每个实验编号、场景、策略和主要指标写入 manifest。
2. 在所有运行完成后，按总表和分场景自动重算窗口指标。
3. 自动生成策略对比图和 CSV。
4. 自动导出论文资产目录。

profile 含义：
  ``core``：快速核心验证。
  ``paper``：论文推荐实验矩阵。
  ``all``：5 个场景 × 6 个策略全量消融。
"""

import argparse
import datetime as dt
import json
from pathlib import Path
from types import SimpleNamespace

from ch4_shared_gt_controller.compare_ch4_results import main as compare_main
from ch4_shared_gt_controller.export_ch4_paper_assets import main as export_main
from ch4_shared_gt_controller.run_ch4_gazebo_suite import _run_one


SCENARIOS = [
    "tangential_correction",
    "unsafe_normal_push",
    "short_pulse_disturbance",
    "sustained_intervention",
    "mixed_sequence",
]

STRATEGIES = [
    "autonomous_only",
    "direct_accept",
    "fixed_blend",
    "single_sigmoid",
    "dynamic_no_projection",
    "full_method",
]

EXPERIMENT_PLAN = [
    {
        "experiment_id": "E1_tangential_acceptance",
        "scenario": "tangential_correction",
        "strategies": ["autonomous_only", "direct_accept", "fixed_blend", "single_sigmoid", "full_method"],
        "purpose": "验证安全切向人类修正能否被接受，同时不引起接触力越界。",
        "primary_metrics": ["R_acc", "tracking_last_rms_m", "T_vio_s", "S_alpha"],
    },
    {
        "experiment_id": "E2_unsafe_normal_suppression",
        "scenario": "unsafe_normal_push",
        "strategies": ["direct_accept", "fixed_blend", "single_sigmoid", "dynamic_no_projection", "full_method"],
        "purpose": "验证危险法向压入是否被安全投影与法向权重抑制。",
        "primary_metrics": ["R_sup", "T_vio_normal_s", "F_peak_normal_N", "safe_projection_delta_max_m"],
    },
    {
        "experiment_id": "E3_short_pulse_rejection",
        "scenario": "short_pulse_disturbance",
        "strategies": ["direct_accept", "fixed_blend", "single_sigmoid", "dynamic_no_projection", "full_method"],
        "purpose": "验证短促误操作不会造成过大的参考突变和权重突跳。",
        "primary_metrics": ["reference_jump_max_m", "S_alpha", "alpha_peak_pulse", "tracking_last_rms_m"],
    },
    {
        "experiment_id": "E4_sustained_intervention",
        "scenario": "sustained_intervention",
        "strategies": ["autonomous_only", "fixed_blend", "single_sigmoid", "dynamic_no_projection", "full_method"],
        "purpose": "验证持续明确人类输入可被平滑接管并保持稳定跟踪。",
        "primary_metrics": ["R_acc", "alpha_mean_sustained", "tracking_last_rms_m", "T_vio_s"],
    },
    {
        "experiment_id": "E5_mixed_sequence_ablation",
        "scenario": "mixed_sequence",
        "strategies": STRATEGIES,
        "purpose": "综合切向修正、短扰动、危险法向压入和持续干预，作为论文主对比。",
        "primary_metrics": [
            "tracking_last_rms_m",
            "R_acc",
            "R_sup",
            "T_vio_s",
            "F_peak_N",
            "S_alpha",
            "reference_jump_max_m",
        ],
    },
]

CORE_PLAN = [
    {
        "experiment_id": "C1_mixed_core",
        "scenario": "mixed_sequence",
        "strategies": ["direct_accept", "dynamic_no_projection", "single_sigmoid", "fixed_blend", "full_method"],
        "purpose": "用最少运行次数验证完整方法相对关键消融基线的优势。",
        "primary_metrics": ["tracking_last_rms_m", "R_acc", "R_sup", "T_vio_s", "F_peak_N"],
    },
    {
        "experiment_id": "C2_normal_core",
        "scenario": "unsafe_normal_push",
        "strategies": ["direct_accept", "dynamic_no_projection", "full_method"],
        "purpose": "单独验证安全投影和法向抑制对危险法向输入的作用。",
        "primary_metrics": ["R_sup", "T_vio_normal_s", "F_peak_normal_N"],
    },
]


def _stamp():
    """生成本次对比实验的 suite_id。"""
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def _expand_plan(profile):
    """根据 profile 展开实验计划。"""
    if profile == "core":
        return CORE_PLAN
    if profile == "paper":
        return EXPERIMENT_PLAN
    if profile == "all":
        return [
            {
                "experiment_id": f"A{i + 1}_{scenario}",
                "scenario": scenario,
                "strategies": STRATEGIES,
                "purpose": "全量策略消融验证。",
                "primary_metrics": ["tracking_last_rms_m", "R_acc", "R_sup", "T_vio_s", "F_peak_N", "S_alpha"],
            }
            for i, scenario in enumerate(SCENARIOS)
        ]
    raise ValueError(profile)


def _suite_args(args):
    """把本脚本参数转换为通用 Gazebo suite runner 所需的参数对象。"""
    return SimpleNamespace(
        task_mode=args.task_mode,
        controller_variant=args.controller_variant,
        task_duration_s=args.task_duration_s,
        fixed_alpha_hr=args.fixed_alpha_hr,
        output_dir=args.output_dir,
        timeout_s=args.timeout_s,
        settle_after_save_s=args.settle_after_save_s,
        suite_id=args.suite_id,
        group_outputs=True,
        rviz=args.rviz,
        enable_control=args.enable_control,
        gz_args=args.gz_args,
        dry_run=args.dry_run,
    )


def _record_run(args, experiment, strategy, repeat_index):
    """执行并记录单个实验项。"""
    suite_args = _suite_args(args)
    record = _run_one(suite_args, strategy, experiment["scenario"], repeat_index)
    record.update({
        "experiment_id": experiment["experiment_id"],
        "purpose": experiment["purpose"],
        "primary_metrics": experiment["primary_metrics"],
    })
    return record


def _run_plan(args, plan):
    """按实验计划顺序执行所有场景/策略组合。"""
    records = []
    for experiment in plan:
        for strategy in experiment["strategies"]:
            for repeat_index in range(args.repeats):
                record = _record_run(args, experiment, strategy, repeat_index)
                records.append(record)
                if record["status"] not in ("ok", "dry_run"):
                    print(f"[comparison] failed: {json.dumps(record, ensure_ascii=False)}", flush=True)
    return records


def _summary_dirs(records):
    """从运行记录中提取有效 summary.json 所在目录。"""
    dirs = []
    for record in records:
        summary = record.get("summary")
        if summary:
            dirs.append(str(Path(summary).parent))
    return dirs


def _write_manifest(args, plan, records):
    """写出完整实验计划和运行结果 manifest。"""
    suite_dir = Path(args.output_dir) / f"suite_{args.suite_id}"
    suite_dir.mkdir(parents=True, exist_ok=True)
    manifest = suite_dir / "ch4_comparison_experiment_manifest.json"
    payload = {
        "suite_id": args.suite_id,
        "profile": args.profile,
        "task_mode": args.task_mode,
        "controller_variant": args.controller_variant,
        "task_duration_s": args.task_duration_s,
        "repeats": args.repeats,
        "dry_run": args.dry_run,
        "plan": plan,
        "records": records,
    }
    with open(manifest, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    return manifest


def _postprocess(args, records):
    """实验完成后的统一后处理：指标重算、对比图生成和论文资产导出。"""
    run_dirs = _summary_dirs(records)
    if args.dry_run or args.skip_postprocess or not run_dirs:
        return {}

    # 先生成包含所有运行的总对比表和总对比图。
    comparison_root = Path(args.output_dir) / f"comparison_{args.suite_id}"
    all_dir = comparison_root / "all"
    compare_main([
        "--runs",
        *run_dirs,
        "--recompute-window-metrics",
        "--output-dir",
        str(all_dir),
    ])

    # 再按场景拆分，生成每个实验场景自己的对比图表。
    by_scenario = {}
    for run_dir in run_dirs:
        summary_path = Path(run_dir) / "summary.json"
        with open(summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
        by_scenario.setdefault(summary.get("scenario", "unknown"), []).append(run_dir)

    scenario_outputs = {}
    for scenario, scenario_run_dirs in by_scenario.items():
        out_dir = comparison_root / scenario
        compare_main([
            "--runs",
            *scenario_run_dirs,
            "--recompute-window-metrics",
            "--output-dir",
            str(out_dir),
        ])
        scenario_outputs[scenario] = str(out_dir)

    # 最后把论文可引用的图、表和 NPZ 数据复制到固定资产目录。
    export_main([
        "--runs",
        *run_dirs,
        "--comparison-dir",
        str(all_dir),
        "--paper-dir",
        args.paper_dir,
    ])

    return {
        "comparison_all": str(all_dir),
        "comparison_by_scenario": scenario_outputs,
        "paper_dir": args.paper_dir,
    }


def main(argv=None):
    """命令行入口：展开实验矩阵、运行 Gazebo、执行后处理。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=["core", "paper", "all"], default="paper")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--task-mode", default="no_rcm", choices=["no_rcm", "with_rcm"])
    ap.add_argument("--controller-variant", default="gt_kf")
    ap.add_argument("--task-duration-s", type=float, default=24.0)
    ap.add_argument("--fixed-alpha-hr", type=float, default=0.5)
    ap.add_argument("--output-dir", default="/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo")
    ap.add_argument("--paper-dir", default="/home/liu/franka_ros2_ws/panda_robot_gt_controller_dev/tests/0630thesis/generated/ch4_assets")
    ap.add_argument("--timeout-s", type=float, default=90.0)
    ap.add_argument("--settle-after-save-s", type=float, default=2.0)
    ap.add_argument("--suite-id", default=None)
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--enable-control", default=None, type=lambda s: s.lower() in ("1", "true", "yes", "on"))
    ap.add_argument("--gz-args", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--skip-postprocess", action="store_true")
    args = ap.parse_args(argv)

    args.suite_id = args.suite_id or _stamp()
    plan = _expand_plan(args.profile)
    records = _run_plan(args, plan)
    manifest = _write_manifest(args, plan, records)
    post = _postprocess(args, records)

    ok_count = sum(1 for r in records if r["status"] in ("ok", "dry_run"))
    print(json.dumps({
        "suite_id": args.suite_id,
        "profile": args.profile,
        "runs": len(records),
        "ok": ok_count,
        "manifest": str(manifest),
        **post,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
