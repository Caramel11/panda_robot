#!/usr/bin/env python3
"""第3/4/5章对比实验与消融实验统一接口。

本脚本不重新实现控制律，而是把已有 ROS2/Gazebo runner 组织成论文实验矩阵：

* 对比实验：本文完整方法与常用基线控制器/仲裁方法比较；
* 消融实验：本文完整方法与去掉关键模块后的变体比较。

默认行为是生成实验计划和可复制命令，不启动 Gazebo。加入 ``--execute`` 后，
脚本会按计划顺序调用各章节已有 runner。这样既能用于论文实验排程，也能用于
短快可行性筛查。
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Sequence


WORKSPACE = Path("/home/liu/franka_ros2_ws")
DEFAULT_OUTPUT_ROOT = WORKSPACE / "results" / "ch345_comparison_ablation_matrix"

ALL_LETTERS = "ustc_smooth_u,ustc_smooth_s,ustc_smooth_t,ustc_smooth_c"
QUICK_LETTER = "ustc_smooth_s"


@dataclass(frozen=True)
class ExperimentCase:
    """单个实验计划条目。

    Attributes:
        case_id: 稳定唯一标识，支持 ``--case-id`` 精确选择。
        chapter: 章节编号，取 ``3``、``4``、``5``。
        experiment_type: ``comparison`` 或 ``ablation``。
        task_scope: ``no_rcm`` 或 ``with_rcm``。
        title: 中文实验名称。
        purpose: 该实验在论文中回答的问题。
        methods: 逗号分隔的方法/策略名，直接传给对应 runner。
        runner: ``ros2 run`` 后的包名和可执行名。
        command: 完整可复制命令，不含环境 source。
    """

    case_id: str
    chapter: str
    experiment_type: str
    task_scope: str
    title: str
    purpose: str
    methods: str
    runner: str
    command: str
    output_dir: str
    expected_observation: str


def _csv(items: Iterable[str]) -> str:
    return ",".join(str(x).strip() for x in items if str(x).strip())


def _split_csv(raw: str) -> List[str]:
    return [x.strip() for x in str(raw).split(",") if x.strip()]


def _shell_join(parts: Sequence[object]) -> str:
    return " ".join(shlex.quote(str(p)) for p in parts if str(p) != "")


def _base_flags(args, output_dir: Path, task_mode: str, scenario_arg: str = "--scenarios") -> List[str]:
    """生成各章节 smooth-letter runner 共有的命令行参数。"""

    duration = args.task_duration_s
    if duration is None:
        duration = 18.0 if args.profile == "quick" else (22.0 if task_mode == "with_rcm" else 26.0)
    scale = args.trajectory_scale
    if scale is None:
        scale = 0.20 if task_mode == "with_rcm" else (0.35 if args.profile == "quick" else 0.45)

    flags = [
        "--run-gazebo",
        scenario_arg,
        args.scenarios,
        "--output-dir",
        output_dir,
        "--task-duration-s",
        f"{duration:.6g}",
        "--trajectory-scale",
        f"{scale:.6g}",
        "--timeout-s",
        f"{args.timeout_s:.6g}",
    ]
    if args.realistic_env:
        flags.extend([
            "--realistic-env",
            "--realistic-profile",
            args.realistic_profile,
            "--realistic-seed",
            str(args.realistic_seed),
            "--realistic-tau-noise-gain",
            f"{args.realistic_tau_noise_gain:.6g}",
        ])
    if args.human_input_profile:
        flags.extend(["--human-input-profile", args.human_input_profile])
    if args.rviz:
        flags.append("--rviz")
    if args.gz_args:
        flags.extend(["--gz-args", args.gz_args])
    return [str(x) for x in flags]


def _case(
    *,
    case_id: str,
    chapter: str,
    experiment_type: str,
    task_scope: str,
    title: str,
    purpose: str,
    methods: str,
    runner_pkg: str,
    runner_exec: str,
    output_dir: Path,
    flags: Sequence[str],
    method_flag: str,
    repeat_flag: str,
    repeats: int,
    expected_observation: str,
    extra: Sequence[str] = (),
) -> ExperimentCase:
    cmd = [
        "ros2",
        "run",
        runner_pkg,
        runner_exec,
        *flags,
        method_flag,
        methods,
        repeat_flag,
        str(repeats),
        *extra,
    ]
    return ExperimentCase(
        case_id=case_id,
        chapter=chapter,
        experiment_type=experiment_type,
        task_scope=task_scope,
        title=title,
        purpose=purpose,
        methods=methods,
        runner=f"{runner_pkg} {runner_exec}",
        command=_shell_join(cmd),
        output_dir=str(output_dir),
        expected_observation=expected_observation,
    )


def _ch3_flags(args, output_dir: Path, alpha_profile: str | None = None) -> List[str]:
    flags = _base_flags(args, output_dir, "no_rcm")
    flags.extend([
        "--task-mode",
        "no_rcm",
        "--execution-alpha-profile",
        alpha_profile or args.execution_alpha_profile,
    ])
    return flags


def _ch4_flags(args, output_dir: Path) -> List[str]:
    flags = _base_flags(args, output_dir, "no_rcm")
    flags.extend(["--task-mode", "no_rcm"])
    return flags


def _ch5_flags(args, output_dir: Path, task_mode: str, tolerance: float) -> List[str]:
    flags = _base_flags(args, output_dir, task_mode)
    flags.extend([
        "--task-mode",
        task_mode,
        "--target-tolerance-m",
        f"{tolerance:.6g}",
        "--execution-alpha-profile",
        args.execution_alpha_profile,
    ])
    return flags


def build_cases(args) -> List[ExperimentCase]:
    """根据命令行参数构造完整实验矩阵。"""

    stamp = args.suite_id or time.strftime("%Y%m%d_%H%M%S")
    repeats = args.trials if args.trials is not None else (1 if args.profile == "quick" else 3)
    out_root = Path(args.output_root) / f"{args.profile}_{stamp}"

    cases: List[ExperimentCase] = []

    # 第3章：只验证执行层力位协调，不引入参考层人机仲裁。
    ch3_comparison_dir = out_root / "ch3" / "comparison"
    cases.append(_case(
        case_id="ch3_comparison_no_rcm",
        chapter="3",
        experiment_type="comparison",
        task_scope="no_rcm",
        title="第三章执行层控制器对比实验",
        purpose="比较标准阻抗、传统混合力位、标准 MPC、固定仲裁与本文动态力位协调方法。",
        methods="standard_impedance,traditional_hybrid,standard_mpc,balanced_fixed,continuous_force_margin",
        runner_pkg="ch3_experiments",
        runner_exec="run_ch3_smooth_letter_comparison",
        output_dir=ch3_comparison_dir,
        flags=_ch3_flags(args, ch3_comparison_dir),
        method_flag="--methods",
        repeat_flag="--repeats",
        repeats=repeats,
        expected_observation="本文方法应在力越界时间、接触力峰值和综合评分上优于固定仲裁及常用基线，同时保持可接受的位置误差。",
    ))
    ch3_fixed_alpha_dir = out_root / "ch3" / "ablation_fixed_alpha"
    cases.append(_case(
        case_id="ch3_ablation_fixed_alpha",
        chapter="3",
        experiment_type="ablation",
        task_scope="no_rcm",
        title="第三章执行层固定 alpha 消融实验",
        purpose="验证动态 alpha_FP 相比固定强位置、固定仲裁和固定强力的必要性。",
        methods="strong_position,balanced_fixed,strong_force,continuous_force_margin",
        runner_pkg="ch3_experiments",
        runner_exec="run_ch3_smooth_letter_comparison",
        output_dir=ch3_fixed_alpha_dir,
        flags=_ch3_flags(args, ch3_fixed_alpha_dir),
        method_flag="--methods",
        repeat_flag="--repeats",
        repeats=repeats,
        expected_observation="固定强位置会增大力风险，固定强力会牺牲轨迹一致性，本文方法应体现更稳定的折中。",
    ))
    for profile in ("stable", "stiffness_sensitive", "sensitive", "alpha_stiffness"):
        alpha_dir = out_root / "ch3" / f"ablation_alpha_{profile}"
        cases.append(_case(
            case_id=f"ch3_ablation_alpha_profile_{profile}",
            chapter="3",
            experiment_type="ablation",
            task_scope="no_rcm",
            title=f"第三章 alpha 调节律参数组消融：{profile}",
            purpose="比较不同 continuous_force_margin 参数组对刚度切换敏感性、力越界和位置误差的影响。",
            methods="continuous_force_margin",
            runner_pkg="ch3_experiments",
            runner_exec="run_ch3_smooth_letter_comparison",
            output_dir=alpha_dir,
            flags=_ch3_flags(args, alpha_dir, alpha_profile=profile),
            method_flag="--methods",
            repeat_flag="--repeats",
            repeats=repeats,
            expected_observation="刚度敏感参数组应使 alpha_FP 对 K_env/K_hat 响应更明显，但需检查是否引入过强振荡。",
        ))

    # 第4章：只验证参考层人机轨迹仲裁，执行层保持固定。
    ch4_comparison_dir = out_root / "ch4" / "comparison"
    cases.append(_case(
        case_id="ch4_comparison_reference",
        chapter="4",
        experiment_type="comparison",
        task_scope="no_rcm",
        title="第四章参考层仲裁方法对比实验",
        purpose="比较直接接受、固定融合、单通道 Sigmoid 仲裁与本文安全动态仲裁。",
        methods="direct_accept,fixed_blend,single_sigmoid,full_method",
        runner_pkg="ch4_shared_gt_controller",
        runner_exec="run_ch4_smooth_letter_comparison",
        output_dir=ch4_comparison_dir,
        flags=_ch4_flags(args, ch4_comparison_dir),
        method_flag="--strategies",
        repeat_flag="--repeats",
        repeats=repeats,
        expected_observation="本文方法应保留安全切向修正，同时降低危险法向输入造成的力峰值和越界时间。",
    ))
    ch4_ablation_dir = out_root / "ch4" / "ablation"
    cases.append(_case(
        case_id="ch4_ablation_reference",
        chapter="4",
        experiment_type="ablation",
        task_scope="no_rcm",
        title="第四章参考层模块消融实验",
        purpose="验证安全投影、方向分解和动态人类权重对参考层安全性的贡献。",
        methods="direct_accept,fixed_blend,single_sigmoid,dynamic_no_projection,full_method",
        runner_pkg="ch4_shared_gt_controller",
        runner_exec="run_ch4_smooth_letter_comparison",
        output_dir=ch4_ablation_dir,
        flags=_ch4_flags(args, ch4_ablation_dir),
        method_flag="--strategies",
        repeat_flag="--repeats",
        repeats=repeats,
        expected_observation="无安全投影会放大危险法向输入，固定融合无法区分输入类型，完整方法应在 R_acc 与 R_sup 间取得更好折中。",
    ))

    # 第5章：综合系统。no-RCM 用于工业柔性表面，with-RCM 用于长工具固定孔场景。
    for task_mode, prefix, tol in (
        ("no_rcm", "", args.target_tolerance_m),
        ("with_rcm", "rcm_", max(args.target_tolerance_m, 0.020)),
    ):
        ch5_comparison_dir = out_root / "ch5" / task_mode / "comparison"
        cases.append(_case(
            case_id=f"ch5_comparison_{task_mode}",
            chapter="5",
            experiment_type="comparison",
            task_scope=task_mode,
            title=f"第五章综合系统对比实验（{task_mode}）",
            purpose="比较常用执行层控制器、固定仲裁与本文完整双层方法。",
            methods=(
                f"{prefix}full_method,{prefix}standard_impedance,"
                f"{prefix}traditional_hybrid,{prefix}standard_mpc,{prefix}balanced_fixed"
            ),
            runner_pkg="ch5_integrated_experiments",
            runner_exec="run_ch5_smooth_letter_comparison",
            output_dir=ch5_comparison_dir,
            flags=_ch5_flags(args, ch5_comparison_dir, task_mode, tol),
            method_flag="--methods",
            repeat_flag="--trials",
            repeats=repeats,
            expected_observation="本文完整方法应在轨迹误差、力安全、参考接受/抑制和 RCM 误差之间取得最低综合代价。",
        ))
        ch5_ablation_dir = out_root / "ch5" / task_mode / "ablation"
        cases.append(_case(
            case_id=f"ch5_ablation_{task_mode}",
            chapter="5",
            experiment_type="ablation",
            task_scope=task_mode,
            title=f"第五章双层控制模块消融实验（{task_mode}）",
            purpose="验证参考层、执行层、安全投影和 RCM 约束处理在完整系统中的必要性。",
            methods=(
                f"{prefix}full_method,{prefix}balanced_fixed,{prefix}reference_only,"
                f"{prefix}execution_only,{prefix}no_projection"
            ),
            runner_pkg="ch5_integrated_experiments",
            runner_exec="run_ch5_smooth_letter_comparison",
            output_dir=ch5_ablation_dir,
            flags=_ch5_flags(args, ch5_ablation_dir, task_mode, tol),
            method_flag="--methods",
            repeat_flag="--trials",
            repeats=repeats,
            expected_observation="仅参考层无法处理执行层刚度切换，仅执行层无法体现人类输入安全，去掉安全投影会增大危险输入风险。",
        ))

    return cases


def _filter_cases(cases: Sequence[ExperimentCase], args) -> List[ExperimentCase]:
    chapters = set(_split_csv(args.chapters))
    types = set(_split_csv(args.experiment_types))
    scopes = set(_split_csv(args.task_scopes))
    ids = set(_split_csv(args.case_ids))
    out = []
    for case in cases:
        if chapters and case.chapter not in chapters:
            continue
        if types and case.experiment_type not in types:
            continue
        if scopes and case.task_scope not in scopes:
            continue
        if ids and case.case_id not in ids:
            continue
        out.append(case)
    return out


def _write_plan(cases: Sequence[ExperimentCase], out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "ch345_experiment_matrix_plan.json"
    md_path = out_dir / "CH345_COMPARISON_ABLATION_EXPERIMENT_PLAN.md"
    json_path.write_text(
        json.dumps([asdict(c) for c in cases], indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# 第3/4/5章对比实验与消融实验计划",
        "",
        "## 实验分类原则",
        "",
        "- 对比实验：本文方法与标准阻抗、传统混合力位、标准 MPC、固定融合等常用基线比较，用于证明方法优越性。",
        "- 消融实验：在本文方法框架内去掉动态权重、安全投影、参考层、执行层或 RCM 处理等模块，用于证明设计必要性。",
        "",
        "## 实验矩阵",
        "",
        "| case_id | 章节 | 类型 | 范围 | 方法/策略 | 目的 |",
        "|---|---:|---|---|---|---|",
    ]
    for c in cases:
        lines.append(
            f"| `{c.case_id}` | {c.chapter} | {c.experiment_type} | {c.task_scope} | "
            f"`{c.methods}` | {c.purpose} |"
        )
    lines.extend(["", "## 可复制运行命令", ""])
    for c in cases:
        lines.extend([
            f"### {c.case_id}",
            "",
            f"**实验名称：** {c.title}",
            "",
            f"**预期观察：** {c.expected_observation}",
            "",
            "```bash",
            "source /opt/ros/humble/setup.bash",
            "source /home/liu/franka_ros2_ws/install/setup.bash",
            c.command,
            "```",
            "",
        ])
    md_path.write_text("\n".join(lines), encoding="utf-8")
    return {"plan_json": str(json_path), "plan_md": str(md_path)}


def _execute_case(case: ExperimentCase) -> int:
    script = (
        "set -e\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /home/liu/franka_ros2_ws/install/setup.bash\n"
        f"{case.command}\n"
    )
    print(f"[matrix] executing {case.case_id}", flush=True)
    return subprocess.run(["bash", "-lc", script], cwd=str(WORKSPACE)).returncode


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["quick", "paper"], default="quick")
    parser.add_argument("--suite-id", default="")
    parser.add_argument("--chapters", default="3,4,5", help="逗号分隔，如 3,5")
    parser.add_argument("--experiment-types", default="comparison,ablation", help="comparison,ablation")
    parser.add_argument("--task-scopes", default="no_rcm,with_rcm", help="no_rcm,with_rcm")
    parser.add_argument("--case-ids", default="", help="只运行/生成指定 case_id，逗号分隔")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--scenarios", default=QUICK_LETTER, help=f"默认短快 S 曲线；完整实验用 {ALL_LETTERS}")
    parser.add_argument("--trials", type=int, default=None)
    parser.add_argument("--task-duration-s", type=float, default=None)
    parser.add_argument("--trajectory-scale", type=float, default=None)
    parser.add_argument("--target-tolerance-m", type=float, default=0.010)
    parser.add_argument("--timeout-s", type=float, default=140.0)
    parser.add_argument("--execution-alpha-profile", default="stable", choices=["stable", "stiffness_sensitive", "sensitive", "alpha_stiffness"])
    parser.add_argument("--realistic-env", action="store_true")
    parser.add_argument("--realistic-profile", default="sponge_300_600_two_block")
    parser.add_argument("--realistic-seed", type=int, default=20260705)
    parser.add_argument("--realistic-tau-noise-gain", type=float, default=0.08)
    parser.add_argument("--human-input-profile", default="realistic_events")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--gz-args", default="")
    parser.add_argument("--execute", action="store_true", help="实际启动 Gazebo；默认只生成计划")
    args = parser.parse_args(argv)

    if args.profile == "paper" and args.scenarios == QUICK_LETTER:
        args.scenarios = ALL_LETTERS
    cases = _filter_cases(build_cases(args), args)
    if not cases:
        raise SystemExit("没有匹配的实验 case，请检查 --chapters/--experiment-types/--task-scopes/--case-ids")

    plan_dir = Path(args.output_root) / f"{args.profile}_{args.suite_id or time.strftime('%Y%m%d_%H%M%S')}" / "plan"
    plan = _write_plan(cases, plan_dir)
    print(json.dumps({"cases": len(cases), **plan}, indent=2, ensure_ascii=False))

    if not args.execute:
        print("[matrix] plan-only mode; add --execute to run Gazebo experiments.", flush=True)
        return 0

    failures = []
    for case in cases:
        code = _execute_case(case)
        if code != 0:
            failures.append({"case_id": case.case_id, "returncode": code})
            print(f"[matrix] failed {case.case_id}: {code}", flush=True)
            break
    if failures:
        print(json.dumps({"failures": failures}, indent=2, ensure_ascii=False))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
