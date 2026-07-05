"""第三章指标兼容层。

历史 no_rcm 包中已经实现了较完整的力/位误差分析流程。本模块通过复用
`no_rcm.analyze_ch3_result` 的命令行入口，使第三章新实验包与历史 no_rcm
控制器保持同一套指标口径。
"""

from pathlib import Path


def analyze_with_no_rcm(input_path, output_dir):
    """调用 no_rcm 分析脚本，并返回生成的关键文件路径。"""

    from no_rcm.analyze_ch3_result import main as no_rcm_analyze_main

    # Prefer direct subprocess-free reuse by temporarily patching argv.
    import sys

    old_argv = sys.argv[:]
    try:
        sys.argv = [
            "analyze_ch3_result",
            "--input",
            str(input_path),
            "--output-dir",
            str(output_dir),
        ]
        no_rcm_analyze_main()
    finally:
        sys.argv = old_argv
    out = Path(output_dir) / "analysis"
    return {
        "analysis_dir": out,
        "trial_metrics": out / "ch3_trial_metrics.csv",
        "summary": out / "ch3_strategy_summary.csv",
        "report": out / "ch3_force_position_analysis_report.md",
    }
