"""第三章绘图入口占位模块。

当前第三章主要复用 `no_rcm.analyze_ch3_result` 和 smooth-letter 专用脚本生成
论文图表。本入口保留给后续统一 CLI 扩展，避免 setup.py 中的命令失效。
"""

import argparse

from .ch3_metrics import analyze_with_no_rcm


def main():
    """保留命令行入口。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or args.input
    analyze_with_no_rcm(args.input, output_dir)


if __name__ == "__main__":
    main()
