#!/usr/bin/env python3
"""第4章 Gazebo 实验命令生成脚本。

该脚本不启动 Gazebo，只打印每个 ``scenario`` 和 ``arbitration_strategy`` 对应的
``ros2 launch`` 命令。用途：

1. 快速检查实验矩阵是否覆盖完整。
2. 方便手工复制单条命令调试。
3. 可把输出附在实验复现说明中。
"""

import argparse


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


def main(argv=None):
    """命令行入口：打印 source 命令和所有 launch 命令。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--task-mode", default="no_rcm", choices=["no_rcm", "with_rcm"])
    ap.add_argument("--controller-variant", default="gt_kf")
    ap.add_argument("--scenarios", nargs="*", default=SCENARIOS)
    ap.add_argument("--strategies", nargs="*", default=STRATEGIES)
    args = ap.parse_args(argv)

    # 先打印环境 source 命令，确保复制整段输出后可以直接执行。
    print("source /opt/ros/humble/setup.bash")
    print("source /home/liu/franka_ros2_ws/install/setup.bash")
    for scenario in args.scenarios:
        for strategy in args.strategies:
            print(
                "ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py "
                f"task_mode:={args.task_mode} controller_variant:={args.controller_variant} "
                f"arbitration_strategy:={strategy} scenario:={scenario}"
            )


if __name__ == "__main__":
    main()
