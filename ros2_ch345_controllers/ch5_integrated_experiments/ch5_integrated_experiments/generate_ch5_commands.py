#!/usr/bin/env python3
"""输出第5章常用命令清单。

该脚本不运行实验，只把常用命令打印到终端，避免手工记忆长参数。
适合在开始第5章实验前执行：

`ros2 run ch5_integrated_experiments generate_ch5_commands`
"""

from .method_switches import DEFAULT_GAZEBO_METHODS


def main():
    """打印 Gazebo、聚合统计、Falcon 实物接口和硬件预检命令。"""

    methods = ",".join(DEFAULT_GAZEBO_METHODS)
    print("# Chapter 5 integrated Gazebo suite")
    print("source /opt/ros/humble/setup.bash")
    print("source /home/liu/franka_ros2_ws/install/setup.bash")
    print(
        "ros2 run ch5_integrated_experiments run_ch5_gazebo_suite "
        f"--task-mode no_rcm --scenario mixed_sequence --methods {methods} --trials 1"
    )
    print()
    print("# Aggregate completed suites into paper tables")
    print(
        "ros2 run ch5_integrated_experiments ch5_aggregate_results "
        "--inputs /home/liu/franka_ros2_ws/results/ch5_integrated "
        "--output-dir /home/liu/franka_ros2_ws/results/ch5_integrated/aggregate"
    )
    print()
    print("# Falcon hardware bridge for live human input")
    print("ros2 launch ros2_falcon falcon.launch.py")
    print("ros2 launch ch5_integrated_experiments ch5_falcon_bridge.launch.py")
    print(
        "ros2 launch ch4_shared_gt_controller ch4_shared_gt_gazebo.launch.py "
        "scenario:=external arbitration_strategy:=full_method execution_strategy:=continuous_force_margin "
        "external_human_topic:=/ch5/human_delta"
    )
    print()
    print("# Hardware topic preflight")
    print(
        "ros2 run ch5_integrated_experiments ch5_hardware_preflight "
        "--publisher-topics /joint_states,/falcon/ee_pose,/falcon/velocity,/falcon/joystick,/ch5/human_delta "
        "--subscriber-topics /no_rcm_effort_controller/commands,/falcon/force_cmd"
    )


if __name__ == "__main__":
    main()
