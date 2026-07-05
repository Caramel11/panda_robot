"""第4章共享控制器 Gazebo 启动文件。

该 launch 文件完成三件事：

1. 启动 Franka/FR3 Gazebo 仿真和 ``no_rcm_effort_controller``。
2. 延时启动第4章 Python 控制器，等待机器人、控制器和 joint_states 准备完成。
3. 将实验场景、仲裁策略、输出目录等参数传给 ``shared_gt_gazebo_node``。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成 ROS2 launch 描述。"""
    # 第4章实验核心参数：任务模式、控制器版本、共享仲裁策略和人类输入场景。
    task_mode_arg = DeclareLaunchArgument("task_mode", default_value="no_rcm")
    controller_variant_arg = DeclareLaunchArgument("controller_variant", default_value="gt_kf")
    strategy_arg = DeclareLaunchArgument("arbitration_strategy", default_value="full_method")
    execution_strategy_arg = DeclareLaunchArgument("execution_strategy", default_value="fixed_05")
    execution_alpha_profile_arg = DeclareLaunchArgument("execution_alpha_profile", default_value="stable")
    scenario_arg = DeclareLaunchArgument("scenario", default_value="mixed_sequence")
    # 数据输出目录和单次实验时长。
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/home/liu/franka_ros2_ws/results/ch4_shared_gt_gazebo",
    )
    task_duration_arg = DeclareLaunchArgument("task_duration_s", default_value="24.0")
    target_tolerance_arg = DeclareLaunchArgument("target_tolerance_m", default_value="0.003")
    trajectory_scale_arg = DeclareLaunchArgument("trajectory_scale", default_value="0.0")
    # 固定权重基线使用的参数。
    fixed_alpha_arg = DeclareLaunchArgument("fixed_alpha_hr", default_value="0.5")
    fixed_alpha_fp_arg = DeclareLaunchArgument("fixed_alpha_fp", default_value="0.5")
    realistic_env_arg = DeclareLaunchArgument("realistic_env_enabled", default_value="false")
    realistic_profile_arg = DeclareLaunchArgument("realistic_profile", default_value="real_env_0607_0213")
    realistic_seed_arg = DeclareLaunchArgument("realistic_seed", default_value="20260703")
    realistic_tau_noise_gain_arg = DeclareLaunchArgument("realistic_tau_noise_gain", default_value="0.12")
    realistic_force_metrics_arg = DeclareLaunchArgument("realistic_use_measured_force_for_metrics", default_value="true")
    human_input_profile_arg = DeclareLaunchArgument("human_input_profile", default_value="smoothstep")
    force_desired_arg = DeclareLaunchArgument("force_desired", default_value="0.7")
    force_min_arg = DeclareLaunchArgument("force_min", default_value="0.35")
    force_max_arg = DeclareLaunchArgument("force_max", default_value="1.2")
    force_margin_arg = DeclareLaunchArgument("force_margin", default_value="0.08")
    contact_stiffness_hat_arg = DeclareLaunchArgument("contact_stiffness_hat", default_value="70.0")
    # 外部人类输入接口，用于 Falcon 或第5章集成实验；第4章默认使用脚本输入。
    external_human_topic_arg = DeclareLaunchArgument("external_human_topic", default_value="/ch5/human_delta")
    external_human_timeout_arg = DeclareLaunchArgument("external_human_timeout_s", default_value="0.25")
    external_human_scale_arg = DeclareLaunchArgument("external_human_scale", default_value="1.0")
    external_human_max_arg = DeclareLaunchArgument("external_human_max_delta_m", default_value="0.025")
    enable_arg = DeclareLaunchArgument("enable_control", default_value="true")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false")
    # 默认使用无重力空世界，便于验证纯控制器稳定性。
    gz_args_arg = DeclareLaunchArgument(
        "gz_args",
        default_value="-r /home/liu/franka_ros2_ws/install/franka_gazebo_bringup/share/franka_gazebo_bringup/worlds/empty_no_gravity.sdf",
    )

    # 先启动官方/本地 franka_gazebo_bringup 的 Gazebo + ros2_control。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_gazebo_bringup"),
                    "launch",
                    "gazebo_franka_arm_example_controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_type": "fr3",
            "load_gripper": "false",
            "rviz": LaunchConfiguration("rviz"),
            "gz_args": LaunchConfiguration("gz_args"),
            "controller": "no_rcm_effort_controller",
        }.items(),
    )

    # 延时启动第4章控制器，避免 joint_states 和 effort controller 尚未准备好。
    controller_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ch4_shared_gt_controller",
                executable="shared_gt_gazebo_node",
                name="shared_gt_gazebo_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ch4_shared_gt_controller"),
                            "config",
                            "shared_gt_gazebo.yaml",
                        ]
                    ),
                    {
                        # 第二个参数字典覆盖 YAML 默认值，用 launch 命令即可切换实验。
                        "task_mode": LaunchConfiguration("task_mode"),
                        "controller_variant": LaunchConfiguration("controller_variant"),
                        "arbitration_strategy": LaunchConfiguration("arbitration_strategy"),
                        "execution_strategy": LaunchConfiguration("execution_strategy"),
                        "execution_alpha_profile": LaunchConfiguration("execution_alpha_profile"),
                        "scenario": LaunchConfiguration("scenario"),
                        "output_dir": LaunchConfiguration("output_dir"),
                        "task_duration_s": LaunchConfiguration("task_duration_s"),
                        "target_tolerance_m": LaunchConfiguration("target_tolerance_m"),
                        "trajectory_scale": LaunchConfiguration("trajectory_scale"),
                        "fixed_alpha_hr": LaunchConfiguration("fixed_alpha_hr"),
                        "fixed_alpha_fp": LaunchConfiguration("fixed_alpha_fp"),
                        "realistic_env_enabled": LaunchConfiguration("realistic_env_enabled"),
                        "realistic_profile": LaunchConfiguration("realistic_profile"),
                        "realistic_seed": LaunchConfiguration("realistic_seed"),
                        "realistic_tau_noise_gain": LaunchConfiguration("realistic_tau_noise_gain"),
                        "realistic_use_measured_force_for_metrics": LaunchConfiguration("realistic_use_measured_force_for_metrics"),
                        "human_input_profile": LaunchConfiguration("human_input_profile"),
                        "force_desired": LaunchConfiguration("force_desired"),
                        "force_min": LaunchConfiguration("force_min"),
                        "force_max": LaunchConfiguration("force_max"),
                        "force_margin": LaunchConfiguration("force_margin"),
                        "contact_stiffness_hat": LaunchConfiguration("contact_stiffness_hat"),
                        "external_human_topic": LaunchConfiguration("external_human_topic"),
                        "external_human_timeout_s": LaunchConfiguration("external_human_timeout_s"),
                        "external_human_scale": LaunchConfiguration("external_human_scale"),
                        "external_human_max_delta_m": LaunchConfiguration("external_human_max_delta_m"),
                        "enable_control": LaunchConfiguration("enable_control"),
                    },
                ],
            )
        ],
    )

    return LaunchDescription([
        task_mode_arg,
        controller_variant_arg,
        strategy_arg,
        execution_strategy_arg,
        execution_alpha_profile_arg,
        scenario_arg,
        output_dir_arg,
        task_duration_arg,
        target_tolerance_arg,
        trajectory_scale_arg,
        fixed_alpha_arg,
        fixed_alpha_fp_arg,
        realistic_env_arg,
        realistic_profile_arg,
        realistic_seed_arg,
        realistic_tau_noise_gain_arg,
        realistic_force_metrics_arg,
        human_input_profile_arg,
        force_desired_arg,
        force_min_arg,
        force_max_arg,
        force_margin_arg,
        contact_stiffness_hat_arg,
        external_human_topic_arg,
        external_human_timeout_arg,
        external_human_scale_arg,
        external_human_max_arg,
        enable_arg,
        rviz_arg,
        gz_args_arg,
        gazebo,
        controller_node,
    ])
