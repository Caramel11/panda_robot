"""启动第5章 Falcon 人类输入桥接节点。

该 launch 只启动 `ch5_falcon_human_bridge`。实际使用时通常需要先启动
`ros2_falcon` 驱动，再启动本 launch，最后把第4章控制器设置为
`scenario:=external`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """声明 Falcon 桥接节点的常用参数并创建 Node。"""

    return LaunchDescription([
        # 输入话题来自 ros2_falcon。
        DeclareLaunchArgument("input_pose_topic", default_value="/falcon/ee_pose"),
        DeclareLaunchArgument("input_velocity_topic", default_value="/falcon/velocity"),
        DeclareLaunchArgument("input_joy_topic", default_value="/falcon/joystick"),
        # 输出给第4章控制器和 Falcon 力反馈驱动。
        DeclareLaunchArgument("human_delta_topic", default_value="/ch5/human_delta"),
        DeclareLaunchArgument("force_cmd_topic", default_value="/falcon/force_cmd"),
        # 安全相关参数：最大参考增量、发布频率、是否要求 clutch、是否给 Falcon 写力反馈。
        DeclareLaunchArgument("max_delta_m", default_value="0.025"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("require_clutch", default_value="true"),
        DeclareLaunchArgument("enable_force_feedback", default_value="true"),
        Node(
            package="ch5_integrated_experiments",
            executable="ch5_falcon_human_bridge",
            name="ch5_falcon_human_bridge",
            output="screen",
            parameters=[{
                "input_pose_topic": LaunchConfiguration("input_pose_topic"),
                "input_velocity_topic": LaunchConfiguration("input_velocity_topic"),
                "input_joy_topic": LaunchConfiguration("input_joy_topic"),
                "human_delta_topic": LaunchConfiguration("human_delta_topic"),
                "force_cmd_topic": LaunchConfiguration("force_cmd_topic"),
                "max_delta_m": LaunchConfiguration("max_delta_m"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "require_clutch": LaunchConfiguration("require_clutch"),
                "enable_force_feedback": LaunchConfiguration("enable_force_feedback"),
            }],
        ),
    ])
