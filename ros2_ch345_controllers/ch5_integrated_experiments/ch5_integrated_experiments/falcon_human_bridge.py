#!/usr/bin/env python3
"""Falcon 主端到第5章人类参考输入的桥接节点。

第5章实物实验需要把 Novint Falcon 力反馈器的主端位移接入第4章参考层。
`ros2_falcon` 已经发布 `/falcon/ee_pose`、`/falcon/velocity` 和
`/falcon/joystick`，而第4章综合控制器在 `scenario:=external` 时订阅
`/ch5/human_delta`。本节点负责二者之间的转换：

`/falcon/ee_pose` -> 缩放/零偏/死区/限幅/低通 -> `/ch5/human_delta`

同时，本节点可以发布一个简单的虚拟弹簧阻尼力到 `/falcon/force_cmd`，
用于给操作者提供主端回中感。该力反馈只作用在 Falcon 上，不直接驱动机器人。
"""
import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy


def _vec_from_point(msg):
    """把 `geometry_msgs/Point` 转换为 numpy 三维向量。"""

    return np.array([msg.x, msg.y, msg.z], dtype=float)


def _clip_norm(vec, limit):
    """按向量范数限幅，保持方向不变。

    Falcon 手柄位移或力反馈都必须限幅，避免操作者瞬间输入过大参考增量，
    或给 Falcon 写入超过舒适范围的反馈力。
    """

    vec = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(vec))
    if limit <= 0.0 or norm <= limit or norm < 1e-12:
        return vec
    return vec * (limit / norm)


class FalconHumanBridge(Node):
    """把 Falcon 主端运动转换为第5章候选人类参考增量。

    主要安全策略：
    - `require_clutch`：要求按钮按下才输出人类输入；
    - `deadband_m`：小幅抖动归零；
    - `max_delta_m`：最大任务空间参考增量限幅；
    - `lowpass_tau_s`：一阶低通，减少手部抖动；
    - `max_feedback_force_n`：主端反馈力限幅。
    """

    def __init__(self):
        """初始化对象参数和运行状态。"""
        super().__init__("ch5_falcon_human_bridge")
        # 输入话题来自 ros2_falcon 驱动。ee_pose 是经过 clutch 逻辑积分后的
        # 主端位移，比 raw_position 更适合作为机器人参考增量。
        self.declare_parameter("input_pose_topic", "/falcon/ee_pose")
        self.declare_parameter("input_velocity_topic", "/falcon/velocity")
        self.declare_parameter("input_joy_topic", "/falcon/joystick")
        self.declare_parameter("human_delta_topic", "/ch5/human_delta")
        self.declare_parameter("force_cmd_topic", "/falcon/force_cmd")
        self.declare_parameter("scale_xyz", [1.0, 1.0, 1.0])
        self.declare_parameter("offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("max_delta_m", 0.025)
        self.declare_parameter("deadband_m", 0.0005)
        self.declare_parameter("lowpass_tau_s", 0.08)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("require_clutch", True)
        self.declare_parameter("clutch_button_index", 0)
        self.declare_parameter("clutch_button_mask", 1)
        self.declare_parameter("zero_when_released", True)
        self.declare_parameter("enable_force_feedback", True)
        self.declare_parameter("force_stiffness", 12.0)
        self.declare_parameter("force_damping", 0.6)
        self.declare_parameter("max_feedback_force_n", 4.0)

        # scale/offset 用于实物标定：如果 Falcon 坐标和机器人任务空间方向
        # 不完全一致，可通过参数调整比例和零点，不必改代码。
        self.scale = np.asarray(self.get_parameter("scale_xyz").value, dtype=float)
        self.offset = np.asarray(self.get_parameter("offset_xyz").value, dtype=float)
        self.max_delta_m = float(self.get_parameter("max_delta_m").value)
        self.deadband_m = float(self.get_parameter("deadband_m").value)
        self.lowpass_tau_s = float(self.get_parameter("lowpass_tau_s").value)
        self.require_clutch = bool(self.get_parameter("require_clutch").value)
        self.clutch_button_index = int(self.get_parameter("clutch_button_index").value)
        self.clutch_button_mask = int(self.get_parameter("clutch_button_mask").value)
        self.zero_when_released = bool(self.get_parameter("zero_when_released").value)
        self.enable_force_feedback = bool(self.get_parameter("enable_force_feedback").value)
        self.force_stiffness = float(self.get_parameter("force_stiffness").value)
        self.force_damping = float(self.get_parameter("force_damping").value)
        self.max_feedback_force_n = float(self.get_parameter("max_feedback_force_n").value)

        self.raw_pose = np.zeros(3)
        self.velocity = np.zeros(3)
        self.filtered_delta = np.zeros(3)
        self.clutch_active = not self.require_clutch
        self.last_time = self.get_clock().now()

        # 输出给第4章控制器的人类候选参考增量。
        self.human_pub = self.create_publisher(
            Point,
            str(self.get_parameter("human_delta_topic").value),
            10,
        )
        # 输出给 Falcon 驱动的力反馈命令。若不需要力反馈，可关闭
        # `enable_force_feedback`，此 publisher 保留但不会发送非零力。
        self.force_pub = self.create_publisher(
            Vector3,
            str(self.get_parameter("force_cmd_topic").value),
            10,
        )
        self.pose_sub = self.create_subscription(
            Point,
            str(self.get_parameter("input_pose_topic").value),
            self._on_pose,
            20,
        )
        self.velocity_sub = self.create_subscription(
            Vector3,
            str(self.get_parameter("input_velocity_topic").value),
            self._on_velocity,
            20,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter("input_joy_topic").value),
            self._on_joy,
            20,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.get_logger().info(
            "Falcon bridge ready: "
            f"{self.get_parameter('input_pose_topic').value} -> {self.get_parameter('human_delta_topic').value}"
        )

    def _on_pose(self, msg):
        """缓存 Falcon 当前主端位移。"""

        self.raw_pose = _vec_from_point(msg)

    def _on_velocity(self, msg):
        """缓存 Falcon 主端速度，用于阻尼反馈。"""

        self.velocity = np.array([msg.x, msg.y, msg.z], dtype=float)

    def _on_joy(self, msg):
        """读取 clutch 按钮状态。

        `ros2_falcon` 当前把按钮 bitmask 放在 `buttons[0]` 中，因此默认
        `clutch_button_index=0`、`clutch_button_mask=1`。
        """

        if not self.require_clutch:
            self.clutch_active = True
            return
        if self.clutch_button_index >= len(msg.buttons):
            self.clutch_active = False
            return
        button_word = int(msg.buttons[self.clutch_button_index])
        self.clutch_active = bool(button_word & self.clutch_button_mask)

    def _target_delta(self):
        """计算未经低通滤波的目标人类参考增量。"""

        if self.require_clutch and not self.clutch_active and self.zero_when_released:
            return np.zeros(3)
        delta = (self.raw_pose - self.offset) * self.scale
        if np.linalg.norm(delta) < self.deadband_m:
            delta = np.zeros(3)
        return _clip_norm(delta, self.max_delta_m)

    def _on_timer(self):
        """周期性发布人类输入，并可选发布 Falcon 力反馈。"""

        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 1e-4)
        self.last_time = now
        target = self._target_delta()
        # 一阶低通离散形式：x[k] = x[k-1] + alpha * (target - x[k-1])。
        alpha = dt / max(self.lowpass_tau_s + dt, dt)
        self.filtered_delta += alpha * (target - self.filtered_delta)

        msg = Point()
        msg.x, msg.y, msg.z = self.filtered_delta.tolist()
        self.human_pub.publish(msg)

        if self.enable_force_feedback:
            # 简单虚拟弹簧阻尼：F = -Kx - Bv。该反馈只帮助操作者感知
            # 手柄偏离程度，不参与 Panda 控制器的力矩计算。
            force = -self.force_stiffness * self.filtered_delta - self.force_damping * self.velocity
            if self.require_clutch and not self.clutch_active:
                force = np.zeros(3)
            force = _clip_norm(force, self.max_feedback_force_n)
            fmsg = Vector3()
            fmsg.x, fmsg.y, fmsg.z = force.tolist()
            self.force_pub.publish(fmsg)


def main(argv=None):
    """ROS2 节点入口。"""

    parser = argparse.ArgumentParser()
    parser.parse_known_args(argv)
    rclpy.init()
    node = FalconHumanBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
