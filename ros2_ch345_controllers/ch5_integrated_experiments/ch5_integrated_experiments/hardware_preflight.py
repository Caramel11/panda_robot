#!/usr/bin/env python3
"""第5章实物实验前 ROS2 话题预检工具。

第5章实物实验涉及 Panda/Gazebo 控制接口、Falcon 主端输入、力反馈输出等
多个 ROS2 话题。真正启动控制器前，必须确认：

1. 传感器/驱动话题有 publisher；
2. 这些话题不仅存在，而且在持续发布新消息；
3. effort controller 和 Falcon driver 的命令话题有 subscriber。

本工具以 JSON 形式输出检查结果，可作为实物实验记录的一部分保存。
它不会发布控制命令，因此适合在正式实验前反复运行。
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray


KNOWN_TYPES = {
    "/joint_states": JointState,
    "/falcon/ee_pose": Point,
    "/falcon/raw_position": Point,
    "/falcon/velocity": Vector3,
    "/falcon/joystick": Joy,
    "/falcon/applied_force": Vector3,
    "/force_sensor/wrench": WrenchStamped,
    "/no_rcm_effort_controller/commands": Float64MultiArray,
    "/falcon/force_cmd": Vector3,
}


def _split_csv(text):
    """把命令行逗号分隔的话题列表解析为字符串列表。"""

    return [x.strip() for x in str(text).split(",") if x.strip()]


class HardwarePreflight(Node):
    """检查第5章实物实验所需话题是否在线。

    `publisher_topics` 会被主动订阅，以判断消息是否“新鲜”；`subscriber_topics`
    只检查 subscriber 数量，因为这些通常是命令入口，例如
    `/no_rcm_effort_controller/commands` 和 `/falcon/force_cmd`。
    """

    def __init__(self, publisher_topics, subscriber_topics, freshness_s):
        """初始化对象参数和运行状态。"""
        super().__init__("ch5_hardware_preflight")
        self.publisher_topics = list(publisher_topics)
        self.subscriber_topics = list(subscriber_topics)
        self.freshness_s = float(freshness_s)
        self.last_seen = {}
        self.subscriptions_ = []
        for topic in self.publisher_topics:
            msg_type = KNOWN_TYPES.get(topic)
            if msg_type is None:
                # 对未知类型话题，只能检查 publisher 数量，无法订阅消息。
                continue
            self.subscriptions_.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda _msg, name=topic: self._mark_seen(name),
                    10,
                )
            )

    def _mark_seen(self, topic):
        """记录某个话题最近一次收到消息的墙钟时间。"""

        self.last_seen[topic] = time.time()

    def snapshot(self):
        """生成当前预检快照。

        返回值分为两类：
        - `publisher_topics`：publisher 数量、是否收到消息、消息年龄；
        - `subscriber_topics`：subscriber 数量。
        """

        now = time.time()
        publishers = {}
        subscribers = {}
        for topic in self.publisher_topics:
            age = None
            if topic in self.last_seen:
                age = now - self.last_seen[topic]
            publishers[topic] = {
                "publisher_count": self.count_publishers(topic),
                "message_seen": topic in self.last_seen,
                "message_age_s": age,
                "fresh": bool(age is not None and age <= self.freshness_s),
                "typed_subscription": topic in KNOWN_TYPES,
            }
        for topic in self.subscriber_topics:
            subscribers[topic] = {
                "subscriber_count": self.count_subscribers(topic),
            }
        return {
            "publisher_topics": publishers,
            "subscriber_topics": subscribers,
            "freshness_s": self.freshness_s,
        }

    def success(self, report):
        """根据预检报告判断是否可以进入下一步实物实验。

        对已知消息类型的话题，要求存在 publisher 且消息足够新；对未知类型
        话题，只检查 publisher 数量。命令话题要求至少有一个 subscriber。
        """

        pub_ok = all(
            item["publisher_count"] > 0 and (item["fresh"] or not item["typed_subscription"])
            for item in report["publisher_topics"].values()
        )
        sub_ok = all(item["subscriber_count"] > 0 for item in report["subscriber_topics"].values())
        return bool(pub_ok and sub_ok)


def main(argv=None):
    """命令行入口。

    返回码：
    - 0：所有检查通过；
    - 2：至少一个话题缺失、无新消息或无 subscriber。
    """

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--publisher-topics",
        default="/joint_states,/falcon/ee_pose,/falcon/velocity,/falcon/joystick",
        help="Comma-separated topics that must have publishers and fresh messages when their type is known.",
    )
    ap.add_argument(
        "--subscriber-topics",
        default="/no_rcm_effort_controller/commands,/falcon/force_cmd",
        help="Comma-separated command topics that must already have subscribers.",
    )
    ap.add_argument("--timeout-s", type=float, default=10.0)
    ap.add_argument("--freshness-s", type=float, default=1.0)
    ap.add_argument("--output-report", default="")
    args = ap.parse_args(argv)

    rclpy.init()
    node = HardwarePreflight(
        _split_csv(args.publisher_topics),
        _split_csv(args.subscriber_topics),
        args.freshness_s,
    )
    deadline = time.time() + args.timeout_s
    report = node.snapshot()
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            report = node.snapshot()
            if node.success(report):
                break
    finally:
        ok = node.success(report)
        report["success"] = ok
        text = json.dumps(report, ensure_ascii=False, indent=2)
        print(text)
        if args.output_report:
            path = Path(args.output_report)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
