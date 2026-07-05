"""第4章控制器使用的 Pinocchio 机器人模型辅助模块。

ROS1 0213 代码中使用 ``PandaKinematics`` 读取 Panda/Franka 的末端和法兰运动学。
ROS2 版本直接使用 Pinocchio，从 Gazebo 使用的 xacro/URDF 构建模型，并提供：

1. 受控 7 个关节到完整 Pinocchio 状态向量的映射。
2. 工具端和法兰坐标系的位姿、速度与雅可比。
3. 可选重力补偿项。

主控制器不直接处理 URDF 细节，而是通过本 helper 获取统一运动学接口。
"""

import os
from pathlib import Path

import numpy as np
import pinocchio as pin
import xacro
from ament_index_python.packages import get_package_share_directory


class PinocchioModelHelper:
    """FR3 Gazebo 机械臂的 Pinocchio 辅助类。

    ROS1 0213 脚本使用 ``PandaKinematics`` 处理 ``panda_link10/11`` 和
    ``panda_link8``。ROS2 版本直接使用 Pinocchio，并同时暴露 no-RCM 和 with-RCM
    控制需要的工具端坐标系与法兰坐标系。
    """

    def __init__(
        self,
        node,
        controlled_joints,
        tool_frame="fr3_link11",
        flange_frame="fr3_link8",
        robot_type="fr3",
    ):
        """初始化对象参数和运行状态。"""
        self.node = node
        self.controlled_joints = list(controlled_joints)
        self.tool_frame = tool_frame
        self.flange_frame = flange_frame
        self.robot_type = robot_type
        self.model = None
        self.data = None
        self.tool_frame_id = None
        self.flange_frame_id = None
        self._q_index = []
        self._v_index = []

    def load(self):
        """加载 URDF，创建 Pinocchio model/data，并缓存关节和坐标系索引。"""
        urdf = self._load_robot_description()
        self.model = pin.buildModelFromXML(urdf)
        self.data = self.model.createData()

        for frame in (self.tool_frame, self.flange_frame):
            if not self.model.existFrame(frame):
                frames = [f.name for f in self.model.frames]
                self.node.get_logger().error(
                    f"Pinocchio frame '{frame}' not found. Available tail: {frames[-16:]}"
                )
                return False
        self.tool_frame_id = self.model.getFrameId(self.tool_frame)
        self.flange_frame_id = self.model.getFrameId(self.flange_frame)

        self._q_index = []
        self._v_index = []
        for joint_name in self.controlled_joints:
            if not self.model.existJointName(joint_name):
                self.node.get_logger().error(f"Pinocchio joint '{joint_name}' not found")
                return False
            jid = self.model.getJointId(joint_name)
            self._q_index.append(self.model.joints[jid].idx_q)
            self._v_index.append(self.model.joints[jid].idx_v)

        self.node.get_logger().info(
            f"Loaded Pinocchio model: nq={self.model.nq}, nv={self.model.nv}, "
            f"tool={self.tool_frame}, flange={self.flange_frame}"
        )
        return True

    def _load_robot_description(self):
        """读取 robot_description 参数；若没有，则从 franka_gazebo_bringup 的 xacro 生成 URDF。"""
        if self.node.has_parameter("robot_description"):
            param_value = self.node.get_parameter("robot_description").value
            if isinstance(param_value, str) and param_value.strip().startswith("<"):
                return param_value

        xacro_path = ""
        if self.node.has_parameter("xacro_path"):
            xacro_path = self.node.get_parameter("xacro_path").value
        if not xacro_path:
            try:
                xacro_path = os.path.join(
                    get_package_share_directory("franka_gazebo_bringup"),
                    "urdf",
                    "franka_arm.gazebo.xacro",
                )
            except Exception:
                xacro_path = str(
                    Path.home()
                    / "franka_ros2_ws/src/franka_ros2/franka_gazebo_bringup/urdf/franka_arm.gazebo.xacro"
                )

        doc = xacro.process_file(
            xacro_path,
            mappings={
                "robot_type": self.robot_type,
                "hand": "false",
                "ros2_control": "true",
                "gazebo": "true",
                "ee_id": "franka_hand",
                "gazebo_effort": "true",
            },
        )
        return doc.toxml()

    def build_full_state(self, q_meas, qd_meas):
        """把 7 维受控关节状态填入 Pinocchio 完整状态向量。"""
        q = pin.neutral(self.model)
        qd = np.zeros(self.model.nv)
        for src, idx in enumerate(self._q_index):
            q[idx] = q_meas[src]
        for src, idx in enumerate(self._v_index):
            qd[idx] = qd_meas[src]
        return q, qd

    def get_frame_state(self, q, qd, frame_id):
        """计算指定坐标系的位置、姿态、线/角雅可比和线/角速度。"""
        pin.forwardKinematics(self.model, self.data, q, qd)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[frame_id]
        jac6 = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        jac6_ctrl = jac6[:, self._v_index]
        v6 = jac6 @ qd
        return (
            placement.translation.copy(),
            placement.rotation.copy(),
            jac6_ctrl[:3, :].copy(),
            jac6_ctrl[3:, :].copy(),
            v6[:3].copy(),
            v6[3:].copy(),
        )

    def gravity(self, q):
        """返回受控 7 个关节对应的广义重力项。"""
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        return np.asarray(g, dtype=float)[self._v_index]
