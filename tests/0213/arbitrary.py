import rospy, sys
import numpy as np
import control as ct
import quaternion

# from numpy import *
from copy import deepcopy
from panda_robot import PandaArm
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from franka_interface import ArmInterface
from scipy.spatial.transform import Rotation
import subprocess
import time
from matplotlib import pyplot as plt
import moveit_commander
import moveit_msgs.msg
from openpyxl import Workbook
from franka_core_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from scipy.spatial.distance import cosine
import control as ct
import copy

from panda_robot import PandaKinematics
from std_msgs.msg import Float64, MultiArrayDimension, Float64MultiArray
from pathlib import Path
from matplotlib import font_manager
import os
from fuzzy_logic import FuzzyLogicTool
from scipy.linalg import expm, block_diag
from scipy.spatial.distance import cdist
from scipy.signal import butter, filtfilt
import copy

# def quaternion_to_euler(quaternion_angles):
#     # 将四元数转换为欧拉角
#     rotation = Rotation.from_quat(quaternion_angles)
#     euler = rotation.as_euler("xyz", degrees=False)
#     return euler
# ========== 预初始化缓存数组（避免每次分配内存） ==========
# 提前初始化固定维度的数组，复用内存
TOOL_POS_CACHE = np.zeros(3)
TOOL_EULER_CACHE = np.zeros(3)
TOOL_VEL_POS_CACHE = np.zeros(3)
TOOL_VEL_EULER_CACHE = np.zeros(3)
FLANGE_POS_CACHE = np.zeros(3)
FLANGE_EULER_CACHE = np.zeros(3)
FLANGE_VEL_POS_CACHE = np.zeros(3)
FLANGE_VEL_EULER_CACHE = np.zeros(3)


# ========== 预初始化状态向量缓存（避免每次hstack） ==========
POS_STATE_VEC_CACHE = np.zeros(6)  # 位置误差(3)+速度误差(3)
ROT_STATE_VEC_CACHE = np.zeros(6)  # 位置误差(3)+速度误差(3)

U_POS_CACHE = np.zeros(3)  # 控制输入缓存
U_ROT_CACHE = np.zeros(3)

P_pos = 100
P_ori = 20
# damping gains
D_pos = 10
D_ori = 1

I_pos = 100
I_ori = 30


def quaternion_to_euler(quaternion_angles):
    # 将四元数转换为欧拉角
    rotation = Rotation.from_quat(quaternion_angles)
    euler = rotation.as_euler("xyz", degrees=False)

    return euler


def get_axis_vectors_direct(euler_angles=None, quat=None, order="xyz", degrees=False):
    """直接旋转基向量获取方向"""
    if euler_angles is not None:
        rotation = Rotation.from_euler(order, euler_angles, degrees=degrees)
    elif quat is not None:
        rotation = Rotation.from_quat(quat)
    else:
        raise ValueError("必须提供欧拉角或四元数")

    x_dir = rotation.apply([1, 0, 0])
    y_dir = rotation.apply([0, 1, 0])
    z_dir = rotation.apply([0, 0, 1])
    return x_dir, y_dir, z_dir


def build_homogeneous_transform(x_dir, y_dir, z_dir, translation):
    """根据方向向量和平移坐标构建齐次变换矩阵"""
    # 验证方向向量是否正交单位化（此处假设已满足条件）
    R = np.column_stack([x_dir, y_dir, z_dir])
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = translation
    return H


def wrap_angle_diff(diff):
    """将角度差值调整到 (-π, π] 区间"""
    # 方法 1：取模后调整
    wrapped_diff = diff % (2 * np.pi)
    wrapped_diff[wrapped_diff > np.pi] -= 2 * np.pi
    return wrapped_diff

    # 方法 2：一步公式（等效）
    # return (diff + np.pi) % (2 * np.pi) - np.pi


def euler_angle_diff(a, b):
    """计算两个欧拉角之间的差值（支持单角度或多轴欧拉角）"""
    diff = b - a
    return wrap_angle_diff(diff)


# -------------------------- Ubuntu中文配置（强制加载字体） --------------------------
def setup_chinese_font_ubuntu():
    font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
    if not os.path.exists(font_path):
        print(f"❌ 字体文件不存在：{font_path}")
        print("👉 请先执行：sudo apt-get install -y fonts-wqy-zenhei")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False
        return
    try:
        font_prop = font_manager.FontProperties(fname=font_path)
        plt.rcParams["font.sans-serif"] = [font_prop.get_name(), "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        print(f"✅ 中文配置成功，使用字体：{font_prop.get_name()}")
    except Exception as e:
        print(f"⚠️  中文配置警告：{e}")
        plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False


def make_log_path(filename, time_str, root="/home/hjh/ljj_ws/logs"):

    save_dir = Path(root) / time_str
    save_dir.mkdir(parents=True, exist_ok=True)
    return save_dir / filename


def update_robot_state(kinematics_tool, kinematics_flange, cache_enable=True):
    """
    优化版机器人状态更新：减少冗余计算+缓存复用+高效欧拉角转换
    :param kinematics_tool: PandaKinematics实例（tool_link）
    :param kinematics_flange: PandaKinematics实例（flange_link）
    :param cache_enable: 是否启用数组缓存（默认True，提升性能）
    :return: dict - 结构化机器人状态
    """
    # ========== 1. 工具末端位姿（核心优化：减少数组转换） ==========
    try:
        # 运动学求解结果直接复用（避免多次np.array转换）
        tool_pose = (
            kinematics_tool.forward_position_kinematics()
        )  # 原始返回tuple/list，长度7

        if cache_enable:
            TOOL_POS_CACHE[:] = tool_pose[:3]  # 直接写入缓存，不分配新内存
            tool_position = TOOL_POS_CACHE
        else:
            tool_position = np.array(tool_pose[:3], dtype=np.float64)

        # 优化：用scipy的Rotation替代低效的自定义四元数转欧拉角
        # 四元数格式：(x,y,z,w) → scipy要求(w,x,y,z)，直接切片避免roll操作
        tool_quat = (
            tool_pose[6],
            tool_pose[3],
            tool_pose[4],
            tool_pose[5],
        )  # (w,x,y,z)
        tool_rot = Rotation.from_quat(tool_quat)
        if cache_enable:
            TOOL_EULER_CACHE[:] = tool_rot.as_euler("xyz", degrees=False)
            tool_rotation_euler = TOOL_EULER_CACHE
        else:
            tool_rotation_euler = tool_rot.as_euler("xyz", degrees=False)

        tool_vel = np.array(list(kinematics_tool.forward_velocity_kinematics()))
        if cache_enable:
            TOOL_VEL_POS_CACHE[:] = tool_vel[:3]
            TOOL_VEL_EULER_CACHE[:] = tool_vel[3:6]
            tool_pos_vel = TOOL_VEL_POS_CACHE
            tool_euler_vel = TOOL_VEL_EULER_CACHE
        else:
            tool_pos_vel = np.array(tool_vel[:3], dtype=np.float64)
            tool_euler_vel = np.array(tool_vel[3:6], dtype=np.float64)

    except Exception as e:
        rospy.logerr(f"工具末端状态更新失败：{str(e)}")
        tool_position = np.zeros(3) if not cache_enable else TOOL_POS_CACHE
        tool_rotation_euler = np.zeros(3) if not cache_enable else TOOL_EULER_CACHE

    # ========== 2. 法兰盘位姿+速度（核心优化：轻量化异常捕获） ==========
    try:
        flange_pose = kinematics_flange.forward_position_kinematics()
        if cache_enable:
            FLANGE_POS_CACHE[:] = flange_pose[:3]
            flange_position = FLANGE_POS_CACHE
        else:
            flange_position = np.array(flange_pose[:3], dtype=np.float64)

        # 法兰盘四元数转欧拉角（优化：移除冗余roll操作）
        flange_quat = (flange_pose[3], flange_pose[4], flange_pose[5], flange_pose[6])
        flange_rot = Rotation.from_quat(flange_quat)
        if cache_enable:
            FLANGE_EULER_CACHE[:] = flange_rot.as_euler("xyz", degrees=False)
            flange_rotation_euler = FLANGE_EULER_CACHE
        else:
            flange_rotation_euler = flange_rot.as_euler("xyz", degrees=False)

        # 法兰盘速度：直接复用列表转数组，避免多次list()转换
        flange_vel = np.array(list(kinematics_flange.forward_velocity_kinematics()))
        if cache_enable:
            FLANGE_VEL_POS_CACHE[:] = flange_vel[:3]
            FLANGE_VEL_EULER_CACHE[:] = flange_vel[3:6]
            flange_pos_vel = FLANGE_VEL_POS_CACHE
            flange_euler_vel = FLANGE_VEL_EULER_CACHE
        else:
            flange_pos_vel = np.array(flange_vel[:3], dtype=np.float64)
            flange_euler_vel = np.array(flange_vel[3:6], dtype=np.float64)

    except Exception as e:
        rospy.logerr(f"法兰盘状态更新失败：{str(e)}")
        flange_position = np.zeros(3) if not cache_enable else FLANGE_POS_CACHE
        flange_rotation_euler = np.zeros(3) if not cache_enable else FLANGE_EULER_CACHE
        flange_pos_vel = np.zeros(3) if not cache_enable else FLANGE_VEL_POS_CACHE
        flange_euler_vel = np.zeros(3) if not cache_enable else FLANGE_VEL_EULER_CACHE

    # ========== 3. 结构化返回（仅保留核心字段，减少冗余） ==========
    robot_state = {
        "tool_position": tool_position,
        "tool_rotation_euler": tool_rotation_euler,
        "tool_position_velocity": tool_pos_vel,
        "tool_rotation_euler_velocity": tool_euler_vel,
        "flange_position": flange_position,
        "flange_rotation_euler": flange_rotation_euler,
        "flange_position_velocity": flange_pos_vel,
        "flange_rotation_euler_velocity": flange_euler_vel,
    }
    return robot_state


def update_u_position(ref_state, robot_state, K_gt, integration_delta_pos, dot_time):
    """
    优化版控制输入更新：矩阵预处理+缓存复用+轨迹变形按需调用
    :param td: TrajDeformMoveit实例
    :param robot_state: dict - 优化后的机器人状态
    :param K_gt: np.array - LQR增益矩阵
    :param alpha: 人机权重系数（外部传入，避免硬编码）
    :param u_max: 控制输入上限（外部传入）
    :param deform_threshold: 轨迹变形调用阈值（None则默认td.traj_N-1）
    :return: np.array (3,) - 优化后的u_position
    """
    # ========== 3. 状态误差计算（复用缓存数组，避免hstack） ==========
    # 位置误差：直接写入缓存前3维
    POS_STATE_VEC_CACHE[:3] = (
        ref_state["flange_position"] - robot_state["flange_position"]
    )
    # 速度误差：直接写入缓存后3维
    POS_STATE_VEC_CACHE[3:] = (
        ref_state["flange_position_velocity"] - robot_state["flange_position_velocity"]
    )
    integration_delta_pos_new = (
        integration_delta_pos + POS_STATE_VEC_CACHE[:3] * dot_time
    )

    # ========== 4. LQR控制输入（优化矩阵乘法） ==========
    # 直接用缓存的状态向量计算，避免hstack分配新内存
    U_POS_CACHE = (
        K_gt @ POS_STATE_VEC_CACHE + I_pos * integration_delta_pos_new
    )  # 矩阵乘法优化

    return U_POS_CACHE, integration_delta_pos_new


def update_ref_state(
    zref_r_array,
    zref_h_array,
    Q_c,
    Q_h,
    Q_r,
    length,
    trocar_position,
    position_rcm,
    robot_state,
):
    zref_h_array_temp = np.hstack((zref_h_array[0:3], zref_h_array[6:9]))
    zref_r_array_temp = np.hstack((zref_r_array[0:3], zref_r_array[6:9]))
    state_ref_temp = np.dot(
        np.transpose(np.linalg.inv(Q_c)),
        (
            np.dot(np.transpose(Q_h), zref_h_array_temp)
            + np.dot(np.transpose(Q_r), zref_r_array_temp)
        ),
    )
    state_ref = np.hstack(
        (state_ref_temp[0:3], np.zeros(3), state_ref_temp[3:6], np.zeros(3))
    )
    ref_tool_position = state_ref[0:3]

    ref_flange_position = ref_tool_position + length * (
        trocar_position - ref_tool_position
    ) / np.linalg.norm(trocar_position - ref_tool_position)
    flange_z_dir_ref = (ref_tool_position - trocar_position) / np.linalg.norm(
        ref_tool_position - trocar_position
    )
    reference_direction = np.array([0, -1, 0])  # 全局y轴
    flange_y_dir_ref = (
        reference_direction
        - np.dot(reference_direction, flange_z_dir_ref) * flange_z_dir_ref
    )
    flange_y_dir_ref = flange_y_dir_ref / np.linalg.norm(flange_y_dir_ref)

    flange_x_dir_ref = np.cross(flange_y_dir_ref, flange_z_dir_ref)
    flange_HT_ref = build_homogeneous_transform(
        flange_x_dir_ref, flange_y_dir_ref, flange_z_dir_ref, ref_flange_position
    )
    flange_rotation_matrix_ref = Rotation.from_matrix(flange_HT_ref[:3, :3])
    ref_flange_rotation_euler = flange_rotation_matrix_ref.as_euler(
        "xyz", degrees=False
    )

    distance_rcm_flange = np.linalg.norm(position_rcm - robot_state["flange_position"])
    distance_rcm_tool = np.linalg.norm(position_rcm - robot_state["tool_position"])
    ref_flange_pos_vel = -state_ref[6:9] * (distance_rcm_flange / distance_rcm_tool)
    ref_flange_euler_vel = state_ref[9:12]
    ref_state = {
        "tool_position": ref_tool_position,
        "tool_rotation_euler": ref_flange_rotation_euler,
        "flange_position": ref_flange_position,
        "flange_rotation_euler": ref_flange_rotation_euler,
        "flange_position_velocity": ref_flange_pos_vel,
        "flange_rotation_euler_velocity": ref_flange_euler_vel,
    }
    return ref_state


def update_u_rotation(ref_state, robot_state, integration_delta_euler, dot_time):
    POS_STATE_VEC_CACHE[:3] = euler_angle_diff(
        robot_state["flange_rotation_euler"], ref_state["flange_rotation_euler"]
    )
    # 速度误差：直接写入缓存后3维
    POS_STATE_VEC_CACHE[3:] = euler_angle_diff(
        robot_state["flange_rotation_euler_velocity"],
        ref_state["flange_rotation_euler_velocity"],
    )
    integration_delta_euler_new = (
        integration_delta_euler + POS_STATE_VEC_CACHE[:3] * dot_time
    )
    U_ROT_CACHE = (
        P_ori * POS_STATE_VEC_CACHE[:3]
        + D_ori * POS_STATE_VEC_CACHE[3:]
        + I_ori * integration_delta_euler
    )

    return U_ROT_CACHE, integration_delta_euler_new


def replan_to_target(
    movegroup,
    kinematics_tool,
    td_publishers,
    freq,
    trocar_position,
    length,
    target_pos,
    way_plan,
):
    """
    针对新目标点重新规划轨迹，并初始化TrajDeformMoveit
    :param movegroup: MoveGroupPythonInterfaceTutorial实例
    :param kinematics_tool: 工具端运动学实例
    :param td_publishers: 轨迹变形相关publisher（元组）
    :param freq: 控制频率
    :param trocar_position: RCM点位置
    :param length: 工具长度
    :param target_pos: 新目标点（tool位置）
    :return: 新的TrajDeformMoveit实例
    """
    # 1. 重新规划路径
    # way_plan, quat_plan, joint_plan = movegroup.plan_path(target_pos)
    # 2. 解包publisher
    (
        Publisher_TrajDeformPos,
        Publisher_TrajDeformVel,
        Publisher_InitTrajPos,
        Publisher_InitTrajVel,
    ) = td_publishers
    # 3. 初始化新的轨迹变形实例
    td_new = TrajDeformMoveit(
        freq,
        flag=True,
        traj=way_plan,
        position_trocar=trocar_position,
        length=length,
        movegroup=movegroup,
        pub1=Publisher_TrajDeformPos,
        pub2=Publisher_TrajDeformVel,
        pub3=Publisher_InitTrajPos,
        pub4=Publisher_InitTrajVel,
    )
    td_new.goal = target_pos
    td_new.old_goal = target_pos
    td_new.h = 0  # 重置轨迹计数器
    td_new.k = 0  # 重置轨迹计数器
    rospy.loginfo(f"已重新规划轨迹到目标点: {target_pos}")
    return td_new


def find_closest_goal(current_tool_pos, goal_positions):
    # 方法：先计算每个目标点与当前点的差值，再对每行（每个点）计算范数
    distances = np.linalg.norm(goal_positions - current_tool_pos, axis=1)

    # 4. 找到最小距离的索引和值
    closest_index = np.argmin(distances)  # 最小距离的索引
    min_distance = distances[closest_index]  # 最小距离值
    closest_goal = goal_positions[closest_index]  # 对应的目标向量

    return min_distance, closest_goal, closest_index


class FrankaSharedController:

    def __init__(
        self,
        Np,
        dt,
        length=0.525,
        trocar_position=np.array([0.3, 0, 0.235]),
        obstacles=[np.array([0.7, 0.2, 0.3])],
        boundaries=[np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])],
    ):
        # 系统参数
        self.dof = 3  # 自由度 (位置控制)
        self.Np = Np  # 预测时域
        self.dt = dt  # 控制周期

        # Franka动力学参数 (示例值)
        self.Mx = np.diag([10, 10, 10])  # 惯性矩阵
        self.Cx = np.diag([100, 100, 100])  # 阻尼矩阵

        # 离散系统矩阵
        self.setup_discrete_system()

        # 轨迹重规划参数
        self.delta_f = 3.0  # 交互力阈值 (N)
        self.f_p = 5.0  # 最大重规划频率 (Hz)
        self.replanning_active = False
        self.replanning_start_time = 0

        # PSI参数
        self.a1, self.a2, self.mu, self.eta = 0.99, 0.19, 200, 0.1
        self.d0 = 0.6  # 最大允许偏差 (m)

        # 博弈控制参数
        self.Qh = np.eye(self.dof * Np) * 250000  # 人类跟踪误差权重
        self.Qr = np.eye(self.dof * Np) * 250000  # 机器人跟踪误差权重
        self.rho = 0.9  # 凸迭代权重,[0,1]
        self.max_iter = 10  # 凸迭代次数
        self.tol = 0.1

        # 交互力滤波器
        self.filter_cutoff = 10  # Hz
        self.b, self.a = butter(2, self.filter_cutoff * dt)
        self.filtered_Fh = np.zeros(self.dof)

        # 状态变量
        self.current_trajectory = None
        self.reference_trajectory = None

        self.P_ori = 20
        self.D_ori = 1
        self.I_ori = 30

        self.integration_delta_euler = np.zeros(3)

        self.length = length
        self.trocar_position = trocar_position

        self.state_Xr = np.zeros(3)
        self.state_Xh = np.zeros(3)

        self.obstacles = obstacles
        self.boundaries = boundaries

        self.ref_h = np.zeros(3)
        self.ref_r = np.zeros(3)

        self.ref_h_output = np.zeros(3)
        self.ref_r_output = np.zeros(3)

    def setup_discrete_system(self):
        """构建离散状态空间方程"""
        # 连续系统矩阵
        A_cont = np.block(
            [
                [np.zeros((self.dof, self.dof)), np.eye(self.dof)],
                [np.zeros((self.dof, self.dof)), -np.linalg.inv(self.Mx) @ self.Cx],
            ]
        )
        B_cont = np.vstack([np.zeros((self.dof, self.dof)), np.linalg.inv(self.Mx)])

        # 离散化 (矩阵指数法)
        n = A_cont.shape[0]
        M = np.zeros((n + self.dof, n + self.dof))
        M[:n, :n] = A_cont
        M[:n, n:] = B_cont
        M_exp = expm(M * self.dt)

        self.Ad = M_exp[:n, :n]
        self.Brd = M_exp[:n, n:]
        self.Bhd = self.Brd.copy()

    def update_filtered_force(self, Fh):
        """更新滤波后的交互力"""
        self.filtered_Fh = filtfilt(self.b, self.a, Fh)
        return self.filtered_Fh

    def check_replanning_trigger(self):
        """检查是否触发重规划"""
        force_magnitude = np.linalg.norm(self.filtered_Fh)
        current_time = time.time()

        if force_magnitude > self.delta_f:
            if not self.replanning_active:
                self.replanning_active = True
                self.replanning_start_time = current_time
            elif current_time - self.replanning_start_time > 1 / self.f_p:
                return True
        else:
            self.replanning_active = False

        return False

    def local_replanning(self, start_pos, end_pos, obstacles):
        """局部轨迹重规划 (简化实现)"""
        # 实际应用中应使用RRT*等算法
        # 这里简化为直线路径添加偏移
        direction = self.filtered_Fh / np.linalg.norm(self.filtered_Fh)
        offset = direction * 0.1  # 10cm偏移

        # 生成新轨迹
        new_trajectory = np.linspace(start_pos, end_pos, self.Np)
        new_trajectory += offset.reshape(1, -1)

        return new_trajectory

    def compute_psi(self, x, gamma_d, obstacles, boundaries):
        """计算预测安全指数PSI"""
        # 计算到障碍物的最小距离
        dist_to_obs = cdist([x], obstacles).min() if len(obstacles) > 0 else np.inf

        # 计算到边界的最小距离
        dist_to_boundary = min(
            np.min(x - boundaries[0]), np.min(boundaries[1] - x)  # 下边界  # 上边界
        )

        # 最大允许偏差
        d_res = min(self.d0, dist_to_obs, dist_to_boundary)

        # 实际位置偏差
        d = np.linalg.norm(x - gamma_d)
        d_max = min(d, d_res)

        # Richards曲线映射
        exponent = -self.mu * (d_max - self.a2 * d_res)
        d_sat = self.a1 * d_res / (1 + self.eta * np.exp(exponent)) ** (1 / self.eta)

        # 计算PSI
        return np.sqrt(max(0, d_res**2 - d_sat**2)) / d_res

    def build_prediction_matrices(self, size):
        """构建预测矩阵Psi_g, theta_h_g, theta_r_g"""
        Psi = np.zeros((self.dof * size, 2 * self.dof))
        theta_h = np.zeros((self.dof * size, self.dof * size))
        theta_r = np.zeros_like(theta_h)

        # C矩阵 (位置输出)
        C = np.hstack([np.eye(self.dof), np.zeros((self.dof, self.dof))])

        # 计算Psi矩阵: Psi = [C*A_d; C*A_d^2; ...; C*A_d^{Np}]
        Ad_power = np.eye(2 * self.dof)
        for i in range(size):
            Ad_power = Ad_power @ self.Ad
            Psi[i * self.dof : (i + 1) * self.dof] = C @ Ad_power

        # 计算theta_h和theta_r矩阵
        for i in range(size):
            for j in range(i + 1):
                # 计算A_d^{i-j}
                power = i - j
                if power == 0:
                    Ad_power = np.eye(2 * self.dof)
                else:
                    Ad_power = np.linalg.matrix_power(self.Ad, power)

                # theta_h[i,j] = C * A_d^{i-j} * B_{hd}
                theta_h[
                    i * self.dof : (i + 1) * self.dof, j * self.dof : (j + 1) * self.dof
                ] = (C @ Ad_power @ self.Bhd)

                # theta_r[i,j] = C * A_d^{i-j} * B_{rd}
                theta_r[
                    i * self.dof : (i + 1) * self.dof, j * self.dof : (j + 1) * self.dof
                ] = (C @ Ad_power @ self.Brd)

        # 构建全局矩阵
        Psi_g = np.vstack([Psi, Psi])
        theta_h_g = np.vstack([theta_h, theta_h])
        theta_r_g = np.vstack([theta_r, theta_r])

        return Psi_g, theta_h_g, theta_r_g

    def decompose_to_S(self, Q, eps=1e-10):
        """
        将对称半正定矩阵Q分解为S^T * S，其中S与Q形状相同（均为n×n）。

        参数:
            Q: 待分解的n×n矩阵（需为对称半正定矩阵）
            eps: 阈值，用于判断特征值是否为0（默认1e-10）

        返回:
            S: 分解得到的n×n矩阵，满足S^T * S = Q
        """
        # 确保Q是对称矩阵（消除数值误差导致的微小不对称）
        Q = (Q + Q.T) / 2
        n = Q.shape[0]

        # 检查矩阵形状是否为方阵
        if Q.shape != (n, n):
            raise ValueError("Q必须是方阵")

        # 对对称矩阵进行特征值分解（eigh适用于对称矩阵，更稳定）
        eig_vals, eig_vecs = np.linalg.eigh(Q)

        # 检查是否为半正定矩阵（特征值允许微小负值，由数值误差导致）
        if np.any(eig_vals < -eps):
            raise ValueError("Q不是半正定矩阵，无法分解为S^T * S")

        # 构造对角矩阵D：特征值的平方根（小于eps的特征值视为0，避免数值问题）
        sqrt_eig = np.sqrt(np.maximum(eig_vals, eps))  # 确保非负，取平方根
        D = np.diag(sqrt_eig)  # n×n对角矩阵

        # 构造S：S = V * D * V^T（V是特征向量矩阵，正交矩阵）
        S = eig_vecs @ D @ eig_vecs.T  # 矩阵乘法

        return S

    def solve_pareto_optimal(self, w, Xdh, Xdr, PSI, Psi_g, theta_h_g, theta_r_g, size):
        """求解Pareto最优控制输入"""
        # 构建全局目标向量
        Xd = np.concatenate([Xdh, Xdr])
        Qh = self.Qh[: size * self.dof, : size * self.dof]
        Qr = self.Qr[: size * self.dof, : size * self.dof]

        # 构建权重矩阵Q(PSI)

        Q = block_diag(PSI * Qh, (1 - PSI) * Qr)

        if PSI != 0 and PSI != 1:
            S_Q = np.linalg.cholesky(Q)
        elif PSI == 0:
            S_Q = block_diag(np.zeros(Qh.shape), np.linalg.cholesky(Qr))
        else:
            S_Q = block_diag(np.linalg.cholesky(Qh), np.zeros(Qr.shape))

        # 初始化控制序列
        Uh = np.zeros(theta_h_g.shape[1])
        Ur = np.zeros(theta_r_g.shape[1])

        # 凸迭代求解
        for iter_ in range(self.max_iter):
            # 计算误差项
            epsilon_r = Xd - Psi_g @ w - theta_h_g @ Uh
            epsilon_h = Xd - Psi_g @ w - theta_r_g @ Ur

            # 求解人类控制序列
            time_start_temp = time.perf_counter()

            # L_h = np.linalg.pinv(
            #     np.vstack([S_Q @ theta_h_g, np.sqrt(1 - PSI) * np.eye(len(Uh))])
            # ) @ np.vstack([S_Q, np.zeros((len(Uh), S_Q.shape[1]))])

            L_h, residuals, rank, s = np.linalg.lstsq(
                np.vstack([S_Q @ theta_h_g, np.sqrt(1 - PSI) * np.eye(len(Uh))]),
                np.vstack([S_Q, np.zeros((len(Uh), S_Q.shape[1]))]), rcond=None
            )
            U_h_new = L_h @ epsilon_h

            # 求解机器人控制序列
            # L_r = np.linalg.pinv(
            #     np.vstack([S_Q @ theta_r_g, np.sqrt(PSI) * np.eye(len(Ur))])
            # ) @ np.vstack([S_Q, np.zeros((len(Ur), S_Q.shape[1]))])

            L_r, residuals, rank, s = np.linalg.lstsq(
                np.vstack([S_Q @ theta_r_g, np.sqrt(PSI) * np.eye(len(Ur))]),
                np.vstack([S_Q, np.zeros((len(Ur), S_Q.shape[1]))]), rcond=None
            )
            U_r_new = L_r @ epsilon_r

            time_end_temp = time.perf_counter()
            timepass_temp = time_end_temp - time_start_temp
            # print("timepass:", timepass_temp, "iter:", iter_)

            # 更新控制序列
            Uh_new = self.rho * U_h_new + (1 - self.rho) * Uh
            Ur_new = self.rho * U_r_new + (1 - self.rho) * Ur

            # 检查收敛
            if (
                np.linalg.norm(Uh_new - Uh) < self.tol
                and np.linalg.norm(Ur_new - Ur) < self.tol
            ):
                # print(f"epsilon_r = {epsilon_r[0:3]}, epsilon_h = {epsilon_h[0:3]}")
                break

            Uh, Ur = Uh_new, Ur_new

        # 返回当前控制输入

        return Uh[: self.dof], Ur[: self.dof]

    def compute_control(self, state, Fh, k, if_rcm, ws_PSI, ws_deform, PSI=0.5):
        """
        主控制循环
        state: [位置, 速度] (2*dof)
        Fh: 当前交互力 (dof)
        obstacles: 障碍物位置列表 (N x 3)
        boundaries: 边界 [min_corner, max_corner]
        """

        # 检查是否触发重规划
        # if self.check_replanning_trigger():
        #     start_pos = self.current_trajectory[0]
        #     end_pos = self.current_trajectory[-1]
        #     self.reference_trajectory = self.local_replanning(
        #         start_pos, end_pos, self.obstacles
        #     )

        # 获取当前位置
        current_pos = state[: self.dof]

        # 计算PSI
        # PSI = self.compute_psi(
        #     current_pos, self.reference_trajectory[0], self.obstacles, self.boundaries
        # )

        # ws_PSI.append(PSI.flatten().tolist())
        # PSI = 0.5

        # 构建机器人期望轨迹
        Xdr = self.reference_trajectory[k : k + self.Np].flatten()
        flag = np.max(Fh) == 0 and np.min(Fh) == 0
        temp_Np = int(Xdr.size / self.dof)
        # 构建人类期望轨迹 (假设恒定力)
        Xdh = np.zeros(len(Xdr))
        w_h = state.copy()
        self.ref_r_output = Xdr[0:3].copy()

        if flag:
            Xdh = Xdr
            if if_rcm:
                self.ref_h_output = Xdh[0:3].copy()
                for i in range(temp_Np):
                    self.ref_h = Xdh[i * self.dof : (i + 1) * self.dof].copy()
                    self.ref_r = Xdr[i * self.dof : (i + 1) * self.dof].copy()
                    temp1 = self.ref_h + self.length * (
                        self.trocar_position - self.ref_h
                    ) / np.linalg.norm(self.trocar_position - self.ref_h)
                    temp2 = self.ref_r + self.length * (
                        self.trocar_position - self.ref_r
                    ) / np.linalg.norm(self.trocar_position - self.ref_r)

                    Xdh[i * self.dof : (i + 1) * self.dof] = temp1
                    Xdr[i * self.dof : (i + 1) * self.dof] = temp2

        else:
            for j in range(temp_Np):
                w_h = self.Ad @ w_h + self.Bhd @ Fh
                Xdh[j * self.dof : (j + 1) * self.dof] = w_h[: self.dof]
                # temp_w = self.Bhd @ Fh
                temp_w_h = w_h[: self.dof].copy()
                self.ref_h = temp_w_h + self.length * (
                    self.trocar_position - temp_w_h
                ) / np.linalg.norm(self.trocar_position - temp_w_h)
                if j == 0:
                    self.ref_h_output = self.ref_h.copy()

            if if_rcm:
                for i in range(temp_Np):
                    self.ref_r = Xdr[i * self.dof : (i + 1) * self.dof].copy()
                    temp2 = self.ref_r + self.length * (
                        self.trocar_position - self.ref_r
                    ) / np.linalg.norm(self.trocar_position - self.ref_r)

                    # temp4 = Xdr[i * self.dof : (i + 1) * self.dof]

                    Xdr[i * self.dof : (i + 1) * self.dof] = temp2
                    if i == 0:
                        self.ref_r_output = self.ref_r.copy()

        ws_deform.append(Xdh[0:6].flatten().tolist())
        # 构建预测矩阵
        Psi_g, theta_h_g, theta_r_g = self.build_prediction_matrices(temp_Np)

        # 求解Pareto最优控制

        u_h, u_r = self.solve_pareto_optimal(
            state, Xdh, Xdr, PSI, Psi_g, theta_h_g, theta_r_g, temp_Np
        )

        self.state_Xr = Xdr[0:3]
        self.state_Xh = Xdh[0:3]

        return u_h + u_r

    def compute_rcm(self, pose_ref, robot_state, integration_euler, delta_time):
        # 工具末端状态更新

        tool_position_ref = pose_ref[0:3]

        flange_position_ref = tool_position_ref + self.length * (
            self.trocar_position - tool_position_ref
        ) / np.linalg.norm(self.trocar_position - tool_position_ref)
        flange_z_dir_ref = (tool_position_ref - self.trocar_position) / np.linalg.norm(
            tool_position_ref - self.trocar_position
        )
        reference_direction = np.array([0, -1, 0])  # 全局y轴
        flange_y_dir_ref = (
            reference_direction
            - np.dot(reference_direction, flange_z_dir_ref) * flange_z_dir_ref
        )
        flange_y_dir_ref = flange_y_dir_ref / np.linalg.norm(flange_y_dir_ref)

        flange_x_dir_ref = np.cross(flange_y_dir_ref, flange_z_dir_ref)
        # flange_y_dir_ref = np.cross(flange_z_dir_ref, flange_x_dir)
        # flange_x_dir_ref = np.cross(flange_y_dir_ref, flange_z_dir_ref)
        # flange_x_dir_ref = np.array([1, 0, 0])
        # print(np.dot(flange_x_dir_ref,flange_y_dir_ref)," ",np.dot(flange_x_dir_ref,flange_z_dir_ref)," ",np.dot(flange_y_dir_ref,flange_z_dir_ref))
        flange_HT_ref = build_homogeneous_transform(
            flange_x_dir_ref, flange_y_dir_ref, flange_z_dir_ref, flange_position_ref
        )
        flange_rotation_matrix_ref = Rotation.from_matrix(flange_HT_ref[:3, :3])
        flange_rotation_euler_ref = flange_rotation_matrix_ref.as_euler(
            "xyz", degrees=False
        )
        flange_rotation_quat_ref = flange_rotation_matrix_ref.as_quat()

        position_rcm = robot_state["flange_position"] + (
            (
                (robot_state["tool_position"] - robot_state["flange_position"])
                @ (self.trocar_position - robot_state["flange_position"])
                * (robot_state["tool_position"] - robot_state["flange_position"])
            )
            / np.square(self.length)
        )

        goal_euler = flange_rotation_euler_ref
        now_euler = robot_state["flange_rotation_euler"]
        delta_euler = euler_angle_diff(now_euler, goal_euler)

        goal_omg = np.zeros(3)
        now_omg = robot_state["flange_rotation_euler_velocity"]
        delta_omg = euler_angle_diff(now_omg, goal_omg)

        integration_euler = integration_euler + delta_euler * delta_time

        if np.linalg.norm(delta_euler)>0.1:
            I_ori=self.I_ori/2
        else:
            I_ori=self.I_ori

        u_rotation = (
            self.P_ori * delta_euler
            + self.D_ori * delta_omg
            + I_ori * integration_euler
        )
        return u_rotation, integration_euler


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, r):
        self.Panda_Arm = r
        self.kinematics = PandaKinematics(self.Panda_Arm, "panda_link10")
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        move_group.set_end_effector_link("panda_link10")
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.wb_joint = Workbook()  # 创建一个新的工作簿
        self.ws_joint = self.wb_joint.active  # 选择默认的工作表
        self.ws_joint.append(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        )

    def plan_cartesian_path(self, goal_pose):
        neu_j = [
            self.Panda_Arm._neutral_pose_joints[j] for j in self.Panda_Arm._joint_names
        ]
        neu_pos, neu_rot = self.Panda_Arm.forward_kinematics(neu_j)
        angle = self.Panda_Arm.angles()
        ee_pose, ee_eul = self.Panda_Arm.ee_pose()
        status, angle2 = self.Panda_Arm.inverse_kinematics(goal_pose, ee_eul)
        error_code, trajectory_msg, planning_time, error_code = self.move_group.plan(
            angle2
        )
        way_plan = []
        quat_plan = []
        angle = self.Panda_Arm.angles()
        ee_pose, ee_angle = self.Panda_Arm.ee_pose()
        angle2 = self.Panda_Arm.inverse_kinematics(ee_pose)

        for i in range(len(trajectory_msg.joint_trajectory.points)):
            joint = trajectory_msg.joint_trajectory.points[i].positions
            pose, eul = self.Panda_Arm.forward_kinematics(joint, "eul")
            way_plan.append(pose)
            quat_plan.append(eul)
        way_plan = np.reshape(way_plan, (-1, 3))
        return way_plan

    def plan_path(self, goal_pose):
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = goal_pose[0]
        wpose.position.y = goal_pose[1]
        wpose.position.z = goal_pose[2]
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.00010, 0.0  # waypoints to follow  # eef_step
        )

        joint_plan = []
        position_plan = []
        rotation_plan = []
        for i in range(len(plan.joint_trajectory.points)):
            joint = plan.joint_trajectory.points[i].positions
            joint_plan.append(joint)
            self.ws_joint.append(joint)
            argument = dict(zip(self.Panda_Arm.joint_names(), joint))
            pose = np.array(self.kinematics.forward_position_kinematics(argument))

            position = pose[0:3]
            position_plan.append(position)
            w = pose[6]
            x = pose[3]
            y = pose[4]
            z = pose[5]  # quarternions

            rotation = quaternion.as_euler_angles(quaternion.quaternion(w, x, y, z))
            rotation_plan.append(rotation)

        #
        # way_plan = []
        # quat_plan = []
        # for i in range(len(plan.joint_trajectory.points)):
        #     joint = plan.joint_trajectory.points[i].positions
        #     pose, eul = self.Panda_Arm.forward_kinematics(joint, "eul")
        #     way_plan.append(pose)
        #     quat_plan.append(eul)
        # way_plan = np.reshape(way_plan, (-1, 3))
        position_plan = np.reshape(position_plan, (-1, 3))
        rotation_plan = np.reshape(rotation_plan, (-1, 3))
        joint_plan = np.reshape(joint_plan, (-1, 7))
        return position_plan, rotation_plan, joint_plan


class GoalIntent:
    def __init__(self):
        return None

    def calculate_transition_probabilities(self, n, delta):
        """
        计算目标转移概率矩阵。

        参数：
        n (int): 目标的数量。
        delta (float): 目标转移的调整参数，范围在[0, 1]之间。

        返回：
        np.ndarray: 转移概率矩阵，大小为(n, n)。
        """
        # 创建一个大小为n x n的矩阵来表示目标转移概率
        transition_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                if i == j:
                    transition_matrix[i, j] = 1 - delta  # 目标没有改变
                else:
                    transition_matrix[i, j] = delta / (n - 1)  # 目标发生改变

        return transition_matrix

    def compute_distance(self, robot_pos, goal_pos):
        """
        计算机器人当前位置与目标位置之间的欧氏距离。
        """
        return np.linalg.norm(np.array(robot_pos) - np.array(goal_pos))

    def boltzmann_rationality(self, robot_pos, goal_pos, beta, action):
        """
        使用Boltzmann理性模型计算人类动作的概率。

        参数：
        robot_pos (tuple): 机器人的当前位置 (x, y, z)。
        goal_pos (tuple): 目标位置 (x, y, z)。
        beta (float): 理性指数，用于控制动作选择的理性。
        action (np.array): 人类的控制命令。
        autonomy_action (np.array): 自主控制命令。

        返回：
        float: 基于Boltzmann模型的动作概率。
        """
        autonomy_action = goal_pos - robot_pos
        # 计算人类控制命令与自主控制命令的余弦相似度
        cos_sim = 1 - cosine(action, autonomy_action)
        boltzmann_rationality_ = np.exp(beta * cos_sim)
        return boltzmann_rationality_

    def rbii_2(self, robot_pos, goal_positions, actions, kappa, beta):
        """
        RBII-2方法实现，考虑多个观察源（距离和动作）。

        参数：
        robot_pos (tuple): 机器人的当前位置 (x, y, z)。
        goal_positions (list of tuples): 目标位置的列表 [(x1, y1, z1), (x2, y2, z2), ...]。
        actions (list of np.array): 人类控制命令列表，每个控制命令为一个向量 [x, y, z]。
        kappa (float): 距离的缩放因子。
        beta (float): 理性指数，用于控制人类动作的理性。

        返回：
        list: 每个目标的概率分布。
        """
        # 计算距离目标的似然性
        distances = [
            self.compute_distance(robot_pos, goal_pos) for goal_pos in goal_positions
        ]
        distance_likelihoods = np.exp(-kappa * np.array(distances))

        # 计算人类控制命令的Boltzmann理性概率
        action_likelihoods = []
        for i in range(goal_positions.shape[0]):
            action_likelihood = self.boltzmann_rationality(
                robot_pos, goal_positions[i], beta, actions
            )
            action_likelihoods.append(action_likelihood)

        # 融合两个观察源的似然性
        total_likelihoods = distance_likelihoods * np.array(action_likelihoods)

        # 归一化得到目标的概率分布
        probabilities = total_likelihoods / np.sum(total_likelihoods)

        return probabilities

    def bayesian_intent(self, likelihood, transition, posterior_old):
        size = likelihood.size
        posterior = np.zeros(size)
        prior = np.zeros(size)
        for i in range(size):
            for j in range(size):
                prior[i] += transition[i, j] * posterior_old[j]

            posterior[i] = likelihood[i] * prior[i]
        posterior = posterior / np.sum(posterior)
        return posterior


class CallBackFunction:
    def __init__(self):
        self.jacobian_joint7 = np.zeros((6, 7))
        self.jacobian_flange = np.zeros((6, 7))
        self.jacobian_joint7_vec = np.zeros(42)
        self.jacobian_flange_vec = np.zeros(42)
        self.position_joint7 = np.zeros(3)
        self.euler_joint7 = np.zeros(3)
        self.position_endeffector = np.zeros(3)
        self.position_flange = np.zeros(3)
        self.position_falcon = np.zeros(3)
        self.vel_falcon = np.zeros(3)
        self.force_falcon = np.zeros(3)
        self.force_time_total = 0
        self.force_time_start = time.perf_counter()
        self.force_time_end = time.perf_counter()
        self.button = 0
        self.position_real_falcon = np.zeros(3)
        self.arbitrary_alpha = 0
        self.Kgt = np.zeros((3, 6))
        self.sub1 = rospy.Subscriber(
            "/franka_jacobian_joint7",
            RobotState,
            self.sub_franka_jacobian_joint7,
            queue_size=5,
        )
        self.sub2 = rospy.Subscriber(
            "/franka_position_joint7",
            Point,
            self.sub_franka_position_joint7,
            queue_size=5,
        )
        self.sub3 = rospy.Subscriber(
            "/franka_position_endeffector",
            Point,
            self.sub_franka_position_endeffector,
            queue_size=5,
        )
        self.sub4 = rospy.Subscriber(
            "/falcon/ee_pose", Point, self.sub_falcon_position, queue_size=5
        )
        self.sub5 = rospy.Subscriber(
            "/falcon/velocity", Vector3, self.sub_falcon_vel, queue_size=5
        )
        self.sub6 = rospy.Subscriber(
            "/falcon/joystick", Joy, self.sub_falcon_joystick, queue_size=5
        )
        self.sub7 = rospy.Subscriber(
            "/falcon/forces", Point, self.sub_falcon_force, queue_size=5
        )
        self.sub8 = rospy.Subscriber(
            "/franka_euler_joint7",
            Point,
            self.sub_franka_euler_joint7,
            queue_size=5,
        )
        self.sub9 = rospy.Subscriber(
            "/franka_position_flange",
            Point,
            self.sub_franka_position_flange,
            queue_size=5,
        )

        self.sub10 = rospy.Subscriber(
            "/franka_jacobian_flange",
            RobotState,
            self.sub_franka_jacobian_flange,
            queue_size=5,
        )

        self.sub11 = rospy.Subscriber(
            "/ArbitraryAlpha", Float64, self.sub_arbitrary_alpha, queue_size=5
        )
        self.sub12 = rospy.Subscriber(
            "/KgtMultiArray", Float64MultiArray, self.sub_Kgt, queue_size=5
        )

        self.beta_prev = None
        self.alpha = 0.9  # 平滑系数

        self.beta_window = []  # 滑动窗口，存储最近N个原始beta值
        self.window_size = 5  # 窗口大小（推荐5~10，根据数据波动调整）

        self.alpha_window = []  # 滑动窗口，存储最近N个原始beta值
        self.alpha_window_size = 10  # 窗口大小（推荐5~10，根据数据波动调整）

        rospy.loginfo("CallBackFunction Node is running...")

    def sub_franka_jacobian_joint7(self, franka_jacobian_joint7):
        for i in range(42):
            self.jacobian_joint7_vec[i] = franka_jacobian_joint7.O_Jac_EE[i]
        self.jacobian_joint7 = self.jacobian_joint7_vec.reshape((6, 7), order="F")
        # rospy.loginfo(f"Received message: {self.jacobian_joint7}")

    def sub_franka_position_joint7(self, franka_position_joint7):
        self.position_joint7[0] = franka_position_joint7.x
        self.position_joint7[1] = franka_position_joint7.y
        self.position_joint7[2] = franka_position_joint7.z
        # rospy.loginfo(f"Received position_joint7: {self.position_joint7}")

    def sub_franka_position_endeffector(self, franka_position_endeffector):
        self.position_endeffector[0] = franka_position_endeffector.x
        self.position_endeffector[1] = franka_position_endeffector.y
        self.position_endeffector[2] = franka_position_endeffector.z
        # rospy.loginfo(f"Received position_endeffector: {self.position_endeffector}")

    def sub_falcon_position(self, falcon_position):
        self.position_falcon[0] = falcon_position.x
        self.position_falcon[1] = falcon_position.y
        self.position_falcon[2] = falcon_position.z

    def sub_falcon_vel(self, falcon_vel):
        self.vel_falcon[0] = falcon_vel.x
        self.vel_falcon[1] = falcon_vel.y
        self.vel_falcon[2] = falcon_vel.z

    def sub_falcon_joystick(self, falcon_joystick):
        self.button = falcon_joystick.buttons[0]
        self.position_real_falcon[0] = falcon_joystick.axes[0]
        self.position_real_falcon[1] = falcon_joystick.axes[1]
        self.position_real_falcon[2] = falcon_joystick.axes[2]

    def sub_falcon_force(self, falcon_force):
        if np.linalg.norm(self.force_falcon) == 0:
            self.force_time_start = time.perf_counter()
            self.force_time_total = 0
        else:
            self.force_time_end = time.perf_counter()
            self.force_time_total = self.force_time_end - self.force_time_start
        self.force_falcon[0] = falcon_force.x
        self.force_falcon[1] = falcon_force.y
        self.force_falcon[2] = falcon_force.z

    def sub_franka_euler_joint7(self, franka_euler_joint7):
        self.euler_joint7[0] = franka_euler_joint7.x
        self.euler_joint7[1] = franka_euler_joint7.y
        self.euler_joint7[2] = franka_euler_joint7.z

    def sub_franka_position_flange(self, franka_position_flange):
        self.position_flange[0] = franka_position_flange.x
        self.position_flange[1] = franka_position_flange.y
        self.position_flange[2] = franka_position_flange.z

    def sub_franka_jacobian_flange(self, franka_jacobian_flange):
        for i in range(42):
            self.jacobian_flange_vec[i] = franka_jacobian_flange.O_Jac_EE[i]
        self.jacobian_flange = self.jacobian_flange_vec.reshape((6, 7), order="F")
        # rospy.loginfo(f"Received message: {self.jacobian_joint7}")

    def sub_arbitrary_alpha(self, arbitrary_alpha):
        self.arbitrary_alpha = arbitrary_alpha.data

    def sub_Kgt(self, Kgt):
        # 提取维度信息

        rows = Kgt.layout.dim[0].size
        cols = Kgt.layout.dim[1].size

        # 将数据重塑为原始矩阵
        self.Kgt = np.array(Kgt.data).reshape(rows, cols)
        # rospy.loginfo(f"Received matrix:\n{self.Kgt}")

    def HumanWilling(self, force=None):
        if force == None:
            force = self.force_falcon
        a = 1
        b = 15
        c = 0.4
        Force = np.linalg.norm(force)
        willing = np.divide(a, 1 + np.exp(-b * (Force - c)))
        # print(willing)
        return willing

    def HumanArbitrary(self, force=None):
        if force == None or force == None:
            force = self.force_falcon
        a = 1
        b = 5
        c = 1
        Force = np.linalg.norm(force)
        willing = a/(1 + np.exp(-b * (Force - c)))
        # print(willing)
        return willing

    def DirectWilling(self, distance):
        a = 1
        b = 600
        c = 0.02
        Force = np.linalg.norm(distance)
        willing = np.divide(a, 1 + np.exp(-b * (Force - c)))
        return willing

    def get_smoothed_beta(self, distance):
        # 计算原始beta
        beta_current = self.DirectWilling(distance)
        # 平滑计算
        if self.beta_prev is None:
            beta_smoothed = beta_current
        else:
            beta_smoothed = (
                self.alpha * beta_current + (1 - self.alpha) * self.beta_prev
            )
        # 更新上一次值
        self.beta_prev = beta_smoothed
        return beta_smoothed

    def get_smoothed_beta_window(self, distance):
        # 1. 计算原始beta并更新滑动窗口
        beta_current_raw = self.DirectWilling(distance)
        self.beta_window.append(beta_current_raw)

        # 2. 保持窗口大小（超出部分删除最旧数据）
        if len(self.beta_window) > self.window_size:
            self.beta_window.pop(0)

        # 3. 计算加权移动平均（近期数据权重更高，线性加权）
        window_len = len(self.beta_window)
        if window_len == 0:
            return beta_current_raw
        if window_len == 1:
            return self.beta_window[0]

        # 生成权重：第i个数据的权重为(i+1)（近期数据权重更大）
        # weights = [1 if i <= 3 else 2 for i in range(window_len)]
        weights = [i + 1 for i in range(window_len)]
        weight_sum = sum(weights)

        # 加权求和计算平滑值
        beta_smoothed = (
            sum(b * w for b, w in zip(self.beta_window, weights)) / weight_sum
        )
        self.beta_window[-1] = beta_smoothed

        return beta_smoothed
    
    def get_smoothed_alpha_window(self, distance):
        # 1. 计算原始beta并更新滑动窗口
        alpha_current_raw = self.HumanArbitrary(distance)
        self.alpha_window.append(alpha_current_raw)

        # 2. 保持窗口大小（超出部分删除最旧数据）
        if len(self.alpha_window) > self.alpha_window_size:
            self.alpha_window.pop(0)

        # 3. 计算加权移动平均（近期数据权重更高，线性加权）
        window_len = len(self.alpha_window)
        if window_len == 0:
            return alpha_current_raw
        if window_len == 1:
            return self.alpha_window[0]

        # 生成权重：第i个数据的权重为(i+1)（近期数据权重更大）
        # weights = [1 if i <= 3 else 2 for i in range(window_len)]
        weights = [i + 1 for i in range(window_len)]
        weight_sum = sum(weights)

        # 加权求和计算平滑值
        alpha_smoothed = (
            sum(b * w for b, w in zip(self.alpha_window, weights)) / weight_sum
        )
        self.alpha_window[-1] = alpha_smoothed

        return alpha_smoothed


class TrajDeform:

    def __init__(
        self,
        flag=False,
        pub1=None,
        pub2=None,
        pub3=None,
        pub4=None,
    ):
        # 参数设置
        # self.T = 0.03  # 阻抗控制采样周期 [s]
        if pub1 != None:
            self.pub1 = pub1
        if pub2 != None:
            self.pub2 = pub2
        if pub3 != None:
            self.pub3 = pub3
        if pub4 != None:
            self.pub4 = pub4
        self.callback = CallBackFunction()
        self.delta = 1 / 60  # 轨迹变形采样周期 [s]
        self.tau = 3  # 轨迹变形时长 [s]
        self.mu = 0.02  # 控制变形量的系数 [m/N·s]
        self.t_stop = 5  # 总仿真时间
        # self.total_N = int(self.t_stop / self.T)  # 总步数
        self.traj_N = int(self.t_stop / self.delta)  # 轨迹节点数
        # self.r = self.delta / self.T  # 两个采样周期的比率
        self.N = int(self.tau / self.delta + 1)  # 沿着γd和γ˜d的航路点数

        # 轨迹矩阵 A, R, B 的初始化
        self.A = (
            np.vstack((np.eye(self.N), np.zeros((3, self.N))))
            + np.vstack(
                (np.zeros((1, self.N)), -3 * np.eye(self.N), np.zeros((2, self.N)))
            )
            + np.vstack(
                (np.zeros((2, self.N)), 3 * np.eye(self.N), np.zeros((1, self.N)))
            )
            + np.vstack((np.zeros((3, self.N)), -1 * np.eye(self.N)))
        )

        self.R = self.A.T @ self.A

        self.B = np.zeros((4, self.N))
        self.B[0, 0] = 1
        self.B[1, 1] = 1
        self.B[2, self.N - 2] = 1
        self.B[3, self.N - 1] = 1

        # 计算 G 矩阵
        self.G = (
            (
                np.eye(self.N)
                - np.linalg.inv(self.R)
                @ self.B.T
                @ np.linalg.inv(self.B @ np.linalg.inv(self.R) @ self.B.T)
                @ self.B
            )
            @ np.linalg.inv(self.R)
            @ np.ones((self.N, 3))
        )

        # 初始化原始期望轨迹
        # self.x_star = traj
        # self.position_flange = np.zeros([self.traj_N, 3])
        # for i in range(self.traj_N):
        #     self.position_flange[i, :] = self.x_star[i, :] + 0.3 * (
        #         position_trocar - self.x_star[i, :]
        #     ) / (np.linalg.norm(position_trocar - self.x_star[i, :]))

        # self.x_star = self.position_flange

        self.x_star = np.zeros((self.traj_N, 3))
        self.x_star[:, 0] = np.linspace(0, 0.005 * self.t_stop, self.traj_N)
        self.x_star[:, 1] = np.linspace(0, 0.005 * self.t_stop, self.traj_N)
        self.x_star[:, 2] = np.linspace(0, 0.005 * self.t_stop, self.traj_N)
        self.x_star_init = self.x_star
        self.dot_x_star = np.zeros((self.traj_N, 3))
        for i in range(self.traj_N):
            if i == 0:
                self.dot_x_star[i, :] = [0, 0, 0]
            else:
                self.dot_x_star[i, :] = (
                    self.x_star[i, :] - self.x_star[i - 1, :]
                ) / self.delta
        self.dot_x_star_init = self.dot_x_star
        self.x_d_curr = self.x_star[0, :]  # 当前期望轨迹位置
        self.x_d_next = self.x_star[1, :]  # 下一期望轨迹位置

        # 复制轨迹
        self.x_star_01 = self.x_star.copy()
        self.x_star_05 = self.x_star.copy()
        self.x_star_10 = self.x_star.copy()
        self.x_star_now = self.x_star.copy()
        self.x_real = self.x_star.copy()

        # 系统参数：刚度和阻尼系数矩阵
        self.K_d = np.diag([100, 100, 100])  # 阻抗控制的刚度
        self.B_d = np.diag([10, 10, 10])  # 阻尼系数

        # 初始化位置和速度
        self.x = np.array([0, 0, 0])  # 初始位置
        self.x_dot = np.array([0, 0, 0])  # 初始速度
        self.f_h = np.array([0, 0, 0])  # 外力初始化

        # 预计算常数向量 H\
        self.H = np.zeros((self.N, 3))
        self.H[:, 0] = np.sqrt(self.N) * self.G[:, 0] / np.linalg.norm(self.G[:, 0])
        self.H[:, 1] = np.sqrt(self.N) * self.G[:, 1] / np.linalg.norm(self.G[:, 1])
        self.H[:, 2] = np.sqrt(self.N) * self.G[:, 2] / np.linalg.norm(self.G[:, 2])

        # 初始化序列参数
        self.h = -1
        self.k = -1

        self.f_append_01 = []
        self.f_append_05 = []
        self.f_append_10 = []
        self.f_append_now = []
        self.sub_force_array = []
        self.sub_force = []
        self.force_traj = []

        self.flag = flag

        # # 创建一个3D图形
        # if flag:
        #     self.fig1 = plt.figure()
        #     self.ax1 = self.fig1.add_subplot(111, projection="3d")

        self.time_end = time.perf_counter()
        self.time_start = time.perf_counter()
        self.weight = []

        self.pospub = Point()
        self.velpub = Point()
        self.pospub_init = Point()
        self.velpub_init = Point()
        if self.flag:
            # 初始化一个1x3的数组，表示三维坐标
            self.sub_force_array = np.array([0, 0, 0])[np.newaxis, :]
            # 初始化轨迹
            self.force_traj = np.array(self.sub_force_array)

    def deform(self):
        # if self.h == self.r * (self.k + 1):
        self.time_end = time.perf_counter()
        self.run_time = self.time_end - self.time_start
        # print(self.run_time)
        self.k = self.k + 1
        self.tau_i = self.k * self.delta

        if self.flag == False:
            # 采样不同时间点的外力
            self.f_h_01 = self.sampleForce(0.1)
            self.f_h_05 = self.sampleForce(0.5)
            self.f_h_10 = self.sampleForce(1)

            self.f_append_01.append(self.f_h_01)
            self.f_append_05.append(self.f_h_05)
            self.f_append_10.append(self.f_h_10)

            # 更新期望轨迹段
            self.gamma_d_01 = self.x_star_01[
                self.k : min(self.k + self.N, self.x_star_01.shape[0]), :
            ]
            self.gamma_d_05 = self.x_star_05[
                self.k : min(self.k + self.N, self.x_star_05.shape[0]), :
            ]
            self.gamma_d_10 = self.x_star_10[
                self.k : min(self.k + self.N, self.x_star_10.shape[0]), :
            ]

            # 更新轨迹变形
            self.gamma_d_new_01 = (
                self.gamma_d_01
                + self.mu
                * self.delta
                * self.H[0 : self.gamma_d_01.shape[0], :]
                @ self.f_h_01
            )
            self.gamma_d_new_05 = (
                self.gamma_d_05
                + self.mu
                * self.delta
                * self.H[0 : self.gamma_d_05.shape[0], :]
                @ self.f_h_05
            )
            self.gamma_d_new_10 = (
                self.gamma_d_10
                + self.mu
                * self.delta
                * self.H[0 : self.gamma_d_10.shape[0], :]
                @ self.f_h_10
            )

            if self.gamma_d_01.shape[0] > 1:
                # 更新当前和下一个期望轨迹点
                x_d_curr = self.gamma_d_new_10[0, :]
                x_d_next = self.gamma_d_new_10[1, :]
                self.dot_x_d_curr = (x_d_next - x_d_curr) / self.delta

                # 更新原始轨迹
                self.x_star_01[
                    self.k : min(self.k + self.N, self.x_star_01.shape[0]), :
                ] = self.gamma_d_new_01
                self.x_star_05[
                    self.k : min(self.k + self.N, self.x_star_05.shape[0]), :
                ] = self.gamma_d_new_05
                self.x_star_10[
                    self.k : min(self.k + self.N, self.x_star_10.shape[0]), :
                ] = self.gamma_d_new_10

        elif self.flag == True:
            self.f_h_now = self.sampleForce()
            self.f_append_now.append(self.f_h_now)

            self.x_d_curr_init = self.x_star[self.k, :]
            if self.k < self.traj_N - 1:
                self.x_d_next_init = self.x_star[self.k + 1, :]
            self.dot_x_d_curr_init = (
                self.x_d_next_init - self.x_d_curr_init
            ) / self.delta

            if np.max(self.f_h_now) == 0.0 and np.min(self.f_h_now) == 0.0:
                self.x_d_curr = self.x_star_now[self.k, :]
                if self.k < self.traj_N - 1:
                    self.x_d_next = self.x_star_now[self.k + 1, :]
                self.dot_x_d_curr = (self.x_d_next - self.x_d_curr) / self.delta

            else:
                self.gamma_d_now = self.x_star_now[
                    self.k : min(self.k + self.N, self.x_star_now.shape[0]), :
                ]
                self.gamma_d_new_now = (
                    self.gamma_d_now
                    + self.mu
                    * self.delta
                    * self.H[0 : self.gamma_d_now.shape[0], :]
                    @ self.f_h_now
                )
                if self.gamma_d_now.shape[0] >= 1:
                    # 更新当前和下一个期望轨迹点
                    self.x_d_curr = self.gamma_d_new_now[0, :]
                    if self.gamma_d_now.shape[0] > 1:
                        self.x_d_next = self.gamma_d_new_now[1, :]
                    self.dot_x_d_curr = (self.x_d_next - self.x_d_curr) / self.delta

                    # 更新原始轨迹
                    self.x_star_now[
                        self.k : min(self.k + self.N, self.x_star_now.shape[0]), :
                    ] = self.gamma_d_new_now

            self.pospub.x = self.x_d_curr[0]
            self.pospub.y = self.x_d_curr[1]
            self.pospub.z = self.x_d_curr[2]

            self.pub1.publish(self.pospub)

            self.velpub.x = self.dot_x_d_curr[0]
            self.velpub.y = self.dot_x_d_curr[1]
            self.velpub.z = self.dot_x_d_curr[2]

            self.pub2.publish(self.velpub)

            self.pospub_init.x = self.x_d_curr_init[0]
            self.pospub_init.y = self.x_d_curr_init[1]
            self.pospub_init.z = self.x_d_curr_init[2]

            self.pub3.publish(self.pospub_init)

            self.velpub_init.x = self.dot_x_d_curr_init[0]
            self.velpub_init.y = self.dot_x_d_curr_init[1]
            self.velpub_init.z = self.dot_x_d_curr_init[2]

            self.pub4.publish(self.velpub_init)

            # self.time_start = time.perf_counter()

    def ShowPlot3D(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        if self.flag == False:
            # 绘制轨迹
            f_append_01 = np.array(self.f_append_01)
            f_append_05 = np.array(self.f_append_05)
            f_append_10 = np.array(self.f_append_10)
            ax.plot(
                self.x_star_01[:, 0],
                self.x_star_01[:, 1],
                self.x_star_01[:, 2],
                c="#FF8C00",
                linewidth=3,
                label="t=0.1s",
            )
            ax.plot(
                self.x_star_05[:, 0],
                self.x_star_05[:, 1],
                self.x_star_05[:, 2],
                c="#D2691E",
                linewidth=3,
                label="t=0.5s",
            )
            ax.plot(
                self.x_star_10[:, 0],
                self.x_star_10[:, 1],
                self.x_star_10[:, 2],
                c="#8B4513",
                linewidth=3,
                label="t=1s",
            )
            ax.plot(
                self.x_star[:, 0],
                self.x_star[:, 1],
                self.x_star[:, 2],
                c="#006400",
                linewidth=3,
                linestyle=":",
                label="init",
            )

            ax.set_title("Impedance Control and Trajectory Deformation, F=1N")
            ax.legend()
            plt.grid(True)
            plt.show()

        elif self.flag == True:
            ax.set_xlim(
                [
                    np.min(self.x_star_now[:, 0]) - 0.05,
                    np.max(self.x_star_now[:, 0]) + 0.05,
                ]
            )
            ax.set_ylim(
                [
                    np.min(self.x_star_now[:, 1]) - 0.05,
                    np.max(self.x_star_now[:, 1]) + 0.05,
                ]
            )
            ax.set_zlim(
                [
                    np.min(self.x_star_now[:, 2]) - 0.05,
                    np.max(self.x_star_now[:, 2]) + 0.05,
                ]
            )
            ax.plot(np.linspace(0, 0.1, 3), np.zeros(3), np.zeros(3), c="black")
            ax.plot(np.zeros(3), np.linspace(0, 0.1, 3), np.zeros(3), c="black")
            ax.plot(np.zeros(3), np.zeros(3), np.linspace(0, 0.1, 3), c="black")
            ax.plot(
                self.x_star_now[:, 0],
                self.x_star_now[:, 1],
                self.x_star_now[:, 2],
                c="#8B4513",
                linewidth=3,
                label="traj",
            )
            ax.plot(
                self.x_star[:, 0],
                self.x_star[:, 1],
                self.x_star[:, 2],
                c="#006400",
                linewidth=3,
                linestyle=":",
                label="init traj",
            )
            ax.plot(
                self.x_real[:, 0],
                self.x_real[:, 1],
                self.x_real[:, 2],
                c="b",
                linewidth=3,
                label="traj_real",
            )

            ax.set_title("Impedance Control and Trajectory Deformation, F=1N")
            ax.legend()
            plt.grid(True)
            plt.show()

    def sampleForce(self, len=1):
        if self.flag == False:
            if self.tau_i >= 1 and self.tau_i <= 1 + len:
                self.f_h = np.diag([1, 0, 0])
            else:
                self.f_h = np.diag([0, 0, 0])
        elif self.flag == True:
            self.sub_force_array = self.callback.force_falcon
            self.force_traj = np.append(self.force_traj, self.sub_force_array)
            self.force_traj_array = np.reshape(self.force_traj, (-1, 3))
            self.f_h = np.diag(self.sub_force_array)

            # # 清除当前绘图
            # self.ax1.cla()

            # # 绘制轨迹
            # self.ax1.plot(
            #     self.force_traj_array[:, 0],
            #     self.force_traj_array[:, 1],
            #     self.force_traj_array[:, 2],
            #     color="b",
            #     marker="o",
            # )
            # # 刷新图形
            # plt.draw()
            # plt.pause(0.00000001)  # 设置暂停时间

        return self.f_h

    def WeightEstimation(self):

        return self.weight

    # 定义点到球体的距离
    def distance_to_sphere(point, center, radius):
        """
        计算点到球体表面的距离
        :param point: np.array, 目标点 (x, y, z)
        :param center: np.array, 球心 (cx, cy, cz)
        :param radius: float, 球体的半径
        :return: float, 点到球体的距离
        """
        return max(0, np.linalg.norm(point - center) - radius)

    # 定义点到立方体的距离
    def distance_to_box(point, box_min, box_max):
        """
        计算点到轴对齐立方体表面的距离
        :param point: np.array, 目标点 (x, y, z)
        :param box_min: np.array, 立方体的最小角点 (xmin, ymin, zmin)
        :param box_max: np.array, 立方体的最大角点 (xmax, ymax, zmax)
        :return: float, 点到立方体的距离
        """
        # 计算点到立方体范围的偏移量
        dx = max(box_min[0] - point[0], 0, point[0] - box_max[0])
        dy = max(box_min[1] - point[1], 0, point[1] - box_max[1])
        dz = max(box_min[2] - point[2], 0, point[2] - box_max[2])

        return np.sqrt(dx**2 + dy**2 + dz**2)


class TrajDeformMoveit:

    def __init__(
        self,
        freq,
        flag=False,
        traj=None,
        position_trocar=None,
        length=None,
        movegroup=None,
        pub1=None,
        pub2=None,
        pub3=None,
        pub4=None,
    ):
        # 参数设置
        # self.T = 0.03  # 阻抗控制采样周期 [s]
        if pub1 != None:
            self.pub1 = pub1
        if pub2 != None:
            self.pub2 = pub2
        if pub3 != None:
            self.pub3 = pub3
        if pub4 != None:
            self.pub4 = pub4
        self.callback = CallBackFunction()
        self.movegroup = movegroup

        self.tau = 3  # 轨迹变形时长 [s]
        self.mu = 0.01  # 控制变形量的系数 [m/N·s]
        # self.t_stop = 15  # 总仿真时间
        # self.total_N = int(self.t_stop / self.T)  # 总步数
        self.traj_N = np.shape(traj)[0]  # 轨迹节点数
        # self.delta = self.t_stop / self.traj_N  # 轨迹变形采样周期 [s]
        self.delta = 1 / freq
        self.t_stop = self.delta * self.traj_N
        # self.r = self.delta / self.T  # 两个采样周期的比率
        self.N = int(self.tau / self.delta + 1)  # 沿着γd和γ˜d的航路点数

        # 轨迹矩阵 A, R, B 的初始化
        self.A = (
            np.vstack((np.eye(self.N), np.zeros((3, self.N))))
            + np.vstack(
                (np.zeros((1, self.N)), -3 * np.eye(self.N), np.zeros((2, self.N)))
            )
            + np.vstack(
                (np.zeros((2, self.N)), 3 * np.eye(self.N), np.zeros((1, self.N)))
            )
            + np.vstack((np.zeros((3, self.N)), -1 * np.eye(self.N)))
        )

        self.R = self.A.T @ self.A

        self.B = np.zeros((4, self.N))
        self.B[0, 0] = 1
        self.B[1, 1] = 1
        self.B[2, self.N - 2] = 1
        self.B[3, self.N - 1] = 1

        # 计算 G 矩阵
        self.G = (
            (
                np.eye(self.N)
                - np.linalg.inv(self.R)
                @ self.B.T
                @ np.linalg.inv(self.B @ np.linalg.inv(self.R) @ self.B.T)
                @ self.B
            )
            @ np.linalg.inv(self.R)
            @ np.ones((self.N, 3))
        )

        # 初始化原始期望轨迹
        self.x_star = traj

        self.x_star_init = self.x_star
        self.dot_x_star = np.zeros((self.traj_N, 3))
        for i in range(self.traj_N):
            if i == 0:
                self.dot_x_star[i, :] = [0, 0, 0]
            else:
                self.dot_x_star[i, :] = (
                    self.x_star[i, :] - self.x_star[i - 1, :]
                ) / self.delta
        self.dot_x_star_init = self.dot_x_star
        self.x_d_curr = self.x_star[0, :]  # 当前期望轨迹位置
        self.x_d_next = self.x_star[1, :]  # 下一期望轨迹位置

        # 复制轨迹
        self.x_star_01 = self.x_star.copy()
        self.x_star_05 = self.x_star.copy()
        self.x_star_10 = self.x_star.copy()
        self.x_star_now = self.x_star.copy()
        self.x_real = self.x_star.copy()

        # 系统参数：刚度和阻尼系数矩阵
        self.K_d = np.diag([100, 100, 100])  # 阻抗控制的刚度
        self.B_d = np.diag([10, 10, 10])  # 阻尼系数

        # 初始化位置和速度
        self.x = np.array([0, 0, 0])  # 初始位置
        self.x_dot = np.array([0, 0, 0])  # 初始速度
        self.f_h = np.array([0, 0, 0])  # 外力初始化
        self.f_h_old = np.diag([0, 0, 0])

        # 预计算常数向量 H\
        self.H = np.zeros((self.N, 3))
        self.H[:, 0] = np.sqrt(self.N) * self.G[:, 0] / np.linalg.norm(self.G[:, 0])
        self.H[:, 1] = np.sqrt(self.N) * self.G[:, 1] / np.linalg.norm(self.G[:, 1])
        self.H[:, 2] = np.sqrt(self.N) * self.G[:, 2] / np.linalg.norm(self.G[:, 2])

        # 初始化序列参数
        self.h = -1
        self.k = -1

        self.f_append_01 = []
        self.f_append_05 = []
        self.f_append_10 = []
        self.f_append_now = []
        self.sub_force_array = []
        self.sub_force = []
        self.force_traj = []
        self.x_star_append = []
        self.x_star_now_append = []
        self.x_star_append.append(self.x_star)
        self.x_star_now_append.append(self.x_star_now)
        self.goal = np.array([0, 0, 0])
        self.old_goal = self.goal

        self.flag = flag

        # # 创建一个3D图形
        # if flag:
        #     self.fig1 = plt.figure()
        #     self.ax1 = self.fig1.add_subplot(111, projection="3d")

        self.time_end = time.perf_counter()
        self.time_start = time.perf_counter()
        self.weight = []

        self.pospub = Point()
        self.velpub = Point()
        self.pospub_init = Point()
        self.velpub_init = Point()
        if self.flag:
            # 初始化一个1x3的数组，表示三维坐标
            self.sub_force_array = np.array([0, 0, 0])[np.newaxis, :]
            # 初始化轨迹
            self.force_traj = np.array(self.sub_force_array)

    def deform(self, controller=None):
        # if self.h == self.r * (self.k + 1):
        self.time_end = time.perf_counter()
        self.run_time = self.time_end - self.time_start
        # print(self.run_time)
        self.k = self.k + 1
        self.tau_i = self.k * self.delta

        if self.flag == False:
            # 采样不同时间点的外力
            self.f_h_01 = self.sampleForce(0.1)
            self.f_h_05 = self.sampleForce(0.5)
            self.f_h_10 = self.sampleForce(1)

            self.f_append_01.append(self.f_h_01)
            self.f_append_05.append(self.f_h_05)
            self.f_append_10.append(self.f_h_10)

            # 更新期望轨迹段
            self.gamma_d_01 = self.x_star_01[
                self.k : min(self.k + self.N, self.x_star_01.shape[0]), :
            ]
            self.gamma_d_05 = self.x_star_05[
                self.k : min(self.k + self.N, self.x_star_05.shape[0]), :
            ]
            self.gamma_d_10 = self.x_star_10[
                self.k : min(self.k + self.N, self.x_star_10.shape[0]), :
            ]

            # 更新轨迹变形
            self.gamma_d_new_01 = (
                self.gamma_d_01
                + self.mu
                * self.delta
                * self.H[0 : self.gamma_d_01.shape[0], :]
                @ self.f_h_01
            )
            self.gamma_d_new_05 = (
                self.gamma_d_05
                + self.mu
                * self.delta
                * self.H[0 : self.gamma_d_05.shape[0], :]
                @ self.f_h_05
            )
            self.gamma_d_new_10 = (
                self.gamma_d_10
                + self.mu
                * self.delta
                * self.H[0 : self.gamma_d_10.shape[0], :]
                @ self.f_h_10
            )

            if self.gamma_d_01.shape[0] > 1:
                # 更新当前和下一个期望轨迹点
                x_d_curr = self.gamma_d_new_10[0, :]
                x_d_next = self.gamma_d_new_10[1, :]
                self.dot_x_d_curr = (x_d_next - x_d_curr) / self.delta

                # 更新原始轨迹
                self.x_star_01[
                    self.k : min(self.k + self.N, self.x_star_01.shape[0]), :
                ] = self.gamma_d_new_01
                self.x_star_05[
                    self.k : min(self.k + self.N, self.x_star_05.shape[0]), :
                ] = self.gamma_d_new_05
                self.x_star_10[
                    self.k : min(self.k + self.N, self.x_star_10.shape[0]), :
                ] = self.gamma_d_new_10

        elif self.flag == True:
            self.f_h_now = self.sampleForce()
            self.f_append_now.append(self.f_h_now)

            self.x_d_curr_init = self.x_star[self.k, :]
            if self.k < self.traj_N - 1:
                self.x_d_next_init = self.x_star[self.k + 1, :]
            self.dot_x_d_curr_init = (
                self.x_d_next_init - self.x_d_curr_init
            ) / self.delta

            if np.max(self.f_h_now) == 0.0 and np.min(self.f_h_now) == 0.0:
                self.x_d_curr = self.x_star_now[self.k, :]
                if self.k < self.traj_N - 1:
                    self.x_d_next = self.x_star_now[self.k + 1, :]
                self.dot_x_d_curr = (self.x_d_next - self.x_d_curr) / self.delta
                # ###########意图识别
                # if np.max(self.f_h_old) != 0.0 or np.min(self.f_h_old) != 0.0:
                #     if not np.array_equal(self.goal, self.old_goal):
                #         # self.goal = np.array([0.3, 0.05, 0.2])
                #         wpose = self.movegroup.move_group.get_current_pose().pose
                #         way_plan, quat_plan, joint_plan = self.movegroup.plan_path(
                #             self.goal
                #         )
                #         print(way_plan)
                #         self.t_stop = self.t_stop - self.run_time
                #         self.traj_N = np.shape(way_plan)[0]  # 轨迹节点数
                #         self.x_star = way_plan
                #         self.x_star_now = way_plan
                #         self.x_star_append.append(self.x_star)
                #         self.x_star_now_append.append(self.x_star_now)
                #         self.h = -1
                #         self.k = -1
                #         self.old_goal = self.goal

            else:
                self.gamma_d_now = self.x_star_now[
                    self.k : min(self.k + self.N, self.x_star_now.shape[0]), :
                ]
                self.gamma_d_new_now = (
                    self.gamma_d_now
                    + self.mu
                    * self.delta
                    * self.H[0 : self.gamma_d_now.shape[0], :]
                    @ self.f_h_now
                )
                if self.gamma_d_now.shape[0] >= 1:
                    # 更新当前和下一个期望轨迹点
                    self.x_d_curr = self.gamma_d_new_now[0, :]
                    if self.gamma_d_now.shape[0] > 1:
                        self.x_d_next = self.gamma_d_new_now[1, :]
                    self.dot_x_d_curr = (self.x_d_next - self.x_d_curr) / self.delta

                    # 更新原始轨迹
                    self.x_star_now[
                        self.k : min(self.k + self.N, self.x_star_now.shape[0]), :
                    ] = self.gamma_d_new_now

            if controller != None:
                controller.reference_trajectory = self.x_star_now

            self.pospub.x = self.x_d_curr[0]
            self.pospub.y = self.x_d_curr[1]
            self.pospub.z = self.x_d_curr[2]

            self.pub1.publish(self.pospub)

            self.velpub.x = self.dot_x_d_curr[0]
            self.velpub.y = self.dot_x_d_curr[1]
            self.velpub.z = self.dot_x_d_curr[2]

            self.pub2.publish(self.velpub)

            self.pospub_init.x = self.x_d_curr_init[0]
            self.pospub_init.y = self.x_d_curr_init[1]
            self.pospub_init.z = self.x_d_curr_init[2]

            self.pub3.publish(self.pospub_init)

            self.velpub_init.x = self.dot_x_d_curr_init[0]
            self.velpub_init.y = self.dot_x_d_curr_init[1]
            self.velpub_init.z = self.dot_x_d_curr_init[2]

            self.pub4.publish(self.velpub_init)
            self.f_h_old = self.f_h_now

            # self.time_start = time.perf_counter()

    def ShowPlot3D(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        if self.flag == False:
            # 绘制轨迹
            f_append_01 = np.array(self.f_append_01)
            f_append_05 = np.array(self.f_append_05)
            f_append_10 = np.array(self.f_append_10)
            ax.plot(
                self.x_star_01[:, 0],
                self.x_star_01[:, 1],
                self.x_star_01[:, 2],
                c="#FF8C00",
                linewidth=3,
                label="t=0.1s",
            )
            ax.plot(
                self.x_star_05[:, 0],
                self.x_star_05[:, 1],
                self.x_star_05[:, 2],
                c="#D2691E",
                linewidth=3,
                label="t=0.5s",
            )
            ax.plot(
                self.x_star_10[:, 0],
                self.x_star_10[:, 1],
                self.x_star_10[:, 2],
                c="#8B4513",
                linewidth=3,
                label="t=1s",
            )
            ax.plot(
                self.x_star[:, 0],
                self.x_star[:, 1],
                self.x_star[:, 2],
                c="#006400",
                linewidth=3,
                linestyle=":",
                label="init",
            )

            ax.set_title("Impedance Control and Trajectory Deformation, F=1N")
            ax.legend()
            plt.grid(True)
            plt.show()

        elif self.flag == True:
            ax.set_xlim(
                [
                    np.min(self.x_star_now[:, 0]) - 0.05,
                    np.max(self.x_star_now[:, 0]) + 0.05,
                ]
            )
            ax.set_ylim(
                [
                    np.min(self.x_star_now[:, 1]) - 0.05,
                    np.max(self.x_star_now[:, 1]) + 0.05,
                ]
            )
            ax.set_zlim(
                [
                    np.min(self.x_star_now[:, 2]) - 0.05,
                    np.max(self.x_star_now[:, 2]) + 0.05,
                ]
            )
            ax.plot(np.linspace(0, 0.1, 3), np.zeros(3), np.zeros(3), c="black")
            ax.plot(np.zeros(3), np.linspace(0, 0.1, 3), np.zeros(3), c="black")
            ax.plot(np.zeros(3), np.zeros(3), np.linspace(0, 0.1, 3), c="black")
            ax.plot(
                self.x_star_now[:, 0],
                self.x_star_now[:, 1],
                self.x_star_now[:, 2],
                c="#8B4513",
                linewidth=3,
                label="traj",
            )
            ax.plot(
                self.x_star[:, 0],
                self.x_star[:, 1],
                self.x_star[:, 2],
                c="#006400",
                linewidth=3,
                linestyle=":",
                label="init traj",
            )
            ax.plot(
                self.x_real[:, 0],
                self.x_real[:, 1],
                self.x_real[:, 2],
                c="b",
                linewidth=3,
                label="traj_real",
            )

            ax.set_title("Impedance Control and Trajectory Deformation, F=1N")
            ax.legend()
            plt.grid(True)
            plt.show()

    def sampleForce(self, len=1):
        if self.flag == False:
            if self.tau_i >= 1 and self.tau_i <= 1 + len:
                self.f_h = np.diag([1, 0, 0])
            else:
                self.f_h = np.diag([0, 0, 0])
        elif self.flag == True:
            self.sub_force_array = self.callback.force_falcon
            self.force_traj = np.append(self.force_traj, self.sub_force_array)
            self.force_traj_array = np.reshape(self.force_traj, (-1, 3))
            self.f_h = np.diag(self.sub_force_array)

            # # 清除当前绘图
            # self.ax1.cla()

            # # 绘制轨迹
            # self.ax1.plot(
            #     self.force_traj_array[:, 0],
            #     self.force_traj_array[:, 1],
            #     self.force_traj_array[:, 2],
            #     color="b",
            #     marker="o",
            # )
            # # 刷新图形
            # plt.draw()
            # plt.pause(0.00000001)  # 设置暂停时间

        return self.f_h

    def WeightEstimation(self):

        return self.weight

    # 定义点到球体的距离
    def distance_to_sphere(point, center, radius):
        """
        计算点到球体表面的距离
        :param point: np.array, 目标点 (x, y, z)
        :param center: np.array, 球心 (cx, cy, cz)
        :param radius: float, 球体的半径
        :return: float, 点到球体的距离
        """
        return max(0, np.linalg.norm(point - center) - radius)

    # 定义点到立方体的距离
    def distance_to_box(point, box_min, box_max):
        """
        计算点到轴对齐立方体表面的距离
        :param point: np.array, 目标点 (x, y, z)
        :param box_min: np.array, 立方体的最小角点 (xmin, ymin, zmin)
        :param box_max: np.array, 立方体的最大角点 (xmax, ymax, zmax)
        :return: float, 点到立方体的距离
        """
        # 计算点到立方体范围的偏移量
        dx = max(box_min[0] - point[0], 0, point[0] - box_max[0])
        dy = max(box_min[1] - point[1], 0, point[1] - box_max[1])
        dz = max(box_min[2] - point[2], 0, point[2] - box_max[2])

        return np.sqrt(dx**2 + dy**2 + dz**2)


class FuzzyFunction:

    def __init__(self, PandaArm, PandaKinematics):
        self.PandaArm = PandaArm
        self.kinematics_tool = PandaKinematics
        self.mu = 0
        self.obj = 0
        self.goal = 0
        self.mu_fuzzy_low = 0
        self.mu_fuzzy_fine = 0
        self.obj_fuzzy_low = 0
        self.obj_fuzzy_fine = 0
        self.goal_fuzzy_close = 0
        self.goal_fuzzy_far = 0
        self.lambda_low = 0
        self.lambda_share = 0
        self.lambda_high = 0
        neu_j = self.PandaArm._neutral_pose_joints
        self.neu_pose = np.array(
            self.kinematics_tool.forward_position_kinematics(joint_values=neu_j)
        )[0:3].flatten()

    def UpdateFuzzy(self, mu, distance_obj, distance_goal):
        self.mu = mu
        self.obj = distance_obj
        self.goal = distance_goal

        if self.mu <= 0.03:
            self.mu_fuzzy_low = -(1 / 0.03) * (self.mu - 0.03)
            self.mu_fuzzy_fine = (1 / 0.03) * self.mu
        else:
            self.mu_fuzzy_low = 0
            self.mu_fuzzy_fine = 1

        if self.obj <= 0.03:
            self.obj_fuzzy_low = 1
            self.obj_fuzzy_fine = 0
        elif self.obj >= 0.15:
            self.obj_fuzzy_low = 0
            self.obj_fuzzy_fine = 1
        else:
            self.obj_fuzzy_low = -(1 / 0.12) * (self.obj - 0.15)
            self.obj_fuzzy_fine = (1 / 0.12) * (self.obj - 0.03)

        if self.goal <= 0.02:
            self.goal_fuzzy_close = -(1 / 0.2) * (self.goal - 0.2)
            self.goal_fuzzy_far = (1 / 0.2) * self.goal
        else:
            self.goal_fuzzy_close = 0
            self.goal_fuzzy_far = 1

        self.lambda_low = np.fmin(
            np.fmax(
                np.fmax(self.mu_fuzzy_low, self.mu_fuzzy_fine),
                np.fmax(self.goal_fuzzy_close, self.goal_fuzzy_far),
            ),
            self.obj_fuzzy_low,
        )
        self.lambda_share = np.fmin(
            np.fmax(
                np.fmin(
                    self.mu_fuzzy_low,
                    np.fmax(self.goal_fuzzy_close, self.goal_fuzzy_far),
                ),
                np.fmin(
                    self.goal_fuzzy_close,
                    np.fmax(self.mu_fuzzy_fine, self.mu_fuzzy_low),
                ),
            ),
            self.obj_fuzzy_fine,
        )
        self.lambda_high = np.fmin(
            np.fmin(self.mu_fuzzy_fine, self.obj_fuzzy_fine), self.goal_fuzzy_far
        )

        delta = 100

        self.lambda_low_excitation = np.zeros(delta)
        self.lambda_share_excitation = np.zeros(delta)
        self.lambda_high_excitation = np.zeros(delta)
        self.lambda_excitation = np.zeros(delta)
        self.lambda_excitation_denominator = np.zeros(delta)
        self.lambda_excitation_molecule = np.zeros(delta)

        for i in range(delta):
            penalty_lambda = i / delta
            if i < 10:
                self.lambda_low_excitation[i] = np.fmin(1, self.lambda_low)

            elif i < 50:
                original_low = -(1 / 0.4) * (penalty_lambda - 0.5)
                original_share = (1 / 0.4) * (penalty_lambda - 0.1)
                self.lambda_low_excitation[i] = np.fmin(original_low, self.lambda_low)
                self.lambda_share_excitation[i] = np.fmin(
                    original_share, self.lambda_share
                )
            elif i < 90:
                original_share = -(1 / 0.4) * (penalty_lambda - 0.9)
                original_high = (1 / 0.4) * (penalty_lambda - 0.5)
                self.lambda_share_excitation[i] = np.fmin(
                    original_share, self.lambda_share
                )
                self.lambda_high_excitation[i] = np.fmin(
                    original_high, self.lambda_high
                )
            else:
                self.lambda_high_excitation[i] = np.fmin(1, self.lambda_high)
            self.lambda_excitation[i] = np.fmax(
                np.fmax(self.lambda_low_excitation[i], self.lambda_share_excitation[i]),
                self.lambda_high_excitation[i],
            )

            self.lambda_excitation_molecule[i] = (
                self.lambda_excitation[i] * penalty_lambda * (1 / delta)
            )
            self.lambda_excitation_denominator[i] = self.lambda_excitation[i] * (
                1 / delta
            )
        final_lambda_molecule = np.sum(self.lambda_excitation_molecule)
        final_lambda_denominator = np.sum(self.lambda_excitation_denominator)
        final_lambda = final_lambda_molecule / final_lambda_denominator
        return final_lambda

    def Calculate_mu(self):

        joint_limits = self.PandaArm.joint_limits()
        joint_limit_lower = np.array([joint["lower"] for joint in joint_limits])
        joint_limit_upper = np.array([joint["upper"] for joint in joint_limits])
        joint_now = self.PandaArm.angles()
        penalty_P = np.zeros(7)
        penalty_k = 10
        for i in range(7):
            penalty_P[i] = 1 - np.exp(
                -penalty_k
                * (
                    joint_now[i]
                    - joint_limit_lower[i] * (joint_limit_upper[i] - joint_now[i])
                )
                / np.square(joint_limit_upper[i] - joint_limit_lower[i])
            )
            # print(penalty_P[i])
        Jacobian = self.PandaArm.zero_jacobian()
        manipulability = np.sqrt(np.linalg.det(Jacobian @ Jacobian.T))
        temp = penalty_P * manipulability
        penalty_mu = np.min(temp)
        return penalty_mu

    def Calculate_delta(self, penalty, penalty_old, delta_time):
        delta_penalty = (penalty - penalty_old) / delta_time
        return delta_penalty

    def Calculate_Distance_Cylinder(self, pose, obj_center, obj_r):
        obj_distance = np.linalg.norm(pose[0:2] - obj_center[0:2]) - obj_r
        obj_distance_positive = np.fmax(0, obj_distance)
        return obj_distance_positive

    def Calculate_Distance_Plant(self, pose, obj_h):
        obj_distance = pose[2] - obj_h
        obj_distance_positive = np.fmax(0, obj_distance)
        return obj_distance_positive

    def Calculate_Distance_Sphere(self, pose, obj_center, obj_r):
        obj_distance = np.linalg.norm(pose - obj_center) - obj_r
        obj_distance_positive = np.fmax(0, obj_distance)
        return obj_distance_positive

    def Calculate_Distance_WorkSpace(self, pose, neu_pose, workspace_r):
        workspace_distance = workspace_r - np.linalg.norm(pose - neu_pose)
        workspace_distance_positive = np.fmax(0, workspace_distance)
        return workspace_distance_positive

    def Calculate_Distance_Goal(self, pose, goal=np.array([0.3, 0.05, 0.2])):
        goal_distance = np.linalg.norm(pose - goal)
        return goal_distance

    def update_alpha(self, tool_position):
        obj1_center = np.array([0.3, 0, 0.4])
        obj1_r = 0.1
        workspace_r = 0.2

        pose = tool_position
        penalty_mu = self.Calculate_mu()

        penalty_distance_workspace = self.Calculate_Distance_WorkSpace(
            pose, self.neu_pose, workspace_r
        )
        penalty_distance_sphere = self.Calculate_Distance_Sphere(
            pose, obj1_center, obj1_r
        )
        penalty_distance_min = np.fmin(
            penalty_distance_sphere,
            penalty_distance_workspace,
        )
        penalty_distance_goal = self.Calculate_Distance_Goal(pose)
        fuzzy_lambda = self.UpdateFuzzy(
            penalty_mu, penalty_distance_min, penalty_distance_goal
        )
        # print(fuzzy_lambda)
        return fuzzy_lambda

    def update_beta(self, tool_position):
        return tool_position


if __name__ == "__main__":
    rospy.init_node("ctrl_test")
    r = PandaArm()
    freq = 1000

    rate = rospy.Rate(freq)

    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)
    Jacobian = r.zero_jacobian()

    r.move_to_neutral()  # move to neutral pose before beginning

    initial_pose = deepcopy(r.angles())
    ee_point, ee_ori = r.ee_pose()
    ee_euler = quaternion_to_euler(quaternion.as_float_array(ee_ori))

    callback = CallBackFunction()

    position_trocar = np.array([0.3, 0.1, 0.3])
    length = 0.107 + 0.5
    error_rcm = 0
    error_rcm_old = 0
    error_rcm_dot = 0

    neu_j = [r._neutral_pose_joints[j] for j in r._joint_names]
    neu_pos, neu_rot = r.forward_kinematics(neu_j, "eul")
    ee_point, ee_ori = r.ee_pose()

    while not rospy.is_shutdown():
        ee_point, ee_ori = r.ee_pose()
        ee_vel, ee_omg = r.ee_velocity()
        ee_euler = quaternion_to_euler(quaternion.as_float_array(ee_ori))
        # print(ee_point)
        position_joint7 = callback.position_joint7
        position_endeffector = callback.position_endeffector
        jacobian_joint7 = callback.jacobian_joint7

        position_joint7_d = position_endeffector + length * (
            position_trocar - position_endeffector
        ) / (np.linalg.norm(position_trocar - position_endeffector))

        error_rcm = position_joint7 - position_joint7_d
        error_rcm_dot = (error_rcm - error_rcm_old) / (1 / freq)
        K_rcm = 5
        D_rcm = 0.1
        force_joint7_ = -K_rcm * error_rcm - D_rcm * error_rcm_dot
        force_joint7 = np.pad(force_joint7_, (0, 3), "constant")

        m_rcm = np.diag([1, 1, 1])
        d_rcm = np.diag([5, 5, 5])
        k_rcm = np.diag([10, 10, 10])
        sys_A_rcm = np.vstack(
            (
                np.hstack((np.zeros((3, 3)), np.identity(3))),
                np.hstack(
                    (-np.linalg.inv(m_rcm) * k_rcm, -np.linalg.inv(m_rcm) * d_rcm)
                ),
            )
        )
        sys_B_rcm = np.vstack((np.zeros((3, 3)), np.linalg.inv(m_rcm)))
        sys_C_rcm = np.array([[1, 1, 1, 0, 0, 0]])
        sys_D_rcm = np.array([[0, 0, 0]])
        Q_rcm = np.diag([10, 10, 10, 0.0001, 0.0001, 0.0001])
        R_rcm = np.diag([0.0005, 0.0005, 0.0005])
        sysStateSpace_rcm = ct.ss(sys_A_rcm, sys_B_rcm, sys_C_rcm, sys_D_rcm)
        K_rcm, solve_P_rcm, E = ct.lqr(sysStateSpace_rcm, Q_rcm, R_rcm)
        delta_z_rcm = np.vstack(
            (error_rcm.reshape([3, 1]), error_rcm_dot.reshape([3, 1]))
        )
        delta_z_rcm = delta_z_rcm.flatten()

        force_joint7_LQR_ = np.dot(K_rcm, delta_z_rcm)
        force_joint7_LQR = np.pad(force_joint7_LQR_, (0, 3), "constant")

        # error_ee = ee_point - neu_pos.flatten()
        temp = neu_pos.flatten()
        error_ee = ee_point - (callback.position_falcon + temp)
        error_ee_dot = ee_vel - callback.vel_falcon
        K_ee = 10
        D_ee = 10

        force_ee_ = -K_ee * error_ee - D_rcm * error_ee_dot
        force_ee = np.pad(force_ee_, (0, 3), "constant")

        Jacobian = r.zero_jacobian()
        Jacobian_trans = np.transpose(Jacobian)
        Jacobian_pinv = np.dot(
            np.linalg.inv(np.dot(Jacobian, Jacobian_trans)), Jacobian
        )

        Identity = np.ones((7, 7))

        NullSpace = Identity - np.dot(np.transpose(Jacobian), Jacobian)
        tau_force_joint7 = np.dot(np.transpose(jacobian_joint7), force_joint7)
        # tau_force_joint7 = np.dot(np.transpose(jacobian_joint7), force_joint7_LQR)
        tau_force_ee = np.dot(np.transpose(Jacobian), force_ee)
        tau_rcm = np.dot(NullSpace, tau_force_joint7)
        r.exec_torque_cmd(tau_force_ee + tau_rcm)
        # r.exec_torque_cmd(tau_force_ee)

        error_rcm_old = error_rcm
        rate.sleep()
