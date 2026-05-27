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
from franka_core_msgs.msg import RobotState, EndPointState
import sys
import tty
import termios
import select
from arbitrary_new import CallBackFunction
from arbitrary_new import TrajDeform
from arbitrary_new import TrajDeformMoveit
from arbitrary_new import GoalIntent
from arbitrary_new import MoveGroupPythonInterfaceTutorial
from scipy.spatial.distance import cosine
from panda_robot import PandaKinematics
from arbitrary_new import FuzzyFunction

# 0829新加的
from Role_law import Role_law
from pathlib import Path
from scipy import linalg

# 0911
import pandas as pd
# 定义路径
base_path = Path('/home/hjh/panda_ws/Experiment_LM')

def euler_to_quaternion(euler_angles):
    # 将欧拉角转换为四元数
    rotation = Rotation.from_euler("xyz", euler_angles, degrees=False)
    quaternion = rotation.as_quat()
    return quaternion

def quaternion_to_euler(quaternion_angles):
    # 将四元数转换为欧拉角
    rotation = Rotation.from_quat(quaternion_angles)
    euler = rotation.as_euler("xyz", degrees=False)

    return euler


def quatdiff_in_euler(quat_curr, quat_des):
    """
    Compute difference between quaternions and return
    Euler angles as difference
    """

    # 检查并转换参数类型
    if not isinstance(quat_curr, quaternion.quaternion):
        quat_curr = quaternion.from_float_array(np.array(quat_curr, dtype=float))
    if not isinstance(quat_des, quaternion.quaternion):
        quat_des = quaternion.from_float_array(np.array(quat_des, dtype=float))



    curr_mat = quaternion.as_rotation_matrix(quat_curr)
    des_mat = quaternion.as_rotation_matrix(quat_des)
    rel_mat = des_mat.T.dot(curr_mat)
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec

    return -des_mat.dot(vec)

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


def GenerateTrajectory_h(numPoint, time_total):
    result = []
    point = np.zeros(12)
    point_old = np.array([0, 0, 0])
    dt = time_total / numPoint
    pi = 3.14159265358979
    omega = 30 * pi
    for i in range(numPoint):
        t = i * dt
        # point[0] = 0.04 * t
        # point[1] = 0.01 * np.sin(omega * point[0])  # 0.1 * np.sin(omega * point[0])
        # point[2] = 0
        point[0] = 0.001 * t
        point[1] = 0.001 * t
        point[2] = 0.001 * t

        point[3] = np.pi
        point[4] = 0
        point[5] = 0
        if i != numPoint - 1 and i != 0:
            point[6:9] = (point[0:3] - point_old) / dt

        point[9] = 0
        point[10] = 0
        point[11] = 0
        point_old = point[0:3]
        result.extend(point)
    result = np.array(result)
    result = np.reshape(result, (numPoint, 12))
    return result


def GenerateTrajectory_r(numPoint, time_total):
    result = []
    point = np.zeros(12)
    point_old = point_old = np.array([0, 0, 0])
    dt = time_total / numPoint
    pi = 3.14159265358979
    omega = 30 * pi
    for i in range(numPoint):
        t = i * dt
        # point[0] = 0.04 * t
        # point[1] = 0.01 * np.sin(omega * point[0])
        # point[2] = 0
        point[0] = 0.002 * t
        point[1] = 0.002 * t
        point[2] = 0.002 * t

        # if pi / (0.02 * omega) <= t and t <= 2.5 * pi / (0.01 * omega):
        #     point[1] = 0.2
        # else:
        #     point[1] = 0.2 * np.sin(omega * point[0])

        point[3] = np.pi
        point[4] = 0
        point[5] = 0

        point[6:9] = (point[0:3] - point_old) / dt
        point[9] = 0
        point[10] = 0
        point[11] = 0
        point_old = point[0:3]
        result.extend(point)
    result = np.array(result)
    result = np.reshape(result, (numPoint, 12))
    return result

def discretized_update(z_current, A, B, u_r, u_h, dt, method='euler'):
    """计算下一时刻的状态

    参数:
        z_current: 当前状态向量
        A, B: 系统矩阵
        u_r, u_h: 输入值
        dt: 时间步长
        method: 积分方法 ('euler', 'rk4'等)

    返回:
        z_next: 下一时刻的状态向量
    """
    if method == 'euler':
        # 欧拉法 (最简单但精度较低)
        return z_current + dt * (A @ z_current + B @ (u_r + u_h))

    elif method == 'rk4':
        # 四阶龙格-库塔法 (更高精度)
        k1 = A @ z_current + B @ (u_r + u_h)
        k2 = A @ (z_current + dt*k1/2) + B @ (u_r + u_h)
        k3 = A @ (z_current + dt*k2/2) + B @ (u_r + u_h)
        k4 = A @ (z_current + dt*k3) + B @ (u_r + u_h)

        return z_current + dt * (k1 + 2*k2 + 2*k3 + k4) / 6

    # else :
    #     acl = u_r/10
    #     z_next_vel = z_current[3:6] + acl*dt
    #     z_next_pos = z_current[0:3] + z_next_vel*dt

    #     return np.hstack((z_next_pos,z_next_vel))

    else:
        raise ValueError(f"不支持的积分方法: {method}")


def pos2rotation_rcm(tool_position,trocar_position,flange_position,output = 'euler'):
    flange_z_dir_ref = (tool_position - trocar_position) / np.linalg.norm(
    tool_position - trocar_position
    )
    # print("tool_2:",tool_position_ref)
    flange_y_dir_ref = np.cross(flange_z_dir_ref, flange_x_dir)
    flange_x_dir_ref = np.cross(flange_y_dir_ref, flange_z_dir_ref)#法兰期望法向量
    # print(np.dot(flange_x_dir_ref,flange_y_dir_ref)," ",np.dot(flange_x_dir_ref,flange_z_dir_ref)," ",np.dot(flange_y_dir_ref,flange_z_dir_ref))
    flange_HT_ref = build_homogeneous_transform(
        flange_x_dir_ref, flange_y_dir_ref, flange_z_dir_ref, flange_position
    )
    flange_rotation_matrix_ref = Rotation.from_matrix(flange_HT_ref[:3, :3])#法兰期望旋转矩阵
    flange_rotation_euler_ref = flange_rotation_matrix_ref.as_euler(
        "xyz", degrees=False
    )#法兰期望欧拉角
    flange_rotation_quat_ref = flange_rotation_matrix_ref.as_quat()#法兰期望四元数

    if output != 'euler':
        return flange_rotation_quat_ref
    else:
        return flange_rotation_euler_ref

def circular_cartesian_traj(n,
                            radius=0.1,
                            center=(0.0, 0.0, 0.0),
                            axis=(0.0, 0.0, 1.0),
                            period=5.0,
                            start_angle=0.0,
                            clockwise=False):
    """
    生成圆形笛卡尔轨迹（位置+速度），shape = (n, 6)。

    参数
    ----
    n : int
        采样点数（≥2）。轨迹按整圈周期等间隔采样。
    radius : float
        圆半径（米）。
    center : (3,) array-like
        圆心的世界坐标 [cx, cy, cz]。
    axis : (3,) array-like
        圆所在平面的法向量（任意方向，函数内会归一化）。
        例如 (0,0,1) 表示在 xy 平面画圆。
    period : float
        完成一整圈的时间（秒）。用于计算角速度与速度向量。
    start_angle : float
        起始相位（弧度）。0 表示在基向量 u 方向的半径上开始。
    clockwise : bool
        True 顺时针，False 逆时针（以 axis 指向的右手法则为准）。

    返回
    ----
    traj : (n, 6) ndarray
        每行为 [x, y, z, vx, vy, vz]。
    t : (n,) ndarray
        每个采样点对应的时间戳（从 0 到 period）。
    """
    assert n >= 2, "n 至少为 2。"
    center = np.asarray(center, dtype=float).reshape(3)
    k = np.asarray(axis, dtype=float).reshape(3)
    nk = np.linalg.norm(k)
    if nk == 0:
        raise ValueError("axis 不能是零向量。")
    k = k / nk  # 法向单位向量

    # 在与 axis 正交的平面里构造一组正交基 {u, v}
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, k)) > 0.99:  # 参考向量与法向接近平行则换一根
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(k, ref); u = u / np.linalg.norm(u)
    v = np.cross(k, u)  # 已正交，范数≈1

    # 时间、角速度、相位
    dt = period / (n - 1)
    direction = -1.0 if clockwise else 1.0
    omega = direction * 2.0 * np.pi / period  # rad/s
    t = np.arange(n) * dt
    theta = start_angle + omega * t

    # 位置 p(t) = c + r*(cosθ*u + sinθ*v)
    cos_th, sin_th = np.cos(theta), np.sin(theta)
    pos = center + radius * (np.outer(cos_th, u) + np.outer(sin_th, v))  # (n,3)

    # 速度 ṗ(t) = r*ω*(-sinθ*u + cosθ*v)
    vel = radius * omega * (-np.outer(sin_th, u) + np.outer(cos_th, v))  # (n,3)

    traj = np.hstack([pos, vel])  # (n,6)
    return traj

if __name__ == "__main__":

    rospy.init_node("panda_env")
    Publisher_TrajDeformPos = rospy.Publisher("/TrajDeform/Pos", Point, queue_size=2)
    Publisher_TrajDeformVel = rospy.Publisher("/TrajDeform/Vel", Point, queue_size=2)
    Publisher_InitTrajPos = rospy.Publisher("/InitTraj/Pos", Point, queue_size=2)
    Publisher_InitTrajVel = rospy.Publisher("/InitTraj/Vel", Point, queue_size=2)
    Publisher_falconForce = rospy.Publisher("/falconForce", Point, queue_size=1)
    r = PandaArm()
    r.move_to_neutral()

# -0.019634035790649828, -0.7550763761790475, -0.1511138492329248, -2.355278205850114, 0.1145643581417179, 1.5380768570201055, 0.7517870086066382


    kinematics_tool = PandaKinematics(r, "panda_link10")
    kinematics_flange = PandaKinematics(r, "panda_link8")
    group_name = "panda_arm"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    callback = CallBackFunction()
    movegroup = MoveGroupPythonInterfaceTutorial(r)
    goal_positions = np.array(
        [
            (0.35, 0.0, 0.2),#0.35
            (0.3, 0.15, 0.2),#x:0.3 y:-0.05 z:0.2
            (0.3, 0.05, 0.2),
        ]
    )  # 可能的目标位置
    trocar_position = np.array([0.3, 0, 0.3])#模型文件里x需+0.005  0.35
    length = 0.4

    choice = input("circle traj input 1：").strip()
    if choice == '1':
        circle_flag = True

    else:
        circle_flag = False

    button = callback.button
    old_button = button

    tool_pose = np.array(kinematics_tool.forward_position_kinematics())
    tool_position = tool_pose[0:3]

    fuzzy_func = FuzzyFunction(r, kinematics_tool)
    fuzzy_lambda = fuzzy_func.update_alpha(tool_position)

    w_tool = tool_pose[6]
    x_tool = tool_pose[3]
    y_tool = tool_pose[4]
    z_tool = tool_pose[5]  # quarternions

    tool_rotation_quat = [x_tool, y_tool, z_tool, w_tool]  # x=1, y=0, z=0, w=0
    tool_rotation_euler = quaternion_to_euler(tool_rotation_quat)

    flange_pose = np.array(kinematics_flange.forward_position_kinematics())
    flange_position = flange_pose[0:3]

    w_flange = flange_pose[6]
    x_flange = flange_pose[3]
    y_flange = flange_pose[4]
    z_flange = flange_pose[5]  # quarternions

    flange_rotation_quat = [
        x_flange,
        y_flange,
        z_flange,
        w_flange,
    ]  # x=1, y=0, z=0, w=0
    flange_rotation_euler = quaternion_to_euler(flange_rotation_quat)

    flange_velocity = list(kinematics_flange.forward_velocity_kinematics())
    flange_position_velocity = np.array(flange_velocity[0:3])
    flange_rotation_euler_velocity = np.array(flange_velocity[3:6])

    flange_position = flange_position.flatten()

    flange_rotation_quat = np.roll(quaternion.as_float_array(flange_rotation_quat), -1)
    flange_rotation_euler = quaternion_to_euler(flange_rotation_quat)

    old_tool_position = tool_position
    old_tool_rotation_euler = tool_rotation_euler

    #---------------

    Goal_ori = np.quaternion(w_flange,x_flange,y_flange,z_flange)

    Goal_omg = euler_to_quaternion(flange_rotation_euler_velocity).reshape(-1)
    Goal_omg = np.quaternion(Goal_omg[3],Goal_omg[0],Goal_omg[1],Goal_omg[2])

    delta_ori = 0

    # --------------

    goal_intent = GoalIntent()
    actions = np.array([0.01, 0.01, 0.01])  # 人类控制命令
    kappa = 0.1  # 距离的缩放因子
    Goal_beta = 3  # 理性指数
    delta = 0.1
    posterior_init = np.array([1.0, 0.0, 0.0])

    likelihood_probabilities = goal_intent.rbii_2(
        tool_position, goal_positions, actions, kappa, Goal_beta
    )
    transition_probabilities = goal_intent.calculate_transition_probabilities(
        goal_positions.shape[0], delta
    )
    # print(likelihood_probabilities)
    posterior_probabilities = goal_intent.bayesian_intent(
        likelihood_probabilities, transition_probabilities, posterior_init
    )

    gp = goal_positions[1, :]
    wpose = movegroup.move_group.get_current_pose().pose
    way_plan, quat_plan, joint_plan = movegroup.plan_path(gp)

    flag = True

    if circle_flag:
        circle_time = 20
        circle_time_points = 4000

        start_point = np.array([0.29712983,0.04218135,0.18591754])
        start_point = tool_position
        way_plan = circular_cartesian_traj(circle_time_points,0.05,start_point-np.array([0,0.05,0]),
                                        np.array([0,0,1]),circle_time,0
                                        ,False)
        way_plan = way_plan[:,0:3]

        td = TrajDeformMoveit(
            flag,
            way_plan,
            trocar_position,
            length,
            movegroup,
            Publisher_TrajDeformPos,
            Publisher_TrajDeformVel,
            Publisher_InitTrajPos,
            Publisher_InitTrajVel,
            circle_time    #不是圆形的话记得删掉
        )
    else:
        td = TrajDeformMoveit(
            flag,
            way_plan,
            trocar_position,
            length,
            movegroup,
            Publisher_TrajDeformPos,
            Publisher_TrajDeformVel,
            Publisher_InitTrajPos,
            Publisher_InitTrajVel,
            6,
        )
    td.goal = gp
    td.old_goal = gp

    numPoints = td.traj_N
    time_total = td.t_stop
    freq = 1 / td.delta
    rate = rospy.Rate(freq)
    zref_r = np.zeros((numPoints, 12))
    zref_h = np.zeros((numPoints, 12))
    zref_r = GenerateTrajectory_r(numPoints, time_total)
    zref_h = GenerateTrajectory_h(numPoints, time_total)
    neu_j = r._neutral_pose_joints
    neu_pos = np.array(kinematics_tool.forward_position_kinematics(joint_values=neu_j))
    z_neu = np.hstack((np.array(neu_pos[0:3].flatten()), np.zeros(9)))
    integration_delta_euler = 0
    integration_position = 0
    integration_delta_ori = 0
    integration_delta_euler_ex = 0
    integration_position_ex = 0
    i = 0

    m2 = np.diag([10, 10, 10])
    d2 = np.diag([100, 100, 100])
    k2 = np.diag([100, 100, 100])
    R_h = np.diag([0.0001, 0.0001, 0.0001])
    R_r = np.diag([0.0005, 0.0005, 0.0005])
    Q_hh = np.diag(
        [
            10,
            10,
            10,
            0.0001,
            0.0001,
            0.0001,
        ]
    )
    Q_rr = np.diag(
        [
            10,
            10,
            10,
            0.0001,
            0.0001,
            0.0001,
        ]
    )
    Q_hr = np.diag([0, 0, 0, 0, 0, 0])
    Q_rh = np.diag([0, 0, 0, 0, 0, 0])
    sys_Ac = np.vstack(
        (
            np.hstack((np.zeros((3, 3)), np.identity(3))),
            np.hstack((-np.linalg.inv(m2) * k2, -np.linalg.inv(m2) * d2)),
        )
    )
    sys_B = np.vstack((np.zeros((3, 3)), np.linalg.inv(m2)))
    sys_Bc = np.hstack((sys_B, sys_B))
    alpha = 0.5

    Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)

    R_c = alpha * R_h + (1 - alpha) * R_r

    Q_h = alpha * Q_hh + (1 - alpha) * Q_hr
    Q_r = alpha * Q_rh + (1 - alpha) * Q_rr

    C = np.array([[1, 1, 1, 0, 0, 0]])

    D = np.array([[0, 0, 0]])
    sysStateSpace = ct.ss(sys_Ac, sys_B, C, D)

    K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)

    # ——————————————————————— 实验 参数 ——————————————————————— #
    ####--------------------参数初始化-----------------------######
    # 质量矩阵 (对角矩阵)
    M = 10 * np.eye(3)
    # 阻尼矩阵
    C = 100 * np.eye(3)
    # 刚度矩阵
    Kc = 100 * np.eye(3)

    # 计算 M 的逆矩阵
    M_inv = np.linalg.inv(M)

    # 构建系统矩阵 A
    A_upper = np.hstack([np.zeros((3, 3)), np.eye(3)])
    A_lower = np.hstack([-M_inv @ Kc, -M_inv @ C])
    A = np.vstack([A_upper, A_lower])

    # print("A:",A)

    # 构建输入矩阵 B
    B_upper = np.zeros((3, 3))
    B_lower = M_inv
    B = np.vstack([B_upper, B_lower])

    # print("B:",B)

    # 状态加权矩阵 Q1
    Q1_upper = 2 * np.eye(3)     # 5
    Q1_lower = 0.0005 * np.eye(3)  #0.01 0.0001
    # 1000 init both
    Q1 = np.block([[Q1_upper, np.zeros((3, 3))],
                [np.zeros((3, 3)), Q1_lower]])


    # 控制加权矩阵 R1
    R1 = 0.000001 * np.eye(3)  #0.000001
    # 状态加权矩阵 Q2 (与Q1相同)
    Q2 = Q1.copy()  # 直接复制 Q1

    # 控制加权矩阵 R2
    R2 = R1.copy()  # 直接复制 R1

    # 初始状态向量
    x0 = np.array([[0.310575],
                [0.00565],
                [0.586541],
                [0],
                [0],
                [0]])

    # 姿态环保持参数
    K_or = 15  #仿真是1
    D_or = 0.1  #

    # 初始化K和迭代参数
    K = np.zeros((3,6))
    max_iter = 100
    iter_steps = 50
    tol = 1e-6
    alpha = 1
    T_max = 20
    bound = 10 #10
    force_Threshold = 5

    zref_r_record = []
    zref_h_record = []
    znow_array_record = []
    param_record = []
    u_total_record = []

    """
    改进版 加上L
    1.先要把tool端的轨迹换算成flange端的(只能迭代计算?)
    2. 算完P之后算M, 再算L

    唯一能做的，测试这个迭代计算耗时多少
    """

    time_start = time.perf_counter()
    L = np.zeros((3,1))
    L_new = L
    C1 = np.zeros((6,1))
    C2 = np.zeros((6,1))
    ####--------------------迭代求解-----------------------######
    for it in range(max_iter):

        # Z_now = np.hstack((td.x_star[2,:],td.dot_x_star[2,:])).T
        # Z_now_next = np.hstack((td.x_star[3,:],td.dot_x_star[3,:])).T
        # Z_now_dot = (Z_now_next - Z_now) / td.delta
        #% 计算当前K下的P1和P2
        A_cl = A + B@K

        #这里要加个负号，python中的形式是AX+XA_H-Q=0，matlab的形式是AX+XA_H+Q=0
        P1 = -linalg.solve_continuous_lyapunov(A_cl.T, Q1 + K.T @ R1 @ K)
        P2 = -linalg.solve_continuous_lyapunov(A_cl.T, Q2 + K.T @ R2 @ K)
        #% 更新K
        K_new = -np.linalg.inv(alpha*R1 + (1-alpha)*R2) @ B.T @ (alpha*P1 + (1-alpha)*P2)


        # print(   (2 * P1 @ ( B @ L + C1 )) .shape  )
        # print(   (2 * K_new.T @ R1 @ L) .shape )
        # M1 = -1/2* np.linalg.inv(A_cl) @ (2*P1 @ ( B @ L + C1 ) + 2 * K_new.T @ R1 @ L)
        # M2 = -1/2* np.linalg.inv(A_cl) @ (2*P2 @ ( B @ L + C2 ) + 2 * K_new.T @ R2 @ L)

        # C1 = C1
        # C2 = A @ Z_now - Z_now_dot

        # L_new = -2 * np.linalg.inv(alpha*R1 + (1-alpha)*R2) @ B.T @ (alpha*M1 + (1-alpha)*M2)

        #% 检查收敛
        if np.linalg.norm(K_new - K) < tol and np.linalg.norm(L_new - L) < tol:
            print("收敛……")
            time_end = time.perf_counter()
            timepass_cal = time_end - time_start
            print("timepass_cal:", timepass_cal)
            break

        K = K_new
        L = L_new


    # wb_master = Workbook()  # 创建一个新的工作簿
    # ws_master = wb_master.active  # 选择默认的工作表
    # ws_master.append(
    #     ["master_x", "master_y", "master_z", "master_a", "master_b", "master_c"]
    # )
    # wb_slave = Workbook()  # 创建一个新的工作簿
    # ws_slave = wb_slave.active  # 选择默认的工作表
    # ws_slave.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    # wb_reshape = Workbook()  # 创建一个新的工作簿
    # ws_reshape = wb_reshape.active  # 选择默认的工作表
    # ws_reshape.append(
    #     ["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"]
    # )

    # wb_direct = Workbook()  # 创建一个新的工作簿
    # ws_direct = wb_direct.active  # 选择默认的工作表
    # ws_direct.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    # wb_robot = Workbook()  # 创建一个新的工作簿
    # ws_robot = wb_robot.active  # 选择默认的工作表
    # ws_robot.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    # wb_joint = Workbook()  # 创建一个新的工作簿
    # ws_joint = wb_joint.active  # 选择默认的工作表
    # ws_joint.append(
    #     ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    # )
    # wb_error = Workbook()
    # ws_error = wb_error.active  # 选择默认的工作表
    # ws_error.append(["error_rcm", "error_ee"])

    # wb_force = Workbook()  # 创建一个新的工作簿
    # ws_force = wb_force.active  # 选择默认的工作表
    # ws_force.append(["master_x", "master_y", "master_z"])

    # wb_arbitrary = Workbook()  # 创建一个新的工作簿
    # ws_arbitrary = wb_arbitrary.active  # 选择默认的工作表
    # ws_arbitrary.append(["alpha", "beta"])

    wb_zn = Workbook()  # 创建一个新的工作簿
    ws_zn = wb_zn.active  # 选择默认的工作表
    ws_zn.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )
    # 机械臂参考轨迹
    wb_zr = Workbook()  # 创建一个新的工作簿
    ws_zr = wb_zr.active  # 选择默认的工作表
    ws_zr.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )
    # 人参考轨迹
    wb_zh = Workbook()  # 创建一个新的工作簿
    ws_zh = wb_zh.active  # 选择默认的工作表
    ws_zh.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )
    # alpha
    wb_param = Workbook()  # 创建一个新的工作簿
    ws_param = wb_param.active  # 选择默认的工作表
    ws_param.append(
        ["alpha","μ","d_o","d_ws"]
    )

    wb_F_ur = Workbook()  # 创建一个新的工作簿
    ws_F_ur = wb_F_ur.active  # 选择默认的工作表
    ws_F_ur.append(["F_x", "F_y", "F_z","F_rx", "F_ry", "F_rz"])

    wb_F_human = Workbook()  # 创建一个新的工作簿
    ws_F_human = wb_F_human.active  # 选择默认的工作表
    ws_F_human.append(["F_x", "F_y", "F_z","F_rx", "F_ry", "F_rz"])

    wb_zn_tool = Workbook()  # 创建一个新的工作簿
    ws_zn_tool = wb_zn_tool.active  # 选择默认的工作表
    ws_zn_tool.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )
    # 机械臂参考轨迹
    wb_zr_tool = Workbook()  # 创建一个新的工作簿
    ws_zr_tool = wb_zr_tool.active  # 选择默认的工作表
    ws_zr_tool.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )
    # 人参考轨迹
    wb_zh_tool = Workbook()  # 创建一个新的工作簿
    ws_zh_tool = wb_zh_tool.active  # 选择默认的工作表
    ws_zh_tool.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )

    wb_zn_euler = Workbook()  # 创建一个新的工作簿
    ws_zn_euler = wb_zn_euler.active  # 选择默认的工作表
    ws_zn_euler.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )

    wb_zn_euler_ref = Workbook()  # 创建一个新的工作簿
    ws_zn_euler_ref = wb_zn_euler_ref.active  # 选择默认的工作表
    ws_zn_euler_ref.append(
        ["X", "Y", "Z", "X_dot", "Y_dot", "Z_dot"]
    )

    wb_rcm_error = Workbook()  # 创建一个新的工作簿
    ws_rcm_error = wb_rcm_error.active  # 选择默认的工作表
    ws_rcm_error.append(
        ["X", "Y", "Z","norm"]
    )

    wb_sys_expect = Workbook()  # 创建一个新的工作簿
    ws_sys_expect = wb_sys_expect.active  # 选择默认的工作表
    ws_sys_expect.append(
        ["X", "Y", "Z",
         "X_tool", "Y_tool", "Z_tool"]
    )

    wb_time_cal = Workbook()  # 创建一个新的工作簿
    ws_time_cal = wb_time_cal.active  # 选择默认的工作表
    ws_time_cal.append(
        ["time"]
    )

    # input("Hit Enter to Start")
    # print("commanding")
    time_end = time.perf_counter()
    time_start = time.perf_counter()

    # 原来全是 false
    flag_start = False
    flag_action = False
    init_flag = False
    test_flag = False
    old_flag = False

    if circle_flag:
        # r.move_to_joint_position((-0.019634035790649828,
        #             -0.7550763761790475,
        #             -0.1511138492329248,
        #             -2.355278205850114,
        #             0.1145643581417179,
        #             1.5380768570201055,
        #             0.7517870086066382))

        # r.move_to_joint_position((-0.01848378427622124,
        #             -0.7547233777613795,
        #             -0.0715708694655489,
        #             -2.3479457068989618,
        #             0.05093241950195976,
        #             1.5384883435234566,
        #             0.7883057236747434))

        # r.move_to_joint_position((-0.017640321250071832,
        #             -0.7361917504234373,
        #             -0.07737017283870784,
        #             -2.3267677138491827,
        #             0.052399383092671024,
        #             1.5323661378178712,
        #             0.7880795507101498))

        r.move_to_joint_position(( -0.017664692494403663,
                    -0.736219099210577,
                    -0.07725435347830424,
                    -2.3272117438017075,
                    0.0521345060270748,
                    1.5333807122210041,
                    0.7880720645160441))

    #    -0.017640321250071832, -0.7361917504234373, -0.07737017283870784, -2.3267677138491827, 0.052399383092671024, 1.5323661378178712, 0.7880795507101498
    #    -0.017664692494403663, -0.736219099210577, -0.07725435347830424, -2.3272117438017075, 0.0521345060270748, 1.5333807122210041, 0.7880720645160441
    """
    等待改进：
    0731
    1.障碍物区域 (ee_pose 距离调整） ok
    2.增加真实力推动  ok

    0731
    1. 产生交互力之前的轨迹长一点，这样轨迹跟踪效果看得更明显        2034 ok
    2. 直接力交互变成遥操作，这部分君洁做过了，找她要一下           ok 0801 2137
    3. 末端夹爪去掉，放上手术器械（这里我想让郝庆做一个，等下我和他说）
    4. 规划器械末端的运动，映射到机械臂末端做控制（这里我的理解就是根据杆的长度换算一下）

       a. Q: 两份代码怎么融合(arbitrary部分) A: 好像没什么好改的
       b. 速度项怎么获取
            flange_velocity = list(kinematics_flange.forward_velocity_kinematics())
            flange_pose = np.array(kinematics_flange.forward_position_kinematics())
    5. 和加权和法对比计算时间，这部分的对比结果具体数值是什么        ok     之前测试完了

    0826 version——
         代码改动: 1.flange末端和工具末端api规划
                  2. plan——path部分的代码改动 (fk.py部分也有

    0829 直接迁移到这里算了

    0831 为什么real会这么抖动而且跟不上

    0903
    1.paint 加上 tool的xyz轨迹  ok
    2.加上obstacle 完整 ok
    3. 留意规划出来的 numPoints 和 time_total 和td.traj_N

    为什么会抖动，姿态算的不对调整不好？

    0910
    1. 加 L 矩阵 计算                     now: 代码写好了               Q：好像需要再实时的跑才可以
    2. 重新规划行走路线实验                 now：
    3. MPC的代码怎么生成 toolbox？
    4. 看rcm约束点的距离(直接文档记录吧)               ok
    5. 实机实验
    6. 改 alpha 计算公式                            ok           但为什么要改？ 现在先没变
    """

    # Vector_dis = flange_position-tool_position
    flange_expect_pos = np.array((0,0,0))
    tool_expect_pos = np.array((0,0,0))
    euler_expect = np.array((0,0,0))

    print(f"numPoints:{numPoints},td.traj_N:{td.traj_N},delta:{td.delta},time:{td.t_stop}")
    print(f"tool_position:{tool_position}")

    """
    提取之前的human traj放便调参
    """
    base_path_old = Path('/home/hjh/panda_ws/Experiment_LM/0911/V4human轨迹')
    if circle_flag:
        # base_path_old = Path('/home/hjh/panda_ws/Experiment_LM/0916/圆形轨迹')
        # base_path_old = Path('/home/hjh/panda_ws/Experiment_LM/0925/V2_traj')
        # base_path_old = Path('/home/hjh/panda_ws/Experiment_LM/0926/V2')
        base_path_old = Path('/home/hjh/panda_ws/Experiment_LM/sim0928/V3')


    df_human_ref_tool = pd.read_excel(base_path_old/'Human_traj_ref_tool.xlsx')

    human_ref_tool_old = np.zeros((len(df_human_ref_tool['X']),6))
    human_ref_tool_old[:,0] = df_human_ref_tool['X']
    human_ref_tool_old[:,1] = df_human_ref_tool['Y']
    human_ref_tool_old[:,2] = df_human_ref_tool['Z']
    # print(human_ref_tool_old)
    for i in range(len(df_human_ref_tool['X'])):
            if i == 0:
                human_ref_tool_old[i, 3:6] = [0, 0, 0]
            else:
               human_ref_tool_old[i, 3:6] = (
                    human_ref_tool_old[i, 0:3] - human_ref_tool_old[i - 1, 0:3]
                ) / td.delta

    df_robot_ref_tool = pd.read_excel(base_path_old/'Robot_traj_ref_tool.xlsx')

    robot_ref_tool_old = np.zeros((len(df_robot_ref_tool['X']),6))
    robot_ref_tool_old[:,0] = df_robot_ref_tool['X']
    robot_ref_tool_old[:,1] = df_robot_ref_tool['Y']
    robot_ref_tool_old[:,2] = df_robot_ref_tool['Z']

    for i in range(len(df_robot_ref_tool['X'])):
        if i == len(df_robot_ref_tool['X'])-1:
            robot_ref_tool_old[i, 3:6] = [0, 0, 0]
        else:
            robot_ref_tool_old[i, 3:6] = (
                robot_ref_tool_old[i+1, 0:3] - robot_ref_tool_old[i, 0:3]
            ) / td.delta


    df_robot_ref = pd.read_excel(base_path_old/'Robot_traj_ref.xlsx')

    robot_ref_old = np.zeros((len(df_robot_ref['X']),6))
    robot_ref_old[:,0] = df_robot_ref['X']
    robot_ref_old[:,1] = df_robot_ref['Y']
    robot_ref_old[:,2] = df_robot_ref['Z']

    for i in range(len(df_robot_ref['X'])):
        if i == len(df_robot_ref['X'])-1:
            robot_ref_old[i, 3:6] = [0, 0, 0]
        else:
            robot_ref_old[i, 3:6] = (
                robot_ref_old[i+1, 0:3] - robot_ref_old[i, 0:3]
            ) / td.delta

        # robot_ref_old[i, 3:6] = [0, 0, 0]


    alpha_prev = 1
    smoothing_factor = 0.0023  # 较小的值提供更强平滑效果

    input("Hit Enter to Start")
    print("commanding")

    time_start = time.perf_counter()

    while not rospy.is_shutdown():
        time_end = time.perf_counter()
        timepass = time_end - time_start
        # print("timepass:", timepass)
        time_start = time.perf_counter()

        td.h = td.h + 1
        if td.h < td.traj_N - 1:
            td.deform()

        # 工具末端状态更新
        tool_pose = np.array(kinematics_tool.forward_position_kinematics())
        tool_position = tool_pose[0:3]
        w = tool_pose[6]
        x = tool_pose[3]
        y = tool_pose[4]
        z = tool_pose[5]  # quarternions
        tool_rotation_quat = [x, y, z, w]  # x=1, y=0, z=0, w=0
        tool_rotation_euler = quaternion_to_euler(tool_rotation_quat)

        tool_velocity = list(kinematics_tool.forward_velocity_kinematics())
        tool_position_velocity = np.array(tool_velocity[0:3])

        # 法兰状态更新
        flange_pose = np.array(kinematics_flange.forward_position_kinematics())
        flange_position = flange_pose[0:3]

        # print(f"flange_position:{flange_position},flange_expect_pos:{flange_expect_pos}")

        w_flange = flange_pose[6]
        x_flange = flange_pose[3]
        y_flange = flange_pose[4]
        z_flange = flange_pose[5]  # quarternions

        flange_rotation_quat = [
            x_flange,
            y_flange,
            z_flange,
            w_flange,
        ]  # x=1, y=0, z=0, w=0
        flange_rotation_euler = quaternion_to_euler(flange_rotation_quat)

        flange_velocity = list(kinematics_flange.forward_velocity_kinematics())
        flange_position_velocity = np.array(flange_velocity[0:3])
        flange_rotation_euler_velocity = np.array(flange_velocity[3:6])

        flange_position = flange_position.flatten()

        flange_x_dir, flange_y_dir, flange_z_dir = get_axis_vectors_direct(
            euler_angles=flange_rotation_euler
        )
        flange_HT = build_homogeneous_transform(
            flange_x_dir, flange_y_dir, flange_z_dir, flange_position
        )
        # print("\n齐次变换矩阵 H:\n", flange_HT)

        button = callback.button
        if button == 4 and old_button == 0:
            z_neu = np.hstack((tool_position, np.zeros(9)))
        elif button == 0 and old_button == 4:
            z_neu = np.hstack((old_tool_position, np.zeros(9)))
        old_button = button

        position_falcon = callback.position_falcon
        vel_falcon = callback.vel_falcon

        # fuzzy_lambda = fuzzy_func.update_alpha(tool_position)
        # willing = callback.HumanWilling()
        # alpha = fuzzy_lambda * willing

        """

        """

        Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)
        R_c = alpha * R_h + (1 - alpha) * R_r
        Q_h = alpha * Q_hh + (1 - alpha) * Q_hr
        Q_r = alpha * Q_rh + (1 - alpha) * Q_rr


        zref_r_array = np.hstack(
            (
                td.x_d_curr_init,#位置
                np.zeros(3),#姿态角度
                td.dot_x_d_curr_init,#速度
                np.zeros(3),#角速度
            )
        )

        zref_h_array_reshape = np.hstack(
            (
                td.x_d_curr,
                np.zeros(3),
                td.dot_x_d_curr,
                np.zeros(3),
            )
        )

        if old_flag:
            zref_h_array_reshape_old = np.hstack(
                (
                    human_ref_tool_old[td.h-1,0:3],#位置
                    np.zeros(3),#姿态角度
                    human_ref_tool_old[td.h-1,3:6],#速度
                    np.zeros(3),#角速度
                )
            )
            zref_h_array_reshape = zref_h_array_reshape_old

            zref_r_array = np.hstack(
                (
                    robot_ref_tool_old[td.h-1,0:3],#位置
                    np.zeros(3),#姿态角度
                    robot_ref_tool_old[td.h-1,3:6],#速度
                    np.zeros(3),#角速度
                )
            )


        # scale = 4
        # position_direct = willing * position_falcon * scale
        # vel_direct = willing * vel_falcon * scale

        # zref_h_array_direct = (
        #     np.hstack(
        #         (
        #             position_direct,
        #             np.zeros(3),
        #             vel_direct,
        #             np.zeros(3),
        #         )
        #     )
        #     + z_neu
        # )
        # DirectWilling_distance = np.linalg.norm(td.x_d_curr - td.x_d_curr_init)

        # beta = callback.DirectWilling(DirectWilling_distance)
        # arbitrary = np.array([alpha, beta])

        # zref_h_array = beta * zref_h_array_direct + (1 - beta) * zref_h_array_reshape#合成的人的期望
        zref_h_array = zref_h_array_reshape   #不用合成直接重塑
        zref_h_array_temp = np.hstack((zref_h_array[0:3], zref_h_array[6:9]))
        zref_r_array_temp = np.hstack((zref_r_array[0:3], zref_r_array[6:9]))
        # state_ref = (
        #     np.hstack((position_falcon, np.zeros(3), vel_falcon, np.zeros(3))) + z_neu
        # )
        # state_ref=zref_r[i, :]+z_neu

        #-----------这里ljj code
        if init_flag ==False:
            K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)
            state_ref_temp = np.dot(
                np.transpose(np.linalg.inv(Q_c)),
                (
                    np.dot(np.transpose(Q_h), zref_h_array_temp)
                    + np.dot(np.transpose(Q_r), zref_r_array_temp)
                ),
            )#工具末端期望状态
            state_ref = np.hstack(
                (state_ref_temp[0:3], np.zeros(3), state_ref_temp[3:6], np.zeros(3))
            )
            tool_position_ref = state_ref[0:3]#工具末端期望位置
            flange_position_ref = tool_position_ref + length * (
                trocar_position - tool_position_ref
            ) / np.linalg.norm(trocar_position - tool_position_ref)#法兰期望位置

            tool_velocity_ref = state_ref_temp[3:6]#工具末端期望速度

            flange_vel_ref = tool_velocity_ref+ length * (
                np.array([0,0,0]) - tool_velocity_ref
            ) / np.linalg.norm(trocar_position - tool_position_ref)#法兰期望位置

            flange_vel_ref = np.array([0,0,0])



            zref_r_array_flange = np.hstack((flange_position_ref,flange_vel_ref))
            zref_h_array_flange = np.hstack((flange_position_ref,flange_vel_ref))


        else:
            # ----------这里分别算出 human_want 和 init 转换后的flange轨迹位置--------------#
            #-----------这里是 human_want 的 flange位置
            state_ref_temp = zref_h_array_temp
            state_ref = np.hstack(
                (state_ref_temp[0:3], np.zeros(3), state_ref_temp[3:6], np.zeros(3))
            )
            tool_position_ref = state_ref[0:3]#工具末端期望位置
            tool_velocity_ref = state_ref_temp[3:6]#工具末端期望速度

            flange_position_ref = tool_position_ref + length * (
                trocar_position - tool_position_ref
            ) / np.linalg.norm(trocar_position - tool_position_ref)#法兰期望位置
            # 没有rcm约束的话，直接反算杆长回去就行
            # flange_position_ref =tool_position_ref + Vector_dis
            # flange_position_ref = tool_position_ref + np.array((0,0,length))
            flange_vel_ref = tool_velocity_ref+ length * (
                np.array([0,0,0]) - tool_velocity_ref
            ) / np.linalg.norm(trocar_position - tool_position_ref)#法兰期望位置

            # if td.h <= 1:
            #     flange_vel_ref = np.array([0,0,0])
            # else :
            #     flange_vel_ref = (flange_position_ref - zref_h_array_flange_old[0:3])/timepass

            zref_h_array_flange = np.hstack((flange_position_ref,flange_vel_ref))

            #-----------这里是 init_traj 的 flange位置
            state_ref_temp = zref_r_array_temp
            state_ref = np.hstack(
                (state_ref_temp[0:3], np.zeros(3), state_ref_temp[3:6], np.zeros(3))
            )
            tool_position_ref = state_ref[0:3]#工具末端期望位置
            tool_velocity_ref = state_ref_temp[3:6]#工具末端期望速度

            # trocar_position = flange_position

            flange_position_ref = tool_position_ref + length * (
                trocar_position - tool_position_ref
            ) / np.linalg.norm(trocar_position - tool_position_ref)#法兰期望位置
            # 没有rcm约束的话，直接反算杆长回去就行 不行
            # flange_position_ref =tool_position_ref + Vector_dis
            # flange_position_ref = tool_position_ref +np.array((0,0,length))
            flange_vel_ref = tool_velocity_ref+ length * (
                np.array([0,0,0]) - tool_velocity_ref
            ) / np.linalg.norm(trocar_position - tool_position_ref)#法兰期望位置

            # if td.h <= 1:
            #     flange_vel_ref = np.array([0,0,0])
            # else :
            #     flange_vel_ref = (flange_position_ref - zref_r_array_flange_old[0:3])/timepass

            zref_r_array_flange = np.hstack((flange_position_ref,flange_vel_ref))

            # 这个方法不行 因为会无限放大跟踪精度
            # tool_position_ref = tool_position
            # 要用关于pos的输入，系统方程求得下一时刻的flange位置，求得下一时刻的tool位置，用这两个期望求的期望姿态，再叠加输入

            # 这里还需要把交互力也换算吗？ uh部分

        #-----------这里用上一刻,也就是现在的tool位置算rcm约束点 的
        #

        flange_z_dir_ref = (tool_position_ref - trocar_position) / np.linalg.norm(
            tool_position_ref - trocar_position
        )
        # print("tool_2:",tool_position_ref)
        flange_y_dir_ref = np.cross(flange_z_dir_ref, flange_x_dir)
        flange_x_dir_ref = np.cross(flange_y_dir_ref, flange_z_dir_ref)#法兰期望法向量
        # print(np.dot(flange_x_dir_ref,flange_y_dir_ref)," ",np.dot(flange_x_dir_ref,flange_z_dir_ref)," ",np.dot(flange_y_dir_ref,flange_z_dir_ref))
        flange_HT_ref = build_homogeneous_transform(
            flange_x_dir_ref, flange_y_dir_ref, flange_z_dir_ref, flange_position_ref
        )
        flange_rotation_matrix_ref = Rotation.from_matrix(flange_HT_ref[:3, :3])#法兰期望旋转矩阵
        flange_rotation_euler_ref = flange_rotation_matrix_ref.as_euler(
            "xyz", degrees=False
        )#法兰期望欧拉角
        flange_rotation_quat_ref = flange_rotation_matrix_ref.as_quat()#法兰期望四元数

        position_rcm = flange_position + (
            (
                (tool_position - flange_position)
                @ (trocar_position - flange_position)
                * (tool_position - flange_position)
            )
            / np.square(length)
        )#杆和RCM垂线交点，认为是实际插入点

        distance_rcm_flange = np.linalg.norm(position_rcm - flange_position)
        distance_rcm_tool = np.linalg.norm(position_rcm - tool_position)

        # *********************************************************************************** PD 控制器***************************************************#
        # stiffness gains
        P_pos = 100
        P_ori = 50

        # damping gains
        D_pos = 10
        D_ori = 1

        I_pos = 100
        I_ori = 100

        if circle_flag:
            P_ori = 50  #20
            D_ori = 5   #3
            I_ori = 150

            # # # 200hz
            # P_ori = 70  #70   50
            # D_ori = 3   #2     3
            # I_ori = 120  #100 150

            # # sim
            # P_ori = 20  #70   50
            # D_ori = 1   #2     3
            # I_ori = 50  #100 150
            # # # 15 3000
            # P_ori = 50  #70
            # D_ori = 2   #2
            # I_ori = 200  #100 150

        # if circle_flag:
        #     P_ori = 10
        #     I_ori = 40 #40

        goal_pos = flange_position_ref
        now_pos = flange_position
        delta_pos = goal_pos - now_pos

        goal_euler = flange_rotation_euler_ref
        now_euler = flange_rotation_euler
        delta_euler = euler_angle_diff(now_euler, goal_euler)

        goal_vel = -state_ref[6:9] * (distance_rcm_flange / distance_rcm_tool)
        now_vel = flange_position_velocity
        delta_vel = goal_vel - now_vel

        goal_omg = state_ref[9:12]
        now_omg = flange_rotation_euler_velocity
        delta_omg = euler_angle_diff(now_omg, goal_omg)

        delta_z_position = np.hstack((delta_pos, delta_vel))
        delta_z_rotation = np.hstack((delta_euler, delta_vel))

        # ——————————————————————— 实验 代码 ——————————————————————— #
        # 不对姿态做调整，所以状态只是6维
        # ******************这里是固定的，后面还要修改****************** #
        if init_flag ==True:
            joint_limit = r.joint_limits()
            angles_now = deepcopy(r.angles())
            Jacobian = r.zero_jacobian()

            link_poses = r._kinematics.get_link_positions(r.joint_angles())
            link_poses['panda_tool_new'] = [tool_position[0],tool_position[1],tool_position[2],
                                            0,0,0,0]
            # print(link_poses)
            # object_poses = [[0,0.4,0.2],[0.4,0.3,0.2],[-0.2,-0.3,0.4]
            #                 ,[0.425,-0.04,0.55]]
            object_poses = [
                            [100,100,100]
                            # [0.312,0.047,0.185] #在两条轨迹中间
                            # [0.312,0.07,0.185] #在两条轨迹中间
                            # [0.314,0.11,0.185]  #在轨迹末尾
                            ]  # sim z 0.55 real 0.50  [0.55,-0.10,0.55][0.33,0.10,0.185]

            workspace = {
                    'x': [0.26, 0.34],
                    'y': [-0.1, 0.25],
                    'z': [0.0, 0.4]
                }
            if circle_flag:
                # workspace = {
                #     'x': [0.20, 0.4],
                #     'y': [-0.15, 0.25],
                #     'z': [0.0, 0.4]
                # }
                workspace = {
                    'x': [0.20, 0.4],
                    'y': [-0.13, 0.25],
                    'z': [0.0, 0.4]
                }
                object_poses = [
                            # [100,100,100]
                            # [0.312,0.047,0.185] #在两条轨迹中间
                            # [0.312,0.07,0.185] #在两条轨迹中间
                            # [0.314,0.11,0.185]  #在轨迹末尾
                            # [0.27,-0.12,0.185] #圆形轨迹
                            #  [0.265,-0.10,0.18] ,#圆形轨迹 2
                             [0.28,-0.098,0.18] ,#圆形轨迹 3 sim0928
                            ]  # sim z 0.55 real 0.50  [0.55,-0.10,0.55][0.33,0.10,0.185]

            μ = Role_law.Manipulability_space(angles_now,Jacobian,joint_limit)
            # miu tool的不用干，问题不大
            d_o = Role_law.Joint_space(link_poses,object_poses)
            # 这里需要改，因为还有末端的轨迹，或者我直接把tool的放进去
            d_ws = Role_law.Cartesian_space(workspace,tool_position)#这里应该是tool的末端

            #在交互的时候对do距离拉大
            # if td.h >2 :
            #     if (np.any(human_interact_old[td.h-1, :] != 0)
            #         and np.any(human_interact_old[td.h-1, :] - human_interact_old[td.h-2, :]) > 0
            #         ):
            #         d_o = 999

            # 注意要修改里面的min值
            alpha = Role_law.Arbitration_cal(μ,d_o,d_ws)


            # # 应用EMA滤波
            alpha_filtered = smoothing_factor * alpha + (1 - smoothing_factor) * alpha_prev
            alpha_prev = alpha_filtered
            alpha = alpha_prev
            # print(alpha)

            # alpha = 1

            # ********************************************************* #
            # zref_r_array_temp 和 zref_h_array_temp 替代下面两行了
            # zref_r_array = np.vstack((zref_r_array[0:3],zref_r_array[6:9]))
            # zref_h_array = np.vstack((zref_h_array[0:3],zref_h_array[6:9]))


            # print(f"znow_array:{znow_array},zref_h_array_temp:{zref_h_array_temp},zref_r_array_temp:{zref_r_array_temp}")

            # delta_z = np.vstack((delta_pos, delta_vel))
            # alpha -> 1 跟人 ->0 跟一开始的规划(安全)



            time_start_cal = time.perf_counter()

            K_x = -np.linalg.inv(alpha * R1 + (1 - alpha) * R2) @ B.T @ (alpha * P1 @ (znow_array-zref_h_array_flange)
                                                                        + (1-alpha)* P2 @ (znow_array-zref_r_array_flange))
            u_position = bound * np.tanh(1 / bound * K_x)

            time_end_cal = time.perf_counter()

            timepass_cal = time_end_cal - time_start_cal

        # ——————————————————————— 实验 代码end ——————————————————————— #

        znow_array = np.hstack((now_pos,now_vel))
        znow_array_tool = np.hstack((tool_position,tool_position_velocity))

        state_now = np.hstack((tool_position, tool_rotation_euler))

        integration_delta_euler = integration_delta_euler + delta_euler * (
            time_total / numPoints
        )
        integration_position = integration_position + delta_pos * (
            time_total / numPoints
        )
        integration_delta_ori = integration_delta_ori + delta_ori * (
            time_total / numPoints
        )
        # ——————————————————————— 控制code 切换 ——————————————————————— #
        if init_flag == True:


            u_position = bound * np.tanh(1 / bound * (K_x))
            # u_position = u_position + I_pos * integration_position   没法用 因为也没有flange的ref

            """
            这里的思路
            1.通过阻抗系统状态方程 求得flange下一时刻的位置
            2.flange下一时刻的位置与rcm点,约束求得下一时刻的tool位置
            3.根据下一时刻的flange和tool,求得姿态
            4.计算姿态环的输入u
            """
            # 0.看误差
            if td.h % 1 == 0:
                error_pos = now_pos-flange_expect_pos
                error_euler = now_euler-euler_expect
                # 1.
                u_human = np.array((0,0,0))  # human在flange端的输入假设为0 ？
                flange_expect_pos = discretized_update(znow_array,A,B,u_position,u_human,
                                                    #    timepass
                                                        td.delta
                                                       ,'rk4')#,'rk4'
                flange_expect_pos = flange_expect_pos[0:3]
                # 2.
                tool_expect_pos = flange_expect_pos + length * (
                    trocar_position - flange_expect_pos
                ) / np.linalg.norm(trocar_position - flange_expect_pos)#法兰期望位置
                # 3.
                euler_expect = pos2rotation_rcm(tool_expect_pos,trocar_position,flange_expect_pos,'euler')
                goal_euler = euler_expect
                now_euler = flange_rotation_euler
                delta_euler = euler_angle_diff(now_euler, goal_euler)
                goal_omg = state_ref[9:12]

                # goal_omg = delta_euler / (
                # timepass
                # )

                now_omg = flange_rotation_euler_velocity
                delta_omg = euler_angle_diff(now_omg, goal_omg)
                integration_delta_euler_ex = 1 * integration_delta_euler_ex + delta_euler * (
                # timepass
                td.delta
                # time_total / numPoints
                )
                print(f"integration_delta_euler_ex:{integration_delta_euler_ex},timepass:{timepass}")

                limit = 0.01
                integration_delta_euler_ex = np.clip(integration_delta_euler_ex, -limit, limit)


                # 4.
                u_rotation = (
                P_ori * delta_euler + D_ori * delta_omg + I_ori * integration_delta_euler_ex
                )
                # 加上积分项？
                delta_pos = flange_expect_pos - now_pos
                integration_position_ex = integration_position_ex + delta_pos * (
                timepass
                )
                # u_position = u_position + I_pos * integration_position_ex

            # print(f"delta_pos:{delta_pos},integration_position_ex:{integration_position_ex},error_pos:{error_pos},delta_euler:{delta_euler},integration_delta_euler_ex:{integration_delta_euler_ex},error_euler:{error_euler}")
            # print(f"delta_pos:{delta_pos},error_pos:{error_pos}")
            # print(f"delta_euler:{delta_euler},integration_delta_euler_ex:{integration_delta_euler_ex}")
            # u_rotation = np.array([0,0,0])

            # Goal_ori = euler_to_quaternion(goal_euler)
            # Goal_omg = euler_to_quaternion(goal_omg)

            # ee_ori = np.quaternion(w_flange,x_flange,y_flange,z_flange)
            # ee_omg = flange_rotation_euler_velocity

            # ee_omg = euler_to_quaternion(ee_omg)
            # ee_omg = np.quaternion(ee_omg[3],ee_omg[0], ee_omg[1], ee_omg[2])

            # delta_ori = quatdiff_in_euler(ee_ori, Goal_ori).reshape([3, 1])
            # delta_omg = quatdiff_in_euler(ee_omg, Goal_omg).reshape([3, 1])
            # force_or = P_ori * delta_ori + D_ori * delta_omg +  I_ori * integration_delta_ori

            # u_rotation = force_or.flatten()

            u = np.hstack((u_position, u_rotation))

            Jacobian = np.array(kinematics_flange.jacobian())

            j_torque = np.dot(np.transpose(Jacobian), u).flatten()

            r.exec_torque_cmd(j_torque)

            # flange_expect_pos = discretized_update(znow_array,A,B,u_position,u_human,timepass,'test')#,'rk4'
            # flange_expect_pos = flange_expect_pos[0:3]
            # # 2.
            # tool_expect_pos = flange_expect_pos + length * (
            #     trocar_position - flange_expect_pos
            # ) / np.linalg.norm(trocar_position - flange_expect_pos)#法兰期望位置
        else:

            u_position = np.dot(K_gt, delta_z_position)
            u_position = u_position + I_pos * integration_position


                # 1.
            if test_flag:
                u_human = np.array((0,0,0))  # human在flange端的输入假设为0 ？
                flange_expect_pos = discretized_update(znow_array,A,B,u_position,u_human,timepass)#,'rk4'
                flange_expect_pos = flange_expect_pos[0:3]
                # 2.
                tool_expect_pos = flange_expect_pos + length * (
                    trocar_position - flange_expect_pos
                ) / np.linalg.norm(trocar_position - flange_expect_pos)#法兰期望位置
                # 3.
                euler_expect = pos2rotation_rcm(tool_expect_pos,trocar_position,flange_expect_pos,'euler')
                goal_euler = euler_expect
                now_euler = flange_rotation_euler
                delta_euler = euler_angle_diff(now_euler, goal_euler)
                goal_omg = state_ref[9:12]
                now_omg = flange_rotation_euler_velocity
                delta_omg = euler_angle_diff(now_omg, goal_omg)
                integration_delta_euler_ex = integration_delta_euler_ex + delta_euler * (
                time_total / numPoints
                )
                # 4.
                u_rotation = (
                P_ori * delta_euler + D_ori * delta_omg + I_ori * integration_delta_euler_ex
                )
                # 4.
            else :
                u_rotation = (
                P_ori * delta_euler + D_ori * delta_omg + I_ori * integration_delta_euler
                )

            # ee_ori = np.quaternion(w_flange,x_flange,y_flange,z_flange)
            # ee_omg = flange_rotation_euler_velocity

            # ee_omg = euler_to_quaternion(ee_omg)
            # ee_omg = np.quaternion(ee_omg[3],ee_omg[0], ee_omg[1], ee_omg[2])

            # delta_ori = quatdiff_in_euler(ee_ori, Goal_ori).reshape([3, 1])
            # delta_omg = quatdiff_in_euler(ee_omg, Goal_omg).reshape([3, 1])
            # force_or = P_ori * delta_ori + D_ori * delta_omg +  I_ori * integration_delta_ori

            # u_rotation = force_or.flatten()
            # u_rotation = np.array([0,0,0])

            u = np.hstack((u_position, u_rotation))

            Jacobian = np.array(kinematics_flange.jacobian())

            j_torque = np.dot(np.transpose(Jacobian), u).flatten()

            r.exec_torque_cmd(j_torque)



        # ——————————————————————— ljj 控制代码end ——————————————————————— #
        # u_position = u_position + I_pos * integration_position

        # u_rotation = (
        #     P_ori * delta_euler + D_ori * delta_omg + I_ori * integration_delta_euler
        # )

        # u = np.hstack((u_position, u_rotation))

        # Jacobian = np.array(kinematics_flange.jacobian())

        # j_torque = np.dot(np.transpose(Jacobian), u).flatten()

        # r.exec_torque_cmd(j_torque)

        error_rcm_distance = trocar_position - position_rcm
        error_rcm_distance_norm = np.linalg.norm(error_rcm_distance)
        error = np.array(
            [
                error_rcm_distance_norm,
                np.linalg.norm(tool_position - tool_position_ref),
            ]
        )

        actions = tool_position_ref - tool_position
        if np.max(td.f_h_now) != 0.0 or np.min(td.f_h_now) != 0.0:
            likelihood_probabilities = goal_intent.rbii_2(
                tool_position, goal_positions, actions, kappa, Goal_beta
            )

            posterior_probabilities = goal_intent.bayesian_intent(
                likelihood_probabilities, transition_probabilities, posterior_init
            )
            order = np.argsort(posterior_probabilities)
            td.goal = goal_positions[order[2]]
            posterior_init = posterior_probabilities
            print(
                # "likelihood:",
                # likelihood_probabilities,
                "posterior:",
                posterior_probabilities,
                "; order:",
                order,
                "; goal:",
                td.goal,
            )




        if (
            (error_rcm_distance_norm >= 0.005
             or np.linalg.norm(delta_euler) >= 0.005
            #  or np.linalg.norm(delta_pos) >= 0.005
             )
            and td.h == 0
            and flag_start == False
            and flag_action == False
        ):
            td.h = td.h - 1
            td.k = td.k - 1

            print(f"delta_euler:{delta_euler},delta_pos:{delta_pos}")

        elif (
            error_rcm_distance_norm < 0.005
            and td.h == 0
            and flag_start == False
            and flag_action == False
            # and np.linalg.norm(delta_euler) < 0.005
        ):
            flag_start = True

        if flag_start == True:
            flag_start = False
            flag_action = True

            Goal_ori = np.quaternion(w_flange,x_flange,y_flange,z_flange)

            Goal_omg = euler_to_quaternion(flange_rotation_euler_velocity).reshape(-1)
            Goal_omg = np.quaternion(Goal_omg[3],Goal_omg[0],Goal_omg[1],Goal_omg[2])

            Vector_dis = flange_position-tool_position

            u_position = np.array((0,0,0))
            u_rotation = np.array((0,0,0))

            print(f"tool_position:{tool_position},flange_position_ref:{tool_position_ref}")
            print(f"tool_position:{flange_position},flange_position_ref:{flange_position_ref}")

            input("Hit Enter to Start")
            print("commanding")

            time_start = time.perf_counter()

            # test_flag =True
            init_flag = True
            old_flag = True

        if init_flag and td.h > 0:
            zref_h_array_flange_old = zref_h_array_flange
            zref_r_array_flange_old = zref_r_array_flange

        if flag_action == True and td.h > 0:
            time_cal = np.hstack((timepass_cal,timepass))

            ws_zn.append(znow_array.flatten().tolist())
            ws_zr.append(zref_r_array_flange.flatten().tolist())
            ws_zh.append(zref_h_array_flange.flatten().tolist())
            ws_param.append([alpha,μ,d_o,d_ws])
            ws_F_ur.append(u.flatten().tolist())
            ws_F_human.append(td.f_h_now.flatten().tolist())

            ws_zn_tool.append(znow_array_tool.flatten().tolist())
            ws_zr_tool.append(zref_r_array_temp.flatten().tolist())
            ws_zh_tool.append(zref_h_array_temp.flatten().tolist())

            ws_zn_euler.append(np.hstack((now_euler,flange_rotation_euler_velocity)).flatten().tolist())
            ws_zn_euler_ref.append(goal_euler.flatten().tolist())

            ws_rcm_error.append(np.hstack((error_rcm_distance,error_rcm_distance_norm)).flatten().tolist())

            ws_time_cal.append(time_cal.flatten().tolist())

            ws_sys_expect.append(np.hstack((flange_expect_pos,tool_expect_pos)).flatten().tolist())
            # ws_master.append(state_ref[0:6].flatten().tolist())
            # ws_slave.append(state_now[0:6].flatten().tolist())
            # ws_error.append(error.flatten().tolist())
            # ws_force.append(callback.force_falcon.flatten().tolist())
            # ws_arbitrary.append(arbitrary.flatten().tolist())
            # ws_reshape.append(zref_h_array_reshape[0:6].flatten().tolist())
            # ws_direct.append(zref_h_array_direct[0:6].flatten().tolist())
            # ws_robot.append(zref_r_array[0:6].flatten().tolist())

        old_tool_position = tool_position
        old_tool_rotation_euler = tool_rotation_euler
        if td.h >= td.traj_N - 1:

            u_position = np.array((0,0,0))
            u_rotation = np.array((0,0,0))

            u = np.hstack((u_position, u_rotation))

            Jacobian = np.array(kinematics_flange.jacobian())

            j_torque = np.dot(np.transpose(Jacobian), u).flatten()

            r.exec_torque_cmd(j_torque)


            print("saving……")

            # wb_master.save("/home/hjh/panda_ws/logs/file_traj_master.xlsx")
            # wb_slave.save("/home/hjh/panda_ws/logs/file_traj_slave.xlsx")
            # wb_reshape.save("/home/hjh/panda_ws/logs/file_traj_reshape.xlsx")
            # wb_direct.save("/home/hjh/panda_ws/logs/file_traj_direct.xlsx")
            # wb_robot.save("/home/hjh/panda_ws/logs/file_traj_robot.xlsx")
            # wb_joint.save("/home/hjh/panda_ws/logs/file_traj_joint.xlsx")
            # wb_error.save("/home/hjh/panda_ws/logs/error.xlsx")
            # wb_force.save("/home/hjh/panda_ws/logs/force.xlsx")
            # wb_arbitrary.save("/home/hjh/panda_ws/logs/arbitrary.xlsx")
            break
        rate.sleep()

    wb_zn.save(base_path/"Robot_traj_real.xlsx")
    wb_zr.save(base_path/"Robot_traj_ref.xlsx")
    wb_zh.save(base_path/"Human_traj_ref.xlsx")
    wb_param.save(base_path/"Param.xlsx")
    wb_F_ur.save(base_path/"U_input.xlsx")
    wb_F_human.save(base_path/"F_interact.xlsx")

    wb_zn_tool.save(base_path/"Robot_traj_real_tool.xlsx")
    wb_zr_tool.save(base_path/"Robot_traj_ref_tool.xlsx")
    wb_zh_tool.save(base_path/"Human_traj_ref_tool.xlsx")

    wb_zn_euler.save(base_path/"Robot_traj_real_euler.xlsx")
    wb_zn_euler_ref.save(base_path/"Robot_traj_real_euler_ref.xlsx")

    wb_rcm_error.save(base_path/"rcm_error.xlsx")

    wb_time_cal.save(base_path/"time_cal.xlsx")

    wb_sys_expect.save(base_path/"sys_expect.xlsx")
