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
import franka_interface
from scipy.spatial.transform import Rotation
import subprocess
import time
from matplotlib import pyplot as plt
import moveit_commander
import moveit_msgs.msg
from openpyxl import Workbook

from arbitrary import CallBackFunction


# 定义黎卡提方程
def Ricatti_P(t, solve_P, sys_A, sys_B, R, Q):
    f = -(
        np.dot(solve_P, sys_A)
        + np.dot(np.transpose(sys_A), solve_P)
        - np.dot(
            np.dot(
                np.dot(np.dot(solve_P, sys_B), np.linalg.inv(R)), np.transpose(sys_B)
            ),
            solve_P,
        )
        + Q
    )

    return f


def solve_Lyapunov(M, W):
    In = np.identity(np.shape(M)[0])
    n = np.shape(M)[0]
    temp = np.kron(In, np.transpose(M)) + np.kron(np.transpose(M), In)
    vec_W = np.reshape(W, (n * n, 1))
    solve_P = np.linalg.solve(temp, vec_W)
    solve_P = np.reshape(solve_P, (n, n))

    return solve_P


def init_care(sys_A, sys_B, R):
    In = np.identity(np.shape(sys_A)[0])
    B_new = np.dot(np.dot(sys_B, np.linalg.inv(R)), np.transpose(sys_B))
    eig_value, eig_vector = np.linalg.eig(sys_A)
    eig_value_real = np.real(eig_value)
    index = np.argmin(eig_value_real)
    alpha = -1 * eig_value_real[index]
    beta = np.argmax([0, alpha]) + 0.02
    A_beta = sys_A + beta * In
    solve_Z = solve_Lyapunov(A_beta, 2 * np.dot(B_new, np.transpose(B_new)))
    solve_F = np.linalg.solve(solve_Z, np.transpose(B_new))

    return solve_F


def solve_care(step, t, sys_A, sys_B, R, Q):
    solve_P = init_care(sys_A, sys_B, R)
    i = 0
    while 1:
        # Kk = np.dot(np.linalg.inv(R), np.dot(np.transpose(sys_B), solve_P))
        # Ak = sys_A - np.dot(sys_B, Kk)
        # solve_P = solve_Lyapunov(Ak, -Q - np.dot(np.dot(np.transpose(Kk), R), Kk))
        if np.linalg.norm(Ricatti_P(t, solve_P, sys_A, sys_B, R, Q)) < 0.001:
            # print(np.linalg.norm(Ricatti_P(t, solve_P, sys_A, sys_B, R, Q)))
            print(i)
            break
        t += step
        k1 = step * Ricatti_P(t, solve_P, sys_A, sys_B, R, Q)
        k2 = step * Ricatti_P(
            t + step * 0.5, solve_P + k1 * step * 0.5, sys_A, sys_B, R, Q
        )
        k3 = step * Ricatti_P(
            t + step * 0.5, solve_P + k2 * step * 0.5, sys_A, sys_B, R, Q
        )
        k4 = step * Ricatti_P(t + step, solve_P + k3 * step, sys_A, sys_B, R, Q)
        solve_P = solve_P + (k1 + k2 * 2 + k3 * 2 + k4) / 6
        solve_P = np.array(solve_P)
        i += 1

    return solve_P


def newton_solve_care(step, t, sys_A, sys_B, R, Q):
    solve_P = init_care(sys_A, sys_B, R)
    i = 0
    while 1:

        gain = np.dot(np.dot(np.linalg.inv(R), np.transpose(sys_B)), solve_P)
        sysA_bar = sys_A - np.dot(sys_B, gain)
        Lya_W = -Q - np.dot(np.dot(np.transpose(gain), R), gain)
        solve_P = solve_Lyapunov(sysA_bar, Lya_W)
        i += 1

        if np.linalg.norm(Ricatti_P(t, solve_P, sys_A, sys_B, R, Q)) < 0.001:
            # print(np.linalg.norm(Ricatti_P(t, solve_P, sys_A, sys_B, R, Q)))
            # print(i)
            break

    return solve_P


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
    curr_mat = quaternion.as_rotation_matrix(quat_curr)
    des_mat = quaternion.as_rotation_matrix(quat_des)
    rel_mat = des_mat.T.dot(curr_mat)
    rel_quat = quaternion.from_rotation_matrix(rel_mat)
    vec = quaternion.as_float_array(rel_quat)[1:]
    if rel_quat.w < 0.0:
        vec = -vec

    return -des_mat.dot(vec)


def execute_rostopic_pub(command):
    subprocess.run(command, shell=True)


# 初始化 gripper
homing_command = (
    "rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal '{}'"
)
loosen_command = "rostopic pub -1 /franka_gripper/grasp/goal franka_gripper/GraspActionGoal '{goal: {width: 0.1, epsilon: {inner: 0.01, outer: 0.01}, speed: 0.1, force: 0}}'"
grasp_command = "rostopic pub -1 /franka_gripper/grasp/goal franka_gripper/GraspActionGoal '{goal: {width: 0.02, epsilon: {inner: 0.01, outer: 0.01}, speed: 0.1, force: 10}}'"


def pos_sub(p, self):
    # rospy.loginfo("Pos: %f, %f ,%f",p.x,p.y,p.z)
    neu_j = [self._neutral_pose_joints[j] for j in self._joint_names]
    neu_pos, neu_rot = self.forward_kinematics(neu_j)
    # rospy.loginfo(neu_pos)
    pos = np.array([-p.x, -p.y, p.z]) + neu_pos.flatten()
    ori = np.quaternion(0, 1, 0, 0)
    status, j_des = self.inverse_kinematics(pos, ori)

    if status:
        # r.set_joint_positions_velocities(j_des, [0.0 for _ in range(7)])
        r.exec_position_cmd(j_des)
        # print(j_des)

    # pos_str = np.array2string(np.array([-p.x, -p.y, p.z]), separator=",")
    # file_master.write(pos_str)


def vec_sub(v, self):
    vec = np.array([v.x, v.y, v.z, 0, 0, 0])
    vec = vec[:, np.newaxis]
    Jacobian = self.jacobian()
    j_vec = np.dot(np.linalg.pinv(Jacobian), vec).flatten()
    # print("jacobian:",np.linalg.pinv(Jacobian))
    # print("j_vec:",j_vec)
    # r.exec_velocity_cmd(j_vec)
    # ee_point, ee_ori = self.ee_pose()
    # print("ee_point:",ee_point)


def joystick_sub(j, self):
    if j.buttons[0] == 1:
        # execute_rostopic_pub(grasp_command)
        self.exec_gripper_cmd(0.02, 10)
        print(j.buttons[0])
        # time.sleep(2)
    elif j.buttons[0] == 8:
        # execute_rostopic_pub(loosen_command)
        self.exec_gripper_cmd(0.1, 0)
        print(j.buttons[0])
        # time.sleep(2)


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
        point[5] = -np.pi
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
        point[0] = 0.005 * t
        point[1] = 0.005 * t
        point[2] = 0.005 * t

        # if pi / (0.02 * omega) <= t and t <= 2.5 * pi / (0.01 * omega):
        #     point[1] = 0.2
        # else:
        #     point[1] = 0.2 * np.sin(omega * point[0])

        point[3] = np.pi
        point[4] = 0
        point[5] = -np.pi

        point[6:9] = (point[0:3] - point_old) / dt
        point[9] = 0
        point[10] = 0
        point[11] = 0
        point_old = point[0:3]
        result.extend(point)
    result = np.array(result)
    result = np.reshape(result, (numPoint, 12))
    return result


if __name__ == "__main__":
    rospy.init_node("ctrl_test")
    r = PandaArm()
    numPoints = 300
    time_total = 5

    time_interval = time_total / numPoints
    freq = numPoints / time_total

    rate = rospy.Rate(freq)

    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)
    Jacobian = r.zero_jacobian()

    r.move_to_neutral()  # move to neutral pose before beginning
    Jacobian = r.zero_jacobian()

    initial_pose = deepcopy(r.angles())
    ee_point, ee_ori = r.ee_pose()

    ee_euler = quaternion_to_euler(quaternion.as_float_array(ee_ori))

    input("Hit Enter to Start")
    print("commanding")

    # 示例欧拉角（单位：弧度）
    euler_angles = [0.0, 1.27, 1.27]  # Roll, Pitch, Yaw
    file_master = open("/home/liu/catkin_franka/logs/file_master.txt", "a")
    file_master.truncate(0)
    file_slave = open("/home/liu/catkin_franka/logs/file_slave.txt", "a")
    file_slave.truncate(0)
    wb_master = Workbook()  # 创建一个新的工作簿
    ws_master = wb_master.active  # 选择默认的工作表
    ws_master.append(
        ["master_x", "master_y", "master_z", "master_a", "master_b", "master_c"]
    )
    wb_slave = Workbook()  # 创建一个新的工作簿
    ws_slave = wb_slave.active  # 选择默认的工作表
    ws_slave.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])
    vals = deepcopy(initial_pose)
    count = 0
    # sub1 = rospy.Subscriber("/falcon/ee_pose", Point, pos_sub, r, queue_size=1)
    # sub2 = rospy.Subscriber("falconVec",Vector3, vec_sub, r, queue_size=1)
    sub3 = rospy.Subscriber("/falcon/joystick", Joy, joystick_sub, r, queue_size=1)

    neu_j = [r._neutral_pose_joints[j] for j in r._joint_names]
    neu_pos, neu_rot = r.forward_kinematics(neu_j, "eul")
    step_num = 200
    t = 10
    step = -t / step_num

    m2 = np.diag([10, 10, 10, 10, 10, 10])
    d2 = np.diag([100, 100, 100, 100, 100, 100])
    k2_imp = np.diag([200, 200, 200, 200, 200, 200])
    k2 = np.diag([100, 100, 100, 100, 100, 100])
    R_h = np.diag([0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001])
    R_r = np.diag([0.0005, 0.0005, 0.0005, 0.0005, 0.0005, 0.0005])
    Q_hh = np.diag(
        [
            2,
            2,
            2,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
        ]
    )
    Q_rr = np.diag(
        [
            1,
            1,
            1,
            0.0005,
            0.0005,
            0.0005,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
            0.0001,
        ]
    )
    Q_hr = np.diag([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    Q_rh = np.diag([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    sys_Ac = np.vstack(
        (
            np.hstack((np.zeros((6, 6)), np.identity(6))),
            np.hstack((-np.linalg.inv(m2) * k2, -np.linalg.inv(m2) * d2)),  # k2_imp
        )
    )
    sys_B = np.vstack((np.zeros((6, 6)), np.linalg.inv(m2)))
    sys_Bc = np.hstack((sys_B, sys_B))
    alpha = 0.5

    zref_r = np.zeros((numPoints, 12))
    zref_h = np.zeros((numPoints, 12))
    zref_r = GenerateTrajectory_r(numPoints, time_total)
    zref_h = GenerateTrajectory_h(numPoints, time_total)

    time_init = rospy.Time.now()
    time_old = rospy.Time.now()
    posenow = []
    print("开始时间：", time_init)
    i = 0

    Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)

    R_c = np.vstack(
        (
            np.hstack((alpha * R_h, np.zeros((6, 6)))),
            np.hstack((np.zeros((6, 6)), (1 - alpha) * R_r)),
        )
    )
    Q_h = alpha * Q_hh + (1 - alpha) * Q_hr
    Q_r = alpha * Q_rh + (1 - alpha) * Q_rr
    time_init = rospy.Time.now()
    # solve_P = solve_care(step, t, sys_Ac, sys_Bc, R_c, Q_c)
    # print("Solution:", solve_P)
    # K_gt = np.dot(np.dot(np.linalg.inv(R_c), np.transpose(sys_Bc)), solve_P)

    # time_old = rospy.Time.now()
    # timepass = time_old - time_init
    # print("time pass：", timepass * np.power(0.1, 9))
    znow_array = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])[:, np.newaxis]
    z_real = []
    prev_z = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])[:, np.newaxis]
    C = np.array([[1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]])

    D = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    sysStateSpace = ct.ss(sys_Ac, sys_Bc, C, D)

    K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)

    callback = CallBackFunction()
    position_trocar = np.array([0.3, 0, 0.4])
    length = 0.107 + 0.5
    error_rcm = 0
    error_rcm_old = 0
    error_rcm_dot = 0

    while not rospy.is_shutdown():
        # if i >= numPoints:
        #     break

        # Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)

        # R_c = np.vstack(
        #     (
        #         np.hstack((alpha * R_h, np.zeros((6, 6)))),
        #         np.hstack((np.zeros((6, 6)), (1 - alpha) * R_r)),
        #     )
        # )
        # Q_h = alpha * Q_hh + (1 - alpha) * Q_hr
        # Q_r = alpha * Q_rh + (1 - alpha) * Q_rr
        # time_init = rospy.Time.now()
        # solve_P = solve_care(step, t, sys_Ac, sys_Bc, R_c, Q_c)
        # K_gt = np.dot(np.dot(np.linalg.inv(R_c), np.transpose(sys_Bc)), solve_P)
        # K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)

        time_start = time.perf_counter()
        ee_point, ee_ori = r.ee_pose()
        ee_euler = quaternion_to_euler(quaternion.as_float_array(ee_ori))

        ee_point_relative = ee_point - neu_pos.flatten()

        posenow.extend(ee_point_relative)

        ee_vel, ee_omg = r.ee_velocity()

        z_neu = np.vstack(
            (
                np.array(neu_pos.flatten())[:, np.newaxis],
                np.array([0, 0, 0])[:, np.newaxis],
                np.array([0, 0, 0])[:, np.newaxis],
                np.array([0, 0, 0])[:, np.newaxis],
            )
        )

        time_now = rospy.Time.now()
        time_diff_sec = time_now.to_sec() - time_old.to_sec()
        time_old = time_now
        time_pass_sec = time_now.to_sec() - time_init.to_sec()

        znow_array = np.hstack((ee_point, ee_euler, ee_vel, ee_omg))[:, np.newaxis]
        # print(znow_array.flatten())
        zref_r_array = zref_r[i, :][:, np.newaxis] + z_neu
        zref_h_array = zref_h[i, :][:, np.newaxis] + z_neu
        if i == 0:
            prev_z = znow_array

        zref_array = np.dot(
            np.transpose(np.linalg.inv(Q_c)),
            (
                np.dot(np.transpose(Q_h), zref_h_array)
                + np.dot(np.transpose(Q_r), zref_r_array)
            ),
        )
        # zref_point_str = np.array2string(zref_array[0:6].flatten(), separator="|")
        # file_master.write(zref_point_str)
        # ws_master.append(zref_array[0:6].flatten().tolist())
        # znow_point_str = np.array2string(znow_array[0:6].flatten(), separator="|")
        # file_slave.write(znow_point_str)
        # ws_slave.append(znow_array[0:6].flatten().tolist())

        # u = np.dot(K_gt, (zref_array - znow_array))

        # uh = u[0:6, :]
        # ur = u[6:12, :]
        # u_total = uh + ur
        # u_total[3:6] = -u_total[3:6]

        # *********************************************************************************** PD 控制器***************************************************#
        # stiffness gains
        P_pos = 50
        P_ori = 25
        # damping gains
        D_pos = 10
        D_ori = 1
        goal_pos = zref_array[0:3].reshape(1, 3)
        ori = euler_to_quaternion(zref_array[3:6].reshape(1, 3)).reshape(-1)
        goal_ori = np.quaternion(ori[0], ori[1], ori[2], ori[3])
        delta_pos = (goal_pos - ee_point).reshape([3, 1])
        # print(goal_ori)
        delta_ori = quatdiff_in_euler(ee_ori, goal_ori).reshape([3, 1])

        goal_vel = zref_array[6:9].reshape(1, 3)
        omg = euler_to_quaternion(zref_array[9:12].reshape(1, 3)).reshape(-1)
        goal_omg = np.quaternion(omg[0], omg[1], omg[2], omg[3])
        ee_omg = euler_to_quaternion(ee_omg)
        ee_omg = np.quaternion(ee_omg[0], ee_omg[1], ee_omg[2], ee_omg[3])
        delta_vel = (goal_vel - ee_vel).reshape([3, 1])
        # print(goal_ori)
        delta_omg = quatdiff_in_euler(ee_omg, goal_omg).reshape([3, 1])

        delta_z = np.vstack((delta_pos, delta_ori, delta_vel, delta_omg))
        # delta_ori = znow_array[3:6] - zref_array[3:6]

        # u = np.vstack(
        #     [
        #         P_pos * delta_pos,
        #         P_ori * delta_ori,
        #     ]
        # ) - np.vstack(
        #     [
        #         D_pos * ee_vel.reshape([3, 1]),
        #         D_ori * ee_omg.reshape([3, 1]),
        #     ]
        # )

        u = np.dot(K_gt, delta_z)

        uh = u[0:6, :]
        ur = u[6:12, :]
        u_total = uh + ur
        u_total[3:6] = -u_total[3:6]

        # print(delta_ori)

        Jacobian = r.zero_jacobian()
        # j_torque = np.dot(np.transpose(Jacobian), u[0:6]).flatten()
        j_torque = np.dot(np.transpose(Jacobian), u_total).flatten()
        # print("Jacobian:", Jacobian)

        # position_joint7 = callback.position_joint7
        # position_endeffector = callback.position_endeffector
        # jacobian_joint7 = callback.jacobian_joint7

        # position_joint7_d = position_endeffector + length * (
        #     position_trocar - position_endeffector
        # ) / (np.linalg.norm(position_trocar - position_endeffector))

        # error_rcm = position_joint7 - position_joint7_d
        # error_rcm_dot = (error_rcm - error_rcm_old) / (1 / freq)
        # K_rcm = 5
        # D_rcm = 0.1
        # force_joint7_ = -K_rcm * error_rcm - D_rcm * error_rcm_dot
        # force_joint7 = np.pad(force_joint7_, (0, 3), "constant")
        # Jacobian_trans = np.transpose(Jacobian)
        # Jacobian_pinv = np.dot(
        #     np.linalg.inv(np.dot(Jacobian, Jacobian_trans)), Jacobian
        # )

        # Identity = np.ones((7, 7))

        # NullSpace = Identity - np.dot(np.transpose(Jacobian), Jacobian)
        # tau_force_joint7 = np.dot(np.transpose(jacobian_joint7), force_joint7)
        # tau_rcm = np.dot(NullSpace, tau_force_joint7)
        # r.exec_torque_cmd(j_torque + tau_rcm)
        # print("error_rcm:", np.linalg.norm(error_rcm))
        coriolis=r.coriolis_comp()
        gravity=r.gravity_comp()*0.001
        j_total=j_torque+coriolis+gravity
        r.exec_torque_cmd(j_total)
        print("delta_ori:", np.linalg.norm(delta_ori.flatten()),";  delta_pose:", np.linalg.norm(delta_pos.flatten()))

        # ori = np.quaternion(0.71, -0.71, 0, 0)
        # status, j_des = r.inverse_kinematics(zref_array[0:3], ori)

        # if status:
        #     r.set_joint_positions_velocities(j_des, [0.0 for _ in range(7)])
        #     # r.exec_position_cmd(j_des)
        # print(j_des)

        prev_z = znow_array

        time_end = time.perf_counter()
        run_time = time_end - time_start
        # print("run_time", run_time)
        # print("eul", zref_array[3] - znow_array[3])
        i = i + 1
        error = np.linalg.norm(zref_array[0:3] - znow_array[0:3])
        # print("error:", error)
        if i >= numPoints:
            i = numPoints - 1
            # error = np.linalg.norm(zref_array[0:3] - znow_array[0:3])
            # print("error:", error)
            if error < 0.002:
                break

        rate.sleep()
    file_slave.close()
    file_master.close()
    wb_master.save("/home/liu/moveit_ws_20240902/logs/file_traj_master.xlsx")
    wb_slave.save("/home/liu/moveit_ws_20240902/logs/file_traj_slave.xlsx")
