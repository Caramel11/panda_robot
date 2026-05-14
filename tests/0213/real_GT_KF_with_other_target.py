import matplotlib.pyplot as plt
from matplotlib import font_manager
import os
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
from arbitrary import (
    CallBackFunction,
    TrajDeform,
    TrajDeformMoveit,
    GoalIntent,
    MoveGroupPythonInterfaceTutorial,
    FuzzyFunction,
    update_ref_state,
    update_robot_state,
    update_u_position,
    update_u_rotation,
    replan_to_target,
    find_closest_goal,
    make_log_path,
    setup_chinese_font_ubuntu,
)
from arbitrary import TrajDeform
from arbitrary import TrajDeformMoveit
from arbitrary import GoalIntent
from arbitrary import MoveGroupPythonInterfaceTutorial
from scipy.spatial.distance import cosine
from panda_robot import PandaKinematics
from arbitrary import FuzzyFunction
import arbitrary as arb
from datetime import datetime
from scipy.spatial.transform import Rotation
from kalman_filter import KalmanFilterFusion, DeltaLambdaUpdater
from fuzzy_logic import FuzzyLogicTool, Method2_IF_Else

root = "/home/hjh/ljj_ws/logs/0124_GT_KF_WithOtherTarget"  # log存储根目录
init_error_threshold = 0.005  # 初始化误差阈值
tracking_error_threshold = 0.01  # 跟踪误差阈值
target_error_threshold = 0.01  # 目标点误差阈值
title_str = "GT_KF_WithOtherTarget"  # 日志文件标题字符串
u_threshold = 30  # 控制输入阈值

if __name__ == "__main__":
    rospy.init_node("panda_env")
    k_gt_database = np.load("k_gt_database.npy", allow_pickle=True).item()
    setup_chinese_font_ubuntu()
    Publisher_TrajDeformPos = rospy.Publisher("/TrajDeform/Pos", Point, queue_size=2)
    Publisher_TrajDeformVel = rospy.Publisher("/TrajDeform/Vel", Point, queue_size=2)
    Publisher_InitTrajPos = rospy.Publisher("/InitTraj/Pos", Point, queue_size=2)
    Publisher_InitTrajVel = rospy.Publisher("/InitTraj/Vel", Point, queue_size=2)
    Publisher_falconForce = rospy.Publisher("/falconForce", Point, queue_size=1)
    td_publishers = (
        Publisher_TrajDeformPos,
        Publisher_TrajDeformVel,
        Publisher_InitTrajPos,
        Publisher_InitTrajVel,
    )

    freq = 100
    rate = rospy.Rate(freq)
    dot_time = 1 / freq

    force_falcon = np.array([0, 0, 0])
    pub_force_falcon = Point()
    K_falcon = 100
    D_falcon = 0

    r = PandaArm()

    r.exec_gripper_cmd(0.018)
    # r.move_to_neutral()
    angles_init = np.array(
        [
            -0.03572926,
            -0.71236292,
            -0.05355629,
            -2.31286173,
            0.04212054,
            1.5332542,
            0.71300622,
        ]
    )
    # r.exec_position_cmd(angles_init)
    r.move_to_joint_position(( -0.03572926,
            -0.71236292,
            -0.05355629,
            -2.31286173,
            0.04212054,
            1.5332542,
            0.71300622,))

    # rospy.sleep(0.5)
    # angles=r.angles()

    kinematics_tool = PandaKinematics(r, "panda_link10")
    kinematics_flange = PandaKinematics(r, "panda_link8")
    group_name = "panda_arm"
    callback = CallBackFunction()
    movegroup = MoveGroupPythonInterfaceTutorial(r)

    lambda_fuzzy = FuzzyLogicTool(type="lambda_based")
    delta_lambda_fuzzy = FuzzyLogicTool(type="delta_lambda_based")
    delta_updater = DeltaLambdaUpdater(lambda0=0.5, dt=dot_time)
    kf_fusion = KalmanFilterFusion(dt=dot_time, epsilon=0.01)
    # 【修改】目标点配置（核心：定义顺序+误差阈值）
    target_points = {
        "a": np.array([0.25, -0.04, 0.007]),  # 目标a
        "b": np.array([0.25, -0.04, 0.03]),  # 目标a
        "c": np.array([0.3, 0.01, 0.03]),  # 目标c
        "d": np.array([0.3, 0.01, 0.02]),  # 目标d
    }
    object_position = np.array([0.3, -0.05, 0.05])
    object_radius = 0.03
    target_order = ["a", "b", "c", "d"]  # 执行顺序：a→b→c
    current_target_idx = 0  # 当前执行的目标点索引
    current_target = target_points[target_order[current_target_idx]]  # 初始目标：a点

    goal_positions = np.array(
        [
            # (0.25, -0.05, 0.10),
            # (0.25, 0.0, 0.10),
            (0.25, 0.01, 0.03),
            # (0.3, -0.05, 0.10),
            # (0.3, 0.0, 0.10),
            (0.3, 0.01, 0.03),
            # (0.35, -0.05, 0.10),
            # (0.35, 0.0, 0.10),
            (0.35, 0.01, 0.03),
        ]
    )  # 可能的目标位置
    trocar_position = np.array([0.3, 0, 0.235])

    length = 0.525

    gp = current_target
    wpose = movegroup.move_group.get_current_pose().pose
    way_plan, quat_plan, joint_plan = movegroup.plan_path(gp)
    # print(joint_plan[-1,:])

    flag = True

    td = TrajDeformMoveit(
        freq,
        flag,
        way_plan,
        trocar_position,
        length,
        movegroup,
        Publisher_TrajDeformPos,
        Publisher_TrajDeformVel,
        Publisher_InitTrajPos,
        Publisher_InitTrajVel,
    )
    td.goal = gp
    td.old_goal = gp

    button = callback.button
    old_button = button

    tool_pose = np.array(kinematics_tool.forward_position_kinematics())
    tool_position = tool_pose[0:3]

    fuzzy_func = FuzzyFunction(r, kinematics_tool)
    robot_state = update_robot_state(kinematics_tool, kinematics_flange)

    goal_intent = GoalIntent()
    actions = np.array([0.01, 0.01, 0.01])  # 人类控制命令
    kappa = 0.1  # 距离的缩放因子
    Goal_beta = 3  # 理性指数
    delta = 0.1  # 转移概率参数
    posterior_init = np.array([0.0, 1.0, 0.0])  # 初始后验概率

    likelihood_probabilities = goal_intent.rbii_2(
        robot_state["tool_position"], goal_positions, actions, kappa, Goal_beta
    )
    transition_probabilities = goal_intent.calculate_transition_probabilities(
        goal_positions.shape[0], delta
    )
    # print(likelihood_probabilities)
    posterior_probabilities = goal_intent.bayesian_intent(
        likelihood_probabilities, transition_probabilities, posterior_init
    )

    neu_j = r._neutral_pose_joints
    neu_pos = np.array(kinematics_tool.forward_position_kinematics(joint_values=neu_j))
    z_neu = np.hstack((np.array(neu_pos[0:3].flatten()), np.zeros(9)))

    integration_delta_euler = 0
    integration_delta_pos = 0

    lambda1_list = []
    lambda_list = []
    delta_lambda_list = []

    F_h_list = []
    T_h_list = []
    D_r_list = []
    dF_h_list = []
    dT_h_list = []
    dD_r_list = []

    F_h = np.linalg.norm(callback.force_falcon)
    T_h = callback.force_time_total
    D_r = np.linalg.norm(robot_state["tool_position"] - object_position) - object_radius

    F_h_list.append(F_h)
    T_h_list.append(T_h)
    D_r_list.append(D_r)

    dF_h = (F_h_list[-1] - F_h_list[-2]) / dot_time if len(F_h_list) > 1 else 0
    dT_h = (T_h_list[-1] - T_h_list[-2]) / dot_time if len(T_h_list) > 1 else 0
    dD_r = (D_r_list[-1] - D_r_list[-2]) / dot_time if len(D_r_list) > 1 else 0

    dF_h_list.append(dF_h)
    dT_h_list.append(dT_h)
    dD_r_list.append(dD_r)

    lambda_w1 = lambda_fuzzy.compute([F_h, T_h, D_r], "lambda_based")
    dlambda_w1 = 0
    delta_lambda_w1 = delta_lambda_fuzzy.compute(
        [dF_h, dT_h, dD_r], "delta_lambda_based"
    )
    lambda_delta1 = delta_updater.update(delta_lambda_w1)
    # KF融合
    tau_k1 = np.array([[dlambda_w1], [lambda_delta1]])
    lambda_kf = kf_fusion.update(lambda_w1, tau_k1)
    print("lambda_kf:", lambda_kf)
    lambda1_list.append(lambda_kf)
    lambda_list.append(lambda_w1)
    delta_lambda_list.append(lambda_delta1)

    m2 = np.diag([10, 10, 10])
    d2 = np.diag([300, 300, 300])
    k2 = np.diag([100, 100, 100])
    R_h = np.diag([0.001, 0.001, 0.001])
    R_r = np.diag([0.002, 0.002, 0.002])
    Q_hh = np.diag([1000, 1000, 1000, 0.01, 0.01, 0.01])
    Q_rr = np.diag([1000, 1000, 1000, 0.01, 0.01, 0.01])
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
    alpha = lambda_kf

    Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)

    R_c = alpha * R_h + (1 - alpha) * R_r

    Q_h = alpha * Q_hh + (1 - alpha) * Q_hr
    Q_r = alpha * Q_rh + (1 - alpha) * Q_rr

    C = np.array([[1, 1, 1, 0, 0, 0]])

    D = np.array([[0, 0, 0]])
    sysStateSpace = ct.ss(sys_Ac, sys_B, C, D)

    K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)

    wb_master = Workbook()  # 创建一个新的工作簿
    ws_master = wb_master.active  # 选择默认的工作表
    ws_master.append(
        ["master_x", "master_y", "master_z", "master_a", "master_b", "master_c"]
    )
    wb_slave = Workbook()  # 创建一个新的工作簿
    ws_slave = wb_slave.active  # 选择默认的工作表
    ws_slave.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_reshape = Workbook()  # 创建一个新的工作簿
    ws_reshape = wb_reshape.active  # 选择默认的工作表
    ws_reshape.append(
        ["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"]
    )

    wb_direct = Workbook()  # 创建一个新的工作簿
    ws_direct = wb_direct.active  # 选择默认的工作表
    ws_direct.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_robot = Workbook()  # 创建一个新的工作簿
    ws_robot = wb_robot.active  # 选择默认的工作表
    ws_robot.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_human = Workbook()  # 创建一个新的工作簿
    ws_human = wb_human.active  # 选择默认的工作表
    ws_human.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_joint = Workbook()  # 创建一个新的工作簿
    ws_joint = wb_joint.active  # 选择默认的工作表
    ws_joint.append(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    )

    wb_joint_torque = Workbook()  # 创建一个新的工作簿
    ws_joint_torque = wb_joint_torque.active  # 选择默认的工作表
    ws_joint_torque.append(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    )

    wb_end_torque = Workbook()  # 创建一个新的工作簿
    ws_end_torque = wb_end_torque.active  # 选择默认的工作表
    ws_end_torque.append(
        ["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"]
    )

    wb_error = Workbook()
    ws_error = wb_error.active  # 选择默认的工作表
    ws_error.append(["error_rcm", "error_ee"])

    wb_force = Workbook()  # 创建一个新的工作簿
    ws_force = wb_force.active  # 选择默认的工作表
    ws_force.append(["master_x", "master_y", "master_z"])

    wb_arbitrary = Workbook()  # 创建一个新的工作簿
    ws_arbitrary = wb_arbitrary.active  # 选择默认的工作表
    ws_arbitrary.append(["alpha", "beta"])

    wb_FVD = Workbook()  # 创建一个新的工作簿
    ws_FVD = wb_FVD.active  # 选择默认的工作表
    ws_FVD.append(["F_h", "T_h", "D_r"])

    wb_dFVD = Workbook()  # 创建一个新的工作簿
    ws_dFVD = wb_dFVD.active  # 选择默认的工作表
    ws_dFVD.append(["dF_h", "dT_h", "dD_r"])

    wb_fuzzy = Workbook()  # 创建一个新的工作簿
    ws_fuzzy = wb_fuzzy.active  # 选择默认的工作表
    ws_fuzzy.append(["lambda", "delta_lambda"])

    wb_posterior_probabilities = Workbook()  # 创建一个新的工作簿
    ws_posterior_probabilities = wb_posterior_probabilities.active  # 选择默认的工作表
    ws_posterior_probabilities.append(["posterior_probabilities"])

    # input("Hit Enter to Start")
    # print("commanding")
    time_end = time.perf_counter()
    time_start = time.perf_counter()
    flag_start = False
    flag_action = False

    while not rospy.is_shutdown():
        time_end = time.perf_counter()
        timepass = time_end - time_start
        # print("timepass:", timepass)
        time_start = time.perf_counter()
        robot_state = update_robot_state(kinematics_tool, kinematics_flange)

        F_h = np.linalg.norm(callback.force_falcon)
        T_h = callback.force_time_total
        # if T_h > 0.5:
        print("force_time_total:", callback.force_time_total)
        D_r = (
            np.linalg.norm(robot_state["tool_position"] - object_position)
            - object_radius
        )

        F_h_list.append(F_h)
        T_h_list.append(T_h)
        D_r_list.append(D_r)

        dF_h = (F_h_list[-1] - F_h_list[-2]) / dot_time if len(F_h_list) > 1 else 0
        dT_h = (T_h_list[-1] - T_h_list[-2]) / dot_time if len(T_h_list) > 1 else 0
        dD_r = (D_r_list[-1] - D_r_list[-2]) / dot_time if len(D_r_list) > 1 else 0

        dF_h_list.append(dF_h)
        dT_h_list.append(dT_h)
        dD_r_list.append(dD_r)

        lambda_w1 = lambda_fuzzy.compute([F_h, T_h, D_r], "lambda_based")
        dlambda_w1 = 0
        delta_lambda_w1 = delta_lambda_fuzzy.compute(
            [dF_h, dT_h, dD_r], "delta_lambda_based"
        )
        lambda_delta1 = delta_updater.update(delta_lambda_w1)
        # KF融合
        tau_k1 = np.array([[dlambda_w1], [lambda_delta1]])
        lambda_kf = kf_fusion.update(lambda_w1, tau_k1)
        lambda1_list.append(lambda_kf)
        lambda_list.append(lambda_w1)
        delta_lambda_list.append(lambda_delta1)

        # 【核心修改】目标点误差判断+切换逻辑
        current_tool_pos = robot_state["tool_position"]
        target_error = np.linalg.norm(
            current_tool_pos - current_target
        )  # tool到当前目标的欧氏距离
        # rospy.loginfo(
        #     f"当前目标：{target_order[current_target_idx]}, 误差：{target_error:.6f}"
        # )
        min_dist, closest_goal_vec, closest_idx = find_closest_goal(
            current_tool_pos, goal_positions
        )
        # print(f"最小距离: {min_dist:.6f}")
        # print(f"对应的目标向量: {closest_goal_vec}")
        # print(f"该向量在goal_positions中的索引: {closest_idx}")

        # 误差达标，切换目标点（未到最后一个目标）
        if current_target_idx < len(target_order) - 1 and (
            (
                target_error < target_error_threshold
                and td.h >= td.traj_N
                and error[1] < tracking_error_threshold
            )
            or callback.button == 5
        ):
            current_target_idx += 1  # 切换到下一个目标
            current_target = target_points[target_order[current_target_idx]]
            # 重新规划轨迹到新目标
            if current_target_idx == 1:
                r.exec_gripper_cmd(0.045)
                # movegroup2 = MoveGroupPythonInterfaceTutorial(r)
                # wpose2 = movegroup.move_group.get_current_pose().pose
                # way_plan2, quat_plan2, joint_plan2 = movegroup2.plan_path(
                #     current_target
                # )
                current_target=robot_state["tool_position"]+np.array([0,0,0.02])

                num_point=int(np.linalg.norm(robot_state["tool_position"]-current_target)/0.00005)

                way_plan2=np.linspace(robot_state["tool_position"], current_target, num_point)
                
                # 重新规划轨迹到新目标
                td = replan_to_target(
                    movegroup=movegroup,
                    kinematics_tool=kinematics_tool,
                    td_publishers=td_publishers,
                    freq=freq,
                    trocar_position=trocar_position,
                    length=length,
                    target_pos=current_target,
                    way_plan=way_plan2,
                )
                print("way_start2:",way_plan2[0,:])
                print("way_end2:",way_plan2[-1,:])
                print("wpose2:",robot_state["tool_position"])


            elif current_target_idx == 2:
                # movegroup3 = MoveGroupPythonInterfaceTutorial(r)
                # wpose3 = movegroup.move_group.get_current_pose().pose
                # way_plan3, quat_plan3, joint_plan3 = movegroup3.plan_path(
                #     current_target
                # )

                num_point=int(np.linalg.norm(robot_state["tool_position"]-current_target)/0.00005)

                way_plan3=np.linspace(robot_state["tool_position"], current_target, num_point)
                # 重新规划轨迹到新目标
                td = replan_to_target(
                    movegroup=movegroup,
                    kinematics_tool=kinematics_tool,
                    td_publishers=td_publishers,
                    freq=freq,
                    trocar_position=trocar_position,
                    length=length,
                    target_pos=current_target,
                    way_plan=way_plan3,
                )
                print("way_start3:",way_plan3[0,:])
                print("way_end3:",way_plan3[-1,:])
                print("wpose3:",robot_state["tool_position"])

            elif current_target_idx == 3:
                # movegroup4 = MoveGroupPythonInterfaceTutorial(r)
                # wpose4 = movegroup.move_group.get_current_pose().pose
                # way_plan4, quat_plan4, joint_plan4 = movegroup4.plan_path(
                #     current_target
                # )
                current_target=robot_state["tool_position"]-np.array([0,0,0.02])

                num_point=int(np.linalg.norm(robot_state["tool_position"]-current_target)/0.00005)

                way_plan4=np.linspace(robot_state["tool_position"], current_target, num_point)
                # 重新规划轨迹到新目标
                td = replan_to_target(
                    movegroup=movegroup,
                    kinematics_tool=kinematics_tool,
                    td_publishers=td_publishers,
                    freq=freq,
                    trocar_position=trocar_position,
                    length=length,
                    target_pos=current_target,
                    way_plan=way_plan4,
                )
                print("way_start4:",way_plan4[0,:])
                print("way_end4:",way_plan4[-1,:])
                print("wpose4:",robot_state["tool_position"])
            # 重置积分项（避免前一目标的积分累积影响）
            integration_delta_pos = 0
            integration_delta_euler = 0

        # 所有目标完成，退出循环
        elif (target_error < target_error_threshold 
              and current_target_idx == len(target_order) - 1
              and td.h >= td.traj_N
              and error[1] < tracking_error_threshold
              ) or callback.button == 5:
            r.exec_gripper_cmd(0.016)
            time_str_init = title_str + "_%Y%m%d_%H%M%S"
            time_str = datetime.now().strftime(time_str_init)

            wb_master.save(
                str((make_log_path("file_traj_master.xlsx", time_str, root=root)))
            )
            wb_slave.save(
                str((make_log_path("file_traj_slave.xlsx", time_str, root=root)))
            )
            wb_reshape.save(
                str((make_log_path("file_traj_reshape.xlsx", time_str, root=root)))
            )
            wb_direct.save(
                str((make_log_path("file_traj_direct.xlsx", time_str, root=root)))
            )
            wb_robot.save(
                str((make_log_path("file_traj_robot.xlsx", time_str, root=root)))
            )
            wb_human.save(
                str((make_log_path("file_traj_human.xlsx", time_str, root=root)))
            )
            wb_joint.save(
                str((make_log_path("file_traj_joint.xlsx", time_str, root=root)))
            )
            wb_error.save(str((make_log_path("error.xlsx", time_str, root=root))))
            wb_force.save(str((make_log_path("force.xlsx", time_str, root=root))))
            wb_arbitrary.save(
                str((make_log_path("arbitrary.xlsx", time_str, root=root)))
            )
            wb_joint_torque.save(
                str((make_log_path("joint_torque.xlsx", time_str, root=root)))
            )
            wb_end_torque.save(
                str((make_log_path("end_torque.xlsx", time_str, root=root)))
            )
            wb_FVD.save(str((make_log_path("FVD.xlsx", time_str, root=root))))
            wb_dFVD.save(str((make_log_path("dFVD.xlsx", time_str, root=root))))
            wb_fuzzy.save(str((make_log_path("fuzzy.xlsx", time_str, root=root))))
            wb_posterior_probabilities.save(
                str(
                    (make_log_path("posterior_probabilities.xlsx", time_str, root=root))
                )
            )
            rospy.loginfo("所有目标点（a→b→c）已完成，退出控制循环")
            break

        td.h = td.h + 1
        if td.h < td.traj_N - 1:
            td.deform()

        zref_r_array = np.hstack(
            (
                td.x_d_curr_init,
                np.zeros(3),
                td.dot_x_d_curr_init,
                np.zeros(3),
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

        scale = 1
        willing = callback.HumanWilling()
        position_falcon = callback.position_falcon
        vel_falcon = callback.vel_falcon
        position_direct = willing * position_falcon * scale
        vel_direct = willing * vel_falcon * scale

        z_neu = np.hstack(
            (
                np.array(target_points[target_order[current_target_idx]].flatten()),
                np.zeros(9),
            )
        )
        zref_h_array_direct = (
            np.hstack(
                (
                    position_direct,
                    np.zeros(3),
                    vel_direct,
                    np.zeros(3),
                )
            )
        ) + zref_r_array

        DirectWilling_distance = np.linalg.norm(td.x_d_curr - td.x_d_curr_init)
        # print("DirectWilling_distance:", DirectWilling_distance)

        beta = callback.get_smoothed_beta_window(DirectWilling_distance)
        # print("beta:", beta)
        alpha = lambda_kf
        # print("alpha:", alpha)
        # beta = 0.0
        arbitrary = np.array([alpha, beta])

        zref_h_array = beta * zref_h_array_direct + (1 - beta) * zref_h_array_reshape

        position_rcm = robot_state["flange_position"] + (
            (
                (robot_state["tool_position"] - robot_state["flange_position"])
                @ (trocar_position - robot_state["flange_position"])
                * (robot_state["tool_position"] - robot_state["flange_position"])
            )
            / np.square(length)
        )

        Q_c = alpha * (Q_hh + Q_hr) + (1 - alpha) * (Q_rh + Q_rr)
        R_c = alpha * R_h + (1 - alpha) * R_r
        Q_h = alpha * Q_hh + (1 - alpha) * Q_hr
        Q_r = alpha * Q_rh + (1 - alpha) * Q_rr
        # K_gt, solve_P, E = ct.lqr(sysStateSpace, Q_c, R_c)

        K_gt = k_gt_database[round(alpha, 3)]

        ref_state = update_ref_state(
            zref_r_array,
            zref_h_array,
            Q_c,
            Q_h,
            Q_r,
            length,
            trocar_position,
            position_rcm,
            robot_state,
        )

        u_position, integration_delta_pos = update_u_position(
            ref_state, robot_state, K_gt, integration_delta_pos, dot_time
        )

        u_rotation, integration_delta_euler = update_u_rotation(
            ref_state, robot_state, integration_delta_euler, dot_time
        )

        force_falcon = -K_falcon * (
            td.x_d_curr_init - robot_state["tool_position"]
        ) - D_falcon * (td.dot_x_d_curr_init - robot_state["tool_position_velocity"])

        pub_force_falcon.x = force_falcon[0]
        pub_force_falcon.y = force_falcon[1]
        pub_force_falcon.z = force_falcon[2]

        Publisher_falconForce.publish(pub_force_falcon)

        u = np.hstack((u_position, u_rotation))
        if np.linalg.norm(u) > u_threshold:
            time_str_init = title_str + "_%Y%m%d_%H%M%S"
            time_str = datetime.now().strftime(time_str_init)
            wb_master.save(
                str((make_log_path("file_traj_master.xlsx", time_str, root=root)))
            )
            wb_slave.save(
                str((make_log_path("file_traj_slave.xlsx", time_str, root=root)))
            )
            wb_reshape.save(
                str((make_log_path("file_traj_reshape.xlsx", time_str, root=root)))
            )
            wb_direct.save(
                str((make_log_path("file_traj_direct.xlsx", time_str, root=root)))
            )
            wb_robot.save(
                str((make_log_path("file_traj_robot.xlsx", time_str, root=root)))
            )
            wb_human.save(
                str((make_log_path("file_traj_human.xlsx", time_str, root=root)))
            )
            wb_joint.save(
                str((make_log_path("file_traj_joint.xlsx", time_str, root=root)))
            )
            wb_error.save(str((make_log_path("error.xlsx", time_str, root=root))))
            wb_force.save(str((make_log_path("force.xlsx", time_str, root=root))))
            wb_arbitrary.save(
                str((make_log_path("arbitrary.xlsx", time_str, root=root)))
            )
            wb_joint_torque.save(
                str((make_log_path("joint_torque.xlsx", time_str, root=root)))
            )
            wb_end_torque.save(
                str((make_log_path("end_torque.xlsx", time_str, root=root)))
            )
            wb_FVD.save(str((make_log_path("FVD.xlsx", time_str, root=root))))
            wb_dFVD.save(str((make_log_path("dFVD.xlsx", time_str, root=root))))
            wb_fuzzy.save(str((make_log_path("fuzzy.xlsx", time_str, root=root))))
            wb_posterior_probabilities.save(
                str(
                    (make_log_path("posterior_probabilities.xlsx", time_str, root=root))
                )
            )
            rospy.loginfo(f"控制力过大,终止,u={u}")
            break
        Jacobian = np.array(kinematics_flange.jacobian())
        j_torque = np.dot(np.transpose(Jacobian), u).flatten()
        r.exec_torque_cmd(j_torque)
        angles = r.angles()

        error_rcm_distance = trocar_position - position_rcm
        error_rcm_distance_norm = np.linalg.norm(error_rcm_distance)
        error = np.array(
            [
                error_rcm_distance_norm,
                np.linalg.norm(
                    robot_state["tool_position"] - ref_state["tool_position"]
                ),
            ]
        )
        # print("error:", error)
        print(
            f"timepass: {timepass:.5f}, Goal: {target_order[current_target_idx]}, Error RCM: {error[0]:.5f}, Error Track: {error[1]:.5f}, 当前目标：{target_order[current_target_idx]}点，误差：{target_error:.6f}"
        )

        actions = zref_h_array[0:3] - ref_state["tool_position"]

        if (
            np.max(td.f_h_now) != 0.0 or np.min(td.f_h_now) != 0.0
        ) and current_target_idx == 2:
            likelihood_probabilities = goal_intent.rbii_2(
                robot_state["tool_position"], goal_positions, actions, kappa, Goal_beta
            )

            posterior_probabilities = goal_intent.bayesian_intent(
                likelihood_probabilities, transition_probabilities, posterior_init
            )
            order = np.argsort(posterior_probabilities)
            td.goal = goal_positions[order[-1]]

            if not np.array_equal(td.goal, td.old_goal) and callback.button == 0:
                print("New goal detected, replanning...")
                td.old_goal = td.goal
                current_target = goal_positions[order[-1]]
                target_points[target_order[2]] = current_target
                target_points[target_order[3]] = current_target - np.array(
                    [0.0, 0.0, 0.01]
                )
                # 重新规划轨迹到新目标
                movegroup5 = MoveGroupPythonInterfaceTutorial(r)
                way_plan5, quat_plan5, joint_plan5 = movegroup5.plan_path(
                    current_target
                )
                td = replan_to_target(
                    movegroup=movegroup,
                    kinematics_tool=kinematics_tool,
                    td_publishers=td_publishers,
                    freq=freq,
                    trocar_position=trocar_position,
                    length=length,
                    target_pos=current_target,
                    way_plan=way_plan5,
                )
                # 重置积分项（避免前一目标的积分累积影响）
                integration_delta_pos = 0
                integration_delta_euler = 0

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
            (error[0] >= init_error_threshold or error[1] >= init_error_threshold)
            and td.h == 0
            and flag_start == False
            and flag_action == False
        ):
            td.h = td.h - 1
            td.k = td.k - 1

        elif (
            error[0] < init_error_threshold
            and error[1] < init_error_threshold
            and td.h == 0
            and flag_start == False
            and flag_action == False
        ):
            flag_start = True
            # r.exec_gripper_cmd(0.05)

        if flag_start == True:
            flag_start = False
            flag_action = True
            print("angle:", angles)
            # input("Hit Enter to Start")
            # print("commanding")

            print("----------------Start Action----------------")

        if flag_action == True:
            ws_master.append(ref_state["tool_position"][0:6].flatten().tolist())
            ws_slave.append(robot_state["tool_position"][0:6].flatten().tolist())
            ws_error.append(error.flatten().tolist())
            ws_force.append(callback.force_falcon.flatten().tolist())
            ws_arbitrary.append(arbitrary.flatten().tolist())
            ws_reshape.append(zref_h_array_reshape[0:6].flatten().tolist())
            ws_direct.append(zref_h_array_direct[0:6].flatten().tolist())
            ws_robot.append(zref_r_array[0:6].flatten().tolist())
            ws_human.append(zref_h_array[0:6].flatten().tolist())
            ws_joint_torque.append(j_torque.flatten().tolist())
            ws_end_torque.append(u.flatten().tolist())
            ws_joint.append(angles.flatten().tolist())
            ws_FVD.append([F_h, T_h, D_r])
            ws_dFVD.append([dF_h, dT_h, dD_r])
            ws_fuzzy.append([lambda_w1, delta_lambda_w1])
            ws_posterior_probabilities.append(posterior_probabilities.flatten().tolist())

        rate.sleep()
