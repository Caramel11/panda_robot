import numpy as np
from scipy.linalg import expm, block_diag
from scipy.spatial.distance import cdist
from scipy.signal import butter, filtfilt
from panda_robot import PandaKinematics
from panda_robot import PandaArm
import time
import rospy, sys
from scipy.spatial.transform import Rotation
from openpyxl import Workbook
from arbitrary import (
    FuzzyFunction,
    CallBackFunction,
    TrajDeform,
    TrajDeformMoveit,
    GoalIntent,
    MoveGroupPythonInterfaceTutorial,
    FrankaSharedController,
    replan_to_target,
)

from geometry_msgs.msg import Point
import arbitrary as arb
from datetime import datetime
from kalman_filter import KalmanFilterFusion, DeltaLambdaUpdater
from fuzzy_logic import FuzzyLogicTool, Method2_IF_Else

root = "/home/liu/moveit_ws_20240902/logs/0127/0127_MPC_KF"  # log存储根目录
init_error_threshold = 0.005  # 初始化误差阈值
tracking_error_threshold = 0.015  # 跟踪误差阈值
target_error_threshold = 0.015  # 目标点误差阈值
title_str = "MPC_KF"  # 日志文件标题字符串
u_threshold = 30  # 控制输入阈值

# 使用示例
if __name__ == "__main__":
    rospy.init_node("panda_env")
    freq = 100
    dot_time = 1 / freq
    rate = rospy.Rate(freq)
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
    # r.set_joint_positions_velocities(angles_init, [0.0 for _ in range(7)])
    r.move_to_joint_position(
        (
            -0.03572926,
            -0.71236292,
            -0.05355629,
            -2.31286173,
            0.04212054,
            1.5332542,
            0.71300622,
        )
    )
    kinematics_tool = PandaKinematics(r, "panda_link10")
    kinematics_flange = PandaKinematics(r, "panda_link8")
    robot_state = arb.update_robot_state(kinematics_tool, kinematics_flange)
    group_name = "panda_arm"
    Publisher_TrajDeformPos = rospy.Publisher("/TrajDeform/Pos", Point, queue_size=2)
    Publisher_TrajDeformVel = rospy.Publisher("/TrajDeform/Vel", Point, queue_size=2)
    Publisher_InitTrajPos = rospy.Publisher("/InitTraj/Pos", Point, queue_size=2)
    Publisher_InitTrajVel = rospy.Publisher("/InitTraj/Vel", Point, queue_size=2)

    # 初始化控制器
    controller = FrankaSharedController(Np=10, dt=0.01)
    fuzzy_func = FuzzyFunction(r, kinematics_tool)
    callback = CallBackFunction()
    movegroup = MoveGroupPythonInterfaceTutorial(r)

    lambda_fuzzy = FuzzyLogicTool(type="lambda_based")
    delta_lambda_fuzzy = FuzzyLogicTool(type="delta_lambda_based")
    delta_updater = DeltaLambdaUpdater(lambda0=0.5, dt=dot_time)
    kf_fusion = KalmanFilterFusion(dt=dot_time, epsilon=0.01)

    target_points = {
        "a": np.array([0.3, 0.01, 0.007]),  # 目标a
        "b": np.array([0.3, 0.01, 0.03]),  # 目标b
        "c": np.array([0.3, 0.06, 0.03]),  # 目标c
        "d": np.array([0.3, 0.06, 0.02]),  # 目标d
    }
    object_position = np.array([0.3, -0.05, 0.05])
    object_radius = 0.03
    target_order = ["a", "b", "c", "d"]  # 执行顺序：a→b→c→d
    current_target_idx = 0  # 当前执行的目标点索引
    current_target = target_points[target_order[current_target_idx]]  # 初始目标：a点

    trocar_position = np.array([0.3, 0, 0.235])  # [GT Modify] slightly adjusted z
    length = 0.525  # [GT Modify] adjusted length
    flag = True

    way_plan, quat_plan, joint_plan = movegroup.plan_path(current_target)
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
    td_publishers = (
        Publisher_TrajDeformPos,
        Publisher_TrajDeformVel,
        Publisher_InitTrajPos,
        Publisher_InitTrajVel,
    )

    num_point = way_plan.shape[0]
    controller.reference_trajectory = way_plan

    # 环境设置
    obstacles = [np.array([0.7, 0.2, 0.3])]
    boundaries = [np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])]

    # 模拟控制循环
    state = np.hstack(
        (robot_state["flange_position"], robot_state["flange_position_velocity"])
    )

    time_end = time.perf_counter()
    time_start = time.perf_counter()
    tool_traj_ref = controller.reference_trajectory[0]
    i = 0

    # Logging setup
    wb_master = Workbook()
    ws_master = wb_master.active
    ws_master.append(
        ["master_x", "master_y", "master_z", "master_a", "master_b", "master_c"]
    )
    wb_slave = Workbook()
    ws_slave = wb_slave.active
    ws_slave.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_reshape = Workbook()
    ws_reshape = wb_reshape.active
    ws_reshape.append(
        ["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"]
    )
    wb_direct = Workbook()
    ws_direct = wb_direct.active
    ws_direct.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_robot = Workbook()
    ws_robot = wb_robot.active
    ws_robot.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_human = Workbook()
    ws_human = wb_human.active
    ws_human.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_joint = Workbook()
    ws_joint = wb_joint.active
    ws_joint.append(
        ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    )

    wb_error = Workbook()
    ws_error = wb_error.active
    ws_error.append(["error_rcm", "error_ee"])

    wb_force = Workbook()
    ws_force = wb_force.active
    ws_force.append(["master_x", "master_y", "master_z"])

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

    wb_PSI = Workbook()
    ws_PSI = wb_PSI.active
    ws_PSI.append(["PSI"])

    wb_deform = Workbook()
    ws_deform = wb_deform.active
    ws_deform.append(["slave_x", "slave_y", "slave_z", "slave_a", "slave_b", "slave_c"])

    wb_FVD = Workbook()  # 创建一个新的工作簿
    ws_FVD = wb_FVD.active  # 选择默认的工作表
    ws_FVD.append(["F_h", "T_h", "D_r"])

    wb_dFVD = Workbook()  # 创建一个新的工作簿
    ws_dFVD = wb_dFVD.active  # 选择默认的工作表
    ws_dFVD.append(["dF_h", "dT_h", "dD_r"])

    wb_fuzzy = Workbook()  # 创建一个新的工作簿
    ws_fuzzy = wb_fuzzy.active  # 选择默认的工作表
    ws_fuzzy.append(["lambda", "delta_lambda"])

    flag_action = False
    flag_start = False

    I_pos = 4
    I_ori = 10
    integration_position = 0
    integration_euler = 0

    # Initialize error for the first check loop
    error = np.array([0.0, 0.0])

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
    # input("Hit Enter to Start")
    # print("commanding")

    while not rospy.is_shutdown():
        time_end = time.perf_counter()
        timepass = time_end - time_start
        # print("timepass:", timepass)
        time_start = time.perf_counter()

        td.h = td.h + 1
        if td.h < td.traj_N - 1:
            td.deform(controller)

        robot_state = arb.update_robot_state(kinematics_tool, kinematics_flange)

        ###################### 计算our alpha ##################

        F_h = np.linalg.norm(callback.force_falcon)
        T_h = callback.force_time_total
        # if T_h > 0.5:
        #     print("callback.vel_falcon:", callback.vel_falcon)
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

        ################# 计算our alpha ##############

        # 计算当前tool到当前目标点的欧式距离（误差）
        target_error = np.linalg.norm(robot_state["tool_position"] - current_target)
        # print(
        #     f"当前目标：{target_order[current_target_idx]}点，误差：{target_error:.6f}"
        # )

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
                # 移动到 b 点前张开夹爪 (Target b logic)
                r.exec_gripper_cmd(0.045)
                # movegroup2 = MoveGroupPythonInterfaceTutorial(r)
                # wpose2 = movegroup.move_group.get_current_pose().pose
                # way_plan2, quat_plan2, joint_plan2 = movegroup2.plan_path(
                #     current_target
                # )
                current_target = tool_position_ref + np.array([0, 0, 0.02])

                num_point = int(
                    np.linalg.norm(tool_position_ref - current_target) / 0.00010
                )

                way_plan2 = np.linspace(tool_position_ref, current_target, num_point)

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
                i = 0  # Reset trajectory index for new target
                controller.reference_trajectory = way_plan2
                controller.current_trajectory = (
                    controller.reference_trajectory
                )  # 重置当前轨迹
                num_point = way_plan2.shape[0]
                print("way_start2:", way_plan2[0, :])
                print("way_end2:", way_plan2[-1, :])
                print("wpose2:", robot_state["tool_position"])

            elif current_target_idx == 2:
                # movegroup3 = MoveGroupPythonInterfaceTutorial(r)
                # wpose3 = movegroup.move_group.get_current_pose().pose
                # way_plan3, quat_plan3, joint_plan3 = movegroup3.plan_path(
                #     current_target
                # )

                num_point = int(
                    np.linalg.norm(tool_position_ref - current_target) / 0.00010
                )

                way_plan3 = np.linspace(tool_position_ref, current_target, num_point)

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
                i = 0  # Reset trajectory index for new target
                controller.reference_trajectory = way_plan3
                controller.current_trajectory = (
                    controller.reference_trajectory
                )  # 重置当前轨迹
                num_point = way_plan3.shape[0]
                print("way_start3:", way_plan3[0, :])
                print("way_end3:", way_plan3[-1, :])
                print("wpose3:", robot_state["tool_position"])

            elif current_target_idx == 3:
                # movegroup4 = MoveGroupPythonInterfaceTutorial(r)
                # wpose4 = movegroup.move_group.get_current_pose().pose
                # way_plan4, quat_plan4, joint_plan4 = movegroup4.plan_path(
                #     current_target
                # )
                current_target = tool_position_ref - np.array([0, 0, 0.02])

                num_point = int(
                    np.linalg.norm(tool_position_ref - current_target) / 0.00010
                )

                way_plan4 = np.linspace(tool_position_ref, current_target, num_point)
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
                i = 0  # Reset trajectory index for new target
                controller.reference_trajectory = way_plan4
                controller.current_trajectory = (
                    controller.reference_trajectory
                )  # 重置当前轨迹
                num_point = way_plan4.shape[0]
                print("way_start4:", way_plan4[0, :])
                print("way_end4:", way_plan4[-1, :])
                print("wpose4:", robot_state["tool_position"])

            # 重置积分项（避免前一目标的积分累积影响）
            integration_position = 0
            integration_euler = 0
        # [GT Modify] 所有目标点完成
        elif (
            target_error < target_error_threshold
            and current_target_idx == len(target_order) - 1
            and td.h >= td.traj_N
            and error[1] < tracking_error_threshold
        ) or callback.button == 5:
            r.exec_gripper_cmd(0.016)

            time_str_init = title_str + "_%Y%m%d_%H%M%S"
            time_str = datetime.now().strftime(time_str_init)
            wb_master.save(
                str((arb.make_log_path("file_traj_master.xlsx", time_str, root=root)))
            )
            wb_slave.save(
                str((arb.make_log_path("file_traj_slave.xlsx", time_str, root=root)))
            )
            wb_reshape.save(
                str((arb.make_log_path("file_traj_reshape.xlsx", time_str, root=root)))
            )
            wb_direct.save(
                str((arb.make_log_path("file_traj_direct.xlsx", time_str, root=root)))
            )
            wb_robot.save(
                str((arb.make_log_path("file_traj_robot.xlsx", time_str, root=root)))
            )
            wb_human.save(
                str((arb.make_log_path("file_traj_human.xlsx", time_str, root=root)))
            )
            wb_joint.save(
                str((arb.make_log_path("file_traj_joint.xlsx", time_str, root=root)))
            )
            wb_error.save(str((arb.make_log_path("error.xlsx", time_str, root=root))))
            wb_force.save(str((arb.make_log_path("force.xlsx", time_str, root=root))))
            wb_PSI.save(str((arb.make_log_path("PSI.xlsx", time_str, root=root))))
            wb_joint_torque.save(
                str((arb.make_log_path("joint_torque.xlsx", time_str, root=root)))
            )
            wb_end_torque.save(
                str((arb.make_log_path("end_torque.xlsx", time_str, root=root)))
            )
            wb_FVD.save(str((arb.make_log_path("FVD.xlsx", time_str, root=root))))
            wb_dFVD.save(str((arb.make_log_path("dFVD.xlsx", time_str, root=root))))
            wb_fuzzy.save(str((arb.make_log_path("fuzzy.xlsx", time_str, root=root))))

            print(f"✅ 所有目标点（a→b→c→d）执行完成，退出控制循环")
            break

        # 模拟交互力
        Fh = callback.force_falcon
        # Fh = np.array([0, 0, 0])
        # Fh = np.array([0, 0, 0.8]) if i > 300 and i < 400 else np.zeros(3)

        # 更新状态
        state = np.hstack(
            (robot_state["flange_position"], robot_state["flange_position_velocity"])
        )

        # 防止 i 超出范围
        if i >= len(controller.reference_trajectory):
            controller.current_trajectory = np.array(
                [controller.reference_trajectory[-1]]
            )
        else:
            controller.current_trajectory = controller.reference_trajectory[i:]

        # 计算控制输入

        alpha = lambda_kf
        # alpha = 0.5

        u_r = controller.compute_control(
            state, Fh, i, if_rcm=True, ws_PSI=ws_PSI, ws_deform=ws_deform, PSI=alpha
        )

        flange_position_ref = (
            alpha * controller.state_Xh + (1 - alpha) * controller.state_Xr
        )
        tool_position_ref = (
            alpha * controller.ref_h_output + (1 - alpha) * controller.ref_r_output
        )

        u_rcm, integration_euler = controller.compute_rcm(
            tool_position_ref, robot_state, integration_euler, (1 / freq)
        )

        goal_pos = flange_position_ref
        now_pos = robot_state["flange_position"]
        delta_pos = goal_pos - now_pos
        if np.linalg.norm(delta_pos) > 0.02:
            I_pos_now = I_pos / 2
        else:
            I_pos_now = I_pos

        integration_position = integration_position + delta_pos * (1 / freq)
        u_position = u_r + I_pos_now * integration_position
        u = np.hstack((u_position, u_rcm))
        if np.linalg.norm(u) > u_threshold:
            time_str_init = title_str + "_%Y%m%d_%H%M%S"
            time_str = datetime.now().strftime(time_str_init)

            wb_master.save(
                str((arb.make_log_path("file_traj_master.xlsx", time_str, root=root)))
            )
            wb_slave.save(
                str((arb.make_log_path("file_traj_slave.xlsx", time_str, root=root)))
            )
            wb_reshape.save(
                str((arb.make_log_path("file_traj_reshape.xlsx", time_str, root=root)))
            )
            wb_direct.save(
                str((arb.make_log_path("file_traj_direct.xlsx", time_str, root=root)))
            )
            wb_robot.save(
                str((arb.make_log_path("file_traj_robot.xlsx", time_str, root=root)))
            )
            wb_human.save(
                str((arb.make_log_path("file_traj_human.xlsx", time_str, root=root)))
            )
            wb_joint.save(
                str((arb.make_log_path("file_traj_joint.xlsx", time_str, root=root)))
            )
            wb_error.save(str((arb.make_log_path("error.xlsx", time_str, root=root))))
            wb_force.save(str((arb.make_log_path("force.xlsx", time_str, root=root))))
            wb_PSI.save(str((arb.make_log_path("PSI.xlsx", time_str, root=root))))
            wb_joint_torque.save(
                str((arb.make_log_path("joint_torque.xlsx", time_str, root=root)))
            )
            wb_end_torque.save(
                str((arb.make_log_path("end_torque.xlsx", time_str, root=root)))
            )
            wb_FVD.save(str((arb.make_log_path("FVD.xlsx", time_str, root=root))))
            wb_dFVD.save(str((arb.make_log_path("dFVD.xlsx", time_str, root=root))))
            wb_fuzzy.save(str((arb.make_log_path("fuzzy.xlsx", time_str, root=root))))
            rospy.loginfo(f"控制力过大,终止,u={u}")
            break

        Jacobian = np.array(kinematics_flange.jacobian())

        j_torque = np.dot(np.transpose(Jacobian), u).flatten()
        r.exec_torque_cmd(j_torque)

        position_rcm = robot_state["flange_position"] + (
            (
                (robot_state["tool_position"] - robot_state["flange_position"])
                @ (controller.trocar_position - robot_state["flange_position"])
                * (robot_state["tool_position"] - robot_state["flange_position"])
            )
            / np.square(controller.length)
        )

        error_rcm_distance = controller.trocar_position - position_rcm
        error_rcm_distance_norm = np.linalg.norm(error_rcm_distance)

        # 更新误差，用于下一轮循环判断
        error = np.array(
            [
                error_rcm_distance_norm,
                np.linalg.norm(robot_state["tool_position"] - tool_position_ref),
            ]
        )
        # print(
        #     f"Goal: {target_order[current_target_idx]}, Error RCM: {error[0]:.5f}, Error Track: {error[1]:.5f}"
        # )
        print(
            f"timepass: {timepass:.5f}, Goal: {target_order[current_target_idx]}, Error RCM: {error[0]:.5f}, Error Track: {error[1]:.5f}, 当前目标：{target_order[current_target_idx]}点，误差：{target_error:.6f},alpha: {alpha}, Fh: {Fh},u: {u}"
        )

        # [GT Modify] 启动逻辑调整 (threshold 0.002 -> 0.0002 kept from MPC, compatible with GT logic)
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

        if flag_start == True:
            flag_start = False
            flag_action = True
            i = i + 1
            # input("Hit Enter to Start")
            # print("commanding")
            print("------------Start Action------------")

        if flag_start == False and flag_action == True and i < num_point - 1:
            i = i + 1

        if flag_action == True:
            ws_master.append(tool_position_ref.flatten().tolist())
            ws_slave.append(robot_state["tool_position"].flatten().tolist())
            ws_error.append(error.flatten().tolist())
            ws_force.append(Fh.flatten().tolist())
            ws_robot.append(controller.ref_r_output.flatten().tolist())
            ws_human.append(controller.ref_h_output.flatten().tolist())
            ws_joint_torque.append(j_torque.flatten().tolist())
            ws_end_torque.append(u.flatten().tolist())
            ws_PSI.append([alpha])
            angles = r.angles()
            ws_joint.append(angles.flatten().tolist())
            ws_FVD.append([F_h, T_h, D_r])
            ws_dFVD.append([dF_h, dT_h, dD_r])
            ws_fuzzy.append([lambda_w1, delta_lambda_w1])

        rate.sleep()
