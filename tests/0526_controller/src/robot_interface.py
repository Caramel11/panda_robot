"""
机器人接口公共模块
====================

所有与机器人交互的底层操作:
  - 状态读取 (FK + Jacobian)
  - 安全控制器切换 (effort ↔ position)
  - RCM 几何计算
  - 姿态 PD+I 控制
  - 力矩装配 (u_cart → τ)
"""
import numpy as np
import rospy
from scipy.spatial.transform import Rotation


INIT_JOINTS = (
    -0.03572926, -0.71236292, -0.05355629,
    -2.31286173,  0.04212054,  1.5332542,   0.71300622,
)


# ================================================================
# 机器人状态读取
# ================================================================
def update_robot_state(kin_tool, kin_flange):
    """读取 tool 和 flange 的位姿与速度。

    no-RCM 控制主要使用 tool_position/tool_position_velocity；
    flange 字段保留给 RCM 版本和共享接口使用。
    """
    tp = kin_tool.forward_position_kinematics()
    tool_pos = np.array(tp[:3], dtype=np.float64)
    tq = (tp[3], tp[4], tp[5], tp[6])
    tool_euler = Rotation.from_quat(tq).as_euler("xyz", degrees=False)
    tv = np.array(list(kin_tool.forward_velocity_kinematics()))

    fp = kin_flange.forward_position_kinematics()
    flange_pos = np.array(fp[:3], dtype=np.float64)
    fq = (fp[3], fp[4], fp[5], fp[6])
    flange_euler = Rotation.from_quat(fq).as_euler("xyz", degrees=False)
    fv = np.array(list(kin_flange.forward_velocity_kinematics()))

    return {
        "tool_position": tool_pos,
        "tool_rotation_euler": tool_euler,
        "tool_position_velocity": tv[:3],
        "tool_rotation_euler_velocity": tv[3:6],
        "flange_position": flange_pos,
        "flange_rotation_euler": flange_euler,
        "flange_position_velocity": fv[:3],
        "flange_rotation_euler_velocity": fv[3:6],
    }


# ================================================================
# 安全控制器切换
# ================================================================
def safe_move_to_joint_position(robot, joints, timeout=10.0):
    """effort → position 控制器切换的安全包装。

    先发送零力矩、停止运动控制器，再尝试 move_to_joint_position。
    如果 MoveIt/控制器调用失败，则退回到简单位置插值，保证实验结束时能回初始位姿。
    """
    try:
        robot.exec_torque_cmd([0.0] * 7)
    except Exception:
        pass
    rospy.sleep(0.3)

    try:
        mgr = robot._ctrl_manager
        for c in mgr.list_active_controllers(only_motion_controllers=True):
            try:
                mgr.stop_controller(c)
            except Exception:
                pass
        rospy.sleep(0.3)
    except Exception:
        pass

    try:
        robot.move_to_joint_position(joints, timeout=timeout)
    except Exception:
        tgt = np.array(joints)
        r = rospy.Rate(100)
        for _ in range(int(timeout * 100)):
            if rospy.is_shutdown():
                break
            cur = np.array(robot.angles())
            if np.linalg.norm(tgt - cur) < 0.01:
                break
            robot.exec_position_cmd((cur + 0.01 * (tgt - cur)).tolist())
            r.sleep()


# ================================================================
# RCM 几何
# ================================================================
def compute_position_rcm(tool_pos, flange_pos, trocar_pos):
    """器械轴线上距 trocar 最近的点 (RCM 投影点)。

    no-RCM 主入口不会使用该函数，但它保留在共享 robot_interface 中供 RCM
    控制代码复用。
    """
    v = tool_pos - flange_pos
    w = trocar_pos - flange_pos
    v_sq = np.dot(v, v)
    if v_sq < 1e-10:
        return flange_pos.copy()
    return flange_pos + (np.dot(v, w) / v_sq) * v


def tool_to_flange_full_ref(x_tool_ref, xdot_tool_ref, trocar_pos, length):
    """
    从 tool 期望位置/速度正向映射到 flange 期望位置/速度/姿态

    几何关系 (RCM 约束):
      x_f = x_t + L · n̂,         n̂ = (p_trocar − x_t) / r,  r = ‖p_trocar − x_t‖

    速度由对时间求导得 (推导见文档):
      ẋ_f = (I − (L/r)(I − n̂ n̂ᵀ)) · ẋ_t

      其中 (I − n̂ n̂ᵀ) 为垂直工具轴方向的投影算子。
      物理含义:
        - 沿工具轴 (轴向) 的 ẋ_t → flange 同方向同速
        - 垂直工具轴 (横向) 的 ẋ_t → flange 反向、按 (L/r − 1) 倍缩放
          (这是 trocar 杠杆的几何效应)

    姿态: 工具轴方向 z_dir = −n̂ (从 trocar 指向工具尖端)
    选定 y_ref = [0,−1,0] 为参考方向, Gram-Schmidt 构造正交基。

    Parameters
    ----------
    x_tool_ref    : (3,)   tool 期望位置
    xdot_tool_ref : (3,)   tool 期望速度
    trocar_pos    : (3,)   trocar 固定点
    length        : float  工具长度 L

    Returns
    -------
    x_flange_ref    : (3,)   flange 期望位置
    xdot_flange_ref : (3,)   flange 期望速度
    euler_ref       : (3,)   flange 期望姿态 (xyz Euler)
    """
    d = trocar_pos - x_tool_ref
    r = np.linalg.norm(d)
    if r < 1e-6:
        # 退化情况: tool 在 trocar 上, 几何不定
        return x_tool_ref.copy(), np.zeros(3), np.zeros(3)

    n_hat = d / r

    # 位置
    x_flange_ref = x_tool_ref + length * n_hat

    # 速度: 投影算子映射
    P_perp = np.eye(3) - np.outer(n_hat, n_hat)
    M_vel = np.eye(3) - (length / r) * P_perp
    xdot_flange_ref = M_vel @ xdot_tool_ref

    # 姿态
    z_dir = -n_hat
    y_ref = np.array([0.0, -1.0, 0.0])
    y_dir = y_ref - np.dot(y_ref, z_dir) * z_dir
    y_norm = np.linalg.norm(y_dir)
    y_dir = y_dir / y_norm if y_norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    x_dir = np.cross(y_dir, z_dir)
    R_mat = np.column_stack([x_dir, y_dir, z_dir])
    euler_ref = Rotation.from_matrix(R_mat).as_euler("xyz", degrees=False)

    return x_flange_ref, xdot_flange_ref, euler_ref


def fixed_downward_tool_euler(y_ref=None):
    """
    构造 no-RCM 模式的固定末端姿态参考。

    实机/仿真当前竖直向下初始姿态读取约为
    [-1.553, 0.040, 0.029]，因此 no-RCM 固定参考取其理想化形式
    [-pi/2, 0, 0]。y_ref 参数保留为兼容旧调用，不参与计算。
    """
    return np.array([-0.5 * np.pi, 0.0, 0.0], dtype=float)


def euler_angle_diff(a, b):
    """带周期性处理的欧拉角差"""
    diff = b - a
    wrapped = diff % (2 * np.pi)
    wrapped[wrapped > np.pi] -= 2 * np.pi
    return wrapped


# ================================================================
# 姿态 PD+I 控制
# ================================================================
def compute_u_rotation(robot_state_euler, robot_state_euler_vel,
                       ref_euler, integ_euler, dt,
                       P_ori=20.0, D_ori=1.0, I_ori=30.0,
                       integ_limit=0.15):
    """独立于博弈框架的姿态控制"""
    e_euler = euler_angle_diff(robot_state_euler, ref_euler)
    e_euler_dot = euler_angle_diff(robot_state_euler_vel, np.zeros(3))
    integ_new = integ_euler + e_euler * dt
    if integ_limit is not None:
        integ_new = np.clip(integ_new, -float(integ_limit), float(integ_limit))
    u_rot = P_ori * e_euler + D_ori * e_euler_dot + I_ori * integ_new
    return u_rot, integ_new


# ================================================================
# 力矩装配 — RCM 模式 (flange-space 控制)
# ================================================================
def compute_torque_with_rcm(ctrl, robot_state, kin_flange,
                            x_tool_ref, xdot_tool_ref,
                            e_f, sigma_f,
                            alpha, K_e_hat,
                            trocar_pos, length,
                            integ_euler, dt):
    """
    RCM 模式 (flange-space 控制)

    控制流程:
      1. tool 期望 (x_tool_ref, ẋ_tool_ref) → RCM 正映射
         → flange 期望 (x_flange_ref, ẋ_flange_ref, euler_ref)
      2. flange 实测 − flange 期望 → e_r1_flange, e_r2_flange
      3. 博弈控制律 u_flange = -K_eff·[e_r1_flange; e_r2_flange; e_f; σ_f]
         注: 力相关项 e_f, σ_f 不做杠杆变换, 直接累加到 u_flange
      4. u_flange + u_rot → J_flange^T → τ

    与旧方案的差异:
      旧: tool 误差 → u_tool → 杠杆缩放 → u_flange
      新: tool 期望 → flange 期望 → flange 误差 → u_flange (无杠杆缩放)

    Parameters
    ----------
    ctrl          : CooperativeGameController
    robot_state   : dict       update_robot_state() 返回值
    kin_flange    : Kinematics flange (panda_link8) 运动学
    x_tool_ref    : (3,)       tool 期望位置
    xdot_tool_ref : (3,)       tool 期望速度
    e_f           : (3,)       力误差 F_meas − F_des (tool 空间)
    sigma_f       : (3,)       力误差泄漏积分 (tool 空间)
    alpha         : float      仲裁参数
    K_e_hat       : float      RLS 估计环境刚度
    trocar_pos    : (3,)       trocar 固定点
    length        : float      工具长度
    integ_euler   : (3,)       姿态积分项
    dt            : float      控制周期

    Returns
    -------
    j_torque        : (7,)
    u_flange        : (3,)     flange 笛卡尔控制力
    K_eff           : (4,)     当前激活增益
    integ_euler_new : (3,)
    x_flange_ref    : (3,)     flange 期望位置 (用于日志)
    error           : (2,)     [RCM 误差, tool 跟踪误差]
    """
    tool_pos = robot_state["tool_position"]
    flange_pos = robot_state["flange_position"]
    flange_vel = robot_state["flange_position_velocity"]

    # 1. tool 期望 → flange 期望 (位置、速度、姿态)
    x_flange_ref, xdot_flange_ref, euler_ref = tool_to_flange_full_ref(
        x_tool_ref, xdot_tool_ref, trocar_pos, length
    )

    # 2. flange 空间的位置/速度误差
    e_r1_flange = flange_pos - x_flange_ref
    e_r2_flange = flange_vel - xdot_flange_ref

    # 3. 博弈控制律 (flange-space, 力误差直接累加无杠杆变换)
    u_flange, K_eff = ctrl.compute_control(
        e_r1_flange, e_r2_flange, e_f, sigma_f, alpha, K_e_hat
    )

    # 4. 姿态控制 (flange 姿态)
    u_rot, integ_euler_new = compute_u_rotation(
        robot_state["flange_rotation_euler"],
        robot_state["flange_rotation_euler_velocity"],
        euler_ref, integ_euler, dt,
        P_ori=ctrl.P_ori, D_ori=ctrl.D_ori, I_ori=ctrl.I_ori,
    )

    # 5. 装配 wrench + 限幅
    u_cart = np.hstack([u_flange, u_rot])
    cart_norm = np.linalg.norm(u_cart)
    if cart_norm > ctrl.u_threshold:
        u_cart = u_cart * ctrl.u_threshold / cart_norm

    # 6. Jacobian → 关节力矩
    J = np.array(kin_flange.jacobian())
    j_torque = np.clip(
        (J.T @ u_cart).flatten(),
        -ctrl.tau_max, ctrl.tau_max,
    )

    # 7. 误差日志
    p_rcm = compute_position_rcm(tool_pos, flange_pos, trocar_pos)
    error_rcm = np.linalg.norm(trocar_pos - p_rcm)
    error_track = np.linalg.norm(tool_pos - x_tool_ref)

    return (j_torque, u_flange, K_eff, integ_euler_new,
            x_flange_ref, np.array([error_rcm, error_track]))


# ================================================================
# 力矩装配 — 无 RCM 模式
# ================================================================
def compute_torque_no_rcm(ctrl, robot_state, kin_tool,
                          e_r1, e_r2, e_f, sigma_f,
                          alpha, K_e_hat,
                          ref_euler_fixed, integ_euler, dt):
    """no-RCM 力矩装配函数。

    输入误差均在 tool 坐标对应的笛卡尔空间中表达。函数内部先调用
    ctrl.compute_control(...) 得到 tool 端笛卡尔力 u_tool，再用 tool Jacobian
    映射到关节力矩，并叠加姿态保持项。
    """
    tool_pos = robot_state["tool_position"]

    # 1. 平动控制: 由当前 alpha/K_hat 查表得到 K_eff，并计算 tool 端笛卡尔力。
    u_tool, K_eff = ctrl.compute_control(e_r1, e_r2, e_f, sigma_f, alpha, K_e_hat)
    if hasattr(ctrl, "no_rcm_u_tool_limits"):
        # 可选的三轴限幅钩子，当前控制器默认没有设置该属性。
        limits = np.asarray(ctrl.no_rcm_u_tool_limits, dtype=float)
        if limits.shape == (3,):
            u_tool = np.clip(u_tool, -limits, limits)

    # 姿态保持: 固定 tool frame 竖直向下，并保持 y 轴朝向不变
    u_rot, integ_euler_new = compute_u_rotation(
        robot_state["tool_rotation_euler"],
        robot_state["tool_rotation_euler_velocity"],
        ref_euler_fixed, integ_euler, dt,
        P_ori=ctrl.P_ori, D_ori=ctrl.D_ori, I_ori=ctrl.I_ori
    )

    # 2. 将平动力和姿态力矩拼成 6D wrench，并做整体范数限幅。
    u_cart = np.hstack([u_tool, u_rot])
    cart_norm = np.linalg.norm(u_cart)
    if cart_norm > ctrl.u_threshold:
        u_cart = u_cart * ctrl.u_threshold / cart_norm

    # 3. tool Jacobian 映射到关节力矩；no-RCM 不做 trocar 杠杆变换。
    J = np.array(kin_tool.jacobian())
    j_torque = np.clip(
        (J.T @ u_cart).flatten(),
        -ctrl.tau_max, ctrl.tau_max,
    )

    error_track = np.linalg.norm(e_r1)
    return (j_torque, u_tool, K_eff, integ_euler_new,
            np.array([0.0, error_track]))
