#!/usr/bin/env python3
"""第4章共享控制 Gazebo 控制器。

该节点是第4章实验使用的在线控制器。它保留已经验证过的 ROS2/Gazebo effort
接口和任务空间跟踪回路，在其上增加第4章的参考层共享控制逻辑：

1. 生成机器人自主参考 ``x_r``。
2. 生成或接收人类输入增量，并形成直接遥操作参考 ``x_tele``。
3. 通过 ``beta`` 平滑人侧候选参考，得到 ``x_h``。
4. 将 ``x_h`` 投影到力安全区间，得到 ``x_h_safe``。
5. 根据人类意图和安全裕度计算 ``alpha_HR``。
6. 组合最终工具端期望参考 ``x_d``。
7. 使用已有任务空间 effort 控制器跟踪 ``x_d``。

代码刻意把“参考层仲裁”和“执行层力矩控制”分开：论文实验主要比较不同仲裁策略，
机器人接口和底层执行控制保持固定。
"""

import json
import os
from datetime import datetime
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .arbitration import SharedFuzzyArbitrator, compute_gt_gain, sigmoid_alpha
from .pinocchio_model import PinocchioModelHelper

try:
    from ch3_experiments.no_rcm_force_position import NoRcmContinuousForceMarginExecution
except Exception:  # pragma: no cover - optional until ch3_experiments is built.
    NoRcmContinuousForceMarginExecution = None

try:
    from real_env_sim.realistic_profile import RealisticEnvModel, default_profile
except Exception:  # pragma: no cover - keeps the controller usable before the optional package is built.
    RealisticEnvModel = None
    default_profile = None

try:
    from real_env_sim.realistic_human_input import RealisticHumanInputGenerator
except Exception:  # pragma: no cover - optional until real_env_sim is built.
    RealisticHumanInputGenerator = None


INIT_JOINTS = np.array(
    [-0.03572926, -0.71236292, -0.05355629, -2.31286173, 0.04212054, 1.5332542, 0.71300622],
    dtype=float,
)


def smoothstep(s):
    """三次平滑插值核，两端斜率为 0。"""
    s = float(np.clip(s, 0.0, 1.0))
    return s * s * (3.0 - 2.0 * s)


def so3_error(R_des, R):
    """SO(3) 姿态误差，返回三维旋转向量。"""
    return pin.log3(R_des @ R.T)


def clamp_norm(vec, limit):
    """限制向量模长，同时保持方向不变。"""
    vec = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(vec))
    if limit <= 0.0 or norm <= limit or norm < 1e-12:
        return vec
    return vec * (float(limit) / norm)


def sat01(value):
    """把标量限制到 [0, 1] 区间。"""
    return float(np.clip(value, 0.0, 1.0))


class TrajectoryDeformer0213:
    """0213 ROS1 版本的交互力轨迹变形器。

    该类移植 ``TrajDeformMoveit(flag=True)`` 中的核心轨迹变形逻辑：
    对未来 ``tau`` 秒的参考轨迹窗口施加由交互力驱动的平滑形变，并把形变写回
    ``x_star_now``，因此人手释放后机器人继续沿已经变形的轨迹执行。
    """

    def __init__(self, traj, dt, tau=3.0, mu=0.01):
        self.delta = float(dt)
        self.tau = float(tau)
        self.mu = float(mu)
        self.x_star = np.asarray(traj, dtype=float).copy()
        if self.x_star.ndim != 2 or self.x_star.shape[1] != 3 or self.x_star.shape[0] < 2:
            raise ValueError("TrajectoryDeformer0213 requires an N x 3 trajectory with N >= 2")
        self.traj_N = int(self.x_star.shape[0])
        self.N = int(self.tau / self.delta + 1)
        self.N = max(self.N, 4)

        A = (
            np.vstack((np.eye(self.N), np.zeros((3, self.N))))
            + np.vstack((np.zeros((1, self.N)), -3.0 * np.eye(self.N), np.zeros((2, self.N))))
            + np.vstack((np.zeros((2, self.N)), 3.0 * np.eye(self.N), np.zeros((1, self.N))))
            + np.vstack((np.zeros((3, self.N)), -1.0 * np.eye(self.N)))
        )
        R = A.T @ A
        B = np.zeros((4, self.N))
        B[0, 0] = 1.0
        B[1, 1] = 1.0
        B[2, self.N - 2] = 1.0
        B[3, self.N - 1] = 1.0
        R_inv = np.linalg.inv(R)
        G = (np.eye(self.N) - R_inv @ B.T @ np.linalg.inv(B @ R_inv @ B.T) @ B) @ R_inv @ np.ones((self.N, 3))
        self.H = np.zeros((self.N, 3))
        for i in range(3):
            denom = max(float(np.linalg.norm(G[:, i])), 1e-12)
            self.H[:, i] = np.sqrt(self.N) * G[:, i] / denom

        self.x_star_now = self.x_star.copy()
        self.dot_x_star = np.zeros_like(self.x_star)
        self.dot_x_star[1:] = (self.x_star[1:] - self.x_star[:-1]) / self.delta
        self.k = -1
        self.x_d_curr = self.x_star[0].copy()
        self.x_d_next = self.x_star[1].copy()
        self.x_d_curr_init = self.x_star[0].copy()
        self.x_d_next_init = self.x_star[1].copy()
        self.dot_x_d_curr = self.dot_x_star[0].copy()
        self.dot_x_d_curr_init = self.dot_x_star[0].copy()
        self.last_force = np.zeros(3)
        self.last_deformation_norm = 0.0

    def step(self, interaction_force):
        """推进一个采样点，返回 0213 中的原始参考和变形参考。"""

        self.k = min(self.k + 1, self.traj_N - 1)
        force = np.asarray(interaction_force, dtype=float).reshape(3)
        self.last_force = force.copy()

        self.x_d_curr_init = self.x_star[self.k].copy()
        next_i = min(self.k + 1, self.traj_N - 1)
        self.x_d_next_init = self.x_star[next_i].copy()
        self.dot_x_d_curr_init = (self.x_d_next_init - self.x_d_curr_init) / self.delta

        if np.max(force) == 0.0 and np.min(force) == 0.0:
            self.x_d_curr = self.x_star_now[self.k].copy()
            self.x_d_next = self.x_star_now[next_i].copy()
            self.dot_x_d_curr = (self.x_d_next - self.x_d_curr) / self.delta
        else:
            end_i = min(self.k + self.N, self.traj_N)
            gamma = self.x_star_now[self.k:end_i, :]
            force_diag = np.diag(force)
            gamma_new = gamma + self.mu * self.delta * self.H[: gamma.shape[0], :] @ force_diag
            self.x_d_curr = gamma_new[0].copy()
            self.x_d_next = gamma_new[1].copy() if gamma_new.shape[0] > 1 else gamma_new[0].copy()
            self.dot_x_d_curr = (self.x_d_next - self.x_d_curr) / self.delta
            self.x_star_now[self.k:end_i, :] = gamma_new

        self.last_deformation_norm = float(np.linalg.norm(self.x_d_curr - self.x_d_curr_init))
        return {
            "nominal": self.x_d_curr_init.copy(),
            "nominal_vel": self.dot_x_d_curr_init.copy(),
            "reshape": self.x_d_curr.copy(),
            "reshape_vel": self.dot_x_d_curr.copy(),
            "deformation_norm": self.last_deformation_norm,
        }


def fixed_forward_tool_rotation():
    """历史 no-RCM 备用姿态：竖直向下，正面接近世界 +x。"""
    return pin.rpy.rpyToMatrix(*np.deg2rad([-90.0, 0.0, -45.0]))


def ustc_polyline_offsets(task_mode="no_rcm"):
    """生成连续 USTC 字母曲线的局部轨迹点。

    字母曲线位于任务切向平面内，z 方向保持轻微恒定压入参考。轨迹采用连续
    折线而非断笔字符，目的是在 Gazebo 中形成可执行的接触扫描任务，同时使
    轨迹形状足够复杂，能够观察控制器在拐角、连接段和多方向切向运动中的
    收敛性能。
    """

    # no-RCM 任务允许稍大切向覆盖；with-RCM 模式保守缩放，避免工具轴线
    # 几何约束与大范围横移同时激发。
    scale = 0.72 if str(task_mode) == "with_rcm" else 1.0
    w = 0.018 * scale
    h = 0.032 * scale
    gap = 0.005 * scale
    # no-RCM 字母轨迹沿用轻微 z 偏置来形成接触代理；with-RCM 的真实环境
    # 代理已经根据实际工具位置和人类法向输入生成接触力。若继续给 RCM
    # 字母轨迹加入正 z 偏置，会在实验开始就被解释为数毫米压入，导致
    # RCM 安全参考门直接暂停，表现为“稳定但不扫描”。
    z = 0.0 if str(task_mode) == "with_rcm" else 0.010 * scale
    pts = []

    def add(x, y):
        """执行本模块中的辅助计算或数据转换步骤。"""
        pts.append([x, y - 0.5 * h, z])

    # U：左上、左下、右下、右上。
    x0 = 0.0
    add(x0, h)
    add(x0, 0.0)
    add(x0 + w, 0.0)
    add(x0 + w, h)

    # S：从右上进入，经过上横、中横、下横。
    x0 = x0 + w + gap
    add(x0 + w, h)
    add(x0, h)
    add(x0, 0.5 * h)
    add(x0 + w, 0.5 * h)
    add(x0 + w, 0.0)
    add(x0, 0.0)

    # T：顶部横线后回到中点下行。
    x0 = x0 + w + gap
    add(x0, h)
    add(x0 + w, h)
    add(x0 + 0.5 * w, h)
    add(x0 + 0.5 * w, 0.0)

    # C：右上进入，绕左侧到右下。
    x0 = x0 + w + gap
    add(x0 + w, h)
    add(x0, h)
    add(x0, 0.0)
    add(x0 + w, 0.0)
    return np.asarray(pts, dtype=float)


SMOOTH_LETTER_SCENARIOS = {
    "ustc_smooth_u": "u",
    "ustc_smooth_s": "s",
    "ustc_smooth_t": "t",
    "ustc_smooth_c": "c",
}

STIFFNESS_LINE_SCENARIOS = {
    "stiffness_line",
    "straight_stiffness",
    "line_stiffness",
    "ch3_stiffness_line",
}


def _line_points(p0, p1, n=36):
    """生成一段稠密线段，用于构造圆滑字母描边轨迹。"""

    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    s = np.linspace(0.0, 1.0, max(int(n), 2))
    return (1.0 - s[:, None]) * p0[None, :] + s[:, None] * p1[None, :]


def _arc_points(center, radius, theta0, theta1, n=80):
    """生成椭圆弧点。"""

    cx, cy = center
    rx, ry = radius
    theta = np.linspace(float(theta0), float(theta1), max(int(n), 3))
    return np.column_stack([cx + rx * np.cos(theta), cy + ry * np.sin(theta)])


def _catmull_rom_chain(points, samples_per_segment=28):
    """Catmull-Rom 样条，用于把字母控制点变成平滑单笔轨迹。"""

    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.copy()
    padded = np.vstack([points[0], points, points[-1]])
    out = []
    for i in range(1, len(padded) - 2):
        p0, p1, p2, p3 = padded[i - 1], padded[i], padded[i + 1], padded[i + 2]
        ts = np.linspace(0.0, 1.0, max(int(samples_per_segment), 2), endpoint=False)
        t2 = ts * ts
        t3 = t2 * ts
        seg = 0.5 * (
            (2.0 * p1)
            + (-p0 + p2) * ts[:, None]
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2[:, None]
            + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3[:, None]
        )
        out.append(seg)
    out.append(points[-1][None, :])
    return np.vstack(out)


def _dedupe_xy(points, eps=1e-9):
    """删除相邻重复点，避免弧长采样出现零长度线段。"""

    points = np.asarray(points, dtype=float)
    keep = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i] - points[keep[-1]]) > eps:
            keep.append(i)
    return points[keep]


def ustc_smooth_letter_offsets(scenario, task_mode="no_rcm", trajectory_scale=None):
    """生成图 4.2 风格的 U/S/T/C 单字母圆滑描边期望轨迹。

    童康论文图 4.2 将 U、S、T、C 四种目标轨迹分别作为跟踪任务。本函数
    采用相同思路，把每个字母作为一个独立的连续描边轨迹。轨迹位于工具
    切向平面；with-RCM 模式只缩放切向范围，避免在固定孔约束下额外引入
    法向参考偏置。
    """

    letter = SMOOTH_LETTER_SCENARIOS.get(str(scenario), str(scenario).lower().replace("ustc_smooth_", ""))
    # with-RCM 轨迹必须满足固定穿刺点几何约束。历史稳定的第三章 RCM
    # Gazebo 验证采用 10 mm 量级局部扫描；这里保守缩放字母轨迹，避免
    # 大横移在虚拟 trocar 杠杆下直接放大为法兰姿态和 RCM 误差振荡。
    if trajectory_scale is not None and float(trajectory_scale) > 0.0:
        scale = float(trajectory_scale)
    else:
        scale = 0.24 if str(task_mode) == "with_rcm" else 1.0
    a = 0.026 * scale
    b = 0.032 * scale
    z = 0.0 if str(task_mode) == "with_rcm" else 0.010 * scale

    if letter == "u":
        left = _line_points([-a, b], [-a, -0.25 * b], 42)
        bottom = _arc_points([0.0, -0.25 * b], [a, 0.75 * b], np.pi, 2.0 * np.pi, 96)
        right = _line_points([a, -0.25 * b], [a, b], 42)
        xy = np.vstack([left, bottom, right])
    elif letter == "s":
        # 从右上到左下的单笔 S 形轨迹。样条控制点形成上下两个圆滑弯折。
        ctrl = np.array([
            [0.82 * a, b],
            [-0.78 * a, 0.82 * b],
            [-0.78 * a, 0.18 * b],
            [0.76 * a, 0.02 * b],
            [0.78 * a, -0.70 * b],
            [-0.82 * a, -b],
        ])
        xy = _catmull_rom_chain(ctrl, samples_per_segment=42)
    elif letter == "t":
        # 顶部横划和中部竖划组成的单笔 T 形轨迹，转折由 sample_polyline 降速处理。
        # 这里不加入额外底部回转，保持与图 4.2 中目标 T 形轨迹相近，避免把
        # 字母描边任务变成局部小闭环跟踪任务。
        top = _line_points([-a, b], [a, b], 54)
        back = _line_points([a, b], [0.0, b], 30)
        stem = _line_points([0.0, b], [0.0, -b], 92)
        xy = np.vstack([top, back, stem])
    elif letter == "c":
        xy = _arc_points([0.0, 0.0], [a, b], 0.25 * np.pi, 1.75 * np.pi, 132)
    else:
        xy = np.array([[0.0, 0.0], [0.0, 0.75 * b], [0.75 * a, 0.75 * b]], dtype=float)

    xy = _dedupe_xy(xy)
    # 物理海绵实验中，字母的“向下”方向与机器人 +x 同向，字母左右方向
    # 与机器人 y 轴同向。因此刚度分界线 local y=0 正好对应字母竖直中轴线；
    # y>0 为软区，y<0 为硬区。
    xy = xy - xy[0]
    robot_x = -xy[:, 1]
    robot_y = xy[:, 0]
    pts = np.column_stack([robot_x, robot_y, np.full(len(xy), z)])
    return pts.astype(float)


def stiffness_line_offsets(task_mode="no_rcm", trajectory_scale=None):
    """Generate a short straight scan across the two-sponge stiffness boundary.

    The real-like sponge profile defines local y=0 as the material boundary:
    y>0 is the 300 N/m soft block and y<0 is the 600 N/m stiff block.  The
    line is intentionally short so Chapter 3 comparison and ablation methods
    can be checked quickly in Gazebo while still crossing the stiffness jump.
    """

    scale = float(trajectory_scale) if trajectory_scale and float(trajectory_scale) > 0.0 else 1.0
    half_y = (0.030 if str(task_mode) == "with_rcm" else 0.042) * scale
    z = 0.0 if str(task_mode) == "with_rcm" else 0.010 * scale
    return np.array(
        [
            [0.000, half_y, z],
            [0.000, 0.50 * half_y, z],
            [0.000, 0.000, z],
            [0.000, -0.50 * half_y, z],
            [0.000, -half_y, z],
        ],
        dtype=float,
    )


def sample_polyline(points, t, duration):
    """沿折线按弧长采样，并使用 smoothstep 平滑每个线段端点。"""

    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points[0].copy(), np.zeros(3)
    duration = max(float(duration), 1e-6)
    seg = points[1:] - points[:-1]
    seg_len = np.linalg.norm(seg, axis=1)
    valid = seg_len > 1e-9
    seg = seg[valid]
    seg_len = seg_len[valid]
    starts = points[:-1][valid]
    if len(seg_len) == 0:
        return points[0].copy(), np.zeros(3)
    total_len = float(np.sum(seg_len))
    if t >= duration:
        return points[-1].copy(), np.zeros(3)
    s_len = float(np.clip(t / duration, 0.0, 1.0)) * total_len
    cum = np.cumsum(seg_len)
    idx = int(np.searchsorted(cum, s_len, side="right"))
    idx = int(np.clip(idx, 0, len(seg_len) - 1))
    prev = float(cum[idx - 1]) if idx > 0 else 0.0
    raw = float(np.clip((s_len - prev) / max(seg_len[idx], 1e-9), 0.0, 1.0))
    h = smoothstep(raw)
    dh = 6.0 * raw * (1.0 - raw)
    pos = starts[idx] + h * seg[idx]
    # 弧长匀速，线段内部用 smoothstep 降低拐角处速度突变。
    dlen_dt = total_len / duration
    vel = dh * dlen_dt * seg[idx] / max(seg_len[idx], 1e-9)
    return pos, vel


def tool_visual_front_axis_local():
    """fr3_link11/tool frame 下用于判断可视正面的近似局部轴。"""
    return np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5)], dtype=float)


def front_axis_error_deg(R_current, R_ref):
    """计算工具可视正面轴与期望正面轴之间的夹角。

    该指标专门用于排查历史上出现过的末端左转 45/90 度和腕部自转抖动问题。
    它不等同于完整 SO(3) 姿态误差，而是直接评价“正面是否朝前”。
    """

    front_local = tool_visual_front_axis_local()
    dot = float(np.clip(np.dot(R_current @ front_local, R_ref @ front_local), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def interpolate_rotation(R_start, R_target, s):
    """SO(3) 插值，用于平滑进入正面朝前姿态，避免姿态阶跃激发腕部自转。"""
    s = float(np.clip(s, 0.0, 1.0))
    return pin.exp3(s * pin.log3(R_target @ R_start.T)) @ R_start


def tool_to_flange_ref(tool_ref, tool_vel_ref, trocar, length, forward_ref=None):
    """在 RCM 几何约束下，把工具尖端参考转换为法兰参考。

    with-RCM 模式下，任务参考定义在工具尖端，但控制对象是法兰位姿。trocar 被视为
    器械轴线上的虚拟固定点。该函数重构法兰位置和相容姿态，使法兰到工具尖端的轴线
    经过 trocar。
    """
    d = np.asarray(trocar, dtype=float) - np.asarray(tool_ref, dtype=float)
    r = float(np.linalg.norm(d))
    if r < 1e-9:
        return np.asarray(tool_ref, dtype=float), np.zeros(3), np.eye(3)
    n_hat = d / r
    flange_ref = np.asarray(tool_ref, dtype=float) + float(length) * n_hat
    p_perp = np.eye(3) - np.outer(n_hat, n_hat)
    flange_vel_ref = (np.eye(3) - (float(length) / r) * p_perp) @ np.asarray(tool_vel_ref, dtype=float)

    z_dir = -n_hat
    # RCM 约束确定工具轴线后，绕轴自转仍有一维自由度。这里用世界 +x
    # 作为可视正面参考来确定该自由度，避免继承旧实现的左前 45 度偏航。
    if forward_ref is None:
        forward_ref = np.array([1.0, 0.0, 0.0])
    forward_ref = np.asarray(forward_ref, dtype=float)
    if float(np.linalg.norm(forward_ref)) < 1e-9:
        forward_ref = np.array([1.0, 0.0, 0.0])
    forward_ref = forward_ref / max(float(np.linalg.norm(forward_ref)), 1e-9)
    x_dir = forward_ref - np.dot(forward_ref, z_dir) * z_dir
    x_norm = float(np.linalg.norm(x_dir))
    if x_norm < 1e-6:
        x_dir = np.array([0.0, -1.0, 0.0])
    else:
        x_dir = x_dir / x_norm
    y_dir = np.cross(z_dir, x_dir)
    y_dir = y_dir / max(np.linalg.norm(y_dir), 1e-9)
    x_dir = x_dir / max(np.linalg.norm(x_dir), 1e-9)
    R_ref = np.column_stack([x_dir, y_dir, z_dir])
    return flange_ref, flange_vel_ref, R_ref


def rcm_point_on_axis(tool_pos, flange_pos, trocar):
    """计算当前工具轴线上距离 trocar 最近的点。"""
    axis = np.asarray(tool_pos, dtype=float) - np.asarray(flange_pos, dtype=float)
    denom = float(np.dot(axis, axis))
    if denom < 1e-12:
        return np.asarray(flange_pos, dtype=float)
    return np.asarray(flange_pos, dtype=float) + (
        np.dot(axis, np.asarray(trocar, dtype=float) - np.asarray(flange_pos, dtype=float)) / denom
    ) * axis


class DataLogger:
    """实验脚本使用的轻量 NPZ 数据记录器。

    向量会被展开为 ``tool_ref_0``、``tool_ref_1`` 这类字段。这样保存的数据可以直接
    被 NumPy、绘图脚本和第4章指标重算脚本读取。
    """

    def __init__(self):
        """初始化对象参数和运行状态。"""
        self._d = {}

    @property
    def count(self):
        """返回当前已记录的数据点数量。"""
        return len(self._d.get("t", []))

    def log(self, **kwargs):
        """执行本模块中的辅助计算或数据转换步骤。"""
        for key, value in kwargs.items():
            arr = np.asarray(value)
            if arr.ndim == 0:
                self._d.setdefault(key, []).append(float(arr))
            else:
                flat = arr.ravel()
                for i, val in enumerate(flat):
                    self._d.setdefault(f"{key}_{i}", []).append(float(val))

    def arrays(self):
        """执行本模块中的辅助计算或数据转换步骤。"""
        return {k: np.asarray(v) for k, v in self._d.items()}

    def save(self, path):
        """把分析结果、图表索引或时序数据写入磁盘文件。"""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **self.arrays())


class SharedGTGazeboNode(Node):
    """实现第4章共享 GT 控制器的 ROS2 节点。"""

    PHASE_WAIT = 0
    PHASE_HOME = 1
    PHASE_INIT = 2
    PHASE_TRACK = 3
    PHASE_DONE = 4

    def __init__(self):
        """初始化对象参数和运行状态。"""
        super().__init__("shared_gt_gazebo_node")
        # 实验开关：对比脚本只改变这些参考层参数，Gazebo 机器人接口和 effort
        # 控制器 topic 保持不变。
        self.declare_parameter("enable_control", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("task_mode", "with_rcm")
        self.declare_parameter("controller_variant", "gt_kf")
        self.declare_parameter("arbitration_strategy", "full_method")
        self.declare_parameter("execution_strategy", "fixed_05")
        self.declare_parameter("execution_alpha_profile", "stable")
        self.declare_parameter("scenario", "mixed_sequence")
        self.declare_parameter("trajectory_scale", 0.0)
        self.declare_parameter("fixed_alpha_hr", 0.5)
        self.declare_parameter("fixed_alpha_fp", 0.5)
        self.declare_parameter("alpha_fp_tau_s", 0.35)
        self.declare_parameter("force_desired", 0.7)
        self.declare_parameter("force_min", 0.35)
        self.declare_parameter("force_max", 1.2)
        self.declare_parameter("force_margin", 0.08)
        self.declare_parameter("contact_stiffness_hat", 70.0)
        self.declare_parameter("kappa_min", 0.05)
        self.declare_parameter("realistic_env_enabled", False)
        self.declare_parameter("realistic_profile", "real_env_0607_0213")
        self.declare_parameter("realistic_seed", 20260703)
        self.declare_parameter("realistic_tau_noise_gain", 0.12)
        self.declare_parameter("realistic_use_measured_force_for_metrics", True)
        self.declare_parameter("human_input_profile", "smoothstep")
        self.declare_parameter("external_human_topic", "/ch5/human_delta")
        self.declare_parameter("external_human_timeout_s", 0.25)
        self.declare_parameter("external_human_scale", 1.0)
        self.declare_parameter("external_human_max_delta_m", 0.025)
        # ROS/Gazebo 接口和机器人模型配置。
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("cmd_topic", "/no_rcm_effort_controller/commands")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("xacro_path", "")
        self.declare_parameter("robot_type", "fr3")
        self.declare_parameter("tool_frame", "fr3_link11")
        self.declare_parameter("flange_frame", "fr3_link8")
        self.declare_parameter("reference_frame", "fr3_link8")
        self.declare_parameter("gravity_compensation_scale", 0.0)
        self.declare_parameter("use_current_as_home", True)
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("max_tau_abs", [55.0, 55.0, 55.0, 55.0, 18.0, 14.0, 10.0])
        self.declare_parameter("max_tau_rate", 220.0)
        self.declare_parameter("joint_damping", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("joint_damping_tau_max", 3.5)
        self.declare_parameter("joint_damping_filter_tau_s", 0.08)
        self.declare_parameter("wrist_posture_enabled", False)
        self.declare_parameter("wrist_posture_deadband_rad", 0.045)
        self.declare_parameter("wrist_posture_kp", [0.0, 0.0, 0.0, 0.0, 0.0, 0.08, 0.28])
        self.declare_parameter("wrist_posture_kd", [0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.20])
        self.declare_parameter("wrist_posture_tau_max", [0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.85])
        self.declare_parameter("home_hold_s", 1.0)
        self.declare_parameter("orientation_ramp_s", 5.0)
        self.declare_parameter("orientation_init_tol_deg", 3.0)
        self.declare_parameter("orientation_init_timeout_s", 7.0)
        self.declare_parameter("orientation_kp", 10.0)
        self.declare_parameter("orientation_kd", 3.0)
        self.declare_parameter("orientation_ki", 0.0)
        self.declare_parameter("orientation_omega_filter_tau_s", 0.06)
        self.declare_parameter("orientation_omega_limit", 1.2)
        self.declare_parameter("orientation_u_rot_limit", 4.0)
        self.declare_parameter("orientation_u_rot_rate_limit", 24.0)
        self.declare_parameter("task_duration_s", 24.0)
        self.declare_parameter("target_tolerance_m", 0.003)
        self.declare_parameter("output_dir", "/home/liu/franka_ros2_ws/results/shared_gt_gazebo")
        self.declare_parameter("controlled_joints", [f"fr3_joint{i}" for i in range(1, 8)])

        # 启动时一次性读取参数。实验设计为可重复的固定试验，不需要运行时动态改参。
        self.enable_control = bool(self.get_parameter("enable_control").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.task_mode = str(self.get_parameter("task_mode").value)
        self.controller_variant = str(self.get_parameter("controller_variant").value)
        self.arbitration_strategy = str(self.get_parameter("arbitration_strategy").value)
        self.execution_strategy = str(self.get_parameter("execution_strategy").value)
        self.execution_alpha_profile = str(self.get_parameter("execution_alpha_profile").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.trajectory_scale = float(self.get_parameter("trajectory_scale").value)
        self.fixed_alpha_hr = float(self.get_parameter("fixed_alpha_hr").value)
        self.fixed_alpha_fp = float(self.get_parameter("fixed_alpha_fp").value)
        self.alpha_fp_tau_s = float(self.get_parameter("alpha_fp_tau_s").value)
        self.force_desired = float(self.get_parameter("force_desired").value)
        self.force_min = float(self.get_parameter("force_min").value)
        self.force_max = float(self.get_parameter("force_max").value)
        self.force_margin = float(self.get_parameter("force_margin").value)
        self.contact_stiffness_hat = float(self.get_parameter("contact_stiffness_hat").value)
        self.kappa_min = float(self.get_parameter("kappa_min").value)
        self.realistic_env_enabled = bool(self.get_parameter("realistic_env_enabled").value)
        self.realistic_profile = str(self.get_parameter("realistic_profile").value)
        self.realistic_seed = int(self.get_parameter("realistic_seed").value)
        self.realistic_tau_noise_gain = float(self.get_parameter("realistic_tau_noise_gain").value)
        self.realistic_use_measured_force_for_metrics = bool(
            self.get_parameter("realistic_use_measured_force_for_metrics").value
        )
        self.human_input_profile = str(self.get_parameter("human_input_profile").value)
        self.external_human_timeout_s = float(self.get_parameter("external_human_timeout_s").value)
        self.external_human_scale = float(self.get_parameter("external_human_scale").value)
        self.external_human_max_delta_m = float(self.get_parameter("external_human_max_delta_m").value)
        self.ctrl_joints = list(self.get_parameter("controlled_joints").value)
        self.dt = 1.0 / float(self.get_parameter("control_rate_hz").value)
        self.gravity_scale = float(self.get_parameter("gravity_compensation_scale").value)
        self.max_tau_abs = np.asarray(self.get_parameter("max_tau_abs").value, dtype=float)
        self.max_tau_rate = float(self.get_parameter("max_tau_rate").value)
        self.joint_damping = np.asarray(self.get_parameter("joint_damping").value, dtype=float)
        self.joint_damping_tau_max = float(self.get_parameter("joint_damping_tau_max").value)
        self.joint_damping_filter_tau_s = float(self.get_parameter("joint_damping_filter_tau_s").value)
        self.wrist_posture_enabled = bool(self.get_parameter("wrist_posture_enabled").value)
        self.wrist_posture_deadband_rad = float(self.get_parameter("wrist_posture_deadband_rad").value)
        self.wrist_posture_kp = np.asarray(self.get_parameter("wrist_posture_kp").value, dtype=float)
        self.wrist_posture_kd = np.asarray(self.get_parameter("wrist_posture_kd").value, dtype=float)
        self.wrist_posture_tau_max = np.asarray(self.get_parameter("wrist_posture_tau_max").value, dtype=float)
        self.home_hold_s = float(self.get_parameter("home_hold_s").value)
        self.orientation_ramp_s = float(self.get_parameter("orientation_ramp_s").value)
        self.orientation_init_tol_deg = float(self.get_parameter("orientation_init_tol_deg").value)
        self.orientation_init_timeout_s = float(self.get_parameter("orientation_init_timeout_s").value)
        self.orientation_kp = float(self.get_parameter("orientation_kp").value)
        self.orientation_kd = float(self.get_parameter("orientation_kd").value)
        self.orientation_ki = float(self.get_parameter("orientation_ki").value)
        self.orientation_omega_filter_tau_s = float(self.get_parameter("orientation_omega_filter_tau_s").value)
        self.orientation_omega_limit = float(self.get_parameter("orientation_omega_limit").value)
        self.orientation_u_rot_limit = float(self.get_parameter("orientation_u_rot_limit").value)
        self.orientation_u_rot_rate_limit = float(self.get_parameter("orientation_u_rot_rate_limit").value)
        self.task_duration_s = float(self.get_parameter("task_duration_s").value)
        self.target_tolerance_m = float(self.get_parameter("target_tolerance_m").value)
        if self.realistic_env_enabled:
            self.target_tolerance_m = max(self.target_tolerance_m, 0.010)
        self.output_dir = str(self.get_parameter("output_dir").value)

        # 在线机器人状态和有限状态机状态。
        self.q = np.zeros(7)
        self.qd = np.zeros(7)
        self.have_state = False
        self.home_joints = INIT_JOINTS.copy()
        self.tau_prev = np.zeros(7)
        self.qd_damping_filtered = np.zeros(7)
        self.phase = self.PHASE_WAIT
        self.phase_start = self.get_clock().now()
        self.t0 = self.get_clock().now()
        self.task_start_time = None

        # Pinocchio 提供坐标系位姿、雅可比和可选重力项。控制器主体不直接处理
        # URDF/Xacro 细节。
        self.pm = PinocchioModelHelper(
            self,
            controlled_joints=self.ctrl_joints,
            tool_frame=str(self.get_parameter("tool_frame").value),
            flange_frame=str(self.get_parameter("flange_frame").value),
            robot_type=str(self.get_parameter("robot_type").value),
        )
        self.model_ready = self.pm.load()

        # 参考层状态：模糊/KF alpha_HR、beta 平滑、外部人类输入缓存，以及运行时初始化
        # 的局部任务几何。
        self.arbitrator = SharedFuzzyArbitrator(dt=self.dt)
        self.logger_npz = DataLogger()
        self.integral_pos = np.zeros(3)
        self.integral_rot = np.zeros(3)
        self.force_feedback_control = float(self.force_desired)
        self.angular_velocity_filtered = np.zeros(3)
        self.u_rot_prev = np.zeros(3)
        self.prev_Fh = 0.0
        self.prev_Th = 0.0
        self.prev_Dr = 0.08
        self.human_time = 0.0
        self.beta = 0.0
        self.beta_window = []
        self.beta_window_size = 5
        self.prev_human_delta = np.zeros(3)
        self.human_button_state = 0.0
        self.human_force_cmd_N = 0.0
        self.human_event_type_id = 0.0
        self.human_raw_force_N = np.zeros(3)
        self.human_delta_cmd = np.zeros(3)
        self.external_human_delta = np.zeros(3)
        self.external_human_stamp = None
        self.prev_nominal = None
        self.traj_deformer = None
        self.last_reshape_ref = None
        self.last_reshape_vel = np.zeros(3)
        self.last_direct_ref = None
        self.last_deformation_norm = 0.0

        self.base_tool = None
        self.base_flange = None
        self.base_tool_R = None
        self.base_flange_R = None
        self.forward_tool_R = fixed_forward_tool_rotation()
        self.forward_tool_rpy_deg = np.array([-90.0, 0.0, -45.0], dtype=float)
        self.with_rcm_forward_ref = np.array([1.0, 0.0, 0.0], dtype=float)
        self.nominal_wrist_joints = INIT_JOINTS.copy()
        self.trocar = None
        self.tool_length = None
        self.last_tool_ref = None
        self.rcm_limited_tool_ref = None
        self.rcm_limited_tool_vel = np.zeros(3)
        self.rcm_reference_scale = 1.0
        self.last_rcm_u = np.zeros(3)
        self.last_alpha = 0.5
        self.alpha_fp = float(np.clip(self.fixed_alpha_fp, 0.0, 1.0))
        self.no_rcm_execution = None
        execution_family = self._execution_family()
        if execution_family in ("dynamic_gt", "continuous_force_margin", "full_method", "fixed_02", "fixed_05", "fixed_08"):
            if NoRcmContinuousForceMarginExecution is None:
                self.get_logger().warning(
                    "continuous_force_margin requested, but ch3_experiments adapter is not importable; "
                    "falling back to compact alpha_FP proxy."
                )
            else:
                stiffness_profile = self.execution_alpha_profile
                if self._is_rcm_execution() and stiffness_profile == "stable":
                    stiffness_profile = "rcm_sensitive"
                fixed_alpha = None
                if execution_family == "fixed_02":
                    fixed_alpha = 0.2
                elif execution_family == "fixed_05":
                    fixed_alpha = 0.5
                elif execution_family == "fixed_08":
                    fixed_alpha = 0.8
                self.no_rcm_execution = NoRcmContinuousForceMarginExecution(
                    dt=self.dt,
                    force_desired=self.force_desired,
                    force_min=self.force_min,
                    force_max=self.force_max,
                    control_scale=1.0,
                    # realistic U/S/T/C sponge scans can excite saturation chatter
                    # when the force-position law is allowed to jump aggressively.
                    # Use the same conservative envelope for no-RCM and RCM
                    # integrated experiments; standard baselines keep their own
                    # PD/MPC envelopes and are not affected by this limit.
                    output_limit=26.0,
                    stiffness_profile=stiffness_profile,
                    fixed_alpha=fixed_alpha,
                )
        self.real_env = None
        self.real_human_input = None
        if self.realistic_env_enabled:
            if RealisticEnvModel is None or default_profile is None:
                self.get_logger().warning("realistic_env_enabled=true, but real_env_sim is not importable; disabling realism.")
                self.realistic_env_enabled = False
            else:
                profile = default_profile(self.realistic_profile)
                self.real_env = RealisticEnvModel(profile, seed=self.realistic_seed)
                if RealisticHumanInputGenerator is not None:
                    self.real_human_input = RealisticHumanInputGenerator(profile, seed=self.realistic_seed + 4096)

        # ROS 接口：力矩发布、关节状态订阅，以及用于第5章集成测试的可选实时人类输入。
        self.pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("cmd_topic").value),
            10,
        )
        self.sub = self.create_subscription(
            JointState,
            str(self.get_parameter("state_topic").value),
            self._on_joint_state,
            20,
        )
        self.external_human_sub = self.create_subscription(
            Point,
            str(self.get_parameter("external_human_topic").value),
            self._on_external_human_delta,
            20,
        )
        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            f"shared GT controller ready: task_mode={self.task_mode}, "
            f"variant={self.controller_variant}, strategy={self.arbitration_strategy}, "
            f"execution={self.execution_strategy}, alpha_profile={self.execution_alpha_profile}, scenario={self.scenario}, "
            f"enable={self.enable_control}, dry_run={self.dry_run}, "
            f"realistic_env={self.realistic_env_enabled}"
        )

    def _on_joint_state(self, msg):
        """从完整 ROS joint-state 消息中提取受控关节状态。"""
        name_to_i = {name: i for i, name in enumerate(msg.name)}
        for i, joint in enumerate(self.ctrl_joints):
            idx = name_to_i.get(joint)
            if idx is None:
                return
            if idx < len(msg.position):
                self.q[i] = float(msg.position[idx])
            self.qd[i] = float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0
        self.have_state = True

    def _on_external_human_delta(self, msg):
        """接收可选的实时人类位移指令。

        第4章 Gazebo 实验默认使用脚本人类输入以保证可重复性。同一个控制器也可以接收
        实时/外部输入；输入在使用前会限幅，避免异常发布者造成过大的参考跳变。
        """
        raw = np.array([msg.x, msg.y, msg.z], dtype=float) * self.external_human_scale
        self.external_human_delta = clamp_norm(raw, self.external_human_max_delta_m)
        self.external_human_stamp = self.get_clock().now()

    def _with_realistic_human_tremor(self, t, delta):
        """在脚本人类输入上叠加实物 Falcon/手部输入的有色小抖动。"""

        if not self.realistic_env_enabled or self.real_env is None:
            return np.asarray(delta, dtype=float)
        return clamp_norm(
            self.real_env.human_delta(t, np.asarray(delta, dtype=float)),
            self.external_human_max_delta_m,
        )

    def _button_envelope(self, t, start, duration, rise_s=0.18):
        """真实力反馈器按键式输入包络：快速上升、保持、松开后立即归零。"""

        t = float(t)
        start = float(start)
        duration = max(float(duration), self.dt)
        rise = max(min(float(rise_s), 0.45 * duration), self.dt)
        if t < start or t > start + duration:
            return 0.0
        if t < start + rise:
            return smoothstep((t - start) / rise)
        return 1.0

    def _apply_human_profile(self, t, delta, button=0.0):
        """按所选人输入模型叠加真实 tremor，并记录按钮状态和等效输入力。"""

        delta = np.asarray(delta, dtype=float)
        active = float(button > 0.5 or np.linalg.norm(delta) > 1e-9)
        self.human_button_state = active
        self.human_event_type_id = 0.0 if not active else self.human_event_type_id
        if self.real_env is not None:
            per_n = max(float(self.real_env.profile.human_delta_per_N_m), 1e-9)
            self.human_force_cmd_N = float(np.linalg.norm(delta) / per_n) if active else 0.0
        else:
            self.human_force_cmd_N = float(np.linalg.norm(delta) / 0.010) if active else 0.0
        if not active:
            self.human_raw_force_N = np.zeros(3)
            self.human_delta_cmd = np.zeros(3)
            return np.zeros(3)
        self.human_delta_cmd = delta.copy()
        if self.real_env is not None:
            per_n = max(float(self.real_env.profile.human_delta_per_N_m), 1e-9)
            self.human_raw_force_N = delta / per_n
        else:
            self.human_raw_force_N = delta / 0.010
        if self.realistic_env_enabled and self.real_env is not None:
            corrected = clamp_norm(self._with_realistic_human_tremor(t, delta), self.external_human_max_delta_m)
            self.human_delta_cmd = corrected.copy()
            return corrected
        corrected = clamp_norm(delta, self.external_human_max_delta_m)
        self.human_delta_cmd = corrected.copy()
        return corrected

    def _sample_realistic_human_event_input(self, t):
        """从真实统计事件模型生成一帧主端人输入。"""

        if self.real_human_input is None:
            delta, button = self._ustc_real_button_delta(t)
            return self._apply_human_profile(t, delta, button)
        sample = self.real_human_input.sample(
            float(t),
            task_duration_s=max(self.task_duration_s, self.dt),
            scenario=self.scenario,
        )
        self.human_button_state = float(sample.button)
        self.human_force_cmd_N = float(sample.force_cmd_N)
        self.human_event_type_id = float(sample.event_type_id)
        self.human_raw_force_N = np.asarray(sample.raw_force_N, dtype=float)
        self.human_delta_cmd = clamp_norm(np.asarray(sample.delta_m, dtype=float), self.external_human_max_delta_m)
        if self.human_button_state <= 0.5:
            self.human_delta_cmd = np.zeros(3)
        return self.human_delta_cmd.copy()

    def _ustc_real_button_delta(self, t):
        """USTC 字母轨迹的真实力反馈器按键式人输入脚本。

        真实数据库中的主端输入表现为：无交互时为零；按下 clutch/操纵按钮后
        在很短时间内升至峰值，保持一段时间；松开按钮后立即归零。这里使用
        `human_delta_per_N_m` 把真实峰值力映射为参考增量，并保留切向修正、
        短时扰动、危险法向压入和持续干预四类论文工况。
        """

        total = max(self.task_duration_s, 1e-6)
        peak_force = 1.32
        if self.real_env is not None:
            peak_force = float(self.real_env.profile.human_active_mean_N)
            delta_per_n = float(self.real_env.profile.human_delta_per_N_m)
        else:
            delta_per_n = 0.010
        amp = peak_force * delta_per_n

        events = [
            # start, duration, unit direction
            (0.20 * total, 0.14 * total, np.array([0.0, 1.00, 0.0])),
            (0.42 * total, 0.035 * total, np.array([0.0, -0.70, -0.30])),
            (0.50 * total, 0.14 * total, np.array([0.0, 0.0, -0.68])),
            (0.70 * total, 0.12 * total, np.array([0.76, -0.72, 0.0])),
        ]
        delta = np.zeros(3)
        button = 0.0
        for start, duration, direction in events:
            env = self._button_envelope(t, start, duration, rise_s=0.18)
            if env > 0.0:
                delta += amp * env * direction
                button = 1.0
        return delta, button

    def _realistic_channels(self, t, out, tau):
        """生成仅用于记录和论文分析的真实环境模拟测量通道。

        Gazebo 中没有真实六维力传感器。本函数优先使用控制器根据实际末端
        压入量计算的 ``y_f_actual_true``，再经过实物知识库中学习到的传感器噪声、
        切向耦合和慢漂模型，得到 ``F_measured/F_raw/Fx/Fy/Fz/Mx/My/Mz``。
        若旧数据流没有提供实际接触力代理，则回退到参考层预测力 ``y_f``。
        """

        if not self.realistic_env_enabled or self.real_env is None:
            return {}
        y_reference = float(out.get("y_f", self.force_desired))
        y_true = max(float(out.get("y_f_actual_true", y_reference)), 0.0)
        wrench = self.real_env.measure_wrench(y_true, t, contact=y_true > 1e-4)
        y_est = self.real_env.estimated_normal_force(wrench["F_measured"], y_true)
        tool_ref_for_region = np.asarray(out.get("tool_ref", np.zeros(3)), dtype=float)
        tool_for_region = np.asarray(out.get("tool_pos_actual", tool_ref_for_region), dtype=float)
        normal_down = float(
            out.get(
                "actual_normal_down_m",
                np.dot(tool_for_region - tool_ref_for_region, np.array([0.0, 0.0, -1.0])),
            )
        )
        penetration_proxy = max(0.0, normal_down)
        k_env = float(out.get("K_env_control", np.nan))
        b_env = float(out.get("B_env_control", np.nan))
        s_env = float(out.get("stiffness_transition_control_s", np.nan))
        if not np.isfinite(k_env) or not np.isfinite(b_env) or not np.isfinite(s_env):
            k_env, b_env, s_env = self._realistic_stiffness(tool_for_region, penetration_proxy)
        k_ref, b_ref, s_ref = self._realistic_stiffness(tool_ref_for_region, penetration_proxy)
        u_noise = self.real_env.u_tool_disturbance()
        return {
            "realistic_env_enabled": 1.0,
            "y_f_nominal": y_reference,
            "y_f_reference": y_reference,
            "y_f_actual_true": y_true,
            "force_feedback_raw": float(out.get("force_feedback_raw", y_true)),
            "force_feedback_clipped": float(out.get("force_feedback_clipped", y_true)),
            "force_feedback_control": float(out.get("force_feedback_control", y_true)),
            "y_f_realistic": y_est,
            "F_estimated": y_est,
            "F_measured": wrench["F_measured"],
            "F_raw": wrench["F_raw"],
            "F_true_normal": wrench["F_true_normal"],
            "Fx": wrench["Fx"],
            "Fy": wrench["Fy"],
            "Fz": wrench["Fz"],
            "Mx": wrench["Mx"],
            "My": wrench["My"],
            "Mz": wrench["Mz"],
            "K_env_realistic": k_env,
            "K_env_actual": k_env,
            "K_env_reference": k_ref,
            "B_env_realistic": b_env,
            "B_env_reference": b_ref,
            "stiffness_transition_s": s_env,
            "stiffness_transition_reference_s": s_ref,
            "sponge_local_y_m": float(out.get("sponge_local_y_actual_m", self._sponge_local_y(tool_for_region))),
            "sponge_local_y_actual_m": float(out.get("sponge_local_y_actual_m", self._sponge_local_y(tool_for_region))),
            "sponge_local_y_reference_m": float(self._sponge_local_y(tool_ref_for_region)),
            "actual_normal_down_m": normal_down,
            "u_tool_noise": u_noise,
            "tau_realistic_norm": float(np.linalg.norm(tau)),
            "wrench_noise_norm": wrench["wrench_noise_norm"],
        }

    def _sponge_local_y(self, tool_ref):
        """Return local y coordinate for the two-sponge USTC scene.

        The physical scene places the material boundary on the vertical
        centerline of the letter.  In the controller this is represented by
        local y=0 relative to the letter centerline: y>0 is the 300 N/m
        soft sponge and y<0 is the 600 N/m stiff sponge.
        """

        tool_ref = np.asarray(tool_ref, dtype=float)
        if self.base_tool is None:
            return float(tool_ref[1])
        y_offset = 0.0
        if self.scenario in SMOOTH_LETTER_SCENARIOS:
            offsets = ustc_smooth_letter_offsets(self.scenario, self.task_mode)
            if self.trajectory_scale > 0.0:
                offsets = ustc_smooth_letter_offsets(
                    self.scenario,
                    self.task_mode,
                    trajectory_scale=self.trajectory_scale,
                )
            y_offset = 0.5 * (float(np.min(offsets[:, 1])) + float(np.max(offsets[:, 1])))
        elif self.scenario in STIFFNESS_LINE_SCENARIOS:
            offsets = stiffness_line_offsets(
                self.task_mode,
                trajectory_scale=self.trajectory_scale if self.trajectory_scale > 0.0 else None,
            )
            y_offset = 0.5 * (float(np.min(offsets[:, 1])) + float(np.max(offsets[:, 1])))
        return float(tool_ref[1] - self.base_tool[1] - y_offset)

    def _realistic_stiffness(self, tool_ref, penetration_proxy):
        """Query the realistic environment with the right scan coordinate."""

        if self.real_env is None:
            return float(self.contact_stiffness_hat), 0.0, 0.0
        tool_ref = np.asarray(tool_ref, dtype=float)
        y_local = self._sponge_local_y(tool_ref)
        return self.real_env.stiffness(float(tool_ref[0]), penetration_proxy, y=y_local)

    def _execution_family(self, strategy=None):
        """Map copied RCM execution strategies back to their theory family."""

        s = str(strategy or self.execution_strategy)
        if s.endswith("_rcm"):
            s = s[:-4]
        return {
            "strong_force": "fixed_02",
            "strong_position": "fixed_08",
            "balanced_fixed": "fixed_05",
            "hybrid_force_position": "traditional_hybrid",
            "hybrid": "traditional_hybrid",
            "impedance": "standard_impedance",
            "mpc_qp": "standard_mpc",
            "mpc": "standard_mpc",
            "full_method": "continuous_force_margin",
        }.get(s, s)

    def _arbitration_family(self, strategy=None):
        """Map copied RCM arbitration strategies back to their theory family."""

        s = str(strategy or self.arbitration_strategy)
        if s.endswith("_rcm"):
            s = s[:-4]
        return s

    def _is_rcm_execution(self):
        return str(self.execution_strategy).endswith("_rcm")

    def _is_rcm_arbitration(self):
        return str(self.arbitration_strategy).endswith("_rcm")

    def _use_rcm_specific_control(self):
        return self.task_mode == "with_rcm" and (self._is_rcm_execution() or self._is_rcm_arbitration())

    def _actual_force_feedback(self, nominal, tool_pos, update_filter=True):
        """Convert actual tool penetration into the force feedback used by execution control.

        The reference layer predicts force from the commanded reference.  That is sufficient
        for safety projection, but it hides differences between execution-layer controllers in
        Chapter 5 comparisons.  For realistic sponge experiments, force feedback must depend on
        the actual tool position: downward deviation from the nominal scan surface increases the
        normal force according to the current 300/600 N/m sponge stiffness, while upward
        deviation reduces it.  The result is still bounded at zero to avoid non-physical tension.
        """

        nominal = np.asarray(nominal, dtype=float).reshape(3)
        tool_pos = np.asarray(tool_pos, dtype=float).reshape(3)
        n_down = np.array([0.0, 0.0, -1.0])
        normal_down = float(np.dot(tool_pos - nominal, n_down))
        penetration_proxy = max(0.0, normal_down)
        if self.realistic_env_enabled and self.real_env is not None:
            k_env, b_env, s_env = self._realistic_stiffness(tool_pos, penetration_proxy)
        else:
            k_env = float(self.contact_stiffness_hat)
            b_env = 0.0
            s_env = 0.0
        y_true = max(0.0, self.force_desired + float(k_env) * normal_down)
        feedback_upper = max(float(self.force_max) + 0.8, float(self.force_desired) + 0.8)
        y_control_target = float(np.clip(y_true, 0.0, feedback_upper))
        if not np.isfinite(self.force_feedback_control):
            self.force_feedback_control = float(self.force_desired)
        if update_filter:
            tau = 0.12
            alpha = self.dt / max(tau + self.dt, self.dt)
            candidate = self.force_feedback_control + alpha * (y_control_target - self.force_feedback_control)
            max_step = max(8.0 * self.dt, 0.01)
            self.force_feedback_control += float(
                np.clip(candidate - self.force_feedback_control, -max_step, max_step)
            )
        y_control = float(self.force_feedback_control)
        return {
            "force_feedback": y_control,
            "force_feedback_raw": float(y_true),
            "force_feedback_clipped": y_control_target,
            "force_feedback_control": y_control,
            "y_f_actual_true": float(y_true),
            "K_env_control": float(k_env),
            "B_env_control": float(b_env),
            "stiffness_transition_control_s": float(s_env),
            "actual_normal_down_m": float(normal_down),
            "sponge_local_y_actual_m": float(self._sponge_local_y(tool_pos)),
        }

    @staticmethod
    def _move_toward(current, target, max_step):
        """Rate-limit a Cartesian reference without changing its direction."""

        current = np.asarray(current, dtype=float).reshape(3)
        target = np.asarray(target, dtype=float).reshape(3)
        delta = target - current
        dist = float(np.linalg.norm(delta))
        if dist <= max_step or dist < 1e-12:
            return target.copy()
        return current + delta * (float(max_step) / dist)

    @staticmethod
    def _soft_slowdown(value, slow_at, pause_at):
        """Continuous slowdown envelope used by the stable Chapter-3 RCM task."""

        value = float(value)
        slow_at = float(slow_at)
        pause_at = float(pause_at)
        if value <= slow_at:
            return 1.0
        if value >= pause_at:
            return 0.0
        return float((pause_at - value) / max(pause_at - slow_at, 1e-9))

    def _with_rcm_limited_reference(self, target_ref, p_tool, p_flange, force_feedback):
        """Apply the stable RCM reference gate before computing flange control.

        The historical stable `run_with_rcm` controller did not let the desired
        tool point advance purely by wall-clock time.  It advanced only while
        tool tracking, RCM error and contact-force margins were acceptable.
        The shared Chapter-5 node now keeps that behavior for copied `*_rcm`
        methods, while no-RCM and non-RCM methods keep their original path.
        """

        target_ref = np.asarray(target_ref, dtype=float).reshape(3)
        if self.rcm_limited_tool_ref is None:
            self.rcm_limited_tool_ref = np.asarray(p_tool, dtype=float).reshape(3).copy()

        p_rcm = rcm_point_on_axis(p_tool, p_flange, self.trocar)
        rcm_error = float(np.linalg.norm(self.trocar - p_rcm))
        follow_error = float(np.linalg.norm(np.asarray(p_tool, dtype=float) - self.rcm_limited_tool_ref))
        force_abs = float(abs(force_feedback))

        speed_scale = min(
            self._soft_slowdown(follow_error, 0.0060, 0.0140),
            self._soft_slowdown(rcm_error, 0.0035, 0.0065),
            self._soft_slowdown(force_abs, self.force_max - 0.10, self.force_max + 0.10),
        )
        # The with-RCM letter task is intentionally slower than no-RCM.  This
        # matches the 0213-style trocar geometry: lateral tip motion creates a
        # levered flange motion, so aggressive time-based references are a
        # primary cause of instability.
        max_speed = 0.00135 if self.realistic_env_enabled else 0.00160
        prev = self.rcm_limited_tool_ref.copy()
        self.rcm_limited_tool_ref = self._move_toward(
            self.rcm_limited_tool_ref,
            target_ref,
            max_speed * speed_scale * self.dt,
        )
        self.rcm_limited_tool_vel = (self.rcm_limited_tool_ref - prev) / max(self.dt, 1e-9)
        self.rcm_reference_scale = float(speed_scale)
        return self.rcm_limited_tool_ref.copy(), self.rcm_limited_tool_vel.copy(), rcm_error

    def _rcm_projection_wrench(self, tool, flange):
        """Return a bounded Cartesian correction at the closest RCM point.

        This is the ROS2 counterpart of the 0213 idea: keep the point on the
        instrument axis closest to the trocar near the trocar by applying a
        small, damped correction through the corresponding interpolated
        Jacobian.  It is deliberately bounded so it assists the main
        flange-space controller instead of fighting the tool trajectory.
        """

        p_tool, _, Jv_tool, _, v_tool, _ = tool
        p_flange, _, Jv_flange, _, v_flange, _ = flange
        axis = np.asarray(p_tool, dtype=float) - np.asarray(p_flange, dtype=float)
        denom = float(np.dot(axis, axis))
        if denom < 1e-12:
            self.last_rcm_u = np.zeros(3)
            return np.zeros(7), np.zeros(3), 0.0
        s = float(np.clip(np.dot(axis, self.trocar - p_flange) / denom, 0.0, 1.0))
        p_rcm = p_flange + s * axis
        J_rcm = (1.0 - s) * Jv_flange + s * Jv_tool
        v_rcm = (1.0 - s) * v_flange + s * v_tool
        e_rcm = np.asarray(self.trocar, dtype=float) - p_rcm
        u_rcm = 520.0 * e_rcm - 48.0 * v_rcm
        u_rcm = clamp_norm(u_rcm, 4.8)
        self.last_rcm_u = u_rcm.copy()
        return J_rcm.T @ u_rcm, u_rcm, float(np.linalg.norm(e_rcm))

    def _elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _phase_elapsed(self):
        return (self.get_clock().now() - self.phase_start).nanoseconds * 1e-9

    def _set_phase(self, phase):
        self.phase = phase
        self.phase_start = self.get_clock().now()

    def _publish_effort(self, tau):
        """发布关节力矩；dry-run 模式下不实际发布。"""
        if self.dry_run:
            return
        msg = Float64MultiArray()
        msg.data = np.asarray(tau, dtype=float).reshape(7).tolist()
        self.pub.publish(msg)

    def _rate_and_clip(self, tau):
        """发布前施加任务力矩限幅、关节自转阻尼和最终变化率限制。"""
        tau = np.clip(np.asarray(tau, dtype=float), -self.max_tau_abs, self.max_tau_abs)
        tau = self._apply_joint_damping(tau)
        tau = np.clip(tau, -self.max_tau_abs, self.max_tau_abs)
        max_delta = self.max_tau_rate * self.dt
        tau = self.tau_prev + np.clip(tau - self.tau_prev, -max_delta, max_delta)
        tau = np.clip(tau, -self.max_tau_abs, self.max_tau_abs)
        self.tau_prev = tau.copy()
        return tau

    def _apply_joint_damping(self, tau):
        """Gazebo effort 后端的关节速度阻尼包络，重点抑制末端自转。"""
        tau = np.asarray(tau, dtype=float).reshape(7).copy()
        n = min(tau.size, self.qd.size, self.joint_damping.size)
        alpha = self.dt / max(self.joint_damping_filter_tau_s + self.dt, self.dt)
        self.qd_damping_filtered[:n] += alpha * (self.qd[:n] - self.qd_damping_filtered[:n])
        tau_damp = -self.joint_damping[:n] * self.qd_damping_filtered[:n]
        tau_damp = np.clip(
            tau_damp,
            -self.joint_damping_tau_max,
            self.joint_damping_tau_max,
        )
        tau[:n] += tau_damp
        tau += self._wrist_posture_tau()
        return tau

    def _wrist_posture_tau(self):
        """轻量腕部包络，抑制 q7 自转漂移但不替代笛卡尔姿态任务。"""
        tau = np.zeros(7)
        if not self.wrist_posture_enabled:
            return tau
        n = min(
            tau.size,
            self.q.size,
            self.qd_damping_filtered.size,
            self.nominal_wrist_joints.size,
            self.wrist_posture_kp.size,
            self.wrist_posture_kd.size,
            self.wrist_posture_tau_max.size,
        )
        if n <= 0:
            return tau
        q_err = self.nominal_wrist_joints[:n] - self.q[:n]
        deadband = max(self.wrist_posture_deadband_rad, 0.0)
        q_eff = np.sign(q_err) * np.maximum(np.abs(q_err) - deadband, 0.0)
        tau_raw = self.wrist_posture_kp[:n] * q_eff - self.wrist_posture_kd[:n] * self.qd_damping_filtered[:n]
        tau[:n] = np.clip(tau_raw, -self.wrist_posture_tau_max[:n], self.wrist_posture_tau_max[:n])
        return tau

    def _frame_states(self):
        """根据当前关节状态返回 Pinocchio 坐标系状态。"""
        q_full, qd_full = self.pm.build_full_state(self.q, self.qd)
        tool = self.pm.get_frame_state(q_full, qd_full, self.pm.tool_frame_id)
        flange = self.pm.get_frame_state(q_full, qd_full, self.pm.flange_frame_id)
        tau_g = self.gravity_scale * self.pm.gravity(q_full)
        return q_full, qd_full, tool, flange, tau_g

    def _hold_home_tau(self, tau_g):
        """任务开始前使用的关节空间 PD 保持控制。"""
        kp = np.array([42.0, 42.0, 42.0, 34.0, 14.0, 11.0, 7.0])
        kd = np.array([8.0, 8.0, 8.0, 7.0, 3.0, 2.4, 1.7])
        return tau_g - kp * (self.q - self.home_joints) - kd * self.qd

    def _init_task_geometry(self, tool, flange):
        """记录局部任务原点、姿态和虚拟 trocar。

        第4章所有参考都是相对实验开始时机器人位姿的局部偏移。即使 Gazebo 初始世界位姿
        有轻微差异，实验轨迹仍保持可重复。
        """
        p_tool, R_tool, _, _, _, _ = tool
        p_flange, R_flange, _, _, _, _ = flange
        self.base_tool = p_tool.copy()
        self.base_flange = p_flange.copy()
        self.base_tool_R = R_tool.copy()
        self.base_flange_R = R_flange.copy()
        self.forward_tool_R = fixed_forward_tool_rotation()
        self.forward_tool_rpy_deg = np.array([-90.0, 0.0, -45.0], dtype=float)
        self.nominal_wrist_joints = self.q.copy()
        axis = p_tool - p_flange
        length = float(np.linalg.norm(axis))
        if length < 1e-6:
            axis = np.array([0.0, 0.0, -1.0])
            length = 0.10
        self.tool_length = length
        # 将虚拟 trocar 放在当前工具轴线上，位置位于法兰和工具尖端之间。
        self.trocar = p_tool - 0.62 * axis
        self.last_tool_ref = self.base_tool.copy()
        self.rcm_limited_tool_ref = self.base_tool.copy()
        self.rcm_limited_tool_vel = np.zeros(3)
        self.last_rcm_u = np.zeros(3)
        # 0213 与第三章稳定 RCM 控制器只由 RCM 约束确定工具轴线；
        # 绕工具轴的正面自由度必须固定。这里使用启动时法兰 +x 轴作为
        # forward_ref，可避免 TRACK 阶段出现左转 45/90 deg 和腕部自转。
        self.with_rcm_forward_ref = R_flange[:, 0].copy()
        self.integral_pos[:] = 0.0
        self.integral_rot[:] = 0.0
        self.force_feedback_control = float(self.force_desired)
        if self.no_rcm_execution is not None:
            self.no_rcm_execution.reset()
        self.angular_velocity_filtered[:] = 0.0
        self.u_rot_prev[:] = 0.0
        self.beta = 0.0
        self.beta_window = []
        self.prev_human_delta[:] = 0.0
        self.prev_nominal = self.base_tool.copy()
        self._init_trajectory_deformer()
        self.get_logger().info(
            f"Initialized local {self.task_mode} task: tool={self.base_tool}, "
            f"flange={self.base_flange}, trocar={self.trocar}, L={self.tool_length:.4f}"
        )
        self.get_logger().info(
            "Forward-facing tool RPY reference: "
            f"roll={self.forward_tool_rpy_deg[0]:.0f}deg, "
            f"pitch={self.forward_tool_rpy_deg[1]:.0f}deg, "
            f"yaw={self.forward_tool_rpy_deg[2]:.0f}deg"
        )

    def _init_trajectory_deformer(self):
        """用当前自主参考轨迹初始化 0213 风格轨迹变形器。"""

        if self.base_tool is None:
            return
        count = max(int(np.ceil(self.task_duration_s / self.dt)) + 3, 4)
        samples = []
        for i in range(count):
            p, _ = self._nominal_reference(i * self.dt)
            samples.append(p)
        self.traj_deformer = TrajectoryDeformer0213(np.asarray(samples, dtype=float), self.dt, tau=3.0, mu=0.01)
        self.last_reshape_ref = self.base_tool.copy()
        self.last_direct_ref = self.base_tool.copy()
        self.last_reshape_vel = np.zeros(3)
        self.last_deformation_norm = 0.0

    def _tool_orientation_ref(self, t):
        if self.base_tool_R is None:
            return self.forward_tool_R
        if self.orientation_ramp_s <= 0.0:
            return self.forward_tool_R
        return interpolate_rotation(
            self.base_tool_R,
            self.forward_tool_R,
            float(t) / max(self.orientation_ramp_s, self.dt),
        )

    def _orientation_control(self, R_ref, R, w):
        """第3章稳定版姿态环：只在笛卡尔空间保持末端姿态。

        历史 no-RCM 调试中，腕部关节姿态项会和末端姿态任务争夺 q7 自转自由度。
        这里复用稳定版本的策略：SO(3) 姿态误差、角速度低通和限幅、姿态控制量限幅
        以及控制量变化率限幅，避免左右自转抖动。
        """
        e_R = so3_error(R_ref, R)
        filter_tau = max(float(self.orientation_omega_filter_tau_s), 0.0)
        alpha = self.dt / max(filter_tau + self.dt, self.dt)
        self.angular_velocity_filtered += alpha * (np.asarray(w, dtype=float) - self.angular_velocity_filtered)
        w_eff = clamp_norm(self.angular_velocity_filtered, self.orientation_omega_limit)

        self.integral_rot = np.clip(self.integral_rot + e_R * self.dt, -0.08, 0.08)
        u_rot_raw = (
            self.orientation_kp * e_R
            - self.orientation_kd * w_eff
            + self.orientation_ki * self.integral_rot
        )
        u_rot_raw = clamp_norm(u_rot_raw, self.orientation_u_rot_limit)

        max_step = max(float(self.orientation_u_rot_rate_limit), 0.0) * self.dt
        if max_step > 0.0:
            step = u_rot_raw - self.u_rot_prev
            step_norm = float(np.linalg.norm(step))
            if step_norm > max_step and step_norm > 1e-12:
                step *= max_step / step_norm
            self.u_rot_prev = self.u_rot_prev + step
            u_rot = self.u_rot_prev.copy()
        else:
            self.u_rot_prev = u_rot_raw.copy()
            u_rot = u_rot_raw
        return u_rot, e_R

    def _prealign_control(self, tool, flange, t):
        """在任务轨迹开始前单独完成末端正面朝前姿态对齐。

        历史 no-RCM 调试表明，若在轨迹跟踪同时给末端姿态一个较大阶跃，
        第 7 轴容易被激发为持续自转。该阶段只保持初始位置并平滑转到
        固定正面姿态，TRACK 阶段再开始真正的共享控制任务。
        """
        p_tool, R_tool, Jv_tool, Jw_tool, v_tool, w_tool = tool
        p_flange, R_flange, Jv_flange, Jw_flange, v_flange, w_flange = flange
        R_tool_ref = self._tool_orientation_ref(t)

        if self.task_mode == "with_rcm":
            pos_ref, vel_ref, R_ref = tool_to_flange_ref(
                self.base_tool,
                np.zeros(3),
                self.trocar,
                self.tool_length,
                forward_ref=self.with_rcm_forward_ref,
            )
            pos = p_flange
            vel = v_flange
            R = R_flange
            w = w_flange
            Jv = Jv_flange
            Jw = Jw_flange
            tracking_error = float(np.linalg.norm(p_tool - self.base_tool))
            p_rcm = rcm_point_on_axis(p_tool, p_flange, self.trocar)
            rcm_error = float(np.linalg.norm(self.trocar - p_rcm))
        else:
            pos_ref = self.base_tool
            vel_ref = np.zeros(3)
            R_ref = R_tool_ref
            pos = p_tool
            vel = v_tool
            R = R_tool
            w = w_tool
            Jv = Jv_tool
            Jw = Jw_tool
            tracking_error = float(np.linalg.norm(p_tool - self.base_tool))
            rcm_error = 0.0

        pos_err = pos_ref - pos
        vel_err = vel_ref - vel
        kp = np.array([210.0, 210.0, 180.0]) if self.task_mode == "with_rcm" else np.array([260.0, 260.0, 210.0])
        kd = np.array([58.0, 58.0, 50.0]) if self.task_mode == "with_rcm" else np.array([72.0, 72.0, 60.0])
        u_pos = clamp_norm(kp * pos_err + kd * vel_err, 32.0 if self.task_mode == "with_rcm" else 42.0)

        u_rot, e_R = self._orientation_control(R_ref, R, w)
        tau_task = Jv.T @ u_pos + Jw.T @ u_rot
        return {
            "tau_task": tau_task,
            "tracking_error": tracking_error,
            "rcm_error": rcm_error,
            "ori_err_deg": float(np.degrees(np.linalg.norm(e_R))),
            "front_axis_err_deg": (
                front_axis_error_deg(R, R_ref)
                if self.task_mode == "with_rcm"
                else front_axis_error_deg(R_tool, R_tool_ref)
            ),
        }

    def _nominal_reference(self, t):
        """机器人自主参考 ``x_r``。

        参考轨迹沿用原 ROS1 脚本中的四点 a-b-c-d 形状。实验最后 35% 时间保持最终目标，
        使收敛指标衡量稳态跟踪误差，而不是路径跟踪瞬态误差。
        """
        total = max(self.task_duration_s, 1e-6)
        if self.scenario in ("ustc_letters", "ustc_curve"):
            motion_total = 0.82 * total
            p_local, v_local = sample_polyline(
                ustc_polyline_offsets(self.task_mode),
                t,
                motion_total,
            )
            return self.base_tool + p_local, v_local
        if self.scenario in SMOOTH_LETTER_SCENARIOS:
            motion_total = 0.78 * total
            p_local, v_local = sample_polyline(
                ustc_smooth_letter_offsets(
                    self.scenario,
                    self.task_mode,
                    trajectory_scale=self.trajectory_scale if self.trajectory_scale > 0.0 else None,
                ),
                t,
                motion_total,
            )
            return self.base_tool + p_local, v_local
        if self.scenario in STIFFNESS_LINE_SCENARIOS:
            motion_total = 0.72 * total
            p_local, v_local = sample_polyline(
                stiffness_line_offsets(
                    self.task_mode,
                    trajectory_scale=self.trajectory_scale if self.trajectory_scale > 0.0 else None,
                ),
                t,
                motion_total,
            )
            return self.base_tool + p_local, v_local

        # 实验前段完成四个目标点过渡，后段保持最终目标；这样末段指标衡量收敛能力，
        # 而不是动态路径跟踪过程中的瞬态误差。
        motion_total = 0.65 * total
        s = float(np.clip(t / max(motion_total, 1e-6), 0.0, 1.0))
        if self.task_mode == "with_rcm":
            waypoints = np.array(
                [
                    [0.000, 0.000, 0.000],
                    [0.000, 0.000, 0.012],
                    [0.000, 0.018, 0.012],
                    [0.000, 0.018, 0.006],
                ],
                dtype=float,
            )
        else:
            waypoints = np.array(
                [
                    [0.000, 0.000, 0.000],
                    [0.000, 0.000, 0.024],
                    [0.000, 0.040, 0.024],
                    [0.000, 0.040, 0.010],
                ],
                dtype=float,
            )
            waypoints[:, 0] += np.linspace(0.0, 0.035, len(waypoints))
        seg_float = s * (len(waypoints) - 1)
        idx = int(np.clip(np.floor(seg_float), 0, len(waypoints) - 2))
        local_s = smoothstep(seg_float - idx)
        p_local = (1.0 - local_s) * waypoints[idx] + local_s * waypoints[idx + 1]

        ds_dt = (len(waypoints) - 1) / max(motion_total, 1e-6)
        d_smooth = 6.0 * (seg_float - idx) * (1.0 - (seg_float - idx)) * ds_dt
        v_local = d_smooth * (waypoints[idx + 1] - waypoints[idx])
        if t >= motion_total:
            v_local = np.zeros(3)
        return self.base_tool + p_local, v_local

    def _human_delta(self, t):
        """第4章场景的人类输入生成器 ``Delta x_m``。

        为保证 Gazebo 实验可重复，人类输入按时间窗脚本生成。每个场景对应一个论文验证点：
        安全切向修正、危险法向压入、短时扰动或持续干预。``mixed_sequence`` 组合全部窗口。
        """
        if self.scenario in ("external", "falcon_external", "live_human"):
            if self.external_human_stamp is None:
                return np.zeros(3)
            age = (self.get_clock().now() - self.external_human_stamp).nanoseconds * 1e-9
            if age > self.external_human_timeout_s:
                return np.zeros(3)
            return self._with_realistic_human_tremor(t, self.external_human_delta.copy())
        if self.human_input_profile in ("real_events", "realistic_events", "real_log_events"):
            return self._sample_realistic_human_event_input(t)
        delta = np.zeros(3)
        scenario = self.scenario
        if scenario in ("ustc_letters", "ustc_curve") or scenario in SMOOTH_LETTER_SCENARIOS:
            if self.human_input_profile in ("real_button", "button", "realistic_button"):
                delta, button = self._ustc_real_button_delta(t)
                return self._apply_human_profile(t, delta, button)
            total = max(self.task_duration_s, 1e-6)
            # USTC 轨迹中保留三类人类输入：绕开局部区域的切向修正、危险法向
            # 压入以及短时扰动。时间窗按总时长归一化，便于 30-45 s 任务复用。
            if 0.20 * total <= t <= 0.34 * total:
                delta += np.array([0.0, 0.012 * smoothstep((t - 0.20 * total) / (0.14 * total)), 0.0])
            if 0.42 * total <= t <= 0.435 * total:
                pulse = np.sin(np.pi * (t - 0.42 * total) / max(0.015 * total, self.dt))
                delta += np.array([0.0, -0.008 * pulse, -0.0035 * pulse])
            if 0.50 * total <= t <= 0.64 * total:
                delta += np.array([0.0, 0.0, -0.0075 * smoothstep((t - 0.50 * total) / (0.14 * total))])
            if 0.70 * total <= t <= 0.82 * total:
                s = smoothstep((t - 0.70 * total) / (0.12 * total))
                delta += np.array([0.008 * s, -0.009 * s, 0.0])
            return self._apply_human_profile(t, delta, float(np.linalg.norm(delta) > 1e-9))
        if scenario in ("mixed_sequence", "tangential_correction"):
            if 4.0 <= t <= 8.5:
                delta += np.array([0.0, 0.016 * smoothstep((t - 4.0) / 4.5), 0.0])
        if scenario in ("mixed_sequence", "unsafe_normal_push"):
            if 10.5 <= t <= 14.0:
                delta += np.array([0.0, 0.0, -0.008 * smoothstep((t - 10.5) / 3.5)])
        if scenario in ("tvio_stress_push", "force_violation_challenge"):
            # Chapter 5 T_vio challenge: a stronger and longer unsafe normal
            # push than the nominal Chapter 4 example.  It is intentionally
            # large enough for direct/fixed/no-projection baselines to cross
            # the force upper bound, while the safety projection should keep
            # the final reference inside the admissible force interval.
            if 6.0 <= t <= 14.0:
                rise = smoothstep((t - 6.0) / 2.0)
                fall = 1.0 - smoothstep((t - 12.5) / 1.5) if t >= 12.5 else 1.0
                amp = 0.014 * min(rise, fall)
                delta += np.array([0.0, 0.004 * amp / 0.014, -amp])
            if 14.5 <= t <= 16.0:
                # A small tangential recovery correction verifies that the
                # method can still accept useful human input after the unsafe
                # normal interval.
                s = smoothstep((t - 14.5) / 1.5)
                delta += np.array([0.0, 0.006 * s, 0.0])
        if scenario in ("mixed_sequence", "short_pulse_disturbance"):
            if 9.5 <= t <= 9.85:
                pulse = np.sin(np.pi * (t - 9.5) / 0.35)
                delta += np.array([0.0, -0.010 * pulse, -0.004 * pulse])
        if scenario in ("mixed_sequence", "sustained_intervention"):
            if 14.2 <= t <= 17.7:
                delta += np.array([0.012 * smoothstep((t - 14.2) / 3.5), -0.010 * smoothstep((t - 14.2) / 3.5), 0.0])
        return self._apply_human_profile(t, delta, float(np.linalg.norm(delta) > 1e-9))

    def _predict_force(self, nominal, candidate):
        """参考层安全逻辑使用的接触力代理模型。

        Gazebo 验证不依赖真实力传感器，而是使用局部线性模型：

            F_hat = F_d + K_hat * n_down^T (candidate - nominal)

        因此，向下法向运动会增加预测接触力。
        """
        n_down = np.array([0.0, 0.0, -1.0])
        normal_down = float(np.dot(candidate - nominal, n_down))
        return float(self.force_desired + self.contact_stiffness_hat * normal_down)

    def _force_margin(self, y_f):
        """归一化预测力距离软安全边界的裕度。"""
        half = max(0.5 * (self.force_max - self.force_min), 1e-9)
        return sat01(min(y_f - self.force_min, self.force_max - y_f) / half)

    def _safe_project_reference(self, nominal, human_candidate):
        """把人侧候选参考 ``x_h`` 投影到力安全区间。

        切向运动通常有助于任务且不增加法向接触风险，因此尽量保留。法向分量根据
        ``F_min + margin <= F_hat <= F_max - margin`` 对应的位移范围进行裁剪。

        返回安全参考、法向权限 ``kappa_N``、力安全裕度 ``rho_F``，以及论文绘图所需
        的诊断量。
        """
        nominal = np.asarray(nominal, dtype=float)
        candidate = np.asarray(human_candidate, dtype=float)
        n_down = np.array([0.0, 0.0, -1.0])
        delta = candidate - nominal
        normal_raw = float(np.dot(delta, n_down))
        tangent = delta - normal_raw * n_down
        max_down = max(0.0, (self.force_max - self.force_margin - self.force_desired) / max(self.contact_stiffness_hat, 1e-9))
        max_up = max(0.0, (self.force_desired - (self.force_min + self.force_margin)) / max(self.contact_stiffness_hat, 1e-9))
        normal_safe = float(np.clip(normal_raw, -max_up, max_down))
        safe = nominal + tangent + normal_safe * n_down
        y_candidate = self._predict_force(nominal, candidate)
        y_safe = self._predict_force(nominal, safe)
        rho = self._force_margin(y_candidate)
        kappa = float(np.clip(rho, self.kappa_min, 1.0))
        return safe, kappa, rho, y_candidate, y_safe, normal_raw, normal_safe

    def _interaction_force_for_deformation(self, h_delta):
        """将当前人输入恢复为 0213 轨迹变形使用的等效交互力。"""

        h_delta = np.asarray(h_delta, dtype=float)
        if np.linalg.norm(h_delta) < 1e-12:
            return np.zeros(3)
        raw_force = np.asarray(self.human_raw_force_N, dtype=float).reshape(3)
        if np.linalg.norm(raw_force) > 1e-12:
            return raw_force
        return h_delta / 0.010

    def _direct_willing_beta(self, distance):
        """0213 ``DirectWilling`` 与 ``get_smoothed_beta_window`` 的移植。"""

        beta_raw = 1.0 / (1.0 + np.exp(-600.0 * (float(distance) - 0.02)))
        self.beta_window.append(float(beta_raw))
        if len(self.beta_window) > self.beta_window_size:
            self.beta_window.pop(0)
        if len(self.beta_window) == 1:
            self.beta = float(self.beta_window[0])
            return self.beta
        weights = np.arange(1, len(self.beta_window) + 1, dtype=float)
        self.beta = float(np.dot(self.beta_window, weights) / np.sum(weights))
        self.beta_window[-1] = self.beta
        return self.beta

    def _candidate_reference(self, nominal, nominal_vel, h_delta):
        """构造 0213 风格的人侧候选参考。

        与 ROS1/0213 保持一致：先由交互力持续变形机器人自主轨迹，得到
        ``x_reshape``；再生成直接人侧参考 ``x_tele``；最后用由变形量驱动的
        ``beta`` 在二者之间融合，得到 ``x_h``。
        """
        if self.traj_deformer is None:
            self._init_trajectory_deformer()
        if self.traj_deformer is None:
            x_robot = nominal.copy()
            v_robot = nominal_vel.copy()
            x_reshape = nominal.copy()
            reshape_vel = np.zeros(3)
            deformation_norm = 0.0
        else:
            deform = self.traj_deformer.step(self._interaction_force_for_deformation(h_delta))
            x_robot = deform["nominal"]
            v_robot = deform["nominal_vel"]
            x_reshape = deform["reshape"]
            reshape_vel = deform["reshape_vel"]
            deformation_norm = deform["deformation_norm"]
        x_tele = x_robot + h_delta
        beta = self._direct_willing_beta(deformation_norm)
        x_h = beta * x_tele + (1.0 - beta) * x_reshape
        self.last_reshape_ref = x_reshape.copy()
        self.last_direct_ref = x_tele.copy()
        self.last_reshape_vel = reshape_vel.copy()
        self.last_deformation_norm = float(deformation_norm)
        d_dev = float(np.linalg.norm(x_h - x_robot))
        return x_tele, x_h, d_dev, x_robot, v_robot

    def _direction_consistency(self, h_delta, nominal_vel):
        """计算人类输入方向与自主参考速度方向的一致性余弦值。"""
        h_norm = float(np.linalg.norm(h_delta))
        r_norm = float(np.linalg.norm(nominal_vel))
        if h_norm < 1e-9 or r_norm < 1e-9:
            return 0.0
        return float(np.clip(np.dot(h_delta, nominal_vel) / (h_norm * r_norm + 1e-9), -1.0, 1.0))

    def _alpha_hr(self, t, human_delta, tool_pos, safety_distance):
        """计算人机共享权限权重 ``alpha_HR``。

        特征设计沿用 ROS1/0213 思路：人类输入强度 ``F_h``、累计干预时间 ``T_h`` 和
        安全距离 ``D_r``。默认分支使用模糊规则和小型 Kalman 平滑器；
        ``*_sigmoid`` 变体使用更简单的单因素基线。
        """
        F_h = float(np.linalg.norm(human_delta) / 0.010)
        if F_h > 0.05:
            self.human_time += self.dt
        else:
            self.human_time = max(0.0, self.human_time - 0.5 * self.dt)
        D_r = float(np.clip(safety_distance, 0.0, 0.12))
        dF = (F_h - self.prev_Fh) / self.dt
        dT = (self.human_time - self.prev_Th) / self.dt
        dD = (D_r - self.prev_Dr) / self.dt
        self.prev_Fh = F_h
        self.prev_Th = self.human_time
        self.prev_Dr = D_r

        if self.controller_variant.endswith("sigmoid"):
            alpha = sigmoid_alpha(F_h)
            raw = alpha
            delta_lambda = 0.0
        else:
            alpha, raw, delta_lambda = self.arbitrator.update(F_h, self.human_time, D_r, dF, dT, dD)
        self.last_alpha = alpha
        return alpha, raw, delta_lambda, F_h, D_r

    def _reference(self, t, tool_pos):
        """第4章完整参考层流水线。

        这是不同对比策略发生分叉的主要位置。所有策略都从相同的 ``x_r``、
        ``x_tele``、``x_h`` 和 ``x_h_safe`` 出发，然后决定有多少人类权限进入最终参考
        ``x_d``。返回的字典既供控制器使用，也会被记录器保存用于实验分析。
        """
        nominal, nominal_vel = self._nominal_reference(t)
        h_delta = self._human_delta(t)
        x_tele, x_h, d_dev, nominal, nominal_vel = self._candidate_reference(nominal, nominal_vel, h_delta)
        x_h_safe, kappa_n, rho_f, y_f_candidate, y_f_safe, normal_raw, normal_safe = self._safe_project_reference(nominal, x_h)
        safety_distance = 0.10 * rho_f
        alpha_dyn, raw, delta_lambda, F_h, D_r = self._alpha_hr(t, h_delta, tool_pos, safety_distance)
        c_h = self._direction_consistency(h_delta, nominal_vel)
        strategy = self._arbitration_family()
        # 论文对比策略：
        # - autonomous_only：不接受人类权限。
        # - direct_accept：直接接受原始遥操作输入，故意保留安全风险作为基线。
        # - fixed_blend：固定 alpha_HR。
        # - single_sigmoid：只根据输入幅值自适应 alpha_HR。
        # - dynamic_no_projection：使用带力裕度的动态 alpha_HR，但不做安全投影。
        # - dynamic_no_projection_open：只按人类输入强度给权重，不做安全投影，也不
        #   使用力裕度门控；用于第5章 T_vio 专项的“无安全投影”强消融基线。
        # - full_method：动态 alpha_HR + 安全投影 + 法向抑制。
        if strategy == "autonomous_only":
            alpha = 0.0
            tool_ref = nominal
            x_h_safe_used = nominal
        elif strategy == "direct_accept":
            alpha = 1.0
            tool_ref = x_tele
            x_h_safe_used = x_tele
        elif strategy == "fixed_blend":
            alpha = float(np.clip(self.fixed_alpha_hr, 0.0, 1.0))
            tool_ref = (1.0 - alpha) * nominal + alpha * x_h
            x_h_safe_used = x_h
        elif strategy == "single_sigmoid":
            alpha = 1.0 / (1.0 + np.exp(-120.0 * (d_dev - 0.004)))
            tool_ref = (1.0 - alpha) * nominal + alpha * x_h
            x_h_safe_used = x_h
        elif strategy == "dynamic_no_projection":
            alpha = alpha_dyn
            tool_ref = (1.0 - alpha) * nominal + alpha * x_h
            x_h_safe_used = x_h
        elif strategy == "dynamic_no_projection_open":
            alpha = float(np.clip(0.15 + 0.65 / (1.0 + np.exp(-120.0 * (d_dev - 0.004))), 0.0, 0.85))
            raw = alpha
            alpha_dyn = alpha
            delta_lambda = 0.0
            tool_ref = (1.0 - alpha) * nominal + alpha * x_h
            x_h_safe_used = x_h
        else:
            alpha = alpha_dyn
            delta_safe = x_h_safe - nominal
            n_down = np.array([0.0, 0.0, -1.0])
            normal_delta = float(np.dot(delta_safe, n_down))
            tangent_delta = delta_safe - normal_delta * n_down
            # 完整方法通过 alpha 保留有用的切向修正；当力代理量接近边界时，再用
            # kappa_N 对法向分量进行二次门控。
            tool_ref = nominal + alpha * tangent_delta + alpha * kappa_n * normal_delta * n_down
            x_h_safe_used = x_h_safe
        y_f_final = self._predict_force(nominal, tool_ref)
        tool_vel_ref = nominal_vel
        return {
            "tool_ref": tool_ref,
            "tool_vel_ref": tool_vel_ref,
            "nominal": nominal,
            "x_tele": x_tele,
            "x_reshape": self.last_reshape_ref if self.last_reshape_ref is not None else nominal,
            "x_reshape_vel": self.last_reshape_vel,
            "deformation_norm": self.last_deformation_norm,
            "x_h": x_h,
            "x_h_safe": x_h_safe_used,
            "alpha": alpha,
            "alpha_raw": raw,
            "alpha_dynamic": alpha_dyn,
            "delta_lambda": delta_lambda,
            "F_h": F_h,
            "D_r": D_r,
            "D_c": 1.0 - rho_f,
            "rho_F": rho_f,
            "kappa_N": kappa_n,
            "beta": self.beta,
            "I_h": F_h,
            "T_h": self.human_time,
            "C_h": c_h,
            "human_button": self.human_button_state,
            "human_force_cmd_N": self.human_force_cmd_N,
            "human_event_type_id": self.human_event_type_id,
            "human_raw_force_N": self.human_raw_force_N,
            "human_delta_cmd": self.human_delta_cmd,
            "y_f": y_f_final,
            "y_f_candidate": y_f_candidate,
            "y_f_safe": y_f_safe,
            "F_min": self.force_min,
            "F_d": self.force_desired,
            "F_max": self.force_max,
            "normal_raw": normal_raw,
            "normal_safe": normal_safe,
        }

    def _alpha_fp_target(self, ref):
        """第3章执行层力/位优先级代理量。

        ``alpha_FP = 1`` 表示更重视位置跟踪；``alpha_FP = 0`` 表示更重视力调节。
        在线 Gazebo 节点使用紧凑代理形式，使第5章集成实验可以复用同一套方法开关：
        基线使用固定优先级，完整方法使用由力安全裕度驱动的动态优先级。
        """
        strategy = self._execution_family()
        if strategy == "fixed_02":
            return 0.2
        if strategy == "fixed_08":
            return 0.8
        if strategy == "fixed_05":
            return 0.5
        if strategy == "standard_impedance":
            return 1.0
        if strategy == "traditional_hybrid":
            return 0.0
        if strategy == "standard_mpc":
            return 0.5
        if strategy in ("dynamic_gt", "continuous_force_margin"):
            rho = float(ref.get("rho_F", 1.0))
            y_f = float(ref.get("y_f", self.force_desired))
            force_span = max(self.force_max - self.force_min, 1e-9)
            force_err = abs(y_f - self.force_desired) / force_span
            # 当力处于安全区间中部时保持较高位置权限；接近力边界或力误差较大时，
            # 降低 alpha_FP，让法向力释放项发挥作用。
            target = 0.75 * rho + 0.25 * (1.0 - np.clip(force_err, 0.0, 1.0))
            return float(np.clip(target, 0.15, 0.85))
        return float(np.clip(self.fixed_alpha_fp, 0.0, 1.0))

    def _update_alpha_fp(self, ref):
        """对执行层力/位优先级进行一阶低通滤波。"""
        target = self._alpha_fp_target(ref)
        tau = max(self.alpha_fp_tau_s, self.dt)
        self.alpha_fp += self.dt * (target - self.alpha_fp) / tau
        self.alpha_fp = float(np.clip(self.alpha_fp, 0.0, 1.0))
        return self.alpha_fp, target

    def _task_control(self, tool, flange, t):
        """把最终参考映射为关节力矩命令。

        第4章主要研究参考层仲裁，因此这里的执行控制保持常规形式：任务空间位置/姿态
        反馈、可选 GT/LQR 形状增益、Gazebo 稳定 PD 包络，以及雅可比转置力矩映射。
        """
        p_tool, R_tool, Jv_tool, Jw_tool, v_tool, w_tool = tool
        p_flange, R_flange, Jv_flange, Jw_flange, v_flange, w_flange = flange
        ref = self._reference(t, p_tool)
        tool_ref = ref["tool_ref"]
        tool_vel_ref = ref["tool_vel_ref"]
        alpha = ref["alpha"]
        alpha_fp, alpha_fp_target = self._update_alpha_fp(ref)
        if self._use_rcm_specific_control():
            gate_nominal = (
                self.rcm_limited_tool_ref
                if self.rcm_limited_tool_ref is not None
                else p_tool
            )
            gate_force_info = self._actual_force_feedback(gate_nominal, p_tool, update_filter=False)
            gate_force = float(gate_force_info["force_feedback"])
            tool_ref, tool_vel_ref, _ = self._with_rcm_limited_reference(
                tool_ref,
                p_tool,
                p_flange,
                gate_force,
            )
            force_feedback_info = self._actual_force_feedback(tool_ref, p_tool)
            force_feedback = float(force_feedback_info["force_feedback"])
        elif self.realistic_env_enabled and self.real_env is not None:
            p_tool_actual = np.asarray(p_tool, dtype=float)
            tool_ref_actual = np.asarray(tool_ref, dtype=float)
            # no-RCM 的历史稳定闭环以参考层力代理作为执行层反馈；实际
            # 末端压入量得到的接触力保留为记录/评价通道。若把高刚度、
            # 带噪声的实际接触力直接喂给此处的连续力位混合控制器，会在
            # 刚度切换处形成未重新整定的高增益力反馈闭环，导致抖动放大。
            force_feedback_info = self._actual_force_feedback(tool_ref_actual, p_tool_actual)
            force_feedback = float(ref["y_f"])
        else:
            # 非真实环境 Gazebo 没有接触传感器模型，继续使用参考层力代理。
            force_feedback_info = {}
            force_feedback = float(ref["y_f"])

        if self.task_mode == "with_rcm":
            # with-RCM 模式下参考定义在工具尖端，但受控坐标系是法兰；
            # RCM 转换用于保持虚拟 trocar 约束。
            pos_ref, vel_ref, R_ref = tool_to_flange_ref(
                tool_ref,
                tool_vel_ref,
                self.trocar,
                self.tool_length,
                forward_ref=self.with_rcm_forward_ref,
            )
            pos = p_flange
            vel = v_flange
            R = R_flange
            w = w_flange
            Jv = Jv_flange
            Jw = Jw_flange
            tracking_error = float(np.linalg.norm(p_tool - tool_ref))
            p_rcm = rcm_point_on_axis(p_tool, p_flange, self.trocar)
            rcm_error = float(np.linalg.norm(self.trocar - p_rcm))
        else:
            # no-RCM 模式下直接控制工具坐标系。
            pos_ref = tool_ref
            vel_ref = tool_vel_ref
            R_ref = self.forward_tool_R
            pos = p_tool
            vel = v_tool
            R = R_tool
            w = w_tool
            Jv = Jv_tool
            Jw = Jw_tool
            tracking_error = float(np.linalg.norm(p_tool - tool_ref))
            rcm_error = 0.0

        pos_err = pos_ref - pos
        vel_err = vel_ref - vel
        self.integral_pos = np.clip(self.integral_pos + pos_err * self.dt, -0.02, 0.02)
        strategy = self._execution_family()
        force_error = float(force_feedback - self.force_desired)
        force_span = max(self.force_max - self.force_min, 1e-9)
        execution_debug = {}

        if strategy == "standard_impedance":
            # 标准阻抗基线：任务空间三轴均按位置/速度误差形成弹簧阻尼力。
            # 它不显式闭合法向力环，因此预期轨迹误差较小，但接触力边界风险更高。
            if self.task_mode == "with_rcm" and self._is_rcm_execution():
                kp = np.array([170.0, 170.0, 130.0])
                kd = np.array([58.0, 58.0, 44.0])
            elif self.task_mode == "with_rcm":
                kp = np.array([220.0, 220.0, 185.0])
                kd = np.array([66.0, 66.0, 54.0])
            else:
                kp = np.array([430.0, 430.0, 250.0])
                kd = np.array([118.0, 118.0, 76.0])
            u_pos = kp * pos_err + kd * vel_err + 6.0 * self.integral_pos
        elif strategy == "traditional_hybrid":
            # 传统混合力位基线：切向仍用位置控制，法向改为力误差反馈。
            # 该方法预期能减小法向力偏差，但在复杂曲线和人类参考变化下会牺牲法向位置跟踪。
            if self.task_mode == "with_rcm" and self._is_rcm_execution():
                kp_xy = np.array([175.0, 175.0])
                kd_xy = np.array([58.0, 58.0])
                kf_z = 3.2
                kp_z_reg = 32.0
                kd_z_reg = 16.0
            elif self.task_mode == "with_rcm":
                kp_xy = np.array([220.0, 220.0])
                kd_xy = np.array([66.0, 66.0])
                kf_z = 13.0
                kp_z_reg = 62.0
                kd_z_reg = 26.0
            else:
                kp_xy = np.array([430.0, 430.0])
                kd_xy = np.array([118.0, 118.0])
                kf_z = 18.0
                kp_z_reg = 92.0
                kd_z_reg = 36.0
            u_pos = np.zeros(3)
            u_pos[:2] = kp_xy * pos_err[:2] + kd_xy * vel_err[:2] + 4.0 * self.integral_pos[:2]
            # force_error > 0 表示实际接触力偏大，应向上释放；force_error < 0 则向下补偿接触。
            # 加入较小的法向位置正则项，避免无明显力误差时传统混合力位基线漂离参考面。
            u_pos[2] = kf_z * force_error + kp_z_reg * pos_err[2] + kd_z_reg * vel_err[2]
        elif strategy == "standard_mpc" or self.controller_variant.startswith("mpc"):
            # 标准 MPC/QP 风格基线：先计算名义位置 PD，再用一阶预测的力边界约束修正法向输入。
            # 与本文动态 GT 不同，它不输出连续可解释的 alpha_FP，只在预测越界时做约束投影。
            if self.task_mode == "with_rcm" and self._is_rcm_execution():
                kp = np.array([185.0, 185.0, 140.0])
                kd = np.array([60.0, 60.0, 45.0])
                kp_z_guard = 130.0
                kd_z_guard = 45.0
            elif self.task_mode == "with_rcm":
                kp = np.array([230.0, 230.0, 198.0])
                kd = np.array([68.0, 68.0, 56.0])
                kp_z_guard = 155.0
                kd_z_guard = 54.0
            else:
                kp = np.array([410.0, 410.0, 240.0])
                kd = np.array([112.0, 112.0, 74.0])
                kp_z_guard = 230.0
                kd_z_guard = 76.0
            u_pos = kp * pos_err + kd * vel_err + 8.0 * self.integral_pos
            force_upper = self.force_max - self.force_margin
            force_lower = self.force_min + self.force_margin
            # 正 u_z 近似向上释放，负 u_z 近似向下压入。早期实现使用
            # “越界后增量修正”，在真实噪声种子下可能把法向输入反向过度修正，
            # 导致 S 形轨迹个别试次 z 向下沉。这里保留标准 MPC/QP 基线的短预测
            # 思路，但直接把 u_z 投影到一阶力预测约束的可行区间。
            if self._use_rcm_specific_control():
                horizon_stiffness = float(force_feedback_info.get("K_env_control", self.contact_stiffness_hat))
                horizon_gain = max(0.012 * horizon_stiffness, 1e-9)
            else:
                horizon_gain = max(0.010 * self.contact_stiffness_hat, 1e-9)
            u_z_min = (force_feedback - force_upper) / horizon_gain
            u_z_max = (force_feedback - force_lower) / horizon_gain
            u_pos[2] = float(np.clip(u_pos[2], u_z_min, u_z_max))
            # 实物化实验允许 10 mm 误差，但不允许法向通道脱离参考面。若
            # z 误差已经超过 12 mm，则位置安全护栏优先于力预测投影，使
            # 对照组保持稳定而不改变其“短预测+约束投影”的基本性质。
            if abs(float(pos_err[2])) > 0.012:
                z_guard = kp_z_guard * pos_err[2] + kd_z_guard * vel_err[2]
                if pos_err[2] > 0.0:
                    u_pos[2] = max(float(u_pos[2]), float(z_guard))
                else:
                    u_pos[2] = min(float(u_pos[2]), float(z_guard))
        elif strategy in (
            "dynamic_gt",
            "continuous_force_margin",
            "fixed_02",
            "fixed_05",
            "fixed_08",
        ) and self.no_rcm_execution is not None:
            if self._use_rcm_specific_control():
                K_hat = float(force_feedback_info.get("K_env_control", self.contact_stiffness_hat))
            else:
                penetration_proxy = max(
                    0.0,
                    (float(ref["y_f"]) - self.force_desired) / max(self.contact_stiffness_hat, 1.0),
                )
                K_hat = float(self.contact_stiffness_hat)
                if self.realistic_env_enabled and self.real_env is not None:
                    K_hat, _, _ = self._realistic_stiffness(tool_ref, penetration_proxy)
            out = self.no_rcm_execution.compute(
                pos=pos,
                vel=vel,
                pos_ref=pos_ref,
                vel_ref=vel_ref,
                force_norm=force_feedback,
                K_hat=K_hat,
                force_blend=1.0,
                tracking_boost_enabled=True,
            )
            u_pos = np.asarray(out.u, dtype=float)
            execution_debug = dict(getattr(out, "debug", {}))
            if self._use_rcm_specific_control():
                # In the long-tool RCM task, the flange command converted from
                # a tool-point reference can leave a small positive penetration
                # while the RCM projection keeps the tool axis constrained.  A
                # bounded upward release driven by the actual force feedback
                # keeps the complete method force-safe without adding the old
                # parallel tool-tip normal loop that caused wrist oscillation.
                high_force = max(0.0, force_feedback - (self.force_desired + 0.12))
                u_pos[2] += min(10.0, 2.8 * high_force)
            self.alpha_fp = float(out.alpha)
            alpha_fp = float(out.alpha)
            alpha_fp_target = float(out.alpha_target)
        else:
            # GT/KF 变体：根据 alpha_HR 计算 0213 连续 LQR 增益，再与调好的 PD 包络融合，
            # 使 Gazebo effort 模式保持稳定且力矩有界。
            K = compute_gt_gain(alpha)
            state_err = np.hstack([pos_err, vel_err])
            u_pos = K @ state_err + 8.0 * self.integral_pos
            # 保留 LQR 形状，但使用有界工程包络适配 Gazebo effort 控制。
            if self.task_mode == "with_rcm":
                kp_pd = np.array([235.0, 235.0, 210.0])
                kd_pd = np.array([70.0, 70.0, 60.0])
            else:
                kp_pd = np.array([460.0, 460.0, 260.0])
                kd_pd = np.array([125.0, 125.0, 80.0])
            # 执行层优先级只改变法向方向。这与第3章 no-RCM 实验约定一致：
            # 切向扫描保持位置主导，接触法向允许在位置跟踪和力安全之间权衡。
            kp_pd[2] *= 0.45 + 0.55 * alpha_fp
            kd_pd[2] *= 0.55 + 0.45 * alpha_fp
            u_pd = kp_pd * pos_err + kd_pd * vel_err
            u_pos = 0.20 * u_pos + 0.80 * u_pd

        if strategy in ("dynamic_gt", "continuous_force_margin") and self.no_rcm_execution is None:
            # 当预测法向力高于期望且 alpha_FP 较低时，在法向加入有界释放项。
            relief = (1.0 - alpha_fp) * 18.0 * force_error
            u_pos[2] += relief

        u_pos_pre_clamp = np.asarray(u_pos, dtype=float).copy()
        u_pos_limit = 42.0 if self.task_mode == "with_rcm" else 60.0
        u_pos_norm_pre_clamp = float(np.linalg.norm(u_pos_pre_clamp))
        u_pos = clamp_norm(u_pos_pre_clamp, u_pos_limit)
        u_pos_norm_post_clamp = float(np.linalg.norm(u_pos))
        u_pos_limit_active = 1.0 if u_pos_norm_pre_clamp > u_pos_limit + 1e-12 else 0.0
        u_pos_limit_scale = (
            u_pos_norm_post_clamp / max(u_pos_norm_pre_clamp, 1e-12)
            if u_pos_norm_pre_clamp > 1e-12
            else 1.0
        )
        u_rot, e_R = self._orientation_control(R_ref, R, w)
        ori_err_deg = float(np.degrees(np.linalg.norm(e_R)))
        front_err_deg = front_axis_error_deg(R, R_ref)

        u_tool_aux = np.zeros(3)
        # Do not add a second tool-tip normal loop in with-RCM mode.  The
        # stable Chapter-3 RCM controller regulates force through the flange
        # reference converted from the tool point; adding a parallel
        # J_tool^T normal wrench competes with the trocar geometry and was the
        # source of the short/fast RCM instability.

        tau_rcm = np.zeros(7)
        u_rcm = np.zeros(3)
        if self._use_rcm_specific_control():
            tau_rcm, u_rcm, rcm_projected_error = self._rcm_projection_wrench(tool, flange)
            rcm_error = max(rcm_error, rcm_projected_error)

        tau_pos_component = Jv.T @ u_pos
        tau_rot_component = Jw.T @ u_rot
        tau_tool_aux_component = Jv_tool.T @ u_tool_aux
        tau_task = tau_pos_component + tau_rot_component + tau_tool_aux_component + tau_rcm
        tangential_pos_err = pos_err.copy()
        tangential_pos_err[2] = 0.0
        tangential_vel_err = vel_err.copy()
        tangential_vel_err[2] = 0.0
        task_debug = {
            "task_pos_err": pos_err.copy(),
            "task_vel_err": vel_err.copy(),
            "task_position_error_norm": float(np.linalg.norm(pos_err)),
            "task_velocity_error_norm": float(np.linalg.norm(vel_err)),
            "task_tangential_pos_err_norm": float(np.linalg.norm(tangential_pos_err)),
            "task_normal_pos_err": float(pos_err[2]),
            "task_tangential_vel_err_norm": float(np.linalg.norm(tangential_vel_err)),
            "task_normal_vel_err": float(vel_err[2]),
            "task_force_error_N": force_error,
            "task_force_error_ratio": float(force_error / force_span),
            "task_force_span_N": force_span,
            "u_pos_pre_clamp": u_pos_pre_clamp,
            "u_pos_norm_pre_clamp": u_pos_norm_pre_clamp,
            "u_pos_norm_post_clamp": u_pos_norm_post_clamp,
            "u_pos_limit": float(u_pos_limit),
            "u_pos_limit_active": u_pos_limit_active,
            "u_pos_limit_scale": float(u_pos_limit_scale),
            "tau_pos_component": tau_pos_component,
            "tau_rot_component": tau_rot_component,
            "tau_tool_aux_component": tau_tool_aux_component,
            "tau_rcm_component": tau_rcm,
            "tau_task_norm": float(np.linalg.norm(tau_task)),
        }
        return {
            "tau_task": tau_task,
            "tool_ref": tool_ref,
            "tool_vel_ref": tool_vel_ref,
            "nominal": ref["nominal"],
            "human": ref["x_h_safe"],
            "x_tele": ref["x_tele"],
            "x_reshape": ref["x_reshape"],
            "x_reshape_vel": ref["x_reshape_vel"],
            "deformation_norm": ref["deformation_norm"],
            "x_h": ref["x_h"],
            "x_h_safe": ref["x_h_safe"],
            "alpha": alpha,
            "alpha_FP": alpha_fp,
            "alpha_FP_target": alpha_fp_target,
            "execution_strategy": self.execution_strategy,
            "alpha_raw": ref["alpha_raw"],
            "alpha_dynamic": ref["alpha_dynamic"],
            "delta_lambda": ref["delta_lambda"],
            "F_h": ref["F_h"],
            "D_r": ref["D_r"],
            "D_c": ref["D_c"],
            "rho_F": ref["rho_F"],
            "kappa_N": ref["kappa_N"],
            "beta": ref["beta"],
            "I_h": ref["I_h"],
            "T_h": ref["T_h"],
            "C_h": ref["C_h"],
            "human_button": ref["human_button"],
            "human_force_cmd_N": ref["human_force_cmd_N"],
            "human_event_type_id": ref["human_event_type_id"],
            "human_raw_force_N": ref["human_raw_force_N"],
            "human_delta_cmd": ref["human_delta_cmd"],
            "y_f": ref["y_f"],
            "y_f_candidate": ref["y_f_candidate"],
            "y_f_safe": ref["y_f_safe"],
            "F_min": ref["F_min"],
            "F_d": ref["F_d"],
            "F_max": ref["F_max"],
            "normal_raw": ref["normal_raw"],
            "normal_safe": ref["normal_safe"],
            "tracking_error": tracking_error,
            "rcm_error": rcm_error,
            "u_pos": u_pos,
            "u_rot": u_rot,
            "u_tool_aux": u_tool_aux,
            "u_rcm": u_rcm,
            "rcm_reference_scale": self.rcm_reference_scale,
            "ori_err_deg": ori_err_deg,
            "front_axis_err_deg": front_err_deg,
            "tool_pos_actual": p_tool,
            **task_debug,
            **execution_debug,
            **force_feedback_info,
        }

    def _finish(self):
        """停止机器人，保存 NPZ 原始数据，并写出 summary JSON。"""
        self._set_phase(self.PHASE_DONE)
        self._publish_effort(np.zeros(7))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(
            self.output_dir,
            (
                f"{self.task_mode}_{self.controller_variant}_"
                f"{self.arbitration_strategy}_{self.execution_strategy}_{self.scenario}_{stamp}"
            ),
        )
        os.makedirs(run_dir, exist_ok=True)
        npz_path = os.path.join(run_dir, "result.npz")
        self.logger_npz.save(npz_path)
        metrics = self._metrics(self.logger_npz.arrays())
        metrics.update({
            "log_schema_version": "ch345_extended_v1",
            "task_mode": self.task_mode,
            "controller_variant": self.controller_variant,
            "arbitration_strategy": self.arbitration_strategy,
            "execution_strategy": self.execution_strategy,
            "execution_alpha_profile": self.execution_alpha_profile,
            "scenario": self.scenario,
            "task_duration_s": self.task_duration_s,
            "dt": self.dt,
            "control_rate_hz": float(1.0 / max(self.dt, 1e-12)),
            "trajectory_scale": self.trajectory_scale,
            "force_desired": self.force_desired,
            "force_min": self.force_min,
            "force_max": self.force_max,
            "force_margin": self.force_margin,
            "contact_stiffness_hat": self.contact_stiffness_hat,
            "realistic_env_enabled": bool(self.realistic_env_enabled),
            "realistic_profile": self.realistic_profile,
            "realistic_seed": self.realistic_seed,
            "realistic_tau_noise_gain": self.realistic_tau_noise_gain,
            "realistic_use_measured_force_for_metrics": bool(self.realistic_use_measured_force_for_metrics),
            "human_input_profile": self.human_input_profile,
            "external_human_scale": self.external_human_scale,
            "external_human_max_delta_m": self.external_human_max_delta_m,
            "gravity_compensation_scale": self.gravity_scale,
            "max_tau_rate": self.max_tau_rate,
            "max_tau_abs": self.max_tau_abs.tolist(),
            "orientation_rpy_deg": self.forward_tool_rpy_deg.tolist(),
            "target_tolerance_m": self.target_tolerance_m,
            "success": bool(
                metrics.get("tracking_last_rms_m", 1.0) <= self.target_tolerance_m
                and metrics.get("tracking_last_max_m", 1.0) <= 1.5 * self.target_tolerance_m
                and (
                    self.task_mode == "no_rcm"
                    or metrics.get("rcm_last_rms_m", 1.0) <= self.target_tolerance_m
                )
            ),
        })
        summary_path = os.path.join(run_dir, "summary.json")
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f"Saved result: {npz_path}")
        self.get_logger().info(f"Saved summary: {summary_path}")
        self.get_logger().info(f"Final metrics: {metrics}")

    @staticmethod
    def _metrics(data):
        """每次运行结束时保存的快速在线摘要指标。

        专用的 ``ch4_metrics.py`` 会从 ``result.npz`` 重新计算更严格的论文时间窗指标。
        这里的轻量摘要用于 Gazebo launch 刚结束时快速判断实验是否基本成功。
        """
        if "tracking_error" not in data or len(data["tracking_error"]) == 0:
            return {}
        err = np.asarray(data["tracking_error"], dtype=float)
        rcm = np.asarray(data.get("rcm_error", np.zeros_like(err)), dtype=float)
        n0 = max(0, int(0.75 * len(err)))
        metrics = {
            "samples": int(len(err)),
            "tracking_rms_m": float(np.sqrt(np.mean(err ** 2))),
            "tracking_max_m": float(np.max(err)),
            "tracking_last_rms_m": float(np.sqrt(np.mean(err[n0:] ** 2))),
            "tracking_last_max_m": float(np.max(err[n0:])),
            "rcm_rms_m": float(np.sqrt(np.mean(rcm ** 2))),
            "rcm_max_m": float(np.max(rcm)),
            "rcm_last_rms_m": float(np.sqrt(np.mean(rcm[n0:] ** 2))),
            "rcm_last_max_m": float(np.max(rcm[n0:])),
            "alpha_mean": float(np.mean(np.asarray(data.get("alpha", [0.0])))),
        }
        try:
            x_r = np.column_stack([np.asarray(data[f"nominal_ref_{i}"], dtype=float) for i in range(3)])
            x_h = np.column_stack([np.asarray(data[f"x_h_{i}"], dtype=float) for i in range(3)])
            x_d = np.column_stack([np.asarray(data[f"tool_ref_{i}"], dtype=float) for i in range(3)])
            n_down = np.array([0.0, 0.0, -1.0])
            delta_h = x_h - x_r
            delta_d = x_d - x_r
            h_n = delta_h @ n_down
            d_n = delta_d @ n_down
            h_t = delta_h - h_n[:, None] * n_down
            d_t = delta_d - d_n[:, None] * n_down
            tangential_window = np.linalg.norm(h_t, axis=1) > 0.001
            normal_window = h_n > 0.001
            if np.any(tangential_window):
                metrics["R_acc"] = float(
                    np.sum(np.linalg.norm(d_t[tangential_window], axis=1))
                    / (np.sum(np.linalg.norm(h_t[tangential_window], axis=1)) + 1e-9)
                )
            else:
                metrics["R_acc"] = 0.0
            if np.any(normal_window):
                metrics["R_sup"] = float(
                    1.0 - np.sum(np.abs(d_n[normal_window])) / (np.sum(np.abs(h_n[normal_window])) + 1e-9)
                )
            else:
                metrics["R_sup"] = 0.0
        except KeyError:
            pass
        y_key = "y_f_realistic" if "y_f_realistic" in data else "y_f"
        if y_key in data:
            y_f = np.asarray(data[y_key], dtype=float)
            f_min = float(np.asarray(data.get("F_min", [0.35]))[0])
            f_max = float(np.asarray(data.get("F_max", [1.2]))[0])
            f_d = float(np.asarray(data.get("F_d", [0.7]))[0])
            dt = float(np.mean(np.diff(np.asarray(data.get("t", np.arange(len(y_f))), dtype=float)))) if len(y_f) > 1 else 0.01
            metrics["T_vio_s"] = float(dt * np.sum((y_f < f_min) | (y_f > f_max)))
            metrics["F_peak_N"] = float(np.max(np.abs(y_f - f_d)))
            metrics["F_max_observed_N"] = float(np.max(y_f))
        if "alpha" in data and len(data["alpha"]) > 1:
            alpha = np.asarray(data["alpha"], dtype=float)
            t = np.asarray(data.get("t", np.arange(len(alpha)) * 0.01), dtype=float)
            dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.01
            metrics["S_alpha"] = float(np.sqrt(np.mean((np.diff(alpha) / max(dt, 1e-9)) ** 2)))
            metrics["alpha_max"] = float(np.max(alpha))
        if "alpha_FP" in data and len(data["alpha_FP"]) > 1:
            alpha_fp = np.asarray(data["alpha_FP"], dtype=float)
            t = np.asarray(data.get("t", np.arange(len(alpha_fp)) * 0.01), dtype=float)
            dt = float(np.mean(np.diff(t))) if len(t) > 1 else 0.01
            metrics["alpha_FP_mean"] = float(np.mean(alpha_fp))
            metrics["alpha_FP_min"] = float(np.min(alpha_fp))
            metrics["alpha_FP_max"] = float(np.max(alpha_fp))
            metrics["S_alpha_FP"] = float(np.sqrt(np.mean((np.diff(alpha_fp) / max(dt, 1e-9)) ** 2)))
        return metrics

    def _on_timer(self):
        if not self.have_state or not self.model_ready:
            return

        _, _, tool, flange, tau_g = self._frame_states()
        p_tool = tool[0]

        if not self.enable_control:
            self._publish_effort(np.zeros(7))
            return

        if self.phase == self.PHASE_WAIT:
            if self._elapsed() > 1.0:
                if bool(self.get_parameter("use_current_as_home").value):
                    self.home_joints = self.q.copy()
                self._set_phase(self.PHASE_HOME)
                self.get_logger().info("Phase HOME: holding current configuration.")
            self._publish_effort(np.zeros(7))
            return

        if self.phase == self.PHASE_HOME:
            tau = self._rate_and_clip(self._hold_home_tau(tau_g))
            self._publish_effort(tau)
            if self._phase_elapsed() >= self.home_hold_s:
                self._init_task_geometry(tool, flange)
                if self.orientation_ramp_s > self.dt:
                    self._set_phase(self.PHASE_INIT)
                    self.get_logger().info(
                        "Phase INIT: pre-aligning end-effector to forward-facing posture."
                    )
                else:
                    self.task_start_time = self.get_clock().now()
                    self._set_phase(self.PHASE_TRACK)
                    self.get_logger().info("Phase TRACK: executing shared-control trajectory.")
            return

        if self.phase == self.PHASE_INIT:
            t = self._phase_elapsed()
            out = self._prealign_control(tool, flange, t)
            tau = self._rate_and_clip(tau_g + out["tau_task"])
            self._publish_effort(tau)
            if int(t / 0.5) != int(max(t - self.dt, 0.0) / 0.5):
                self.get_logger().info(
                    f"init t={t:.1f}s err={out['tracking_error']*1000:.2f}mm "
                    f"rcm={out['rcm_error']*1000:.2f}mm ori={out['ori_err_deg']:.2f}deg "
                    f"front={out['front_axis_err_deg']:.2f}deg |tau|={np.linalg.norm(tau):.2f}"
                )
            init_done = (
                out["ori_err_deg"] <= self.orientation_init_tol_deg
                and out["front_axis_err_deg"] <= self.orientation_init_tol_deg
                and out["tracking_error"] <= 0.006
            )
            init_timeout = t >= max(self.orientation_ramp_s, self.orientation_init_timeout_s)
            if init_done or init_timeout:
                self.integral_pos[:] = 0.0
                self.integral_rot[:] = 0.0
                self.force_feedback_control = float(self.force_desired)
                self.task_start_time = self.get_clock().now()
                self._set_phase(self.PHASE_TRACK)
                reason = "converged" if init_done else "timeout"
                self.get_logger().info(
                    f"Phase TRACK: executing shared-control trajectory ({reason}, "
                    f"ori={out['ori_err_deg']:.2f}deg, front={out['front_axis_err_deg']:.2f}deg)."
                )
            return

        if self.phase == self.PHASE_TRACK:
            t = (self.get_clock().now() - self.task_start_time).nanoseconds * 1e-9
            out = self._task_control(tool, flange, t)
            tau_task = np.asarray(out["tau_task"], dtype=float)
            tau_pre_realistic = tau_g + tau_task
            tau_after_realistic = tau_pre_realistic.copy()
            if self.realistic_env_enabled and self.real_env is not None:
                tau_after_realistic = self.real_env.tau_command(
                    tau_pre_realistic,
                    progress=t / max(self.task_duration_s, self.dt),
                    gain_override=self.realistic_tau_noise_gain,
                )
            tau = self._rate_and_clip(tau_after_realistic)
            self._publish_effort(tau)
            real_channels = self._realistic_channels(t, out, tau)
            diagnostic_channels = {}
            for key, value in out.items():
                if (
                    key.startswith(("task_", "execution_", "alpha_FP_", "u_pos_"))
                    or key
                    in (
                        "tau_pos_component",
                        "tau_rot_component",
                        "tau_tool_aux_component",
                        "tau_rcm_component",
                        "tau_task_norm",
                    )
                ):
                    arr = np.asarray(value)
                    if arr.dtype.kind in ("b", "i", "u", "f", "c"):
                        diagnostic_channels[key] = value
            for duplicate_key in ("alpha_FP", "alpha_FP_target", "u_pos"):
                diagnostic_channels.pop(duplicate_key, None)
            self.logger_npz.log(
                t=t,
                q=self.q,
                qd=self.qd,
                tool_pos=tool[0],
                flange_pos=flange[0],
                tool_ref=out["tool_ref"],
                nominal_ref=out["nominal"],
                human_ref=out["human"],
                x_tele=out["x_tele"],
                x_reshape=out["x_reshape"],
                x_reshape_vel=out["x_reshape_vel"],
                deformation_norm=out["deformation_norm"],
                x_h=out["x_h"],
                x_h_safe=out["x_h_safe"],
                tau=tau,
                tau_task=tau_task,
                tau_gravity=tau_g,
                tau_pre_realistic=tau_pre_realistic,
                tau_after_realistic=tau_after_realistic,
                tau_realistic_delta=tau_after_realistic - tau_pre_realistic,
                tau_rate_clip_delta=tau - tau_after_realistic,
                tau_gravity_norm=float(np.linalg.norm(tau_g)),
                tau_pre_realistic_norm=float(np.linalg.norm(tau_pre_realistic)),
                tau_after_realistic_norm=float(np.linalg.norm(tau_after_realistic)),
                tau_command_norm=float(np.linalg.norm(tau)),
                u_pos=out["u_pos"],
                u_rot=out["u_rot"],
                u_tool_aux=out["u_tool_aux"],
                u_rcm=out["u_rcm"],
                rcm_reference_scale=out["rcm_reference_scale"],
                ori_err_deg=out["ori_err_deg"],
                front_axis_err_deg=out["front_axis_err_deg"],
                alpha=out["alpha"],
                alpha_FP=out["alpha_FP"],
                alpha_FP_target=out["alpha_FP_target"],
                alpha_raw=out["alpha_raw"],
                alpha_dynamic=out["alpha_dynamic"],
                delta_lambda=out["delta_lambda"],
                beta=out["beta"],
                kappa_N=out["kappa_N"],
                rho_F=out["rho_F"],
                D_c=out["D_c"],
                I_h=out["I_h"],
                T_h=out["T_h"],
                C_h=out["C_h"],
                human_button=out["human_button"],
                human_force_cmd_N=out["human_force_cmd_N"],
                human_event_type_id=out["human_event_type_id"],
                human_raw_force_N=out["human_raw_force_N"],
                human_delta_cmd=out["human_delta_cmd"],
                y_f=out["y_f"],
                y_f_candidate=out["y_f_candidate"],
                y_f_safe=out["y_f_safe"],
                F_min=out["F_min"],
                F_d=out["F_d"],
                F_max=out["F_max"],
                normal_raw=out["normal_raw"],
                normal_safe=out["normal_safe"],
                F_h=out["F_h"],
                D_r=out["D_r"],
                tracking_error=out["tracking_error"],
                rcm_error=out["rcm_error"],
                **diagnostic_channels,
                **real_channels,
            )
            if self.logger_npz.count % 50 == 0:
                self.get_logger().info(
                    f"track t={t:.1f}s err={out['tracking_error']*1000:.2f}mm "
                    f"rcm={out['rcm_error']*1000:.2f}mm alpha={out['alpha']:.2f} "
                    f"alphaFP={out['alpha_FP']:.2f} "
                    f"|tau|={np.linalg.norm(tau):.2f}"
                )
            if t >= self.task_duration_s:
                self._finish()
            return

        self._publish_effort(np.zeros(7))


def main(args=None):
    """ROS2 节点入口。

    关闭时额外发布一次零力矩，避免 Gazebo 中 effort 控制器保持上一帧命令。
    """

    rclpy.init(args=args)
    node = SharedGTGazeboNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_effort(np.zeros(7))
        except BaseException:
            pass
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
