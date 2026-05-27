#!/usr/bin/env python3
import os
import sys
import types
from datetime import datetime

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from franka_msgs.msg import FrankaRobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rcm_controller.pinocchio_model import PinocchioModelHelper

from .alpha_scheduler_gt import (
    ContinuousForceMarginFuzzyAlphaScheduler,
    FixedAlphaScheduler,
    ForceMarginFuzzyAlphaScheduler,
    OnlinePriorityAdaptationAlphaScheduler,
    PhaseAwareFuzzyAlphaScheduler,
)
from .env_estimator import EnvironmentEstimator
from .gt_controller import CooperativeGameController
from .leaky_integrator import LeakyIntegrator
from .ros2_utils import DataLogger, VirtualStiffnessSurface


class Config:
    """Copied from legacy_0526_controller/run_no_rcm.py without value changes."""

    scan_start_x = 0.40
    scan_end_x = 0.48
    scan_y = 0.0
    scan_z = 0.298
    approach_z = 0.30
    scan_vx = 0.002

    F_desired = 1.0
    F_min = 0.3
    F_max = 2.0
    force_axis = 2

    stiffness_zones = [
        (0.40, 0.44, 300, 5),
        (0.44, 0.48, 500, 8),
    ]
    ctrl_rate = 100
    dt = 1.0 / ctrl_rate
    eps_r = 1.0
    eps_f = 2.0
    settle_time = 2.0
    contact_force_threshold = 0.3
    contact_z_tolerance = 0.001
    contact_settle_time = 1.0
    contact_stable_z_std = 0.0003
    contact_stable_force_std = 0.05
    contact_stable_force_slope = 0.20
    contact_force_blend_time = 0.5
    orientation_ramp_time = 1.0
    approach_xy_ramp_time = 2.0


INIT_JOINTS = np.array([
    -0.03572926, -0.71236292, -0.05355629,
    -2.31286173, 0.04212054, 1.5332542, 0.71300622,
], dtype=float)


def euler_angle_diff(a, b):
    diff = b - a
    wrapped = diff % (2 * np.pi)
    wrapped[wrapped > np.pi] -= 2 * np.pi
    return wrapped


class NoRCMForcePositionNode(Node):
    """ROS 2 adapter for the legacy no-RCM force-position controller."""

    PHASE_WAIT = 0
    PHASE_HOME = 1
    PHASE_APPROACH = 2
    PHASE_SCAN = 3
    PHASE_DONE = 4

    def __init__(self):
        super().__init__("no_rcm_force_position_node")
        self.cfg = Config()

        # ROS 2 adapter parameters only. Experiment/control parameters stay in
        # Config above to match 0526_controller/run_no_rcm.py.
        self.declare_parameter("enable_control", False)
        self.declare_parameter("cmd_topic", "/joint_group_effort_controller/commands")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("robot_state_topic", "/franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("rsp_node", "/robot_state_publisher")
        self.declare_parameter("watchdog_sec", 0.2)
        self.declare_parameter(
            "controlled_joints",
            [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
        )
        self.declare_parameter("reference_frame", "fr3_link8")
        self.declare_parameter("tip_frame", "fr3_link11")

        self.declare_parameter("strategy", "continuous_force_margin")
        self.declare_parameter("controller_mode", "are")
        self.declare_parameter("gains_file", "")
        self.declare_parameter("save_gains_file", "")
        self.declare_parameter("use_virtual_env", True)

        self.declare_parameter("max_tau_abs", 87.0)
        self.declare_parameter("max_tau_rate", 1000.0)
        self.declare_parameter("kp_ori", 12.0)
        self.declare_parameter("kd_ori", 1.0)
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("enable_npz_log", True)

        self.enable_control = bool(self.get_parameter("enable_control").value)
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.robot_state_topic = self.get_parameter("robot_state_topic").value
        self.rsp_node = self.get_parameter("rsp_node").value
        self.rate_hz = float(self.cfg.ctrl_rate)
        self.watchdog_sec = float(self.get_parameter("watchdog_sec").value)
        self.ctrl_joints = list(self.get_parameter("controlled_joints").value)
        self.reference_frame = self.get_parameter("reference_frame").value
        self.tip_frame = self.get_parameter("tip_frame").value

        self.scan_start_x = float(self.cfg.scan_start_x)
        self.scan_end_x = float(self.cfg.scan_end_x)
        self.scan_y = float(self.cfg.scan_y)
        self.scan_z = float(self.cfg.scan_z)
        self.surface_z = float(self.cfg.approach_z)
        self.approach_speed = 0.005
        self.scan_vx = float(self.cfg.scan_vx)

        self.force_desired = float(self.cfg.F_desired)
        self.force_min = float(self.cfg.F_min)
        self.force_max = float(self.cfg.F_max)
        self.contact_force_threshold = float(self.cfg.contact_force_threshold)
        self.contact_z_tolerance = float(self.cfg.contact_z_tolerance)
        self.contact_settle_sec = float(self.cfg.contact_settle_time)
        self.contact_stable_z_std = float(self.cfg.contact_stable_z_std)
        self.contact_stable_force_std = float(self.cfg.contact_stable_force_std)
        self.contact_stable_force_slope = float(self.cfg.contact_stable_force_slope)
        self.force_blend_sec = float(self.cfg.contact_force_blend_time)
        self.orientation_ramp_sec = float(self.cfg.orientation_ramp_time)
        self.approach_xy_ramp_sec = float(self.cfg.approach_xy_ramp_time)
        self.use_virtual_env = bool(self.get_parameter("use_virtual_env").value)
        self.capture_ref_after_sec = float(self.cfg.settle_time)
        self.max_tau_abs = float(self.get_parameter("max_tau_abs").value)
        self.max_tau_rate = float(self.get_parameter("max_tau_rate").value)
        self.kp_ori = float(self.get_parameter("kp_ori").value)
        self.kd_ori = float(self.get_parameter("kd_ori").value)
        self.output_dir = self.get_parameter("output_dir").value
        self.enable_npz_log = bool(self.get_parameter("enable_npz_log").value)

        self._dt = 1.0 / max(self.rate_hz, 1.0)
        self._q_meas = np.zeros(len(self.ctrl_joints))
        self._qd_meas = np.zeros(len(self.ctrl_joints))
        self._real_force_z = 0.0
        self._last_robot_state_time = None
        self._last_state_time = None
        self._tau_prev = np.zeros(len(self.ctrl_joints))
        self._phase = self.PHASE_WAIT
        self._t0 = self.get_clock().now()
        self._phase_t0 = None
        self._scan_t0 = None
        self._x_scan = self.scan_start_x
        self._z_ref_approach = None
        self._xy_ref_approach_start = None
        self._xy_ref_approach_target = None
        self._ref_R = None
        self._ref_euler_fixed = None
        self._integ_euler = np.zeros(3)
        self._prev_tool_euler = None
        self._prev_tool_euler_time = None
        self._ori_debug_count = 0
        self._prev_ef = 0.0
        self._prev_er = 0.0
        self._prev_K = 500.0
        self._contact_settle_start = None
        self._surface_touched = False
        self._settle_forces = []
        self._settle_z_values = []
        self._settle_window = max(3, int(self.cfg.contact_settle_time * self.cfg.ctrl_rate))
        self._approach_count = 0
        self._approach_timeout = None
        self._home_started = False
        self._home_kp = np.array([35.0, 35.0, 35.0, 35.0, 18.0, 12.0, 10.0])
        self._home_kd = np.array([6.0, 6.0, 6.0, 6.0, 3.0, 2.0, 1.5])
        # self._home_kp = np.array([60.0, 60.0, 60.0, 60.0, 35.0, 25.0, 20.0])
        # self._home_kd = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 4.0, 3.0])

        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.sub = self.create_subscription(JointState, self.state_topic, self._on_joint_state, 10)
        self.robot_state_sub = self.create_subscription(
            FrankaRobotState, self.robot_state_topic, self._on_robot_state, 10
        )

        self.pm = PinocchioModelHelper(
            node=self,
            rsp_node=self.rsp_node,
            controlled_joints=self.ctrl_joints,
            reference_frame=self.reference_frame,
            tip_frame=self.tip_frame,
        )
        if not self.pm.load():
            self.get_logger().error("Pinocchio model not ready. Node will publish zero torque.")

        self._install_rospy_log_shim()
        self.ctrl = CooperativeGameController(
            control_mode=self.get_parameter("controller_mode").value
        )
        self._load_or_precompute_gains()
        self.scheduler = self._make_scheduler(self.get_parameter("strategy").value)
        self.estimator = EnvironmentEstimator()
        self.sigma_f_int = LeakyIntegrator(eps=2.0, dt=self._dt, dim=3)
        self.logger_npz = DataLogger()
        self.venv = VirtualStiffnessSurface(self._parse_stiffness_zones()) if self.use_virtual_env else None

        self.timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f"no-RCM ROS 2 node ready: enable_control={self.enable_control}, "
            f"cmd={self.cmd_topic}, state={self.state_topic}, tip={self.tip_frame}"
        )
        if not self.enable_control:
            self.get_logger().warn("enable_control is false; publishing zero torque only.")

    def _install_rospy_log_shim(self):
        """Keep legacy controller logging calls working without editing them."""
        if "rospy" in sys.modules:
            return
        shim = types.SimpleNamespace(
            loginfo=lambda msg: self.get_logger().info(str(msg)),
            logwarn=lambda msg: self.get_logger().warn(str(msg)),
            logerr=lambda msg: self.get_logger().error(str(msg)),
        )
        sys.modules["rospy"] = shim

    def _parse_stiffness_zones(self):
        return list(self.cfg.stiffness_zones)

    def _make_scheduler(self, name):
        schedulers = {
            "fixed_08": lambda: FixedAlphaScheduler(0.8),
            "fixed_05": lambda: FixedAlphaScheduler(0.5),
            "fixed_02": lambda: FixedAlphaScheduler(0.2),
            "coop_fuzzy": lambda: PhaseAwareFuzzyAlphaScheduler(dt=self._dt),
            "force_margin": lambda: ForceMarginFuzzyAlphaScheduler(
                dt=self._dt,
                F_min=self.force_min,
                F_max=self.force_max,
                F_desired=self.force_desired,
            ),
            "continuous_force_margin": lambda: ContinuousForceMarginFuzzyAlphaScheduler(
                dt=self._dt,
                F_min=self.force_min,
                F_max=self.force_max,
                F_desired=self.force_desired,
            ),
            "online_priority": lambda: OnlinePriorityAdaptationAlphaScheduler(
                dt=self._dt,
                F_min=self.force_min,
                F_max=self.force_max,
                F_desired=self.force_desired,
            ),
        }
        if name not in schedulers:
            self.get_logger().warn(f"Unknown strategy '{name}', using online_priority.")
            name = "online_priority"
        return schedulers[name]()

    def _load_or_precompute_gains(self):
        gains_file = self.get_parameter("gains_file").value
        if gains_file and os.path.exists(gains_file):
            self.ctrl.load_gains(gains_file)
            if self.ctrl.has_precomputed_gains():
                self.get_logger().info(f"Loaded gains from {gains_file}")
                return
            self.get_logger().warn(f"Gains file {gains_file} had no usable gains; recomputing.")

        Ke_vals = sorted(set([300, 500, 50, 80, 100, 150, 200, 800, 1000, 1500, 2000, 3000, 5000]))
        self.ctrl.precompute_gains(alpha_grid=np.linspace(0.0, 1.0, 21), Ke_grid=Ke_vals)
        save_gains_file = self.get_parameter("save_gains_file").value
        if save_gains_file:
            os.makedirs(os.path.dirname(save_gains_file) or ".", exist_ok=True)
            self.ctrl.save_gains(save_gains_file)
            self.get_logger().info(f"Saved gains to {save_gains_file}")

    def _on_joint_state(self, msg):
        name_to_i = {n: i for i, n in enumerate(msg.name)}
        for k, joint_name in enumerate(self.ctrl_joints):
            idx = name_to_i.get(joint_name)
            if idx is None:
                self.get_logger().error(f"missing joint {joint_name} in joint_states")
                return
            if idx < len(msg.position):
                self._q_meas[k] = float(msg.position[idx])
            if idx < len(msg.velocity):
                self._qd_meas[k] = float(msg.velocity[idx])
        self._last_state_time = self.get_clock().now()

    def _on_robot_state(self, msg):
        self._real_force_z = float(msg.o_f_ext_hat_k.wrench.force.z)
        self._last_robot_state_time = self.get_clock().now()

    def _elapsed(self):
        return (self.get_clock().now() - self._t0).nanoseconds * 1e-9

    def _phase_elapsed(self):
        if self._phase_t0 is None:
            return 0.0
        return (self.get_clock().now() - self._phase_t0).nanoseconds * 1e-9

    def _state_valid(self):
        if self.pm.model is None or self.pm.data is None:
            return False
        if self._last_state_time is None:
            return False
        age = (self.get_clock().now() - self._last_state_time).nanoseconds * 1e-9
        return age <= self.watchdog_sec

    def _publish_tau(self, tau):
        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.pub.publish(msg)

    def _publish_zero(self):
        self._tau_prev[:] = 0.0
        self._publish_tau(np.zeros(len(self.ctrl_joints)))

    def _rate_and_clip(self, tau):
        tau = np.clip(tau, -self.max_tau_abs, self.max_tau_abs)
        max_delta = self.max_tau_rate * self._dt
        delta = np.clip(tau - self._tau_prev, -max_delta, max_delta)
        return self._tau_prev + delta

    def _compute_tau_home(self, q, qd):
        tau = -self._home_kp * (q - INIT_JOINTS) - self._home_kd * qd
        tau = self._rate_and_clip(tau)
        self._tau_prev = tau.copy()
        return tau

    def _frame_state(self):
        q, qd = self.pm.build_full_state(self._q_meas, self._qd_meas)
        p, R, J6, Jv, Jw, v = self.pm.get_frame_state(q, qd, self.pm.tip_frame_id)
        w = Jw @ qd
        euler = Rotation.from_matrix(R).as_euler("xyz", degrees=False)
        now = self.get_clock().now()
        if self._prev_tool_euler is None or self._prev_tool_euler_time is None:
            euler_vel = np.zeros(3)
        else:
            dt_euler = (now - self._prev_tool_euler_time).nanoseconds * 1e-9
            if dt_euler <= 0.0 or not np.isfinite(dt_euler):
                dt_euler = self._dt
            euler_vel = -euler_angle_diff(euler, self._prev_tool_euler) / dt_euler
        self._prev_tool_euler = euler.copy()
        self._prev_tool_euler_time = now
        return q, qd, p, R, Jv, Jw, v, w, euler, euler_vel

    def _contact_delta(self, z):
        return max(0.0, self.surface_z - float(z))

    def _virtual_force(self, p, v):
        if self.venv is None:
            return 0.0
        return self.venv.compute_force(p[0], self._contact_delta(p[2]), -v[2])

    def _compute_u_rotation(self, euler, euler_vel):
        if self._ref_euler_fixed is None:
            return np.zeros(3)
        e_euler = euler_angle_diff(euler, self._ref_euler_fixed)
        e_euler_dot = euler_angle_diff(euler_vel, np.zeros(3))
        self._integ_euler = self._integ_euler + e_euler * self._dt
        self._integ_euler = np.clip(self._integ_euler, -0.15, 0.15)
        return (
            self.ctrl.P_ori * e_euler
            + self.ctrl.D_ori * e_euler_dot
            + self.ctrl.I_ori * self._integ_euler
        )
        # return np.zeros(3)

    def _compute_u_rotation_so3(self, R, w):
        if self._ref_R is None:
            return np.zeros(3), np.zeros(3)

        R_err = self._ref_R @ R.T
        e_R = pin.log3(R_err)
        self._integ_euler = self._integ_euler + e_R * self._dt
        self._integ_euler = np.clip(self._integ_euler, -0.15, 0.15)
        u_rot = (
            self.ctrl.P_ori * e_R
            - self.ctrl.D_ori * w
            + self.ctrl.I_ori * self._integ_euler
        )
        return u_rot, e_R

    def _orientation_ramp(self):
        if self._phase_t0 is None:
            return 0.0
        ramp_time = max(self.orientation_ramp_sec, 1e-9)
        return min(1.0, max(0.0, self._phase_elapsed() / ramp_time))

    def _approach_xy_reference(self):
        target = np.array([self.scan_start_x, self.scan_y], dtype=float)
        if self._xy_ref_approach_start is None:
            return target, np.zeros(2)

        ramp_time = max(self.approach_xy_ramp_sec, 1e-9)
        s = min(1.0, max(0.0, self._phase_elapsed() / ramp_time))
        xy_ref = (1.0 - s) * self._xy_ref_approach_start + s * target
        if s < 1.0:
            xy_vel = (target - self._xy_ref_approach_start) / ramp_time
        else:
            xy_vel = np.zeros(2)
        return xy_ref, xy_vel

    def _compute_tau(self, e_r1, e_r2, e_f, sigma_f, alpha, K_hat, Jv, Jw, R, w):
        u_tool, K_eff = self.ctrl.compute_control(e_r1, e_r2, e_f, sigma_f, alpha, K_hat)
        if hasattr(self.ctrl, "no_rcm_u_tool_limits"):
            limits = np.asarray(self.ctrl.no_rcm_u_tool_limits, dtype=float)
            if limits.shape == (3,):
                u_tool = np.clip(u_tool, -limits, limits)

        u_rot, e_R = self._compute_u_rotation_so3(R, w)
        ori_ramp = self._orientation_ramp()
        u_rot = ori_ramp * u_rot
        self._ori_debug_count += 1
        if self._ori_debug_count % 20 == 0:
            self.get_logger().info(
                "ori_so3 | "
                f"e_R={np.array2string(e_R, precision=4, suppress_small=True)} "
                f"w={np.array2string(w, precision=4, suppress_small=True)} "
                f"u_rot={np.array2string(u_rot, precision=4, suppress_small=True)} "
                f"|u_rot|={np.linalg.norm(u_rot):.4f} "
                f"|u_tool|={np.linalg.norm(u_tool):.4f} "
                f"ramp={ori_ramp:.2f}"
            )
        u_cart = np.hstack([u_tool, u_rot])
        cart_norm = np.linalg.norm(u_cart)
        if cart_norm > self.ctrl.u_threshold:
            u_cart = u_cart * self.ctrl.u_threshold / cart_norm
        J6 = np.vstack([Jv, Jw])
        tau = np.clip((J6.T @ u_cart).flatten(), -self.ctrl.tau_max, self.ctrl.tau_max)
        self._tau_prev = tau.copy()
        return tau, u_tool, K_eff

    def _start_approach(self, p, R, euler):
        self._phase = self.PHASE_APPROACH
        self._phase_t0 = self.get_clock().now()
        self._z_ref_approach = float(p[2])
        self._xy_ref_approach_start = np.array([p[0], p[1]], dtype=float)
        self._xy_ref_approach_target = np.array([self.scan_start_x, self.scan_y], dtype=float)
        self._ref_R = R.copy()
        self._ref_euler_fixed = euler.copy()
        self._integ_euler = np.zeros(3)
        self._surface_touched = False
        self._contact_settle_start = None
        self._settle_forces = []
        self._settle_z_values = []
        self._approach_count = 0
        self._approach_timeout = max(
            20.0,
            1.5 * max(0.0, self._z_ref_approach - self.scan_z) / self.approach_speed + 5.0,
        )
        self.estimator.reset()
        self.scheduler.reset()
        self.sigma_f_int.reset()
        self.get_logger().info(
            f"Starting approach from z={p[2]:.4f} toward scan_z={self.scan_z:.4f}, "
            f"surface_z={self.surface_z:.4f}, timeout={self._approach_timeout:.1f}s, "
            f"xy_ref_start=[{self._xy_ref_approach_start[0]:.4f},{self._xy_ref_approach_start[1]:.4f}]"
        )

    def _start_home(self):
        self._phase = self.PHASE_HOME
        self._phase_t0 = self.get_clock().now()
        self._home_started = True
        self.get_logger().info("Moving to INIT_JOINTS before approach.")

    def _start_scan(self):
        self._phase = self.PHASE_SCAN
        self._phase_t0 = self.get_clock().now()
        self._scan_t0 = self.get_clock().now()
        self._x_scan = self.scan_start_x
        self.sigma_f_int.reset()
        self._integ_euler = np.zeros(3)
        self.get_logger().info("Starting no-RCM force-position scan.")

    def _finish_scan(self):
        self._phase = self.PHASE_DONE
        self._phase_t0 = self.get_clock().now()
        self._publish_zero()
        if self.enable_npz_log and self.logger_npz.count > 0:
            os.makedirs(self.output_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.output_dir, f"no_rcm_ros2_{stamp}.npz")
            self.logger_npz.save(path)
            self.get_logger().info(f"Saved no-RCM log to {path}")
        if hasattr(self.scheduler, "set_retreat"):
            self.scheduler.set_retreat(True)
        self.get_logger().info("Scan complete. Zero torque published.")

    def _on_timer(self):
        if not self._state_valid() or not self.enable_control:
            self._publish_zero()
            return

        q, qd, p, R, Jv, Jw, v, w, euler, euler_vel = self._frame_state()

        if self._phase == self.PHASE_WAIT:
            if self._elapsed() >= self.capture_ref_after_sec:
                # self._start_approach(p, R, euler)
                self._start_home()
            else:
                self._publish_zero()
            return

        if self._phase == self.PHASE_HOME:
            tau = self._compute_tau_home(q, qd)
            self._publish_tau(tau)
            q_err = float(np.linalg.norm(q - INIT_JOINTS))
            qd_norm = float(np.linalg.norm(qd))
            if int(self._phase_elapsed() / 0.5) != int((self._phase_elapsed() - self._dt) / 0.5):
                self.get_logger().info(
                    f"homing t={self._phase_elapsed():.2f}s "
                    f"q_err={q_err:.4f} qd_norm={qd_norm:.4f} "
                    f"tcp=[{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}]"
                )
            if self._phase_elapsed() >= self.capture_ref_after_sec and q_err < 0.08 and qd_norm < 0.15:
                self._start_approach(p, R, euler)
            return

        F_z = self._real_force_z

        if self._phase == self.PHASE_APPROACH:
            self._z_ref_approach -= self.approach_speed * self._dt
            if not self._surface_touched and abs(F_z) > self.contact_force_threshold:
                self._surface_touched = True
                self.get_logger().info(
                    f"Surface contact detected at z={p[2]:.4f}, F={F_z:.3f}N"
                )
            if self._surface_touched:
                self._z_ref_approach = max(self._z_ref_approach, self.scan_z)

            xy_ref, xy_vel = self._approach_xy_reference()
            x_ref = np.array([xy_ref[0], xy_ref[1], self._z_ref_approach])
            xdot_ref = np.array([xy_vel[0], xy_vel[1], -self.approach_speed])
            if self._surface_touched and self._z_ref_approach <= self.scan_z:
                xdot_ref[2] = 0.0

            e_r1 = p - x_ref
            e_r2 = v - xdot_ref
            e_f = np.zeros(3)
            sigma_f = self.sigma_f_int.get()
            tau, _, _ = self._compute_tau(
                e_r1, e_r2, e_f, sigma_f, 1.0, 500.0, Jv, Jw, R, w
            )
            self._publish_tau(tau)

            self._approach_count += 1
            if self._approach_count % 10 == 0:
                self.get_logger().info(
                    f"approaching | z={p[2]:.4f}m, z_ref={self._z_ref_approach:.4f}m, "
                    f"x={p[0]:.4f}/{x_ref[0]:.4f}, "
                    f"target_z<={self.scan_z:.4f}m, F={F_z:.3f}N, "
                    f"vz={v[2]:+.4f}m/s, touched={self._surface_touched}"
                )

            at_height = p[2] <= self.scan_z + self.contact_z_tolerance
            if self._surface_touched and at_height:
                if self._contact_settle_start is None:
                    self._contact_settle_start = self.get_clock().now()
                    self._settle_forces = []
                    self._settle_z_values = []
                    self.get_logger().info(
                        f"Contact height reached at z={p[2]:.4f}; settling..."
                    )
                self._settle_forces.append(abs(F_z))
                self._settle_z_values.append(float(p[2]))
                if len(self._settle_forces) > self._settle_window:
                    self._settle_forces.pop(0)
                    self._settle_z_values.pop(0)

                settle_t = (self.get_clock().now() - self._contact_settle_start).nanoseconds * 1e-9
                force_std = float(np.std(self._settle_forces)) if len(self._settle_forces) > 1 else float("inf")
                z_std = float(np.std(self._settle_z_values)) if len(self._settle_z_values) > 1 else float("inf")
                z_err = abs(float(p[2]) - self.scan_z)
                force_slope = 0.0
                if len(self._settle_forces) > 1:
                    force_slope = abs(self._settle_forces[-1] - self._settle_forces[0]) / max(
                        (len(self._settle_forces) - 1) * self._dt, self._dt
                    )
                stable = (
                    settle_t >= self.contact_settle_sec
                    and z_err <= self.contact_z_tolerance
                    and z_std <= self.contact_stable_z_std
                    and force_std <= self.contact_stable_force_std
                    and force_slope <= self.contact_stable_force_slope
                )
                if self._approach_count % 10 == 0:
                    self.get_logger().info(
                        f"settling | t={settle_t:.2f}s, z_err={z_err*1000:.2f}mm, "
                        f"z_std={z_std*1000:.3f}mm, F={abs(F_z):.3f}N, "
                        f"F_std={force_std:.3f}, F_slope={force_slope:.3f}N/s"
                    )
                if stable:
                    self.get_logger().info(
                        f"Contact settled at z={p[2]:.4f}, F={abs(F_z):.3f}N, "
                        f"z_err={z_err*1000:.2f}mm, z_std={z_std*1000:.3f}mm"
                    )
                    self._start_scan()
            else:
                self._contact_settle_start = None
                self._settle_forces = []
                self._settle_z_values = []

            if self._approach_timeout is not None and self._phase_elapsed() > self._approach_timeout:
                self.get_logger().warn(
                    f"Approach timeout (z={p[2]:.4f}, z_ref={self._z_ref_approach:.4f}, "
                    f"target_z<={self.scan_z:.4f}, F={F_z:.3f}N, "
                    f"touched={self._surface_touched}, timeout={self._approach_timeout:.1f}s)"
                )
                self._finish_scan()
            return

        if self._phase == self.PHASE_SCAN:
            t_scan = (self.get_clock().now() - self._scan_t0).nanoseconds * 1e-9
            delta = self._contact_delta(p[2])
            delta_dot = -v[2]
            if self.venv is not None:
                K_env_true, B_env_true = self.venv.get_stiffness(p[0])
            else:
                K_env_true, B_env_true = 0.0, 0.0
            K_hat, B_hat = self.estimator.update(
                abs(F_z),
                delta,
                delta_dot,
            )

            x_ref = np.array([self._x_scan, self.scan_y, self.scan_z])
            xdot_ref = np.array([self.scan_vx, 0.0, 0.0])
            F_des = np.array([0.0, 0.0, self.force_desired])
            F_meas = np.array([0.0, 0.0, abs(F_z)])
            e_r1 = p - x_ref
            e_r2 = v - xdot_ref
            e_f_vec = F_meas - F_des

            force_blend = min(1.0, max(0.0, t_scan / max(self.force_blend_sec, 1e-9)))
            e_f_vec_ctrl = force_blend * e_f_vec
            sigma_f = self.sigma_f_int.update(e_f_vec_ctrl)

            e_f_scalar = abs(F_z) - self.force_desired
            e_f_dot = (e_f_scalar - self._prev_ef) / self._dt
            self._prev_ef = e_f_scalar
            e_r_scalar = float(np.linalg.norm(p[:2] - np.array([self._x_scan, self.scan_y])))
            de_r = (e_r_scalar - self._prev_er) / self._dt
            self._prev_er = e_r_scalar
            dK = (K_hat - self._prev_K) / self._dt
            self._prev_K = K_hat

            if isinstance(self.scheduler, FixedAlphaScheduler):
                alpha = self.scheduler.compute()
                phase_val = -1
            else:
                alpha = self.scheduler.compute(
                    F_norm=abs(F_z),
                    e_f=e_f_scalar,
                    K_hat=K_hat,
                    e_r=e_r_scalar,
                    z_vel=v[2],
                    de_f=e_f_dot,
                    dK=dK,
                    de_r=de_r,
                    F_desired=self.force_desired,
                    F_min=self.force_min,
                    F_max=self.force_max,
                )
                phase_val = self.scheduler.phase_detector.phase.value

            tau, u_tool, K_eff = self._compute_tau(
                e_r1, e_r2, e_f_vec_ctrl, sigma_f, alpha, K_hat, Jv, Jw, R, w
            )
            self._publish_tau(tau)
            self._x_scan += self.scan_vx * self._dt

            pos_err = p - x_ref
            F_err = abs(F_z) - self.force_desired
            self.logger_npz.log(
                t=t_scan,
                pos=p,
                pos_des=x_ref,
                pos_err=pos_err,
                F_measured=abs(F_z),
                F_desired=self.force_desired,
                F_err=F_err,
                e_f=e_f_scalar,
                e_r=e_r_scalar,
                sigma_f_norm=np.linalg.norm(sigma_f),
                e_r1_norm=np.linalg.norm(e_r1),
                alpha=alpha,
                K_hat=K_hat,
                K_eff=K_eff,
                x_desired=self._x_scan,
                u_norm=np.linalg.norm(tau),
                phase=phase_val,
                v_x=v[0],
                xdot_ref_x=xdot_ref[0],
                e_r1_x=e_r1[0],
                e_r2_x=e_r2[0],
                u_tool_x=u_tool[0],
                u_tool_norm=np.linalg.norm(u_tool),
                v_z=v[2],
                delta=delta,
                delta_dot=delta_dot,
                K_env_true=K_env_true,
                B_env_true=B_env_true,
                B_hat=B_hat,
            )

            if self.logger_npz.count % 10 == 0:
                self.get_logger().info(
                    f"t={t_scan:5.2f}s | "
                    f"tool=[{p[0]*1000:6.2f},{p[1]*1000:6.2f},{p[2]*1000:6.2f}]mm | "
                    f"des=[{x_ref[0]*1000:6.2f},{x_ref[1]*1000:6.2f},{x_ref[2]*1000:6.2f}]mm | "
                    f"err={np.linalg.norm(pos_err)*1000:5.2f}mm | "
                    f"F={abs(F_z):.3f}N (des={self.force_desired:.3f}, err={F_err:+.3f}) | "
                    f"K={K_hat:7.1f} B={B_hat:5.2f} | "
                    f"e_f={e_f_scalar:+.3f} e_r={e_r_scalar*1000:5.2f}mm | "
                    f"alpha={alpha:.2f} phase={phase_val} | "
                    f"vx={v[0]:+.4f}/{xdot_ref[0]:+.4f} "
                    f"ex={e_r1[0]*1000:+.2f}mm evx={e_r2[0]:+.4f} "
                    f"u_x={u_tool[0]:+.3f} "
                    f"delta={delta*1000:.2f}mm ddot={delta_dot:+.4f} "
                    f"Ktrue={K_env_true:.1f}"
                )

            if self._x_scan >= self.scan_end_x:
                self._finish_scan()
            return

        self._publish_zero()


def main(args=None):
    rclpy.init(args=args)
    node = NoRCMForcePositionNode()
    try:
        rclpy.spin(node)
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
