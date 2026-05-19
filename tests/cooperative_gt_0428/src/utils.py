"""
通用工具: 虚拟接触环境, 力传感器输入, 数据记录
"""
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped


class VirtualStiffnessSurface:
    """
    分段 Kelvin-Voigt 虚拟接触面
    zones: [(x_start, x_end, K_e, B_e), ...]
    """

    def __init__(self, zones):
        self.zones = zones

    def get_stiffness(self, x):
        for x0, x1, K, B in self.zones:
            if x0 <= x < x1:
                return K, B
        return self.zones[-1][2], self.zones[-1][3]

    def compute_force(self, x, delta, delta_dot):
        """单向接触: δ ≤ 0 时无力"""
        if delta <= 0:
            return 0.0
        K, B = self.get_stiffness(x)
        return K * delta + B * delta_dot


class ForceSensorInput:
    """
    六维力传感器 ROS 输入。

    force_sensor_ros_node.py 发布 geometry_msgs/WrenchStamped，本类只负责订阅、
    新鲜度判定和按指定轴提取接触力。若 timeout 内没有新数据，控制层会回退
    到 Kelvin-Voigt 虚拟环境。
    """

    def __init__(self, topic="/force_sensor/wrench", timeout=0.2,
                 force_axis=2, force_sign=1.0):
        if force_axis not in (0, 1, 2):
            raise ValueError("force_axis must be 0, 1, or 2")
        self.topic = topic
        self.timeout = float(timeout)
        self.force_axis = int(force_axis)
        self.force_sign = float(force_sign)
        self._force = np.zeros(3)
        self._torque = np.zeros(3)
        self._stamp = None
        self._seq = 0
        self._sub = rospy.Subscriber(
            self.topic, WrenchStamped, self._callback, queue_size=1
        )

    def _callback(self, msg):
        force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        ], dtype=float)
        torque = np.array([
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ], dtype=float)
        if not (np.all(np.isfinite(force)) and np.all(np.isfinite(torque))):
            return
        self._force = force
        self._torque = torque
        self._stamp = rospy.Time.now()
        self._seq += 1

    def available(self):
        if self._stamp is None:
            return False
        age = (rospy.Time.now() - self._stamp).to_sec()
        return age <= self.timeout

    def age(self):
        if self._stamp is None:
            return float("inf")
        return (rospy.Time.now() - self._stamp).to_sec()

    def seq(self):
        return self._seq

    def wait_for_data(self, timeout=2.0):
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.available():
                return True
            rate.sleep()
        return self.available()

    def force_vector(self):
        return self._force.copy()

    def wrench_vector(self):
        return np.hstack([self._force, self._torque])

    def signed_axis_force(self):
        return self.force_sign * self._force[self.force_axis]

    def contact_force(self):
        """返回按本文 +z 接触反力约定使用的非负标量力。"""
        return abs(self.signed_axis_force())


class FirstOrderLowPass:
    """一阶低通滤波器，用于抑制力/刚度等测量噪声。"""

    def __init__(self, tau, initial=None):
        self.tau = float(tau)
        self._value = None if initial is None else np.asarray(initial, dtype=float)

    def reset(self, value=None):
        self._value = None if value is None else np.asarray(value, dtype=float)

    def update(self, value, dt):
        value = np.asarray(value, dtype=float)
        if self._value is None:
            self._value = value.copy()
            return self._value.copy()
        beta = float(np.clip(dt / max(self.tau, dt), 0.0, 1.0))
        self._value = self._value + beta * (value - self._value)
        return self._value.copy()


class VectorRateLimiter:
    """按元素限制向量每秒变化率，避免力矩命令跳变激发高频抖动。"""

    def __init__(self, max_rate, initial=None):
        self.max_rate = float(max_rate)
        self._value = None if initial is None else np.asarray(initial, dtype=float)

    def reset(self, value=None):
        self._value = None if value is None else np.asarray(value, dtype=float)

    def update(self, value, dt):
        value = np.asarray(value, dtype=float)
        if self._value is None:
            self._value = value.copy()
            return self._value.copy()
        max_step = self.max_rate * max(float(dt), 1e-6)
        step = np.clip(value - self._value, -max_step, max_step)
        self._value = self._value + step
        return self._value.copy()


class DataLogger:
    """时序数据记录器"""

    def __init__(self):
        self._d = {
            't': [],
            # 当前 tool 位置 (3 轴)
            'pos_x': [], 'pos_y': [], 'pos_z': [],
            # 期望 tool 位置 (3 轴)
            'pos_des_x': [], 'pos_des_y': [], 'pos_des_z': [],
            # tool 位置误差 (3 轴 + 范数)
            'pos_err_x': [], 'pos_err_y': [], 'pos_err_z': [],
            'pos_err_norm': [],
            # 力 (标量)
            'F_measured': [], 'F_desired': [], 'F_err': [],
            # 六维力传感器/力源状态
            'Fx': [], 'Fy': [], 'Fz': [], 'Mx': [], 'My': [], 'Mz': [],
            'force_source': [], 'sensor_available': [],
            # 兼容旧字段
            'e_r': [], 'e_f': [],
            'sigma_f_norm': [], 'e_r1_norm': [],
            'alpha': [], 'K_hat': [],
            'K_total': [], 'K_r2': [], 'K_ef': [], 'K_sf': [],
            'x_desired': [],
            'error_rcm': [], 'error_track': [],
            'u_norm': [], 'phase': [],
        }

    def log(self, **kw):
        for k, v in kw.items():
            if k == 'pos':
                self._d['pos_x'].append(v[0])
                self._d['pos_y'].append(v[1])
                self._d['pos_z'].append(v[2])
            elif k == 'pos_des':
                self._d['pos_des_x'].append(v[0])
                self._d['pos_des_y'].append(v[1])
                self._d['pos_des_z'].append(v[2])
            elif k == 'pos_err':
                self._d['pos_err_x'].append(v[0])
                self._d['pos_err_y'].append(v[1])
                self._d['pos_err_z'].append(v[2])
                self._d['pos_err_norm'].append(float(np.linalg.norm(v)))
            elif k == 'K_eff':
                self._d['K_total'].append(v[0])
                self._d['K_r2'].append(v[1])
                self._d['K_ef'].append(v[2])
                self._d['K_sf'].append(v[3])
            elif k == 'wrench':
                self._d['Fx'].append(v[0])
                self._d['Fy'].append(v[1])
                self._d['Fz'].append(v[2])
                self._d['Mx'].append(v[3])
                self._d['My'].append(v[4])
                self._d['Mz'].append(v[5])
            elif k in self._d:
                self._d[k].append(v)

    @property
    def count(self):
        return len(self._d['t'])

    def save(self, filepath):
        np.savez(filepath, **{k: np.array(v) for k, v in self._d.items() if v})

    @staticmethod
    def load(filepath):
        return dict(np.load(filepath))
