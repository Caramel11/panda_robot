"""第4章共享控制仲裁算法模块。

原 ROS1/0213 脚本根据人类输入强度、干预持续时间和安全距离计算人机权限系数。
本模块把这部分逻辑整理成不依赖 ROS 的纯 Python 形式：

* ``SharedFuzzyArbitrator``：完整方法和动态消融基线使用的动态 ``alpha_HR``。
* ``sigmoid_alpha``：只依赖输入强度的单因素 sigmoid 基线。
* ``compute_gt_gain``：重构 0213 中连续 LQR/博弈控制增益，供任务空间控制器使用。
"""

import numpy as np
from scipy.linalg import solve_continuous_are


class DeltaLambdaUpdater:
    """模糊规则修正项 ``delta_lambda`` 的积分器。"""

    def __init__(self, lambda0=0.5, dt=0.01):
        """初始化对象参数和运行状态。"""
        self.lambda_current = float(lambda0)
        self.dt = float(dt)

    def update(self, delta_lambda):
        """积分修正项，并把结果限制在 [0, 1]。"""
        self.lambda_current += float(delta_lambda) * self.dt
        self.lambda_current = float(np.clip(self.lambda_current, 0.0, 1.0))
        return self.lambda_current


class KalmanFilterFusion:
    """兼容 0213 思路的二状态 lambda 平滑器。

    第一个状态是滤波后的人机权限值，第二个状态是趋势项。输入 ``tau_k`` 包含原始
    lambda 变化率和模糊规则修正项，用于抑制 ``alpha_HR`` 的突变。
    """

    def __init__(self, dt=0.01, epsilon=0.01, alpha=0.9):
        """初始化对象参数和运行状态。"""
        self.y_hat = np.array([[0.5], [0.0]], dtype=float)
        self.P = np.diag([0.001, 0.001])
        self.A = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        self.B = np.array([[epsilon * dt, -epsilon * dt], [epsilon, -epsilon]], dtype=float)
        self.H = np.eye(2)
        self.Q = 0.0004 * np.eye(2)
        self.R = np.eye(2)
        self.alpha = float(alpha)
        self.lambda_prev = 0.5
        self.history_z = []
        self.history_y_hat_prior = []
        self.N = 20

    def update(self, z_k, tau_k):
        """执行一次预测/更新，并返回滤波后的 lambda 值。

        ROS1/0213 中 ``z_k`` 是标量 ``lambda_w``，在 ``z_k - H x`` 中按
        NumPy 广播为二状态观测。这里保留该行为，避免把第二个观测量强行置零。
        """
        tau = np.asarray(tau_k, dtype=float).reshape(2, 1)
        y_prior = self.A @ self.y_hat + self.B @ tau
        self.y_hat_prior = y_prior
        p_prior = self.A @ self.P @ self.A.T + self.Q
        self.history_z.append(float(z_k))
        self.history_y_hat_prior.append(float(y_prior[0, 0]))
        if len(self.history_z) >= self.N:
            z_window = np.asarray(self.history_z[-self.N :], dtype=float)
            y_window = np.asarray(self.history_y_hat_prior[-self.N :], dtype=float)
            residuals = z_window - y_window
            var = float(np.mean(residuals**2))
            self.R = np.eye(2) * max(var, 1e-6)
        s = self.H @ p_prior @ self.H.T + self.R
        k = p_prior @ self.H.T @ np.linalg.inv(s)
        innovation = float(z_k) - self.H @ y_prior
        self.y_hat = y_prior + k @ innovation
        self.P = (np.eye(2) - k @ self.H) @ p_prior
        value = float(np.clip(self.y_hat[0, 0], 0.0, 1.0))
        value = self.alpha * value + (1.0 - self.alpha) * self.lambda_prev
        self.lambda_prev = value
        return value


def _tri(x, a, b, c):
    """紧凑模糊规则使用的三角隶属函数。"""
    x = float(x)
    if x <= a or x >= c:
        return 0.0
    if x <= b:
        return (x - a) / max(b - a, 1e-9)
    return (c - x) / max(c - b, 1e-9)


class FuzzyLogicTool:
    """ROS1/0213 模糊规则的纯 Python 复刻。"""

    def __init__(self, kind="lambda_based"):
        self.kind = str(kind)
        self._init_fuzzy_sets(self.kind)
        self._init_fuzzy_rules(self.kind)

    def _init_fuzzy_sets(self, kind):
        if kind == "lambda_based":
            self.input_sets = [
                {"PS": (0.0, 0.4, 0.8), "PM": (0.4, 1.0, 1.5), "PL": (1.2, 1.5, 2.0)},
                {"PS": (0.0, 1.0, 2.0), "PM": (1.0, 3.0, 5.0), "PL": (3.0, 5.0, 6.0)},
                {"PS": (0.0, 0.02, 0.04), "PM": (0.03, 0.05, 0.07), "PL": (0.06, 0.08, 0.1)},
            ]
            self.output_sets = {
                "Z": (0.0, 0.05, 0.1),
                "PS": (0.05, 0.25, 0.5),
                "PM": (0.25, 0.5, 0.7),
                "P": (0.5, 0.7, 0.95),
                "PL": (0.9, 0.95, 1.0),
            }
        else:
            self.input_sets = [
                {"N": (-3.0, -2.0, -0.2), "Z": (-1.5, 0.0, 1.5), "P": (0.2, 2.0, 3.0)},
                {"N": (0.0, 0.7, 1.0), "Z": (0.7, 1.0, 1.3), "P": (1.0, 1.3, 2.0)},
                {"N": (-0.08, -0.04, -0.01), "Z": (-0.03, 0.0, 0.03), "P": (0.01, 0.04, 0.08)},
            ]
            self.output_sets = {
                "NL": (-0.5, -0.35, -0.2),
                "N": (-0.35, -0.2, -0.0),
                "Z": (-0.2, 0.0, 0.2),
                "P": (0.0, 0.2, 0.35),
                "PL": (0.2, 0.35, 0.5),
            }

    def _init_fuzzy_rules(self, kind):
        if kind == "lambda_based":
            self.rule_dict = {
                ("PS", "PS", "PS"): "Z", ("PS", "PS", "PM"): "Z", ("PS", "PS", "PL"): "PS",
                ("PS", "PM", "PS"): "Z", ("PS", "PM", "PM"): "PS", ("PS", "PM", "PL"): "PS",
                ("PS", "PL", "PS"): "Z", ("PS", "PL", "PM"): "PS", ("PS", "PL", "PL"): "PM",
                ("PM", "PS", "PS"): "Z", ("PM", "PS", "PM"): "PS", ("PM", "PS", "PL"): "PM",
                ("PM", "PM", "PS"): "PS", ("PM", "PM", "PM"): "PM", ("PM", "PM", "PL"): "PM",
                ("PM", "PL", "PS"): "PS", ("PM", "PL", "PM"): "P", ("PM", "PL", "PL"): "PL",
                ("PL", "PS", "PS"): "PS", ("PL", "PS", "PM"): "PM", ("PL", "PS", "PL"): "PM",
                ("PL", "PM", "PS"): "PS", ("PL", "PM", "PM"): "P", ("PL", "PM", "PL"): "PL",
                ("PL", "PL", "PS"): "PM", ("PL", "PL", "PM"): "P", ("PL", "PL", "PL"): "PL",
            }
        else:
            self.rule_dict = {
                ("N", "N", "N"): "NL", ("N", "N", "Z"): "NL", ("N", "N", "P"): "N",
                ("N", "Z", "N"): "NL", ("N", "Z", "Z"): "N", ("N", "Z", "P"): "Z",
                ("N", "P", "N"): "N", ("N", "P", "Z"): "Z", ("N", "P", "P"): "P",
                ("Z", "N", "N"): "NL", ("Z", "N", "Z"): "N", ("Z", "N", "P"): "Z",
                ("Z", "Z", "N"): "N", ("Z", "Z", "Z"): "Z", ("Z", "Z", "P"): "P",
                ("Z", "P", "N"): "Z", ("Z", "P", "Z"): "P", ("Z", "P", "P"): "PL",
                ("P", "N", "N"): "N", ("P", "N", "Z"): "Z", ("P", "N", "P"): "P",
                ("P", "Z", "N"): "Z", ("P", "Z", "Z"): "P", ("P", "Z", "P"): "PL",
                ("P", "P", "N"): "P", ("P", "P", "Z"): "PL", ("P", "P", "P"): "PL",
            }

    @staticmethod
    def triangular_mf(x, params):
        a, b, c = params
        x = float(x)
        if x <= a or x >= c:
            return 0.0
        if x <= b:
            return (x - a) / max(b - a, 1e-9)
        return (c - x) / max(c - b, 1e-9)

    @staticmethod
    def trapezoidal_mf_type1(x, params):
        a, b, c = params
        x = float(x)
        if x < a or x > c:
            return 0.0
        if x <= b:
            return 1.0
        return (c - x) / max(c - b, 1e-9)

    @staticmethod
    def trapezoidal_mf_type2(x, params):
        a, b, c = params
        x = float(x)
        if x < a or x > c:
            return 0.0
        if x <= b:
            return (x - a) / max(b - a, 1e-9)
        return 1.0

    def fuzzify(self, inputs, kind=None):
        kind = self.kind if kind is None else str(kind)
        if kind == "lambda_based":
            inputs = [np.clip(inputs[0], 0.0, 2.0), np.clip(inputs[1], 0.0, 6.0), np.clip(inputs[2], 0.0, 0.1)]
        else:
            inputs = [np.clip(inputs[0], -3.0, 3.0), np.clip(inputs[1], 0.0, 2.0), np.clip(inputs[2], -0.08, 0.08)]
        fuzzified = []
        for input_val, sets in zip(inputs, self.input_sets):
            membership = {}
            for label, params in sets.items():
                if label in ("PS", "N"):
                    membership[label] = self.trapezoidal_mf_type1(input_val, params)
                elif label in ("PL", "P"):
                    membership[label] = self.trapezoidal_mf_type2(input_val, params)
                else:
                    membership[label] = self.triangular_mf(input_val, params)
            fuzzified.append(membership)
        return fuzzified

    def infer(self, fuzzified_inputs):
        output_membership = {}
        for (f_label, t_label, d_label), out_label in self.rule_dict.items():
            activation = min(
                fuzzified_inputs[0][f_label],
                fuzzified_inputs[1][t_label],
                fuzzified_inputs[2][d_label],
            )
            output_membership[out_label] = max(output_membership.get(out_label, 0.0), activation)
        return output_membership

    def defuzzify(self, output_membership, kind=None):
        kind = self.kind if kind is None else str(kind)
        z_range = np.linspace(0.0, 1.0, 100) if kind == "lambda_based" else np.linspace(-0.5, 0.5, 100)
        numerator = 0.0
        denominator = 0.0
        for z in z_range:
            mu_total = 0.0
            for out_label, mu in output_membership.items():
                params = self.output_sets[out_label]
                if out_label == "PL":
                    mu_z = self.trapezoidal_mf_type2(z, params)
                elif (out_label == "Z" and kind == "lambda_based") or (out_label == "NL" and kind == "delta_lambda_based"):
                    mu_z = self.trapezoidal_mf_type1(z, params)
                else:
                    mu_z = self.triangular_mf(z, params)
                mu_total = max(mu_total, min(mu, mu_z))
            numerator += mu_total * z
            denominator += mu_total
        result = 1e-6 if denominator < 1e-6 else numerator / denominator
        return float(np.clip(result, 0.0, 1.0) if kind == "lambda_based" else np.clip(result, -0.5, 0.5))

    def compute(self, inputs, kind=None):
        kind = self.kind if kind is None else str(kind)
        return self.defuzzify(self.infer(self.fuzzify(inputs, kind)), kind)


class SharedFuzzyArbitrator:
    """用于计算 ``alpha_HR`` 的 ROS1/0213 模糊/KF 仲裁器。

    输入沿用原始脚本含义：
      F_h: 归一化人类输入强度。
      T_h: 累计有效干预时间。
      D_r: 到障碍物或安全边界的距离/安全裕度。

    输出含义：
      ``alpha_HR`` 越接近 0，越偏向机器人自主参考；越接近 1，越允许人类输入。
      当安全距离变小或接触风险升高时，人类权限会被抑制。
    """

    def __init__(self, dt=0.01):
        """初始化对象参数和运行状态。"""
        self.dt = float(dt)
        self.lambda_fuzzy = FuzzyLogicTool(kind="lambda_based")
        self.delta_lambda_fuzzy = FuzzyLogicTool(kind="delta_lambda_based")
        self.kf = KalmanFilterFusion(dt=dt, epsilon=0.01, alpha=0.9)
        self.delta = DeltaLambdaUpdater(lambda0=0.5, dt=dt)

    def update(self, F_h, T_h, D_r, dF_h=0.0, dT_h=0.0, dD_r=0.0):
        """返回滤波后的 ``alpha_HR``、原始模糊值和 ``delta_lambda`` 状态。"""
        raw = self.lambda_fuzzy.compute([F_h, T_h, D_r], "lambda_based")
        delta_rule = self.delta_lambda_fuzzy.compute([dF_h, dT_h, dD_r], "delta_lambda_based")
        delta_lambda = self.delta.update(delta_rule)
        tau = np.array([[0.0], [delta_lambda]], dtype=float)
        value = self.kf.update(raw, tau)
        return value, raw, delta_lambda


def sigmoid_alpha(F_h, center=0.55, slope=6.0):
    """单特征基线：只根据人类输入强度映射到 ``alpha_HR``。"""
    return float(1.0 / (1.0 + np.exp(-slope * (float(F_h) - center))))


def compute_gt_gain(alpha):
    """与 0213 ``lqr.py`` 模型一致的连续 LQR 增益。

    ``alpha`` 在人类侧和机器人侧代价矩阵之间插值。得到的增益把任务空间状态误差
    ``[位置误差; 速度误差]`` 映射为期望笛卡尔反馈力，再由雅可比转置映射到关节力矩。
    """
    alpha = float(np.clip(alpha, 0.0, 1.0))
    m = np.diag([10.0, 10.0, 10.0])
    d = np.diag([300.0, 300.0, 300.0])
    k = np.diag([100.0, 100.0, 100.0])
    r_h = np.diag([0.001, 0.001, 0.001])
    r_r = np.diag([0.002, 0.002, 0.002])
    q_hh = np.diag([1000.0, 1000.0, 1000.0, 0.01, 0.01, 0.01])
    q_rr = np.diag([1000.0, 1000.0, 1000.0, 0.01, 0.01, 0.01])
    a = np.vstack((
        np.hstack((np.zeros((3, 3)), np.eye(3))),
        np.hstack((-np.linalg.inv(m) @ k, -np.linalg.inv(m) @ d)),
    ))
    b = np.vstack((np.zeros((3, 3)), np.linalg.inv(m)))
    q = alpha * q_hh + (1.0 - alpha) * q_rr
    r = alpha * r_h + (1.0 - alpha) * r_r
    p = solve_continuous_are(a, b, q, r)
    return np.linalg.inv(r) @ b.T @ p
