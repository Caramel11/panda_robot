"""
力-位仲裁 α 调度器 — 阶段感知 + 模糊逻辑 + KF 融合
=====================================================

三层框架:
  α = w_φ · α_φ + (1 − w_φ) · α_fuzzy

  - 阶段先验 (α_φ, w_φ): 处理模糊逻辑无法覆盖的边界情况
  - 模糊推理 + KF: 在线自适应调节 α_fuzzy
  - 输入映射: 力-位物理量 → 模糊逻辑原始论域

输入映射:
  |e_f|  (力误差)      → F_h (0-2,    比例 0.4)
  K̂_e   (环境刚度)    → T_h (0-6,    比例 0.6/K_ref)
  |e_r|  (位置误差)    → D_r (0-0.1,  反向映射)  [关键]
"""
import numpy as np
from enum import Enum

from fuzzy_logic import FuzzyLogicTool
from kalman_filter import KalmanFilterFusion, DeltaLambdaUpdater


# ================================================================
# 任务阶段
# ================================================================
class TaskPhase(Enum):
    FREE_SPACE = 0
    APPROACHING = 1
    CONTACT_TRANSIENT = 2
    CONTACT_STEADY = 3
    RETREAT = 4


# 阶段参数: (α_φ, w_φ)
PHASE_PARAMS = {
    TaskPhase.FREE_SPACE:        (1.0,  1.0),
    TaskPhase.APPROACHING:       (0.85, 0.6),
    TaskPhase.CONTACT_TRANSIENT: (0.25, 0.5),
    TaskPhase.CONTACT_STEADY:    (0.50, 0.0),
    TaskPhase.RETREAT:           (1.0,  1.0),
}


FORCE_MARGIN_PHASE_PARAMS = {
    TaskPhase.FREE_SPACE:        (1.0,  1.0),
    TaskPhase.APPROACHING:       (0.85, 0.6),
    TaskPhase.CONTACT_TRANSIENT: (0.35, 0.5),
    TaskPhase.CONTACT_STEADY:    (0.50, 0.0),
    TaskPhase.RETREAT:           (1.0,  1.0),
}


class PhaseDetector:
    """
    任务阶段自动检测

    规则:
      |F| < F_thresh AND v_z ≥ 0              → FREE_SPACE
      |F| < F_thresh AND v_z < 0              → APPROACHING
      |F| ≥ F_thresh AND t_contact < T_trans  → CONTACT_TRANSIENT
      |F| ≥ F_thresh AND t_contact ≥ T_trans  → CONTACT_STEADY
      外部调用 set_retreat(True)               → RETREAT
    """

    def __init__(self, F_thresh=0.3, T_transient=1.0, dt=0.01):
        self.F_thresh = F_thresh
        self.T_transient = T_transient
        self.dt = dt
        self._phase = TaskPhase.FREE_SPACE
        self._t_contact = 0.0
        self._is_retreat = False

    def update(self, F_norm, z_vel):
        if self._is_retreat:
            self._phase = TaskPhase.RETREAT
            return self._phase

        if F_norm >= self.F_thresh:
            self._t_contact += self.dt
            if self._t_contact < self.T_transient:
                self._phase = TaskPhase.CONTACT_TRANSIENT
            else:
                self._phase = TaskPhase.CONTACT_STEADY
        else:
            self._t_contact = 0.0
            if z_vel < -0.001:
                self._phase = TaskPhase.APPROACHING
            else:
                self._phase = TaskPhase.FREE_SPACE
        return self._phase

    def set_retreat(self, val=True):
        self._is_retreat = val

    def reset(self):
        self._phase = TaskPhase.FREE_SPACE
        self._t_contact = 0.0
        self._is_retreat = False

    @property
    def phase(self):
        return self._phase


# ================================================================
# α 调度器
# ================================================================
class PhaseAwareFuzzyAlphaScheduler:
    """
    完整的 α 计算管线 (阶段感知 + 模糊 + KF)
    """

    def __init__(self, dt=0.01, K_ref=1000.0, phase_params=None):
        self.dt = dt
        self.K_ref = K_ref
        self.name = "phase_fuzzy_KF"

        self.phase_detector = PhaseDetector(dt=dt)
        self.phase_params = phase_params or PHASE_PARAMS

        self.lambda_fuzzy = FuzzyLogicTool(type="lambda_based")
        self.delta_lambda_fuzzy = FuzzyLogicTool(type="delta_lambda_based")

        self.kf_fusion = KalmanFilterFusion(dt=dt, epsilon=0.01)
        self.delta_updater = DeltaLambdaUpdater(lambda0=0.5, dt=dt)

        self.lambda_list = [0.5]
        self.alpha_history = []
        self.phase_history = []

        # 输入映射比例
        self.e_f_scale = 2.0 / 5.0          # |e_f| [0,5N] → F_h [0,2]
        self.K_scale = 6.0 / 10.0           # K̂/K_ref [0,10] → T_h [0,6]
        self.e_r_scale = 0.1 / 0.005        # e_r [0,5mm] → D_r [0,0.1]

    def _map_inputs(self, e_f, K_hat, e_r):
        F_h = np.clip(abs(e_f) * self.e_f_scale, 0.0, 2.0)
        T_h = np.clip(K_hat / self.K_ref * self.K_scale, 0.0, 6.0)
        # 反向映射: e_r 大 → D_r 小 → λ 小 → α 大 (位控接管)
        D_r = np.clip(0.1 - abs(e_r) * self.e_r_scale, 0.0, 0.1)
        return F_h, T_h, D_r

    def _map_delta_inputs(self, de_f, dK, de_r):
        dF = np.clip(de_f * 0.5, -3.0, 3.0)
        dT = np.clip(dK / self.K_ref + 1.0, 0.0, 2.0)
        dD = np.clip(-de_r * 20.0, -0.08, 0.08)
        return dF, dT, dD

    def compute(self, F_norm, e_f, K_hat, e_r, z_vel,
                de_f=0.0, dK=0.0, de_r=0.0, **unused):
        """
        计算当前 α

        Parameters
        ----------
        F_norm : float   接触力范数 (N), 用于阶段检测
        e_f    : float   力误差 (N)
        K_hat  : float   估计环境刚度 (N/m)
        e_r    : float   位置误差范数 (m)
        z_vel  : float   z 方向速度 (m/s)
        de_f, dK, de_r : 各量导数

        Returns
        -------
        alpha : float ∈ [0, 1]
        """
        # 1. 阶段检测
        phase = self.phase_detector.update(F_norm, z_vel)
        alpha_phi, w_phi = self.phase_params[phase]

        # 2. 模糊 → KF → α_fuzzy
        F_h, T_h, D_r = self._map_inputs(e_f, K_hat, e_r)
        dF, dT, dD = self._map_delta_inputs(de_f, dK, de_r)

        lambda_w = self.lambda_fuzzy.compute([F_h, T_h, D_r], "lambda_based")
        dlam_w = self.delta_lambda_fuzzy.compute([dF, dT, dD], "delta_lambda_based")
        lambda_delta = self.delta_updater.update(dlam_w)

        dl_w = (lambda_w - self.lambda_list[-1]) / self.dt
        tau_k = np.array([[dl_w], [lambda_delta]])
        lambda_kf = self.kf_fusion.update(lambda_w, tau_k)

        self.lambda_list.append(lambda_w)

        alpha_fuzzy = 1.0 - np.clip(lambda_kf, 0.0, 1.0)

        # 3. 阶段融合
        alpha = w_phi * alpha_phi + (1 - w_phi) * alpha_fuzzy
        alpha = float(np.clip(alpha, 0.01, 0.99))

        self.alpha_history.append(alpha)
        self.phase_history.append(phase.value)
        return alpha

    def set_retreat(self, val=True):
        self.phase_detector.set_retreat(val)

    def reset(self):
        self.lambda_list = [0.5]
        self.alpha_history = []
        self.phase_history = []
        self.phase_detector.reset()
        self.kf_fusion = KalmanFilterFusion(dt=self.dt, epsilon=0.01)
        self.delta_updater = DeltaLambdaUpdater(lambda0=0.5, dt=self.dt)


class ForceMarginFuzzyAlphaScheduler:
    """
    基于交互力上下界安全裕度的 α 仲裁器。

    输入保持与旧调度器兼容，额外接收 F_desired/F_min/F_max。核心变量:
      F_h   = 2|F-Fd|/(Fmax-Fmin)
      S_h   = 6ρ_F, ρ_F 为到上下界的归一化安全裕度
      D_r   = 0.1 - |e_r| * 0.1/0.005
      s_F   = (F-Fd)/((Fmax-Fmin)/2)

    模糊表直接输出 alpha_fuzzy，再按 s_F 做上下界方向安全修正。
    """

    def __init__(self, dt=0.01, F_min=0.2, F_max=1.0,
                 F_desired=0.5, phase_params=None,
                 k_upper=0.35, k_lower=0.25,
                 alpha_min=0.05, alpha_max=0.95,
                 upper_guard_alpha=0.75, lower_guard_alpha=0.25,
                 smooth_tau=0.08):
        self.dt = dt
        self.F_min = float(F_min)
        self.F_max = float(F_max)
        self.F_desired = float(F_desired)
        self.phase_params = phase_params or FORCE_MARGIN_PHASE_PARAMS
        self.phase_detector = PhaseDetector(dt=dt)
        self.name = "force_margin_alpha"

        self.k_upper = float(k_upper)
        self.k_lower = float(k_lower)
        self.alpha_min = float(alpha_min)
        self.alpha_max = float(alpha_max)
        self.upper_guard_alpha = float(upper_guard_alpha)
        self.lower_guard_alpha = float(lower_guard_alpha)
        self.smooth_beta = float(np.clip(dt / max(smooth_tau, dt), 0.0, 1.0))

        self.alpha_history = []
        self.phase_history = []
        self.rho_history = []
        self.margin_history = []
        self._alpha_filt = 0.5

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
        self.rule_dict = {
            ("PS", "PS", "PS"): "PL", ("PS", "PS", "PM"): "P",  ("PS", "PS", "PL"): "PM",
            ("PS", "PM", "PS"): "PL", ("PS", "PM", "PM"): "P",  ("PS", "PM", "PL"): "PM",
            ("PS", "PL", "PS"): "P",  ("PS", "PL", "PM"): "PM", ("PS", "PL", "PL"): "PM",
            ("PM", "PS", "PS"): "PL", ("PM", "PS", "PM"): "P",  ("PM", "PS", "PL"): "PS",
            ("PM", "PM", "PS"): "P",  ("PM", "PM", "PM"): "PM", ("PM", "PM", "PL"): "PS",
            ("PM", "PL", "PS"): "P",  ("PM", "PL", "PM"): "PM", ("PM", "PL", "PL"): "PS",
            ("PL", "PS", "PS"): "PL", ("PL", "PS", "PM"): "PM", ("PL", "PS", "PL"): "Z",
            ("PL", "PM", "PS"): "P",  ("PL", "PM", "PM"): "PS", ("PL", "PM", "PL"): "Z",
            ("PL", "PL", "PS"): "P",  ("PL", "PL", "PM"): "PS", ("PL", "PL", "PL"): "Z",
        }

    @staticmethod
    def _triangular_mf(x, params):
        a, b, c = params
        if x <= a or x >= c:
            return 0.0
        if a < x <= b:
            return (x - a) / max(b - a, 1e-12)
        return (c - x) / max(c - b, 1e-12)

    @staticmethod
    def _left_shoulder_mf(x, params):
        a, b, c = params
        if x <= b:
            return 1.0 if x >= a else 0.0
        if x >= c:
            return 0.0
        return (c - x) / max(c - b, 1e-12)

    @staticmethod
    def _right_shoulder_mf(x, params):
        a, b, c = params
        if x <= a:
            return 0.0
        if x >= b:
            return 1.0 if x <= c else 0.0
        return (x - a) / max(b - a, 1e-12)

    def _membership(self, x, label, params):
        if label in ("PS", "Z"):
            return self._left_shoulder_mf(x, params)
        if label == "PL":
            return self._right_shoulder_mf(x, params)
        return self._triangular_mf(x, params)

    def _fuzzify(self, F_h, S_h, D_r):
        inputs = [
            np.clip(F_h, 0.0, 2.0),
            np.clip(S_h, 0.0, 6.0),
            np.clip(D_r, 0.0, 0.1),
        ]
        fuzzified = []
        for value, sets in zip(inputs, self.input_sets):
            fuzzified.append({
                label: self._membership(value, label, params)
                for label, params in sets.items()
            })
        return fuzzified

    def _infer(self, fuzzified):
        output_membership = {}
        for (f_label, s_label, d_label), out_label in self.rule_dict.items():
            activation = min(
                fuzzified[0][f_label],
                fuzzified[1][s_label],
                fuzzified[2][d_label],
            )
            output_membership[out_label] = max(
                activation, output_membership.get(out_label, 0.0)
            )
        return output_membership

    def _defuzzify_alpha(self, output_membership):
        z_range = np.linspace(0.0, 1.0, 101)
        numerator = 0.0
        denominator = 0.0
        for z in z_range:
            mu_total = 0.0
            for label, mu in output_membership.items():
                mu_z = self._membership(z, label, self.output_sets[label])
                mu_total = max(mu_total, min(mu, mu_z))
            numerator += mu_total * z
            denominator += mu_total
        if denominator < 1e-9:
            return 0.5
        return float(np.clip(numerator / denominator, 0.0, 1.0))

    def _force_margin_inputs(self, F_norm, F_desired, F_min, F_max, e_r):
        width = max(F_max - F_min, 1e-6)
        half_width = max(0.5 * width, 1e-6)
        F_err = F_norm - F_desired
        d_lower = F_norm - F_min
        d_upper = F_max - F_norm
        rho_F = float(np.clip(min(d_lower, d_upper) / half_width, 0.0, 1.0))
        s_F = float(np.clip(F_err / half_width, -1.0, 1.0))
        F_h = float(np.clip(abs(F_err) / width * 2.0, 0.0, 2.0))
        S_h = 6.0 * rho_F
        D_r = float(np.clip(0.1 - abs(e_r) * (0.1 / 0.005), 0.0, 0.1))
        return F_h, S_h, D_r, rho_F, s_F, F_err

    def compute(self, F_norm, e_f=0.0, K_hat=None, e_r=0.0, z_vel=0.0,
                de_f=0.0, dK=0.0, de_r=0.0, F_desired=None,
                F_min=None, F_max=None, **unused):
        F_desired = self.F_desired if F_desired is None else float(F_desired)
        F_min = self.F_min if F_min is None else float(F_min)
        F_max = self.F_max if F_max is None else float(F_max)
        if F_max <= F_min:
            F_max = F_min + 1e-6

        phase = self.phase_detector.update(F_norm, z_vel)
        alpha_phi, w_phi = self.phase_params[phase]

        F_h, S_h, D_r, rho_F, s_F, F_err = self._force_margin_inputs(
            F_norm, F_desired, F_min, F_max, e_r
        )
        alpha_0 = self._defuzzify_alpha(self._infer(self._fuzzify(F_h, S_h, D_r)))

        r_F = 1.0 - rho_F
        alpha_safe = (
            alpha_0
            + self.k_upper * r_F * max(s_F, 0.0)
            - self.k_lower * r_F * max(-s_F, 0.0)
        )
        if F_norm >= F_max:
            alpha_safe = max(alpha_safe, self.upper_guard_alpha)
        elif F_norm <= F_min:
            alpha_safe = min(alpha_safe, self.lower_guard_alpha)
        alpha_safe = float(np.clip(alpha_safe, self.alpha_min, self.alpha_max))

        alpha_raw = w_phi * alpha_phi + (1.0 - w_phi) * alpha_safe
        alpha_raw = float(np.clip(alpha_raw, 0.01, 0.99))
        alpha = self._alpha_filt + self.smooth_beta * (alpha_raw - self._alpha_filt)
        alpha = float(np.clip(alpha, 0.01, 0.99))
        self._alpha_filt = alpha

        self.alpha_history.append(alpha)
        self.phase_history.append(phase.value)
        self.rho_history.append(rho_F)
        self.margin_history.append((F_h, S_h, D_r, s_F, F_err))
        return alpha

    def set_retreat(self, val=True):
        self.phase_detector.set_retreat(val)

    def reset(self):
        self.alpha_history = []
        self.phase_history = []
        self.rho_history = []
        self.margin_history = []
        self._alpha_filt = 0.5
        self.phase_detector.reset()


class FixedAlphaScheduler:
    """固定 α 对照组"""

    def __init__(self, val=0.5):
        self.alpha = val
        self.name = f"fixed_{val:.1f}"
        self.alpha_history = []
        self.phase_history = []

    def compute(self, **kw):
        self.alpha_history.append(self.alpha)
        self.phase_history.append(-1)
        return self.alpha

    def set_retreat(self, val=True):
        pass

    def reset(self):
        self.alpha_history = []
        self.phase_history = []

    @property
    def phase_detector(self):
        class _D:
            phase = type('P', (), {'value': -1})()
        return _D()
