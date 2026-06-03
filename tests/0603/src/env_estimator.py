"""
环境参数 RLS 在线估计器 (Kelvin-Voigt 模型)
===========================================

模型: F = K_e·δ + B_e·δ̇ = φᵀθ
      φ = [δ, δ̇]ᵀ,  θ = [K_e, B_e]ᵀ

带遗忘因子 λ_RLS 的递归最小二乘:
  k = Pφ / (λ_RLS + φᵀ P φ)
  θ ← θ + k·(F − φᵀθ)
  P ← (P − k·φᵀP) / λ_RLS

输出 EMA 低通滤波: K̂_filt ← K̂_filt + α_lp·(θ[0] − K̂_filt)
"""
import numpy as np


class EnvironmentEstimator:
    """RLS Kelvin-Voigt 估计器。

    在扫描阶段，控制器并不知道真实环境刚度，因此用测得的接触力 F、
    压入量 delta 和压入速度 delta_dot 在线估计 [K_e, B_e]。
    估计出的 K_e 会作为 `K_hat` 送入 `CooperativeGameController.get_gain`，
    用于在刚度网格上插值控制增益。
    """

    def __init__(self, forgetting_factor=0.995, P0=1e4,
                 theta_init=None, alpha_lp=0.05):
        self.lam = float(forgetting_factor)
        self.P0 = float(P0)
        self.alpha_lp = float(alpha_lp)

        # theta = [K_e, B_e]，默认从中等偏软环境开始估计。
        if theta_init is None:
            theta_init = [200.0, 5.0]
        self._theta_init = np.asarray(theta_init, dtype=float)

        self.theta = self._theta_init.copy()
        self.P = self.P0 * np.eye(2)
        self._K_filt = float(self._theta_init[0])
        self._prev_K = self._K_filt

    def update(self, F_meas, delta, delta_dot):
        """
        更新估计

        非接触保护: F_meas < 0.3N 或 δ < 1e-7 时跳过更新
        参数下界: K_e ≥ 10, B_e ≥ 0.1

        Returns
        -------
        K_e_filt : scalar   低通滤波后的刚度估计
        B_e      : scalar   粘滞阻尼估计
        """
        self._prev_K = self._K_filt

        # 非接触保护: 未接触时 phi 信息不足，强行更新会污染刚度估计。
        if abs(F_meas) < 0.3 or abs(delta) < 1e-7:
            return self._K_filt, max(self.theta[1], 0.1)

        # RLS 标准递推。phi 是 Kelvin-Voigt 模型的回归向量。
        phi = np.array([delta, delta_dot])
        Pp = self.P @ phi
        gain = Pp / (self.lam + phi @ Pp)
        err = F_meas - phi @ self.theta
        self.theta = self.theta + gain * err
        self.P = (self.P - np.outer(gain, phi @ self.P)) / self.lam

        # 参数下界
        self.theta[0] = max(self.theta[0], 10.0)
        self.theta[1] = max(self.theta[1], 0.1)

        # EMA 低通: 控制器使用滤波后的刚度，避免查表增益抖动。
        self._K_filt += self.alpha_lp * (self.theta[0] - self._K_filt)

        return self._K_filt, self.theta[1]

    @property
    def K_e(self):
        return self._K_filt

    @property
    def B_e(self):
        return max(self.theta[1], 0.1)

    @property
    def dK_e(self):
        """刚度变化率 (per step)"""
        return self._K_filt - self._prev_K

    def reset(self):
        self.theta = self._theta_init.copy()
        self.P = self.P0 * np.eye(2)
        self._K_filt = float(self._theta_init[0])
        self._prev_K = self._K_filt
