"""
合作博弈力-位控制器 — 4D 逐轴 ARE + 基线虚拟刚度 K_v
=====================================================

数学模型 (逐轴标量, 三轴相同由 Kronecker 扩展)
----------------------------------------------

1. 修正的阻抗动力学:
     m ẍ + c ẋ + K_v (x − x_r) = u + f_ext        (m=10, c=300, K_v=100)

   关键: 新增 K_v(x−x_r) 基线虚拟弹簧, 消除原方案 Ā 矩阵 A[1,0]=0 的
         结构奇异性。

2. 增广状态 (每轴 4D):
     z_axis = [e_r1, e_r2, e_f, σ_f]ᵀ ∈ ℝ⁴

   其中 e_r1 和 σ_f 是**泄漏积分器**的状态:
     ė_r1 = e_r2 − ε_r · e_r1                (ε_r = 1 s⁻¹)
     σ̇_f  = e_f  − ε_f · σ_f                 (ε_f = 2 s⁻¹)
     ė_r2 = -K_v/m · e_r1 − c/m · e_r2 + 1/m · e_f + 1/m · u
     ė_f  = -(K_e − B_e·c/m) · e_r2 − B_e/m · e_f − B_e/m · u

3. 每轴 4×4 系统矩阵:

     Ā_axis = ┌ −ε_r     1         0        0   ┐
              │ −K_v/m  −c/m       1/m      0   │
              │  0      −κ        −β        0   │
              └  0       0         1       −ε_f ┘

     b̄_axis = [0, 1/m, −β, 0]ᵀ

     其中 κ = K_e − B_e·c/m,  β = B_e/m

4. 三轴合并 (Kronecker 积):
     z = [e_r1 (3), e_r2 (3), e_f (3), σ_f (3)]ᵀ ∈ ℝ¹²
     Ā_12 = Ā_axis ⊗ I_3    ∈ ℝ¹²ˣ¹²
     B̄_12 = b̄_axis ⊗ I_3    ∈ ℝ¹²ˣ³

5. 归一化代价矩阵 (每轴 4×4):
     Q_1 = diag(q_r1, q_r2, 0, 0) = diag(40000, 400, 0, 0)
     Q_2 = diag(0, 0, q_f, q_sf) = diag(0, 0, 1.0, 0.25)
     Q_α = α·Q_1 + (1−α)·Q_2
     R_α = α·R_1 + (1−α)·R_2 = 1.0

   归一化原理: q_i · e_i² 在典型误差下均为 O(1):
     q_r1 · (5mm)²  = 40000 × 0.025×10⁻³ = 1.0
     q_r2 · (50mm/s)² = 400 × 2.5×10⁻³ = 1.0
     q_f · (1N)²    = 1.0
     q_sf · (2Ns)²  = 0.25 × 4 = 1.0

6. ARE 解与增益:
     Ā_axisᵀ P + P Ā_axis − P b̄ R_α⁻¹ b̄ᵀ P + Q_α_axis = 0
     K_raw_axis = R_α⁻¹ b̄ᵀ P    ∈ ℝ¹ˣ⁴
     K_raw_axis = [K_r1, K_r2, K_ef, K_sf]

7. 等效增益 (折入 K_v 基线):
     K_eff_axis = [K_v + K_r1, K_r2, K_ef, K_sf]
     K_total = K_v + K_r1    (总等效位置刚度)

   应用控制律 (每轴):
     u_axis = −K_eff_axis · [e_r1, e_r2, e_f, σ_f]ᵀ

   三轴合并:
     u = −(K_eff_axis ⊗ I_3) · z     ∈ ℝ³

8. 离线预计算:
     α_grid  = 0.0, 0.05, ..., 1.0          (21 点)
     K_e_grid = [50, 80, 100, ..., 5000]     (13 点)
     对每个 (α, K_e) 求 ARE → 增益表

9. 在线查表 + 双线性插值: gain_table[α, K_e] → K_eff_axis
"""
import numpy as np
from scipy.linalg import solve_continuous_are


# ================================================================
# 系统矩阵构造 (逐轴 4D)
# ================================================================
def build_axis_system(M, C, Kv, Ke, Be, eps_r, eps_f):
    """
    构造每轴 4×4 A 矩阵和 4×1 b 向量

    Parameters
    ----------
    M, C  : scalar   期望惯性, 阻尼
    Kv    : scalar   基线虚拟刚度 (N/m)
    Ke, Be: scalar   环境刚度, 阻尼
    eps_r : scalar   位置泄漏因子 (s⁻¹)
    eps_f : scalar   力积分泄漏因子 (s⁻¹)

    Returns
    -------
    A : (4, 4)
    b : (4, 1)
    """
    kappa = Ke - Be * C / M        # K_e − B_e·c/m
    beta  = Be / M                 # B_e / m

    A = np.array([
        [-eps_r,   1.0,      0.0,     0.0   ],
        [-Kv/M,   -C/M,      1/M,     0.0   ],
        [ 0.0,    -kappa,   -beta,    0.0   ],
        [ 0.0,     0.0,      1.0,    -eps_f ],
    ])

    b = np.array([
        [0.0],
        [1/M],
        [-beta],
        [0.0],
    ])

    return A, b


# ================================================================
# 单点 ARE 求解 (Pareto 加权合并)
# ================================================================
def solve_axis_ARE(A, b, alpha, q_r1, q_r2, q_f, q_sf, R1, R2):
    """
    求解给定 (α) 下的每轴 4×4 合作博弈 ARE

    Q_α = α·diag(q_r1, q_r2, 0, 0) + (1−α)·diag(0, 0, q_f, q_sf)
    R_α = α·R_1 + (1−α)·R_2

    Āᵀ P + P Ā − P b R_α⁻¹ bᵀ P + Q_α = 0
    K_raw = R_α⁻¹ bᵀ P                      ∈ ℝ¹ˣ⁴

    Returns
    -------
    K_raw : (4,)   原始 ARE 增益 [K_r1, K_r2, K_ef, K_sf]
    P     : (4, 4) Riccati 解
    max_re: scalar 闭环最大特征值实部 (必须 < 0)
    """
    Q_alpha = np.diag([
        alpha * q_r1,
        alpha * q_r2,
        (1 - alpha) * q_f,
        (1 - alpha) * q_sf,
    ]) + 1e-8 * np.eye(4)   # 正则化防止端点 (α=0 或 α=1) 的可检测性问题

    R_alpha = alpha * R1 + (1 - alpha) * R2
    R_mat = np.array([[R_alpha]])

    P = solve_continuous_are(A, b, Q_alpha, R_mat)
    K_raw = (1.0 / R_alpha) * b.T @ P        # (1, 4)

    A_cl = A - b @ K_raw
    max_re = max(e.real for e in np.linalg.eigvals(A_cl))

    return K_raw.flatten(), P, max_re


# ================================================================
# 控制器类
# ================================================================
class CooperativeGameController:
    """
    合作博弈力-位控制器 (4D 每轴 ARE + K_v 基线)

    使用方法:
      ctrl = CooperativeGameController()
      ctrl.precompute_gains()
      ctrl.save_gains('gains.npy')    # 保存以免重复计算
      # 在线:
      u_tool = ctrl.compute_control(e_r1, e_r2, e_f, sigma_f, alpha, K_e_hat)
    """

    def __init__(self):
        # ---- 阻抗模型参数 ----
        self.M = 10.0         # kg    期望惯性
        self.C = 300.0        # Ns/m  阻尼
        self.Kv = 100.0       # N/m   基线虚拟刚度 [新增]

        # ---- 泄漏积分器参数 ----
        self.eps_r = 1.0      # s⁻¹   位置泄漏因子
        self.eps_f = 2.0      # s⁻¹   力积分泄漏因子

        # ---- 归一化代价矩阵参数 [新增] ----
        self.q_r1 = 40000.0   # 1/(5mm)²
        self.q_r2 = 400.0     # 1/(50mm/s)²
        self.q_f  = 100.0       # 1/(1N)²
        self.q_sf = 0.25      # 1/(2N·s)²
        self.R1 = 1.0
        self.R2 = 1.0

        # ---- 姿态 PD+I 增益 ----
        self.P_ori = 20.0
        self.D_ori = 1.0
        self.I_ori = 30.0

        # ---- 增益表 ----
        # 键: (alpha_round, Ke_round)
        # 值: np.array shape (4,) = [K_v + K_r1, K_r2, K_ef, K_sf]  (即 K_eff)
        self.gains_db = {}
        self.alpha_grid = None
        self.Ke_grid = None

        # ---- 安全限幅 ----
        self.tau_max = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)
        self.u_threshold = 30.0

    # ----------------------------------------------------------------
    # 预计算
    # ----------------------------------------------------------------
    def precompute_gains(self, alpha_grid=None, Ke_grid=None, Be_default=5.0):
        """
        在 (α, K_e) 网格上预计算增益表

        每个网格点求一次 4D ARE (标量系统), 典型耗时 < 1ms/点。
        273 个点总计约 0.5 秒完成。
        """
        import rospy

        if alpha_grid is None:
            alpha_grid = np.linspace(0.0, 1.0, 21)
        if Ke_grid is None:
            Ke_grid = [50, 80, 100, 150, 200, 300, 500, 800, 1000, 1500,
                       2000, 3000, 5000]

        self.alpha_grid = np.array(alpha_grid)
        self.Ke_grid = np.array(Ke_grid)

        n_alpha = len(alpha_grid)
        n_Ke = len(Ke_grid)
        rospy.loginfo(f"Precomputing cooperative ARE gains: "
                      f"{n_alpha} α × {n_Ke} K_e = {n_alpha * n_Ke} points")

        count = 0
        fail = 0
        for alpha in alpha_grid:
            a_key = round(float(alpha), 3)
            for Ke in Ke_grid:
                Ke_key = int(Ke)
                A, b = build_axis_system(
                    self.M, self.C, self.Kv, Ke, Be_default,
                    self.eps_r, self.eps_f
                )
                try:
                    K_raw, P, max_re = solve_axis_ARE(
                        A, b, a_key,
                        self.q_r1, self.q_r2, self.q_f, self.q_sf,
                        self.R1, self.R2
                    )
                    if max_re < -1e-8:
                        # 折入 K_v: K_eff[0] = K_v + K_r1
                        K_eff = K_raw.copy()
                        K_eff[0] = self.Kv + K_raw[0]
                        self.gains_db[(a_key, Ke_key)] = K_eff
                        count += 1
                    else:
                        fail += 1
                        self._fallback(a_key, Ke_key)
                except Exception:
                    fail += 1
                    self._fallback(a_key, Ke_key)

        rospy.loginfo(f"  Done. {count} stable, {fail} failures.")

    def _fallback(self, a_key, Ke_key):
        """ARE 失败时用最近的成功增益填充"""
        if not self.gains_db:
            return
        nearest = min(
            self.gains_db.keys(),
            key=lambda k: abs(k[0] - a_key) + abs(k[1] - Ke_key) / 1000.0
        )
        self.gains_db[(a_key, Ke_key)] = self.gains_db[nearest]

    def save_gains(self, path):
        np.save(path, {
            'gains_db': self.gains_db,
            'alpha_grid': self.alpha_grid,
            'Ke_grid': self.Ke_grid,
        })

    def load_gains(self, path):
        data = np.load(path, allow_pickle=True).item()
        self.gains_db = data['gains_db']
        self.alpha_grid = data['alpha_grid']
        self.Ke_grid = data['Ke_grid']

    # ----------------------------------------------------------------
    # 在线查表 (双线性插值)
    # ----------------------------------------------------------------
    def _bracket(self, grid, value):
        """在有序 grid 中找到 value 的左右相邻点"""
        if value <= grid[0]:
            return grid[0], grid[0], 0.0
        if value >= grid[-1]:
            return grid[-1], grid[-1], 0.0
        for i in range(len(grid) - 1):
            if grid[i] <= value <= grid[i + 1]:
                t = (value - grid[i]) / (grid[i + 1] - grid[i])
                return grid[i], grid[i + 1], t
        return grid[0], grid[0], 0.0

    def get_gain(self, alpha, Ke):
        """
        双线性插值查增益表

        Returns
        -------
        K_eff_axis : (4,)  [K_v+K_r1, K_r2, K_ef, K_sf]
        """
        if not self.gains_db:
            # 未预计算时的兜底: 返回纯 K_v + 少量阻尼
            return np.array([self.Kv, 20.0, 0.0, 0.0])

        # 找 α 和 K_e 的相邻网格点
        a_lo, a_hi, t_a = self._bracket(self.alpha_grid, alpha)
        Ke_lo, Ke_hi, t_Ke = self._bracket(self.Ke_grid, Ke)

        a_lo = round(float(a_lo), 3)
        a_hi = round(float(a_hi), 3)
        Ke_lo = int(Ke_lo)
        Ke_hi = int(Ke_hi)

        # 四角增益
        def _get(ak, Kk):
            if (ak, Kk) in self.gains_db:
                return self.gains_db[(ak, Kk)]
            # fallback to nearest
            nearest = min(
                self.gains_db.keys(),
                key=lambda k: abs(k[0] - ak) + abs(k[1] - Kk) / 1000.0
            )
            return self.gains_db[nearest]

        K_00 = _get(a_lo, Ke_lo)
        K_01 = _get(a_lo, Ke_hi)
        K_10 = _get(a_hi, Ke_lo)
        K_11 = _get(a_hi, Ke_hi)

        # 双线性
        K = ((1 - t_a) * (1 - t_Ke) * K_00
             + (1 - t_a) * t_Ke * K_01
             + t_a * (1 - t_Ke) * K_10
             + t_a * t_Ke * K_11)

        return K

    # ----------------------------------------------------------------
    # 控制律
    # ----------------------------------------------------------------
    def compute_control(self, e_r1, e_r2, e_f, sigma_f, alpha, Ke):
        """
        计算工具尖端笛卡尔控制力

        每个误差量为 3D 向量 (x, y, z)。因为三轴动力学解耦且各向同性,
        增益各向同性: K_eff_12 = K_eff_axis ⊗ I_3 ∈ ℝ³ˣ¹²

        统一矩阵乘法:
          u = -K_eff_12 · z
            = -K_eff[0]·e_r1 - K_eff[1]·e_r2 - K_eff[2]·e_f - K_eff[3]·σ_f

        注: K_eff[0] = K_v + K_r1 (已包含基线刚度 K_v, 阻抗模型的
            K_v(x−x_r) 项由此项在高层显式实现)。

        Parameters
        ----------
        e_r1   : (3,)   泄漏积分的位置误差状态
        e_r2   : (3,)   速度误差 ẋ − ẋ_r
        e_f    : (3,)   力误差 F_ext − F_d
        sigma_f: (3,)   泄漏积分的力误差状态
        alpha  : float  仲裁参数
        Ke     : float  RLS 估计的环境刚度

        Returns
        -------
        u_tool : (3,)   笛卡尔控制力
        K_eff  : (4,)   当前激活的增益 (用于日志)
        """
        K = self.get_gain(alpha, Ke)

        u = -(K[0] * np.asarray(e_r1)
              + K[1] * np.asarray(e_r2)
              + K[2] * np.asarray(e_f)
              + K[3] * np.asarray(sigma_f))

        return u, K
